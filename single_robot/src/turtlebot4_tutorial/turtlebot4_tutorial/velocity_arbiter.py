"""Select one velocity source and stop when safety inputs are unavailable."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .motion_safety import finite_twist, scan_is_valid, stamp_is_fresh


class VelocityArbiter(Node):
    """Arbitrate navigation, teleoperation and auxiliary velocity commands."""

    def __init__(self):
        super().__init__('velocity_arbiter')
        self.declare_parameter('nav_topic', 'cmd_vel')
        self.declare_parameter('teleop_topic', 'cmd_vel_raw')
        self.declare_parameter('aux_topic', 'cmd_vel_aux')
        self.declare_parameter('output_topic', 'cmd_vel_safe')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('scan_timeout', 0.8)
        self.declare_parameter('odom_timeout', 0.8)
        self.declare_parameter('tf_timeout', 0.8)
        self.declare_parameter('require_odom_tf', True)
        self.declare_parameter('require_nav_localization', True)
        self.declare_parameter('base_frame', 'base_link')

        self.estop = False
        self.commands = {}
        self.scan = None
        self.scan_received = 0.0
        self.scan_valid = False
        self.odom = None
        self.odom_received = 0.0
        self.odom_valid = False
        self.last_status = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.status_publisher = self.create_publisher(String, '~/status', 10)

        nav_topic = self.get_parameter('nav_topic').value
        teleop_topic = self.get_parameter('teleop_topic').value
        aux_topic = self.get_parameter('aux_topic').value
        self.create_subscription(
            Twist, nav_topic,
            lambda message: self.on_command('nav', message), 10)
        self.create_subscription(
            Twist, teleop_topic,
            lambda message: self.on_command('teleop', message), 10)
        self.create_subscription(
            Twist, aux_topic,
            lambda message: self.on_command('aux', message), 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, 10)
        self.create_service(SetBool, '~/set_estop', self.on_set_estop)
        self.create_timer(0.05, self.tick)

    def on_command(self, source, message):
        """Store a finite command or reject the whole message."""
        if not finite_twist(message):
            self.commands.pop(source, None)
            self.get_logger().error(
                f'{source} 速度指令包含非有限数值，已忽略。',
                throttle_duration_sec=2.0)
            return
        self.commands[source] = (time.monotonic(), message)

    def on_scan(self, message):
        """Track the latest validated laser scan."""
        self.scan = message
        self.scan_received = time.monotonic()
        self.scan_valid = scan_is_valid(message)

    def on_odom(self, message):
        """Track the latest finite odometry message."""
        position = message.pose.pose.position
        velocity = message.twist.twist
        self.odom = message
        self.odom_received = time.monotonic()
        self.odom_valid = all(math.isfinite(value) for value in (
            position.x, position.y, velocity.linear.x, velocity.angular.z))

    def _transform_age(self, parent, child):
        try:
            transform = self.buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        age = (
            self.get_clock().now()
            - Time.from_msg(transform.header.stamp)
        ).nanoseconds * 1e-9
        timeout = float(self.get_parameter('tf_timeout').value)
        if -0.5 <= age <= timeout:
            return age
        return None

    def _sensor_reason(self, source):
        now = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        scan_timeout = float(self.get_parameter('scan_timeout').value)
        odom_timeout = float(self.get_parameter('odom_timeout').value)
        if self.scan is None or now - self.scan_received > scan_timeout:
            return '雷达超时'
        if not self.scan_valid:
            return '雷达数据无效'
        if not stamp_is_fresh(self.scan.header.stamp, now_ns, scan_timeout):
            return '雷达时间戳超时或无效'
        if self.odom is None or now - self.odom_received > odom_timeout:
            return '里程计超时'
        if not self.odom_valid:
            return '里程计数据无效'
        if not stamp_is_fresh(self.odom.header.stamp, now_ns, odom_timeout):
            return '里程计时间戳超时或无效'
        base_frame = self.get_parameter('base_frame').value
        if (self.get_parameter('require_odom_tf').value
                and self._transform_age('odom', base_frame) is None):
            return '里程计坐标变换不可用'
        if (source == 'nav'
                and self.get_parameter('require_nav_localization').value
                and self._transform_age('map', base_frame) is None):
            return '定位坐标变换不可用'
        return ''

    def _select(self):
        timeout = float(self.get_parameter('command_timeout').value)
        now = time.monotonic()
        for source in ('teleop', 'aux', 'nav'):
            entry = self.commands.get(source)
            if entry is not None and now - entry[0] <= timeout:
                return source, entry[1]
        return None, None

    def publish_zero(self):
        """Publish one explicit stop command."""
        self.publisher.publish(Twist())

    def _publish_status(self, source, reason):
        now = time.monotonic()
        if now - self.last_status < 0.5:
            return
        self.last_status = now
        self.status_publisher.publish(String(data=json.dumps({
            'source': source or '',
            'reason': reason,
            'estop': self.estop,
            'scan_age': (
                now - self.scan_received if self.scan_received else None
            ),
            'odom_age': (
                now - self.odom_received if self.odom_received else None
            ),
        }, ensure_ascii=False)))

    def tick(self):
        """Publish the selected command or a fail-closed stop."""
        if self.estop:
            self.publish_zero()
            self._publish_status('', '急停锁定')
            return
        source, command = self._select()
        if command is None:
            self.publish_zero()
            self._publish_status('', '等待速度指令')
            return
        reason = self._sensor_reason(source)
        if reason:
            self.publish_zero()
            self._publish_status(source, reason)
            return
        self.publisher.publish(command)
        self._publish_status(source, '')

    def on_set_estop(self, request, response):
        """Latch or release the emergency stop."""
        self.estop = bool(request.data)
        self.commands.clear()
        self.publish_zero()
        response.success = True
        if self.estop:
            response.message = '急停已锁定'
        else:
            response.message = '急停已解除，等待新指令'
        self.get_logger().warning(response.message)
        return response


def main(args=None):
    """Run the velocity arbiter node."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
