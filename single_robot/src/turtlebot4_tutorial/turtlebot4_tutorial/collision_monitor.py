"""Fail-closed collision filter with full footprint and rotation checks."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .motion_safety import (
    check_motion,
    finite_twist,
    scan_is_valid,
)


def identity_transform(frame_id):
    """Return a planar identity transform for legacy scans without a frame."""
    transform = TransformStamped()
    transform.header.frame_id = frame_id
    transform.child_frame_id = frame_id
    transform.transform.rotation.w = 1.0
    return transform


class CollisionMonitor(Node):
    """Block or limit motion using a transformed 360-degree laser scan."""

    def __init__(self):
        super().__init__('collision_monitor')
        self.declare_parameter('input_topic', 'cmd_vel_raw')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.70)
        self.declare_parameter('front_angle', 0.70)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('tf_timeout', 0.8)
        self.declare_parameter('robot_radius', 0.175)
        self.declare_parameter('reaction_time', 0.60)
        self.declare_parameter('lateral_margin', 0.02)
        self.declare_parameter('rotation_margin', 0.10)

        self.nearest_front = math.inf
        self.has_front_measurement = False
        self.scan_valid = False
        self.scan_message = None
        self.last_scan = None
        self.last_check = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value,
            self.on_command, 10)
        self.publisher = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

    def on_scan(self, scan):
        """Cache the scan and its front-sector distance for diagnostics."""
        front_angle = float(self.get_parameter('front_angle').value)
        half_angle = front_angle / 2.0 if (
            math.isfinite(front_angle) and front_angle > 0.0) else -1.0
        front = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if (abs(angle) <= half_angle and math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max):
                front.append(distance)
        self.nearest_front = min(front) if front else math.inf
        self.has_front_measurement = bool(front)
        self.scan_valid = scan_is_valid(scan)
        self.scan_message = scan
        self.last_scan = time.monotonic()

    def scan_is_fresh(self):
        """Return True only for a fresh and structurally valid scan."""
        timeout = float(self.get_parameter('scan_timeout').value)
        if not math.isfinite(timeout):
            return False
        timeout = max(timeout, 0.0)
        return (
            self.last_scan is not None
            and time.monotonic() - self.last_scan <= timeout
            and self.scan_valid
        )

    def _scan_transform(self, scan):
        base_frame = self.get_parameter('base_frame').value
        frame = scan.header.frame_id
        if not frame or frame == base_frame:
            return identity_transform(base_frame)
        try:
            return self.buffer.lookup_transform(
                base_frame, frame, Time.from_msg(scan.header.stamp))
        except TransformException:
            try:
                transform = self.buffer.lookup_transform(
                    base_frame, frame, Time())
            except TransformException:
                return None
            age = (
                self.get_clock().now()
                - Time.from_msg(transform.header.stamp)
            ).nanoseconds * 1e-9
            timeout = float(self.get_parameter('tf_timeout').value)
            if not -0.5 <= age <= timeout:
                return None
            return transform

    def _safety_parameters(self):
        return {
            'robot_radius': float(self.get_parameter('robot_radius').value),
            'stop_distance': float(self.get_parameter('stop_distance').value),
            'slow_distance': float(self.get_parameter('slow_distance').value),
            'reaction_time': float(self.get_parameter('reaction_time').value),
            'lateral_margin': float(
                self.get_parameter('lateral_margin').value),
            'rotation_margin': float(
                self.get_parameter('rotation_margin').value),
        }

    def on_command(self, command):
        """Filter one command through the full swept-footprint check."""
        output = Twist()
        output.linear.x = command.linear.x
        output.linear.y = command.linear.y
        output.linear.z = command.linear.z
        output.angular.x = command.angular.x
        output.angular.y = command.angular.y
        output.angular.z = command.angular.z
        if not finite_twist(command):
            self.get_logger().error(
                '速度指令包含非有限数值，已改为停车。',
                throttle_duration_sec=1.0)
            self.publisher.publish(Twist())
            return
        if not self.scan_is_fresh():
            self.get_logger().warning(
                '尚未收到有效雷达数据或雷达数据超时，'
                '已停车。',
                throttle_duration_sec=1.0)
            self.publisher.publish(Twist())
            return
        transform = self._scan_transform(self.scan_message)
        if transform is None:
            self.get_logger().warning(
                '雷达坐标变换不可用，已停车。',
                throttle_duration_sec=1.0)
            self.publisher.publish(Twist())
            return
        check = check_motion(
            self.scan_message,
            transform,
            command.linear.x,
            command.angular.z,
            **self._safety_parameters(),
        )
        self.last_check = check
        if check.linear_scale < 1.0:
            output.linear.x *= check.linear_scale
            self.get_logger().warning(
                check.reason or '前进方向障碍物较近，已限速。',
                throttle_duration_sec=1.0)
        if not check.angular_allowed:
            output.angular.z = 0.0
            message = check.reason or (
                '旋转方向障碍物过近，'
                '已停止转向。')
            self.get_logger().warning(
                message, throttle_duration_sec=1.0)
        self.publisher.publish(output)


def main(args=None):
    """Run the collision monitor node."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
