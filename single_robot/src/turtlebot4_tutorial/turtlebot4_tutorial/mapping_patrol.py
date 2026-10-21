#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class MappingPatrol(Node):
    """Run a conservative scan/drive sequence for tutorial map generation."""

    def __init__(self):
        super().__init__('mapping_patrol')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('side_length', 0.90)
        self.declare_parameter('turn_angle', math.pi / 2.0)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.nearest_front = math.inf
        self.has_front_measurement = False
        self.last_scan = None
        self.odom_pose = None
        self.last_odom = None

    def on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        pose = (position.x, position.y, yaw)
        if all(math.isfinite(value) for value in pose):
            self.odom_pose = pose
            self.last_odom = time.monotonic()

    def on_scan(self, scan):
        front = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if (abs(angle) < 0.35 and math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max):
                front.append(distance)
        self.nearest_front = min(front) if front else math.inf
        self.has_front_measurement = bool(front)
        self.last_scan = time.monotonic()

    def scan_is_fresh(self):
        timeout = float(self.get_parameter('scan_timeout').value)
        if not math.isfinite(timeout):
            return False
        timeout = max(timeout, 0.0)
        return (self.has_front_measurement
                and self.last_scan is not None
                and time.monotonic() - self.last_scan <= timeout)

    def run_segment(self, linear, angular, duration):
        target_distance = abs(linear) * duration
        target_turn = abs(angular) * duration
        if target_distance == 0.0 and target_turn == 0.0:
            return
        started_at = time.monotonic()
        deadline = started_at + max(30.0, duration * 20.0)
        start_pose = None
        last_yaw = None
        turned = 0.0
        blocked_since = None
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                now = time.monotonic()
                if now > deadline:
                    raise RuntimeError('里程计未在限时内达到建图动作目标，已停车。')
                if self.last_odom is None or now - self.last_odom > 2.0:
                    self.publisher.publish(Twist())
                    if now - started_at > 5.0:
                        raise RuntimeError('里程计无数据或已超时，建图巡航已停车。')
                    time.sleep(0.1)
                    continue
                x, y, yaw = self.odom_pose
                if start_pose is None:
                    start_pose = self.odom_pose
                elif target_distance > 0.0:
                    dx = x - start_pose[0]
                    dy = y - start_pose[1]
                    direction = math.copysign(1.0, linear)
                    forward = direction * (
                        dx * math.cos(start_pose[2])
                        + dy * math.sin(start_pose[2]))
                    if forward >= target_distance:
                        break
                elif target_turn > 0.0 and last_yaw is not None:
                    delta = math.atan2(
                        math.sin(yaw - last_yaw),
                        math.cos(yaw - last_yaw))
                    turned = max(
                        0.0, turned + delta * math.copysign(1.0, angular))
                    if turned >= target_turn:
                        break
                last_yaw = yaw
                scan_safe = (self.scan_is_fresh()
                             and self.nearest_front > float(
                                 self.get_parameter('stop_distance').value))
                if linear > 0.0 and not scan_safe:
                    if blocked_since is None:
                        blocked_since = now
                    self.publisher.publish(Twist())
                    if now - blocked_since > 3.0:
                        raise RuntimeError(
                            '雷达数据缺失或前方受阻超过 3 秒，建图巡航已停止。')
                    self.get_logger().warning(
                        '雷达无新数据或前方距离过近，暂停直行。',
                        throttle_duration_sec=1.0)
                else:
                    blocked_since = None
                    command = Twist()
                    command.linear.x = linear
                    command.angular.z = angular
                    self.publisher.publish(command)
                time.sleep(0.1)
        finally:
            if rclpy.ok():
                self.publisher.publish(Twist())

    def run(self):
        # 按里程计转一周并走一个约 0.9 m 的方形；受阻则停车退出。
        linear = abs(float(self.get_parameter('linear_speed').value))
        angular = abs(float(self.get_parameter('angular_speed').value))
        side_length = abs(float(self.get_parameter('side_length').value))
        turn_angle = abs(float(self.get_parameter('turn_angle').value))
        if not all(math.isfinite(value) for value in (
                linear, angular, side_length, turn_angle)):
            raise ValueError('建图运动参数必须是有限数值')
        stop_distance = float(self.get_parameter('stop_distance').value)
        if not math.isfinite(stop_distance) or stop_distance < 0.0:
            raise ValueError('stop_distance 必须是非负的有限数值')
        if linear <= 0.0 or angular <= 0.0:
            raise ValueError('linear_speed 和 angular_speed 必须大于 0')
        segments = [(0.0, angular, 2.0 * math.pi / angular)]
        for _ in range(4):
            segments.extend([
                (linear, 0.0, side_length / linear),
                (0.0, angular, turn_angle / angular),
            ])
        for index, segment in enumerate(segments, start=1):
            self.get_logger().info(f'执行建图动作 {index}/{len(segments)}')
            self.run_segment(*segment)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MappingPatrol()
    success = True
    try:
        node.run()
    except KeyboardInterrupt:
        success = False
    except (ValueError, RuntimeError) as error:
        node.get_logger().error(str(error))
        success = False
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
    return 0 if success else 1
