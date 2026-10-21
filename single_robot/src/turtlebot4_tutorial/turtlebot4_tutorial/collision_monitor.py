#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class CollisionMonitor(Node):
    """Block forward commands when the front laser sector is too close."""

    def __init__(self):
        super().__init__('collision_monitor')
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.70)
        self.declare_parameter('front_angle', 0.70)
        self.declare_parameter('scan_timeout', 1.0)
        self.nearest_front = math.inf
        self.has_front_measurement = False
        self.last_scan = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_command, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def on_scan(self, scan):
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
        self.last_scan = time.monotonic()

    def scan_is_fresh(self):
        timeout = float(self.get_parameter('scan_timeout').value)
        if not math.isfinite(timeout):
            return False
        timeout = max(timeout, 0.0)
        return (self.has_front_measurement
                and self.last_scan is not None
                and time.monotonic() - self.last_scan <= timeout)

    def on_command(self, command):
        output = Twist()
        output.linear.x = command.linear.x
        output.linear.y = command.linear.y
        output.linear.z = command.linear.z
        output.angular.x = command.angular.x
        output.angular.y = command.angular.y
        output.angular.z = command.angular.z
        components = (
            output.linear.x, output.linear.y, output.linear.z,
            output.angular.x, output.angular.y, output.angular.z)
        if not all(math.isfinite(value) for value in components):
            self.get_logger().error(
                '速度指令包含非有限数值，已改为停车。',
                throttle_duration_sec=1.0)
            self.publisher.publish(Twist())
            return
        stop = float(self.get_parameter('stop_distance').value)
        slow = float(self.get_parameter('slow_distance').value)
        distances_are_valid = (
            math.isfinite(stop) and math.isfinite(slow)
            and stop >= 0.0 and slow > stop)
        if output.linear.x > 0.0 and not distances_are_valid:
            output.linear.x = 0.0
            self.get_logger().error(
                'stop_distance 和 slow_distance 参数无效，已停止前进。',
                throttle_duration_sec=1.0)
        elif output.linear.x > 0.0 and not self.scan_is_fresh():
            output.linear.x = 0.0
            self.get_logger().warning(
                '尚未收到有效雷达数据或雷达数据已超时，已停止前进。',
                throttle_duration_sec=1.0)
        elif output.linear.x > 0.0 and self.nearest_front <= stop:
            output.linear.x = 0.0
            self.get_logger().warning(
                f'前方障碍物 {self.nearest_front:.2f} m，已停止前进。',
                throttle_duration_sec=1.0)
        elif output.linear.x > 0.0 and self.nearest_front < slow:
            scale = (self.nearest_front - stop) / (slow - stop)
            output.linear.x *= max(0.0, min(1.0, scale))
        self.publisher.publish(output)


def main(args=None):
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
