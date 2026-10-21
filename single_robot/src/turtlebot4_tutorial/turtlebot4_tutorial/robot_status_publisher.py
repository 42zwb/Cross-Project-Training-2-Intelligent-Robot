#!/usr/bin/env python3
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.declare_parameter('stale_timeout', 2.0)
        self.publisher = self.create_publisher(
            DiagnosticArray, 'tutorial/robot_status', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, 10)
        self.last_odom = None
        self.last_scan = None
        self.last_battery = None
        self.odom = None
        self.scan_min = None
        self.battery_percent = None
        self.create_timer(1.0, self.publish_status)

    def on_odom(self, message):
        self.last_odom = time.monotonic()
        self.odom = message

    def on_scan(self, message):
        self.last_scan = time.monotonic()
        finite = [value for value in message.ranges if math.isfinite(value)]
        self.scan_min = min(finite) if finite else None

    def on_battery(self, message):
        self.last_battery = time.monotonic()
        self.battery_percent = message.percentage

    def publish_status(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('stale_timeout').value)
        if not math.isfinite(timeout):
            timeout = 0.0
        timeout = max(timeout, 0.0)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        values = []
        level = DiagnosticStatus.OK
        problems = []

        if self.last_odom is None or now - self.last_odom > timeout:
            level = DiagnosticStatus.STALE
            problems.append('odom 超时')
        else:
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist
            values.extend([
                KeyValue(key='position_x_m', value=f'{p.x:.3f}'),
                KeyValue(key='position_y_m', value=f'{p.y:.3f}'),
                KeyValue(key='linear_speed_mps', value=f'{v.linear.x:.3f}'),
                KeyValue(key='angular_speed_radps', value=f'{v.angular.z:.3f}'),
            ])
        if self.last_scan is None or now - self.last_scan > timeout:
            level = max(level, DiagnosticStatus.STALE)
            problems.append('scan 超时')
        elif self.scan_min is not None:
            values.append(KeyValue(key='nearest_obstacle_m', value=f'{self.scan_min:.3f}'))
        if (self.last_battery is not None and now - self.last_battery <= timeout
                and self.battery_percent is not None
                and math.isfinite(self.battery_percent)):
            values.append(KeyValue(
                key='battery_percent', value=f'{self.battery_percent * 100.0:.1f}'))
        elif self.last_battery is not None:
            level = max(level, DiagnosticStatus.STALE)
            problems.append('battery_state 超时')

        status = DiagnosticStatus()
        status.name = 'TurtleBot4 Tutorial Status'
        status.hardware_id = 'turtlebot4'
        status.level = level
        status.message = '正常' if not problems else '；'.join(problems)
        status.values = values
        array.status = [status]
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
