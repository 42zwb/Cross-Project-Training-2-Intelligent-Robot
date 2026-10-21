#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy


class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('enable_button', -1)
        self.declare_parameter('max_linear_speed', 0.50)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('joy_timeout', 0.60)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.last_joy = None
        self.is_stopped = True
        self.create_timer(0.1, self.stop_if_stale)

    def on_joy(self, joy):
        linear_axis = self.get_parameter('linear_axis').value
        angular_axis = self.get_parameter('angular_axis').value
        enable_button = self.get_parameter('enable_button').value
        if linear_axis < 0 or angular_axis < 0 or enable_button < -1:
            self.get_logger().error(
                '手柄轴编号必须非负，使能键只能为 -1 或非负编号。',
                throttle_duration_sec=2.0)
            self.publish_stop()
            return
        required_axis = max(linear_axis, angular_axis)
        if required_axis >= len(joy.axes):
            self.get_logger().warning('Joy 消息轴数量不足，已忽略。', throttle_duration_sec=2.0)
            self.publish_stop()
            return
        self.last_joy = time.monotonic()
        enabled = enable_button == -1 or (
            enable_button < len(joy.buttons) and joy.buttons[enable_button] == 1)
        msg = Twist()
        if enabled:
            linear = joy.axes[linear_axis]
            angular = joy.axes[angular_axis]
            deadzone_value = float(self.get_parameter('deadzone').value)
            linear_speed = float(self.get_parameter('max_linear_speed').value)
            angular_speed = float(self.get_parameter('max_angular_speed').value)
            if not all(math.isfinite(value) for value in (
                    linear, angular, deadzone_value, linear_speed, angular_speed)):
                self.get_logger().error(
                    'Joy 轴值或速度参数不是有限数值，已停止。',
                    throttle_duration_sec=2.0)
                self.publish_stop()
                return
            deadzone = min(max(deadzone_value, 0.0), 1.0)
            linear = min(max(linear, -1.0), 1.0)
            angular = min(max(angular, -1.0), 1.0)
            msg.linear.x = 0.0 if abs(linear) < deadzone else (
                linear * abs(linear_speed))
            msg.angular.z = 0.0 if abs(angular) < deadzone else (
                angular * abs(angular_speed))
        self.is_stopped = msg.linear.x == 0.0 and msg.angular.z == 0.0
        self.publisher.publish(msg)

    def publish_stop(self):
        self.is_stopped = True
        self.publisher.publish(Twist())

    def stop_if_stale(self):
        if self.last_joy is None or self.is_stopped:
            return
        timeout = float(self.get_parameter('joy_timeout').value)
        if not math.isfinite(timeout):
            timeout = 0.0
        timeout = max(timeout, 0.0)
        if time.monotonic() - self.last_joy > timeout:
            self.get_logger().warning(
                '手柄数据超时，已发送停止指令。', throttle_duration_sec=2.0)
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = JoystickControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
