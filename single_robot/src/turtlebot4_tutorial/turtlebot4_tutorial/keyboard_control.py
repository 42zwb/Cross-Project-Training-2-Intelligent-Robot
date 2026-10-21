#!/usr/bin/env python3
import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


HELP = """
TurtleBot 4 键盘控制
--------------------
W/S：前进/后退    A/D：左转/右转
空格：停止        Q：退出
"""


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.80)
        self.declare_parameter('key_timeout', 0.60)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.command = Twist()
        self.last_motion_key = None
        self.is_stopped = True

    def publish_key(self, key):
        msg = Twist()
        linear_value = float(self.get_parameter('linear_speed').value)
        angular_value = float(self.get_parameter('angular_speed').value)
        if not math.isfinite(linear_value) or not math.isfinite(angular_value):
            self.get_logger().error('速度参数不是有限数值，已停止。')
            self.stop()
            return
        linear = abs(linear_value)
        angular = abs(angular_value)
        if key == 'w':
            msg.linear.x = linear
        elif key == 's':
            msg.linear.x = -linear
        elif key == 'a':
            msg.angular.z = angular
        elif key == 'd':
            msg.angular.z = -angular
        self.command = msg
        self.last_motion_key = time.monotonic() if key != ' ' else None
        self.is_stopped = key == ' '
        self.publisher.publish(msg)

    def publish_current(self):
        if self.last_motion_key is None:
            return
        timeout = max(float(self.get_parameter('key_timeout').value), 0.0)
        if time.monotonic() - self.last_motion_key <= timeout:
            self.publisher.publish(self.command)
        elif not self.is_stopped:
            self.stop()

    def stop(self):
        self.command = Twist()
        self.last_motion_key = None
        self.is_stopped = True
        self.publisher.publish(Twist())


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1).lower() if ready else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardControl()
    if not sys.stdin.isatty():
        node.get_logger().error('键盘控制需要在交互式终端中运行。')
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings)
            if key in ('q', '\x03'):
                break
            if key in ('w', 's', 'a', 'd', ' '):
                node.publish_key(key)
            else:
                node.publish_current()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
    return 0
