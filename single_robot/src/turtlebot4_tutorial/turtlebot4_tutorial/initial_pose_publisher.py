#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .common import parse_cli_args


def create_initial_pose_publisher(node):
    """Create an AMCL-compatible publisher for the initial pose topic."""
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    return node.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos)


def main():
    parser = argparse.ArgumentParser(description='向 AMCL 发布初始位姿')
    parser.add_argument('x', nargs='?', type=float, default=0.0)
    parser.add_argument('y', nargs='?', type=float, default=0.0)
    parser.add_argument('yaw', nargs='?', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--wait-for-amcl', action='store_true',
                        help='持续发布直到收到 AMCL 位姿，超时则报错')
    args = parse_cli_args(parser)
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error('duration 必须是大于 0 的有限数值')
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        parser.error('x、y 和 yaw 必须是有限数值')

    rclpy.init()
    node = Node('initial_pose_publisher')
    publisher = create_initial_pose_publisher(node)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.pose.pose.position.x = args.x
    message.pose.pose.position.y = args.y
    message.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    message.pose.covariance[0] = 0.25
    message.pose.covariance[7] = 0.25
    message.pose.covariance[35] = 0.0685

    received_amcl_pose = False

    def on_amcl_pose(pose):
        nonlocal received_amcl_pose
        if pose.header.frame_id == 'map':
            received_amcl_pose = True

    if args.wait_for_amcl:
        # A best-effort reader accepts either AMCL publisher reliability level.
        node.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', on_amcl_pose,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    node.get_logger().info(
        f'发布 AMCL 初始位姿: x={args.x:.2f}, y={args.y:.2f}, '
        f'yaw={args.yaw:.2f}')
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.5)
            if received_amcl_pose:
                node.get_logger().info('AMCL 已确认初始位姿。')
                return 0
        if args.wait_for_amcl:
            node.get_logger().error(
                f'{args.duration:.1f} 秒内没有收到 AMCL 位姿；'
                '请检查地图、雷达、里程计和 AMCL 生命周期。')
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
