#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from .common import parse_cli_args


def create_initial_pose_publisher(node):
    """Create an AMCL-compatible publisher for the initial pose topic."""
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    return node.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos)


def matches_initial_pose(pose, x, y, yaw, first_stamp_ns, now_ns):
    """Accept only a fresh, finite pose consistent with the requested start."""
    stamp = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec
    p, q = pose.pose.pose.position, pose.pose.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, *pose.pose.covariance)
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if (pose.header.frame_id != 'map' or first_stamp_ns <= 0 or stamp < first_stamp_ns
            or stamp > now_ns + 1_000_000_000 or now_ns - stamp > 2_000_000_000
            or not all(math.isfinite(value) for value in values)
            or any(pose.pose.covariance[index] < 0 for index in (0, 7, 14, 21, 28, 35))
            or not 0.99 <= norm <= 1.01):
        return False
    actual_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                            1 - 2 * (q.y*q.y + q.z*q.z))
    yaw_error = abs(math.atan2(math.sin(actual_yaw - yaw), math.cos(actual_yaw - yaw)))
    return math.hypot(p.x - x, p.y - y) <= 1.0 and yaw_error <= 0.8


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

    latest_amcl_pose = None
    first_stamp_ns = 0

    def on_amcl_pose(pose):
        nonlocal latest_amcl_pose
        latest_amcl_pose = pose

    if args.wait_for_amcl:
        # A best-effort reader accepts either AMCL publisher reliability level.
        node.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', on_amcl_pose,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        lifecycle = node.create_client(GetState, 'amcl/get_state')
        state_future = None
        amcl_active = False
        active_checked_at = 0.0
        tf_buffer = Buffer()
        node.tf_listener = TransformListener(tf_buffer, node)

    node.get_logger().info(
        f'发布 AMCL 初始位姿: x={args.x:.2f}, y={args.y:.2f}, '
        f'yaw={args.yaw:.2f}')
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            message.header.stamp = node.get_clock().now().to_msg()
            stamp_ns = (message.header.stamp.sec * 1_000_000_000
                        + message.header.stamp.nanosec)
            if not first_stamp_ns and stamp_ns > 0:
                first_stamp_ns = stamp_ns
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.5)
            if args.wait_for_amcl:
                if state_future is not None and state_future.done():
                    try:
                        response = state_future.result()
                        amcl_active = (response is not None
                                       and response.current_state.label == 'active')
                        active_checked_at = time.monotonic()
                    except Exception:
                        amcl_active = False
                    state_future = None
                if state_future is None and lifecycle.service_is_ready():
                    state_future = lifecycle.call_async(GetState.Request())
            now_ns = node.get_clock().now().nanoseconds
            pose_confirmed = (latest_amcl_pose is not None and matches_initial_pose(
                latest_amcl_pose, args.x, args.y, args.yaw, first_stamp_ns, now_ns))
            tf_confirmed = False
            if args.wait_for_amcl and tf_buffer.can_transform('map', 'odom', Time()):
                try:
                    transform = tf_buffer.lookup_transform('map', 'odom', Time())
                    tf_stamp = (transform.header.stamp.sec * 1_000_000_000
                                + transform.header.stamp.nanosec)
                    t, q = transform.transform.translation, transform.transform.rotation
                    tf_confirmed = (tf_stamp >= first_stamp_ns > 0
                                    and now_ns - 2_000_000_000 <= tf_stamp
                                    <= now_ns + 2_000_000_000
                                    and all(math.isfinite(v) for v in
                                            (t.x, t.y, t.z, q.x, q.y, q.z, q.w))
                                    and 0.99 <= math.sqrt(
                                        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) <= 1.01)
                except TransformException:
                    pass
            if (args.wait_for_amcl and pose_confirmed and amcl_active and tf_confirmed
                    and time.monotonic() - active_checked_at <= 2.0):
                node.get_logger().info('AMCL 已确认初始位姿。')
                return 0
        if args.wait_for_amcl:
            node.get_logger().error(
                f'{args.duration:.1f} 秒内 AMCL 初始位姿、生命周期或 map 到 odom TF 未就绪；'
                '请检查地图、雷达、里程计和 AMCL。')
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
