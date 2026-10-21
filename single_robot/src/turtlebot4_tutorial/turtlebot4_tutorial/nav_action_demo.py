#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .common import make_pose, normalize_namespace, parse_cli_args


class NavigateActionDemo(Node):
    def __init__(self, namespace=''):
        super().__init__('nav_action_demo', namespace=namespace)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None
        self.done = False
        self.success = False

    def send(self, x, y, yaw):
        self.get_logger().info('等待 navigate_to_pose 动作服务器……')
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('30 秒内未找到 Nav2 动作服务器')
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(self, x, y, yaw)
        self.goal_future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        self.goal_future.add_done_callback(self.on_goal_response)

    def on_feedback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f'剩余距离: {feedback.distance_remaining:.2f} m，'
            f'恢复次数: {feedback.number_of_recoveries}',
            throttle_duration_sec=2.0)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'发送导航目标失败: {error}')
            self.done = True
            return
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝。')
            self.done = True
            return
        self.goal_handle = goal_handle
        self.get_logger().info('目标已接受。')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().error(f'读取导航结果失败: {error}')
            self.done = True
            return
        self.success = status == GoalStatus.STATUS_SUCCEEDED
        if self.success:
            self.get_logger().info('目标导航成功。')
        else:
            self.get_logger().error(f'目标导航结束，状态码: {status}')
        self.done = True

    def cancel(self, timeout=5.0):
        """Wait for cancellation acknowledgement and a terminal result."""
        if self.goal_future is None:
            return True
        deadline = time.monotonic() + timeout
        while (rclpy.ok() and self.goal_handle is None and not self.done
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.goal_handle is None:
            if not self.done:
                self.get_logger().error('目标发送结果未返回，无法确认取消。')
            return self.done
        if self.done:
            return True
        future = self.goal_handle.cancel_goal_async()
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done() or not future.result().goals_canceling:
            self.get_logger().error('导航服务器未确认取消目标。')
            return False
        while rclpy.ok() and not self.done and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.done:
            self.get_logger().warning('取消已确认，但导航结束结果尚未返回。')
        return self.done


def main():
    parser = argparse.ArgumentParser(description='原生 ROS 2 NavigateToPose 动作示例')
    parser.add_argument('x', nargs='?', type=float, default=0.8)
    parser.add_argument('y', nargs='?', type=float, default=0.0)
    parser.add_argument('yaw', nargs='?', type=float, default=0.0)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='导航目标超时秒数')
    args = parse_cli_args(parser)
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        parser.error('x、y 和 yaw 必须是有限数值')
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error('timeout 必须是大于 0 的有限数值')
    try:
        namespace = normalize_namespace(args.namespace)
    except Exception as error:
        parser.error(str(error))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavigateActionDemo(namespace)
    timed_out = False
    try:
        node.send(args.x, args.y, args.yaw)
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= deadline:
                node.get_logger().error(
                    f'导航目标超过 {args.timeout:.1f} 秒，正在取消。')
                node.cancel()
                timed_out = True
                break
    except RuntimeError as error:
        node.get_logger().error(str(error))
    except KeyboardInterrupt:
        node.get_logger().warning('用户中断导航。')
        node.cancel()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.success and not timed_out else 1
