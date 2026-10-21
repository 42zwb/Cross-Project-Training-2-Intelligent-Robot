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


class RobotPatrolClient:
    def __init__(self, owner, namespace, waypoints, goal_timeout):
        self.owner = owner
        self.namespace = namespace.strip('/')
        action_name = f'/{self.namespace}/navigate_to_pose'
        self.client = ActionClient(owner, NavigateToPose, action_name)
        self.waypoints = waypoints
        self.index = 0
        self.goal_timeout = goal_timeout
        self.goal_started = None
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None
        self.cancel_future = None
        self.completed = False
        self.failed = False

    def start(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.owner.get_logger().error(
                f'机器人 {self.namespace} 的 Nav2 动作服务器不可用。')
            self.failed = True
            return
        self.send_next()

    def send_next(self):
        if self.index >= len(self.waypoints):
            self.completed = True
            self.owner.get_logger().info(f'{self.namespace} 完成巡航。')
            return
        x, y, yaw = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(self.owner, x, y, yaw)
        self.owner.get_logger().info(
            f'{self.namespace} -> 目标 {self.index + 1}: ({x:.2f}, {y:.2f})')
        future = self.client.send_goal_async(goal)
        self.goal_future = future
        self.goal_started = time.monotonic()
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.owner.get_logger().error(
                f'{self.namespace} 发送目标失败: {error}')
            self.failed = True
            return
        if not handle.accepted:
            self.owner.get_logger().error(f'{self.namespace} 的目标被拒绝。')
            self.failed = True
            return
        self.goal_handle = handle
        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(self.on_result)
        if self.failed:
            self.cancel()

    def on_result(self, future):
        if self.failed:
            return
        try:
            status = future.result().status
        except Exception as error:
            self.owner.get_logger().error(
                f'{self.namespace} 读取导航结果失败: {error}')
            self.failed = True
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.owner.get_logger().error(f'{self.namespace} 导航失败。')
            self.failed = True
            return
        self.index += 1
        self.goal_started = None
        self.goal_handle = None
        self.send_next()

    def check_timeout(self):
        if (self.completed or self.failed or self.goal_started is None
                or time.monotonic() - self.goal_started <= self.goal_timeout):
            return
        self.owner.get_logger().error(
            f'{self.namespace} 的导航目标超过 {self.goal_timeout:.1f} 秒，正在取消。')
        self.cancel()

    def cancel(self):
        if self.completed:
            return
        self.failed = True
        if (self.goal_handle is not None and self.result_future is not None
                and not self.result_future.done() and self.cancel_future is None):
            self.cancel_future = self.goal_handle.cancel_goal_async()


class MultiRobotPatrol(Node):
    def __init__(self, namespaces, goal_timeout):
        super().__init__('multi_robot_patrol')
        routes = [
            [(0.8, 0.0, 0.0), (0.8, 0.8, math.pi / 2.0)],
            [(-0.8, 0.0, 0.0), (-0.8, -0.8, -math.pi / 2.0)],
        ]
        self.robots = [
            RobotPatrolClient(
                self, namespace, routes[index % len(routes)], goal_timeout)
            for index, namespace in enumerate(namespaces)
        ]
        self.create_timer(0.5, self.check_timeouts)

    def check_timeouts(self):
        for robot in self.robots:
            robot.check_timeout()

    def start(self):
        for robot in self.robots:
            robot.start()

    def finished(self):
        return all(robot.completed or robot.failed for robot in self.robots)

    def cancel_all(self):
        for robot in self.robots:
            robot.cancel()
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            pending = any(
                not robot.completed and (
                    (robot.goal_future is not None
                     and not robot.goal_future.done())
                    or (robot.cancel_future is not None
                        and not robot.cancel_future.done())
                    or (robot.result_future is not None
                        and not robot.result_future.done()))
                for robot in self.robots)
            if not pending:
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        for robot in self.robots:
            if (robot.goal_handle is not None and robot.result_future is not None
                    and not robot.result_future.done()):
                self.get_logger().error(
                    f'{robot.namespace} 的导航取消未在 5 秒内完成。')


def main():
    parser = argparse.ArgumentParser(description='多机器人命名空间巡航示例')
    parser.add_argument('namespaces', nargs='*', default=['robot1', 'robot2'])
    parser.add_argument('--goal-timeout', type=float, default=180.0,
                        help='每个导航目标的超时秒数')
    args = parse_cli_args(parser)
    try:
        namespaces = [
            normalize_namespace(namespace, allow_empty=False)
            for namespace in args.namespaces
        ]
    except Exception as error:
        parser.error(str(error))
    if not namespaces:
        parser.error('至少需要一个机器人命名空间')
    if len(namespaces) != len(set(namespaces)):
        parser.error('机器人命名空间不能重复')
    if not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0.0:
        parser.error('goal-timeout 必须是大于 0 的有限数值')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MultiRobotPatrol(namespaces, args.goal_timeout)
    node.start()
    interrupted = False
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning('用户中断多机器人巡航。')
        interrupted = True
    finally:
        failed = interrupted or any(robot.failed for robot in node.robots)
        node.cancel_all()
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if failed else 0
