#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.signals import SignalHandlerOptions

from .common import make_pose, normalize_namespace, parse_cli_args


def wait_for_lifecycle_active(navigator, node_name, deadline):
    client = navigator.create_client(GetState, f'{node_name}/get_state')
    request = GetState.Request()
    while rclpy.ok() and time.monotonic() < deadline:
        if not client.wait_for_service(timeout_sec=0.5):
            continue
        future = client.call_async(request)
        remaining = max(deadline - time.monotonic(), 0.0)
        rclpy.spin_until_future_complete(
            navigator, future, timeout_sec=min(remaining, 1.0))
        if future.done():
            try:
                response = future.result()
            except Exception as error:
                navigator.get_logger().warning(
                    f'读取 {node_name} 生命周期状态失败: {error}')
                continue
            if (response is not None
                    and response.current_state.label == 'active'):
                return True
    return False


def wait_for_nav2_active(navigator, timeout, set_initial_pose):
    deadline = time.monotonic() + timeout
    if not wait_for_lifecycle_active(navigator, 'amcl', deadline):
        return False
    while (rclpy.ok() and not navigator.initial_pose_received
           and time.monotonic() < deadline):
        if set_initial_pose:
            navigator.initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator._setInitialPose()
        rclpy.spin_once(navigator, timeout_sec=0.5)
    if not navigator.initial_pose_received:
        return False
    return wait_for_lifecycle_active(navigator, 'bt_navigator', deadline)


def wait_for_task(navigator, deadline, timeout):
    while not navigator.isTaskComplete():
        if time.monotonic() >= deadline:
            navigator.get_logger().error(
                f'导航目标超过 {timeout:.1f} 秒，正在取消。')
            return False
        feedback = navigator.getFeedback()
        if feedback is not None:
            remaining = feedback.distance_remaining
            navigator.get_logger().info(f'剩余距离: {remaining:.2f} m',
                                        throttle_duration_sec=2.0)
        time.sleep(0.1)
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航成功。')
        return True
    if result == TaskResult.CANCELED:
        navigator.get_logger().warning('导航已取消。')
    else:
        navigator.get_logger().error('导航失败。')
    return False


class GoalSender:
    """Track a goal response even when it arrives after timeout or interruption."""

    def __init__(self, navigator):
        self.navigator = navigator
        self.future = None
        self.cancel_requested = False
        self.cancel_future = None
        self.response_processed = False
        self.response_failed = False

    def on_response(self, future):
        if self.response_processed:
            return
        self.response_processed = True
        try:
            handle = future.result()
            if not handle.accepted:
                return
            self.navigator.goal_handle = handle
            try:
                self.navigator.result_future = handle.get_result_async()
            except Exception:
                # An accepted goal may already be moving even if result setup fails.
                self.cancel_requested = True
                raise
            if self.cancel_requested:
                self.cancel_future = handle.cancel_goal_async()
        except Exception as error:
            self.response_failed = True
            self.navigator.get_logger().error(f'导航目标响应失败: {error}')
            if self.cancel_requested and self.navigator.goal_handle is not None:
                try:
                    self.cancel_future = self.navigator.goal_handle.cancel_goal_async()
                except Exception as cancel_error:
                    self.navigator.get_logger().error(f'取消目标失败: {cancel_error}')

    def send(self, pose, deadline):
        client = self.navigator.nav_to_pose_client
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if client.wait_for_server(timeout_sec=min(0.2, max(0.0, remaining))):
                break
        else:
            return False
        if time.monotonic() >= deadline:
            return False
        self.navigator.goal_handle = None
        self.navigator.result_future = None
        self.navigator.feedback = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.future = client.send_goal_async(goal, self.navigator._feedbackCallback)
        self.future.add_done_callback(self.on_response)
        while rclpy.ok() and time.monotonic() < deadline and not self.response_processed:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.navigator, timeout_sec=min(0.2, remaining))
        return (self.response_processed and not self.response_failed
                and self.navigator.goal_handle is not None
                and self.navigator.result_future is not None
                and time.monotonic() < deadline)

    def cancel_and_drain(self, timeout=5.0):
        self.cancel_requested = True
        if self.navigator.goal_handle is not None and self.cancel_future is None:
            return cancel_task(self.navigator, timeout)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.future is None:
                return True
            if self.response_processed:
                if self.navigator.goal_handle is None:
                    return True
                if self.cancel_future is not None and self.cancel_future.done():
                    try:
                        if not self.cancel_future.result().goals_canceling:
                            break
                    except Exception:
                        break
                    if self.navigator.result_future is None:
                        self.navigator.get_logger().warning(
                            '取消已确认，但没有导航结束结果；请检查机器人状态。')
                        return False
                    if result_is_terminal(self.navigator.result_future):
                        return True
                    if self.navigator.result_future.done():
                        break
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        self.navigator.get_logger().error('目标响应或取消未确认结束；请检查服务器和机器人状态。')
        return False


def result_is_terminal(future):
    if future is None or not future.done():
        return False
    try:
        return future.result().status in (
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED)
    except Exception:
        return False


def cancel_task(navigator, timeout=5.0):
    """Cancel without the unbounded wait in BasicNavigator.cancelTask."""
    if navigator.goal_handle is None or result_is_terminal(navigator.result_future):
        return True
    deadline = time.monotonic() + timeout
    future = navigator.goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout)
    if not future.done() or not future.result().goals_canceling:
        navigator.get_logger().error('导航服务器未确认取消目标。')
        return False
    if navigator.result_future is None:
        navigator.get_logger().error('取消已确认，但结果订阅不可用，无法确认目标结束。')
        return False
    remaining = max(0.0, deadline - time.monotonic())
    rclpy.spin_until_future_complete(
        navigator, navigator.result_future, timeout_sec=remaining)
    if not result_is_terminal(navigator.result_future):
        navigator.get_logger().warning('取消已确认，但导航结束结果尚未确认。')
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description='Nav2 Simple Commander 巡航示例')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--initial-x', type=float, default=0.0)
    parser.add_argument('--initial-y', type=float, default=0.0)
    parser.add_argument('--initial-yaw', type=float, default=0.0)
    parser.add_argument('--set-initial-pose', action='store_true',
                        help='向 AMCL 重新发布初始位姿；默认使用现有定位')
    parser.add_argument('--loops', type=int, default=1,
                        help='巡航圈数；0 表示持续循环')
    parser.add_argument('--goal-timeout', type=float, default=180.0,
                        help='每个导航目标的超时秒数')
    parser.add_argument('--nav2-timeout', type=float, default=60.0,
                        help='等待 Nav2 和 AMCL 激活的超时秒数')
    args = parse_cli_args(parser)
    if args.loops < 0:
        parser.error('loops 不能为负数')
    initial_pose = (args.initial_x, args.initial_y, args.initial_yaw)
    if not all(math.isfinite(value) for value in initial_pose):
        parser.error('初始位姿必须是有限数值')
    if not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0.0:
        parser.error('goal-timeout 必须是大于 0 的有限数值')
    if not math.isfinite(args.nav2_timeout) or args.nav2_timeout <= 0.0:
        parser.error('nav2-timeout 必须是大于 0 的有限数值')
    try:
        namespace = normalize_namespace(args.namespace)
    except Exception as error:
        parser.error(str(error))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    navigator = BasicNavigator(namespace=namespace)
    set_initial_pose = args.set_initial_pose or initial_pose != (0.0, 0.0, 0.0)
    if set_initial_pose:
        navigator.setInitialPose(make_pose(navigator, *initial_pose))
    success = True
    sender = None
    try:
        navigator.get_logger().info('等待 Nav2 激活……')
        if not wait_for_nav2_active(
                navigator, args.nav2_timeout, set_initial_pose):
            raise RuntimeError(
                f'Nav2 或 AMCL 定位未在 {args.nav2_timeout:.1f} 秒内就绪；'
                '可用 --set-initial-pose 发布初始位姿')

        # 仓库地图中心附近的一组保守目标，可按自己的地图修改。
        waypoints = [
            (0.8, 0.0, 0.0),
            (0.8, 0.8, math.pi / 2.0),
            (0.0, 0.8, math.pi),
            (0.0, 0.0, -math.pi / 2.0),
        ]
        loop = 0
        while rclpy.ok() and (args.loops == 0 or loop < args.loops):
            for index, (x, y, yaw) in enumerate(waypoints, start=1):
                navigator.get_logger().info(
                    f'发送目标 {index}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
                deadline = time.monotonic() + args.goal_timeout
                sender = GoalSender(navigator)
                accepted = sender.send(make_pose(navigator, x, y, yaw), deadline)
                if not accepted:
                    raise RuntimeError(f'目标 {index} 未在期限内被动作服务器接受')
                if not wait_for_task(navigator, deadline, args.goal_timeout):
                    raise RuntimeError(f'目标 {index} 未完成')
                sender = None
            loop += 1
    except (KeyboardInterrupt, RuntimeError) as error:
        navigator.get_logger().warning(str(error) or '用户中断巡航。')
        success = False
    except Exception as error:
        navigator.get_logger().error(f'导航发生异常: {error}')
        success = False
    finally:
        if sender is not None:
            try:
                if not sender.cancel_and_drain():
                    success = False
            except Exception as error:
                navigator.get_logger().error(f'取消导航失败，需检查机器人状态: {error}')
                success = False
        navigator.destroy_node()
        rclpy.try_shutdown()
    return 0 if success else 1
