#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from lifecycle_msgs.srv import GetState
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


def wait_for_task(navigator, timeout):
    deadline = time.monotonic() + timeout
    while not navigator.isTaskComplete():
        if time.monotonic() >= deadline:
            navigator.get_logger().error(
                f'导航目标超过 {timeout:.1f} 秒，正在取消。')
            cancel_task(navigator)
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


def cancel_task(navigator, timeout=5.0):
    """Cancel without the unbounded wait in BasicNavigator.cancelTask."""
    if (navigator.goal_handle is None or navigator.result_future is None
            or navigator.result_future.done()):
        return True
    deadline = time.monotonic() + timeout
    future = navigator.goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout)
    if not future.done() or not future.result().goals_canceling:
        navigator.get_logger().error('导航服务器未确认取消目标。')
        return False
    remaining = max(0.0, deadline - time.monotonic())
    rclpy.spin_until_future_complete(
        navigator, navigator.result_future, timeout_sec=remaining)
    if not navigator.result_future.done():
        navigator.get_logger().warning('取消已确认，但导航结束结果尚未返回。')
    return navigator.result_future.done()


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
                accepted = navigator.goToPose(make_pose(navigator, x, y, yaw))
                if not accepted:
                    raise RuntimeError(f'目标 {index} 被动作服务器拒绝')
                if not wait_for_task(navigator, args.goal_timeout):
                    raise RuntimeError(f'目标 {index} 未完成')
            loop += 1
    except (KeyboardInterrupt, RuntimeError) as error:
        navigator.get_logger().warning(str(error) or '用户中断巡航。')
        cancel_task(navigator)
        success = False
    finally:
        navigator.destroy_node()
        rclpy.try_shutdown()
    return 0 if success else 1
