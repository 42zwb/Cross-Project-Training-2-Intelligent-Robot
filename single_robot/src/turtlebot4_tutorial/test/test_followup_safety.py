import ast
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.task import Future

from turtlebot4_tutorial import multi_robot_patrol, nav_commander_demo, nav_action_demo


class Logger:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


def test_navigation_initial_pose_remaps_both_tf_topics():
    launch_file = Path(__file__).resolve().parents[1] / 'launch/tutorial_navigation.launch.py'
    tree = ast.parse(launch_file.read_text())
    assignments = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'initial_pose' for t in n.targets)]
    assert len(assignments) == 1
    keywords = {k.arg: k.value for k in assignments[0].value.keywords}
    assert ast.literal_eval(keywords['remappings']) == [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def test_multirobot_starts_no_goals_while_waiting_for_other_server(monkeypatch):
    now, sends = [0.0], []
    monkeypatch.setattr(multi_robot_patrol.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        rclpy, 'spin_once',
        lambda node, timeout_sec: now.__setitem__(0, now[0] + timeout_sec))
    robots = [SimpleNamespace(namespace='r1', client=SimpleNamespace(server_is_ready=lambda: True),
                              send_next=lambda: sends.append(('r1', now[0])), failed=False),
              SimpleNamespace(namespace='r2',
                              client=SimpleNamespace(server_is_ready=lambda: now[0] >= 4),
                              send_next=lambda: sends.append(('r2', now[0])), failed=False)]
    fake = SimpleNamespace(robots=robots, get_logger=lambda: Logger())
    multi_robot_patrol.MultiRobotPatrol.start(fake)
    assert len(sends) == 2 and all(t >= 4 for _, t in sends)


def test_multirobot_result_setup_exception_cancels_accepted_goal():
    rclpy.init()
    node = rclpy.create_node('goal_setup_failure_test')
    try:
        robot = multi_robot_patrol.RobotPatrolClient(node, 'robot1', [(0, 0, 0)], 5)
        canceled = []
        def fail(): raise ValueError('result client failed')
        handle = SimpleNamespace(accepted=True, get_result_async=fail,
                                 cancel_goal_async=lambda: canceled.append(True) or Future())
        response = Future()
        response.set_result(handle)
        robot.on_goal_response(response)
        assert robot.failed and canceled and robot.cancel_future is not None
        robot.client.destroy()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_commander_result_setup_exception_cancels_accepted_goal():
    canceled = []
    def fail(): raise ValueError('result setup failed')
    handle = SimpleNamespace(accepted=True, get_result_async=fail,
                             cancel_goal_async=lambda: canceled.append(True) or Future())
    navigator = SimpleNamespace(goal_handle=None, result_future=None, get_logger=lambda: Logger())
    sender = nav_commander_demo.GoalSender(navigator)
    response = Future()
    response.set_result(handle)
    sender.on_response(response)
    assert canceled and sender.cancel_requested and sender.cancel_future is not None


def test_action_result_setup_exception_cancels_accepted_goal():
    rclpy.init()
    node = nav_action_demo.NavigateActionDemo()
    try:
        canceled = []
        def fail(): raise ValueError('result setup failed')
        handle = SimpleNamespace(accepted=True, get_result_async=fail,
                                 cancel_goal_async=lambda: canceled.append(True) or Future())
        response = Future()
        response.set_result(handle)
        node.on_goal_response(response)
        assert node.done and canceled and not node.terminal_confirmed
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_commander_cancels_after_arbitrary_exception(monkeypatch):
    calls = []
    args = SimpleNamespace(namespace='', initial_x=0., initial_y=0., initial_yaw=0.,
                           set_initial_pose=False, loops=1, goal_timeout=10., nav2_timeout=10.)

    class Navigator:
        def __init__(self, **kw): pass
        def get_logger(self): return Logger()
        def destroy_node(self): calls.append('destroy')

    class Sender:
        def __init__(self, navigator): pass
        def send(self, pose, deadline): raise ValueError('unexpected')

        def cancel_and_drain(self): calls.append('cancel')
    monkeypatch.setattr(nav_commander_demo, 'parse_cli_args', lambda parser: args)
    monkeypatch.setattr(nav_commander_demo, 'BasicNavigator', Navigator)
    monkeypatch.setattr(nav_commander_demo, 'GoalSender', Sender)
    monkeypatch.setattr(nav_commander_demo, 'wait_for_nav2_active', lambda *args: True)
    monkeypatch.setattr(nav_commander_demo, 'make_pose', lambda *args: None)
    monkeypatch.setattr(rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    assert nav_commander_demo.main() == 1
    assert calls == ['cancel', 'destroy']


def test_failed_completed_result_still_cancels_commander(monkeypatch):
    bad_result = Future()
    bad_result.set_exception(ValueError('result response lost'))
    canceled = []
    ack = Future()
    ack.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle = SimpleNamespace(
        cancel_goal_async=lambda: canceled.append(True) or ack)
    navigator = SimpleNamespace(
        goal_handle=handle, result_future=bad_result, get_logger=lambda: Logger())
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *args, **kw: None)
    assert not nav_commander_demo.cancel_task(navigator)
    assert canceled


def test_failed_completed_result_still_cancels_patrol():
    rclpy.init()
    node = rclpy.create_node('result_failure_test')
    try:
        robot = multi_robot_patrol.RobotPatrolClient(node, 'robot1', [(0, 0, 0)], 5)
        canceled = []
        robot.goal_handle = SimpleNamespace(
            cancel_goal_async=lambda: canceled.append(True) or Future())
        result = Future()
        result.set_exception(ValueError('no result'))
        robot.result_future = result
        robot.on_result(result)
        assert robot.failed and canceled and not robot.goal_terminal
        robot.client.destroy()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
