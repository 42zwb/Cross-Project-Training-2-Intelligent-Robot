import time
from types import SimpleNamespace

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import LaserScan
import yaml

from turtlebot4_tutorial.collision_monitor import CollisionMonitor
from turtlebot4_tutorial.initial_pose_publisher import matches_initial_pose
from turtlebot4_tutorial.mapping_patrol import MappingPatrol
from turtlebot4_tutorial.map_tools import repair_map, validate_map
from turtlebot4_tutorial.multi_robot_patrol import RobotPatrolClient
from turtlebot4_tutorial.nav_commander_demo import GoalSender
from turtlebot4_tutorial.robot_status_publisher import RobotStatusPublisher


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.mark.parametrize('reliability', [
    ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE])
def test_scan_publishers_reach_all_consumers(ros, reliability):
    nodes = [CollisionMonitor(), MappingPatrol(), RobotStatusPublisher()]
    source = rclpy.create_node('scan_qos_source')
    publisher = source.create_publisher(
        LaserScan, 'scan', QoSProfile(depth=10, reliability=reliability))
    executor = SingleThreadedExecutor()
    for node in nodes + [source]:
        executor.add_node(node)
    scan = LaserScan()
    scan.angle_min = -0.5
    scan.angle_increment = 0.5
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [1.0, 1.0, 1.0]
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and any(n.last_scan is None for n in nodes):
            publisher.publish(scan)
            executor.spin_once(timeout_sec=0.02)
        assert all(n.last_scan is not None for n in nodes)
    finally:
        executor.shutdown()
        for node in nodes + [source]:
            node.destroy_node()


def test_status_reports_invalid_scan_but_treats_positive_inf_as_no_return(ros):
    node = RobotStatusPublisher()
    messages = []
    node.publisher = SimpleNamespace(publish=messages.append)
    node.on_odom(Odometry())
    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 12.0
    try:
        scan.ranges = [float('nan'), -float('inf')]
        node.on_scan(scan)
        node.publish_status()
        assert messages[-1].status[0].level == DiagnosticStatus.WARN
        assert '无有效数据' in messages[-1].status[0].message
        scan.ranges = [float('inf'), float('inf')]
        node.on_scan(scan)
        node.publish_status()
        assert messages[-1].status[0].level == DiagnosticStatus.OK
        scan.ranges = [float('inf'), float('nan')]
        node.on_scan(scan)
        node.publish_status()
        assert messages[-1].status[0].level == DiagnosticStatus.WARN
    finally:
        node.destroy_node()


def test_mixed_gray_map_requires_manual_resolution(tmp_path):
    Image.frombytes('L', (2, 1), bytes([205, 127])).save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': 0.05,
        'origin': [0, 0, 0], 'mode': 'trinary', 'free_thresh': 0.25,
        'occupied_thresh': 0.65, 'negate': 0}))
    with pytest.raises(ValueError, match='未知'):
        validate_map(path)
    with pytest.raises(ValueError, match='混合灰度'):
        repair_map(path)
    assert yaml.safe_load(path.read_text())['free_thresh'] == 0.25


def test_amcl_pose_requires_fresh_finite_matching_pose():
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 10
    pose.pose.pose.position.x = 2.0
    pose.pose.pose.position.y = 3.0
    pose.pose.pose.orientation.w = 1.0
    assert matches_initial_pose(pose, 2, 3, 0, 9_000_000_000, 10_000_000_000)
    pose.pose.pose.position.x = -99.0
    assert not matches_initial_pose(pose, 2, 3, 0, 9_000_000_000, 10_000_000_000)
    pose.pose.pose.position.x = 2.0
    assert not matches_initial_pose(pose, 2, 3, 0, 11_000_000_000, 11_000_000_000)
    assert not matches_initial_pose(pose, 2, 3, 0, 9_000_000_000, 13_000_000_000)
    pose.pose.pose.orientation.w = float('nan')
    assert not matches_initial_pose(pose, 2, 3, 0, 9_000_000_000, 10_000_000_000)


def test_late_goal_acceptance_is_canceled(ros):
    class Handle:
        accepted = True

        def __init__(self):
            self.result = Future()
            self.canceled = False

        def get_result_async(self):
            return self.result

        def cancel_goal_async(self):
            self.canceled = True
            return Future()

    node = rclpy.create_node('late_goal_test')
    try:
        handle = Handle()
        robot = RobotPatrolClient(node, 'robot1', [(0, 0, 0)], 1)
        robot.cancel()
        response = Future()
        response.set_result(handle)
        robot.on_goal_response(response)
        assert handle.canceled
        robot.client.destroy()

        navigator = SimpleNamespace(goal_handle=None, result_future=None,
                                    get_logger=node.get_logger)
        sender = GoalSender(navigator)
        sender.cancel_requested = True
        handle2 = Handle()
        response2 = Future()
        response2.set_result(handle2)
        sender.on_response(response2)
        assert handle2.canceled
    finally:
        node.destroy_node()
