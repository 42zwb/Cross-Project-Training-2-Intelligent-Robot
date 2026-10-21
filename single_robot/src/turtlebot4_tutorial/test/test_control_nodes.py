import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, LaserScan

from turtlebot4_tutorial.collision_monitor import CollisionMonitor
from turtlebot4_tutorial.common import normalize_namespace
from turtlebot4_tutorial.joystick_control import JoystickControl
from turtlebot4_tutorial.keyboard_control import KeyboardControl
from turtlebot4_tutorial.mapping_patrol import MappingPatrol
from turtlebot4_tutorial.initial_pose_publisher import create_initial_pose_publisher


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_scan(front_distance):
    scan = LaserScan()
    scan.angle_min = -0.5
    scan.angle_increment = 0.5
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [1.0, front_distance, 1.0]
    return scan


def forward_command():
    command = Twist()
    command.linear.x = 0.3
    command.angular.z = 0.2
    return command


class ControlNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_collision_monitor_fails_closed_without_scan(self):
        node = CollisionMonitor()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            node.on_command(forward_command())
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)
            self.assertAlmostEqual(recorder.messages[-1].angular.z, 0.2)
        finally:
            node.destroy_node()

    def test_collision_monitor_blocks_obstacle_and_allows_clear_scan(self):
        node = CollisionMonitor()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            node.on_scan(make_scan(0.2))
            node.on_command(forward_command())
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)

            node.on_scan(make_scan(1.0))
            node.on_command(forward_command())
            self.assertAlmostEqual(recorder.messages[-1].linear.x, 0.3)
        finally:
            node.destroy_node()

    def test_collision_monitor_blocks_stale_scan(self):
        node = CollisionMonitor()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            node.on_scan(make_scan(1.0))
            node.last_scan = time.monotonic() - 2.0
            node.on_command(forward_command())
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)
        finally:
            node.destroy_node()

    def test_collision_monitor_blocks_invalid_scan_and_configuration(self):
        node = CollisionMonitor()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            scan = make_scan(float('nan'))
            scan.ranges = [float('inf'), float('nan'), 0.01]
            node.on_scan(scan)
            node.on_command(forward_command())
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)

            invalid_command = forward_command()
            invalid_command.linear.x = float('nan')
            node.on_command(invalid_command)
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)

            node.set_parameters([
                Parameter('stop_distance', value=0.8),
                Parameter('slow_distance', value=0.4),
            ])
            node.on_scan(make_scan(1.0))
            node.on_command(forward_command())
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)
        finally:
            node.destroy_node()

    def test_joystick_scaling_and_timeout_stop(self):
        node = JoystickControl()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            joy = Joy()
            joy.axes = [0.5, 0.8]
            node.on_joy(joy)
            self.assertAlmostEqual(recorder.messages[-1].linear.x, 0.4)
            self.assertAlmostEqual(recorder.messages[-1].angular.z, 0.5)

            node.last_joy = time.monotonic() - 2.0
            node.stop_if_stale()
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)
            self.assertEqual(recorder.messages[-1].angular.z, 0.0)
        finally:
            node.destroy_node()

    def test_joystick_rejects_invalid_axis_data(self):
        node = JoystickControl()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            joy = Joy()
            joy.axes = [float('nan'), 0.8]
            node.on_joy(joy)
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)
            self.assertEqual(recorder.messages[-1].angular.z, 0.0)

            joy.axes = [0.5]
            node.on_joy(joy)
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)

            joy.axes = [4.0, -3.0]
            node.on_joy(joy)
            self.assertAlmostEqual(recorder.messages[-1].linear.x, -0.5)
            self.assertAlmostEqual(recorder.messages[-1].angular.z, 1.0)
        finally:
            node.destroy_node()

    def test_keyboard_command_times_out_to_stop(self):
        node = KeyboardControl()
        recorder = RecordingPublisher()
        node.publisher = recorder
        try:
            node.publish_key('w')
            self.assertAlmostEqual(recorder.messages[-1].linear.x, 0.2)

            node.last_motion_key = time.monotonic() - 2.0
            node.publish_current()
            self.assertEqual(recorder.messages[-1].linear.x, 0.0)
        finally:
            node.destroy_node()

    def test_mapping_patrol_requires_valid_front_scan(self):
        node = MappingPatrol()
        try:
            scan = make_scan(float('nan'))
            scan.ranges = [float('inf'), float('nan'), 0.01]
            node.on_scan(scan)
            self.assertFalse(node.scan_is_fresh())
            node.on_scan(make_scan(1.0))
            self.assertTrue(node.scan_is_fresh())
        finally:
            node.destroy_node()

    def test_namespace_normalization_and_validation(self):
        self.assertEqual(normalize_namespace('/robot1/'), 'robot1')
        self.assertEqual(normalize_namespace(''), '')
        with self.assertRaises(Exception):
            normalize_namespace('robot with spaces')

    def test_initial_pose_publisher_matches_amcl_reliable_qos(self):
        publisher_node = rclpy.create_node('initial_pose_qos_publisher_test')
        subscriber_node = rclpy.create_node('initial_pose_qos_subscriber_test')
        publisher = create_initial_pose_publisher(publisher_node)
        subscriber = subscriber_node.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', lambda message: None,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        try:
            deadline = time.monotonic() + 3.0
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(subscriber_node, timeout_sec=0.05)
            self.assertEqual(publisher.get_subscription_count(), 1)
        finally:
            subscriber_node.destroy_subscription(subscriber)
            publisher_node.destroy_node()
            subscriber_node.destroy_node()
