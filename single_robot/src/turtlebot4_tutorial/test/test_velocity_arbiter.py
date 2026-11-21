"""Tests for command arbitration, sensor gating and latched emergency stop."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from turtlebot4_tutorial.motion_safety import scan_is_valid
from turtlebot4_tutorial.velocity_arbiter import VelocityArbiter


class RecordingPublisher:
    """Record published messages for direct node tests."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        """Append one message."""
        self.messages.append(message)


@pytest.fixture
def ros():
    """Initialize a ROS context for one test."""
    rclpy.init()
    yield
    rclpy.try_shutdown()


def make_scan(node):
    """Return a valid 360-degree scan stamped with the node clock."""
    scan = LaserScan()
    scan.header.stamp = node.get_clock().now().to_msg()
    scan.angle_min = -3.141592653589793
    scan.angle_increment = 0.01
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [2.0] * 629
    assert scan_is_valid(scan)
    return scan


def make_odom(node):
    """Return finite odometry stamped with the node clock."""
    odom = Odometry()
    odom.header.stamp = node.get_clock().now().to_msg()
    odom.pose.pose.position.x = 1.0
    odom.twist.twist.linear.x = 0.1
    return odom


def make_twist(linear, angular=0.0):
    """Return a simple planar velocity command."""
    command = Twist()
    command.linear.x = linear
    command.angular.z = angular
    return command


def make_node():
    """Create an arbiter without TF requirements for unit testing."""
    node = VelocityArbiter()
    node.set_parameters([
        Parameter('require_odom_tf', value=False),
        Parameter('require_nav_localization', value=False),
    ])
    recorder = RecordingPublisher()
    node.publisher = recorder
    node.on_scan(make_scan(node))
    node.on_odom(make_odom(node))
    return node, recorder


def test_teleop_has_priority_over_aux_and_nav(ros):
    node, recorder = make_node()
    try:
        node.on_command('nav', make_twist(0.2))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.2
        node.on_command('aux', make_twist(0.1))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.1
        node.on_command('teleop', make_twist(0.05))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.05
    finally:
        node.destroy_node()


def test_stale_command_publishes_zero(ros):
    node, recorder = make_node()
    try:
        node.on_command('nav', make_twist(0.2))
        node.commands['nav'] = (time.monotonic() - 2.0, make_twist(0.2))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0
    finally:
        node.destroy_node()


def test_stale_scan_publishes_zero(ros):
    node, recorder = make_node()
    try:
        node.on_command('teleop', make_twist(0.2))
        node.scan_received = time.monotonic() - 2.0
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0
    finally:
        node.destroy_node()


def test_invalid_scan_publishes_zero(ros):
    node, recorder = make_node()
    try:
        node.on_command('teleop', make_twist(0.2))
        node.scan_valid = False
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0
    finally:
        node.destroy_node()


def test_nonfinite_command_is_ignored(ros):
    node, recorder = make_node()
    try:
        node.on_command('teleop', make_twist(0.2))
        node.on_command('teleop', make_twist(float('nan')))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0
    finally:
        node.destroy_node()


def test_estop_latches_and_requires_new_command(ros):
    node, recorder = make_node()
    request = SetBool.Request()
    response = SetBool.Response()
    try:
        node.on_command('teleop', make_twist(0.2))
        request.data = True
        assert node.on_set_estop(request, response).success
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0

        request.data = False
        assert node.on_set_estop(request, response).success
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0

        node.on_command('teleop', make_twist(0.1))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.1
    finally:
        node.destroy_node()


def test_nav_requires_localization_when_enabled(ros):
    node, recorder = make_node()
    try:
        node.set_parameters([
            Parameter('require_nav_localization', value=True)])
        node._transform_age = lambda parent, child: None
        node.on_command('nav', make_twist(0.2))
        node.tick()
        assert recorder.messages[-1].linear.x == 0.0
    finally:
        node.destroy_node()
