"""Tests for planar lidar geometry and fail-closed motion checks."""

import math

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

from turtlebot4_tutorial.motion_safety import (
    beam_is_observed,
    check_motion,
    scan_is_valid,
)


def make_scan(fill=2.0):
    """Return a complete 360-degree scan with a configurable base range."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = 2.0 * math.pi / 720.0
    scan.range_min = 0.10
    scan.range_max = 10.0
    scan.ranges = [fill] * 720
    return scan


def make_transform(yaw=0.0, x=0.0, y=0.0):
    """Return a planar transform for the scan frame."""
    transform = TransformStamped()
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def set_beam(scan, angle, distance):
    """Set the beam nearest a world angle."""
    index = int(round((angle - scan.angle_min) / scan.angle_increment))
    scan.ranges[index % len(scan.ranges)] = distance


def test_beam_observation_accepts_positive_infinity_only():
    scan = make_scan()
    assert beam_is_observed(1.0, scan)
    assert beam_is_observed(float('inf'), scan)
    assert not beam_is_observed(float('nan'), scan)
    assert not beam_is_observed(-float('inf'), scan)
    assert not beam_is_observed(0.01, scan)


def test_scan_requires_enough_valid_beams():
    scan = make_scan()
    assert scan_is_valid(scan)
    scan.ranges = scan.ranges[:3]
    assert not scan_is_valid(scan)


def test_clear_scan_allows_forward_and_rotation():
    check = check_motion(
        make_scan(), make_transform(), 0.2, 0.5)
    assert check.linear_scale == 1.0
    assert check.angular_allowed
    assert check.forward_covered and check.backward_covered
    assert check.full_covered
    assert check.reason == ''


def test_front_obstacle_limits_forward_but_not_rotation():
    scan = make_scan()
    set_beam(scan, 0.0, 0.60)
    check = check_motion(scan, make_transform(), 0.2, 0.5)
    assert 0.0 < check.linear_scale < 1.0
    assert check.angular_allowed


def test_side_obstacle_blocks_rotation_but_not_forward():
    scan = make_scan()
    set_beam(scan, math.pi / 2.0, 0.20)
    check = check_motion(scan, make_transform(), 0.2, 0.5)
    assert check.linear_scale == 1.0
    assert not check.angular_allowed
    assert '旋转' in check.reason


def test_rear_blind_sector_blocks_backward_and_rotation():
    scan = make_scan()
    for index in range(len(scan.ranges)):
        angle = scan.angle_min + index * scan.angle_increment
        if angle < -math.pi / 2.0 or angle > math.pi / 2.0:
            scan.ranges[index] = float('nan')
    forward = check_motion(scan, make_transform(), 0.2, 0.0)
    assert forward.linear_scale == 1.0
    backward = check_motion(scan, make_transform(), -0.2, 0.0)
    assert backward.linear_scale == 0.0
    rotation = check_motion(scan, make_transform(), 0.0, 0.5)
    assert not rotation.angular_allowed
    assert not rotation.backward_covered


def test_scan_transform_moves_side_obstacle_into_front():
    scan = make_scan()
    set_beam(scan, -math.pi / 2.0, 0.20)
    check = check_motion(
        scan, make_transform(yaw=math.pi / 2.0), 0.2, 0.5)
    assert check.linear_scale == 0.0
    assert not check.angular_allowed


def test_tilted_transform_fails_closed():
    transform = make_transform()
    transform.transform.rotation.x = 0.1
    check = check_motion(make_scan(), transform, 0.2, 0.0)
    assert check.linear_scale == 0.0
    assert not check.angular_allowed
    assert '坐标变换' in check.reason


def test_nonfinite_speed_fails_closed():
    check = check_motion(make_scan(), make_transform(), float('nan'), 0.0)
    assert check.linear_scale == 0.0
    assert not check.angular_allowed
    assert '有限' in check.reason
