"""Pure 2-D lidar safety checks shared by the safety nodes."""

import math
from typing import NamedTuple, Optional

import numpy as np


SECTOR_COUNT = 72


class MotionCheck(NamedTuple):
    """Result of one planar collision check."""

    linear_scale: float
    angular_allowed: bool
    nearest: Optional[float]
    forward_covered: bool
    backward_covered: bool
    full_covered: bool
    reason: str


def finite_twist(twist):
    """Return True when every velocity component is a finite number."""
    values = (
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def stamp_is_fresh(stamp, now_ns, max_age=0.8, future_tolerance=0.5):
    """Compare a message stamp with the ROS clock, including simulated time."""
    nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
    age = (now_ns - nanoseconds) * 1e-9
    return (
        nanoseconds > 0
        and 0 <= stamp.nanosec < 1_000_000_000
        and -future_tolerance <= age <= max_age
    )


def beam_is_observed(value, scan):
    """Return True for a measured or positive-infinity no-return beam."""
    if math.isinf(value) and value > 0:
        return True
    return (
        math.isfinite(value)
        and scan.range_min <= value <= scan.range_max
    )


def scan_is_valid(scan):
    """Return True when scan metadata and enough beams are usable."""
    metadata = [
        scan.range_min, scan.range_max,
        scan.angle_min, scan.angle_increment,
    ]
    if not all(math.isfinite(value) for value in metadata):
        return False
    if not 0.0 <= scan.range_min < scan.range_max:
        return False
    if not 0.0 < abs(scan.angle_increment) <= 2.0 * math.pi:
        return False
    if not 20 <= len(scan.ranges) <= 100_000:
        return False
    return sum(beam_is_observed(value, scan) for value in scan.ranges) >= 20


def _sector_angle(index):
    return -math.pi + (index + 0.5) * 2.0 * math.pi / SECTOR_COUNT


def _sector_index(angle):
    normalized = math.atan2(math.sin(angle), math.cos(angle))
    index = int((normalized + math.pi) / (2.0 * math.pi) * SECTOR_COUNT)
    return min(SECTOR_COUNT - 1, max(0, index))


def _is_front_sector(index):
    angle = _sector_angle(index)
    return -math.pi / 2.0 <= angle <= math.pi / 2.0


def scan_geometry(scan, transform):
    """
    Return base-frame obstacle points and sector coverage.

    Positive infinity is a valid clear observation.  NaN, negative infinity,
    out-of-range values and distances behind an obstacle may be invalid.  The
    transform must be planar; a tilted or non-finite transform is rejected
    instead of being projected approximately.
    """
    if not scan_is_valid(scan):
        return None
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = (
        translation.x, translation.y, translation.z,
        rotation.x, rotation.y, rotation.z, rotation.w,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(
        rotation.x * rotation.x + rotation.y * rotation.y
        + rotation.z * rotation.z + rotation.w * rotation.w
    )
    if norm < 1e-6:
        return None
    qx, qy, qz, qw = (
        rotation.x / norm, rotation.y / norm,
        rotation.z / norm, rotation.w / norm,
    )
    if abs(qx) > 1e-6 or abs(qy) > 1e-6:
        return None
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))

    points = []
    sectors = [False] * SECTOR_COUNT
    front_total = front_valid = 0
    rear_total = rear_valid = 0
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        base_angle = yaw + angle
        seen = beam_is_observed(value, scan)
        if -math.pi / 2.0 <= math.atan2(
                math.sin(base_angle), math.cos(base_angle)) <= math.pi / 2.0:
            front_total += 1
            front_valid += int(seen)
        else:
            rear_total += 1
            rear_valid += int(seen)
        if not seen:
            continue
        sectors[_sector_index(base_angle)] = True
        if math.isfinite(value):
            points.append((
                translation.x + value * math.cos(base_angle),
                translation.y + value * math.sin(base_angle),
            ))
    return (
        np.asarray(points, dtype=float).reshape(-1, 2),
        sectors, front_total, front_valid, rear_total, rear_valid,
    )


def _coverage(sectors, front):
    indices = [
        index for index in range(SECTOR_COUNT)
        if _is_front_sector(index) == front
    ]
    total = len(indices)
    return total > 0 and all(sectors[index] for index in indices)


def check_motion(
    scan,
    transform,
    linear_x,
    angular_z,
    robot_radius=0.175,
    stop_distance=0.35,
    slow_distance=0.70,
    reaction_time=0.60,
    lateral_margin=0.02,
    rotation_margin=0.10,
):
    """
    Check a planar command against a transformed laser scan.

    Forward and backward motion are checked inside a swept strip.  Any
    in-place rotation requires complete 360-degree coverage and clearance
    around the circular footprint.  Invalid parameters or sensor geometry
    fail closed.
    """
    parameters = (
        robot_radius, stop_distance, slow_distance,
        reaction_time, lateral_margin, rotation_margin,
    )
    if not all(math.isfinite(value) for value in parameters):
        return MotionCheck(0.0, False, None, False, False, False,
                           '安全参数不是有限数值')
    if (robot_radius <= 0.0 or stop_distance < 0.0
            or slow_distance <= stop_distance
            or reaction_time < 0.0 or lateral_margin < 0.0
            or rotation_margin < 0.0):
        return MotionCheck(0.0, False, None, False, False, False,
                           '安全参数超出有效范围')
    if not math.isfinite(linear_x) or not math.isfinite(angular_z):
        return MotionCheck(0.0, False, None, False, False, False,
                           '速度指令不是有限数值')

    geometry = scan_geometry(scan, transform)
    if geometry is None:
        return MotionCheck(0.0, False, None, False, False, False,
                           '雷达数据或坐标变换无效')
    (points, sectors, front_total, front_valid,
     rear_total, rear_valid) = geometry
    forward_covered = (
        front_total > 0
        and front_valid / front_total >= 0.9
        and _coverage(sectors, True)
    )
    backward_covered = (
        rear_total > 0
        and rear_valid / rear_total >= 0.9
        and _coverage(sectors, False)
    )
    full_covered = _coverage(sectors, True) and _coverage(sectors, False)
    nearest = None
    if len(points):
        distances = np.hypot(points[:, 0], points[:, 1])
        nearest = float(np.min(distances))

    reasons = []
    linear_scale = 1.0
    if abs(linear_x) > 1e-6:
        direction = 1.0 if linear_x > 0.0 else -1.0
        covered = forward_covered if direction > 0.0 else backward_covered
        if not covered:
            linear_scale = 0.0
            reasons.append('运动方向雷达覆盖不足')
        else:
            horizon = max(
                slow_distance,
                abs(linear_x) * reaction_time + stop_distance,
            )
            along = points[:, 0] * direction
            lateral = np.abs(points[:, 1])
            mask = (
                (along >= 0.0) & (along <= horizon)
                & (lateral <= robot_radius + lateral_margin)
            )
            if np.any(mask):
                nearest_along = float(np.min(along[mask]))
                if nearest_along <= stop_distance:
                    linear_scale = 0.0
                    reasons.append('运动方向障碍物过近')
                elif nearest_along < slow_distance:
                    linear_scale = max(
                        0.0,
                        min(1.0, (nearest_along - stop_distance)
                            / (slow_distance - stop_distance)),
                    )
                    reasons.append('运动方向障碍物较近，已限速')

    angular_allowed = True
    if abs(angular_z) > 1e-6:
        if not full_covered:
            angular_allowed = False
            reasons.append('旋转所需雷达覆盖不足')
        else:
            clearance = robot_radius + rotation_margin
            if nearest is not None and nearest <= clearance:
                angular_allowed = False
                reasons.append('旋转范围内障碍物过近')

    return MotionCheck(
        linear_scale,
        angular_allowed,
        nearest,
        forward_covered,
        backward_covered,
        full_covered,
        '；'.join(reasons),
    )
