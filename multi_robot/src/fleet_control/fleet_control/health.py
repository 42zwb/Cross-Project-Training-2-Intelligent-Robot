"""Sensor validation and conservative planar lidar motion checks."""

import math


def observed_range(value, scan):
    # Gazebo/RPLidar convention: +inf means no return within range_max.
    return (math.isinf(value) and value > 0) or (
        math.isfinite(value) and scan.range_min <= value <= scan.range_max)


def scan_health(scan):
    metadata = [scan.range_min, scan.range_max, scan.angle_min, scan.angle_increment]
    if (not all(math.isfinite(v) for v in metadata)
            or not 0 <= scan.range_min < scan.range_max
            or not 0 < abs(scan.angle_increment) <= 2 * math.pi
            or not 20 <= len(scan.ranges) <= 100_000):
        return False, None
    values = [v for v in scan.ranges
              if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
    observed = sum(observed_range(v, scan) for v in scan.ranges)
    return observed >= 20, min(values) if values else None


def valid_odom(message):
    p, v = message.pose.pose.position, message.twist.twist
    return all(math.isfinite(x) for x in [p.x, p.y, v.linear.x, v.angular.z])


def stamp_fresh(stamp, now_ns, max_age=0.8, future_tolerance=0.5):
    """Compare message and ROS clock, not wall clock (also works with sim time)."""
    ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age = (now_ns - ns) * 1e-9
    return ns > 0 and 0 <= stamp.nanosec < 1_000_000_000 and -future_tolerance <= age <= max_age


def lidar_motion_check(scan, transform, speed, radius, angular_speed=0.0, check_backward=False):
    """Check a circular footprint swept along a straight stopping horizon.

    Both turning and reversing use the same enclosing straight capsule: with a
    circular footprint this also covers the robot's rotation in place. A scan
    must observe every five-degree sector of the motion-facing half-plane and
    at least 90% of its beams there. Positive infinity is an observed clear ray.
    """
    p, q = transform.translation, transform.rotation
    components = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in components):
        return False, False
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-6:
        return False, False
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    # Rotate the lidar XY axes into base coordinates.
    xx, xy = 1-2*(y*y+z*z), 2*(x*y-w*z)
    yx, yy = 2*(x*y+w*z), 1-2*(x*x+z*z)
    # Reject near vertical scan planes: projection alone cannot certify coverage.
    if abs(1 - (xx*yy-xy*yx)) > 0.05:
        return False, False
    direction = -1.0 if speed < 0 or check_backward else 1.0
    bins = [False] * 36
    total = valid = 0
    blocked = False
    horizon = abs(speed) * 0.6 + 0.06 if abs(speed) > 0.001 else 0.0
    # Bound lateral drift of a turning arc over the 0.6 s horizon.
    footprint = radius + 0.02 + horizon * min(1.0, abs(angular_speed) * 0.3)
    for j, value in enumerate(scan.ranges):
        angle = scan.angle_min + j * scan.angle_increment
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = xx*c + xy*s, yx*c + yy*s
        relative = math.atan2(direction*dy, direction*dx)
        seen = observed_range(value, scan)
        if -math.pi/2 <= relative <= math.pi/2:
            total += 1
            if seen:
                valid += 1
                index = min(35, int((relative + math.pi/2) * 36 / math.pi))
                bins[index] = True
        if seen and math.isfinite(value):
            px, py = p.x + value*dx, p.y + value*dy
            along = max(0.0, min(horizon, direction*px))
            if math.hypot(px-direction*along, py) < footprint:
                blocked = True
    coverage = total >= 36 and valid / total >= 0.9 and all(bins)
    return coverage, blocked


def peer_motion_clear(position, heading, speed, angular_speed, peers, radius, max_speed, own_pose_age=0.0):
    """Conservative planar clearance against other circular robots.

    The peer can move by max_speed for 0.6 s, even if its last odometry says
    it is stopped. The extra 10 cm covers some localization and sampling error;
    it is a configured engineering margin, not a certified error bound.
    """
    horizon = abs(speed) * 0.6 + 0.06 if abs(speed) > 0.001 else 0.0
    angular_margin = horizon * min(1.0, abs(angular_speed) * 0.3)
    base_margin = 2 * radius + 0.10 + angular_margin
    direction = 1.0 if speed >= 0 else -1.0
    dx, dy = direction * math.cos(heading), direction * math.sin(heading)
    for peer in peers:
        px, py = peer[:2]
        peer_age = peer[2] if len(peer) > 2 else 0.0
        # Both poses may be old by up to the accepted TF freshness window.
        minimum = base_margin + max_speed * (0.6 + max(0.0, peer_age) + max(0.0, own_pose_age))
        rx, ry = px - position[0], py - position[1]
        along = max(0.0, min(horizon, rx * dx + ry * dy))
        if math.hypot(rx - along * dx, ry - along * dy) <= minimum:
            return False
    return True
