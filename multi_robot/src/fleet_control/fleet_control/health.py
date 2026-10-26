"""Small sensor validation helpers shared by runtime safety and regression checks."""

import math


def scan_health(scan):
    metadata = [scan.range_min, scan.range_max, scan.angle_min, scan.angle_increment]
    if (not all(math.isfinite(v) for v in metadata)
            or not 0 <= scan.range_min < scan.range_max
            or not 0 < abs(scan.angle_increment) <= 2 * math.pi
            or not 20 <= len(scan.ranges) <= 100_000):
        return False, None
    values = [v for v in scan.ranges
              if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
    return len(values) >= 20, min(values) if values else None


def valid_odom(message):
    p, v = message.pose.pose.position, message.twist.twist
    return all(math.isfinite(x) for x in [p.x, p.y, v.linear.x, v.angular.z])
