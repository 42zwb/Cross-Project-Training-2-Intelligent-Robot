"""The only publisher allowed to drive a controller; fail closed on stale input.
A native controller timeout adds an independent second layer if this node dies.
"""

import json, math, time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class Safety(Node):
    def __init__(self):
        super().__init__("safety")
        self.declare_parameter("config", "")
        self.cfg = yaml.safe_load(Path(self.get_parameter("config").value).read_text())
        self.max_speed = float(self.cfg["max_speed"])
        self.robot_radius = float(self.cfg["robot_radius"])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.state = {}
        self.lease = 0.0
        self.commands = {}
        self.scans = {}
        self.odoms = {}
        self.pubs = {}
        self.subs = []
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/fleet/drive_lease", self.lease_cb, 1)
        self.status = self.create_publisher(String, "/fleet/safety_state", 1)
        for r in self.cfg["robots"]:
            n = r["name"]
            self.pubs[n] = self.create_publisher(
                Twist, f"/{n}/diffdrive_controller/cmd_vel_unstamped", 1
            )
            self.subs.append(
                self.create_subscription(
                    Twist,
                    f"/{n}/cmd_vel_nav",
                    lambda m, n=n: self.commands.update({n: (time.monotonic(), m)}),
                    1,
                )
            )
            self.subs.append(
                self.create_subscription(
                    LaserScan,
                    f"/{n}/scan",
                    lambda m, n=n: self.scans.update({n: (time.monotonic(), m)}),
                    qos_profile_sensor_data,
                )
            )
            self.subs.append(
                self.create_subscription(
                    Odometry,
                    f"/{n}/odom",
                    lambda m, n=n: self.odoms.update({n: (time.monotonic(), m)}),
                    qos_profile_sensor_data,
                )
            )
        self.create_timer(0.05, self.tick)

    def lease_cb(self, msg):
        try:
            state = json.loads(msg.data)
            regions = state.get("regions") if isinstance(state, dict) else None
            valid_regions = (
                isinstance(regions, list)
                and len(regions) == len(self.cfg["robots"])
                and all(
                    isinstance(bounds, list)
                    and len(bounds) == 4
                    and all(
                        isinstance(value, (int, float))
                        and not isinstance(value, bool)
                        and math.isfinite(value)
                        for value in bounds
                    )
                    and bounds[0] < bounds[2]
                    and bounds[1] < bounds[3]
                    for bounds in regions
                )
            )
            if (
                not isinstance(state, dict)
                or not isinstance(state.get("active"), list)
                or not isinstance(state.get("estop"), bool)
                or not isinstance(state.get("region_enabled"), bool)
                or not valid_regions
            ):
                raise ValueError("invalid drive lease")
            self.state = state
            self.lease = time.monotonic()
        except (ValueError, TypeError, OverflowError):
            self.state = {}
            self.lease = 0.0

    def tick(self):
        now = time.monotonic()
        report = {}
        for i, r in enumerate(self.cfg["robots"]):
            n = r["name"]
            reason = ""
            out = Twist()
            active = n in self.state.get("active", [])
            if now - self.lease > 0.8:
                reason = "调度心跳中断"
            elif self.state.get("estop", True):
                reason = "急停锁定"
            elif not active:
                reason = "待命"
            elif n not in self.scans or now - self.scans[n][0] > 0.8:
                reason = "雷达超时"
            elif (
                sum(
                    math.isfinite(v) and v >= self.scans[n][1].range_min
                    for v in self.scans[n][1].ranges
                )
                < 20
            ):
                reason = "雷达数据无效"
            elif n not in self.odoms or now - self.odoms[n][0] > 0.8:
                reason = "里程计超时"
            else:
                try:
                    t = self.buffer.lookup_transform(
                        "map", n + "/base_link", rclpy.time.Time()
                    )
                    age = (
                        self.get_clock().now()
                        - rclpy.time.Time.from_msg(t.header.stamp)
                    ).nanoseconds * 1e-9
                    if age > 1.0 or age < -0.5:
                        reason = "定位超时"
                except TransformException:
                    reason = "定位丢失"
            if not reason:
                if n not in self.commands or now - self.commands[n][0] > 0.5:
                    reason = "等待新速度"
                else:
                    cmd = self.commands[n][1]
                    if not math.isfinite(cmd.linear.x) or not math.isfinite(
                        cmd.angular.z
                    ):
                        reason = "非法速度"
                    else:
                        out.linear.x = max(
                            -self.max_speed, min(self.max_speed, cmd.linear.x)
                        )
                        out.angular.z = max(-0.6, min(0.6, cmd.angular.z))
                        scan = self.scans[n][1]
                        # Sensor is close to the body centre; never rely on wheel odometry alone.
                        direction = 0.0 if out.linear.x >= 0 else math.pi
                        ranges = [
                            v
                            for j, v in enumerate(scan.ranges)
                            if math.isfinite(v)
                            and abs(
                                math.atan2(
                                    math.sin(
                                        scan.angle_min
                                        + j * scan.angle_increment
                                        - direction
                                    ),
                                    math.cos(
                                        scan.angle_min
                                        + j * scan.angle_increment
                                        - direction
                                    ),
                                )
                            )
                            < 0.45
                        ]
                        if (
                            ranges
                            and min(ranges) < self.robot_radius + 0.06
                            and abs(out.linear.x) > 0.001
                        ):
                            out.linear.x = 0.0
                            reason = "近障停车"
                        if self.state.get("region_enabled", False):
                            bounds = self.state["regions"][i]
                            p = t.transform.translation
                            q = t.transform.rotation
                            heading = math.atan2(
                                2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z),
                            )
                            px = p.x + math.cos(heading) * out.linear.x * 0.6
                            py = p.y + math.sin(heading) * out.linear.x * 0.6
                            if (
                                not bounds[0] + self.robot_radius + 0.02
                                <= px
                                <= bounds[2] - self.robot_radius - 0.02
                                or not bounds[1] + self.robot_radius + 0.02
                                <= py
                                <= bounds[3] - self.robot_radius - 0.02
                            ):
                                out = Twist()
                                reason = "分区边界保护"
            self.pubs[n].publish(out)
            report[n] = {
                "reason": reason,
                "moving_allowed": active and not bool(reason),
                "v": out.linear.x,
                "w": out.angular.z,
            }
        self.status.publish(String(data=json.dumps(report)))


def main():
    rclpy.init()
    n = Safety()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        for p in n.pubs.values():
            p.publish(Twist())
        n.destroy_node()
        rclpy.try_shutdown()
