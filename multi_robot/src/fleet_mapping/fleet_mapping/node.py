"""SLAM scan masking, stationary startup calibration and shared-map fusion."""

import copy
import json
import math
import time
import uuid
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import (
    Buffer,
    TransformListener,
    StaticTransformBroadcaster,
    TransformException,
)
from cartographer_ros_msgs.srv import WriteState
from fleet_mapping.grid import fuse, save_map, mask_robot_returns, valid_grid_message


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Mapping(Node):
    def __init__(self):
        super().__init__("mapping")
        self.declare_parameter("config", "")
        self.declare_parameter("run_dir", "/tmp/fleet")
        self.cfg = yaml.safe_load(Path(self.get_parameter("config").value).read_text())
        self.robots = self.cfg["robots"]
        self.run = Path(self.get_parameter("run_dir").value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.static = StaticTransformBroadcaster(self)
        self.static_sent = 0
        self.align = {}
        self.maps = {}
        self.last_mask_drop = {}
        self.subs = []
        self.scanpub = {}
        self.received = {}
        self.save_pending = {}
        self.qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Save requests and acknowledgements are commands/events, not state.
        # Volatile durability prevents a restarted mapper from replaying an old save.
        self.save_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, "/map", self.qos)
        self.status = self.create_publisher(String, "/fleet/mapping_state", self.qos)
        self.save_result_pub = self.create_publisher(
            String, "/fleet/save_result", self.save_qos
        )
        for r in self.robots:
            n = r["name"]
            self.scanpub[n] = self.create_publisher(
                LaserScan, f"/{n}/scan_slam", qos_profile_sensor_data
            )
            self.subs.append(
                self.create_subscription(
                    LaserScan,
                    f"/{n}/scan",
                    lambda m, n=n: self.scan(n, m),
                    qos_profile_sensor_data,
                )
            )
            self.subs.append(
                self.create_subscription(
                    OccupancyGrid,
                    f"/{n}/local_map",
                    lambda m, n=n: self.local_map(n, m),
                    self.qos,
                )
            )
        self.create_subscription(
            String, "/fleet/save_request", self.save, self.save_qos
        )
        self.create_timer(0.5, self.tick)

    def local_map(self, name, message):
        if valid_grid_message(message):
            self.maps[name] = message
        else:
            self.maps.pop(name, None)

    def lookup(self, parent, child, stamp=None, require_fresh=False):
        t = self.buffer.lookup_transform(parent, child, stamp or rclpy.time.Time())
        if require_fresh:
            age = (
                self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)
            ).nanoseconds * 1e-9
            if age > 0.5 or age < -0.2:
                raise TransformException(
                    f"stale transform {parent} -> {child}: {age:.3f}s"
                )
        p = t.transform.translation
        pose = (p.x, p.y, yaw(t.transform.rotation))
        if not all(math.isfinite(v) for v in pose):
            raise TransformException("nonfinite pose")
        return pose

    def global_pose(self, name, child, stamp=None, require_fresh=False):
        try:
            return self.lookup("map", child, stamp, require_fresh)
        except TransformException:
            pass
        robot = next(r for r in self.robots if r["name"] == name)
        try:
            local = self.lookup(name + "/map", child, stamp, require_fresh)
            base = self.lookup(
                name + "/map", name + "/base_link", stamp, require_fresh
            )
            alignment = self.align.get(name)
            if alignment is None:
                alignment = compose(robot["start"], inv(base))
            return compose(alignment, local)
        except TransformException:
            if name in self.align:
                raise
            # Before Cartographer publishes its first map transform, odometry
            # is still available and is relative to the configured spawn pose.
            odom_pose = self.lookup(name + "/odom", child, stamp, require_fresh)
            return compose(robot["start"], odom_pose)

    def scan(self, n, msg):
        self.received[n] = time.monotonic()
        m = copy.deepcopy(msg)
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            a = self.global_pose(n, msg.header.frame_id, stamp, require_fresh=True)
            positions = [
                self.global_pose(
                    r["name"],
                    r["name"] + "/base_link",
                    stamp,
                    require_fresh=True,
                )
                for r in self.robots
                if r["name"] != n
            ]
        except TransformException:
            try:
                # A scan may precede its matching transform by a few milliseconds.
                # Retry the complete mask at latest time; never forward it unmasked.
                a = self.global_pose(n, msg.header.frame_id, require_fresh=True)
                positions = [
                    self.global_pose(
                        r["name"],
                        r["name"] + "/base_link",
                        require_fresh=True,
                    )
                    for r in self.robots
                    if r["name"] != n
                ]
            except TransformException:
                m.ranges = [float("nan")] * len(m.ranges)
                now = time.monotonic()
                if now - self.last_mask_drop.get(n, 0.0) > 5.0:
                    self.get_logger().warning(
                        f"{n} 回波过滤所需位姿不可用，暂时丢弃该帧，避免未过滤回波进入 SLAM"
                    )
                    self.last_mask_drop[n] = now
                self.scanpub[n].publish(m)
                return
        ranges = mask_robot_returns(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            a,
            positions,
            radius=self.cfg["robot_radius"] + 0.17,
        )
        m.ranges = ranges.tolist()
        self.scanpub[n].publish(m)

    def expire_saves(self):
        # A dead WriteState server must not retain clients and block future saves.
        for request_id, pending in list(self.save_pending.items()):
            if time.monotonic() > pending["deadline"]:
                pending["errors"].append("PBStream 保存服务响应超时")
                self.finish_save(request_id)
                for client, future in pending["clients"]:
                    if not future.done():
                        future.cancel()
                    self.destroy_client(client)

    def tick(self):
        self.expire_saves()
        transforms = []
        for r in self.robots:
            n = r["name"]
            if n not in self.align:
                try:
                    local = self.lookup(n + "/map", n + "/base_link")
                except TransformException:
                    continue
                self.align[n] = compose(r["start"], inv(local))
                self.get_logger().info(f"{n} 静止标定完成：{self.align[n]}")
            a = self.align[n]
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = n + "/map"
            t.transform.translation.x = a[0]
            t.transform.translation.y = a[1]
            t.transform.rotation.z = math.sin(a[2] / 2)
            t.transform.rotation.w = math.cos(a[2] / 2)
            transforms.append(t)
        if len(transforms) > self.static_sent:
            self.static.sendTransform(transforms)
            self.static_sent = len(transforms)
        layers = []
        for n, msg in self.maps.items():
            if n not in self.align:
                continue
            p = msg.info.origin
            layers.append(
                (
                    np.asarray(msg.data, dtype=np.int16).reshape(
                        msg.info.height, msg.info.width
                    ),
                    msg.info.resolution,
                    p.position.x,
                    p.position.y,
                    yaw(p.orientation),
                    self.align[n],
                )
            )
        shape = (
            round(
                (self.cfg["arena"][3] - self.cfg["arena"][1]) / self.cfg["resolution"]
            ),
            round(
                (self.cfg["arena"][2] - self.cfg["arena"][0]) / self.cfg["resolution"]
            ),
        )
        self.data = fuse(layers, shape, self.cfg["resolution"], self.cfg["arena"][:2])
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.info.resolution = self.cfg["resolution"]
        m.info.height, m.info.width = shape
        m.info.origin.position.x = self.cfg["arena"][0]
        m.info.origin.position.y = self.cfg["arena"][1]
        m.info.origin.orientation.w = 1.0
        m.data = self.data.flatten().tolist()
        self.pub.publish(m)
        self.status.publish(
            String(
                data=json.dumps(
                    {
                        "aligned": list(self.align),
                        "local_maps": list(self.maps),
                        "known_fraction": float(np.mean(self.data >= 0)),
                        "scan_age": {
                            n: time.monotonic() - t for n, t in self.received.items()
                        },
                    }
                )
            )
        )

    def save(self, msg):
        try:
            request = json.loads(msg.data)
        except ValueError:
            request = {"destination": msg.data, "request_id": uuid.uuid4().hex}
        if not isinstance(request, dict):
            return
        request_id = str(request.get("request_id") or uuid.uuid4().hex)
        try:
            destination = request.get("destination")
            if not isinstance(destination, str) or not destination.strip():
                raise ValueError("保存目录必须是非空路径")
            dest = Path(destination).expanduser().resolve()
            if not hasattr(self, "data"):
                raise RuntimeError("共享地图尚未生成")
            if len(self.save_pending) >= 4:
                raise RuntimeError("地图保存队列已满，请等待现有任务完成")
            if any(
                pending["destination"] == str(dest)
                for pending in self.save_pending.values()
            ):
                raise RuntimeError("该目录已有地图保存任务正在执行")
            dest.mkdir(parents=True, exist_ok=True)
            save_map(
                dest / "shared_map",
                self.data,
                self.cfg["resolution"],
                self.cfg["arena"][:2],
            )
            (dest / "alignment.yaml").write_text(
                yaml.safe_dump({n: list(t) for n, t in self.align.items()})
            )
            (dest / "fleet.yaml").write_text(yaml.safe_dump(self.cfg))
        except (KeyError, OSError, RuntimeError, ValueError, TypeError) as e:
            self.publish_save_result(request_id, request.get("destination", ""), False, str(e))
            self.get_logger().error(f"地图文件保存失败：{e}")
            return

        self.save_pending[request_id] = {
            "destination": str(dest),
            "remaining": 0,
            "errors": [],
            "started": False,
            "deadline": time.monotonic() + 12.0,
            "clients": [],
        }
        for r in self.robots:
            n = r["name"]
            client = self.create_client(WriteState, f"/{n}/write_state")
            req = WriteState.Request()
            req.filename = str(dest / f"{n}.pbstream")
            req.include_unfinished_submaps = True
            if client.service_is_ready():
                self.save_pending[request_id]["remaining"] += 1
                future = client.call_async(req)
                self.save_pending[request_id]["clients"].append((client, future))
                future.add_done_callback(
                    lambda f, n=n, rid=request_id, c=client: self.save_done(
                        rid, n, c, f
                    )
                )
            else:
                self.get_logger().error(f"{n} WriteState 服务不可用，PBStream 未保存")
                self.save_pending[request_id]["errors"].append(
                    f"{n} WriteState 服务不可用"
                )
                self.destroy_client(client)
        pending = self.save_pending[request_id]
        pending["started"] = True
        if pending["remaining"] == 0:
            self.finish_save(request_id)

    def save_done(self, request_id, name, client, future):
        pending = self.save_pending.get(request_id)
        if pending is None:
            return
        pending["clients"] = [(c, f) for c, f in pending["clients"] if c is not client]
        self.destroy_client(client)
        try:
            response = future.result()
            if response.status.code != 0:
                pending["errors"].append(
                    f"{name} PBStream 保存失败：{response.status.message}"
                )
        except Exception as e:
            pending["errors"].append(f"{name} PBStream 保存异常：{e}")
        pending["remaining"] -= 1
        if pending["started"] and pending["remaining"] == 0:
            self.finish_save(request_id)

    def finish_save(self, request_id):
        pending = self.save_pending.pop(request_id, None)
        if pending is None:
            return
        success = not pending["errors"]
        message = (
            "共享地图与三份 PBStream 已保存"
            if success
            else "共享地图已保存；" + "；".join(pending["errors"])
        )
        self.publish_save_result(
            request_id, pending["destination"], success, message
        )
        self.get_logger().info(f"{message}：{pending['destination']}")

    def publish_save_result(self, request_id, destination, success, message):
        self.save_result_pub.publish(
            String(
                data=json.dumps(
                    {
                        "request_id": request_id,
                        "destination": str(destination),
                        "success": success,
                        "message": message,
                    },
                    ensure_ascii=False,
                )
            )
        )


def main():
    rclpy.init()
    n = Mapping()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
