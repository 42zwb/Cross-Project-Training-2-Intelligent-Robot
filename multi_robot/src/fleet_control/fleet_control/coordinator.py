"""Shared exploration/formation state machine. No simulator truth input.
A worker performs long operations; ROS callbacks and safety leases stay live.
Every operation carries a generation token so cancellation cannot revive motion.
"""

import datetime, itertools, json, math, queue, threading, time, shutil, uuid
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PolygonStamped
from bond.msg import Status as BondStatus
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from lifecycle_msgs.srv import GetState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from fleet_interfaces.srv import SetRegions, SetFormation, FleetCommand
from .geometry import (
    Grid,
    angle,
    yaw,
    formation,
    formation_pass,
    validate_regions,
    plan_formation,
)


class Coordinator(Node):
    def __init__(self):
        super().__init__("coordinator")
        self.declare_parameter("config", "")
        self.declare_parameter("run_dir", "/tmp/fleet")
        self.cfg = yaml.safe_load(Path(self.get_parameter("config").value).read_text())
        self.robots = self.cfg["robots"]
        self.names = [r["name"] for r in self.robots]
        self.regions = [r["region"] for r in self.robots]
        validate_regions(
            self.regions,
            [r["start"] for r in self.robots],
            self.cfg["arena"],
            margin=self.cfg["region_margin"],
            robot_radius=self.cfg["robot_radius"],
        )
        self.run = Path(self.get_parameter("run_dir").value)
        self.run.mkdir(parents=True, exist_ok=True)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.save_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.mode = "BOOTSTRAP"
        self.message = "等待三车雷达、SLAM、TF 和 Nav2 就绪"
        self.ready = False
        self.initial_free_confirmed = False
        self.estop = False
        self.started = False
        self.region_enabled = True
        self.generation = 0
        self.active = set()
        self.mapping = {}
        self.mapping_received = 0.0
        self.safety_received = 0.0
        self.safety = {}
        self.footprints = {}
        self.grid = None
        self.poses = {}
        self.odoms = {}
        self.travel = {n: 0.0 for n in self.names}
        self.trails = {n: [] for n in self.names}
        self.handles = {}
        self.goals = {}
        self.blacklist = {n: [] for n in self.names}
        self.visits = {n: [] for n in self.names}
        self.slots = None
        self.assignment = None
        self.errors = {}
        self.taskqueue = queue.Queue()
        self.explore_done = {n: False for n in self.names}
        self.bond_instances = {}
        self.nav_ready = False
        self.last_ready_check = 0.0
        self.saved = None
        self.save_condition = threading.Condition()
        self.save_results = {}
        self.running = True
        self.statepub = self.create_publisher(String, "/fleet/state", self.qos)
        self.leasepub = self.create_publisher(String, "/fleet/drive_lease", 1)
        self.savepub = self.create_publisher(
            String, "/fleet/save_request", self.save_qos
        )
        self.maskpub = {}
        self.infopub = {}
        self.actions = {}
        for n in self.names:
            self.maskpub[n] = self.create_publisher(
                OccupancyGrid, f"/{n}/keepout_mask", self.qos
            )
            self.infopub[n] = self.create_publisher(
                CostmapFilterInfo, f"/{n}/keepout_info", self.qos
            )
            self.actions[n] = ActionClient(
                self,
                NavigateToPose,
                f"/{n}/navigate_to_pose",
                callback_group=self.group,
            )
            self.create_subscription(
                BondStatus, f"/{n}/bond", lambda m, n=n: self.bond_cb(n, m), 100
            )
            self.create_subscription(
                Odometry,
                f"/{n}/odom",
                lambda m, n=n: self.odom(n, m),
                qos_profile_sensor_data,
            )
            for scope in ["local_costmap", "global_costmap"]:
                key = n + "/" + scope
                self.create_subscription(
                    PolygonStamped,
                    f"/{key}/published_footprint",
                    lambda m, k=key: self.footprints.update({k: m.header.stamp}),
                    10,
                )
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, self.qos)
        self.create_subscription(
            String, "/fleet/mapping_state", lambda m: self.mapping_cb(m), self.qos
        )
        self.create_subscription(
            String, "/fleet/safety_state", lambda m: self.safety_cb(m), 1
        )
        self.create_subscription(
            String, "/fleet/save_result", self.save_result_cb, self.save_qos
        )
        self.create_service(
            FleetCommand, "/fleet/command", self.command, callback_group=self.group
        )
        self.create_service(
            SetRegions,
            "/fleet/set_regions",
            self.set_regions,
            callback_group=self.group,
        )
        self.create_service(
            SetFormation,
            "/fleet/set_formation",
            self.set_formation,
            callback_group=self.group,
        )
        self.create_timer(0.1, self.tick, callback_group=self.group)
        self.create_timer(2.0, self.publish_masks)
        self.eventfile = open(self.run / "events.jsonl", "a", buffering=1)
        self.telemetry = open(self.run / "telemetry.jsonl", "a", buffering=1)
        self.event_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.lastlog = 0.0
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()
        self.event("startup", config=self.cfg)

    def event(self, kind, **kwargs):
        entry = {
            "wall_time": time.time(),
            "sim_time": self.get_clock().now().nanoseconds * 1e-9,
            "event": kind,
            **kwargs,
        }
        with self.event_lock:
            self.eventfile.write(json.dumps(entry, ensure_ascii=False) + "\n")
        self.get_logger().info(f"{kind}: {kwargs}")

    def mapping_cb(self, m):
        try:
            mapping = json.loads(m.data)
            if not isinstance(mapping, dict):
                raise ValueError("invalid mapping state")
            aligned = mapping.get("aligned")
            local_maps = mapping.get("local_maps")
            scan_age = mapping.get("scan_age")
            known_fraction = mapping.get("known_fraction")
            if (
                not isinstance(aligned, list)
                or not all(isinstance(name, str) for name in aligned)
                or set(aligned) != set(self.names)
                or not isinstance(local_maps, list)
                or not all(isinstance(name, str) for name in local_maps)
                or set(local_maps) != set(self.names)
                or not isinstance(scan_age, dict)
                or set(scan_age) != set(self.names)
                or isinstance(known_fraction, bool)
                or not isinstance(known_fraction, (int, float))
                or not 0.0 <= known_fraction <= 1.0
                or any(
                    isinstance(age, bool)
                    or not isinstance(age, (int, float))
                    or not 0.0 <= age < 60.0
                    for age in scan_age.values()
                )
            ):
                raise ValueError("invalid mapping state")
            self.mapping = mapping
            self.mapping_received = time.monotonic()
        except ValueError:
            self.mapping = {}
            self.mapping_received = 0.0

    def safety_cb(self, m):
        try:
            safety = json.loads(m.data)
            if (
                not isinstance(safety, dict)
                or set(safety) != set(self.names)
                or any(
                    not isinstance(state, dict)
                    or not isinstance(state.get("reason"), str)
                    for state in safety.values()
                )
            ):
                raise ValueError("invalid safety state")
            self.safety = safety
            self.safety_received = time.monotonic()
        except ValueError:
            self.safety = {}
            self.safety_received = 0.0

    def save_result_cb(self, m):
        try:
            result = json.loads(m.data)
        except ValueError:
            return
        if not isinstance(result, dict):
            return
        request_id = result.get("request_id")
        if (
            not isinstance(request_id, str)
            or not request_id
            or not isinstance(result.get("success"), bool)
        ):
            return
        with self.save_condition:
            self.save_results[request_id] = result
            while len(self.save_results) > 32:
                self.save_results.pop(next(iter(self.save_results)))
            self.save_condition.notify_all()
        if result.get("success"):
            with self.lock:
                self.saved = result.get("destination")
        self.event(
            "save_result",
            destination=result.get("destination"),
            success=bool(result.get("success")),
            message=result.get("message", ""),
        )

    def bond_cb(self, name, message):
        key = (name, message.id, message.instance_id)
        if message.active:
            self.bond_instances[key] = time.monotonic()
        else:
            self.bond_instances.pop(key, None)

    def bonds_fresh(self, now):
        # A bond has two instances: lifecycle node and manager. Requiring both
        # prevents the manager's own heartbeats from hiding a dead server.
        return all(
            sum(
                robot == name and role == server and now - stamp < 2.0
                for (robot, role, _), stamp in list(self.bond_instances.items())
            )
            >= 2
            for name in self.names
            for server in [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ]
        )

    def odom(self, n, m):
        old = self.odoms.get(n)
        if old and self.mode == "EXPLORING" and abs(m.twist.twist.linear.x) > 0.01:
            a = old[1].pose.pose.position
            b = m.pose.pose.position
            ds = math.hypot(a.x - b.x, a.y - b.y)
            if ds < 0.15:
                self.travel[n] += ds
        self.odoms[n] = (time.monotonic(), m)

    def map_cb(self, m):
        self.grid = Grid(
            np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width),
            m.info.resolution,
            (m.info.origin.position.x, m.info.origin.position.y),
            self.cfg["robot_radius"],
            self.cfg["region_margin"],
        )

    def publish_masks(self):
        res = self.cfg["resolution"]
        a = self.cfg["arena"]
        w = round((a[2] - a[0]) / res)
        h = round((a[3] - a[1]) / res)
        yy, xx = np.indices((h, w))
        x = a[0] + (xx + 0.5) * res
        y = a[1] + (yy + 0.5) * res
        with self.lock:
            regions = list(self.regions)
        margin = self.cfg["region_margin"]
        for n, b in zip(self.names, regions):
            m = OccupancyGrid()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.info.resolution = res
            m.info.width = w
            m.info.height = h
            m.info.origin.position.x = a[0]
            m.info.origin.position.y = a[1]
            m.info.origin.orientation.w = 1.0
            m.data = (
                np.where(
                    (x >= b[0] + margin)
                    & (x <= b[2] - margin)
                    & (y >= b[1] + margin)
                    & (y <= b[3] - margin),
                    0,
                    100,
                )
                .astype(np.int8)
                .flatten()
                .tolist()
            )
            self.maskpub[n].publish(m)
            f = CostmapFilterInfo()
            f.header = m.header
            f.type = 0
            f.filter_mask_topic = f"/{n}/keepout_mask"
            f.base = 0.0
            f.multiplier = 1.0
            self.infopub[n].publish(f)

    def tick(self):
        with self.lock:
            now = time.monotonic()
            for n in self.names:
                try:
                    t = self.buffer.lookup_transform(
                        "map", n + "/base_link", rclpy.time.Time()
                    )
                    p = t.transform.translation
                    age = (
                        self.get_clock().now()
                        - rclpy.time.Time.from_msg(t.header.stamp)
                    ).nanoseconds * 1e-9
                    if not -0.5 <= age < 1.0:
                        self.poses.pop(n, None)
                        continue
                    pos = (p.x, p.y, yaw(t.transform.rotation))
                    old = self.poses.get(n)
                    self.poses[n] = pos
                    if (
                        not self.trails[n]
                        or math.dist(self.trails[n][-1], pos[:2]) > 0.08
                    ):
                        self.trails[n].append(pos[:2])
                        self.trails[n] = self.trails[n][-3000:]
                except TransformException:
                    self.poses.pop(n, None)
            nav_poses_fresh = len(self.footprints) == 6 and all(
                -0.1
                < (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds
                * 1e-9
                < 2.0
                for stamp in self.footprints.values()
            )
            dependencies_fresh = (
                now - self.mapping_received < 1.5
                and now - self.safety_received < 1.0
                and len(self.mapping.get("scan_age", {})) == 3
            )
            nav_bonds_fresh = self.bonds_fresh(now)
            initial_free_ready = self.grid is not None and all(
                name in self.poses
                and self.grid.inside(self.grid.cell(self.poses[name]))
                and self.grid.free_clearance[self.grid.cell(self.poses[name])]
                >= self.grid.clearance_radius
                for name in self.names
            )
            if initial_free_ready:
                self.initial_free_confirmed = True
            self.ready = (
                self.initial_free_confirmed
                and nav_bonds_fresh
                and dependencies_fresh
                and nav_poses_fresh
                and len(self.poses) == 3
                and len(self.mapping.get("aligned", [])) == 3
                and len(self.mapping.get("local_maps", [])) == 3
                and self.nav_ready
                and self.grid is not None
                and all(age < 0.8 for age in self.mapping.get("scan_age", {}).values())
            )
            if self.mode == "BOOTSTRAP" and self.ready:
                self.mode = "IDLE"
                self.message = "三车就绪，可以划分区域并开始探索"
                self.event("ready")
            critical = [
                v.get("reason")
                for n, v in self.safety.items()
                if n in self.active
                and v.get("reason")
                in [
                    "雷达超时",
                    "雷达数据无效",
                    "里程计超时",
                    "定位超时",
                    "定位丢失",
                    "调度心跳中断",
                ]
            ]
            if self.active and not nav_poses_fresh:
                critical.append("导航位姿超时")
            if self.active and not nav_bonds_fresh:
                critical.append("导航关键节点心跳中断")
            if self.active and (
                not dependencies_fresh
                or len(self.poses) != 3
                or any(age > 0.8 for age in self.mapping.get("scan_age", {}).values())
            ):
                critical.append("机群传感或定位中断")
            if critical and self.mode not in ["BOOTSTRAP", "IDLE", "ESTOP", "FAULT"]:
                self.generation += 1
                self.active.clear()
                self.mode = "FAULT"
                self.message = "安全停车：" + ",".join(critical)
                self.cancel_all()
                self.event("fault", reason=self.message)
            self.leasepub.publish(
                String(
                    data=json.dumps(
                        {
                            "active": list(self.active),
                            "estop": self.estop,
                            "region_enabled": self.region_enabled,
                            "regions": self.regions,
                        }
                    )
                )
            )
            if self.slots and self.assignment and len(self.poses) == 3:
                self.errors = {
                    n: {
                        "position": math.dist(
                            self.poses[n][:2], self.slots[self.assignment[i]][:2]
                        ),
                        "yaw": abs(
                            angle(self.poses[n][2] - self.slots[self.assignment[i]][2])
                        ),
                    }
                    for i, n in enumerate(self.names)
                }
            state = {
                "mode": self.mode,
                "message": self.message,
                "ready": self.ready,
                "estop": self.estop,
                "robots": {
                    n: {
                        "pose": self.poses.get(n),
                        "travel": self.travel[n],
                        "goal": self.goals.get(n),
                        "trail": self.trails[n],
                        "safety": self.safety.get(n, {}),
                    }
                    for n in self.names
                },
                "regions": self.regions,
                "region_enabled": self.region_enabled,
                "known_fraction": self.mapping.get("known_fraction", 0.0),
                "formation_slots": self.slots,
                "formation_assignment": self.assignment,
                "formation_errors": self.errors,
                "saved": self.saved,
                "arena": self.cfg["arena"],
                "nav_pose_age": {
                    k: (
                        self.get_clock().now() - rclpy.time.Time.from_msg(stamp)
                    ).nanoseconds
                    * 1e-9
                    for k, stamp in self.footprints.items()
                },
            }
            self.statepub.publish(String(data=json.dumps(state, ensure_ascii=False)))
            if now - self.lastlog > 0.5:
                with self.telemetry_lock:
                    self.telemetry.write(
                        json.dumps(
                            {k: v for k, v in state.items() if k != "robots"}
                            | {
                                "wall_time": time.time(),
                                "sim_time": self.get_clock().now().nanoseconds * 1e-9,
                                "robots": {
                                    n: {k: v for k, v in r.items() if k != "trail"}
                                    for n, r in state["robots"].items()
                                },
                            },
                            ensure_ascii=False,
                        )
                        + "\n"
                    )
                self.lastlog = now

    def cancel_all(self):
        with self.lock:
            handles = list(self.handles.values())
            self.handles.clear()
            self.goals.clear()
        for h in handles:
            if h:
                try:
                    h.cancel_goal_async()
                except Exception:
                    pass

    def valid(self, token):
        with self.lock:
            return (
                self.running
                and rclpy.ok()
                and token == self.generation
                and not self.estop
                and self.mode != "FAULT"
            )

    def set_task_state(self, token, mode, message):
        with self.lock:
            if not self.valid(token):
                return False
            self.mode = mode
            self.message = message
            return True

    def command(self, req, res):
        if req.command == "save":
            return self.save_command(req, res)
        with self.lock:
            cmd = req.command
            if cmd == "estop":
                self.estop = True
                self.generation += 1
                self.active.clear()
                self.cancel_all()
                self.mode = "ESTOP"
                self.message = "急停已锁定"
                self.event("estop")
                res.success = True
            elif cmd in ["pause", "clear-estop"]:
                if cmd == "clear-estop":
                    self.estop = False
                self.generation += 1
                self.active.clear()
                self.cancel_all()
                if self.estop:
                    self.mode = "ESTOP"
                    self.message = "急停仍锁定；请解除急停后再发出任务"
                else:
                    self.mode = "IDLE" if self.ready else "BOOTSTRAP"
                    self.message = "已停车，等待新指令"
                self.event(cmd)
                res.success = True
            elif cmd == "explore":
                if not self.ready or self.estop or self.mode in ["FAULT", "ESTOP"]:
                    res.message = "系统未就绪、处于故障停车或急停未解除"
                    return res
                self.generation += 1
                self.active.clear()
                self.cancel_all()
                self.mode = "SWITCHING"
                self.started = True
                self.message = "准备探索"
                self.taskqueue.put(("explore", self.generation, None))
                res.success = True
            else:
                res.message = "未知指令"
                return res
            res.message = self.message
            return res

    def save_command(self, req, res):
        with self.lock:
            if not self.mapping or time.monotonic() - self.mapping_received > 1.5:
                res.message = "映射节点未就绪，无法保存地图"
                return res
            if self.grid is None:
                res.message = "没有地图"
                return res
            dest = (
                Path(req.argument).expanduser().resolve()
                if req.argument
                else self.run
                / "results"
                / datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            )
            snapshot = {
                "travel": dict(self.travel),
                "errors": dict(self.errors),
                "mode": self.mode,
                "known_fraction": self.mapping.get("known_fraction", 0.0),
            }
            regions = list(self.regions)
        request_id = uuid.uuid4().hex
        try:
            dest.mkdir(parents=True, exist_ok=True)
            (dest / "regions.yaml").write_text(
                yaml.safe_dump({"regions": regions})
            )
            (dest / "snapshot.json").write_text(
                json.dumps(snapshot, ensure_ascii=False, indent=2)
            )
            self.event("save", destination=str(dest))
            for filename in ["events.jsonl", "telemetry.jsonl", "resources.jsonl"]:
                source = self.run / filename
                if not source.exists():
                    continue
                target = dest / filename
                if source.resolve() == target.resolve():
                    continue
                if filename == "events.jsonl":
                    with self.event_lock:
                        shutil.copyfile(source, target)
                elif filename == "telemetry.jsonl":
                    with self.telemetry_lock:
                        shutil.copyfile(source, target)
                else:
                    shutil.copyfile(source, target)
            (dest / "metrics.json").write_text(
                json.dumps(
                    {
                        "known_fraction": snapshot["known_fraction"],
                        "odometry_explore_distance": snapshot["travel"],
                        "formation_errors": snapshot["errors"],
                        "mode": snapshot["mode"],
                        "truth_coverage": "需独立运行 evaluate 计算",
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
        except OSError as e:
            res.message = f"保存前准备运行结果失败：{e}"
            return res
        self.savepub.publish(
            String(
                data=json.dumps(
                    {"request_id": request_id, "destination": str(dest)}
                )
            )
        )
        end = time.monotonic() + 8.0
        with self.save_condition:
            while request_id not in self.save_results and time.monotonic() < end:
                self.save_condition.wait(timeout=min(0.2, end - time.monotonic()))
            result = self.save_results.pop(request_id, None)
        if result is None:
            res.success = True
            res.message = f"地图保存仍在后台执行，结果目录：{dest}"
        else:
            res.success = bool(result.get("success"))
            res.message = str(result.get("message") or dest)
        return res

    def set_regions(self, req, res):
        with self.lock:
            try:
                if self.started or self.mode not in ["BOOTSTRAP", "IDLE"]:
                    raise ValueError("只能在第一次开始探索前设置区域。")
                regions = np.asarray(req.bounds).reshape(3, 4).tolist()
                validate_regions(
                    regions,
                    [r["start"] for r in self.robots],
                    self.cfg["arena"],
                    margin=self.cfg["region_margin"],
                    robot_radius=self.cfg["robot_radius"],
                )
                self.regions = regions
                self.publish_masks()
                self.event("set_regions", regions=regions)
                res.success = True
                res.message = "区域已更新"
            except ValueError as e:
                res.message = str(e)
            return res

    def set_formation(self, req, res):
        with self.lock:
            try:
                if not self.ready or self.estop or self.mode in ["FAULT", "ESTOP"]:
                    raise ValueError("系统未就绪、处于故障停车或急停未解除。")
                slots = formation(req.shape, req.x, req.y, req.yaw, req.spacing)
                ok = self.grid.allowed()
                for p in slots:
                    cell = self.grid.cell(p)
                    if not self.grid.inside(cell) or not ok[cell]:
                        raise ValueError(
                            "队形目标包含未知、障碍或安全余量不足的区域，请先探索或调整中心/间距。"
                        )
                self.generation += 1
                self.active.clear()
                self.cancel_all()
                self.mode = "SWITCHING"
                self.message = "停车并准备队形"
                self.taskqueue.put(("formation", self.generation, slots))
                self.event("formation_request", shape=req.shape, slots=slots)
                res.success = True
                res.message = "队形请求已接受，正在规划分步到位顺序"
            except ValueError as e:
                res.message = str(e)
            return res

    def wait(self, future, timeout=10.0, token=None):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            if not self.running or (token is not None and not self.valid(token)):
                return None
            time.sleep(0.02)
        return future.result() if future.done() else None

    def call(self, typ, name, req, timeout=8.0, token=None):
        c = self.create_client(typ, name, callback_group=self.group)
        try:
            service_deadline = time.monotonic() + min(2.0, timeout)
            while self.running and not c.service_is_ready() and time.monotonic() < service_deadline:
                time.sleep(0.05)
            if not self.running:
                raise RuntimeError("调度器正在关闭")
            if not c.service_is_ready():
                raise RuntimeError(f"服务不可用：{name}")
            result = self.wait(c.call_async(req), timeout, token)
            if result is None:
                raise RuntimeError(f"服务超时/操作取消：{name}")
            return result
        finally:
            self.destroy_client(c)

    def filters(self, enabled, token):
        for n in self.names:
            for scope in ["global_costmap", "local_costmap"]:
                req = SetBool.Request()
                req.data = enabled
                r = self.call(
                    SetBool,
                    f"/{n}/{scope}/keepout_filter/toggle_filter",
                    req,
                    token=token,
                )
                if not r.success:
                    raise RuntimeError("分区过滤器切换失败：" + r.message)
                self.call(
                    ClearEntireCostmap,
                    f"/{n}/{scope}/clear_entirely_{scope}",
                    ClearEntireCostmap.Request(),
                    token=token,
                )
        with self.lock:
            if not self.valid(token):
                return
            self.region_enabled = enabled
        self.event("region_filters", enabled=enabled)
        end = time.monotonic() + 1.2
        while self.valid(token) and time.monotonic() < end:
            time.sleep(0.05)

    def stopped(self, token):
        with self.lock:
            self.active.clear()
        self.cancel_all()
        start = time.monotonic()
        stable = None
        while self.valid(token) and time.monotonic() - start < 8.0:
            is_stopped = all(
                n in self.odoms
                and time.monotonic() - self.odoms[n][0] < 0.8
                and abs(self.odoms[n][1].twist.twist.linear.x) < 0.015
                and abs(self.odoms[n][1].twist.twist.angular.z) < 0.04
                for n in self.names
            )
            if is_stopped:
                if stable is None:
                    stable = time.monotonic()
                if time.monotonic() - stable > 0.6:
                    return True
            else:
                stable = None
            time.sleep(0.05)
        return False

    def send_goal(self, n, target, token):
        with self.lock:
            if not self.valid(token):
                return None
            self.active.add(n)
        req = NavigateToPose.Goal()
        req.pose.header.frame_id = "map"
        req.pose.header.stamp = self.get_clock().now().to_msg()
        req.pose.pose.position.x = float(target[0])
        req.pose.pose.position.y = float(target[1])
        req.pose.pose.orientation.z = math.sin(target[2] / 2)
        req.pose.pose.orientation.w = math.cos(target[2] / 2)
        h = self.wait(self.actions[n].send_goal_async(req), 8.0, token)
        if h is None or not h.accepted:
            with self.lock:
                self.active.discard(n)
            return None
        with self.lock:
            if not self.valid(token):
                canceled = True
            else:
                canceled = False
                self.handles[n] = h
                self.goals[n] = list(target)
        if canceled:
            h.cancel_goal_async()
            return None
        self.event("goal", robot=n, target=target, mode=self.mode)
        return h.get_result_async()

    def navigate(self, n, target, token, timeout=240.0):
        f = self.send_goal(n, target, token)
        if f is None:
            return False
        result = self.wait(f, timeout, token)
        with self.lock:
            self.active.discard(n)
            h = self.handles.pop(n, None)
            self.goals.pop(n, None)
        if result is None:
            if h:
                h.cancel_goal_async()
            return False
        self.event("goal_result", robot=n, status=result.status, target=target)
        return result.status == 4

    def check_nav(self):
        active = True
        for n in self.names:
            if not self.running:
                return False
            if not self.actions[n].server_is_ready():
                return False
            for server in [
                "controller_server",
                "planner_server",
                "bt_navigator",
                "behavior_server",
            ]:
                try:
                    if not self.running:
                        return False
                    r = self.call(
                        GetState,
                        f"/{n}/{server}/get_state",
                        GetState.Request(),
                        timeout=2.0,
                    )
                    active &= r.current_state.id == 3
                except RuntimeError:
                    return False
        return active

    def work(self):
        last_explore = 0.0
        while self.running and rclpy.ok():
            token = None
            try:
                if time.monotonic() - self.last_ready_check > 5.0:
                    self.nav_ready = self.check_nav()
                    self.last_ready_check = time.monotonic()
                try:
                    kind, token, payload = self.taskqueue.get(timeout=0.2)
                except queue.Empty:
                    with self.lock:
                        exploring = self.mode == "EXPLORING"
                        token = self.generation
                    if exploring and time.monotonic() - last_explore > 2.0:
                        self.explore_tick(token)
                        last_explore = time.monotonic()
                    continue
                if not self.valid(token):
                    continue
                if not self.stopped(token):
                    if self.valid(token):
                        raise RuntimeError("未能确认三车停止")
                    continue
                if kind == "formation":
                    self.do_formation(payload, token)
                elif kind == "explore":
                    self.start_explore(token)
            except Exception as e:
                with self.lock:
                    if token is not None and token != self.generation:
                        continue
                    if self.running and rclpy.ok():
                        self.active.clear()
                        self.cancel_all()
                        self.mode = "FAULT"
                        self.message = str(e)
                        self.event("operation_error", error=str(e))

    def start_explore(self, token):
        # A robot in formation can be outside its region. Return before enabling masks.
        outside = any(
            not b[0] + self.cfg["region_margin"]
            <= self.poses[n][0]
            <= b[2] - self.cfg["region_margin"]
            or not b[1] + self.cfg["region_margin"]
            <= self.poses[n][1]
            <= b[3] - self.cfg["region_margin"]
            for n, b in zip(self.names, self.regions)
        )
        if outside:
            self.filters(False, token)
            if not self.set_task_state(token, "RETURNING", "返回各自建图区域"):
                return
            for r in self.robots:
                if not self.valid(token):
                    return
                if not self.navigate(r["name"], r["start"], token):
                    raise RuntimeError(r["name"] + " 返回区域失败，已停车")
        if not self.valid(token):
            return
        self.filters(True, token)
        if not self.set_task_state(
            token, "EXPLORING", "三车正在所属区域自主探索"
        ):
            return
        with self.lock:
            if not self.valid(token):
                return
            self.explore_done = {n: False for n in self.names}
            self.exploring = {}
        self.event("explore_start")

    def explore_tick(self, token):
        if not self.valid(token) or self.grid is None:
            return
        for i, n in enumerate(self.names):
            if n in getattr(self, "exploring", {}):
                future, started, target = self.exploring[n]
                if not future.done() and time.monotonic() - started < 150.0:
                    continue
                with self.lock:
                    h = self.handles.pop(n, None)
                if not future.done():
                    if h:
                        h.cancel_goal_async()
                    status = 0
                else:
                    status = future.result().status
                with self.lock:
                    self.active.discard(n)
                    self.goals.pop(n, None)
                    self.exploring.pop(n, None)
                if status != 4:
                    self.blacklist[n].append(target)
                else:
                    self.visits[n].append(target)
                self.event(
                    "explore_goal_result", robot=n, status=status, travel=self.travel[n]
                )
            if not self.valid(token) or n not in self.poses:
                return
            with self.lock:
                reserved = [t for name, t in self.goals.items() if name != n]
            result = self.grid.frontiers(
                self.poses[n],
                self.regions[i],
                reserved,
                self.blacklist[n],
                self.visits[n],
                self.travel[n] < 3.2,
            )
            if result is None:
                self.explore_done[n] = True
                continue
            self.explore_done[n] = False
            p = result[1]
            heading = math.atan2(p[1] - self.poses[n][1], p[0] - self.poses[n][0])
            target = (p[0], p[1], heading)
            f = self.send_goal(n, target, token)
            if f:
                self.exploring[n] = (f, time.monotonic(), target)
            else:
                self.blacklist[n].append(target)
        if all(self.explore_done.values()) and not self.exploring:
            with self.lock:
                travel = dict(self.travel)
            if any(distance < 3.2 for distance in travel.values()):
                # Initial SLAM maps can contain obstacles before free-cell
                # confidence converges. Keep retrying observations, instead of
                # declaring completion before any robot has participated.
                with self.lock:
                    if self.valid(token):
                        self.message = "等待足够的空闲观测后继续探索"
                return
            with self.lock:
                if not self.valid(token):
                    return
                self.active.clear()
                self.mode = "EXPLORED"
                self.message = "可达前沿探索完成，可保存地图或切换队形"
            self.event("explore_complete", travel=travel)

    def do_formation(self, slots, token):
        result = plan_formation(self.grid, [self.poses[n] for n in self.names], slots)
        if not self.valid(token):
            return
        if result is None:
            raise RuntimeError("无法找到安全到位顺序或等待点；保持停车")
        _, assignment, routes = result
        with self.lock:
            if not self.valid(token):
                return
            self.slots = slots
            self.assignment = list(assignment)
        self.filters(False, token)
        if not self.set_task_state(token, "FORMING", "正在分步到达队形位置"):
            return
        self.event(
            "formation_plan",
            assignment=assignment,
            order=[self.names[i] for i, _, _ in routes],
        )
        for i, target, path in routes:
            if not self.valid(token):
                return
            if not self.navigate(self.names[i], target, token):
                if self.valid(token):
                    raise RuntimeError(self.names[i] + " 队形导航失败，已停车")
                return
        with self.lock:
            self.active.clear()
        stable = None
        end = time.monotonic() + 12.0
        while self.valid(token) and time.monotonic() < end:
            good = len(self.poses) == 3 and formation_pass(
                [self.poses[n] for n in self.names],
                slots,
                assignment,
                self.cfg["robot_radius"],
            )
            if good:
                if stable is None:
                    stable = time.monotonic()
                if time.monotonic() - stable >= 3.0:
                    if not self.set_task_state(
                        token, "FORMATION_READY", "队形已到位并稳定保持 3 秒"
                    ):
                        return
                    self.event(
                        "formation_ready",
                        slots=slots,
                        assignment=assignment,
                        errors=self.errors,
                    )
                    return
            else:
                stable = None
            time.sleep(0.1)
        if self.valid(token):
            raise RuntimeError("导航结束，但独立队形误差检查未通过")


def main():
    rclpy.init()
    n = Coordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(n)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        with n.lock:
            n.running = False
            n.generation += 1
            n.active.clear()
        n.cancel_all()
        n.worker.join()
        executor.shutdown(timeout_sec=3.0)
        n.eventfile.close()
        n.telemetry.close()
        n.destroy_node()
        rclpy.try_shutdown()
