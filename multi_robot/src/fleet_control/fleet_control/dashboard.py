"""Offline, loopback-only dashboard; browser lifetime is independent of ROS."""

import io, json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from fleet_interfaces.srv import FleetCommand, SetFormation, SetRegions


class Dashboard(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.declare_parameter("config", "")
        cfg = yaml.safe_load(Path(self.get_parameter("config").value).read_text())
        self.port = cfg["dashboard_port"]
        self.state = {"mode": "BOOTSTRAP", "message": "正在连接 ROS"}
        self.png = b""
        self.mapmeta = {}
        self.version = 0
        self.web = Path(get_package_share_directory("fleet_control")) / "web"
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/fleet/state", self.state_cb, q)
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, q)
        self.fleet_clients = {
            "command": self.create_client(FleetCommand, "/fleet/command"),
            "formation": self.create_client(SetFormation, "/fleet/set_formation"),
            "regions": self.create_client(SetRegions, "/fleet/set_regions"),
        }

    def state_cb(self, m):
        try:
            state = json.loads(m.data)
        except ValueError:
            self.state = {"mode": "FAULT", "message": "收到无效的机群状态"}
            return
        if isinstance(state, dict):
            self.state = state
        else:
            self.state = {"mode": "FAULT", "message": "收到无效的机群状态"}

    def map_cb(self, m):
        if (
            m.info.width <= 0
            or m.info.height <= 0
            or m.info.resolution <= 0
            or len(m.data) != m.info.width * m.info.height
        ):
            return
        a = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        rgb = np.zeros((*a.shape, 3), dtype=np.uint8)
        rgb[:] = [68, 81, 96]
        rgb[a == 0] = [238, 242, 246]
        rgb[a >= 65] = [28, 34, 42]
        out = io.BytesIO()
        Image.fromarray(np.flipud(rgb)).save(out, format="PNG")
        self.png = out.getvalue()
        self.version += 1
        self.mapmeta = {
            "width": m.info.width,
            "height": m.info.height,
            "resolution": m.info.resolution,
            "origin": [m.info.origin.position.x, m.info.origin.position.y],
        }

    def request(self, kind, data):
        if kind == "formation":
            r = SetFormation.Request()
            r.shape = str(data["shape"])
            r.x = float(data.get("x", 0))
            r.y = float(data.get("y", 0))
            r.yaw = float(data.get("yaw", 0))
            r.spacing = float(data.get("spacing", 1.2))
        elif kind == "regions":
            r = SetRegions.Request()
            r.bounds = [float(v) for row in data["regions"] for v in row]
        elif kind == "command":
            r = FleetCommand.Request()
            r.command = str(data["command"])
            r.argument = str(data.get("argument", ""))
        else:
            raise ValueError("不存在的接口")
        c = self.fleet_clients[kind]
        if not c.service_is_ready():
            raise RuntimeError("ROS 服务尚未就绪")
        f = c.call_async(r)
        end = time.monotonic() + 12
        while not f.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not f.done():
            raise RuntimeError("服务超时")
        r = f.result()
        return {"success": r.success, "message": r.message}


def main():
    rclpy.init()
    n = Dashboard()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, data, ctype):
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path.startswith("/api/state"):
                self.send(
                    200,
                    json.dumps(
                        n.state | {"map": n.mapmeta, "map_version": n.version},
                        ensure_ascii=False,
                    ).encode(),
                    "application/json; charset=utf-8",
                )
            elif self.path.startswith("/api/map.png"):
                self.send(200, n.png, "image/png")
            elif self.path in ["/", "/index.html"]:
                self.send(
                    200, (n.web / "index.html").read_bytes(), "text/html; charset=utf-8"
                )
            else:
                self.send(404, b"Not found", "text/plain")

        def do_POST(self):
            try:
                origin = self.headers.get("Origin", "")
                if origin and origin not in [
                    f"http://127.0.0.1:{n.port}",
                    f"http://localhost:{n.port}",
                ]:
                    raise ValueError("禁止跨站控制请求")
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError as e:
                    raise ValueError("请求长度无效") from e
                if not 1 <= length <= 16384:
                    raise ValueError("请求长度必须在 1 到 16384 字节之间")
                data = json.loads(self.rfile.read(length))
                kind = self.path.removeprefix("/api/")
                r = n.request(kind, data)
                self.send(
                    200,
                    json.dumps(r, ensure_ascii=False).encode(),
                    "application/json; charset=utf-8",
                )
            except (ValueError, KeyError, RuntimeError, TypeError) as e:
                self.send(
                    400,
                    json.dumps(
                        {"success": False, "message": str(e)}, ensure_ascii=False
                    ).encode(),
                    "application/json; charset=utf-8",
                )

    server = ThreadingHTTPServer(("127.0.0.1", n.port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    n.get_logger().info(f"网页：http://127.0.0.1:{n.port}")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        n.destroy_node()
        rclpy.try_shutdown()
