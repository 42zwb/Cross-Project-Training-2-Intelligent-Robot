import argparse, json, sys
from pathlib import Path
import rclpy
import yaml
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from fleet_interfaces.srv import FleetCommand, SetFormation, SetRegions


def main():
    p = argparse.ArgumentParser(description="三机器人协同系统指令")
    sub = p.add_subparsers(dest="cmd", required=True)
    for cmd in ["explore", "pause", "estop", "clear-estop"]:
        sub.add_parser(cmd)
    s = sub.add_parser("save")
    s.add_argument("--output", default="")
    s = sub.add_parser("patrol", help="执行 YAML/JSON 航点任务")
    s.add_argument("--file", required=True)
    s = sub.add_parser("formation")
    s.add_argument("shape", choices=["triangle", "row", "column"])
    s.add_argument("--x", type=float, default=0.0)
    s.add_argument("--y", type=float, default=0.0)
    s.add_argument("--yaw", type=float, default=0.0)
    s.add_argument("--spacing", type=float, default=1.2)
    s = sub.add_parser("regions")
    s.add_argument("bounds", type=float, nargs=12, help="三组 xmin ymin xmax ymax")
    sub.add_parser("status")
    a = p.parse_args()
    rclpy.init()
    n = rclpy.create_node("fleetctl")
    try:
        if a.cmd == "status":
            result = []
            n.create_subscription(
                String,
                "/fleet/state",
                lambda m: result.append(m.data),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            import time

            end = time.monotonic() + 5
            while not result and time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.1)
            if not result:
                raise RuntimeError("未收到状态，请先启动 ./run.sh 并 source ./env.sh")
            print(json.dumps(json.loads(result[-1]), ensure_ascii=False, indent=2))
            return
        if a.cmd == "formation":
            typ = SetFormation
            name = "/fleet/set_formation"
            req = typ.Request()
            req.shape = a.shape
            req.x = a.x
            req.y = a.y
            req.yaw = a.yaw
            req.spacing = a.spacing
        elif a.cmd == "regions":
            typ = SetRegions
            name = "/fleet/set_regions"
            req = typ.Request()
            req.bounds = a.bounds
        else:
            typ = FleetCommand
            name = "/fleet/command"
            req = typ.Request()
            req.command = a.cmd
            req.argument = (
                str(Path(a.output).expanduser().resolve())
                if a.cmd == "save" and a.output
                else ""
            )
            if a.cmd == "patrol":
                path = Path(a.file).expanduser()
                if path.stat().st_size > 16384:
                    raise ValueError("巡逻文件不能超过 16384 字节")
                req.argument = json.dumps(yaml.safe_load(path.read_text()), allow_nan=False)
        c = n.create_client(typ, name)
        if not c.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("调度服务不可用，请先启动 ./run.sh 并 source ./env.sh")
        f = c.call_async(req)
        rclpy.spin_until_future_complete(n, f, timeout_sec=15.0)
        if not f.done():
            raise RuntimeError("调度服务响应超时")
        r = f.result()
        print(("成功：" if r.success else "拒绝：") + r.message)
        if not r.success:
            sys.exit(1)
    except (RuntimeError, ValueError, TypeError, OSError, yaml.YAMLError, KeyboardInterrupt) as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
