"""Start Nav2 only after the complete stationary-calibrated TF tree exists."""

import argparse, json, signal, subprocess, time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--assets", required=True)
    a, _ = p.parse_known_args()
    rclpy.init()
    n = rclpy.create_node("navigation_bootstrap")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    d = {}
    b = Buffer()
    listener = TransformListener(b, n)

    def cb(m):
        try:
            state = json.loads(m.data)
        except ValueError:
            return
        if isinstance(state, dict):
            d.update(state)

    sub = n.create_subscription(
        String,
        "/fleet/mapping_state",
        cb,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    stable = None
    proc = None
    try:
        end = time.monotonic() + 120
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
            aligned = d.get("aligned")
            local_maps = d.get("local_maps")
            ready = (
                isinstance(aligned, list)
                and len(aligned) == 3
                and isinstance(local_maps, list)
                and len(local_maps) == 3
            )
            if ready:
                try:
                    for name in ["robot1", "robot2", "robot3"]:
                        t = b.lookup_transform(
                            "map", name + "/base_link", rclpy.time.Time()
                        )
                        age = (
                            n.get_clock().now()
                            - rclpy.time.Time.from_msg(t.header.stamp)
                        ).nanoseconds * 1e-9
                        ready &= -0.1 < age < 0.3
                except TransformException:
                    ready = False
            if ready:
                if stable is None:
                    stable = time.monotonic()
                if time.monotonic() - stable > 1.5:
                    break
            else:
                stable = None
        else:
            raise RuntimeError("三车 SLAM/TF 未能就绪，未启动导航")
        n.get_logger().info("完整 TF 树和三张局部地图已就绪，启动三套 Nav2")
        proc = subprocess.Popen(
            [
                "ros2",
                "launch",
                "fleet_bringup",
                "navigation.launch.py",
                f"assets:={Path(a.assets)}",
            ]
        )
        while rclpy.ok() and proc.poll() is None:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                proc.terminate()
        n.destroy_node()
        rclpy.try_shutdown()
