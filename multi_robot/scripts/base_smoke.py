#!/usr/bin/env python3
"""Run ONLY with ./run.sh --base-only. Check physical truth, isolated actuation,
640-ray valid scans, prefixed odom and native timeout. Saves machine evidence.
"""
import json, math, signal, subprocess, time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

root = Path(__file__).resolve().parents[1]
run = Path((root / "runs/latest.txt").read_text())
probe = subprocess.Popen(
    [str(root / "build/tools/truth_probe"), str(run / "base_truth.jsonl")]
)
rclpy.init()
node = rclpy.create_node("base_smoke")
names = ["robot1", "robot2", "robot3"]
scans = {}
odom = {}
pub = {}
subs = []
for n in names:
    subs.append(
        node.create_subscription(
            LaserScan,
            f"/{n}/scan",
            lambda m, n=n: scans.update({n: m}),
            qos_profile_sensor_data,
        )
    )
    subs.append(
        node.create_subscription(
            Odometry,
            f"/{n}/odom",
            lambda m, n=n: odom.update({n: m}),
            qos_profile_sensor_data,
        )
    )
    pub[n] = node.create_publisher(
        Twist, f"/{n}/diffdrive_controller/cmd_vel_unstamped", 1
    )


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.03)


def truth():
    rows = (run / "base_truth.jsonl").read_text().splitlines()
    return next(
        json.loads(row)["robots"]
        for row in reversed(rows)
        if len(json.loads(row)["robots"]) == 3
    )


try:
    end = time.monotonic() + 30
    while (len(scans) < 3 or len(odom) < 3) and time.monotonic() < end:
        pump(0.1)
    assert len(scans) == 3 and len(odom) == 3, "三车传感器未就绪"
    report = {"scans": {}, "runs": []}
    for n in names:
        m = scans[n]
        valid = [v for v in m.ranges if math.isfinite(v) and v > m.range_min + 0.02]
        assert len(m.ranges) == 640 and len(valid) > 100, (n, len(valid))
        assert m.header.frame_id.startswith(n + "/"), m.header.frame_id
        assert (
            odom[n].header.frame_id == n + "/odom"
            and odom[n].child_frame_id == n + "/base_link"
        )
        report["scans"][n] = {
            "rays": len(m.ranges),
            "valid": len(valid),
            "frame": m.header.frame_id,
        }
    for n in names:
        before = truth()
        end = time.monotonic() + 4
        while time.monotonic() < end:
            m = Twist()
            m.linear.x = 0.12
            pub[n].publish(m)
            pump(0.05)
        # Stop publishing: verify native timeout, rather than sending a zero ourselves.
        pump(1.3)
        after = truth()
        delta = {k: math.dist(before[k][:2], after[k][:2]) for k in names}
        assert delta[n] > 0.30, delta
        assert max(delta[k] for k in names if k != n) < 0.04, delta
        final = after[n]
        pump(0.8)
        assert math.dist(final[:2], truth()[n][:2]) < 0.015, "底盘超时停车失败"
        report["runs"].append(
            {"robot": n, "actual_displacement": delta, "timeout_stopped": True}
        )
        print(report["runs"][-1], flush=True)
    (run / "base_acceptance.json").write_text(json.dumps(report, indent=2))
    print("三车底座验收通过：" + str(run / "base_acceptance.json"))
finally:
    for p in pub.values():
        p.publish(Twist())
    probe.send_signal(signal.SIGINT)
    probe.wait(timeout=4)
    node.destroy_node()
    rclpy.try_shutdown()
