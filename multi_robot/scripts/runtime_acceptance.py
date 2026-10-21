#!/usr/bin/env python3
"""Read-only acceptance: isolated scans/odom/control and a connected TF forest."""
import json, math, time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

root = Path(__file__).resolve().parents[1]
run = Path((root / "runs/latest.txt").read_text())
rclpy.init()
node = rclpy.create_node("fleet_runtime_acceptance")
buffer = Buffer()
listener = TransformListener(buffer, node)
scans = {}
odom = {}
parents = {}
sources = {}
subs = []


def tf(msg):
    for t in msg.transforms:
        parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)


for topic, qos in [
    ("/tf", 100),
    ("/tf_static", QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
]:
    subs.append(node.create_subscription(TFMessage, topic, tf, qos))
for i in range(1, 4):
    name = f"robot{i}"
    subs.append(
        node.create_subscription(
            LaserScan,
            f"/{name}/scan",
            lambda m, n=name: scans.update({n: m}),
            qos_profile_sensor_data,
        )
    )
    subs.append(
        node.create_subscription(
            Odometry,
            f"/{name}/odom",
            lambda m, n=name: odom.update({n: m}),
            qos_profile_sensor_data,
        )
    )
try:
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(scans) == len(odom) == 3
    assert all(len(p) == 1 for p in parents.values()), parents
    report = {
        "passed": True,
        "robots": {},
        "tf_frames": len(parents),
        "multiple_parents": [],
        "tf_publisher_nodes": [],
    }
    publishers = node.get_publishers_info_by_topic("/tf")
    publisher_nodes = [(p.node_namespace, p.node_name) for p in publishers]
    assert len(publisher_nodes) == len(set(publisher_nodes)), publisher_nodes
    report["tf_publisher_nodes"] = publisher_nodes
    for name in ["robot1", "robot2", "robot3"]:
        scan = scans[name]
        o = odom[name]
        assert (
            len(scan.ranges) == 640
            and sum(math.isfinite(v) and v > scan.range_min + 0.02 for v in scan.ranges)
            > 100
        )
        assert scan.header.frame_id == name + "/rplidar_link"
        assert (
            o.header.frame_id == name + "/odom"
            and o.child_frame_id == name + "/base_link"
        )
        for link in ["base_link", "rplidar_link", "left_wheel", "right_wheel"]:
            assert buffer.can_transform("map", name + "/" + link, rclpy.time.Time()), (
                name,
                link,
            )
        counts = {
            suffix: len(node.get_publishers_info_by_topic("/" + name + "/" + suffix))
            for suffix in ["scan", "odom", "diffdrive_controller/cmd_vel_unstamped"]
        }
        assert all(v == 1 for v in counts.values()), counts
        report["robots"][name] = {
            "scan_rays": len(scan.ranges),
            "frame": scan.header.frame_id,
            "publisher_counts": counts,
            "wheel_tf_connected": True,
        }
    (run / "runtime_acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
finally:
    node.destroy_node()
    rclpy.try_shutdown()
