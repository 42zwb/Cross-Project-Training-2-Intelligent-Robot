#!/usr/bin/env python3
"""Isolated payload and active-producer SIGINT/SIGTERM bridge regression.

Requires a built fleet_bridge. Never publishes to the real /clock or /robotN/scan.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_prefix

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--rounds", type=int, default=10)
p.add_argument("--output", required=True)
p.add_argument("--stock", action="store_true")
p.add_argument("--domain", type=int, default=231)
p.add_argument("--valgrind")
a = p.parse_args()
if not 1 <= a.rounds <= 100 or not 0 <= a.domain <= 232:
    p.error("rounds must be 1..100 and domain 0..232")
out = Path(a.output).resolve()
out.mkdir(parents=True, exist_ok=False)
env = os.environ.copy()
env["ROS_DOMAIN_ID"] = str(a.domain)
env["IGN_PARTITION"] = env["GZ_PARTITION"] = "fleet_bridge_probe_" + uuid.uuid4().hex
os.environ["ROS_DOMAIN_ID"] = str(a.domain)
folder = Path(get_package_prefix("fleet_bridge")) / "lib/fleet_bridge"
binary = Path("/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge") if a.stock else folder / "parameter_bridge"
counts = {"clock": 0, "scan": 0}
invalid = []
rclpy.init()
receiver = rclpy.create_node("fleet_bridge_probe_receiver")
def receive_clock(message):
    counts["clock"] += 1
    if not 0 <= message.clock.nanosec < 1_000_000_000:
        invalid.append("clock stamp")
def receive_scan(message):
    counts["scan"] += 1
    if (len(message.ranges) != 640 or message.header.frame_id != "probe/rplidar_link"
            or not math.isclose(message.angle_increment, 2 * math.pi / 640, rel_tol=1e-6)
            or any(value != 6.0 for value in message.ranges)):
        invalid.append({"scan_size":len(message.ranges),"frame":message.header.frame_id,
                        "angle_increment":message.angle_increment})
receiver.create_subscription(Clock, "/fleet_bridge_probe/clock", receive_clock, qos_profile_sensor_data)
receiver.create_subscription(LaserScan, "/fleet_bridge_probe/scan", receive_scan, qos_profile_sensor_data)
rows = []
source_log = (out / "source.log").open("w")
source = subprocess.Popen([str(folder / "bridge_traffic_source")], env=env, stdout=source_log, stderr=subprocess.STDOUT)
child = None
try:
    for index in range(a.rounds):
        before = dict(counts)
        cmd = [str(binary), "/fleet_bridge_probe/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
               "/fleet_bridge_probe/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"]
        if a.valgrind:
            tool = Path(a.valgrind).resolve()
            env["VALGRIND_LIB"] = str(tool.parents[1] / "libexec/valgrind")
            cmd = [str(tool), "--leak-check=no", "--error-exitcode=98",
                   "--log-file=" + str(out / f"memcheck-{index}.txt")] + cmd
        with (out / f"bridge-{index}.log").open("w") as log:
            child = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
            started = time.monotonic()
            end = started + (25 if a.valgrind else 10)
            while time.monotonic() < end and child.poll() is None:
                rclpy.spin_once(receiver, timeout_sec=0.05)
                if counts["clock"] - before["clock"] >= 10 and counts["scan"] - before["scan"] >= 3:
                    break
            payload = {key: counts[key] - before[key] for key in counts}
            # Keep the producer alive while the bridge receives its exit signal.
            sent = signal.SIGINT if index % 2 == 0 else signal.SIGTERM
            if child.poll() is None:
                child.send_signal(sent)
            try:
                status = child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill(); status = child.wait()
            row = {"round": index, "signal": sent.name, "returncode": status,
                   "payload": payload, "payload_valid": not invalid,
                   "elapsed": time.monotonic() - started}
            row["pass"] = status == 0 and payload["clock"] >= 10 and payload["scan"] >= 3 and not invalid
            rows.append(row)
            print(json.dumps(row), flush=True)
finally:
    if child is not None and child.poll() is None:
        child.kill(); child.wait()
    source.send_signal(signal.SIGTERM)
    try:
        source.wait(timeout=5)
    except subprocess.TimeoutExpired:
        source.kill(); source.wait()
    source_log.close()
    receiver.destroy_node()
    rclpy.shutdown()
    report = {"binary": str(binary), "stock": a.stock, "rounds": rows, "invalid_payloads":invalid,
              "all_pass": len(rows) == a.rounds and all(row["pass"] for row in rows)}
    (out / "report.json").write_text(json.dumps(report, indent=2))
if not report["all_pass"]:
    raise SystemExit(1)
