#!/usr/bin/env python3
"""Base-only physical/laser check. Teleports robot1 to safe concave bays.

This independent acceptance tool reads truth; production mapping never does.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from scipy.spatial import ConvexHull
import yaml

from fleet_mapping.shapes import raycast, shape_distance, vertices, bounds
from runtime_context import RuntimeClient

p = argparse.ArgumentParser()
p.add_argument("--run-dir", required=True)
a = p.parse_args()
run = Path(a.run_dir).resolve()
RuntimeClient(run).verify_ros_domain()
supervisor = int((run/"supervisor.pid").read_text())
args = Path(f"/proc/{supervisor}/cmdline").read_bytes().split(b"\0")
assert b"--base-only" in args, "只允许在 --base-only 模式执行物理探针"
manifest = json.loads((run/"manifest.json").read_text())
import os
assert os.environ.get("IGN_PARTITION") == manifest["environment"]["IGN_PARTITION"]
geometry = yaml.safe_load((run/"assets/ground_truth.yaml").read_text())["obstacles"]
rclpy.init()
node = rclpy.create_node("mixed_shape_acceptance")
scans = {}
subs = [node.create_subscription(LaserScan, f"/{name}/scan",
        lambda msg, name=name: scans.update({name: msg}), qos_profile_sensor_data)
        for name in ["robot1", "robot2", "robot3"]]


def pump(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.03)


def truth(stamp=None):
    rows = [json.loads(row) for row in (run/"truth.jsonl").read_text().splitlines()[-100:]]
    rows = [row for row in rows if len(row["robots"]) == 3]
    return rows[-1] if stamp is None else min(rows, key=lambda row: abs(row["sim_time"]-stamp))


def teleport(point):
    request = f'name: "robot1" position {{x: {point[0]} y: {point[1]} z: .02}} orientation {{w: 1}}'
    result = subprocess.run(["ign", "service", "-s", "/world/fleet_arena/set_pose",
        "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
        "--timeout", "5000", "--req", request], capture_output=True, text=True, check=True, timeout=8)
    assert "data: true" in result.stdout, result.stdout+result.stderr


errors = {kind: [] for kind in {s["kind"] for s in geometry}}
samples = []


def sample(label, name):
    m = scans[name]
    stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    row = truth(stamp)
    pose = row["robots"][name]
    assert abs(row["sim_time"]-stamp) < .2
    yaw = pose[2]
    origin = np.array(pose[:2])+ .00393584*np.array([math.cos(yaw), math.sin(yaw)])
    angles = m.angle_min+np.arange(len(m.ranges))*m.angle_increment+yaw+math.pi/2
    expected, hits = raycast(origin, angles, geometry, m.range_max)
    measured = np.array(m.ranges)
    valid = (hits >= 0) & np.isfinite(measured) & (measured > m.range_min+.02)
    # Exclude other robot bodies, including rays grazing their rendered meshes.
    for other, q in row["robots"].items():
        if other == name:
            continue
        near, _ = raycast(origin, angles,
            [dict(kind="circle", x=q[0], y=q[1], yaw=0, radius=.3)], m.range_max)
        valid &= near >= expected+.1
    error = abs(measured-expected)
    assert valid.sum() >= 100, (label, valid.sum())
    for index in np.flatnonzero(valid):
        errors[geometry[hits[index]]["kind"]].append(float(error[index]))
    record = dict(view=label, robot=name, pose=pose, rays=int(valid.sum()),
                  median_error_m=float(np.median(error[valid])), p95_error_m=float(np.quantile(error[valid], .95)),
                  within_6cm=float(np.mean(error[valid] <= .06)))
    samples.append(record)
    print(json.dumps(record), flush=True)
    assert record["within_6cm"] >= .97, record


try:
    deadline = time.monotonic()+40
    while len(scans) < 3 and time.monotonic() < deadline:
        pump(.1)
    assert len(scans) == 3
    original = truth()["robots"]["robot1"][:2]
    for name in sorted(scans):
        sample("spawn", name)
    bays = []
    for obstacle in geometry[4:]:
        if obstacle["kind"] not in ["concave_polygon", "sector"]:
            continue
        points = vertices(obstacle)
        hull = points[ConvexHull(points).vertices]
        x0,y0,x1,y1 = bounds(obstacle)
        yy,xx = np.mgrid[y0:y1:.03, x0:x1:.03]
        inside = shape_distance(xx, yy, dict(kind="polygon", vertices=hull.tolist(), x=0, y=0, yaw=0)) == 0
        distances = shape_distance(xx, yy, obstacle)
        candidates = inside & (distances >= .45)
        if not candidates.any():
            continue
        index = np.unravel_index(np.argmax(np.where(candidates, distances, -1)), xx.shape)
        point = [float(xx[index]), float(yy[index])]
        assert min(float(shape_distance(*point, s)) for s in geometry) >= .4
        teleport(point)
        pump(1.5)
        before = truth()["robots"]["robot1"][:2]
        pump(1.2)
        after = truth()["robots"]["robot1"][:2]
        displacement = math.dist(before, after)
        assert math.dist(point, after) < .06 and displacement < .025, (point, before, after)
        bays.append(dict(obstacle=obstacle["name"], kind=obstacle["kind"], point=point,
                         stationary_displacement_m=displacement, collision_bay_clear=True))
        sample(obstacle["name"]+"_bay", "robot1")
    assert sum(b["kind"] == "concave_polygon" for b in bays) == 2, bays
    summary = {kind: dict(rays=len(values), median_error_m=float(np.median(values)),
                         p95_error_m=float(np.quantile(values, .95)),
                         within_6cm=float(np.mean(np.array(values) <= .06)))
               for kind, values in errors.items() if values}
    assert set(summary) == {"rectangle", "circle", "convex_polygon", "concave_polygon", "sector"}
    # GPU ray interpolation near a silhouette can differ by an entire occluded
    # depth. Require the raw 95th percentile for each kind; retain all outliers.
    assert all(record["p95_error_m"] <= .06 for record in summary.values()), summary
    report = dict(passed=True, samples=samples, by_kind=summary, concave_bays=bays)
    (run/"mixed-shape-acceptance.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["by_kind"], indent=2))
finally:
    if "original" in globals():
        teleport(original)
        pump(.5)
    node.destroy_node()
    rclpy.try_shutdown()
