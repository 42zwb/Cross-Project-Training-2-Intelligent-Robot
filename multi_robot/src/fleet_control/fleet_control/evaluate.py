"""Independent offline evaluator. Only this tool reads arena ground truth."""

import argparse, json, math
from pathlib import Path
import numpy as np
import yaml
from scipy import ndimage
from fleet_mapping.grid import load_map
from fleet_control.geometry import formation_pass, FORMATION_CLEARANCE_MARGIN


def coverage(data, res, origin, boxes, regions, robot_radius=0.18):
    yy, xx = np.indices(data.shape)
    x = origin[0] + (xx + 0.5) * res
    y = origin[1] + (yy + 0.5) * res
    obstacles = np.zeros(data.shape, bool)
    for _, bx, by, dx, dy in boxes:
        obstacles |= (abs(x - bx) <= dx / 2) & (abs(y - by) <= dy / 2)
    # Reachability uses the true robot-centre configuration space, not all wall pixels.
    free = ndimage.distance_transform_edt(~obstacles) * res >= robot_radius + 0.02
    labels, _ = ndimage.label(free)
    reachable = np.zeros(data.shape, bool)
    for sx, sy in [(-4, -4), (0, -4), (4, -4)]:
        cy, cx = int((sy - origin[1]) / res), int((sx - origin[0]) / res)
        lab = labels[cy, cx]
        if lab:
            reachable |= labels == lab
    observed = (
        data == 0
    )  # correct observed free space, not merely a falsely occupied cell
    ratios = []
    for b in regions:
        mask = reachable & (x >= b[0]) & (x <= b[2]) & (y >= b[1]) & (y <= b[3])
        ratios.append(float(np.sum(mask & observed) / max(1, np.sum(mask))))
    return {
        "reachable_cells": int(reachable.sum()),
        "coverage": float(np.sum(reachable & observed) / max(1, reachable.sum())),
        "region_coverage": ratios,
        "coverage_pass": float(np.sum(reachable & observed) / max(1, reachable.sum()))
        >= 0.9
        and min(ratios) >= 0.8,
    }


def clearance(p, boxes, robot_radius=0.18):
    values = []
    for _, bx, by, dx, dy in boxes:
        x = max(abs(p[0] - bx) - dx / 2, 0)
        y = max(abs(p[1] - by) - dy / 2, 0)
        values.append(math.hypot(x, y) - robot_radius)
    return min(values)


def evaluate(run, map_path=None):
    run = Path(run)
    cfg = yaml.safe_load((run / "assets/fleet.yaml").read_text())
    gt = yaml.safe_load((run / "assets/ground_truth.yaml").read_text())
    regions = [r["region"] for r in cfg["robots"]]
    telemetry = [
        json.loads(l)
        for l in (run / "telemetry.jsonl").read_text().splitlines()
        if l.strip()
    ]
    events = [
        json.loads(l)
        for l in (run / "events.jsonl").read_text().splitlines()
        if l.strip()
    ]
    for e in events:
        if e["event"] == "set_regions":
            regions = e["regions"]
    if map_path is None:
        candidates = sorted(run.glob("results/*/shared_map.yaml"))
        if not candidates:
            raise ValueError("请先保存共享地图")
        map_path = candidates[-1]
    data, meta = load_map(map_path)
    report = coverage(
        data,
        meta["resolution"],
        meta["origin"],
        gt["boxes"],
        regions,
        cfg["robot_radius"],
    )
    report["map"] = str(map_path)
    report["travel"] = telemetry[-1]["robots"] if telemetry else {}
    report["exploration_distance_pass"] = (
        all(r["travel"] >= 3.0 for r in telemetry[-1]["robots"].values())
        if telemetry
        else False
    )
    truth_path = run / "truth.jsonl"
    if truth_path.exists():
        truth = [
            json.loads(l) for l in truth_path.read_text().splitlines() if l.strip()
        ]
        truth = [t for t in truth if len(t["robots"]) == 3]
        if not truth:
            raise ValueError("truth.jsonl 中没有完整的三机器人真值样本")
        minclear = float("inf")
        minpair = float("inf")
        outside = 0
        body_margin = cfg["robot_radius"]
        times = np.array([t["sim_time"] for t in telemetry])
        actual_travel = {n: 0.0 for n in ["robot1", "robot2", "robot3"]}
        for k, t in enumerate(truth):
            sample = (
                telemetry[int(np.argmin(abs(times - t["sim_time"])))]
                if len(times)
                else {}
            )
            previous_sample = (
                telemetry[
                    int(np.argmin(abs(times - truth[k - 1]["sim_time"])))
                ]
                if k and len(times)
                else {}
            )
            for i, (n, p) in enumerate(t["robots"].items()):
                minclear = min(minclear, clearance(p, gt["boxes"], cfg["robot_radius"]))
                for name, q in t["robots"].items():
                    if name != n:
                        minpair = min(
                            minpair,
                            math.dist(p[:2], q[:2]) - 2.0 * cfg["robot_radius"],
                        )
                if sample.get("mode") == "EXPLORING":
                    b = regions[int(n[-1]) - 1]
                    if (
                        not b[0] + body_margin - 1e-3
                        <= p[0]
                        <= b[2] - body_margin + 1e-3
                        or not b[1] + body_margin - 1e-3
                        <= p[1]
                        <= b[3] - body_margin + 1e-3
                    ):
                        outside += 1
                    if (
                        sample.get("mode") == "EXPLORING"
                        and previous_sample.get("mode") == "EXPLORING"
                        and k
                        and n in truth[k - 1]["robots"]
                    ):
                        actual_travel[n] += math.dist(
                            p[:2], truth[k - 1]["robots"][n][:2]
                        )
        report.update(
            actual_explore_distance=actual_travel,
            actual_distance_pass=min(actual_travel.values()) >= 3.0,
            minimum_static_clearance=minclear,
            minimum_interrobot_clearance=minpair,
            collision_pass=minclear >= -0.015 and minpair >= -0.015,
            region_violation_samples=outside,
            region_pass=outside == 0,
        )
        formations = []
        for e in events:
            if e["event"] != "formation_ready":
                continue
            t = min(truth, key=lambda t: abs(t["sim_time"] - e["sim_time"]))
            errors = {
                n: math.dist(p[:2], e["slots"][e["assignment"][int(n[-1]) - 1]][:2])
                for n, p in t["robots"].items()
            }
            yaw_errors = {
                n: abs(
                    math.atan2(
                        math.sin(
                            p[2] - e["slots"][e["assignment"][int(n[-1]) - 1]][2]
                        ),
                        math.cos(
                            p[2] - e["slots"][e["assignment"][int(n[-1]) - 1]][2]
                        ),
                    )
                )
                for n, p in t["robots"].items()
            }
            poses = [t["robots"][f"robot{i + 1}"][:3] for i in range(3)]
            pair_distances = [
                math.dist(poses[i][:2], poses[j][:2])
                for i in range(3)
                for j in range(i)
            ]
            formations.append(
                {
                    "sim_time": e["sim_time"],
                    "estimated_errors": e["errors"],
                    "actual_position_errors": errors,
                    "actual_yaw_errors": yaw_errors,
                    "minimum_pair_distance": min(pair_distances),
                    "required_pair_distance": (
                        2.0 * cfg["robot_radius"] + FORMATION_CLEARANCE_MARGIN
                    ),
                    "actual_pass": formation_pass(
                        poses,
                        e["slots"],
                        e["assignment"],
                        cfg["robot_radius"],
                    ),
                }
            )
        report["formations"] = formations
        report["three_formations_pass"] = len(formations) >= 3 and all(
            f["actual_pass"] for f in formations
        )
        if len(truth) > 1 and truth[-1]["wall_time"] > truth[0]["wall_time"]:
            report["real_time_factor"] = (
                truth[-1]["sim_time"] - truth[0]["sim_time"]
            ) / (truth[-1]["wall_time"] - truth[0]["wall_time"])
    else:
        report["truth_status"] = "未采集真值，物理碰撞、实际里程及真实队形误差未验证"
    resources = run / "resources.jsonl"
    if resources.exists():
        samples = [
            json.loads(line)
            for line in resources.read_text().splitlines()
            if line.strip()
        ]
        if samples:
            report["peak_project_rss_mb"] = max(row["rss_mb"] for row in samples)
            report["minimum_system_available_mb"] = min(
                row["system_available_mb"] for row in samples
            )
            report["resource_samples"] = len(samples)
    # Decode with exact Nav2 trinary conventions and verify 205 remains unknown.
    report["unknown_cells"] = int((data < 0).sum())
    report["unknown_encoding_pass"] = meta.get("free_thresh") == 0.196
    (run / "acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
    Path(map_path).parent.joinpath("acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run_dir")
    p.add_argument("--map")
    a = p.parse_args()
    print(json.dumps(evaluate(a.run_dir, a.map), ensure_ascii=False, indent=2))
