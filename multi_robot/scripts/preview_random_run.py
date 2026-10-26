#!/usr/bin/env python3
"""Exercise the real preview through its ordinary API and retain observations."""
import argparse
import json
from pathlib import Path
import time

from runtime_context import RuntimeClient
from fleet_control.evaluate import evaluate

p = argparse.ArgumentParser()
p.add_argument("--run-dir", required=True)
p.add_argument("--seconds", type=float, default=240)
p.add_argument("--require-release", action="store_true")
a = p.parse_args()
if not 10 <= a.seconds <= 1800:
    p.error("seconds must be in [10, 1800]")
run = Path(a.run_dir).resolve()
api = RuntimeClient(run)
state = api.wait_ready()
result = api.request("command", {"command": "explore"})
if not result["success"]:
    raise RuntimeError(result["message"])
samples = []
deadline = time.monotonic() + a.seconds
next_print = 0
try:
    while time.monotonic() < deadline:
        state = api.state()
        if state["mode"] in ["FAULT", "ESTOP"]:
            raise RuntimeError(state["message"])
        if time.monotonic() >= next_print:
            sample = {k: state[k] for k in ["mode", "known_fraction", "region_enabled", "exploration_unrestricted", "region_release_reason"]}
            sample["wall_time"] = time.time()
            sample["travel"] = {n: r["travel"] for n, r in state["robots"].items()}
            samples.append(sample)
            print(json.dumps(sample, ensure_ascii=False), flush=True)
            next_print = time.monotonic() + 20
        if state["mode"] == "EXPLORED":
            break
        time.sleep(1)
finally:
    print(api.request("command", {"command": "pause"}), flush=True)
    dest = run / "results/final"
    saved = api.request("command", {"command": "save", "argument": str(dest)})
    if not saved["success"]:
        raise RuntimeError(saved["message"])
    (run / "preview-samples.json").write_text(json.dumps(samples, ensure_ascii=False, indent=2))
report = evaluate(run, dest / "shared_map.yaml")
summary = {k: report.get(k) for k in ["coverage", "region_coverage", "actual_explore_distance", "minimum_static_clearance", "minimum_interrobot_clearance", "collision_pass", "region_pass", "authorized_cross_region_samples", "exploration_region_releases"]}
(run / "preview-summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
print(json.dumps(summary, ensure_ascii=False), flush=True)
if not summary["collision_pass"] or not summary["region_pass"]:
    raise RuntimeError("碰撞或分区保护检查未通过")
if a.require_release and not report.get("exploration_region_releases"):
    raise RuntimeError("本轮没有触发自动跨区")
