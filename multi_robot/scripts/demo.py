#!/usr/bin/env python3
"""Real end-to-end acceptance against a running fleet (./run.sh --record-truth).
Does not inject maps, poses or motion. Uses the same commands as the dashboard.
"""
import argparse, json, time, urllib.request
from pathlib import Path
from runtime_context import RuntimeClient

p = argparse.ArgumentParser()
p.add_argument("--explore-timeout", type=float, default=1200)
p.add_argument("--formation-timeout", type=float, default=500)
p.add_argument("--skip-explore", action="store_true")
p.add_argument("--keep-regions", action="store_true", help="保留前序网页验收已经锁定的默认分区")
p.add_argument("--run-dir")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
run = Path(a.run_dir or (root / "runs/latest.txt").read_text()).resolve()
api = RuntimeClient(run)
log = open(run / "demo.jsonl", "a", buffering=1)


def state():
    return api.state()


def record(event_type, **kw):
    item = {"wall_time": time.time(), "event": event_type, **kw}
    log.write(json.dumps(item, ensure_ascii=False) + "\n")
    print(item, flush=True)


def send(kind, data):
    r = api.request(kind, data)
    record("command", kind=kind, data=data, result=r)
    if not r["success"]:
        raise RuntimeError(r["message"])
    return r


def wait_for(check, timeout):
    end = time.monotonic() + timeout
    last = 0
    while time.monotonic() < end:
        s = state()
        if check(s):
            return s
        if s["mode"] in ["FAULT", "ESTOP"]:
            raise RuntimeError(s["message"])
        if time.monotonic() - last > 15:
            record(
                "progress",
                mode=s["mode"],
                known=s["known_fraction"],
                robots={
                    n: {
                        "pose": r["pose"],
                        "travel": r["travel"],
                        "goal": r["goal"],
                        "safety": r["safety"],
                    }
                    for n, r in s["robots"].items()
                },
            )
            last = time.monotonic()
        time.sleep(0.8)
    raise TimeoutError("操作超时：" + state()["message"])


try:
    wait_for(lambda s: s["ready"], 180)
    record("ready")
    if not a.skip_explore:
        import yaml
        fleet_cfg = yaml.safe_load((run / "assets/fleet.yaml").read_text())
        regions = [r["region"] for r in fleet_cfg["robots"]]
        if a.keep_regions:
            if state()["regions"] != regions:
                raise RuntimeError("前序任务的分区与演示默认分区不一致")
        else:
            send("regions", {"regions": regions})
        send("command", {"command": "explore"})
        # Evaluate saved observations against independent truth; stop only if the
        # required coverage AND every robot's actual participation have passed.
        from fleet_control.evaluate import coverage
        from fleet_mapping.grid import load_map
        import yaml

        fleet_cfg = yaml.safe_load((run / "assets/fleet.yaml").read_text())
        boxes = yaml.safe_load((run / "assets/ground_truth.yaml").read_text())["boxes"]
        end = time.monotonic() + a.explore_timeout
        last = 0
        while time.monotonic() < end:
            s = state()
            if s["mode"] in ["FAULT", "ESTOP"]:
                raise RuntimeError(s["message"])
            if time.monotonic() - last > 25:
                dest = run / "results/coverage_probe"
                send("command", {"command": "save", "argument": str(dest)})
                time.sleep(1.5)
                data, meta = load_map(dest / "shared_map.yaml")
                metrics = coverage(
                    data,
                    meta["resolution"],
                    meta["origin"],
                    boxes,
                    s["regions"],
                    fleet_cfg["robot_radius"],
                    starts=[r["start"] for r in fleet_cfg["robots"]],
                )
                record(
                    "coverage",
                    **metrics,
                    travel={n: r["travel"] for n, r in s["robots"].items()},
                )
                last = time.monotonic()
                if metrics["coverage_pass"] and all(
                    r["travel"] >= 3.2 for r in s["robots"].values()
                ):
                    break
            time.sleep(1.0)
        else:
            raise TimeoutError("探索未达到验收指标")
    send("command", {"command": "save", "argument": str(run / "results/final")})
    for shape in ["triangle", "row", "column"]:
        send("formation", {"shape": shape, "x": 0, "y": 0, "yaw": 0, "spacing": 1.2})
        from fleet_control.geometry import formation as compute_slots

        expected = [list(p) for p in compute_slots(shape, 0, 0, 0, 1.2)]
        s = wait_for(
            lambda s: s["mode"] == "FORMATION_READY"
            and s.get("formation_slots") == expected,
            a.formation_timeout,
        )
        record("formation_ready", shape=shape, errors=s["formation_errors"])
    send("command", {"command": "explore"})
    wait_for(lambda s: s["mode"] == "EXPLORING", a.formation_timeout)
    record("resume_explore")
    send("command", {"command": "pause"})
    time.sleep(2.0)
    send("command", {"command": "save", "argument": str(run / "results/final")})
    time.sleep(4)
    from fleet_control.evaluate import evaluate

    report = evaluate(run, run / "results/final/shared_map.yaml")
    record("complete", report=report)
    if not all(
        report.get(k, False)
        for k in [
            "coverage_pass",
            "exploration_distance_pass",
            "actual_distance_pass",
            "collision_pass",
            "region_pass",
            "three_formations_pass",
            "unknown_encoding_pass",
        ]
    ):
        raise RuntimeError("完整验收有未通过项，请查看 acceptance.json")
except Exception as e:
    record("failed", error=str(e))
    try:
        send("command", {"command": "pause"})
    except Exception:
        pass
    raise
finally:
    log.close()
