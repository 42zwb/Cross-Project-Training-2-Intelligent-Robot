#!/usr/bin/env python3
"""Readiness must remain true for 30 seconds, not merely one startup sample."""
import argparse, json, time, urllib.request
from pathlib import Path
from runtime_context import RuntimeClient

root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument("--run-dir")
a = p.parse_args()
run = Path(a.run_dir or (root / "runs/latest.txt").read_text()).resolve()
api = RuntimeClient(run)
end = time.monotonic() + 150
stable = None
samples = []
while time.monotonic() < end:
    try:
        s = api.state(timeout=3)
    except Exception:
        time.sleep(0.5)
        continue
    samples.append(
        {"wall_time": time.time(), "ready": s.get("ready"), "mode": s.get("mode")}
    )
    if s.get("ready"):
        if stable is None:
            stable = time.monotonic()
        if time.monotonic() - stable >= 30:
            (run / "cold_smoke.json").write_text(
                json.dumps(
                    {"passed": True, "stable_seconds": 30, "samples": samples}, indent=2
                )
            )
            print("冷启动检查通过：" + str(run))
            break
    else:
        stable = None
    time.sleep(0.5)
else:
    (run / "cold_smoke.json").write_text(
        json.dumps({"passed": False, "samples": samples}, indent=2)
    )
    raise RuntimeError("未能保持 30 秒就绪，请查看导航位姿与启动日志")
