#!/usr/bin/env python3
"""Readiness must remain true for 30 seconds, not merely one startup sample."""
import json, time, urllib.request
from pathlib import Path

root = Path(__file__).resolve().parents[1]
run = Path((root / "runs/latest.txt").read_text())
end = time.monotonic() + 150
stable = None
samples = []
while time.monotonic() < end:
    try:
        s = json.load(
            urllib.request.urlopen("http://127.0.0.1:8765/api/state", timeout=3)
        )
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
