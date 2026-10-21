#!/usr/bin/env python3
"""Suspend one actual Nav2 BT process and verify fleet-wide physical stop."""
import json, math, os, signal, time, urllib.request
from pathlib import Path

root = Path(__file__).resolve().parents[1]
run = Path((root / "runs/latest.txt").read_text())
group = int((run / "launch.pid").read_text())


def state():
    return json.load(
        urllib.request.urlopen("http://127.0.0.1:8765/api/state", timeout=3)
    )


def command(cmd):
    request = urllib.request.Request(
        "http://127.0.0.1:8765/api/command",
        json.dumps({"command": cmd}).encode(),
        {"Content-Type": "application/json"},
        method="POST",
    )
    return json.load(urllib.request.urlopen(request, timeout=10))


def truth():
    return json.loads((run / "truth.jsonl").read_text().splitlines()[-1])["robots"]


pid = None
report = {}
try:
    assert command("explore")["success"]
    end = time.monotonic() + 50
    while time.monotonic() < end:
        s = state()
        names = [
            n for n, r in s["robots"].items() if abs(r["safety"].get("v", 0)) > 0.04
        ]
        if s["mode"] == "EXPLORING" and names:
            break
        time.sleep(0.2)
    else:
        raise RuntimeError("未进入实际探索运动")
    name = names[0]
    matches = []
    for p in Path("/proc").glob("[0-9]*"):
        try:
            if os.getpgid(int(p.name)) != group:
                continue
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode()
            if (
                cmd.startswith("/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator")
                and f"__ns:=/{name}" in cmd
            ):
                matches.append(int(p.name))
        except (OSError, ValueError):
            pass
    assert len(matches) == 1, matches
    pid = matches[0]
    start = time.monotonic()
    os.kill(pid, signal.SIGSTOP)
    end = start + 6
    while time.monotonic() < end and state()["mode"] != "FAULT":
        time.sleep(0.1)
    assert state()["mode"] == "FAULT", state()["message"]
    latency = time.monotonic() - start
    reason = state()["message"]
    time.sleep(0.7)
    before = truth()
    time.sleep(0.8)
    after = truth()
    delta = {n: math.dist(p[:2], after[n][:2]) for n, p in before.items()}
    assert max(delta.values()) < 0.025, delta
    report = {
        "passed": True,
        "robot": name,
        "node": "bt_navigator",
        "fault_latency_seconds": latency,
        "reason": reason,
        "stopped_displacement": delta,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
finally:
    if pid:
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
    try:
        command("pause")
    except Exception:
        pass
    (run / "node_failure_acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
