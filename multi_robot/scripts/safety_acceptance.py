#!/usr/bin/env python3
"""Real fault injection. Run after the normal demo, while fleet is IDLE.
Only stops/resumes PIDs belonging to this invocation's recorded process group.
"""
import argparse, json, math, os, signal, time, urllib.request
from pathlib import Path
from runtime_context import RuntimeClient

root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument("--run-dir")
a = p.parse_args()
run = Path(a.run_dir or (root / "runs/latest.txt").read_text()).resolve()
api = RuntimeClient(run)
group = int((run / "launch.pid").read_text())
report = {}


def state():
    return api.state(timeout=4)


def send(kind, data):
    return api.request(kind, data, timeout=15)


def command(cmd):
    r = send("command", {"command": cmd})
    assert r["success"], r
    return r


def wait(check, timeout=40):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = state()
        if check(s):
            return s
        time.sleep(0.15)
    raise RuntimeError("安全测试等待超时：" + state()["message"])


def actual():
    lines = (run / "truth.jsonl").read_text().splitlines()
    return json.loads(lines[-1])["robots"]


def prove_stop():
    time.sleep(1.0)
    before = actual()
    time.sleep(1.0)
    after = actual()
    d = {n: math.dist(p[:2], after[n][:2]) for n, p in before.items()}
    assert max(d.values()) < 0.025, d
    return d


def moving():
    r = send(
        "formation", {"shape": "triangle", "x": 0, "y": 0, "yaw": 0, "spacing": 1.2}
    )
    assert r["success"], r
    return wait(
        lambda s: s["mode"] == "FORMING"
        and any(abs(r["safety"].get("v", 0)) > 0.04 for r in s["robots"].values()),
        90,
    )


def find_pid(test):
    found = []
    for path in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(path.name)
            if os.getpgid(pid) != group:
                continue
            cmd = (path / "cmdline").read_bytes().replace(b"\0", b" ").decode()
            if test(cmd):
                found.append(pid)
        except (OSError, ValueError):
            pass
    if len(found) != 1:
        raise RuntimeError("无法唯一确定故障注入进程：" + str(found))
    return found[0]


resumed = []
try:
    assert state()["ready"], "系统尚未就绪"
    # Invalid target must be rejected without cancelling the current safe task.
    before = state()["mode"]
    r = send(
        "formation", {"shape": "triangle", "x": 20, "y": 20, "yaw": 0, "spacing": 1.2}
    )
    assert not r["success"]
    report["invalid_target_rejected"] = r
    moving()
    command("estop")
    wait(lambda s: s["mode"] == "ESTOP")
    report["estop_stop_displacement"] = prove_stop()
    command("clear-estop")
    wait(lambda s: s["mode"] == "IDLE")
    time.sleep(1.0)
    assert state()["mode"] == "IDLE"
    report["clear_estop_does_not_resume"] = True
    # Select an actively moving robot, then suspend its actual bridge.
    s = moving()
    name = next(
        n for n, r in s["robots"].items() if abs(r["safety"].get("v", 0)) > 0.04
    )
    pid = find_pid(
        lambda c: any(path in c for path in ["/fleet_bridge/parameter_bridge", "/ros_gz_bridge/parameter_bridge"])
        and f"__ns:=/{name}" in c
    )
    os.kill(pid, signal.SIGSTOP)
    resumed.append(pid)
    wait(lambda s: s["mode"] == "FAULT", 8)
    report["lidar_loss"] = {
        "robot": name,
        "reason": state()["message"],
        "stop_displacement": prove_stop(),
    }
    os.kill(pid, signal.SIGCONT)
    resumed.remove(pid)
    wait(lambda s: s["ready"], 20)
    command("pause")
    # Suspend the mapper for one moving robot: SLAM map->odom becomes stale.
    s = moving()
    name = next(
        n for n, r in s["robots"].items() if abs(r["safety"].get("v", 0)) > 0.04
    )
    pid = find_pid(
        lambda c: "/cartographer_ros/cartographer_node " in c and f"__ns:=/{name}" in c
    )
    os.kill(pid, signal.SIGSTOP)
    resumed.append(pid)
    wait(lambda s: s["mode"] == "FAULT", 8)
    report["localization_loss"] = {
        "robot": name,
        "reason": state()["message"],
        "stop_displacement": prove_stop(),
    }
    os.kill(pid, signal.SIGCONT)
    resumed.remove(pid)
    wait(lambda s: s["ready"], 25)
    command("pause")
    report["passed"] = True
    print(json.dumps(report, ensure_ascii=False, indent=2))
finally:
    for pid in resumed:
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
    try:
        command("pause")
    except Exception:
        pass
    (run / "safety_acceptance.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2)
    )
