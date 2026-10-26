#!/usr/bin/env python3
"""Two final cold runs after a full one-click demo; optionally exercise node loss."""
import argparse, json, os, signal, subprocess, sys, time
from pathlib import Path
from runtime_context import RuntimeClient

p = argparse.ArgumentParser()
p.add_argument("--count", type=int, default=2)
p.add_argument("--node-fault", action="store_true")
p.add_argument("--planner", choices=["navfn", "smac2d"], default="navfn")
p.add_argument("--move-before-stop", action="store_true")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
reports = []
for i in range(a.count):
    run = root / "runs" / time.strftime(f"cold-final-{i+1}-%Y%m%d-%H%M%S")
    child = subprocess.Popen(
        [str(root / "run.sh"), "--headless", "--record-truth", "--run-dir", str(run), "--planner", a.planner],
        cwd=root,
        start_new_session=True,
    )
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end and not (run / "launch.pid").exists():
            if child.poll() is not None:
                raise RuntimeError("冷启动进程提前退出")
            time.sleep(0.1)
        if not (run / "launch.pid").exists():
            raise RuntimeError("启动管理器未创建本次运行目录")
        subprocess.run(
            [sys.executable, str(root / "scripts/cold_smoke.py"), "--run-dir", str(run)], cwd=root, check=True
        )
        if i == 0 and a.node_fault:
            subprocess.run(
                [sys.executable, str(root / "scripts/node_failure_acceptance.py"), "--run-dir", str(run)],
                cwd=root,
                check=True,
            )
        if a.move_before_stop:
            api = RuntimeClient(run)
            end = time.monotonic() + 20
            while time.monotonic() < end and not api.state()["ready"]:
                time.sleep(0.2)
            assert api.request("command", {"command":"explore"})["success"]
            while time.monotonic() < end:
                if any(abs(r["safety"].get("v", 0)) > 0.02 for r in api.state()["robots"].values()):
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError("退出复验未观察到真实探索运动")
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=55)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    group = int((run / "launch.pid").read_text())
    remaining = []
    for entry in Path("/proc").glob("[0-9]*"):
        try:
            if (
                os.getpgid(int(entry.name)) == group
                and (entry / "stat").read_text().split(") ", 1)[1][0] != "Z"
            ):
                remaining.append(int(entry.name))
        except (OSError, ValueError):
            pass
    assert not remaining, remaining
    cleanup = json.loads((run / "cleanup.json").read_text())
    assert child.returncode == 0 and not cleanup.get("abnormal_process_exits"), cleanup
    shutdown = json.loads((run / "shutdown_preflight.json").read_text())
    assert shutdown.get("simulator_stopped") and len(shutdown.get("simulator_pids", [])) == 1, shutdown
    reports.append(
        {
            "run": str(run),
            "cold_start_pass": json.loads((run / "cold_smoke.json").read_text())[
                "passed"
            ],
            "cleanup_pass": not remaining,
            "supervisor_exit": child.returncode,
            "shutdown": shutdown,
        }
    )
    print(reports[-1], flush=True)
(root / "docs/final-cold-starts.json").write_text(
    json.dumps(reports, ensure_ascii=False, indent=2)
)
print("最终冷启动与清理检查通过", flush=True)
