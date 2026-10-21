#!/usr/bin/env python3
"""Two final cold runs after a full one-click demo; optionally exercise node loss."""
import argparse, json, os, signal, subprocess, sys, time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--count", type=int, default=2)
p.add_argument("--node-fault", action="store_true")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
reports = []
for i in range(a.count):
    run = root / "runs" / time.strftime(f"cold-final-{i+1}-%Y%m%d-%H%M%S")
    child = subprocess.Popen(
        [str(root / "run.sh"), "--headless", "--record-truth", "--run-dir", str(run)],
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
            [sys.executable, str(root / "scripts/cold_smoke.py")], cwd=root, check=True
        )
        if i == 0 and a.node_fault:
            subprocess.run(
                [sys.executable, str(root / "scripts/node_failure_acceptance.py")],
                cwd=root,
                check=True,
            )
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=35)
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
    reports.append(
        {
            "run": str(run),
            "cold_start_pass": json.loads((run / "cold_smoke.json").read_text())[
                "passed"
            ],
            "cleanup_pass": not remaining,
            "supervisor_exit": child.returncode,
        }
    )
    print(reports[-1], flush=True)
(root / "docs/final-cold-starts.json").write_text(
    json.dumps(reports, ensure_ascii=False, indent=2)
)
print("最终冷启动与清理检查通过", flush=True)
