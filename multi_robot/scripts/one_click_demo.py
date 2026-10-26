#!/usr/bin/env python3
"""Own a complete demo invocation, including its backend and cleanup."""
import argparse, datetime, json, signal, subprocess, sys, time, urllib.request
from pathlib import Path
from runtime_context import RuntimeClient

p = argparse.ArgumentParser()
p.add_argument("--headless", action="store_true")
p.add_argument("--with-faults", action="store_true")
p.add_argument("--seed", type=int)
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
run = root / "runs" / datetime.datetime.now().strftime("demo-%Y%m%d-%H%M%S")
args = [str(root / "run.sh"), "--record-truth", "--run-dir", str(run)]
if a.seed is not None:
    args += ["--seed", str(a.seed)]
if a.headless:
    args.append("--headless")
backend = subprocess.Popen(args, start_new_session=True)
api = RuntimeClient(run)
completed = False
try:
    end = time.monotonic() + 180
    while time.monotonic() < end:
        if backend.poll() is not None:
            raise RuntimeError("仿真启动失败或此域已有运行实例；未接管其他会话。")
        try:
            s = api.state(timeout=2)
            if s.get("ready"):
                break
        except (OSError, ValueError, RuntimeError):
            pass
        time.sleep(0.5)
    else:
        raise RuntimeError("启动就绪超时")
    subprocess.run(
        [sys.executable, str(root / "scripts/demo.py"), "--run-dir", str(run)], cwd=root, check=True
    )
    subprocess.run(
        [
            sys.executable,
            str(root / "scripts/map_roundtrip.py"),
            str(run / "results/final/shared_map.yaml"),
        ],
        cwd=root,
        check=True,
    )
    if a.with_faults:
        subprocess.run(
            [sys.executable, str(root / "scripts/safety_acceptance.py"), "--run-dir", str(run)],
            cwd=root,
            check=True,
        )
    print("演示及验收完成。结果：" + str(run), flush=True)
    completed = True
finally:
    if backend.poll() is None:
        backend.send_signal(signal.SIGINT)
        try:
            backend.wait(timeout=55)
        except subprocess.TimeoutExpired:
            backend.kill()
            backend.wait()
    if completed and backend.returncode != 0:
        raise RuntimeError("仿真后台退出异常，请查看 cleanup.json 和 launch.log")
