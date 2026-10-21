#!/usr/bin/env python3
"""Own and reap only this invocation's process group; preserve other simulators."""
import argparse, datetime, fcntl, hashlib, json, os, signal, subprocess, sys, time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--headless", action="store_true")
p.add_argument("--base-only", action="store_true")
p.add_argument("--record-truth", action="store_true")
p.add_argument("--config")
p.add_argument("--run-dir")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
lock = open(f'/tmp/turtlebot_fleet_{os.environ["ROS_DOMAIN_ID"]}.lock', "w")
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit("本项目已在此 ROS 域运行。请先在原启动终端按 Ctrl+C。")
run = (
    Path(a.run_dir).resolve()
    if a.run_dir
    else root / "runs" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
)
if run.exists() and any(run.iterdir()):
    sys.exit("运行目录已存在且非空，请使用新目录，以免覆盖历史证据。")
run.mkdir(parents=True, exist_ok=True)
sources = []
for folder in [root / "src", root / "scripts", root / "vendor/tf2_compat"]:
    sources.extend(
        p
        for p in folder.rglob("*")
        if p.is_file()
        and p.suffix
        in [".py", ".sh", ".yaml", ".lua", ".config", ".srv", ".cpp", ".hpp"]
    )
sources.extend(root / name for name in ["env.sh", "build.sh", "run.sh", "demo.sh"])
manifest = {
    "source_sha256": {
        str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(sources)
    },
    "environment": {
        key: os.environ.get(key)
        for key in [
            "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION",
            "IGN_PARTITION",
            "PYTHONNOUSERSITE",
        ]
    },
    "memory_total_kb": int(
        Path("/proc/meminfo").read_text().splitlines()[0].split()[1]
    ),
    "logical_cpus": os.cpu_count(),
    "kernel": os.uname().release,
}
(run / "manifest.json").write_text(json.dumps(manifest, indent=2))
tool = None
if a.record_truth:
    tool = root / "build/tools/truth_probe"
    if not tool.exists():
        subprocess.run([str(root / "scripts/build_tools.sh")], check=True)
(root / "runs/latest.txt").write_text(str(run))
cmd = [
    "ros2",
    "launch",
    "fleet_bringup",
    "fleet.launch.py",
    f"gui:={str(not a.headless).lower()}",
    f"assets:={run}/assets",
    f'mode:={"base" if a.base_only else "full"}',
]
if a.config:
    cmd.append(f"config:={Path(a.config).resolve()}")
log = open(run / "launch.log", "w")
proc = subprocess.Popen(
    cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
)
(run / "supervisor.pid").write_text(str(os.getpid()))
(run / "launch.pid").write_text(str(proc.pid))
print(
    f"运行目录：{run}\n日志：{run}/launch.log\n网页：http://127.0.0.1:8765\n在此终端按 Ctrl+C 完整关闭本次仿真。",
    flush=True,
)
sampler = subprocess.Popen(
    [sys.executable, str(root / "scripts/resources.py"), str(run)],
    start_new_session=True,
)
probe = None
if a.record_truth:
    probe = subprocess.Popen(
        [str(tool), str(run / "truth.jsonl")], start_new_session=True
    )
stop = False


def shutdown(sig, frame):
    global stop
    stop = True


for s in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
    signal.signal(s, shutdown)
try:
    while proc.poll() is None and not stop:
        time.sleep(0.25)
finally:
    if proc.poll() is None:
        for sig, timeout in [
            (signal.SIGINT, 18),
            (signal.SIGTERM, 5),
            (signal.SIGKILL, 3),
        ]:
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                pass
    # Reap descendants even if ros2 launch exited early.
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    # The launch parent may have exited before a stubborn native child.
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        alive = []
        for entry in Path("/proc").glob("[0-9]*"):
            try:
                if (
                    os.getpgid(int(entry.name)) == proc.pid
                    and (entry / "stat").read_text().split(") ", 1)[1][0] != "Z"
                ):
                    alive.append(int(entry.name))
            except (OSError, ValueError):
                pass
        if not alive:
            break
        time.sleep(0.1)
    if alive:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    (run / "cleanup.json").write_text(
        json.dumps(
            {"residual_before_final_kill": alive, "launch_exit": proc.returncode}
        )
    )
    sampler.send_signal(signal.SIGTERM)
    try:
        sampler.wait(timeout=4)
    except subprocess.TimeoutExpired:
        sampler.kill()
        sampler.wait()
    if probe:
        probe.send_signal(signal.SIGINT)
        try:
            probe.wait(timeout=4)
        except subprocess.TimeoutExpired:
            probe.kill()
            probe.wait()
    log.close()
sys.exit(proc.returncode if not stop else 0)
