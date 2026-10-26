#!/usr/bin/env python3
"""Own and reap only this invocation's process group; preserve other simulators."""
import argparse, datetime, fcntl, hashlib, json, os, signal, subprocess, sys, time
import yaml
import socket
from pathlib import Path
from shutdown_context import quiesce

p = argparse.ArgumentParser()
p.add_argument("--headless", action="store_true")
p.add_argument("--base-only", action="store_true")
p.add_argument("--record-truth", action="store_true")
p.add_argument("--config")
p.add_argument("--run-dir")
p.add_argument("--planner", choices=["navfn", "smac2d"], help="覆盖配置中的全局规划器")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
lock = open(f'/tmp/turtlebot_fleet_{os.environ["ROS_DOMAIN_ID"]}.lock', "w")
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit("本项目已在此 ROS 域运行。请先在原启动终端按 Ctrl+C。")
partition_key = hashlib.sha256(os.environ["IGN_PARTITION"].encode()).hexdigest()[:16]
partition_lock = open(f"/tmp/turtlebot_fleet_ign_{partition_key}.lock", "w")
try:
    fcntl.flock(partition_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit("本项目已在此 Gazebo 分区运行；多开时还须设置独立的 FLEET_IGN_PARTITION。")
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
        in [".py", ".sh", ".yaml", ".lua", ".config", ".srv", ".cpp", ".hpp", ".html", ".js", ".css", ".xml"]
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
cmd = [
    "ros2",
    "launch",
    "fleet_bringup",
    "fleet.launch.py",
    f"gui:={str(not a.headless).lower()}",
    f"assets:={run}/assets",
    f'mode:={"base" if a.base_only else "full"}',
]
config = Path(a.config).expanduser().resolve() if a.config else root / "src/fleet_bringup/config/fleet.yaml"
cfg = yaml.safe_load(config.read_text())
if a.planner:
    cfg["planner"] = a.planner
from fleet_bringup.prepare import validate_config
validate_config(cfg)
if not a.base_only:
    with socket.socket() as probe_socket:
        probe_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe_socket.bind(("127.0.0.1", cfg["dashboard_port"]))
        except OSError:
            sys.exit(f"网页端口 {cfg['dashboard_port']} 已被占用，请关闭原实例或更改 dashboard_port。")
effective_config = run / "requested_fleet.yaml"
effective_config.write_text(yaml.safe_dump(cfg))
cmd.append(f"config:={effective_config}")
log = open(run / "launch.log", "w")
proc = subprocess.Popen(
    cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
)
(root / "runs/latest.txt").write_text(str(run))
(run / "supervisor.pid").write_text(str(os.getpid()))
(run / "launch.pid").write_text(str(proc.pid))
print(
    f"运行目录：{run}\n日志：{run}/launch.log\n网页：http://127.0.0.1:{cfg['dashboard_port']}\n规划器：{cfg['planner']}\n在此终端按 Ctrl+C 完整关闭本次仿真。",
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
shutdown_failed = False


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
        try:
            shutdown_report = quiesce(run, process_group=proc.pid, base_only=a.base_only)
        except Exception as error:
            shutdown_report = {"error": str(error)}
        shutdown_failed = not shutdown_report.get("simulator_stopped", False)
        try:
            (run / "shutdown_preflight.json").write_text(json.dumps(shutdown_report, ensure_ascii=False, indent=2))
        except OSError as error:
            print(f"无法记录退出准备结果：{error}", file=sys.stderr)
        for sig, timeout, process_group in [
            # Let ros2 launch own the first shutdown wave. Signalling the whole
            # group here reaches launch's children directly, then launch sends
            # them SIGINT again and interrupts their cleanup handlers.
            (signal.SIGINT, 18, False),
            (signal.SIGTERM, 5, False),
            (signal.SIGKILL, 3, True),
        ]:
            try:
                if process_group:
                    os.killpg(proc.pid, sig)
                else:
                    proc.send_signal(sig)
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
    log.flush()
    abnormal_exits = [line for line in (run / "launch.log").read_text(errors="replace").splitlines()
                      if "process has died" in line]
    (run / "cleanup.json").write_text(
        json.dumps(
            {"residual_before_final_kill": alive, "launch_exit": proc.returncode,
             "abnormal_process_exits": abnormal_exits}
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
sys.exit(1 if abnormal_exits or alive or shutdown_failed else proc.returncode if not stop else 0)
