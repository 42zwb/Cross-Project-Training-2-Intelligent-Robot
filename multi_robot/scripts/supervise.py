#!/usr/bin/env python3
"""Clean previous project simulations before launch; supervise this new session."""
import argparse, datetime, fcntl, hashlib, json, os, signal, subprocess, sys, time
import yaml
import socket
import secrets
from pathlib import Path
from shutdown_context import quiesce
from runtime_context import reproduction_command
from fleet_bringup.simulation_preflight import begin_session
from ament_index_python.packages import get_package_prefix

p = argparse.ArgumentParser()
display = p.add_mutually_exclusive_group()
display.add_argument('--gui', action='store_true', help='打开 Gazebo 三维窗口')
display.add_argument('--headless', action='store_true', help='仅运行仿真后台和网页（默认）')
p.add_argument("--base-only", action="store_true")
p.add_argument("--record-truth", action="store_true")
p.add_argument('--architecture', choices=['centralized', 'decentralized'],
               help='三车控制架构，默认 decentralized')
p.add_argument('--no-dashboard', action='store_true')
p.add_argument('--spawn-layout', type=int, choices=[0, 1, 2], default=0)
p.add_argument('--acceptance-scene', choices=['separated-rooms', 'symmetric-square'],
               help='仅供去中心化负例验收，替换仿真几何；不传给机器人')
p.add_argument("--config")
p.add_argument("--run-dir")
p.add_argument("--planner", choices=["navfn", "smac2d"], help="覆盖配置中的全局规划器")
p.add_argument("--seed", type=int, help="覆盖场景种子，内置配置使用 24")
p.add_argument("--scene-generator", choices=["mixed-shapes-v2", "rectangles-v1"], help="选择混合形状或旧矩形生成器")
a = p.parse_args()
architecture_explicit = a.architecture is not None
a.architecture = a.architecture or 'decentralized'
if a.no_dashboard and a.architecture != 'decentralized':
    p.error('--no-dashboard 仅适用于去中心化模式')
if a.acceptance_scene and a.architecture != 'decentralized':
    p.error('--acceptance-scene 仅适用于去中心化负例验收')
root = Path(__file__).resolve().parents[1]
run = (
    Path(a.run_dir).resolve()
    if a.run_dir
    else root / "runs" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
)
if run.exists() and any(run.iterdir()):
    sys.exit("运行目录已存在且非空，请使用新目录，以免覆盖历史证据。")
run.mkdir(parents=True, exist_ok=True)
# Before domain/partition locks and dashboard-port checks: old supervisors and
# orphan nodes must release their resources before we test the new environment.
begin_session('fleet', run / 'startup_preflight.json')
lock = open(f'/tmp/turtlebot_fleet_{os.environ["ROS_DOMAIN_ID"]}.lock', "w")
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit("启动前清理后 ROS 域锁仍被占用，拒绝启动。请检查 startup_preflight.json。")
partition_key = hashlib.sha256(os.environ["IGN_PARTITION"].encode()).hexdigest()[:16]
partition_lock = open(f"/tmp/turtlebot_fleet_ign_{partition_key}.lock", "w")
try:
    fcntl.flock(partition_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit("启动前清理后 Gazebo 分区锁仍被占用，拒绝启动。请检查 startup_preflight.json。")
sources = []
for folder in [root / "src", root / "scripts", root / "tools", root / "vendor/tf2_compat"]:
    sources.extend(
        p
        for p in folder.rglob("*")
        if p.is_file()
        and (p.name == "CMakeLists.txt" or p.suffix
        in [".py", ".sh", ".yaml", ".lua", ".config", ".cfg", ".srv", ".msg", ".action", ".cpp", ".h", ".hpp", ".cmake", ".html", ".js", ".css", ".xml"])
    )
sources.extend(root / name for name in ["env.sh", "build.sh", "run.sh", "demo.sh"])
server_binary = Path(get_package_prefix('fleet_bridge'))/'lib/fleet_bridge/fleet_sim_server'
manifest = {
    'architecture': a.architecture,
    'initialization': 'unknown' if a.architecture == 'decentralized' else 'known',
    'spawn_layout': a.spawn_layout,
    'acceptance_scene': a.acceptance_scene,
    'simulator_backend': 'managed_fortress',
    'simulator_binary': str(server_binary),
    'simulator_binary_sha256': hashlib.sha256(server_binary.read_bytes()).hexdigest(),
    "source_sha256": {
        str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(sources)
    },
    "environment": {
        key: os.environ.get(key)
        for key in [
            "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION",
            "ROS_LOCALHOST_ONLY",
            "CYCLONEDDS_URI",
            "IGN_IP",
            "GZ_IP",
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
    if not tool.exists() or tool.stat().st_mtime_ns < (root/'tools/truth_probe.cpp').stat().st_mtime_ns:
        subprocess.run([str(root / "scripts/build_tools.sh")], check=True)
cmd = [
    "ros2",
    "launch",
    "fleet_bringup",
    "fleet.launch.py",
    f"assets:={run}/assets",
    f'mode:={"base" if a.base_only else "full"}',
    f'no_dashboard:={str(a.no_dashboard).lower()}',
    f'acceptance_scene:={a.acceptance_scene or "none"}',
]
if a.gui:
    cmd.append('gui:=true')
if architecture_explicit:
    cmd.append(f'architecture:={a.architecture}')
config = Path(a.config).expanduser().resolve() if a.config else root / "src/fleet_bringup/config/fleet.yaml"
cfg = yaml.safe_load(config.read_text())
if a.spawn_layout:
    from fleet_bringup.peer_assets import apply_spawn_layout
    apply_spawn_layout(cfg, a.spawn_layout)
if a.scene_generator:
    if not isinstance(cfg, dict) or not isinstance(cfg.setdefault("scene", {}), dict):
        raise ValueError("配置及 scene 必须为映射。")
    cfg.setdefault("scene", {})["generator"] = a.scene_generator
if a.planner:
    cfg["planner"] = a.planner
from fleet_bringup.prepare import validate_config
validate_config(cfg)
if a.seed is not None:
    cfg["scene"]["seed"] = a.seed
if cfg["scene"]["seed"] is None:
    cfg["scene"]["seed"] = secrets.randbits(63)
validate_config(cfg)
if not a.base_only and not a.no_dashboard:
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
proc = None
sampler = None
probe = None
recorder = None
stop = False
shutdown_failed = False
auxiliary_error = None
starting_auxiliary = "launch"


def shutdown(sig, frame):
    global stop
    stop = True


for s in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
    signal.signal(s, shutdown)
try:
    proc = subprocess.Popen(
        # This child has its own session. launch must not mistake our terminal
        # for its own interactive foreground group and skip forwarding SIGINT.
        cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    (root / "runs").mkdir(parents=True, exist_ok=True)
    (root / "runs/latest.txt").write_text(str(run))
    (run / "supervisor.pid").write_text(str(os.getpid()))
    (run / "launch.pid").write_text(str(proc.pid))
    print(
        f"运行目录：{run}\n日志：{run}/launch.log\n架构：{a.architecture}，初始化：{manifest['initialization']}\n"
        + ('网页：未启动（直接 fleetctl 可用）\n' if a.no_dashboard else f"网页：http://127.0.0.1:{cfg['dashboard_port']}\n")
        + f"规划器：{cfg['planner']}\n场景：15×15 m，种子 {cfg['scene']['seed']}，负例 {a.acceptance_scene or '无'}\n复现场景：{reproduction_command(effective_config, cfg['scene']['seed'], architecture=a.architecture, headless=not a.gui, no_dashboard=a.no_dashboard, acceptance_scene=a.acceptance_scene, gui=a.gui)}\n在此终端按 Ctrl+C 完整关闭本次仿真。",
        flush=True,
    )
    starting_auxiliary = "resource_sampler"
    # Child creation belongs inside the cleanup scope too. A missing/unusable
    # probe must not orphan a live simulator and resource sampler.
    sampler = subprocess.Popen(
        [sys.executable, str(root / "scripts/resources.py"), str(run)],
        start_new_session=True,
    )
    if a.record_truth:
        starting_auxiliary = "truth_probe"
        probe = subprocess.Popen([str(tool), str(run / "truth.jsonl")], start_new_session=True)
        if a.architecture == 'decentralized':
            starting_auxiliary = 'peer_recorder'
            recorder = subprocess.Popen([sys.executable, str(root/'scripts/peer_recorder.py'), str(run)], start_new_session=True)
    while proc.poll() is None and not stop:
        for name, child in [("resource_sampler", sampler), ("truth_probe", probe), ('peer_recorder', recorder)]:
            if child is not None and child.poll() is not None:
                auxiliary_error = {"process": name, "returncode": child.returncode}
                stop = True
                break
        time.sleep(0.25)
except OSError as error:
    auxiliary_error = {"process": starting_auxiliary, "error": str(error)}
    raise
finally:
    if proc is not None:
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
        if a.architecture == 'decentralized':
            try:
                from fleet_bringup.peer_restart import stop_detached_session
                detached = stop_detached_session(os.environ['TB4_SIM_SESSION'], proc.pid)
                shutdown_failed = shutdown_failed or bool(detached['remaining'])
                (run/'detached_cleanup.json').write_text(json.dumps(detached, indent=2))
            except Exception as error:
                shutdown_failed = True
                (run/'detached_cleanup.json').write_text(json.dumps({'error': str(error)}))
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
        try:
            log.flush()
            abnormal_exits = [line for line in (run / "launch.log").read_text(errors="replace").splitlines()
                              if "process has died" in line]
        except OSError as error:
            abnormal_exits = [f"无法读取启动日志：{error}"]
        try:
            (run / "cleanup.json").write_text(
                json.dumps(
                    {"residual_before_final_kill": alive, "launch_exit": proc.returncode,
                     "abnormal_process_exits": abnormal_exits, "auxiliary_error": auxiliary_error}
                )
            )
        except OSError as error:
            print(f"无法记录清理结果：{error}", file=sys.stderr)
    if sampler is not None and sampler.poll() is None:
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
    if recorder is not None and recorder.poll() is None:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=4)
        except subprocess.TimeoutExpired:
            recorder.kill()
            recorder.wait()
    log.close()
sys.exit(1 if abnormal_exits or alive or shutdown_failed or auxiliary_error else proc.returncode if not stop else 0)
