"""Stop this invocation's data producer before destroying ROS-Gazebo bridges."""

import os
import signal
import subprocess
import time
from pathlib import Path
from runtime_context import RuntimeClient


def running(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1][0] != "Z"
    except (OSError, IndexError):
        return False


def simulator_command_matches(raw, world):
    args = [part for part in raw.split(b'\0') if part]
    if (len(args) == 2 and Path(os.fsdecode(args[0])).name == 'fleet_sim_server'
            and args[1] == world):
        return True
    # The ign Ruby launcher rewrites argv into one process-title string.
    # Match the complete SDF path in both layouts, including paths with spaces.
    title = b" ".join(part for part in raw.split(b"\0") if part) + b" "
    return b"gazebo " + world + b" " in title


def quiesce(run, process_group, base_only=False):
    report = {}
    if not base_only:
        try:
            report["pause_command"] = RuntimeClient(run).request(
                "command", {"command":"pause"}, timeout=1.2)
        except (OSError, ValueError, RuntimeError) as error:
            report["pause_command_error"] = str(error)
        time.sleep(0.2)
    pids = []
    managed = False
    world = str(Path(run).resolve() / "assets/arena.sdf").encode()
    for entry in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(entry.name)
            command = (entry / "cmdline").read_bytes()
            if os.getpgid(pid) == process_group and simulator_command_matches(command, world):
                pids.append(pid)
                managed = b'fleet_sim_server' in command.split(b'\0')[0]
        except (OSError, ValueError):
            pass
    report["simulator_pids"] = pids
    if len(pids) != 1:
        report.update(simulator_stopped=False, error="无法唯一确定本次 Gazebo 服务器，交给 launch 清理")
        return report
    if managed:
        # The managed host handles one signal synchronously and keeps its ROS
        # context alive through world destruction. Do not also call Fortress's
        # asynchronous server_control service or interrupt its teardown twice.
        report['stop_method'] = 'managed_server_signal'
        os.kill(pids[0], signal.SIGINT)
        deadline = time.monotonic() + 12.
        while running(pids[0]) and time.monotonic() < deadline:
            time.sleep(.05)
        report['simulator_stopped'] = not running(pids[0])
        if report['simulator_stopped']:
            time.sleep(.3)
            report['bridge_drain_seconds'] = .3
        return report
    # Ask the owned Fortress server to stop its simulation thread before a
    # Unix signal starts shared ROS context teardown. Keep the bridges alive.
    try:
        response = subprocess.run(['ign', 'service', '-s', '/server_control',
            '--reqtype', 'ignition.msgs.ServerControl', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1500', '--req', 'stop: true'],
            capture_output=True, text=True, timeout=3.)
        report['server_stop_service'] = dict(returncode=response.returncode,
            accepted=response.returncode == 0 and 'data: true' in response.stdout)
    except (OSError, subprocess.TimeoutExpired) as error:
        report['server_stop_service_error'] = str(error)
    deadline = time.monotonic() + 8.0
    while any(running(pid) for pid in pids) and time.monotonic() < deadline:
        time.sleep(0.05)
    if any(running(pid) for pid in pids):
        report['signal_fallback'] = True
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic()+6.
        while any(running(pid) for pid in pids) and time.monotonic() < deadline:
            time.sleep(.05)
    report["simulator_stopped"] = not any(running(pid) for pid in pids)
    if report["simulator_stopped"]:
        # Keep clock and scan bridges alive until the producer has exited.
        time.sleep(0.3)
        report["bridge_drain_seconds"] = 0.3
    return report
