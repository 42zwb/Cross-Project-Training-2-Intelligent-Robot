"""Stop this invocation's data producer before destroying ROS-Gazebo bridges."""

import os
import signal
import time
from pathlib import Path
from runtime_context import RuntimeClient


def running(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1][0] != "Z"
    except (OSError, IndexError):
        return False


def simulator_command_matches(raw, world):
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
    world = str(Path(run).resolve() / "assets/arena.sdf").encode()
    for entry in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(entry.name)
            command = (entry / "cmdline").read_bytes()
            if os.getpgid(pid) == process_group and simulator_command_matches(command, world):
                pids.append(pid)
        except (OSError, ValueError):
            pass
    report["simulator_pids"] = pids
    if len(pids) != 1:
        report.update(simulator_stopped=False, error="无法唯一确定本次 Gazebo 服务器，交给 launch 清理")
        return report
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 6.0
    while any(running(pid) for pid in pids) and time.monotonic() < deadline:
        time.sleep(0.05)
    report["simulator_stopped"] = not any(running(pid) for pid in pids)
    if report["simulator_stopped"]:
        # Keep clock and scan bridges alive until the producer has exited.
        time.sleep(0.3)
        report["bridge_drain_seconds"] = 0.3
    return report
