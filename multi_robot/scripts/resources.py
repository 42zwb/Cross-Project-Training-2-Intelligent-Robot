#!/usr/bin/env python3
"""Sample this project's process tree, without including the old simulator."""
import json, os, signal, sys, time
from pathlib import Path

run = Path(sys.argv[1])
pid = int((run / "launch.pid").read_text())
session = os.environ.get('TB4_SIM_SESSION')
running = True


def oom_kills():
    return int(next(line.split()[1] for line in Path('/proc/vmstat').read_text().splitlines()
                    if line.startswith('oom_kill ')))


initial_oom_kills = oom_kills()


def stop(*a):
    global running
    running = False


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
with open(run / "resources.jsonl", "a", buffering=1) as out:
    while running and Path(f"/proc/{pid}").exists():
        processes = {}
        for p in Path("/proc").glob("[0-9]*"):
            try:
                fields = (p / "stat").read_text().split(") ", 1)[1].split()
                rss = int(fields[21]) * os.sysconf("SC_PAGE_SIZE")
                processes[int(p.name)] = (int(fields[1]), rss)
            except (OSError, ValueError, IndexError):
                pass
        selected = {pid}
        if session:
            for member in processes:
                try:
                    raw = Path(f'/proc/{member}/environ').read_bytes().split(b'\0')
                    if ('TB4_SIM_SESSION='+session).encode() in raw:
                        selected.add(member)
                except OSError:
                    pass
        while True:
            extended = selected | {
                p for p, (parent, _) in processes.items() if parent in selected
            }
            if extended == selected:
                break
            selected = extended
        mem = {
            k: int(v.split()[0])
            for k, v in [
                l.split(":", 1) for l in Path("/proc/meminfo").read_text().splitlines()
            ]
        }
        out.write(
            json.dumps(
                {
                    "wall_time": time.time(),
                    "rss_mb": sum(processes.get(p, (0, 0))[1] for p in selected)
                    / 1048576,
                    "process_count": len(selected),
                    "system_available_mb": mem["MemAvailable"] / 1024,
                    "system_oom_kills_since_start": oom_kills()-initial_oom_kills,
                }
            )
            + "\n"
        )
        time.sleep(2)
