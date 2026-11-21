"""Verify shutdown of a spawned process tree without touching another process."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def is_live(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1][0] != 'Z'
    except FileNotFoundError:
        return False


def test_shutdown_reaches_grandchild_and_spares_unrelated_process():
    script = (
        'import subprocess,sys,time; '
        'p=subprocess.Popen([sys.executable,"-c",'
        '"import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); '
        'time.sleep(60)"]); '
        'print(p.pid,flush=True); time.sleep(60)')
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    wrapper = subprocess.Popen(
        [sys.executable, '-m', 'turtlebot4_tutorial.process_group', sys.executable, '-c', script],
        stdout=subprocess.PIPE, text=True)
    child_pid = None
    try:
        child_pid = int(wrapper.stdout.readline().strip())
        assert is_live(child_pid)
        wrapper.send_signal(signal.SIGINT)
        assert wrapper.wait(timeout=12) == 0
        deadline = time.monotonic() + 2
        while is_live(child_pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not is_live(child_pid)
        assert unrelated.poll() is None
    finally:
        if wrapper.poll() is None:
            wrapper.kill()
            wrapper.wait(timeout=3)
        if child_pid and is_live(child_pid):
            os.kill(child_pid, signal.SIGKILL)
        unrelated.terminate()
        unrelated.wait(timeout=3)
