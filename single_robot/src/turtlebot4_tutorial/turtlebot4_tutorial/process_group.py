"""Run one process tree in its own session and terminate only that tree."""

import os
import signal
import subprocess
import sys
import time


def group_exists(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def main():
    command = sys.argv[1:]
    if not command:
        print('missing child command', file=sys.stderr)
        return 2
    stopping = False
    child = None

    def request_stop(signum, _frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            if child is not None:
                signal_group(child.pid, signum)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    child = subprocess.Popen(command, start_new_session=True)
    if stopping:
        signal_group(child.pid, signal.SIGINT)
    try:
        while child.poll() is None and not stopping:
            time.sleep(0.1)
    finally:
        # The CLI may exit before its Gazebo child. Keep the process group
        # isolated and drain it even after the direct child has exited.
        signal_group(child.pid, signal.SIGINT)
        for sig, seconds in ((signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
            deadline = time.monotonic() + seconds
            while group_exists(child.pid) and time.monotonic() < deadline:
                child.poll()
                time.sleep(0.05)
            if group_exists(child.pid):
                signal_group(child.pid, sig)
        try:
            child.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            print('child did not exit after SIGKILL', file=sys.stderr)
            return 1
    return 0 if stopping else (child.returncode or 0)


if __name__ == '__main__':
    sys.exit(main())
