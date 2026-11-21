"""Use the owned simulator API before signaling a producer being destroyed."""
import signal
import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import shutdown_context


def simulator(monkeypatch, tmp_path):
    proc = tmp_path/'17'
    proc.mkdir()
    run = tmp_path/'run'
    world = str(run/'assets/arena.sdf').encode()
    (proc/'cmdline').write_bytes(b'ign gazebo '+world+b' -r -s\0')
    original = Path.glob
    monkeypatch.setattr(Path, 'glob', lambda self, pattern:
        [proc] if str(self) == '/proc' else original(self, pattern))
    monkeypatch.setattr(shutdown_context.os, 'getpgid', lambda pid: 3)
    alive = [True]
    monkeypatch.setattr(shutdown_context, 'running', lambda pid: alive[0])
    return run, alive


def test_server_stop_api_does_not_send_a_second_signal_during_clean_teardown(monkeypatch, tmp_path):
    run, alive = simulator(monkeypatch, tmp_path)
    def request(*args, **kwargs):
        alive[0] = False
        return SimpleNamespace(returncode=0, stdout='data: true\n')
    monkeypatch.setattr(shutdown_context.subprocess, 'run', request)
    monkeypatch.setattr(shutdown_context.os, 'kill', lambda *a: (_ for _ in ()).throw(AssertionError('double signal')))
    monkeypatch.setattr(shutdown_context.time, 'sleep', lambda _: None)
    report = shutdown_context.quiesce(run, process_group=3, base_only=True)
    assert report['simulator_stopped'] and report['server_stop_service']['accepted']
    assert 'signal_fallback' not in report


def test_unavailable_server_api_retains_the_owned_pid_signal_fallback(monkeypatch, tmp_path):
    run, alive = simulator(monkeypatch, tmp_path)
    clock, signals = [0.], []
    monkeypatch.setattr(shutdown_context.subprocess, 'run',
                        lambda *a, **k: SimpleNamespace(returncode=1, stdout=''))
    monkeypatch.setattr(shutdown_context.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(shutdown_context.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))
    def stop(pid, sig):
        signals.append((pid, sig))
        alive[0] = False
    monkeypatch.setattr(shutdown_context.os, 'kill', stop)
    report = shutdown_context.quiesce(run, process_group=3, base_only=True)
    assert report['simulator_stopped'] and report['signal_fallback']
    assert signals == [(17, signal.SIGINT)]
