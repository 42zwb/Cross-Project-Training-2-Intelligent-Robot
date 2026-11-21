"""Real process faults plus process-identity and ownership boundaries."""
import json
import os
import signal
import subprocess
import sys

import pytest
from fleet_bringup import simulation_preflight as pf


def process(pid=100, ppid=1, argv=(), env=None, cwd=''):
    return pf.Process(pid, ppid, '1234', tuple(argv), env or {}, cwd, 'S')


def test_legacy_identification_requires_executable_and_project_evidence():
    assert pf.legacy_reason(process(argv=['/usr/bin/python3', '/opt/ros/humble/bin/ros2',
                                          'launch', 'turtlebot4_tutorial',
                                          'tutorial_navigation.launch.py']))
    assert pf.legacy_reason(process(argv=['ign gazebo server'],
                                    env={'IGN_PARTITION': 'tb4_single_demo'}))
    assert pf.legacy_reason(process(argv=['/opt/ros/humble/lib/nav2_controller/controller_server'],
                                    env={'AMENT_PREFIX_PATH': str(pf.ROOTS[1] / 'install/x')}))
    assert not pf.legacy_reason(process(argv=['ign gazebo server'],
                                        env={'IGN_PARTITION': 'another_project'}))
    shell = ['bash', '-c', 'ros2 launch turtlebot4_tutorial tutorial_mapping.launch.py']
    assert not pf.legacy_reason(process(argv=shell))
    assert not pf.legacy_reason(process(
        argv=['/usr/bin/python3', '-c', str(pf.ROOTS[1] / 'scripts/supervise.py')]))


def test_capture_descendants_but_protect_caller_ancestors_and_current_session():
    tag = {pf.PROJECT: 'single', pf.SESSION: 'previous'}
    processes = {10: process(10, 1, env=tag), 11: process(11, 10),
                 12: process(12, 11), 20: process(20, 1),
                 30: process(30, 1, env={pf.PROJECT: 'fleet', pf.SESSION: 'current'})}
    result = pf.find_targets(processes, {1, 20}, 'current', legacy=False)
    assert set(result) == {10, 11, 12}
    assert not pf.find_targets({10: processes[10]}, {1, 10}, legacy=False)


def test_relative_supervisor_path_is_resolved_against_target_cwd():
    assert pf.legacy_reason(process(argv=['python3', 'scripts/supervise.py'],
                                    cwd=str(pf.ROOTS[1])))
    assert not pf.legacy_reason(process(argv=['python3', 'scripts/supervise.py'],
                                        cwd='/tmp/unrelated-project'))


def isolated_snapshot(monkeypatch, pids):
    original = pf.snapshot

    def only_test_processes():
        return {p: v for p, v in original().items() if p in pids or p == os.getpid()}
    monkeypatch.setattr(pf, 'snapshot', only_test_processes)


def reap(pid):
    proc = pf.read_process(pid)
    if proc and pf.still_alive(proc):
        os.kill(pid, signal.SIGKILL)


def test_reaps_stopped_orphan_ignoring_signals_and_spares_unrelated(tmp_path, monkeypatch):
    env = dict(os.environ, TB4_SIM_PROJECT='fleet', TB4_SIM_SESSION='preflight-test-orphan')
    child = ('import os,signal,time; '
             'signal.signal(signal.SIGINT,signal.SIG_IGN); '
             'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
             'print(os.getpid(),flush=True); time.sleep(60)')
    parent = subprocess.Popen([
        sys.executable, '-c',
        'import subprocess,sys; subprocess.Popen([sys.executable,"-c",sys.argv[1]],'
        'start_new_session=True)', child], env=env, stdout=subprocess.PIPE, text=True)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    orphan = int(parent.stdout.readline())
    parent.wait(timeout=3)
    original = pf.read_process(orphan)
    try:
        assert original and pf.still_alive(original)
        os.kill(orphan, signal.SIGSTOP)
        isolated_snapshot(monkeypatch, {orphan, unrelated.pid})
        result = pf.clean_previous(tmp_path / 'report.json', legacy=False,
                                   timeouts=(0.15, 0.15, 1))
        assert result['success'] and not pf.still_alive(original)
        assert any(s['signal'] == 'SIGKILL' for s in result['signals'])
        assert unrelated.poll() is None
        assert json.loads((tmp_path / 'report.json').read_text())['remaining'] == []
    finally:
        reap(orphan)
        unrelated.terminate()
        unrelated.wait(timeout=3)
        parent.stdout.close()


def test_reaps_child_that_cleared_environment_after_parent_exits(tmp_path, monkeypatch):
    env = dict(os.environ, TB4_SIM_PROJECT='single', TB4_SIM_SESSION='preflight-test-tree')
    script = ('import subprocess,sys,time; p=subprocess.Popen([sys.executable,"-c",'
              '"import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(60)"],'
              'env={},start_new_session=True); print(p.pid,flush=True); time.sleep(60)')
    parent = subprocess.Popen([sys.executable, '-c', script], env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    child = int(parent.stdout.readline())
    original = pf.read_process(child)
    try:
        isolated_snapshot(monkeypatch, {parent.pid, child})
        result = pf.clean_previous(tmp_path / 'report.json', legacy=False,
                                   timeouts=(0.2, 0.2, 1))
        assert result['success']
        parent.wait(timeout=3)
        assert not pf.still_alive(original)
        assert {p['pid'] for p in result['processes']} == {parent.pid, child}
    finally:
        reap(child)
        if parent.poll() is None:
            parent.kill()
        parent.wait(timeout=3)
        parent.stdout.close()


def test_fail_closed_and_write_report_if_cleanup_cannot_stop_process(tmp_path, monkeypatch):
    child = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(60)'],
        env=dict(os.environ, TB4_SIM_PROJECT='single', TB4_SIM_SESSION='preflight-test-survivor'))
    try:
        isolated_snapshot(monkeypatch, {child.pid})
        monkeypatch.setattr(pf, '_send', lambda proc, sig: None)
        with pytest.raises(RuntimeError, match='拒绝启动'):
            pf.clean_previous(tmp_path / 'report.json', legacy=False, timeouts=(0, 0, 0))
        assert json.loads((tmp_path / 'report.json').read_text())['remaining'] == [child.pid]
    finally:
        child.kill()
        child.wait(timeout=3)


def test_nested_launch_reuses_session_without_cleaning_current_simulator(monkeypatch):
    current = pf.read_process(os.getpid())
    env = {pf.PROJECT: 'fleet', pf.SESSION: 'test-current', pf.OWNER: str(current.pid),
           pf.START: current.start}
    monkeypatch.setattr(pf, 'clean_previous', lambda *a, **kw: pytest.fail('duplicate cleanup'))
    assert pf.begin_session('fleet', env=env) == env


def test_recycled_pid_identity_is_not_considered_alive(monkeypatch):
    old = process(100)
    new = process(100)
    new.start = '9876'
    monkeypatch.setattr(pf, 'read_process', lambda pid: new)
    assert not pf.still_alive(old)
