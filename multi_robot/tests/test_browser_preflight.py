import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import acceptance_process as process
import one_click_demo as demo


def test_browser_failure_prevents_any_robot_process_launch(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['demo', '--architecture', 'decentralized', '--with-ui-checks',
                                    '--run-dir', str(tmp_path/'run')])
    def unavailable(root):
        raise RuntimeError('browser unavailable')
    monkeypatch.setattr(demo, 'browser_environment', unavailable)
    def forbidden(*args, **kwargs):
        pytest.fail('robot process launched before browser preflight')
    monkeypatch.setattr(demo.subprocess, 'Popen', forbidden)
    with pytest.raises(RuntimeError, match='browser unavailable'):
        demo.main()
    assert not (tmp_path/'run').exists()


def test_browser_cache_is_passed_only_in_returned_child_environment(tmp_path, monkeypatch):
    cache = tmp_path/'isolated browser dependencies'
    cache.mkdir()
    (tmp_path/'requirements-browser.txt').write_text('playwright==1.64.1\n')
    monkeypatch.setenv('FLEET_BROWSER_PYTHON', str(cache))
    monkeypatch.setenv('PYTHONPATH', '/ros/python')
    calls = []
    def run(command, **kwargs):
        calls.append(kwargs)
        return SimpleNamespace(stdout=json.dumps(dict(playwright='1.64.1', chromium='test')))
    monkeypatch.setattr(process.subprocess, 'run', run)
    env, report = process.browser_environment(tmp_path)
    assert env['PYTHONPATH'] == str(cache)+':/ros/python'
    assert process.os.environ['PYTHONPATH'] == '/ros/python'
    assert calls[0]['env'] == env and report['passed']


def test_wrong_browser_package_version_is_rejected_before_simulation(tmp_path, monkeypatch):
    (tmp_path/'requirements-browser.txt').write_text('playwright==1.64.1\n')
    monkeypatch.setattr(process.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(stdout=json.dumps(dict(playwright='old', chromium='test'))))
    with pytest.raises(RuntimeError, match='预检失败'):
        process.browser_environment(tmp_path)


def test_exit_between_poll_and_signal_is_reaped_without_hiding_the_original_result(monkeypatch):
    reaped = []
    child = SimpleNamespace(pid=123, poll=lambda: None, wait=lambda **k: reaped.append(k))
    def exited(*args):
        raise ProcessLookupError()
    monkeypatch.setattr(process.os, 'killpg', exited)
    process.stop_child(child)
    assert reaped == [dict(timeout=10)]


def test_cancellation_is_saved_before_nested_cleanup_and_keeps_its_full_budget(tmp_path, monkeypatch):
    from acceptance_progress import AcceptanceProgress, AcceptanceInterrupted
    progress = AcceptanceProgress(tmp_path, 'driver', SimpleNamespace(), backend_status=lambda: None)
    progress.stop_reason = 'SIGINT'
    child = SimpleNamespace(pid=123, poll=lambda: None)
    monkeypatch.setattr(process.subprocess, 'Popen', lambda *a, **k: child)
    observed = []
    def cleanup(actual, grace):
        saved = json.loads((tmp_path/'driver.json').read_text())
        assert actual is child and saved['outcome'] == 'interrupted' and not saved['passed']
        observed.append(grace)
    monkeypatch.setattr(process, 'stop_child', cleanup)
    with pytest.raises(AcceptanceInterrupted):
        process.run_command(['unused'], cwd=tmp_path, log=tmp_path/'driver.log',
                            progress=progress, timeout=30, shutdown_grace=420)
    assert observed == [420]
