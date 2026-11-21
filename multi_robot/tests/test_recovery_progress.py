"""Cancellation and backend death must preserve the actual failing stage."""
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import acceptance_progress
from acceptance_progress import AcceptanceProgress, AcceptanceInterrupted, BackendExited


def progress(tmp_path, monkeypatch, backend=lambda: None):
    clock = [100.]
    monkeypatch.setattr(acceptance_progress.time, 'monotonic', lambda: clock[0])
    def state(timeout):
        clock[0] += .1
        return dict(team_ready=False, mode='IDLE', robots={'robot3': dict(ready=True,
            mission_id='restored', alignment={'target_revision': 12, 'confirmed_revision': 10})})
    api = SimpleNamespace(state=state)
    return AcceptanceProgress(tmp_path, 'check', api, backend), clock


def test_timeout_reports_map_confirmation_instead_of_task_delivery(tmp_path, monkeypatch):
    check, clock = progress(tmp_path, monkeypatch)
    with pytest.raises(TimeoutError, match='stationary_map_confirmation'):
        with check:
            check.stage('stationary_map_confirmation', .4)
            check.wait(lambda s: s['team_ready'], .4)
    report = json.loads((tmp_path/'check.json').read_text())
    assert report['outcome'] == 'failed' and not report['passed']
    assert report['last_state']['robots']['robot3']['alignment']['confirmed_revision'] == 10
    assert report['stages'][-1]['outcome'] == 'failed'


def test_signal_is_a_flag_and_cancellation_restores_handlers(tmp_path, monkeypatch):
    check, clock = progress(tmp_path, monkeypatch)
    previous = signal.getsignal(signal.SIGINT)
    with pytest.raises(AcceptanceInterrupted, match='SIGINT'):
        with check:
            check.stage('operator_closed_past_proposal_deadline', 21)
            # Deliver through the installed handler without interrupting a C callback.
            signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
            check.hold(21)
    assert signal.getsignal(signal.SIGINT) == previous
    report = json.loads((tmp_path/'check.json').read_text())
    assert report['outcome'] == 'interrupted' and not report['passed']
    assert clock[0] == 100.  # No extra state request or timeout after cancellation.


def test_backend_exit_during_state_read_cannot_be_mistaken_for_success(tmp_path, monkeypatch):
    dead = [False]
    check, clock = progress(tmp_path, monkeypatch, lambda: 'owned backend exited' if dead[0] else None)
    def state(timeout):
        dead[0] = True
        return {'team_ready': True}
    check.api.state = state
    with pytest.raises(BackendExited):
        with check:
            check.stage('sensor_and_task_rejoin', 12)
            check.wait(lambda s: s['team_ready'], 12)
    report = json.loads((tmp_path/'check.json').read_text())
    assert report['outcome'] == 'backend_exited' and report['stage'] == 'sensor_and_task_rejoin'
    assert not report['passed']


def test_explicit_stop_marker_is_interruption_not_timeout(tmp_path, monkeypatch):
    check, clock = progress(tmp_path, monkeypatch)
    with pytest.raises(AcceptanceInterrupted):
        with check:
            check.stage('sensor_and_task_rejoin', 12)
            (tmp_path/'user_requested_stop.json').write_text('{}')
            check.wait(lambda s: False, 12)
    assert json.loads((tmp_path/'check.json').read_text())['outcome'] == 'interrupted'


def test_stage_evidence_keeps_successful_prefix_and_does_not_pass_without_completion(tmp_path, monkeypatch):
    check, clock = progress(tmp_path, monkeypatch)
    with check:
        check.stage('task_rejoined', 12)
        check.wait(lambda s: s['robots']['robot3']['mission_id'] == 'restored', 12)
        check.stage('stationary_map_confirmation', 60)
    report = json.loads((tmp_path/'check.json').read_text())
    assert report['stages'][0]['outcome'] == 'passed'
    assert report['stages'][1]['outcome'] == 'failed' and not report['passed']
