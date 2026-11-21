import json
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import peer_release_evidence as ev
import peer_release_acceptance as runner
from acceptance_progress import AcceptanceInterrupted


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def fault_evidence(tmp_path):
    cases = []
    for name, component, action, jump in ev.expected_faults():
        value = dict(robot=name, component=component, action=action, jump_seconds=jump, passed=True)
        if component:
            value.update(boot_id=f'{name}-{component}-{action}', map_epoch=2, restart_alignment_seconds=100.)
        if action in ('one_way_loss', 'loss_reorder_duplicates', 'late_votes'):
            from peer_network_evidence import save_audit
            value['affected'] = 'robot2'
            for phase in ('no_old_task_resumption', 'new_task_recovery', 'affected_motion', 'authorization_window'):
                moving = phase != 'no_old_task_resumption'
                cmd = [(i/20, .1 if moving else 0., 0.) for i in range(25)]
                guards = [(t, dict(v=v, w=w, moving_allowed=moving, lease_valid=moving)) for t, v, w in cmd]
                truth = [dict(steady_time=t, robots=dict(robot2=[v*t, 0., 0.])) for t, v, w in cmd]
                value[phase] = save_audit(tmp_path/'network-evidence'/f'{action}-{phase}.json',
                    cmd, guards, truth, 'robot2', 0., 1.2, raw=[dict(steady_time=0.)],
                    stationary=not moving, require_motion=moving)
        cases.append(value)
    write(tmp_path/'peer-fault-acceptance.json', cases)
    write(tmp_path/'peer-fault-manifest.json', dict(full_matrix=True))
    for name in ('peer-fault-run.json', 'peer-fault-summary.json'):
        write(tmp_path/name, dict(passed=True, full_matrix=True, completed_cases=42, process_cases=30, outcome='passed'))
    return tmp_path, cases


def test_all_required_fault_cases_must_be_present(fault_evidence):
    run, cases = fault_evidence
    assert len(ev.expected_faults()) == 42
    ev.complete_faults(run)
    write(run/'peer-fault-acceptance.json', cases[:5])
    with pytest.raises(AssertionError):
        ev.complete_faults(run)


def test_duplicate_case_does_not_replace_a_missing_fault(fault_evidence):
    run, cases = fault_evidence
    cases[-1] = cases[0]
    write(run/'peer-fault-acceptance.json', cases)
    with pytest.raises(AssertionError):
        ev.complete_faults(run)


def test_interruption_cannot_pass_even_with_full_requested_scope(fault_evidence):
    run, _ = fault_evidence
    write(run/'peer-fault-run.json', dict(passed=False, full_matrix=True, completed_cases=42,
                                        process_cases=30, outcome='interrupted'))
    with pytest.raises(AssertionError):
        ev.complete_faults(run)


def test_network_summary_without_original_motion_window_cannot_pass(fault_evidence):
    run, cases = fault_evidence
    case = next(c for c in cases if c['action'] == 'late_votes')
    Path(case['authorization_window']['evidence']).unlink()
    with pytest.raises(FileNotFoundError):
        ev.complete_faults(run)


def test_restart_deadline_is_checked_independently_of_preparation(fault_evidence):
    run, cases = fault_evidence
    case = next(c for c in cases if c.get('component'))
    case['restart_alignment_seconds'] = 300.01
    write(run/'peer-fault-acceptance.json', cases)
    with pytest.raises(AssertionError):
        ev.complete_faults(run)


def test_new_source_file_invalidates_frozen_run(tmp_path):
    frozen = dict(source_sha256={'old.py': 'a', 'new.py': 'b'}, artifacts_sha256={
        'install/fleet_bridge/lib/fleet_bridge/fleet_sim_server': 'binary'})
    write(tmp_path/'manifest.json', dict(architecture='decentralized', initialization='unknown',
        source_sha256={'old.py': 'a'}, simulator_binary_sha256='binary'))
    with pytest.raises(AssertionError):
        ev.check_manifest(tmp_path, frozen)


@pytest.mark.parametrize('failure, expected', [(RuntimeError('gate failed'), 'failed'),
                                             (AcceptanceInterrupted('SIGINT'), 'interrupted')])
def test_release_failure_preserves_pending_gates_without_launching_more(tmp_path, monkeypatch, failure, expected):
    root = tmp_path/'workspace'
    for name in ev.DEFAULT_FILES:
        path = root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("default='centralized'\n")
    monkeypatch.setattr(runner, '__file__', str(root/'scripts/peer_release_acceptance.py'))
    monkeypatch.setattr(runner, 'freeze', lambda root: dict(source_sha256={}, artifacts_sha256={}, tests_sha256={}))
    launched = []
    def fail(command, **kwargs):
        launched.append(command)
        raise failure
    monkeypatch.setattr(runner, 'run_command', fail)
    out = tmp_path/'release'
    monkeypatch.setattr(sys, 'argv', ['release', '--output', str(out)])
    with pytest.raises(type(failure)):
        runner.main()
    result = ev.read(out/'release-acceptance.json')
    assert len(launched) == 1 and not result['passed'] and not result['prerequisites_ready']
    assert result['outcome'] == result['gates']['full']['status'] == expected
    assert all(result['gates'][n]['status'] == 'not_run' for n in runner.GATES[1:])
    assert (out/'release-evidence-sha256.json').exists()


def test_default_cannot_be_finalized_before_all_prerequisites(tmp_path):
    write(tmp_path/'release-acceptance.json', dict(prerequisites_ready=False))
    write(tmp_path/'source-freeze.json', {})
    with pytest.raises(AssertionError):
        runner.finalize_default(tmp_path, tmp_path, tmp_path/'default-run')
