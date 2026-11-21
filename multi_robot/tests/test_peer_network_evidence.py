import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_network_evidence import audit_motion
from peer_network_evidence import source_steady, save_audit


def samples(moving=True):
    times = [i/20 for i in range(25)]
    commands = [(t, .1 if moving else 0., 0.) for t in times]
    guards = [(t, dict(v=v, w=w, moving_allowed=moving, lease_valid=moving)) for t, v, w in commands]
    truth = [dict(steady_time=t, robots=dict(robot2=[.1*t if moving else 0., 0., 0.])) for t in times]
    return commands, guards, truth


def test_affected_member_requires_actuator_authority_and_physical_translation():
    result = audit_motion(*samples(), 'robot2', 0., 1.2, require_motion=True)
    assert result['displacement'] == pytest.approx(.12)


@pytest.mark.parametrize('missing', ['command', 'guard', 'truth'])
def test_missing_network_observation_fails_closed(missing):
    data = list(samples())
    data[['command', 'guard', 'truth'].index(missing)] = []
    with pytest.raises(AssertionError):
        audit_motion(*data, 'robot2', 0., 1.2)


def test_velocity_without_a_valid_lease_cannot_pass():
    cmd, guards, truth = samples()
    for _, status in guards:
        status['lease_valid'] = False
    with pytest.raises(AssertionError, match='多数授权'):
        audit_motion(cmd, guards, truth, 'robot2', 0., 1.2)


def test_stationary_status_alone_does_not_prove_physical_recovery():
    with pytest.raises(AssertionError, match='真实平移'):
        audit_motion(*samples(False), 'robot2', 0., 1.2, require_motion=True)


def test_no_old_task_resumption_requires_both_zero_commands_and_stationary_truth():
    audit_motion(*samples(False), 'robot2', 0., 1.2, stationary=True)
    with pytest.raises(AssertionError, match='重新运动'):
        audit_motion(*samples(), 'robot2', 0., 1.2, stationary=True)
    cmd, guard, truth = samples(False)
    truth[-1]['robots']['robot2'][2] = .2
    with pytest.raises(AssertionError, match='重新运动'):
        audit_motion(cmd, guard, truth, 'robot2', 0., 1.2, stationary=True)


def test_callback_stall_does_not_shift_source_time_or_hide_missing_guard_samples():
    cmd, guards, truth = samples()
    # The command reader drains six queued samples after a 300 ms stall;
    # the depth-one status reader kept only the last one. No valid audit can
    # be invented for those missing matching velocities.
    cmd = [(t, .05+.03*t, 0.) for t, _, _ in cmd]
    guards = [(t, dict(v=v, w=w, moving_allowed=True, lease_valid=True)) for t, v, w in cmd]
    with pytest.raises(AssertionError, match='多数授权'):
        audit_motion(cmd, guards[6:], truth, 'robot2', 0., 1.2)
    # DDS source time makes different callback delays irrelevant, while the
    # complete independently drained status stream preserves exact velocities.
    actual = []
    for t, v, w in cmd:
        delay = .3 if t <= .3 else .02
        received = t+delay
        stamp = source_steady({'source_timestamp': int((1000+t)*1e9)}, received,
                              int((1000+received)*1e9))
        actual.append((stamp, v, w))
    audit_motion(actual, guards, truth, 'robot2', 0., 1.2, require_motion=True)


@pytest.mark.parametrize('info', [{}, {'source_timestamp': 0}, {'source_timestamp': 900_000_000_000},
                                {'source_timestamp': 1001_000_000_000}])
def test_invalid_dds_clock_fails_closed(info):
    with pytest.raises(ValueError):
        source_steady(info, 5., 1000_000_000_000)


def test_failed_audit_preserves_actual_inputs_and_cannot_overwrite_previous_window(tmp_path):
    import json
    cmd, guards, truth = samples()
    guards[0][1]['lease_valid'] = False
    guards = guards[:1]
    path = tmp_path/'fault.json'
    with pytest.raises(AssertionError):
        save_audit(path, cmd, guards, truth, 'robot2', 0., 1.2)
    result = json.loads(path.read_text())
    assert not result['passed'] and result['error'] and len(result['commands']) == len(cmd)
    previous = path.read_bytes()
    with pytest.raises(FileExistsError):
        save_audit(path, *samples(), 'robot2', 0., 1.2)
    assert path.read_bytes() == previous
