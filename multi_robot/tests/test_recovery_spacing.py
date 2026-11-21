"""Do not stop a new alignment in the recorded unreleasable 0.5965 m pair."""
import copy
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_acceptance_readiness import ReadinessHold, RecoverySpacingHold


def sample(which):
    value = json.loads((Path(__file__).parent/'data/recovery_pause_close_pair.json').read_text())
    state = value[which]
    state['estop'] = False
    for robot in state['robots'].values():
        # The fault progress snapshot does not include this parent client's
        # projection field; its real full-membership readiness was checked.
        robot['connected_members'] = ['robot1', 'robot2', 'robot3']
    return state


def fresh(state, now):
    state = copy.deepcopy(state)
    for r in state['robots'].values():
        r['pose_steady_time'], r['state_age'] = now-.1, .1
    return state


def test_real_close_pair_is_ready_but_not_a_valid_point_to_pause_recovery():
    state = sample('state')
    old = ReadinessHold(['robot1', 'robot2', 'robot3'], all_members=True, stationary=False)
    new = RecoverySpacingHold()
    for t in (1., 2.1, 3.2):
        old.update(fresh(state, t), t)
        assert not new.update(fresh(state, t), t)
    assert old.update(fresh(state, 4.), 4.)
    assert new.details['minimum_separation'] == pytest.approx(.596513527664551)


def test_recovery_spacing_is_continuous_and_allows_the_existing_mission_to_move():
    close, safe = sample('state'), sample('safe_moving_state')
    assert any(r.get('goal') for r in safe['robots'].values())
    hold = RecoverySpacingHold()
    assert not hold.update(fresh(safe, 1.), 1.)
    assert not hold.update(fresh(close, 1.9), 1.9)
    assert not hold.update(fresh(safe, 2.), 2.)
    assert hold.update(fresh(safe, 3.1), 3.1)


@pytest.mark.parametrize('invalid', ['stale_pose', 'future_pose', 'missing_pose', 'not_nav_ready',
                                    'stale_state', 'wrong_frame', 'estop'])
def test_spacing_alone_does_not_replace_current_identity_and_readiness(invalid):
    state = fresh(sample('safe_moving_state'), 5.)
    robot = state['robots']['robot1']
    if invalid == 'stale_pose': robot['pose_steady_time'] = 4.1
    elif invalid == 'future_pose': robot['pose_steady_time'] = 5.1
    elif invalid == 'missing_pose': robot['pose'] = None
    elif invalid == 'not_nav_ready': robot['nav_ready'] = False
    elif invalid == 'stale_state': robot['state_age'] = .9
    elif invalid == 'wrong_frame': robot['frame_epoch'] = 'retired'
    elif invalid == 'estop': state['estop'] = True
    hold = RecoverySpacingHold()
    assert not hold.update(state, 5.)
    assert not hold.update(state, 6.1)
