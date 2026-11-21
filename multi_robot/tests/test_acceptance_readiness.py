"""Replay the real post-export TF dip before allowing another test mission."""
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_acceptance_readiness import ReadinessHold


def samples():
    return [row['state'] for row in json.loads((Path(__file__).parent/
        'data/readiness-after-map-export.json').read_text())['samples']]


def test_actual_export_dip_and_stationary_revalidation_must_finish_before_patrol():
    good, missing_tf, unverified = samples()
    assert good['robots']['robot2']['aligned']
    assert missing_tf['robots']['robot2']['pose_sim_time'] is None
    assert unverified['robots']['robot2']['ready'] and not unverified['robots']['robot2']['aligned']
    hold = ReadinessHold(['robot1'], all_members=True)
    assert not hold.update(good, 0.)
    assert not hold.update(missing_tf, .5)
    assert not hold.update(unverified, 1.)
    assert not hold.update(unverified, 4.)  # Elapsed time alone is not evidence.
    recovered = copy.deepcopy(good)
    assert not hold.update(recovered, 5.)
    assert not hold.update(recovered, 5.9)
    assert hold.update(recovered, 6.1)


def test_network_checks_may_use_two_ready_members_but_not_an_unavailable_owner():
    state = copy.deepcopy(samples()[0])
    state['robots'].pop('robot1')
    state['team_ready'] = False
    for robot in state['robots'].values():
        robot['connected_members'] = ['robot2', 'robot3']
    hold = ReadinessHold(['robot3'])
    assert not hold.update(state, 0.)
    assert hold.update(state, 1.1)
    all_members = ReadinessHold(['robot3'], all_members=True)
    assert not all_members.update(state, 0.) and not all_members.update(state, 10.)
    unavailable_owner = ReadinessHold(['robot1'])
    assert not unavailable_owner.update(state, 0.) and not unavailable_owner.update(state, 10.)


def test_reboot_or_a_new_frame_requires_a_new_continuous_readiness_window():
    state = copy.deepcopy(samples()[0])
    hold = ReadinessHold(['robot1'], all_members=True)
    assert not hold.update(state, 0.)
    state['robots']['robot1']['boot_id'] = 'replacement'
    state['robots']['robot1']['map_epoch'] += 1
    assert not hold.update(state, .9)
    assert not hold.update(state, 1.5)
    assert hold.update(state, 2.)
    state['coordinate_frame'] = 'new-frame'
    for robot in state['robots'].values():
        robot['frame_epoch'] = 'new-frame'
    assert not hold.update(state, 2.1)
    assert hold.update(state, 3.2)


def test_cached_readiness_or_nonzero_and_invalid_speed_are_not_stationary_evidence():
    state = copy.deepcopy(samples()[0])
    hold = ReadinessHold(['robot1'], all_members=True)
    assert not hold.update(state, 0.)
    state['robots']['robot1']['state_age'] = .9
    assert not hold.update(state, 1.1)
    state['robots']['robot1']['state_age'] = .1
    state['robots']['robot1']['safety']['v'] = .1
    assert not hold.update(state, 2.2)
    state['robots']['robot1']['safety']['v'] = float('nan')
    assert not hold.update(state, 3.3)
    state['robots']['robot1']['safety']['v'] = 0.
    assert not hold.update(state, 4.4)
    assert hold.update(state, 5.5)
