"""A cold direct client cannot turn missing poses into UI patrol waypoints."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_acceptance_readiness import stationary_patrol_state


def ready_state():
    state = json.loads((Path(__file__).parent/'data/readiness-after-map-export.json').read_text())['samples'][0]['state']
    # The captured readiness subset predates the client's aggregate flag.
    state['team_ready'] = True
    return state


class Clock:
    def __init__(self):
        self.value = 0.

    def __call__(self):
        return self.value


class Client:
    def __init__(self, clock, states, increments=None):
        self.clock, self.states, self.calls = clock, states, 0
        self.increments = increments or [.25]

    def state(self, timeout):
        self.clock.value += self.increments[min(self.calls, len(self.increments)-1)]
        result = self.states[min(self.calls, len(self.states)-1)]
        self.calls += 1
        return copy.deepcopy(result)

    def request(self, *args, **kwargs):
        raise AssertionError('Preparing a UI route must not publish a command.')


def test_partial_cold_states_are_not_returned_before_all_current_poses_are_stable():
    good = ready_state()
    cold = dict(robots={n: dict(ready=False, pose=None) for n in good['robots']})
    partial = copy.deepcopy(good)
    partial['robots']['robot3']['pose'] = None
    clock = Clock()
    api = Client(clock, [cold, partial, good])
    result = stationary_patrol_state(api, timeout=4., clock=clock)
    assert result == good and api.calls >= 7 and clock.value >= 1.75


@pytest.mark.parametrize('failure', ['missing_member', 'missing_pose', 'bool_pose', 'nonfinite_pose',
    'stale_state', 'unaligned', 'moving_peer', 'active_goal', 'wrong_frame', 'estop', 'not_team_ready'])
def test_unavailable_or_unsafe_inputs_time_out_without_sending_any_mission(failure):
    state = ready_state()
    robot = state['robots']['robot3']
    if failure == 'missing_member':
        del state['robots']['robot3']
    elif failure == 'missing_pose':
        robot['pose'] = None
    elif failure == 'bool_pose':
        robot['pose'][0] = True
    elif failure == 'nonfinite_pose':
        robot['pose'][0] = float('nan')
    elif failure == 'stale_state':
        robot['state_age'] = .81
    elif failure == 'unaligned':
        robot['aligned'] = False
    elif failure == 'moving_peer':
        robot['safety']['v'] = .02
    elif failure == 'active_goal':
        robot['goal'] = robot['pose']
    elif failure == 'wrong_frame':
        robot['frame_epoch'] = 'other-frame'
    elif failure == 'estop':
        state['estop'] = True
    elif failure == 'not_team_ready':
        state['team_ready'] = False
    clock = Clock()
    with pytest.raises(TimeoutError):
        stationary_patrol_state(Client(clock, [state]), timeout=2., clock=clock)


@pytest.mark.parametrize('change', ['identity', 'frame'])
def test_changed_cohort_requires_a_new_continuous_readiness_window(change):
    old = ready_state()
    new = copy.deepcopy(old)
    if change == 'identity':
        new['robots']['robot2']['boot_id'] = 'replacement'
        new['robots']['robot2']['map_epoch'] += 1
    else:
        new['coordinate_frame'] = 'new-frame'
        for robot in new['robots'].values():
            robot['frame_epoch'] = 'new-frame'
    clock = Clock()
    api = Client(clock, [old, old, new])
    assert stationary_patrol_state(api, timeout=4., clock=clock) == new
    assert clock.value >= 1.75


def test_a_ready_response_after_the_monotonic_deadline_is_not_accepted():
    clock = Clock()
    api = Client(clock, [ready_state()], increments=[.1, .1, 1.4])
    with pytest.raises(TimeoutError):
        stationary_patrol_state(api, timeout=1.5, clock=clock)
