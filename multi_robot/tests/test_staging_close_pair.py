"""Exact saved replica maps: a close pair needs one bounded separating move."""
import copy
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from fleet_control.geometry import Grid
from fleet_control.peer_protocol import MEMBERS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_acceptance_staging import (staging_plan, staging_release, release_segment_clear,
                                     release_observation_valid, release_state_valid, release_clock_delay)


def captured():
    grids, poses, seeds = {}, {}, {}
    with np.load(Path(__file__).parent/'data/staging_close_pair.npz') as values:
        for name in MEMBERS:
            grids[name] = Grid(values[name+'_data'], float(values[name+'_resolution']),
                               tuple(values[name+'_origin']))
            poses[name] = values[name+'_pose'].tolist()
            seeds[name] = values[name+'_seed'].tolist()
    return grids, poses, seeds


def space():
    return Grid(np.zeros((140, 320), np.int16), .05, (-2., -3.5))


def test_exact_close_pair_has_targets_but_cannot_step_out_of_the_original_mask():
    grids, poses, seeds = captured()
    assert staging_plan(grids, poses, seeds) is None
    for name in MEMBERS[:2]:
        grid = grids[name]
        root = grid.cell(poses[name])
        others = [p for n, p in poses.items() if n != name]
        mask = grid.allowed(others=others, other_clearance=.9)
        assert grid.data[root] == 0 and grid.free_clearance[root] > .28
        assert not mask[root]
        assert not any(mask[root[0]+dy, root[1]+dx] for dy, dx in
                       ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)))


def test_exact_inputs_supply_one_release_then_unchanged_strict_staging():
    grids, poses, seeds = captured()
    before = copy.deepcopy((poses, seeds))
    cells = {n: g.data.copy() for n, g in grids.items()}
    release = staging_release(grids, poses, seeds)
    assert release is not None and release['owner'] == 'robot1'
    owner, target = release['owner'], release['target']
    assert .05 <= math.dist(poses[owner][:2], target[:2]) <= 1.
    assert release_segment_clear(grids[owner], poses[owner], target, release['others'].values())
    moved = copy.deepcopy(poses)
    moved[owner] = target
    assert release_observation_valid(moved, release)
    assert min(math.dist(target[:2], p[:2]) for p in release['others'].values()) >= 1.25
    plan = staging_plan(grids, moved, seeds)
    assert plan == release['normal_plan'] and len(plan) == 3
    for name, goal in plan:
        grid = grids[name]
        allowed = grid.allowed(others=[p for n, p in moved.items() if n != name], other_clearance=.9)
        assert grid.astar(moved[name], goal, allowed) is not None
        assert math.dist(goal[:2], seeds[name][:2]) <= .75
        moved[name] = goal
    assert (poses, seeds) == before
    assert all(np.array_equal(cells[n], g.data) for n, g in grids.items())


def test_ordinary_valid_plans_do_not_gain_a_temporary_release():
    grids = {n: space() for n in MEMBERS}
    seeds = {n: [i*5., 0., 0.] for i, n in enumerate(MEMBERS)}
    poses = {n: [i*5.+.5, 0., 0.] for i, n in enumerate(MEMBERS)}
    assert staging_plan(grids, poses, seeds) is not None
    assert staging_release(grids, poses, seeds) is None


@pytest.mark.parametrize('value', [-1, 100])
def test_unknown_or_occupied_segment_is_never_cleared(value):
    grid = space()
    data = grid.data.copy()
    data[grid.cell([.5, 0.])] = value
    blocked = Grid(data, grid.res, grid.origin)
    assert not release_segment_clear(blocked, [0., 0., 0.], [1., 0., 0.], [[0., .7, 0.]])


@pytest.mark.parametrize('invalid', ['unsafe_start', 'closer_first', 'other_peer_disk',
                                    'overlong', 'arrival_margin', 'nonfinite'])
def test_a_retreat_must_preserve_every_segment_and_endpoint_condition(invalid):
    start, target, others = [0., 0., 0.], [1., 0., 0.], [[0., .7, 0.]]
    if invalid == 'unsafe_start':
        others[0][1] = .59
    elif invalid == 'closer_first':
        target = [math.sqrt(1.-.02**2), .02, 0.]
    elif invalid == 'other_peer_disk':
        others.append([.15, -.89, 0.])
    elif invalid == 'overlong':
        target[0] = 1.1
    elif invalid == 'arrival_margin':
        target[0] = .9
    elif invalid == 'nonfinite':
        target[0] = math.nan
    assert not release_segment_clear(space(), start, target, others)


@pytest.mark.parametrize('failure', ['missing_member', 'invalid_pose', 'no_observed_map', 'occupied_start'])
def test_incomplete_or_unobserved_geometry_does_not_create_a_release(failure):
    grids, poses, seeds = captured()
    if failure == 'missing_member':
        del poses['robot3']
    elif failure == 'invalid_pose':
        poses['robot1'][0] = math.nan
    elif failure == 'no_observed_map':
        grids = {n: Grid(np.full_like(g.data, -1), g.res, g.origin) for n, g in grids.items()}
    elif failure == 'occupied_start':
        for n in MEMBERS[:2]:
            g = grids[n]
            data = g.data.copy()
            data[g.cell(poses[n])] = 100
            grids[n] = Grid(data, g.res, g.origin)
    assert staging_release(grids, poses, seeds) is None


def release_state():
    grids, poses, seeds = captured()
    release = staging_release(grids, poses, seeds)
    identities = {n: ('boot-'+n, 1) for n in MEMBERS}
    state = dict(coordinate_frame='team', robots={n: dict(pose=p, boot_id=identities[n][0],
        map_epoch=1, frame_epoch='team', ready=True, aligned=True, nav_ready=True,
        state_age=.1, pose_steady_time=9.9, goal=None, safety=dict(v=0., w=0.)) for n, p in poses.items()})
    return release, identities, state


def test_live_departure_accepts_fresh_stationary_cohort_and_the_reached_endpoint():
    release, identities, state = release_state()
    assert release_state_valid(state, release, identities, 'team', 10.)
    state['robots'][release['owner']]['pose'] = release['target']
    assert release_state_valid(state, release, identities, 'team', 10.)


@pytest.mark.parametrize('failure', ['closer', 'corridor', 'peer_moves', 'peer_goal', 'peer_velocity',
    'retired_boot', 'changed_frame', 'peer_frame', 'stale_state', 'frozen_pose', 'unaligned', 'nan_age', 'estop'])
def test_live_departure_rejects_motion_or_context_that_invalidates_the_release(failure):
    release, identities, state = release_state()
    owner = state['robots'][release['owner']]
    peer = state['robots']['robot2']
    if failure == 'closer':
        owner['pose'] = [owner['pose'][0], owner['pose'][1]+.08, owner['pose'][2]]
    elif failure == 'corridor':
        owner['pose'] = [owner['pose'][0]+.2, owner['pose'][1], owner['pose'][2]]
    elif failure == 'peer_moves':
        peer['pose'] = [peer['pose'][0]+.06, *peer['pose'][1:]]
    elif failure == 'peer_goal':
        peer['goal'] = peer['pose']
    elif failure == 'peer_velocity':
        peer['safety']['v'] = .05
    elif failure == 'retired_boot':
        peer['boot_id'] = 'other-instance'
    elif failure == 'changed_frame':
        state['coordinate_frame'] = 'other-team'
    elif failure == 'peer_frame':
        peer['frame_epoch'] = 'other-team'
    elif failure == 'stale_state':
        peer['state_age'] = .81
    elif failure == 'frozen_pose':
        owner['pose_steady_time'] = 9.
    elif failure == 'unaligned':
        owner['aligned'] = False
    elif failure == 'nan_age':
        peer['state_age'] = math.nan
    elif failure == 'estop':
        state['estop'] = True
    assert not release_state_valid(state, release, identities, 'team', 10.)


def test_exact_future_pose_snapshot_passes_only_after_the_bounded_clock_wait():
    captured = json.loads((Path(__file__).parent/'data/staging_release_future_pose.json').read_text())
    state, release, identities, frame, now = (captured[k] for k in
        ('state', 'release', 'identities', 'frame', 'now'))
    assert not release_state_valid(state, release, identities, frame, now)
    delay = release_clock_delay(state, release, identities, frame, now)
    assert delay == pytest.approx(.002401013)
    assert not release_state_valid(state, release, identities, frame, now+delay/2)
    assert release_state_valid(state, release, identities, frame, now+delay)
    assert release_clock_delay(state, release, identities, frame, now+delay) == 0.


@pytest.mark.parametrize('lead', [.001, .01, .049, .05])
def test_only_small_future_pose_skew_can_request_one_wait(lead):
    release, identities, state = release_state()
    state['robots']['robot3']['pose_steady_time'] = 10.+lead
    assert not release_state_valid(state, release, identities, 'team', 10.)
    delay = release_clock_delay(state, release, identities, 'team', 10.)
    assert delay == pytest.approx(lead)
    assert release_state_valid(state, release, identities, 'team', 10.+delay)


@pytest.mark.parametrize('failure', ['already_current', 'large_future', 'stale_after_wait', 'state_ages_out',
    'frozen_pose', 'missing_stamp', 'nan_stamp', 'bool_stamp', 'missing_member',
    'retired_boot', 'moving_peer', 'corridor', 'estop', 'nan_now'])
def test_clock_wait_cannot_bypass_another_guard_or_extend_freshness(failure):
    release, identities, state = release_state()
    now = 10.
    peer = state['robots']['robot3']
    peer['pose_steady_time'] = now+.05
    if failure == 'already_current':
        peer['pose_steady_time'] = now-.1
    elif failure == 'large_future':
        peer['pose_steady_time'] = now+.050001
    elif failure == 'stale_after_wait':
        state['robots']['robot1']['pose_steady_time'] = now-.78
    elif failure == 'state_ages_out':
        state['robots']['robot1']['state_age'] = .78
    elif failure == 'frozen_pose':
        state['robots']['robot1']['pose_steady_time'] = now-1.
    elif failure == 'missing_stamp':
        del peer['pose_steady_time']
    elif failure == 'nan_stamp':
        peer['pose_steady_time'] = math.nan
    elif failure == 'bool_stamp':
        peer['pose_steady_time'] = True
    elif failure == 'missing_member':
        del state['robots']['robot2']
    elif failure == 'retired_boot':
        peer['boot_id'] = 'old'
    elif failure == 'moving_peer':
        peer['safety']['v'] = .01
    elif failure == 'corridor':
        owner = state['robots'][release['owner']]
        owner['pose'] = [owner['pose'][0]+.2, *owner['pose'][1:]]
    elif failure == 'estop':
        state['estop'] = True
    elif failure == 'nan_now':
        now = math.nan
    assert release_clock_delay(state, release, identities, 'team', now) == 0.


def test_elapsed_clock_wait_counts_against_the_original_state_freshness_limit():
    release, identities, state = release_state()
    state['robots']['robot2']['state_age'] = .78
    assert release_state_valid(state, release, identities, 'team', 10.)
    assert not release_state_valid(state, release, identities, 'team', 10.05, state_elapsed=.05)
