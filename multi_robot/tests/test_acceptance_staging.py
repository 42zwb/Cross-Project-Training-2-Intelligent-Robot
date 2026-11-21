"""Blocked fixed seeds must not become automatic fault-staging goals.

Captured inputs are reconstructed map replicas, not the unrecorded exact
rejected grid. They contain raw-observation-derived geometry and estimated
poses only; the independent evaluator's truth is not used for target selection.
"""
from pathlib import Path
import copy
import math
import sys

import numpy as np
import pytest

from fleet_control.geometry import Grid
from fleet_control.peer_protocol import MEMBERS
from fleet_mapping.registration import compose, dynamic_fusion

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_acceptance_staging import staging_plan


def captured():
    grids, poses, seeds = {}, {}, {}
    folder = Path(__file__).parent/'data/staging_route'
    for name in MEMBERS:
        with np.load(folder/(name+'.npz')) as z:
            resolution = float(z['resolution'])
            transform = tuple(z['team_from_local'])
            data, origin = dynamic_fusion([(z['data'], resolution, *z['origin'], 0., transform)], resolution)
            grids[name] = Grid(data, resolution, origin)
            poses[name] = list(compose(transform, z['pose']))
            seeds[name] = z['seed'].tolist()
    return grids, poses, seeds


def open_space():
    grids = {n: Grid(np.zeros((140, 320), np.int16), .05, (-2., -3.5)) for n in MEMBERS}
    seeds = {n: [i*5., 0., 0.] for i, n in enumerate(MEMBERS)}
    poses = {n: [i*5.+.5, 0., 0.] for i, n in enumerate(MEMBERS)}
    return grids, poses, seeds


def verified_plan(grids, poses, seeds):
    plan = staging_plan(grids, poses, seeds)
    assert plan is not None and len(plan) == 3 and {n for n, _ in plan} == set(MEMBERS)
    current, final = copy.deepcopy(poses), {}
    for owner, target in plan:
        grid = grids[owner]
        others = [p for n, p in current.items() if n != owner]
        allowed = grid.allowed(others=others, other_clearance=.9)
        assert grid.astar(current[owner], target, allowed) is not None
        assert grid.free_clearance[grid.cell(target)] >= grid.clearance_radius+2*grid.res
        assert math.dist(target[:2], seeds[owner][:2]) <= .75
        current[owner] = target
        final[owner] = target
    assert min(math.dist(final[a][:2], final[b][:2])
               for i, a in enumerate(MEMBERS) for b in MEMBERS[:i]) >= 3.
    return plan


def test_reconstructed_start_is_valid_but_the_fixed_seed_is_not_a_goal():
    grids, poses, seeds = captured()
    grid = grids['robot1']
    assert grid.data[grid.cell(poses['robot1'])] == 0
    assert grid.free_clearance[grid.cell(poses['robot1'])] >= grid.clearance_radius
    assert grid.free_clearance[grid.cell(seeds['robot1'])] < grid.clearance_radius
    assert grid.astar(poses['robot1'], seeds['robot1'], grid.allowed()) is None


def test_reconstructed_owner_maps_supply_observed_safe_staging_without_changing_seeds():
    grids, poses, seeds = captured()
    before, cells = copy.deepcopy(seeds), {n: g.data.copy() for n, g in grids.items()}
    plan = verified_plan(grids, poses, seeds)
    assert dict(plan)['robot1'] != seeds['robot1']
    assert seeds == before
    assert all(np.array_equal(cells[n], grid.data) for n, grid in grids.items())


def test_valid_original_seeds_remain_the_requested_targets():
    grids, poses, seeds = open_space()
    assert dict(verified_plan(grids, poses, seeds)) == seeds


def test_peer_occupying_another_members_seed_must_move_first():
    grids, poses, seeds = open_space()
    poses['robot1'], poses['robot2'] = [1.1, 0., 0.], [0., 0., 0.]
    plan = verified_plan(grids, poses, seeds)
    assert plan[0][0] == 'robot2'


@pytest.mark.parametrize('value', [-1, 100])
def test_unknown_or_occupied_owner_map_cannot_supply_a_staging_goal(value):
    grids, poses, seeds = open_space()
    grids['robot1'] = Grid(np.full((140, 320), value), .05, (-2., -3.5))
    assert staging_plan(grids, poses, seeds) is None


@pytest.mark.parametrize('value', [-1, 100])
def test_a_wall_or_unknown_strip_cannot_be_crossed_to_reach_a_seed(value):
    grids, poses, seeds = open_space()
    data = grids['robot1'].data.copy()
    data[:, 80:84] = value  # x=2 m separates robot1 from all its seed-near goals.
    grids['robot1'] = Grid(data, .05, (-2., -3.5))
    poses['robot1'] = [3.5, 0., 0.]
    assert staging_plan(grids, poses, seeds) is None


def test_an_occupied_start_is_not_cleared_to_make_staging_possible():
    grids, poses, seeds = open_space()
    grid = grids['robot1']
    data = grid.data.copy()
    data[grid.cell(poses['robot1'])] = 100
    grids['robot1'] = Grid(data, grid.res, grid.origin)
    assert staging_plan(grids, poses, seeds) is None


@pytest.mark.parametrize('failure', ['missing_member', 'nonfinite_pose'])
def test_incomplete_or_invalid_inputs_do_not_create_a_staging_plan(failure):
    grids, poses, seeds = open_space()
    if failure == 'missing_member':
        del grids['robot3']
    else:
        poses['robot1'][0] = math.nan
    assert staging_plan(grids, poses, seeds) is None
