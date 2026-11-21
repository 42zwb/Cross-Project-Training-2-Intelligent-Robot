"""Captured goal rejection: an escapable start is not a valid return goal."""
from pathlib import Path
import sys

import numpy as np
import pytest
from fleet_control.geometry import Grid, formation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from peer_acceptance_routes import formation_target, short_patrol


def captured(name):
    with np.load(Path(__file__).parent/'data/patrol_route'/(name+'.npz')) as data:
        grid = Grid(data['data'], float(data['resolution']), tuple(data['origin']), .18, .35)
        poses, old_route = data['poses'].tolist(), data['old_route'].tolist()
    index = int(name[-1])-1
    return grid, poses[index], [p for i, p in enumerate(poses) if i != index], old_route


def test_captured_outbound_success_does_not_validate_return_to_escaped_start():
    grid, pose, peers, route = captured('robot1')
    allowed = grid.allowed(others=peers, other_clearance=.85)
    assert grid.clearance[grid.cell(pose)] < grid.clearance_radius
    assert grid.astar(pose, route[0], allowed) is not None
    assert grid.astar(route[0], pose, allowed) is None


@pytest.mark.parametrize('name', ['robot1', 'robot2', 'robot3'])
def test_captured_replica_routes_validate_every_leg_and_both_goal_endpoints(name):
    grid, pose, peers, _ = captured(name)
    route = short_patrol(grid, pose, peers)
    assert route is not None and len(route) == 2
    allowed = grid.allowed(others=peers, other_clearance=.85)
    previous = pose
    for goal in route:
        assert allowed[grid.cell(goal)]
        assert grid.free_clearance[grid.cell(goal)] >= grid.clearance_radius+2*grid.res
        assert grid.astar(previous, goal, allowed) is not None
        previous = goal
    if name == 'robot1':
        assert route[-1] != pose


def test_open_space_patrol_keeps_an_already_safe_original_return_pose():
    grid = Grid(np.zeros((100, 100)), .05, (0., 0.), .18, .35)
    pose = [2., 2., 0.]
    route = short_patrol(grid, pose)
    assert route is not None and route[-1] == pose


@pytest.mark.parametrize('value', [-1, 100])
def test_unobserved_or_occupied_area_cannot_supply_a_patrol(value):
    grid = Grid(np.full((100, 100), value), .05, (0., 0.), .18, .35)
    assert short_patrol(grid, [2., 2., 0.]) is None


def test_peer_obstacle_cannot_be_crossed_to_find_a_return_anchor():
    grid = Grid(np.zeros((100, 100)), .05, (0., 0.), .18, .35)
    pose = [2., 2., 0.]
    assert short_patrol(grid, pose, [pose], other_clearance=.9) is None


def test_formation_target_moves_when_one_replica_blocks_a_slot():
    poses = {'robot1': [-.35, -.65, 0.], 'robot2': [-.35, .65, 0.], 'robot3': [.75, 0., 0.]}
    blank = np.zeros((120, 120), dtype=np.int16)
    open_grid = Grid(blank, .05, (-3., -3.))
    replicas = dict.fromkeys(poses, open_grid)
    old = formation_target(replicas, poses, 'row')
    assert old is not None
    blocked = blank.copy()
    blocked[open_grid.cell((old['x'], old['y']-old['spacing']))] = 100
    replicas['robot2'] = Grid(blocked, .05, (-3., -3.))

    new = formation_target(replicas, poses, 'row')
    assert new is not None and new != old
    slots = formation(new['shape'], new['x'], new['y'], new['yaw'], new['spacing'])
    assert all(replicas['robot2'].free_clearance[open_grid.cell(slot)] >= .40 for slot in slots)
