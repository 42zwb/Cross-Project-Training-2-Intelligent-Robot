import copy
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from types import SimpleNamespace
from fleet_control.geometry import Grid, formation, plan_formation, region_return
from fleet_control.peer_plans import validate_formation_plan, agreed_formation_plan
from fleet_control.peer_protocol import digest
from fleet_control.local_safety import inside_region
from fleet_control.peer_persistence import Journal
from fleet_bringup.peer_restart import advance_identity, selected
from fleet_bringup.simulation_preflight import Process


def test_formation_stops_when_peer_pose_is_valid_but_its_navigation_has_failed():
    from fleet_control.peer_agent import PeerAgent
    agent = SimpleNamespace(all_aligned=lambda: True, work={'kind': 'formation'},
        live=lambda: {'robot2': (1., SimpleNamespace(local_ready=True, nav_ready=False), {})})
    agent.cancel = lambda: setattr(agent, 'work', None)
    PeerAgent.formation_tick(agent, 10.)
    assert agent.work is None and '导航可用' in agent.message


def test_region_return_uses_observed_boundary_when_initial_seed_is_unknown():
    data = np.zeros((100, 100), np.int16)
    data[:, 80:] = -1
    grid = Grid(data, .05, (-2.5, -2.5), .18, .35)
    yy, xx = np.indices(data.shape)
    mask = xx >= 65
    target = region_return(grid, (-1., 0., 0.), mask)
    assert target is not None and target[0] >= .75
    assert grid.data[grid.cell(target)] == 0
    data[:, 60:65] = 100
    blocked = Grid(data, .05, (-2.5, -2.5), .18, .35)
    assert region_return(blocked, (-1., 0., 0.), mask) is None


def test_out_of_region_peer_stops_region_renewals_until_it_has_returned():
    from fleet_control.peer_agent import PeerAgent
    poses = {'robot1': [0., 0., 0.], 'robot2': [5., 0., 0.], 'robot3': [5.5, 0., 0.]}
    agent = SimpleNamespace(mode='EXPLORING', unrestricted=False, all_aligned=lambda: True,
        seeds={'robot1': [0., 0., 0.], 'robot2': [5., 0., 0.], 'robot3': [10., 0., 0.]},
        regions=None, cfg={'robot_radius': .18}, team_poses=lambda: poses,
        work={'kind': 'explore'}, resource='region/robot1')
    agent.homing_pending = lambda: PeerAgent.homing_pending(agent)
    assert agent.homing_pending()
    assert not PeerAgent.motion_turn(agent, 10.)
    poses['robot3'] = [9., 0., 0.]
    assert not agent.homing_pending()
    assert PeerAgent.motion_turn(agent, 10.)


def test_formation_agreement_survives_reorder_and_waits_for_all_validations():
    boots = {n: 'boot-'+n for n in ('robot1', 'robot2', 'robot3')}
    a, b = {'path': 'a'}, {'path': 'b'}
    proposals = {'robot3': None, 'robot2': b, 'robot1': a}
    votes = {digest(b): {n: (boot, True) for n, boot in boots.items()}}
    assert agreed_formation_plan(proposals, votes, boots) is None
    votes[digest(a)] = {n: (boot, n != 'robot3') for n, boot in boots.items()}
    assert agreed_formation_plan(proposals, votes, boots) == (digest(b), b)
    votes[digest(a)]['robot3'] = ('retired', False)
    assert agreed_formation_plan(proposals, votes, boots) is None
    votes[digest(a)]['robot3'] = (boots['robot3'], True)
    assert agreed_formation_plan(proposals, votes, boots) == (digest(a), a)
    assert agreed_formation_plan({'robot2': b}, votes, boots) is None


def test_every_formation_voter_checks_routes_obstacles_and_assignment():
    data = np.zeros((160, 160), np.int8)
    data[:2] = data[-2:] = 100
    data[:, :2] = data[:, -2:] = 100
    grid = Grid(data, .1, (-8., -8.), .18, .1)
    poses = {'robot1': (-4., -3., 0.), 'robot2': (0., -3., 0.), 'robot3': (4., -3., 0.)}
    cmd = dict(shape='triangle', x=0., y=0., yaw=0., spacing=1.2)
    slots = formation(**dict(shape=cmd['shape'], x=cmd['x'], y=cmd['y'], heading=cmd['yaw'], spacing=cmd['spacing']))
    result = plan_formation(grid, list(poses.values()), slots, peer_clearance=.85)
    assert result is not None
    _, assignment, routes = result
    plan = dict(slots=[list(p) for p in slots], assignment=list(assignment),
                routes=[[i, list(target), [list(p) for p in path]] for i, target, path in routes])
    assert validate_formation_plan(plan, cmd, poses, grid, (0., 0., 0.))
    bad = copy.deepcopy(plan)
    bad['routes'][0][2].insert(1, list(poses['robot2'][:2]))
    assert not validate_formation_plan(bad, cmd, poses, grid, (0., 0., 0.))
    bad = copy.deepcopy(plan)
    bad['assignment'] = [0, 0, 2]
    assert not validate_formation_plan(bad, cmd, poses, grid, (0., 0., 0.))
    bad = copy.deepcopy(plan)
    bad['routes'][0][2].append([40., 40.])
    assert not validate_formation_plan(bad, cmd, poses, grid, (0., 0., 0.))


@pytest.mark.parametrize('limits', [
    {'region_enabled': True, 'regions': [[0, 0, 1, 1]]},
    {'region_enabled': True, 'regions': [[0, 0, float('nan'), 1]]*3},
    {'region_enabled': True, 'seeds': {'robot1': [0, 0, 0]}},
])
def test_malformed_region_never_crashes_or_authorizes_guard(limits):
    assert not inside_region('robot1', (0., 0., 0.), limits, .2)


def test_estop_and_command_fence_survive_same_session_restart(tmp_path):
    first = Journal(tmp_path, 'robot2', 'safety', 'run-a')
    first.update(estop=True, command_order=[10, 'operator', 'stop'])
    second = Journal(tmp_path, 'robot2', 'safety', 'run-a')
    assert second.data['estop'] and second.data['command_order'][2] == 'stop'
    assert Journal(tmp_path, 'robot2', 'safety', 'run-b').data == {}


def test_restart_selection_never_touches_world_other_robot_or_other_project():
    def process(pid, **env):
        return Process(pid, 1, '123', ('python3',), env, '/tmp', 'S')
    scope = dict(TB4_SIM_SESSION='run', TB4_SIM_PROJECT='fleet',
                 TB4_SIM_ROBOT='robot2', TB4_SIM_COMPONENT='autonomy')
    processes = {1: process(1, **scope),
        2: process(2, **dict(scope, TB4_SIM_ROBOT='robot1')),
        3: process(3, **dict(scope, TB4_SIM_COMPONENT='world')),
        4: process(4, **dict(scope, TB4_SIM_SESSION='other')),
        5: process(5, **dict(scope, TB4_SIM_PROJECT='single'))}
    assert [p.pid for p in selected(processes, 'run', 'robot2')] == [1]


def test_restart_changes_boot_and_map_frame_without_touching_other_assets(tmp_path):
    (tmp_path/'agents').mkdir()
    cfg = dict(robot_id='robot2', assets=str(tmp_path), boot_id='old', map_epoch=1)
    path = tmp_path/'agents/robot2.yaml'
    path.write_text(yaml.safe_dump(cfg))
    (tmp_path/'robot2.lua').write_text('map_frame="robot2/map",tracking_frame="robot2/base_link"')
    (tmp_path/'robot2_nav.yaml').write_text(yaml.safe_dump({'global_frame': 'robot2/map', 'odom_frame': 'robot2/odom'}))
    other = tmp_path/'robot1.lua'
    other.write_text('unchanged')
    updated = advance_identity(path)
    assert updated['boot_id'] != 'old' and updated['map_epoch'] == 2
    assert 'map_frame="robot2/map_e2"' in (tmp_path/'robot2.lua').read_text()
    assert yaml.safe_load((tmp_path/'robot2_nav.yaml').read_text())['global_frame'] == 'robot2/map_e2'
    assert other.read_text() == 'unchanged'
