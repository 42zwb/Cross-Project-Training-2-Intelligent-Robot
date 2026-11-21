"""A real returned robot must be able to start its own regional routes."""
from pathlib import Path
import math
import threading
from types import SimpleNamespace
import numpy as np
from fleet_control.geometry import Grid, region_return, region_entry_margin
from fleet_control.peer_agent import PeerAgent
from fleet_control.local_safety import inside_region
from fleet_mapping.registration import compose, inverse, voronoi_mask


def case():
    a = np.load(Path(__file__).parent/'data/region_reentry/robot3.npz')
    grid = Grid(a['data'], float(a['resolution']), tuple(a['origin']), .18, .35)
    seeds = {n: list(p) for n, p in zip(('robot1', 'robot2', 'robot3'), a['seeds'])}
    pose, tr = tuple(a['pose']), tuple(a['transform'])
    local = {n: compose(inverse(tr), p) for n, p in seeds.items()}
    mask = voronoi_mask(grid.data.shape, grid.res, grid.origin, local, 'robot3', grid.region_clearance)
    return grid, seeds, pose, tr, mask


def fake_agent(grid, seeds, pose, tr):
    return SimpleNamespace(lock=threading.RLock(), grid=grid, pose=pose, mode='EXPLORING',
        raw_grid=None, aligned=lambda: True, all_aligned=lambda: True, limits=lambda: {},
        seeds=seeds, regions=None, unrestricted=False, travel=17., regional_travel=0.,
        blacklist=[], visits=[], team_transform=lambda: tr, name='robot3', team_initialized=True,
        cfg={'robot_radius': .18, 'region_margin': .35, 'resolution': grid.res})


def test_real_boundary_stop_cannot_be_declared_an_exhausted_region():
    grid, seeds, pose, tr, mask = case()
    assert inside_region('robot3', compose(tr, pose), dict(region_enabled=True, seeds=seeds), .20)
    data = grid.data.copy()
    data[~mask] = 100
    regional = Grid(data, grid.res, grid.origin, .18, .35)
    assert regional.route_field(pose, None) is None  # The reproduced old dead end.
    node = fake_agent(grid, seeds, pose, tr)
    result = PeerAgent.candidate(node)
    assert result and result.get('kind') == 'return' and result['resource'] == 'team_motion'


def test_return_remains_usable_when_nav2_stops_eight_centimetres_short():
    grid, seeds, pose, tr, mask = case()
    target = region_return(grid, pose, mask)
    assert target is not None
    direction = np.asarray(target[:2])-pose[:2]
    stopped = np.asarray(target[:2])-.08*direction/np.linalg.norm(direction)
    pose = (float(stopped[0]), float(stopped[1]), target[2])
    assert inside_region('robot3', compose(tr, pose), dict(region_enabled=True, seeds=seeds),
                         region_entry_margin(.18, .35, grid.res))
    node = fake_agent(grid, seeds, pose, tr)
    node.visits = [target]
    result = PeerAgent.candidate(node)
    assert result and result.get('kind') == 'explore' and result['resource'] == 'region/robot3'


def test_known_map_still_supports_required_independent_viewpoint_participation():
    grid = Grid(np.zeros((120, 120), np.int16), .05, (-3., -3.))
    assert grid.frontiers((0., 0., 0.), None) is None
    result = grid.frontiers((0., 0., 0.), None, min_travel=True)
    assert result and math.dist((0., 0.), result[1]) >= 1.2


def test_peers_wait_until_the_returned_robot_can_start_a_regional_path():
    grid, seeds, pose, tr, mask = case()
    poses = dict(seeds)
    poses['robot3'] = compose(tr, pose)
    node = SimpleNamespace(mode='EXPLORING', unrestricted=False, all_aligned=lambda: True,
        seeds=seeds, regions=None, cfg={'robot_radius': .18, 'region_margin': .35,
        'resolution': grid.res}, team_poses=lambda: poses)
    assert PeerAgent.homing_pending(node)
    poses['robot3'] = seeds['robot3']
    assert not PeerAgent.homing_pending(node)
