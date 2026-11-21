import math
import numpy as np
import yaml
from fleet_control.geometry import peer_obstacle_grid
from fleet_bringup.prepare import nav_config


def test_peer_overlay_is_transparent_and_clears_old_position():
    shape, resolution, origin = (100, 100), .05, (-2.5, -2.5)
    a = peer_obstacle_grid(shape, resolution, origin, [(0., 0.)], .32)
    b = peer_obstacle_grid(shape, resolution, origin, [(1., 0.)], .32)
    assert set(np.unique(a)) == {-1, 100}
    assert a[50, 50] == 100 and b[50, 50] == -1
    assert a[50, 70] == -1 and b[50, 70] == 100
    # No negative cells become free when composing with a shared map.
    underlying = np.full(shape, -1, dtype=np.int8)
    combined = np.where(a == 100, a, underlying)
    assert combined[0, 0] == -1
    assert a[50, 56] == 100
    assert math.sqrt(.325**2 + .025**2) <= .32 + resolution * math.sqrt(2)/2


def test_exclusive_path_can_leave_a_close_stationary_peer_with_full_guard_checks():
    from fleet_control.geometry import Grid, navigation_peer_radius
    from fleet_control.health import peer_motion_clear
    base = np.zeros((120, 120), np.int16)
    own, target = (.63, 0., 0.), (1.2, 0., 0.)
    for exclusive, expected in ((False, False), (True, True)):
        overlay = peer_obstacle_grid(base.shape, .05, (-3., -3.), [(0., 0.)],
                                    navigation_peer_radius(.18, .2, exclusive), base)
        grid = Grid(overlay, .05, (-3., -3.), .18, .35)
        assert bool(grid.astar(own, target, grid.allowed())) is expected
        assert overlay[60, 60] == 100, 'exclusive planning erased a physical peer'
    assert peer_motion_clear(own[:2], 0., .1, 0., [(0., 0., .05)], .18, .2)
    assert not peer_motion_clear(own[:2], math.pi, .1, 0., [(0., 0., .05)], .18, .2)


def test_combined_map_preserves_shared_data_and_does_not_mutate_it():
    shared = np.zeros((100, 100), dtype=np.int16)
    shared[0:20, :] = -1
    shared[80:100, :] = 100
    before = shared.copy()
    result = peer_obstacle_grid(shared.shape, .05, (-2.5, -2.5),
                                [(0., 0.)], .32, shared)
    assert np.array_equal(shared, before)
    no_peer = peer_obstacle_grid(shared.shape, .05, (-2.5, -2.5), [], .32, shared)
    assert np.array_equal(no_peer, before)
    assert np.array_equal(result[:20, :], before[:20, :])
    assert np.array_equal(result[80:, :], before[80:, :])
    assert result[50, 50] == 100 and before[50, 50] == 0


def test_both_nav_costmaps_use_one_combined_navigation_map():
    from pathlib import Path
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / 'src/fleet_bringup/config/fleet.yaml').read_text())
    nav = nav_config('robot1', cfg)['robot1']
    for scope in ('global_costmap', 'local_costmap'):
        params = nav[scope][scope]['ros__parameters']
        assert params['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
        assert params['static_layer']['map_topic'] == '/robot1/navigation_map'
        assert params['use_maximum'] is False
    assert nav['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is True


def test_overlay_publisher_keeps_last_maps_when_peer_pose_is_lost():
    import threading
    from types import SimpleNamespace
    from rclpy.clock import Clock
    from fleet_control.coordinator import Coordinator
    names = ['robot1', 'robot2', 'robot3']
    published = {name: [] for name in names}
    fake = SimpleNamespace(lock=threading.RLock(), names=names,
        cfg={'resolution': .05, 'arena': [-2.5, -2.5, 2.5, 2.5],
             'robot_radius': .18, 'max_speed': .2},
        poses={'robot1': (-1, 0, 0), 'robot2': (0, 0, 0), 'robot3': (1, 0, 0)},
        peer_maps_complete=False,
        grid=SimpleNamespace(data=np.full((100, 100), -1, dtype=np.int8),
                             res=.05, origin=(-2.5, -2.5)),
        peerpub={name:SimpleNamespace(publish=published[name].append) for name in names},
        get_clock=lambda:Clock())
    Coordinator.publish_peer_obstacles(fake)
    first = np.array(published['robot1'][-1].data).reshape(100, 100)
    assert first[50, 50] == 100 and first[50, 70] == 100
    assert first[50, 30] == -1  # Own robot must not be painted.
    fake.poses.pop('robot2')
    fake.poses['robot3'] = (1, 1, 0)
    Coordinator.publish_peer_obstacles(fake)
    assert len(published['robot1']) == 1
    fake.poses['robot2'] = (0, -1, 0)
    Coordinator.publish_peer_obstacles(fake)
    second = np.array(published['robot1'][-1].data).reshape(100, 100)
    assert len(published['robot1']) == 2 and second[50, 50] == -1
    assert second[30, 50] == 100 and second[70, 70] == 100


def test_formation_paths_respect_navigation_peer_clearance():
    from fleet_control.geometry import Grid, formation, plan_formation, navigation_peer_radius
    grid = Grid(np.zeros((100, 100)), .1, (-5, -5))
    poses = [(-3, -3, 0), (0, -3, 0), (3, -3, 0)]
    slots = formation("triangle", 0, 0, 0, 1.2)
    clearance = navigation_peer_radius(.18, .2) + .18 + grid.res * math.sqrt(2) / 2
    result = plan_formation(grid, poses, slots, peer_clearance=clearance)
    assert result
    current = list(poses)
    for index, target, path in result[2]:
        for point in path:
            assert all(math.dist(point, p[:2]) > clearance
                       for j, p in enumerate(current) if j != index)
        current[index] = target
