"""Actual repeated-corner false match captured during a stationary restart.

Only original local occupancy grids are stored. They contain no simulator
geometry, spawn coordinates or truth pose, and are never loaded by agents.
"""
from pathlib import Path
import numpy as np
from fleet_mapping.registration import Snapshot, register, intrinsic_ambiguity, anchor_key, distinctive_support


def test_first_component_anchor_can_have_restarted_before_its_initial_alignment():
    root = ('robot2', 'boot-after-restart', 2)
    assert anchor_key({root: (0., 0., 0.)}, 'robot2', 'boot-after-restart') == root
    assert anchor_key({root: (0., 0., 0.)}, 'robot2', 'retired') is None


def test_repeated_disconnected_rooms_are_not_made_unique_by_raster_noise():
    folder = Path(__file__).parent/'data/separated_rooms'
    snapshots = []
    for name in ('robot2', 'robot3'):
        with np.load(folder/(name+'.npz')) as data:
            snapshots.append(Snapshot(name, str(data['boot']), int(data['epoch']), int(data['revision']),
                                      data['data'], float(data['resolution']), tuple(data['origin'])))
    assert all(intrinsic_ambiguity(snapshot) for snapshot in snapshots)
    assert register(*snapshots) is None


def test_real_repeated_corner_does_not_become_a_unique_alignment():
    folder = Path(__file__).parent/'data/registration_alias'
    snapshots = []
    for name in ('robot2', 'robot3'):
        with np.load(folder/(name+'.npz')) as data:
            snapshots.append(Snapshot(name, str(data['boot']), int(data['epoch']), int(data['revision']),
                                      data['data'], float(data['resolution']), tuple(data['origin'])))
    assert register(*snapshots) is None


def test_opposite_visible_corners_of_empty_square_do_not_identify_the_room():
    folder = Path(__file__).parent/'data/symmetric_square'
    snapshots = []
    for name in ('robot1', 'robot3'):
        with np.load(folder/(name+'.npz')) as data:
            snapshots.append(Snapshot(name, str(data['boot']), int(data['epoch']), int(data['revision']),
                                      data['data'], float(data['resolution']), tuple(data['origin'])))
    assert all(not distinctive_support(s.occupied(edges=True)) for s in snapshots)
    assert register(*snapshots) is None


def test_corner_raster_noise_and_one_robot_return_cannot_manufacture_distinctiveness():
    from fleet_mapping.registration import transform_points
    rng = np.random.default_rng(7)
    wall = np.linspace(0., 6., 300)
    points = np.vstack([np.column_stack([wall, np.zeros_like(wall)]),
                        np.column_stack([np.zeros_like(wall), wall]),
                        rng.normal((2., 3.), .04, (40, 2))])
    points += rng.normal(0., .015, points.shape)
    assert not distinctive_support(transform_points((5., -4., .73), points))
