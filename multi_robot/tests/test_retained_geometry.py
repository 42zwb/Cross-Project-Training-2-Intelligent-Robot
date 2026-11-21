"""Same-frame retention rejects coordinate changes and supports a real recovery."""
from pathlib import Path
import math
import cv2
import numpy as np
from fleet_mapping import registration as reg
from fleet_mapping.registration import Snapshot
from fleet_mapping.continuity import retained_geometry


def geometry():
    data = np.zeros((160, 160), np.int16)
    data[:3] = data[-3:] = 100
    data[:, :3] = data[:, -3:] = 100
    data[30:55, 35:65] = 100
    cv2.circle(data, (110, 80), 12, 100, -1)
    cv2.fillPoly(data, [np.array([[50, 100], [70, 98], [80, 140], [45, 125]])], 100)
    return data


def test_map_growth_retains_the_existing_frame_without_fitting_an_alias():
    data = geometry()
    old = Snapshot('robot1', 'boot', 1, 10, data, .05, (-2., -2., 0.))
    new = Snapshot('robot1', 'boot', 1, 12, np.pad(data, 10, constant_values=-1),
                   .05, (-2.5, -2.5, 0.))
    assert retained_geometry(old, new)
    new.origin = (-2.25, -2.5, 0.)
    assert retained_geometry(old, new) is None
    new.origin, new.boot = (-2.5, -2.5, 0.), 'replacement'
    assert retained_geometry(old, new) is None


def captured():
    folder = Path(__file__).parent/'data/retained_geometry'
    snapshots = {}
    for name in ('robot1', 'robot3'):
        for kind in ('reference', 'current'):
            with np.load(folder/f'{name}-{kind}.npz') as z:
                snapshots[name, kind] = Snapshot(name, str(z['boot']), int(z['epoch']),
                    int(z['revision']), z['data'], float(z['resolution']), tuple(z['origin']))
    return snapshots


def test_real_same_boot_recovery_retains_geometry_without_any_new_registration(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Continuity must retain the certified frame without fitting a new one')

    for method in ('register', '_refine_edges', '_refine_edges_bidirectional'):
        monkeypatch.setattr(reg, method, forbidden)
    snapshots = captured()
    for name in ('robot1', 'robot3'):
        assert retained_geometry(snapshots[name, 'reference'], snapshots[name, 'current'])


def test_current_map_fit_agrees_with_independent_initial_frame_relation():
    snapshots = captured()
    match = reg.register(snapshots['robot1', 'current'], snapshots['robot3', 'current'])
    # One first stationary local-pose/truth sample per robot, before exploration;
    # no alignment of the later trajectory or optimized truth fit. The previous
    # assertion that no fresh match existed encoded the old uncertain-cell bug.
    expected = (10.004647662799302, .0017421236615247082, -.0007325165962097941)
    assert match is not None and match.score >= .80 and match.rms <= .06
    assert math.dist(match.transform[:2], expected[:2]) <= .10
    assert abs(reg.angle(match.transform[2]-expected[2])) <= math.radians(2)
