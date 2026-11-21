"""Captured partial-map restart: preserve precision and ambiguous alternatives.

Fixtures contain only the two original local grids. The expected transform is
an offline check from one initial anchor sample, never an agent input.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from fleet_mapping import registration as reg


def snapshots():
    folder = Path(__file__).parent/'data/restart_refinement'
    result = []
    for name in ('robot1', 'robot2'):
        with np.load(folder/(name+'.npz')) as data:
            result.append(reg.Snapshot(name, str(data['boot']), int(data['epoch']),
                int(data['revision']), data['data'], float(data['resolution']), tuple(data['origin'])))
    return result


@pytest.mark.parametrize('reverse', [False, True])
def test_real_partial_restart_map_passes_original_precision_gate_in_both_orders(reverse):
    a, b = snapshots()
    expected = (4.731242930708085, .6657968585535485, .15281669630503564)
    if reverse:
        a, b, expected = b, a, reg.inverse(expected)
    match = reg.register(a, b)
    assert match is not None
    assert match.score >= .80 and match.rms <= .06 and match.overlap >= .20
    assert math.dist(match.transform[:2], expected[:2]) < .10
    assert abs(reg.angle(match.transform[2]-expected[2])) < math.radians(2)


def test_failed_joint_refinement_does_not_erase_a_plausible_rival(monkeypatch):
    a, b = snapshots()
    monkeypatch.setattr(reg, 'distinctive_support', lambda *args: True)
    monkeypatch.setattr(reg, 'intrinsic_ambiguity', lambda snapshot: False)
    monkeypatch.setattr(reg, '_features', lambda snapshot: ([], None))
    monkeypatch.setattr(reg, '_geometry_hypotheses', lambda a, b: [(0., 0., 0.), (1., 0., 0.)])
    monkeypatch.setattr(reg, '_refine_edges', lambda source, target, pose: pose)
    monkeypatch.setattr(reg, '_refine_edges_bidirectional',
        lambda source, target, pose: pose if pose[0] == 0. else (99., 0., 0.))
    monkeypatch.setattr(reg, 'assess', lambda a, b, pose:
        (.84 if pose[0] == 0. else .80, .8, .03) if pose[0] < 2. else None)
    assert reg.register(a, b) is None
