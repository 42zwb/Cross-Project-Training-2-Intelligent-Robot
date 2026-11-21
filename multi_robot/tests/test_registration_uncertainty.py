"""Observed unknown-to-uncertain changes cannot invent geometric conflict.

The grids come from a failed full-map restart. No truth or spawn data is
present. The fixed pose below is a map-only fit, not a claim of truth accuracy.
"""
from dataclasses import replace
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from fleet_mapping.grid import fuse
from fleet_mapping import registration as reg


POSE = (4.801717347107854, .22493424438073567, .04674983467751048)


def snapshots():
    result = []
    folder = Path(__file__).parent/'data/registration_uncertainty'
    for name in ('original', 'uncertain', 'peer'):
        with np.load(folder/(name+'.npz')) as z:
            result.append(reg.Snapshot(str(z['robot']), str(z['boot']), int(z['epoch']),
                int(z['revision']), z['data'], float(z['resolution']), tuple(z['origin'])))
    return result


def passes(stats):
    return bool(stats and stats[0] >= reg.MIN_MATCH_SCORE and stats[2] <= reg.MAX_MATCH_RMS)


@pytest.mark.parametrize('value', [26, 50, 64])
def test_distant_uncertain_cells_leave_definite_geometry_and_residual_unchanged(value):
    original, uncertain, peer = snapshots()
    changed = original.data != uncertain.data
    assert changed.sum() == 696
    assert np.all(original.data[changed] == -1)
    data = uncertain.data.copy()
    data[changed] = value
    uncertain = replace(uncertain, data=data)
    assert np.array_equal(original.data >= 65, uncertain.data >= 65)
    assert np.array_equal((original.data >= 0) & (original.data <= 25),
                          (uncertain.data >= 0) & (uncertain.data <= 25))
    layers = [[(s.data, s.resolution, *s.origin, (0., 0., 0.))]
              for s in (original, uncertain)]
    assert np.array_equal(*(fuse(layer, original.data.shape, original.resolution,
                                 original.origin[:2]) for layer in layers))
    a, b = reg.assess(original, peer, POSE), reg.assess(uncertain, peer, POSE)
    assert passes(a) and passes(b)
    assert a[0] == b[0] and a[2] == b[2]
    assert min(a[1], b[1]) >= .20


def test_complete_registration_is_unchanged_by_distant_uncertain_evidence():
    original, uncertain, peer = snapshots()
    a, b = reg.registration_job(original, peer), reg.registration_job(uncertain, peer)
    assert a['match'] is not None and b['match'] is not None
    assert np.allclose(a['match'].transform, b['match'].transform, atol=1e-12, rtol=0.)
    assert a['match'].score == b['match'].score and a['match'].rms == b['match'].rms
    for result in (a, b):
        trace = result['diagnostics']
        assert trace['reason'] == 'accepted'
        assert trace['hypotheses'] > 0 and trace['assessments']['plausible'] > 0
        assert len(trace['best_evaluated']) <= 4 and len(trace['candidates']) <= 4
        json.dumps(trace, allow_nan=False)


@pytest.mark.parametrize('value', [0, 25])
def test_definite_free_conflicts_are_still_rejected_without_trimming(value):
    original, uncertain, peer = snapshots()
    data = uncertain.data.copy()
    data[original.data != uncertain.data] = value
    assert reg.assess(replace(uncertain, data=data), peer, POSE) is None


@pytest.mark.parametrize('delta', [(.15, 0., 0.), (-.15, 0., 0.),
                                  (0., .15, 0.), (0., 0., math.radians(3))])
def test_wrong_transform_does_not_pass_the_unchanged_precision_gate(delta):
    _, uncertain, peer = snapshots()
    assert not passes(reg.assess(uncertain, peer, reg.compose(delta, POSE)))


def test_small_matching_fragment_cannot_certify_large_uncertain_maps():
    data = np.full((300, 300), 50, np.int16)
    cv2.circle(data, (35, 35), 8, 100, -1)
    cv2.circle(data, (70, 80), 12, 100, -1)
    cv2.rectangle(data, (20, 65), (25, 93), 100, -1)
    source = reg.Snapshot('robot1', 'a', 1, 1, data, .05, (0., 0., 0.))
    extra = data.copy()
    extra[120:270, 120:270] = 100
    target = reg.Snapshot('robot2', 'b', 1, 1, extra, .05, (0., 0., 0.))
    assert reg.distinctive_support(source.occupied(), source.resolution)
    assert passes(reg.assess(source, source, (0., 0., 0.)))
    assert reg.assess(source, target, (0., 0., 0.)) is None


def test_worker_reports_a_rejection_without_manufacturing_a_match():
    data = np.full((40, 40), -1, np.int16)
    empty = reg.Snapshot('robot1', 'a', 1, 1, data, .05, (0., 0., 0.))
    result = reg.registration_job(empty, empty)
    assert result['match'] is None
    assert result['diagnostics']['reason'] == 'insufficient_distinctive_geometry'
    assert result['diagnostics']['candidates'] == []
