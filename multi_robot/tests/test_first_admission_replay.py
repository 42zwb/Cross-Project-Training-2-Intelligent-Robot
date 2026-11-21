"""Recorded v24 first admission: continuity revision 80 hid a revision-73 fit."""
import json
import math
from pathlib import Path
import numpy as np
from fleet_control.peer_agent import PeerAgent
from fleet_mapping.registration import Snapshot, register, compose, inverse, angle
from test_alignment_verification import agent

FOLDER = Path(__file__).parent/'data/first_admission_v24'


def test_recorded_new_identity_cannot_be_admitted_by_retention_of_a_pre_stop_fit():
    data = json.loads((FOLDER/'sequence.json').read_text())
    node = agent()
    for sample in data['rows']:
        a = sample['alignment']
        if a['target_revision'] is None:
            continue
        node.pose = sample['pose']
        tr = sample['aggregate_team_transform']
        node.team_transform = lambda tr=tr: tr
        node.mapping[1].revision = a['current_revision']
        node.mapping[2].update(confirmed_local_revision=a['confirmed_revision'],
            registered_local_revision=max(sample['reference_revisions'], default=0))
        if node.alignment_hold is None:
            node.alignment_hold = (sample['steady_time']-a['hold_seconds'], node.pose,
                                   compose(tr, node.pose), a['target_revision'])
        PeerAgent.verify_alignment(node, sample['steady_time'])
        assert not node.alignment_verified
    assert data['rows'][-1]['alignment']['verified'] and data['error_m'] > .1


def test_later_saved_maps_support_a_new_fit_without_changing_the_initial_gauge():
    # These are the first-admission recorder's LATER grids, not the worker's
    # original revision-73 inputs. This test proves a fresh fit is possible;
    # only the next frozen simulation can establish the new admission outcome.
    data = json.loads((FOLDER/'sequence.json').read_text())
    maps = []
    for name in ('robot2', 'robot3'):
        with np.load(FOLDER/f'{name}.npz') as z:
            maps.append(Snapshot(name, str(z['boot']), int(z['epoch']), int(z['revision']),
                z['data'], float(z['resolution']), tuple(z['origin'])))
    match = register(*maps)
    assert match is not None and maps[1].revision >= 75
    expected = compose(inverse(data['expected']['robot2']), data['expected']['robot3'])
    local_pose = data['rows'][-1]['pose']
    predicted, actual = compose(match.transform, local_pose), compose(expected, local_pose)
    assert math.dist(predicted[:2], actual[:2]) < .08
    assert abs(angle(predicted[2]-actual[2])) < math.radians(2)
