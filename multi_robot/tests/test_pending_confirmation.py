"""Independent workers finish out of phase without losing the second witness."""
import copy
from types import MethodType, SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from fleet_interfaces.msg import RelativeConstraint
from fleet_control.local_mapper import LocalMapper
from fleet_control.peer_common import set_pose
from fleet_control.peer_geometry_certificates import GeometryCertificates
from fleet_mapping.registration import ConstraintGraph, Match, Snapshot


def pair(monkeypatch):
    clock, delivery, nodes = [100.], [], {}
    monkeypatch.setattr('fleet_control.local_mapper.time.monotonic', lambda: clock[0])
    keys = [('robot1', 'a', 1), ('robot2', 'b', 1)]
    for name, boot, epoch in keys:
        maps = {n: Snapshot(n, b, 1, 1, np.zeros((30, 30)), .05, (0., 0., 0.))
                for n, b, _ in keys}
        node = SimpleNamespace(session='s', name=name, boot=boot, epoch=epoch,
            maps=maps, peers={}, cfg={'robot_radius': .18, 'max_speed': .2},
            graph=ConstraintGraph(), frozen_anchor=None, odom_priors={},
            continuity_rejections=0, constraints={}, candidates={}, confirmed_local_revision=0,
            registered_local_revision=0, admitted=True,
            local_history={1: maps[name]}, geometry=GeometryCertificates('s', name, boot, 1),
            own_key=lambda key=(name, boot, 1): key, persist=lambda: None,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())))
        node.constraint = MethodType(LocalMapper.constraint, node)
        node.confirm_pending = MethodType(LocalMapper.confirm_pending, node)
        node.constraintpub = SimpleNamespace(publish=lambda m, owner=name:
            delivery.append((owner, copy.deepcopy(m))))
        nodes[name] = node

    def refresh(now):
        clock[0] = now
        for node in nodes.values():
            node.peers = {n: (now, SimpleNamespace(boot_id=b, map_epoch=1,
                pose=SimpleNamespace(x=0., y=0., theta=0.),
                odom_pose=SimpleNamespace(x=0., y=0., theta=0.), odom_epoch='encoder',
                odom_valid=True, local_ready=True)) for n, b, _ in keys}

    refresh(clock[0])
    return nodes, delivery, refresh


def test_phase_offset_and_different_fit_digests_still_obtain_both_witnesses(monkeypatch):
    nodes, delivery, refresh = pair(monkeypatch)
    # Each pair completes every 20.2 s; the other endpoint's result is 10.1 s
    # old on receipt. Their 4 mm difference is valid but creates distinct IDs.
    for i in range(3):
        for name, offset in [('robot1', 0.), ('robot2', 10.1)]:
            refresh(100.+i*20.2+offset)
            node = nodes[name]
            a, b = (node.maps[n] for n in ('robot1', 'robot2'))
            match = Match((5.+i*.001+(.004 if name == 'robot2' else 0.), 0., 0.),
                          .95, .9, .03, 30, .03, .02)
            LocalMapper.propose_match(node, a, b, match)
            while delivery:
                sender, message = delivery.pop(0)
                nodes['robot2' if sender == 'robot1' else 'robot1'].constraint(sender, message)
            if i < 2:
                assert all(not n.graph.edges for n in nodes.values())
    assert all(n.graph.edges and n.confirmed_local_revision == 1 for n in nodes.values())
    assert nodes['robot2'].last_registration['pending_endorsed'] == 1
    assert all(any(len(m.witnesses) == 2 for m in n.constraints.values()) for n in nodes.values())


@pytest.mark.parametrize('failure', ['missing_reference', 'future_revision', 'retired_boot',
    'changed_epoch', 'conflicting_transform', 'stale_match', 'too_few_matches', 'too_short_stability'])
def test_reconsideration_preserves_reference_identity_and_independent_match_gates(monkeypatch, failure):
    nodes, delivery, refresh = pair(monkeypatch)
    node = nodes['robot2']
    message = RelativeConstraint(session_id='s', robot_id='robot1', boot_id='a',
        constraint_id='cached-peer-proposal', source_robot='robot1', source_boot='a',
        source_epoch=1, source_revision=1, target_robot='robot2', target_boot='b',
        target_epoch=1, target_revision=1, score=.95, overlap=.9, translation_sigma=.03,
        rotation_sigma=.02, witnesses=['robot1'])
    set_pose(message.transform, (5., 0., 0.))
    node.constraint('robot1', message)
    assert not node.graph.edges
    refresh(150.)
    key = ('robot1', 'a', 1, 'robot2', 'b', 1)
    node.candidates[key] = ((5., 0., 0.), 3, 100., 150.)
    cached = node.constraints[message.constraint_id]
    if failure == 'missing_reference':
        node.local_history.clear()
    elif failure == 'future_revision':
        cached.source_revision = 99
    elif failure == 'retired_boot':
        cached.source_boot = 'retired'
    elif failure == 'changed_epoch':
        cached.source_epoch = 2
    elif failure == 'conflicting_transform':
        set_pose(cached.transform, (5.2, 0., 0.))
    elif failure == 'stale_match':
        node.candidates[key] = ((5., 0., 0.), 3, 100., 139.)
    elif failure == 'too_few_matches':
        node.candidates[key] = ((5., 0., 0.), 2, 100., 150.)
    elif failure == 'too_short_stability':
        node.candidates[key] = ((5., 0., 0.), 3, 149., 150.)
    assert node.confirm_pending(node.maps['robot1'], node.maps['robot2']) == 0
    assert not node.graph.edges and not delivery and node.confirmed_local_revision == 0
