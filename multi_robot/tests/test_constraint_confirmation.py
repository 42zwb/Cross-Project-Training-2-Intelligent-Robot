"""A good residual alone must not mint the second endpoint's confirmation."""
import copy
from types import SimpleNamespace
import numpy as np
from fleet_interfaces.msg import RelativeConstraint
from fleet_mapping.registration import ConstraintGraph, Snapshot
from fleet_control.local_mapper import LocalMapper
from fleet_control.peer_common import set_pose
from fleet_control.peer_geometry_certificates import GeometryCertificates


def test_receiving_a_proposal_requires_a_fresh_stable_independent_match(monkeypatch):
    monkeypatch.setattr('fleet_control.local_mapper.time.monotonic', lambda: 15.)
    source, target = ('robot1', 'a', 1), ('robot2', 'b', 1)
    peers = {n: (15., SimpleNamespace(boot_id=boot, map_epoch=1, pose=SimpleNamespace(x=0., y=0., theta=0.),
              odom_pose=SimpleNamespace(x=0., y=0., theta=0.), odom_epoch='encoder', odom_valid=True,
              local_ready=True)) for n, boot, _ in (source, target)}
    messages = []
    mapper = SimpleNamespace(session='s', name='robot2', boot='b', epoch=1, peers=peers,
        cfg={'robot_radius': .18, 'max_speed': .2}, graph=ConstraintGraph(), frozen_anchor=None,
        odom_priors={}, continuity_rejections=0, constraints={}, candidates={}, confirmed_local_revision=0,
        registered_local_revision=0,
        local_history={}, geometry=GeometryCertificates('s', 'robot2', 'b', 1),
        own_key=lambda: target, persist=lambda: None,
        constraintpub=SimpleNamespace(publish=lambda m: messages.append(copy.deepcopy(m))),
        maps={n: Snapshot(n, boot, 1, 1, np.zeros((20, 20)), .05, (0., 0., 0.))
              for n, boot, _ in (source, target)})
    msg = RelativeConstraint(session_id='s', robot_id='robot1', boot_id='a', constraint_id='proposal',
        source_robot='robot1', source_boot='a', source_epoch=1, target_robot='robot2', target_boot='b',
        target_epoch=1, score=.95, overlap=.9, translation_sigma=.05, rotation_sigma=.02, witnesses=['robot1'])
    set_pose(msg.transform, (5., 0., 0.))
    LocalMapper.constraint(mapper, 'robot1', msg)
    assert not mapper.graph.edges and not messages
    key = ('robot1', 'a', 1, 'robot2', 'b', 1)
    for candidate in [((5., 0., 0.), 2, 10., 15.), ((5., 0., 0.), 3, 10., 4.),
                      ((-5., 0., 0.), 3, 10., 15.)]:
        mapper.candidates[key] = candidate
        LocalMapper.constraint(mapper, 'robot1', msg)
        assert not mapper.graph.edges and not messages
    mapper.candidates[key] = ((5., 0., 0.), 3, 10., 15.)
    LocalMapper.constraint(mapper, 'robot1', msg)
    assert mapper.graph.edges and set(messages[-1].witnesses) == {'robot1', 'robot2'}
