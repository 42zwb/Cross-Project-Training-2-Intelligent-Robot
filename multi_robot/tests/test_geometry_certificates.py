import copy
import numpy as np
import pytest
from fleet_interfaces.msg import RelativeConstraint
from fleet_mapping.registration import Snapshot
from fleet_control.peer_common import set_pose
from fleet_control.peer_geometry_certificates import GeometryCertificates, certificate_digest


def certificates():
    message = RelativeConstraint(session_id='s', robot_id='robot1', boot_id='a',
        constraint_id='already-certified', source_robot='robot1', source_boot='a', source_epoch=1,
        source_revision=10, target_robot='robot2', target_boot='b', target_epoch=1,
        target_revision=8, score=.95, overlap=.8, translation_sigma=.025, rotation_sigma=.02,
        witnesses=['robot1', 'robot2'])
    set_pose(message.transform, (5., 0., 0.))
    a, b = GeometryCertificates('s', 'robot1', 'a', 1), GeometryCertificates('s', 'robot2', 'b', 1)
    for obj, robot, boot, revision in ((a, 'robot1', 'a', 10), (b, 'robot2', 'b', 8)):
        obj.remember(message, Snapshot(robot, boot, 1, revision, np.zeros((20, 20)), .05, (0., 0., 0.)))
    token = certificate_digest(message)
    a.completed([(token, 10, 12, 100_000_000_000)])
    b.completed([(token, 8, 14, 100_000_000_000)])
    return a, b, {'robot1': ('a', 1), 'robot2': ('b', 1)}, {'robot1': 12, 'robot2': 14}


def test_both_endpoints_and_a_real_local_proof_are_required():
    a, b, members, versions = certificates()
    own, remote = next(a.outgoing(100.)), next(b.outgoing(100.))
    a.receive('robot1', own, members, versions, 100.)
    assert a.confirmed_revision(members, 100.) == 0
    a.receive('robot2', remote, members, versions, 100.)
    assert a.confirmed_revision(members, 100.) == 12
    assert a.confirmed_revision(members, 116.) == 0
    assert a.confirmed_revision(dict(members, robot2=('restarted', 2)), 100.) == 0
    a, b, members, versions = certificates()
    own, remote = next(a.outgoing(100.)), next(b.outgoing(100.))
    saved = dict(a.own_results)
    a.own_results.clear()
    own.sequence = 1000
    a.receive('robot1', own, members, versions, 100.)
    a.receive('robot2', remote, members, versions, 100.)
    assert a.confirmed_revision(members, 100.) == 0
    a.own_results = saved
    a.receive('robot1', next(a.outgoing(100.)), members, versions, 100.)
    assert a.confirmed_revision(members, 100.) == 12


def test_retention_work_is_only_for_a_pending_endpoint_with_the_same_identity():
    a, _, members, _ = certificates()
    assert not a.active_references(members, set())
    assert not a.active_references(members, {'robot3'})
    assert a.active_references(members, {'robot2'})
    assert not a.active_references(dict(members, robot2=('replacement', 2)), {'robot2'})


def test_initial_reference_survives_many_recent_certificates_without_unbounded_history():
    a, b, members, _ = certificates()
    first = next(iter(a.certificates))
    initial = copy.deepcopy(a.certificates[first])
    for revision in range(20, 120):
        message = copy.deepcopy(initial)
        message.constraint_id = 'refresh-'+str(revision)
        message.source_revision, message.target_revision = revision, revision-2
        for obj, robot, boot, value in ((a, 'robot1', 'a', revision), (b, 'robot2', 'b', revision-2)):
            obj.remember(message, Snapshot(robot, boot, 1, value, np.zeros((20, 20)), .05, (0., 0., 0.)))
    for obj in (a, b):
        selected = dict(obj.active_references(members, {'robot2'}))
        assert first in selected and len(selected) == 2
        assert len(obj.certificates) <= 4 and len(obj.references) <= 4
    # Retired identities may not reactivate the preserved reference.
    assert not a.active_references(dict(members, robot2=('replacement', 2)), {'robot2'})


@pytest.mark.parametrize('field,value', [
    ('session_id', 'foreign'), ('boot_id', 'retired'), ('map_epoch', 2),
    ('reference_revision', 9), ('revision', 1000),
    ('validated_steady_ns', 84_000_000_000), ('validated_steady_ns', 101_000_000_000),
    ('constraint_digest', 'uncertified-transform'),
])
def test_old_future_or_unbound_proofs_cannot_refresh_a_map_frame(field, value):
    a, b, members, versions = certificates()
    a.receive('robot1', next(a.outgoing(100.)), members, versions, 100.)
    message = next(b.outgoing(100.))
    setattr(message, field, value)
    a.receive('robot2', message, members, versions, 100.)
    assert a.confirmed_revision(members, 100.) == 0


def test_replays_do_not_replace_a_later_validation_and_missing_bases_create_nothing():
    a, b, members, versions = certificates()
    a.receive('robot1', next(a.outgoing(100.)), members, versions, 100.)
    message = next(b.outgoing(100.))
    a.receive('robot2', message, members, versions, 100.)
    old = copy.deepcopy(message)
    old.validated_steady_ns = 99_000_000_000
    a.receive('robot2', old, members, versions, 100.)
    assert a.proofs[message.constraint_digest]['robot2'].validated_steady_ns == 100_000_000_000
    unknown = GeometryCertificates('s', 'robot1', 'a', 1)
    unknown.receive('robot2', message, members, versions, 100.)
    assert not unknown.certificates and not unknown.proofs
    assert unknown.confirmed_revision(members, 100.) == 0
