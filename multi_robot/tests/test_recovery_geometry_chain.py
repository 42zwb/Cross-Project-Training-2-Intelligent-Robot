"""Real recovery maps through workers, peer proofs, mapper and stationary readiness."""
import copy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from fleet_interfaces.msg import RelativeConstraint
from fleet_mapping.registration import ConstraintGraph, Snapshot
from fleet_mapping.continuity import retained_geometry_job
from fleet_control.local_mapper import LocalMapper
from fleet_control.peer_agent import PeerAgent
from fleet_control.peer_common import set_pose
from fleet_control.peer_geometry_certificates import GeometryCertificates


def recovery(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('fleet_control.local_mapper.time.monotonic', lambda: clock[0])
    monkeypatch.setattr('fleet_mapping.continuity.time.monotonic_ns', lambda: int(clock[0]*1e9))
    snapshots = {}
    for name in ('robot1', 'robot3'):
        for kind in ('reference', 'current'):
            with np.load(Path(__file__).parent/'data/retained_geometry'/f'{name}-{kind}.npz') as z:
                snapshots[name, kind] = Snapshot(name, str(z['boot']), int(z['epoch']),
                    int(z['revision']), z['data'], float(z['resolution']), tuple(z['origin']))
    a, b = (snapshots[n, 'reference'] for n in ('robot1', 'robot3'))
    message = RelativeConstraint(session_id='s', robot_id=a.robot, boot_id=a.boot,
        constraint_id='existing-bilateral-frame', source_robot=a.robot, source_boot=a.boot,
        source_epoch=a.epoch, source_revision=a.revision, target_robot=b.robot,
        target_boot=b.boot, target_epoch=b.epoch, target_revision=b.revision,
        score=.95, overlap=.8, translation_sigma=.025, rotation_sigma=.02,
        witnesses=['robot1', 'robot3'])
    set_pose(message.transform, (10.0361226935, .01261589, -.00282462))
    members = {n: (snapshots[n, 'current'].boot, snapshots[n, 'current'].epoch) for n in ('robot1', 'robot3')}
    geometry = {}
    for name in members:
        obj = GeometryCertificates('s', name, *members[name])
        obj.remember(message, snapshots[name, 'reference'])
        geometry[name] = obj
    graph = ConstraintGraph()
    graph.add((a.robot, a.boot, a.epoch), (b.robot, b.boot, b.epoch), (10.0361226935, .01261589, -.00282462))
    mapper = SimpleNamespace(geometry=geometry['robot3'], confirmed_local_revision=0,
        maps={n: snapshots[n, 'current'] for n in members},
        geometry_members=lambda now: members)
    target = snapshots['robot3', 'current'].revision
    node = SimpleNamespace(map_aligned=lambda: True, local_ready=True, stopped=lambda now: True,
        pose=(0., 0., 0.), alignment_verified=False, alignment_ever_verified=True, alignment_hold=None, verified_at=None,
        team_transform=lambda: (0., 0., 0.), mapping=(100., SimpleNamespace(revision=target),
            {'confirmed_local_revision': 0}), pose_sim_time=0., pose_steady_time=100.,
        record=lambda *a, **k: None)
    return clock, snapshots, geometry, members, graph, mapper, node


def compute(name, snapshots, geometry, members, graph, now):
    obj = geometry[name]
    result = retained_geometry_job(snapshots[name, 'current'], obj.active_references(members, {'robot3'}),
        graph, [], now, .35, {n: snapshots[n, 'current'] for n in members}, True)
    obj.completed(result['proofs'])
    return result


def test_real_same_boot_recovery_requires_both_proofs_before_stationary_ready(monkeypatch):
    clock, snaps, geometry, members, graph, mapper, node = recovery(monkeypatch)
    old_edges = copy.deepcopy(graph.edges)
    PeerAgent.verify_alignment(node, clock[0])
    for name in members:
        result = compute(name, snaps, geometry, members, graph, clock[0])
        assert result['proofs'] and all(c['reason'] == 'accepted' for c in result['checks'])
    LocalMapper.continuity(mapper, 'robot3', next(geometry['robot3'].outgoing(clock[0])))
    assert mapper.confirmed_local_revision == 0
    clock[0] += 3.1
    PeerAgent.verify_alignment(node, clock[0])
    assert not node.alignment_verified
    LocalMapper.continuity(mapper, 'robot1', next(geometry['robot1'].outgoing(clock[0])))
    node.mapping[2]['confirmed_local_revision'] = mapper.confirmed_local_revision
    PeerAgent.verify_alignment(node, clock[0])
    assert node.alignment_verified
    assert mapper.confirmed_local_revision >= node.mapping[1].revision
    assert graph.edges == old_edges  # Continuity did not fit, create or move an edge.


def test_map_must_arrive_before_a_proof_and_retransmission_can_recover_ordering(monkeypatch):
    clock, snaps, geometry, members, graph, mapper, node = recovery(monkeypatch)
    for name in members:
        compute(name, snaps, geometry, members, graph, clock[0])
    LocalMapper.continuity(mapper, 'robot3', next(geometry['robot3'].outgoing(clock[0])))
    mapper.maps['robot1'] = snaps['robot1', 'reference']
    proof = next(geometry['robot1'].outgoing(clock[0]))
    LocalMapper.continuity(mapper, 'robot1', proof)
    assert mapper.confirmed_local_revision == 0
    assert mapper.geometry.last_receive['reason'] == 'map_version_not_available'
    mapper.maps['robot1'] = snaps['robot1', 'current']
    LocalMapper.continuity(mapper, 'robot1', proof)
    assert mapper.confirmed_local_revision == snaps['robot3', 'current'].revision


def test_transient_recent_observations_do_not_hide_a_valid_early_reference(monkeypatch):
    clock, snaps, geometry, members, graph, mapper, node = recovery(monkeypatch)
    base = copy.deepcopy(next(iter(geometry['robot3'].certificates.values())))
    revision = max(s.revision for s in snaps.values())+1
    # Model intermediate occupied observations which are unknown again in the
    # final raw map. The preserved initial observations are real recovery maps.
    for i in range(70):
        message = copy.deepcopy(base)
        message.constraint_id = 'transient-'+str(i)
        message.source_revision = message.target_revision = revision+i
        for name in members:
            current = snaps[name, 'current']
            data = current.data.copy()
            yy, xx = np.indices(data.shape)
            data[(data < 0) & ((xx+yy) % 2 == 0)] = 100
            reference = Snapshot(name, current.boot, current.epoch, revision+i,
                                 data, current.resolution, current.origin)
            geometry[name].remember(message, reference)
    for name in members:
        current = copy.copy(snaps[name, 'current'])
        current.revision = revision+70
        snaps[name, 'current'] = mapper.maps[name] = current
        result = compute(name, snaps, geometry, members, graph, clock[0])
        assert len(result['proofs']) == 1
        assert {c['reason'] for c in result['checks']} == {'accepted', 'insufficient_observed_overlap'}
    node.mapping[1].revision = revision+70
    PeerAgent.verify_alignment(node, clock[0])
    for name in members:
        for proof in geometry[name].outgoing(clock[0]):
            LocalMapper.continuity(mapper, name, proof)
    node.mapping[2]['confirmed_local_revision'] = mapper.confirmed_local_revision
    PeerAgent.verify_alignment(node, clock[0]+3.1)
    assert node.alignment_verified and mapper.confirmed_local_revision == revision+70


@pytest.mark.parametrize('invalid', ['shifted', 'new_boot', 'missing_reference', 'expired_worker'])
def test_recovery_chain_cannot_confirm_invalid_geometry_or_identity(monkeypatch, invalid):
    clock, snaps, geometry, members, graph, mapper, node = recovery(monkeypatch)
    compute('robot1', snaps, geometry, members, graph, clock[0])
    if invalid == 'shifted':
        current = copy.copy(snaps['robot3', 'current'])
        current.origin = (current.origin[0]+.25, *current.origin[1:])
        snaps['robot3', 'current'] = current
    elif invalid == 'new_boot':
        members['robot3'] = ('restarted', members['robot3'][1]+1)
    elif invalid == 'missing_reference':
        geometry['robot3'].references.clear()
    result = compute('robot3', snaps, geometry, members, graph, clock[0])
    if invalid == 'expired_worker':
        clock[0] += 16.
    else:
        assert not result['proofs']
    for name in members:
        for proof in geometry[name].outgoing(clock[0]):
            LocalMapper.continuity(mapper, name, proof)
    PeerAgent.verify_alignment(node, clock[0])
    PeerAgent.verify_alignment(node, clock[0]+4.)
    assert not node.alignment_verified and mapper.confirmed_local_revision == 0
