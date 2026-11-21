"""A locally completed member must not strand the remaining explorer."""
from concurrent.futures import Future
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from fleet_interfaces.msg import CoordinationEvent
from fleet_control.geometry import Grid
from fleet_control.peer_agent import PeerAgent
from fleet_control.peer_protocol import MEMBERS


class ImmediatePool:
    def __init__(self):
        self.calls = 0

    def submit(self, function):
        self.calls += 1
        result = Future()
        try:
            result.set_result(function())
        except Exception as error:
            result.set_exception(error)
        return result


def team():
    grid = Grid(np.zeros((100, 300), np.int16), .05, (-2., -2.))
    poses = {n: [i*5., 0., 0.] for i, n in enumerate(MEMBERS)}
    offer = dict(kind='explore', frontier_id='last-frontier', frame_epoch='frame',
        team_target=[5.5, 1., 0.], target=[5.5, 1., 0.], resource='team_motion',
        visible_unknown_m2=2., score=1., offered_at=100.)
    nodes, messages = {}, []
    for name in MEMBERS:
        completed = name != 'robot3'
        node = SimpleNamespace(name=name, boot='boot-'+name, mission_id='explore',
            mode='EXPLORED' if completed else 'EXPLORING', unrestricted=True,
            local_ready=True, nav_ready=True, alignment_valid=True,
            mapping=(100., SimpleNamespace(frame_epoch='frame'), {}), grid=grid, pose=poses[name],
            bid=copy.deepcopy(offer) if not completed else None,
            peer_bids={'robot3': (100., copy.deepcopy(offer))},
            quote_future=None, pool=ImmediatePool(), local_quotes={}, frontier_quotes={},
            last_quote=0., last_quote_emit=0., last_plan=0., work=None, release_pending=None,
            explore_done=completed, empty_since=80. if completed else None, events=[])
        node.aligned = lambda node=node: node.alignment_valid
        node.team_transform = lambda: (0., 0., 0.)
        node.team_poses = lambda: poses
        node.record = lambda event, node=node, **data: node.events.append(dict(event=event, **data))
        nodes[name] = node

    def live():
        return {n: (100., SimpleNamespace(boot_id=p.boot, aligned=p.alignment_valid,
                    local_ready=p.local_ready, nav_ready=p.nav_ready, frame_epoch='frame',
                    mission_id=p.mission_id),
                    {'active_frontier_id': p.work.get('frontier_id') if p.work else None})
                for n, p in nodes.items()}

    def emit(owner, phase, payload):
        messages.append((owner, phase, copy.deepcopy(payload)))
        if payload.get('operation') == 'frontier_quotes':
            for peer in nodes.values():
                for key, cost in payload['quotes'].items():
                    peer.frontier_quotes.setdefault(key, {})[owner] = dict(
                        boot_id=nodes[owner].boot, cost=cost, sent=payload['sent'])

    for name, node in nodes.items():
        node.live = live
        node.emit = lambda phase, *, payload, name=name: emit(name, phase, payload)
    return nodes, messages


def collect_quotes(nodes):
    for now in (100., 100.2):
        for node in nodes.values():
            PeerAgent.frontier_tick(node, now)


def test_completed_members_quote_and_the_cheapest_one_resumes_without_granting_motion():
    nodes, messages = team()
    collect_quotes(nodes)
    for node in nodes.values():
        PeerAgent.frontier_tick(node, 100.4)
    winner = nodes['robot2']
    assert winner.mode == 'EXPLORING' and not winner.explore_done and winner.empty_since is None
    assert winner.work['frontier_id'] == 'last-frontier'
    assert winner.work['target'] == [5.5, 1., 0.]
    assert winner.request_id and winner.resource == 'team_motion'
    assert nodes['robot1'].mode == 'EXPLORED'
    assert nodes['robot1'].work is None and nodes['robot3'].work is None
    assert {n for n, _, _ in messages} == set(MEMBERS)
    assert all(phase == CoordinationEvent.BID for _, phase, _ in messages)
    event = winner.events[-1]
    assert event['event'] == 'frontier_claim' and event['resumed_from_completed']
    assert set(event['feasible_quotes']) == set(MEMBERS)


@pytest.mark.parametrize('mode', ['IDLE', 'ESTOP', 'FORMING', 'PATROL_COMPLETE', 'BOOTSTRAP'])
def test_other_missions_do_not_quote_or_resume_from_cached_frontiers(mode):
    nodes, messages = team()
    node = nodes['robot1']
    node.mode = mode
    PeerAgent.frontier_tick(node, 100.)
    assert node.mode == mode and node.work is None and node.pool.calls == 0
    assert not messages


@pytest.mark.parametrize('missing', ['local_ready', 'nav_ready', 'alignment_valid', 'unrestricted'])
def test_unavailable_completed_member_neither_quotes_nor_resumes(missing):
    nodes, messages = team()
    node = nodes['robot1']
    setattr(node, missing, False)
    PeerAgent.frontier_tick(node, 100.)
    assert node.mode == 'EXPLORED' and node.work is None and node.pool.calls == 0
    assert not messages


@pytest.mark.parametrize('invalid', ['expired', 'retired_boot'])
def test_resuming_a_completed_member_still_requires_all_current_fresh_quotes(invalid):
    nodes, _ = team()
    collect_quotes(nodes)
    node = nodes['robot2']
    row = node.frontier_quotes['last-frontier']['robot1']
    if invalid == 'expired':
        row['sent'] = 90.
    else:
        row['boot_id'] = 'retired'
    PeerAgent.frontier_tick(node, 100.4)
    assert node.mode == 'EXPLORED' and node.work is None
