"""No member can finish on region flags before the global completion round."""
from types import SimpleNamespace
from fleet_control.peer_agent import PeerAgent


def test_region_release_discards_old_empty_timer_and_quotes_before_global_rescan():
    node = SimpleNamespace(cancel=lambda: None, unrestricted=False, explore_done=True,
        empty_since=1., bid={'empty': True, 'regional': True}, peer_bids={'robot2': {}},
        local_quotes={'old': {}}, frontier_quotes={'old': {}}, last_plan=10., record=lambda *a, **k: None)
    PeerAgent.release_regions(node)
    assert node.unrestricted and not node.explore_done and node.empty_since is None
    assert node.bid is None and not node.peer_bids and not node.frontier_quotes
    assert node.last_plan == 0.


def test_completion_requires_current_mission_global_scan_from_each_live_member():
    peers = {n: (20., SimpleNamespace(mission_id='current'),
                     dict(explore_done=True, unrestricted=False)) for n in ('robot1', 'robot2', 'robot3')}
    node = SimpleNamespace(live=lambda: peers, unrestricted=False, explore_done=True,
                           work=None, empty_since=1., mission_id='current')
    assert not PeerAgent.exploration_finished(node, 20.)
    node.unrestricted = True
    assert not PeerAgent.exploration_finished(node, 20.)
    for _, _, details in peers.values():
        details['unrestricted'] = True
    assert PeerAgent.exploration_finished(node, 20.)
    peers['robot3'][1].mission_id = 'retired'
    assert not PeerAgent.exploration_finished(node, 20.)
