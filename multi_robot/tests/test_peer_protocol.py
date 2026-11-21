import itertools
import math
import pytest
from fleet_control.peer_protocol import (GrantTracker, LeaseVoter, ReplayWindow,
                                        lost_peer_radius, resources_conflict, choose_motion_owner)


def test_previously_unseen_old_boot_cannot_replace_a_newer_startup_epoch():
    window = ReplayWindow()
    assert window.accept('robot2', 'new', 1, 3)
    assert not window.accept('robot2', 'old-unseen', 900, 2)
    assert not window.accept('robot2', 'different-identity-same-epoch', 900, 3)
    assert not window.accept('robot2', 'new', 900, 4)
    assert window.accept('robot2', 'new', 2, 3)
    assert window.accept('robot2', 'next', 1, 4)
    assert not window.accept('robot2', 'new', 901, 3)


def test_majority_reservations_prevent_conflicting_motion():
    for order in itertools.permutations(('robot1', 'robot2', 'robot3')):
        voters = {n: LeaseVoter(n, 0) for n in order}
        first = {n for n, v in voters.items()
                 if v.grant('team_motion', 'robot1', 'boot-a', 'a', 'mission', 1, 3)}
        second = {n for n, v in voters.items()
                  if v.grant('team_motion', 'robot2', 'boot-b', 'b', 'mission', 1, 3.1)}
        assert len(first) >= 2 and len(second) < 2


def test_intersection_of_competing_two_member_partitions_cannot_double_grant():
    voters = {n: LeaseVoter(n, 0) for n in ('robot1', 'robot2', 'robot3')}
    assert voters['robot1'].grant('team_motion', 'robot1', 'a', 'a', 'm', 1, 3)
    assert voters['robot2'].grant('team_motion', 'robot1', 'a', 'a', 'm', 1, 3)
    assert not voters['robot2'].grant('team_motion', 'robot3', 'b', 'b', 'm', 1, 3.1)
    assert voters['robot3'].grant('team_motion', 'robot3', 'b', 'b', 'm', 1, 3.1)


def test_quarantine_release_and_replay_fences():
    voter = LeaseVoter('robot2', 5)
    assert not voter.grant('team_motion', 'robot1', 'a', 'a', 'm', 1, 6.99)
    assert voter.grant('team_motion', 'robot1', 'a', 'a', 'm', 1, 7)
    assert not voter.release('robot1', 'a', 'a', False)
    assert voter.release('robot1', 'a', 'a', True)
    assert not voter.grant('team_motion', 'robot1', 'a', 'a', 'm', 2, 8)
    assert voter.grant('team_motion', 'robot3', 'b', 'b', 'm', 1, 8)


def test_delayed_votes_and_old_boots_cannot_authorize_motion():
    tracker = GrantTracker()
    tracker.requested('r', 'm', 'team_motion', 1, 3, 3)
    for voter in ('robot1', 'robot2'):
        tracker.granted(voter, 'boot-'+voter, 'r', 'm', 'team_motion', 1)
    boots = {n: 'boot-'+n for n in ('robot1', 'robot2')}
    assert tracker.valid('r', 'm', 'team_motion', 3.49, boots)
    assert not tracker.valid('r', 'm', 'team_motion', 3.51, boots)
    assert not tracker.valid('r', 'm', 'team_motion', 3.1, {'robot1': 'reboot', 'robot2': boots['robot2']})
    assert not tracker.valid('r', 'another-mission', 'team_motion', 3.1, boots)
    assert not tracker.valid('r', 'm', 'team_motion', 3.1, {'robot1': boots['robot1']})


def test_duplicate_voter_does_not_make_a_quorum():
    tracker = GrantTracker()
    tracker.requested('r', 'm', 'team_motion', 1, 3, 3)
    for _ in range(3):
        tracker.granted('robot1', 'b1', 'r', 'm', 'team_motion', 1)
    assert not tracker.valid('r', 'm', 'team_motion', 3.1, {'robot1': 'b1'})


def test_regions_parallel_but_transit_exclusive():
    assert not resources_conflict('region/robot1', 'region/robot2')
    assert resources_conflict('team_motion', 'region/robot1')
    voter = LeaseVoter('robot1', 0)
    assert voter.grant('region/robot1', 'robot1', 'a', 'r1', 'm', 1, 3)
    assert voter.grant('region/robot2', 'robot2', 'b', 'r2', 'm', 1, 3)
    assert not voter.grant('team_motion', 'robot3', 'c', 'r3', 'm', 1, 3)


def test_boot_identity_never_switches_back_to_retired_session():
    replay = ReplayWindow()
    assert replay.accept('robot1', 'old', 1)
    assert not replay.accept('robot1', 'old', 1)
    assert replay.accept('robot1', 'new', 1)
    assert not replay.accept('robot1', 'old', 200)


def test_lost_robot_reachable_set_grows_instead_of_disappearing():
    assert lost_peer_radius(.18, .2, 5) > lost_peer_radius(.18, .2, .8)
    assert lost_peer_radius(.18, .2, 100) > 15
    with pytest.raises(ValueError):
        lost_peer_radius(.18, .2, math.nan)


def test_serial_patrol_contenders_do_not_renew_three_conflicting_self_votes_forever():
    pending = {n: dict(kind='patrol', completed=0, held=False) for n in ('robot1', 'robot2', 'robot3')}
    assert choose_motion_owner(pending) == 'robot1'
    pending['robot1']['completed'] = 1
    assert choose_motion_owner(pending) == 'robot2'
    pending['robot2']['completed'] = 1
    assert choose_motion_owner(pending) == 'robot3'
    pending['robot3']['held'] = True
    pending['robot1']['completed'] = 0
    assert choose_motion_owner(pending) == 'robot3'
    del pending['robot3']
    assert choose_motion_owner(pending) == 'robot1'
