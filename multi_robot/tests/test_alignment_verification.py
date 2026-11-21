"""An accepted map match is not yet a stable motion-ready localization."""
import threading
from types import SimpleNamespace
from nav_msgs.msg import Odometry
from fleet_control.peer_agent import PeerAgent


def agent():
    return SimpleNamespace(map_aligned=lambda: True, local_ready=True, stopped=lambda now: True,
        pose=(0., 0., 0.), alignment_verified=False, alignment_ever_verified=False, alignment_hold=None, verified_at=None,
        team_transform=lambda: (0., 0., 0.),
        mapping=(0., SimpleNamespace(revision=5), {'confirmed_local_revision': 5, 'registered_local_revision': 5}),
        pose_sim_time=0., pose_steady_time=0., epoch=2, team_initialized=True, record=lambda *a, **k: None)


def test_map_match_requires_three_seconds_of_stationary_pose_before_motion_ready():
    node = agent()
    PeerAgent.verify_alignment(node, 1.)
    PeerAgent.verify_alignment(node, 3.9)
    assert not node.alignment_verified
    PeerAgent.verify_alignment(node, 4.)
    assert node.alignment_verified and node.verified_at == 4.
    node.pose = (2., 0., 1.)
    PeerAgent.verify_alignment(node, 5.)
    assert node.alignment_verified  # Verified robots may explore normally.
    node.local_ready = False
    PeerAgent.verify_alignment(node, 6.)
    assert not node.alignment_verified


def test_moving_or_unstable_pose_cannot_finish_initial_verification():
    node = agent()
    PeerAgent.verify_alignment(node, 1.)
    node.pose = (.03, 0., 0.)
    PeerAgent.verify_alignment(node, 3.)
    PeerAgent.verify_alignment(node, 5.)
    assert not node.alignment_verified
    node.stopped = lambda now: False
    PeerAgent.verify_alignment(node, 10.)
    assert node.alignment_hold is None and not node.alignment_verified


def test_bootstrap_mileage_does_not_replace_each_robots_regional_participation():
    node = SimpleNamespace(lock=threading.RLock(), distance=0., travel=0., regional_travel=0.,
        last_odom_pose=None, mode='ALIGNING', resource='team_motion')
    message = Odometry()
    message.pose.pose.orientation.w = 1.
    for i in range(21):
        message.pose.pose.position.x = i*.1
        PeerAgent.sensor(node, 'odom', message)
    assert node.travel > 1.9 and node.regional_travel == 0.
    node.mode, node.resource = 'EXPLORING', 'region/robot1'
    for i in range(21, 41):
        message.pose.pose.position.x = i*.1
        PeerAgent.sensor(node, 'odom', message)
    assert 1.9 < node.regional_travel < 2.1 and node.travel > 3.9


def test_common_transform_and_post_stop_map_confirmation_must_also_be_stable():
    node = agent()
    PeerAgent.verify_alignment(node, 1.)
    node.team_transform = lambda: (.03, 0., 0.)
    PeerAgent.verify_alignment(node, 3.)
    node.mapping[1].revision = 6
    PeerAgent.verify_alignment(node, 5.)
    assert not node.alignment_verified
    # Start another stationary window with a newer local-map revision.
    node.pose = (.03, 0., 0.)
    PeerAgent.verify_alignment(node, 6.)
    PeerAgent.verify_alignment(node, 10.)
    assert not node.alignment_verified  # Only revision 5 had both witnesses.
    node.mapping[2]['confirmed_local_revision'] = 6
    PeerAgent.verify_alignment(node, 10.1)
    assert not node.alignment_verified  # Continuity cannot improve the first fit.
    node.mapping[2]['registered_local_revision'] = 6
    PeerAgent.verify_alignment(node, 10.2)
    assert node.alignment_verified


def test_verified_identity_can_use_retention_but_new_identity_cannot():
    node = agent()
    PeerAgent.verify_alignment(node, 1.)
    PeerAgent.verify_alignment(node, 4.)
    assert node.alignment_ever_verified
    node.local_ready = False
    PeerAgent.verify_alignment(node, 5.)
    node.local_ready = True
    node.mapping[1].revision = 20
    node.mapping[2]['confirmed_local_revision'] = 20
    PeerAgent.verify_alignment(node, 6.)
    PeerAgent.verify_alignment(node, 9.)
    assert node.alignment_verified
    node.alignment_ever_verified = node.alignment_verified = False
    node.alignment_hold = None
    PeerAgent.verify_alignment(node, 10.)
    PeerAgent.verify_alignment(node, 14.)
    assert not node.alignment_verified


def test_original_startup_can_confirm_an_existing_match_with_a_post_stop_proof():
    node = agent()
    node.epoch, node.team_initialized = 1, False
    node.mapping[1].revision = 216
    node.mapping[2].update(registered_local_revision=0, confirmed_local_revision=215)
    PeerAgent.verify_alignment(node, 1.)
    PeerAgent.verify_alignment(node, 4.)
    assert not node.alignment_verified
    node.mapping[2]['confirmed_local_revision'] = 297
    PeerAgent.verify_alignment(node, 5.)
    assert not node.alignment_verified
    node.mapping[2]['registered_local_revision'] = 174
    PeerAgent.verify_alignment(node, 6.)
    assert node.alignment_verified


def test_frame_change_requires_fresh_admission_and_foreign_mapping_is_ignored():
    from fleet_interfaces.msg import MappingState
    node = agent()
    node.name, node.boot, node.epoch, node.session = 'robot1', 'a', 1, 's'
    node.lock, node.cancel = threading.RLock(), lambda: None
    node.mapping[1].frame_epoch = 'old'
    node.alignment_verified = node.alignment_ever_verified = True
    for wrong in (dict(robot_id='robot2'), dict(map_epoch=2), dict(boot_id='retired')):
        fields = dict(robot_id='robot1', boot_id='a', map_epoch=1, session_id='s',
                      frame_epoch='new', details_json='{}')
        fields.update(wrong)
        PeerAgent.mapping_cb(node, MappingState(**fields))
        assert node.alignment_verified and node.mapping[1].frame_epoch == 'old'
    PeerAgent.mapping_cb(node, MappingState(robot_id='robot1', boot_id='a', map_epoch=1,
        session_id='s', frame_epoch='new', details_json='{}'))
    assert not node.alignment_verified and not node.alignment_ever_verified
