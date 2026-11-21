"""Drive the real guard with isolated ROS messages; no simulator actuators."""
import json
import math
import time
from pathlib import Path
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from fleet_interfaces.msg import (PeerState, MappingState, CoordinationEvent,
                                   LocalAuthorization, MissionCommand)
from fleet_control.local_safety import LocalSafety
from fleet_control.peer_common import STATE_QOS, EVENT_QOS, dumps, peer_topic, set_pose
import yaml


@pytest.fixture
def guard(tmp_path):
    cfg = dict(architecture='decentralized', robot_id='robot1', members=['robot1', 'robot2', 'robot3'],
        session_id='guard-test', boot_id='b1', map_epoch=1, resolution=.05, robot_radius=.18,
        max_speed=.2, region_margin=.35, run_dir=str(tmp_path), dashboard_port=8765, assets=str(tmp_path))
    path = tmp_path/'agent.yaml'
    path.write_text(yaml.safe_dump(cfg))
    rclpy.init(args=['--ros-args', '-p', 'config:='+str(path), '-r', '/tf:=/robot1/tf',
                    '-r', '/tf_static:=/robot1/tf_static'], domain_id=191)
    safety = LocalSafety()
    node = rclpy.create_node('isolated_guard_probe')
    peers = {n: node.create_publisher(PeerState, peer_topic(n, 'state'), STATE_QOS) for n in cfg['members']}
    events = {n: node.create_publisher(CoordinationEvent, peer_topic(n, 'event'), EVENT_QOS) for n in cfg['members']}
    mp = node.create_publisher(MappingState, peer_topic('robot1', 'mapping'), STATE_QOS)
    ap = node.create_publisher(LocalAuthorization, '/robot1/authorization', 10)
    cp = node.create_publisher(Twist, '/robot1/cmd_vel_nav', 1)
    sp = node.create_publisher(LaserScan, '/robot1/scan', 10)
    op = node.create_publisher(Odometry, '/robot1/odom', 10)
    tp = node.create_publisher(TFMessage, '/robot1/tf', 10)
    mission = node.create_publisher(MissionCommand, '/fleet/peer/mission', EVENT_QOS)
    outputs = []
    node.create_subscription(Twist, '/robot1/diffdrive_controller/cmd_vel_unstamped',
                             lambda m: outputs.append((time.monotonic(), m.linear.x)), 100)
    serial = dict.fromkeys(cfg['members'], 0)
    sequence, request = 0, 0
    frozen = None

    def spin(seconds, grants=True, sensors=True, freeze=False, late=False, old_boot=False, bad_map_age=None):
        nonlocal sequence, request, frozen
        end, sent = time.monotonic()+seconds, 0.
        while time.monotonic() < end:
            now = time.monotonic()
            if now-sent >= .06:
                sent = now
                sequence += 1
                stamp = node.get_clock().now().to_msg()
                if freeze and frozen is None:
                    frozen = stamp
                for i, name in enumerate(cfg['members'], 1):
                    p = PeerState(session_id='guard-test', robot_id=name, boot_id='b'+str(i),
                                  sequence=sequence, map_epoch=1, local_ready=True, nav_ready=True,
                                  aligned=True, frame_epoch='frame')
                    peers[name].publish(p)
                    serial[name] += 1
                    events[name].publish(CoordinationEvent(session_id='guard-test', robot_id=name,
                        boot_id='b'+str(i), sequence=serial[name], phase=CoordinationEvent.MISSION_VOTE,
                        mission_id='mission', payload_json='{"accepted":true}'))
                mapping = MappingState(session_id='guard-test', robot_id='robot1', boot_id='b1',
                                       map_epoch=1, aligned=True, frame_epoch='frame')
                mapping.details_json = dumps({'transforms': {n: {'boot_id': 'b'+str(i),
                    'transform': [0. if i == 1 else 5. if i == 2 else -5., 0., 0.], 'sigma': .025}
                    for i, n in enumerate(cfg['members'], 1)},
                    'map_ages': {'robot1': 0. if bad_map_age is None else bad_map_age}})
                mp.publish(mapping)
                m = MissionCommand(session_id='guard-test', issuer_id='test', command_id='mission', revision=1,
                    kind='explore', participant_boot_ids=['b1', 'b2', 'b3'], expires_at=now+1.)
                mission.publish(m)
                auth = LocalAuthorization(session_id='guard-test', robot_id='robot1', boot_id='b1',
                    sequence=sequence, mission_id='mission', request_id='request', resource='team_motion',
                    frame_epoch='frame', enabled=True, maximum_speed=.2, limits_json='{}')
                ap.publish(auth)
                command = Twist()
                command.linear.x = .1
                cp.publish(command)
                transform = TransformStamped()
                transform.header.stamp, transform.header.frame_id = stamp, 'robot1/map'
                transform.child_frame_id, transform.transform.rotation.w = 'robot1/base_link', 1.
                tp.publish(TFMessage(transforms=[transform]))
                if sensors:
                    scan = LaserScan(angle_min=-math.pi, angle_max=math.pi, angle_increment=2*math.pi/640,
                                     range_min=.12, range_max=12., ranges=[math.inf]*640)
                    scan.header.stamp, scan.header.frame_id = frozen if freeze else stamp, 'robot1/base_link'
                    sp.publish(scan)
                    odom = Odometry()
                    odom.header.stamp = frozen if freeze else stamp
                    odom.pose.pose.orientation.w = 1.
                    op.publish(odom)
                if grants:
                    request += 1
                    for i, name in enumerate(cfg['members'][:2], 1):
                        if i == 1:
                            serial[name] += 1
                            q = CoordinationEvent(session_id='guard-test', robot_id=name, boot_id='b1',
                                sequence=serial[name], phase=CoordinationEvent.REQUEST, mission_id='mission',
                                resource='team_motion', owner_id='robot1', owner_boot='b1', request_id='request',
                                request_sequence=request, sent_steady_ns=int((now-(2. if late else 0.))*1e9))
                            events[name].publish(q)
                        serial[name] += 1
                        g = CoordinationEvent(session_id='guard-test', robot_id=name,
                            boot_id='old'+str(i) if old_boot else 'b'+str(i), sequence=serial[name],
                            phase=CoordinationEvent.GRANT, mission_id='mission', resource='team_motion',
                            owner_id='robot1', owner_boot='b1', request_id='request', request_sequence=request)
                        events[name].publish(g)
            rclpy.spin_once(safety, timeout_sec=.001)
            rclpy.spin_once(node, timeout_sec=.001)
        return outputs
    try:
        yield safety, node, spin, outputs
    finally:
        safety.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


def test_guard_expires_majority_even_while_agent_command_and_heartbeat_continue(guard):
    safety, node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs), 'test never established actual authorization'
    failure = time.monotonic()
    spin(.85, grants=False)
    assert outputs[-1][1] == 0.
    assert all(v == 0. for t, v in outputs if t >= failure+.8)


def test_sensor_data_freeze_stops_before_eight_tenths_despite_live_commands_and_votes(guard):
    safety, node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)
    failure = time.monotonic()
    spin(.95, freeze=True)
    assert outputs[-1][1] == 0.
    assert all(v == 0. for t, v in outputs if t >= failure+.8)


def test_pause_fences_fresh_retransmissions_of_the_old_authorization(guard):
    safety, node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)
    safety.mission(MissionCommand(session_id='guard-test', issuer_id='test', command_id='pause', revision=2, kind='pause'))
    spin(.3)
    assert outputs[-1][1] == 0.


def test_rejected_motion_proposal_does_not_interrupt_current_valid_mission(guard):
    safety, node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)
    safety.mission(MissionCommand(session_id='guard-test', issuer_id='test', command_id='invalid', revision=2,
        kind='formation', participant_boot_ids=['b1', 'b2', 'b3'], expires_at=time.monotonic()+2.))
    spin(.3)
    assert outputs[-1][1] > .05


@pytest.mark.parametrize('age', ['stale', -1., True, []])
def test_malformed_mapping_does_not_crash_or_refresh_map_health(guard, age):
    safety, node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)
    spin(1.8, bad_map_age=age)
    assert outputs[-1][1] == 0.


@pytest.mark.parametrize('fault', ['late', 'old_boot', 'frozen_sensor'])
def test_late_or_old_identity_votes_and_frozen_data_cannot_restore_motion(guard, fault):
    safety, node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)
    spin(1.0, late=fault == 'late', old_boot=fault == 'old_boot', freeze=fault == 'frozen_sensor')
    assert outputs[-1][1] == 0.


def test_new_estop_fences_old_clear_and_needs_fresh_authorization(guard):
    safety, _node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)

    def command(kind, ident, revision):
        safety.mission(MissionCommand(session_id='guard-test', issuer_id='test',
            command_id=ident, revision=revision, kind=kind,
            participant_boot_ids=['b1', 'b2', 'b3'], expires_at=time.monotonic()+1.))

    def clear(ident, revision, age=0.):
        command('clear-estop', ident, revision)
        safety.mission_votes[ident] = {'robot1': 'b1', 'robot2': 'b2'}
        received = LocalAuthorization(session_id='guard-test', robot_id='robot1', boot_id='b1',
            sequence=safety.auth_sequence+1, mission_id=ident, enabled=False, estop=False)
        safety.authorize(received)
        if age:
            safety.authorization = (time.monotonic()-age, received)
        safety.tick()

    command('estop', 'stop-1', 10)
    clear('clear-1', 11)
    assert not safety.estop

    command('estop', 'stop-2', 12)
    safety.tick()
    assert safety.estop and json.loads(safety.journal.path.read_text())['estop']
    assert safety.clear_command is None

    clear('clear-2', 13, age=.7)
    assert safety.estop
    clear('clear-2', 13)
    assert not safety.estop and not json.loads(safety.journal.path.read_text())['estop']

    # A retransmission of a consumed release cannot arm it again.
    command('clear-estop', 'clear-2', 13)
    assert safety.clear_command is None


def test_pending_clear_survives_stop_heartbeat_and_commits_atomically(guard, monkeypatch):
    safety, _node, spin, outputs = guard
    spin(1.)
    assert any(v > .05 for _, v in outputs)
    previous_order = safety.command_order
    clear = MissionCommand(session_id='guard-test', issuer_id='test',
        command_id='new-clear', revision=10, kind='clear-estop',
        participant_boot_ids=['b1', 'b2', 'b3'], expires_at=time.monotonic()+1.)
    safety.mission(clear)
    safety.mission_votes[clear.command_id] = {'robot1': 'b1', 'robot2': 'b2'}

    def authorize(estop):
        safety.authorize(LocalAuthorization(session_id='guard-test', robot_id='robot1',
            boot_id='b1', sequence=safety.auth_sequence+1, mission_id=clear.command_id,
            enabled=False, estop=estop))

    # The stop heartbeat and the clear command travel on different topics.
    authorize(True)
    safety.tick()
    assert safety.estop and safety.command_order == previous_order

    authorize(False)
    with monkeypatch.context() as patch:
        def fail_write(**_values):
            raise OSError('simulated unavailable journal')
        patch.setattr(safety.journal, 'update', fail_write)
        with pytest.raises(OSError, match='simulated unavailable journal'):
            safety.tick()
    assert safety.estop and safety.command_order == previous_order
    stopped_state = json.loads(safety.journal.path.read_text())
    assert stopped_state['estop'] and tuple(stopped_state['command_order']) == previous_order

    safety.tick()
    state = json.loads(safety.journal.path.read_text())
    assert not safety.estop and state['estop'] is False
    assert safety.command_id == clear.command_id == state['command_order'][2]
