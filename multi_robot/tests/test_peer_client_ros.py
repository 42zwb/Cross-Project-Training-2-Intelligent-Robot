"""The real direct client cannot approve work rejected by its executor."""
import json
import time
import pytest
import rclpy
from rclpy.clock import Clock, ClockType
from fleet_interfaces.msg import PeerState, MissionCommand, CoordinationEvent
from fleet_control.peer_client import PeerClient
from fleet_control.peer_common import STATE_QOS, EVENT_QOS, peer_topic
from fleet_control.peer_protocol import MEMBERS


@pytest.mark.parametrize('executor_accepts', [False, True])
def test_patrol_requires_executor_vote_in_addition_to_majority(tmp_path, executor_accepts):
    (tmp_path/'architecture.json').write_text(json.dumps(dict(architecture='decentralized', session_id='client-test')))
    rclpy.init(domain_id=192)
    client = PeerClient(tmp_path)
    states = {n: client.node.create_publisher(PeerState, peer_topic(n, 'state'), STATE_QOS) for n in MEMBERS}
    votes = {n: client.node.create_publisher(CoordinationEvent, peer_topic(n, 'event'), EVENT_QOS) for n in MEMBERS}
    sequence, pending = 0, None
    def heartbeat():
        nonlocal sequence
        sequence += 1
        for name in MEMBERS:
            states[name].publish(PeerState(session_id='client-test', robot_id=name, boot_id=name+'-boot',
                sequence=sequence, map_epoch=1, details_json='{}',
                mission_id=pending[1].command_id if pending and (name != 'robot3' or executor_accepts) else ''))
        if pending and time.monotonic()-pending[0] >= .35:
            publish('robot3', pending[1], executor_accepts)
    def publish(name, command, accepted):
        nonlocal sequence
        sequence += 1
        votes[name].publish(CoordinationEvent(session_id='client-test', robot_id=name, boot_id=name+'-boot',
            sequence=sequence, phase=CoordinationEvent.MISSION_VOTE, mission_id=command.command_id,
            payload_json=json.dumps(dict(accepted=accepted, reason='executor rejected' if not accepted else 'valid'))))
    def received(command):
        nonlocal pending
        if pending is None:
            pending = time.monotonic(), command
        publish('robot1', command, True)
        publish('robot2', command, True)
    client.node.create_timer(.05, heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))
    client.node.create_subscription(MissionCommand, '/fleet/peer/mission', received, EVENT_QOS)
    try:
        client.state(.5)
        result = client.request('command', dict(command='patrol', argument=json.dumps(
            dict(routes={'robot3': [[0., 0., 0.]]}, coordinate_frame='frame'))), timeout=3.)
        assert result['success'] is executor_accepts
        assert result['results']['robot3']['accepted'] is executor_accepts
        if not executor_accepts:
            assert result['message'] == 'executor rejected'
    finally:
        client.close()
        rclpy.try_shutdown()
