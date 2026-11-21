"""A committed peer task reaches a temporarily unready member of the same boot."""
import threading
from types import SimpleNamespace
from fleet_interfaces.msg import MissionCommand
from fleet_control.peer_agent import PeerAgent
from fleet_control.peer_protocol import command_targets_boot, command_is_current


def command():
    return MissionCommand(session_id='s', issuer_id='operator', command_id='c', revision=1,
        kind='explore', participant_boot_ids=['b1', 'b2', 'b3'], expires_at=10.)


def test_revalidation_after_transient_rejection_is_allowed_for_the_same_order_and_boot(monkeypatch):
    monkeypatch.setattr('fleet_control.peer_agent.time.monotonic', lambda: 5.)
    ready, votes = [False], []
    def validate(message):
        if not ready[0]:
            raise ValueError('本车尚未就绪')
    node = SimpleNamespace(session='s', name='robot3', boot='b3', lock=threading.RLock(), peers={},
        validate_command=validate, last_command_order=(0, '', ''), last_command=None,
        pending_mission=None, estop=False, journal=SimpleNamespace(update=lambda **k: None),
        emit=lambda *a, **k: votes.append(k['payload']['accepted']))
    message = command()
    PeerAgent.mission(node, message)
    assert node.pending_mission is None and votes == [False]
    ready[0] = True
    PeerAgent.mission(node, message)
    assert node.pending_mission.command_id == 'c' and votes == [False, True]
    node.boot, node.pending_mission = 'new-boot', None
    PeerAgent.mission(node, message)
    assert node.pending_mission is None  # The old relay cannot address a restart.


def test_expired_proposal_needs_a_fresh_already_committed_quorum():
    task = command()
    peers = {'robot1': (11., SimpleNamespace(boot_id='b1', mission_id='c')),
             'robot2': (11., SimpleNamespace(boot_id='b2', mission_id='pending'))}
    assert command_targets_boot(task, 'robot3', 'b3')
    assert not command_is_current(task, 11.1, peers)
    peers['robot2'][1].mission_id = 'c'
    assert command_is_current(task, 11.1, peers)
    assert not command_is_current(task, 12., peers)
    peers['robot2'][1].boot_id = 'new-b2'
    assert not command_is_current(task, 11.1, peers)


def test_restart_cannot_replay_an_old_clear_estop_but_stop_commands_remain_global():
    task = command()
    task.kind = 'clear-estop'
    assert not command_targets_boot(task, 'robot3', 'new-boot')
    task.kind = 'estop'
    assert command_targets_boot(task, 'robot3', 'new-boot')


def test_delayed_stop_relay_does_not_override_a_newer_explicit_clear():
    from fleet_control.local_safety import LocalSafety
    guard = SimpleNamespace(session='s', name='robot3', boot='b3', peers={},
                             command_order=(2, 'operator', 'clear'), estop=False)
    task = command()
    task.kind = 'estop'
    LocalSafety.mission(guard, task)
    assert not guard.estop
