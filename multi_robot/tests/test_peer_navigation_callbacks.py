import threading
from concurrent.futures import Future
from types import SimpleNamespace

from rclpy.clock import Clock

from fleet_control.peer_agent import PeerAgent


def agent_with_action(*replies):
    replies = iter(replies)
    events = []
    agent = SimpleNamespace(
        action=SimpleNamespace(server_is_ready=lambda: True,
                               send_goal_async=lambda _: next(replies)),
        lock=threading.RLock(), frame='map', get_clock=Clock,
        work={'target': (1., 2., 0.), 'kind': 'explore'},
        goal_sending=False, goal_handle=None, goal_generation=0,
        mission_id='mission', bid=None, request_id='', resource='', release_pending=None,
        visits=[], blacklist=[], plan_points=[],
        record=lambda event, **fields: events.append((event, fields)),
    )
    agent.cancel = PeerAgent.cancel.__get__(agent)
    agent._cancel_goal_handle = PeerAgent._cancel_goal_handle.__get__(agent)
    agent.finish_work = PeerAgent.finish_work.__get__(agent)
    return agent, events


def test_late_acceptance_keeps_replacement_goal_pending():
    first_reply, second_reply = Future(), Future()
    agent, _ = agent_with_action(first_reply, second_reply)
    PeerAgent.send_goal(agent)
    agent.cancel()
    agent.work = {'target': (2., 3., 0.), 'kind': 'explore'}
    PeerAgent.send_goal(agent)
    cancelled = []
    first_reply.set_result(SimpleNamespace(accepted=True,
        cancel_goal_async=lambda: cancelled.append(True)))

    assert cancelled == [True]
    assert agent.goal_sending
    PeerAgent.send_goal(agent)  # No third request must be sent.
    second_handle = SimpleNamespace(accepted=True, get_result_async=Future)
    second_reply.set_result(second_handle)
    assert not agent.goal_sending and agent.goal_handle is second_handle


def test_failed_result_releases_work_even_if_cancel_transport_is_closed():
    reply, result = Future(), Future()
    agent, events = agent_with_action(reply)
    agent.request_id, agent.resource = 'lease', 'team_motion'

    def cancel_unavailable():
        raise RuntimeError('cancel transport closed')

    PeerAgent.send_goal(agent)
    reply.set_result(SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                    cancel_goal_async=cancel_unavailable))
    result.set_exception(RuntimeError('result transport closed'))

    assert agent.work is None and agent.goal_handle is None
    assert agent.release_pending == ('lease', 'team_motion', 'mission')
    assert ('goal_action_error', {'stage': 'result', 'error': 'result transport closed'}) in events
    assert ('goal_action_error', {'stage': 'cancel', 'error': 'cancel transport closed'}) in events
