import pytest
from fleet_control.peer_transport import FaultQueue


def test_one_way_isolation_drops_only_that_direction_and_preserves_local_events():
    queue = FaultQueue('robot2', 'run', 'agent')
    queue.profile(dict(session_id='run', links=[dict(source='robot1', target='robot2', channels=['event'], drop=1)]))
    result = []
    queue.receive('robot1', 'event', result.append, ('dropped',), 0)
    queue.receive('robot3', 'event', result.append, ('other-peer',), 0)
    queue.receive('robot2', 'event', result.append, ('own',), 0)
    assert result == ['other-peer', 'own']
    assert queue.dropped == 1


def test_delayed_duplicate_and_out_of_order_packets_keep_original_content_after_reconnection():
    queue = FaultQueue('robot2', 'run', 'agent')
    queue.profile(dict(session_id='run', links=[dict(source='robot1', target='robot2', channels=['event'],
                                                   delay=.6, reorder=.7, duplicates=2)]))
    result = []
    original = {'boot': 'old', 'sequence': 1, 'sent': 0}
    queue.receive('robot1', 'event', result.append, (original,), 0)
    original['boot'] = 'mutated-after-send'
    for i in range(2, 20):
        queue.receive('robot1', 'event', result.append, ({'boot': 'old', 'sequence': i, 'sent': .01*i},), .01*i)
    queue.profile(dict(session_id='run', links=[]))
    queue.flush(.5)
    assert result == []
    queue.flush(3.)
    assert len(result) == 57
    assert {r['boot'] for r in result} == {'old'}
    assert [r['sequence'] for r in result] != sorted(r['sequence'] for r in result)


def test_fault_queue_has_a_byte_limit_and_cannot_consume_map_sized_unbounded_memory():
    queue = FaultQueue('robot2', 'run', 'mapper', maximum_bytes=2000)
    queue.profile(dict(session_id='run', links=[dict(source='robot1', target='robot2', channels=['map'], delay=2)]))
    for i in range(100):
        queue.receive('robot1', 'map', lambda _: None, (i,), 0, size=1000)
    assert queue.bytes == 2000 and len(queue.pending) == 2 and queue.dropped == 98
    with pytest.raises(ValueError):
        queue.profile(dict(session_id='different-run', links=[]))
