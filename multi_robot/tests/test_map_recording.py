"""Record original source revisions without accepting stale or foreign maps."""
from types import SimpleNamespace
import numpy as np
from fleet_interfaces.msg import MapSnapshot
from fleet_control.peer_common import grid_message
from fleet_control.peer_recording import RawMapRecorder


def snapshot(revision=1, epoch=1, session='run'):
    data = np.zeros((20, 20), np.int16)
    data[3, revision] = 100
    message = MapSnapshot(session_id=session, robot_id='robot1', boot_id='boot'+str(epoch),
                          map_epoch=epoch, revision=revision)
    from builtin_interfaces.msg import Time
    message.grid = grid_message(data, .05, (0., 0.), 'robot1/map', Time())
    return message


def test_recorder_keeps_raw_revisions_and_first_aligned_source_without_fusion(tmp_path):
    record = RawMapRecorder(tmp_path, 'run')
    record.receive('robot1', snapshot(session='foreign'))
    assert not record.latest
    record.receive('robot1', snapshot())
    record.receive('robot1', snapshot(2))
    record.receive('robot1', snapshot(1))
    assert record.latest['robot1'].revision == 2
    state = dict(robots={'robot1': dict(boot_id='boot1', map_epoch=1, ready=True, aligned=True)})
    record.checkpoint(state)
    fixed = np.load(record.folder/'robot1-e1-first-aligned-robot1.npz')
    assert fixed['revision'] == 2 and fixed['data'][3, 2] == 100
    record.receive('robot1', snapshot(3))
    record.checkpoint(state)
    assert np.load(record.folder/'robot1-e1-first-aligned-robot1.npz')['revision'] == 2
    record.receive('robot1', snapshot(1, 2))
    record.receive('robot1', snapshot(4, 1))
    assert record.latest['robot1'].epoch == 2
