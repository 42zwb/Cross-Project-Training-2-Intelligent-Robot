"""Failure evidence must retain actual occupied inputs and stale frame names."""
import json
import numpy as np
from fleet_interfaces.msg import PeerState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from fleet_control.peer_recording import NavigationRecorder


def test_navigation_checkpoint_retains_obstacles_scan_and_stale_frames(tmp_path):
    recorder = NavigationRecorder(tmp_path)
    grid = OccupancyGrid()
    grid.header.frame_id = 'robot3/map'  # The actual input is from the old epoch.
    grid.info.width = grid.info.height = 2
    grid.info.resolution = .05
    grid.info.origin.orientation.w = 1.
    grid.data = [0, 100, -1, 0]
    recorder.grid('robot3', 'navigation_map', grid)
    scan = LaserScan()
    scan.header.frame_id = 'robot3/rplidar_link'
    scan.ranges = [float('inf'), .17, .5]
    recorder.scan('robot3', scan)
    for _ in range(80):
        recorder.tf('robot3', 'tf', TFMessage())
    peer = PeerState()
    peer.robot_id, peer.boot_id, peer.map_epoch = 'robot3', 'new-boot', 2
    peer.pose.x, peer.pose.y = 4., 5.
    state = dict(coordinate_frame='fixed-team-frame', robots={'robot3': dict(
        boot_id='new-boot', map_epoch=2, mode='FORMING', goal=[1., 2., 0.])})
    recorder.checkpoint(state, {'robot3': (1., peer, {})})
    with np.load(tmp_path/'navigation-checkpoints/robot3-e2-latest.npz') as stored:
        assert stored['navigation_map'].tolist() == [[0, 100], [-1, 0]]
        assert np.isinf(stored['ranges'][0]) and abs(stored['ranges'][1]-.17) < 1e-6
        meta = json.loads(str(stored['metadata']))
        assert meta['peer_state']['pose']['x'] == 4.
        assert meta['expected_frame'] == 'robot3/map_e2'
        assert not meta['grids']['navigation_map']['expected_frame']
        assert len(meta['tf']['tf']) == 60
    state['robots']['robot3']['mode'] = 'FAULT'
    recorder.checkpoint(state, {'robot3': (1., peer, {})})
    assert (tmp_path/'navigation-checkpoints/robot3-e2-fault.npz').exists()
