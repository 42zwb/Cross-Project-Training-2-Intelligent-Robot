import json
import time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from fleet_mapping.node import Mapping
from fleet_control.coordinator import Coordinator, local_maps_fresh


def test_local_map_receipt_refreshes_even_when_contents_unchanged(monkeypatch):
    now = [1.0]
    monkeypatch.setattr('fleet_mapping.node.time.monotonic', lambda: now[0])
    fake = SimpleNamespace(maps={}, map_received={}, map_revision=0)
    msg = OccupancyGrid()
    msg.info.resolution = 0.1
    msg.info.width = msg.info.height = 1
    msg.info.origin.orientation.w = 1.0
    msg.data = [0]
    Mapping.local_map(fake, 'robot1', msg)
    now[0] = 16.0
    assert not local_maps_fresh({'local_map_age': {'robot1': now[0] - fake.map_received['robot1']}}, ['robot1'])
    Mapping.local_map(fake, 'robot1', msg)
    assert local_maps_fresh({'local_map_age': {'robot1': now[0] - fake.map_received['robot1']}}, ['robot1'])
    msg.data = []
    Mapping.local_map(fake, 'robot1', msg)
    assert not fake.maps and not fake.map_received


def test_mapping_state_accepts_stale_age_then_fresh_recovery():
    fake = SimpleNamespace(names=['robot1'], mapping={}, mapping_received=0.0)
    state = {'aligned':['robot1'], 'local_maps':['robot1'], 'local_map_age':{'robot1': 16.0},
             'scan_age':{'robot1': 0.1}, 'known_fraction':0.5}
    Coordinator.mapping_cb(fake, String(data=json.dumps(state)))
    assert fake.mapping and not local_maps_fresh(fake.mapping, fake.names)
    state['local_map_age']['robot1'] = 0.1
    Coordinator.mapping_cb(fake, String(data=json.dumps(state)))
    assert local_maps_fresh(fake.mapping, fake.names)


class FakeClient:
    def __init__(self):
        self.future = Future()
    def service_is_ready(self): return True
    def call_async(self, request):
        self.filename = Path(request.filename)
        return self.future


def mapper():
    results, clients, destroyed = [], [], []
    fake = SimpleNamespace(cfg={'resolution':0.1, 'arena':[0,0,1,1]}, robots=[{'name':'r1'}, {'name':'r2'}],
        data=np.zeros((2,2), dtype=np.int8), observed=np.zeros((2,2), dtype=np.int8),
        contour=np.zeros((2,2), dtype=np.int8), inferred=np.zeros((2,2), dtype=bool),
        completion=SimpleNamespace(boundary_inferred=np.zeros((2,2), dtype=bool)),
        map_metrics={}, align={}, save_pending={},
        get_logger=lambda: SimpleNamespace(info=lambda *args:None, error=lambda *args:None),
        publish_save_result=lambda *args: results.append(args), destroy_client=destroyed.append)
    def create_client(*args):
        client = FakeClient(); clients.append(client); return client
    fake.create_client = create_client
    fake.save_done = lambda *args: Mapping.save_done(fake, *args)
    fake.finish_save = lambda *args: Mapping.finish_save(fake, *args)
    return fake, clients, results, destroyed


def test_save_duplicate_id_and_late_callbacks_do_not_complete_another_request(tmp_path):
    fake, clients, results, destroyed = mapper()
    def submit(dest):
        Mapping.save(fake, String(data=json.dumps({'request_id':'same', 'destination': str(dest)})))
    submit(tmp_path/'first')
    submit(tmp_path/'second')
    assert len(clients) == 2 and results[-1][2] is False
    old = clients[:]
    fake.save_pending['same']['deadline'] = 0
    Mapping.expire_saves(fake)
    assert len(destroyed) == 2
    submit(tmp_path/'second')
    assert len(clients) == 4
    Mapping.save_done(fake, 'same', 'r1', old[0], old[0].future)
    assert fake.save_pending['same']['remaining'] == 2
    for client in clients[2:]:
        client.filename.write_bytes(b'pbstream')
        client.future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='')))
    assert not fake.save_pending and results[-1][2] is True
    assert len(destroyed) == 4


def test_duplicate_save_retry_with_same_destination_waits_for_original(tmp_path):
    fake, clients, results, _ = mapper()
    request = String(data=json.dumps({'request_id': 'same', 'destination': str(tmp_path)}))
    Mapping.save(fake, request)
    Mapping.save(fake, request)
    assert len(clients) == 2 and not results
    for client in clients:
        client.filename.write_bytes(b'pbstream')
        client.future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='')))
    assert len(results) == 1 and results[0][2] is True


def test_save_checks_service_success_has_file(tmp_path):
    fake, clients, results, _ = mapper()
    Mapping.save(fake, String(data=json.dumps({'request_id':'x', 'destination':str(tmp_path/'missing')})))
    for client in clients:
        client.future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='')))
    assert results[-1][2] is False and '不存在或为空' in results[-1][3]


def test_goal_requires_fresh_safety_epoch_ack_before_send(monkeypatch):
    import threading
    import fleet_control.coordinator as coordinator
    now = [10.0]
    sent, leases = [], []
    monkeypatch.setattr(coordinator.time, 'monotonic', lambda: now[0])
    def sleep(duration): now[0] += duration
    monkeypatch.setattr(coordinator.time, 'sleep', sleep)
    fake = SimpleNamespace(lock=threading.RLock(), active=set(), command_epochs={'r1':7},
        valid=lambda token:True, safety={'r1': {'command_epoch':7}}, safety_received=now[0],
        publish_lease=lambda: leases.append(True), event=lambda *args, **kw:None,
        actions={'r1':SimpleNamespace(send_goal_async=lambda goal:sent.append(goal))})
    assert Coordinator.send_goal(fake, 'r1', (0,0,0), 1) is None
    assert not sent and fake.command_epochs['r1'] == 8 and not fake.active and len(leases) == 2


def test_repeated_save_replaces_only_completed_pbstreams(tmp_path):
    fake, clients, results, _ = mapper()
    destination = tmp_path/'same-destination'
    destination.mkdir()
    (destination/'r1.pbstream').write_bytes(b'old1')
    (destination/'r2.pbstream').write_bytes(b'old2')
    Mapping.save(fake, String(data=json.dumps({'request_id':'repeat', 'destination':str(destination)})))
    clients[0].filename.write_bytes(b'new1')
    clients[0].future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='')))
    clients[1].future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='')))
    assert not results[-1][2]
    assert (destination/'r1.pbstream').read_bytes() == b'new1'
    assert (destination/'r2.pbstream').read_bytes() == b'old2'
    Mapping.save(fake, String(data=json.dumps({'request_id':'repeat2', 'destination':str(destination)})))
    for client in clients[2:]:
        client.filename.write_bytes(b'fresh')
        client.future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='')))
    assert results[-1][2] and (destination/'r2.pbstream').read_bytes() == b'fresh'


import pytest

@pytest.mark.parametrize('reason', ['雷达时间戳超时或无效', '里程计时间戳超时或无效', '雷达坐标变换不可用', '局部地图更新超时'])
def test_active_task_faults_and_cancels_for_new_diagnostics(reason):
    import threading
    from tf2_ros import TransformException
    def no_tf(*args): raise TransformException('unavailable')
    events, cancellations, leases = [], [], []
    names = ['r1', 'r2', 'r3']
    state = {'aligned':names, 'local_maps':names, 'scan_age':dict.fromkeys(names, 0.1),
             'local_map_age':dict.fromkeys(names, 0.1)}
    safety_reason = reason
    if reason == '局部地图更新超时':
        state['local_map_age']['r1'] = 16.0
        safety_reason = '正常'
    now = time.monotonic()
    fake = SimpleNamespace(lock=threading.RLock(), names=names,
        buffer=SimpleNamespace(lookup_transform=no_tf), poses={}, trails={n:[] for n in names},
        footprints={}, mapping=state, mapping_received=now, safety_received=now,
        safety={'r1':{'reason':safety_reason}}, bonds_fresh=lambda now:True,
        grid=None, initial_free_confirmed=True, nav_ready=True, mode='EXPLORING',
        active={'r1'}, generation=4, patrol={'status':'running'}, slots=None, assignment=None,
        cancel_all=lambda:cancellations.append(True), event=lambda *args,**kw:events.append((args,kw)),
        publish_lease=lambda:leases.append(True), last_state_publish=now)
    Coordinator.tick(fake)
    assert fake.mode == 'FAULT' and fake.generation == 5 and not fake.active
    assert reason in fake.message and fake.patrol['status'] == 'failed'
    assert cancellations and leases


def test_idle_readiness_requires_fresh_sensor_and_localization_diagnostics():
    import threading
    from rclpy.clock import Clock, ClockType
    from geometry_msgs.msg import TransformStamped
    now = time.monotonic()
    clock = Clock(clock_type=ClockType.ROS_TIME)
    transform = TransformStamped()
    transform.header.stamp = clock.now().to_msg()
    transform.transform.rotation.w = 1.0
    names = ['r1', 'r2', 'r3']
    state = {'aligned':names, 'local_maps':names, 'scan_age':dict.fromkeys(names, 0.1),
             'local_map_age':dict.fromkeys(names, 0.1)}
    safety = {n:{'reason':'待命', 'scan_valid':True, 'odom_valid':True,
                 'localization_ok':True, 'scan_age':0.1, 'odom_age':0.1} for n in names}
    grid = SimpleNamespace(inside=lambda cell:True, cell=lambda pose:(0,0),
                           free_clearance=np.array([[1.0]]), clearance_radius=0.2)
    fake = SimpleNamespace(lock=threading.RLock(), names=names, get_clock=lambda:clock,
        buffer=SimpleNamespace(lookup_transform=lambda *args:transform), poses={},
        trails={n:[] for n in names}, footprints={str(i): clock.now().to_msg() for i in range(6)},
        mapping=state, mapping_received=now, safety_received=now, safety=safety,
        bonds_fresh=lambda now:True, grid=grid, initial_free_confirmed=True, nav_ready=True,
        mode='IDLE', active=set(), generation=0, patrol=None, slots=None, assignment=None,
        publish_lease=lambda:None, last_state_publish=now)
    Coordinator.tick(fake)
    assert fake.ready
    for field in ['scan_valid', 'odom_valid', 'localization_ok']:
        safety['r1'][field] = False
        Coordinator.tick(fake)
        assert not fake.ready
        safety['r1'][field] = True
    safety['r2']['scan_age'] = 1.0
    Coordinator.tick(fake)
    assert not fake.ready
    safety['r2']['scan_age'] = 0.1
    safety['r3'].pop('odom_valid')
    Coordinator.tick(fake)
    assert not fake.ready
