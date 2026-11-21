"""Regression checks for laser geometry, blindness, timestamps and leases."""
import json
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Transform, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from builtin_interfaces.msg import Time

from fleet_control.health import lidar_motion_check, scan_health, stamp_fresh
from fleet_control.safety import Safety


def scan_with(value=float('inf')):
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 12.0
    scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 640
    scan.ranges = [value] * 640
    return scan


def transform(yaw=0.0, x=0.0):
    t = Transform()
    t.translation.x = x
    t.rotation.z = math.sin(yaw / 2)
    t.rotation.w = math.cos(yaw / 2)
    return t


def set_ray(scan, angle, distance):
    scan.ranges[round((angle - scan.angle_min) / scan.angle_increment)] = distance


def test_positive_infinity_means_clear_without_hiding_blindness():
    scan = scan_with()
    assert scan_health(scan) == (True, None)
    assert lidar_motion_check(scan, transform(), 0.2, 0.18) == (True, False)
    scan.ranges = [float('nan') if abs(scan.angle_min + i*scan.angle_increment) < 1.2 else float('inf') for i in range(640)]
    scan.ranges[320] = 1.0
    assert scan_health(scan)[0]
    assert not lidar_motion_check(scan, transform(), 0.2, 0.18)[0]


@pytest.mark.parametrize('yaw,ray,speed', [(math.pi/2, -math.pi/2, 0.2), (0, math.pi/4, 0.2), (0, -math.pi, -0.2), (-math.pi/2, math.pi/2, 0.2)])
def test_transformed_obstacles_in_swept_footprint_stop(yaw, ray, speed):
    scan = scan_with()
    set_ray(scan, ray, 0.24 if abs(ray) == math.pi/4 else 0.2)
    covered, blocked = lidar_motion_check(scan, transform(yaw, 0.004), speed, 0.18)
    assert covered and blocked


def test_coverage_cannot_pass_with_a_contiguous_blind_sector_or_narrow_fov():
    scan = scan_with()
    for i in range(310, 330):
        scan.ranges[i] = float('nan')
    assert not lidar_motion_check(scan, transform(), 0.2, 0.18)[0]
    scan = scan_with()
    scan.ranges = scan.ranges[200:440]
    scan.angle_min += 200 * scan.angle_increment
    assert not lidar_motion_check(scan, transform(), 0.2, 0.18)[0]


def test_tilted_or_invalid_laser_transform_is_rejected():
    scan = scan_with()
    bad = transform()
    bad.rotation.x, bad.rotation.w = math.sin(0.3), math.cos(0.3)
    assert not lidar_motion_check(scan, bad, 0.2, 0.18)[0]
    bad.rotation.x = float('nan')
    assert not lidar_motion_check(scan, bad, 0.2, 0.18)[0]


def test_timestamp_is_relative_to_ros_clock_and_rejects_future_old_and_zero():
    now_ns = 100 * 10**9
    assert stamp_fresh(Time(sec=99, nanosec=500000000), now_ns)
    assert not stamp_fresh(Time(sec=1), now_ns)
    assert not stamp_fresh(Time(sec=101), now_ns)
    assert not stamp_fresh(Time(), now_ns)
    assert stamp_fresh(Time(sec=2), 2*10**9)  # valid after simulation clock reset


def test_repeated_timestamp_does_not_refresh_progress(monkeypatch):
    now = [1.0]
    monkeypatch.setattr('fleet_control.safety.time.monotonic', lambda: now[0])
    fake = SimpleNamespace(scans={}, odoms={}, stamp_progress={'scan':{}, 'odom':{}})
    scan = scan_with()
    scan.header.stamp = Time(sec=2)
    Safety.sensor_cb(fake, 'scan', 'robot1', scan)
    now[0] = 2.0
    Safety.sensor_cb(fake, 'scan', 'robot1', scan)
    assert fake.scans['robot1'][0] == 2.0
    assert fake.stamp_progress['scan']['robot1'][1] == 1.0
    scan.header.stamp = Time(sec=3)
    Safety.sensor_cb(fake, 'scan', 'robot1', scan)
    assert fake.stamp_progress['scan']['robot1'][1] == 2.0


def test_lease_transition_epoch_change_and_heartbeat_loss_drop_commands(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('fleet_control.safety.time.monotonic', lambda: now[0])
    fake = SimpleNamespace(cfg={'robots':[{'name':'robot1'}]}, pubs={'robot1':None},
                           commands={}, state={}, lease=0.0)
    lease = {'active':['robot1'], 'estop':False, 'region_enabled':False,
             'regions':[[0,0,1,1]], 'generation':1, 'command_epochs':{'robot1':10}}
    def send(): Safety.lease_cb(fake, String(data=json.dumps(lease)))
    send()
    Safety.command_cb(fake, 'robot1', Twist())
    assert 'robot1' in fake.commands
    lease['command_epochs']['robot1'] = 11
    send()
    assert not fake.commands
    Safety.command_cb(fake, 'robot1', Twist())
    lease['active'] = []
    send()
    Safety.command_cb(fake, 'robot1', Twist())
    assert not fake.commands
    lease['active'] = ['robot1']
    send()
    assert not fake.commands
    Safety.command_cb(fake, 'robot1', Twist())
    now[0] += 1.0
    send()
    assert not fake.commands
    Safety.command_cb(fake, 'robot1', Twist())
    lease['command_epochs']['robot1'] = True
    send()
    assert not fake.commands and not fake.state


def test_tick_fails_closed_on_frozen_stamp_and_missing_laser_tf():
    import time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from tf2_ros import LookupException

    now = time.monotonic()
    clock = Clock(clock_type=ClockType.ROS_TIME)
    scan = scan_with()
    scan.header.frame_id = 'robot1/rplidar_link'
    scan.header.stamp = clock.now().to_msg()
    odom = Odometry()
    odom.header.stamp = clock.now().to_msg()
    cmd = Twist()
    cmd.linear.x = 0.2
    tf = TransformStamped()
    tf.header.stamp = clock.now().to_msg()
    tf.transform.rotation.w = 1.0
    def lookup(target, *_args):
        if target != 'map':
            raise LookupException('unavailable')
        return tf
    output, reports = [], []
    fake = SimpleNamespace(cfg={'robots':[{'name':'robot1'}]}, lease=now,
        state={'active':['robot1'], 'estop':False, 'region_enabled':False, 'command_epochs':{'robot1':5}},
        scans={'robot1':(now,scan)}, odoms={'robot1':(now,odom)},
        stamp_progress={'scan':{'robot1':((scan.header.stamp.sec,scan.header.stamp.nanosec), now-1)},
                        'odom':{'robot1':((odom.header.stamp.sec,odom.header.stamp.nanosec), now)}},
        commands={'robot1':(now,cmd)}, max_speed=0.2, robot_radius=0.18,
        buffer=SimpleNamespace(lookup_transform=lookup), get_clock=lambda:clock,
        pubs={'robot1':SimpleNamespace(publish=output.append)},
        status=SimpleNamespace(publish=lambda msg:reports.append(json.loads(msg.data))), last_diagnostics=now)
    Safety.tick(fake)
    assert output[-1].linear.x == 0
    assert reports[-1]['robot1']['reason'] == '雷达时间戳超时或无效'
    assert reports[-1]['robot1']['command_epoch'] == 5
    fake.stamp_progress['scan']['robot1'] = (None, now)
    Safety.tick(fake)
    assert output[-1].linear.x == 0
    assert reports[-1]['robot1']['reason'] == '雷达坐标变换不可用'


def test_rotation_checks_circle_and_full_coverage_without_fictitious_translation():
    scan = scan_with()
    set_ray(scan, 0.0, 0.23)  # Outside 0.18 m body + 0.02 m clearance.
    forward = lidar_motion_check(scan, transform(), 0.0, 0.18, angular_speed=0.6)
    rear = lidar_motion_check(scan, transform(), 0.0, 0.18, check_backward=True)
    assert forward == rear == (True, False)
    set_ray(scan, 0.0, 0.19)
    assert lidar_motion_check(scan, transform(), 0.0, 0.18, angular_speed=0.6) == (True, True)
    scan = scan_with()
    for i in range(30, 60):
        scan.ranges[i] = float('nan')
    assert lidar_motion_check(scan, transform(), 0.0, 0.18)[0]
    assert not lidar_motion_check(scan, transform(), 0.0, 0.18, check_backward=True)[0]


def test_peer_sweep_protects_against_approach_crossing_and_reverse():
    from fleet_control.health import peer_motion_clear
    # Measured 0.336 m gap from the failed fleet acceptance must never pass.
    assert not peer_motion_clear((0, 0), 0, 0.2, 0.0, [(0.336, 0)], 0.18, 0.2)
    # A crossing peer inside the stopping envelope is also stopped.
    assert not peer_motion_clear((0, 0), 0, 0.2, 0.4, [(0.6, 0.3)], 0.18, 0.2)
    assert not peer_motion_clear((0, 0), 0, -0.2, 0.0, [(-0.7, 0)], 0.18, 0.2)
    # Stale but still permitted TF increases the swept safety margin.
    assert peer_motion_clear((0, 0), 0, 0.2, 0.0, [(0.85, 0, 0.0)], 0.18, 0.2)
    assert not peer_motion_clear((0, 0), 0, 0.2, 0.0, [(0.85, 0, 0.8)], 0.18, 0.2)
    # Moving away from a sufficiently separated peer and ordinary navigation remain possible.
    assert peer_motion_clear((0, 0), 0, 0.2, 0.0, [(-0.7, 0)], 0.18, 0.2)
    assert peer_motion_clear((0, 0), 0, 0.2, 0.0, [(2.0, 0)], 0.18, 0.2)


def test_peer_missing_or_stale_stops_real_safety_tick():
    import time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from tf2_ros import LookupException
    now = time.monotonic()
    clock = Clock(clock_type=ClockType.ROS_TIME)
    names = ['robot1', 'robot2', 'robot3']
    scan = scan_with()
    scan.header.stamp = clock.now().to_msg()
    scan.header.frame_id = 'robot1/rplidar_link'
    odoms = {}
    progress = {'odom':{}, 'scan':{}}
    scans = {}
    for name in names:
        odom = Odometry()
        odom.header.stamp = clock.now().to_msg()
        odoms[name] = (now, odom)
        scans[name] = (now, scan)
        progress['odom'][name] = (None, now)
        progress['scan'][name] = (None, now)
    poses = {}
    for name, x in zip(names, (0, 0.336, 3.0)):
        tf = TransformStamped()
        tf.header.stamp = clock.now().to_msg()
        tf.transform.rotation.w = 1.0
        tf.transform.translation.x = float(x)
        poses[name] = tf
    def lookup(target, source, *_args):
        if target == 'map':
            name = source.split('/')[0]
            if name not in poses:
                raise LookupException('lost peer')
            return poses[name]
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        return tf
    outputs, reports = {n:[] for n in names}, []
    cmd = Twist()
    cmd.linear.x = 0.2
    fake = SimpleNamespace(cfg={'robots':[{'name':n} for n in names]}, lease=now,
        state={'active':['robot1'], 'estop':False, 'region_enabled':False},
        scans=scans, odoms=odoms, stamp_progress=progress, commands={'robot1':(now,cmd)},
        max_speed=0.2, robot_radius=0.18, buffer=SimpleNamespace(lookup_transform=lookup),
        get_clock=lambda:clock, pubs={n:SimpleNamespace(publish=outputs[n].append) for n in names},
        status=SimpleNamespace(publish=lambda msg:reports.append(json.loads(msg.data))), last_diagnostics=now)
    Safety.tick(fake)
    assert reports[-1]['robot1']['reason'] == '机器人间距保护'
    assert outputs['robot1'][-1].linear.x == 0
    poses['robot2'].transform.translation.x = 3.0
    Safety.tick(fake)
    assert outputs['robot1'][-1].linear.x == 0.2
    progress['odom']['robot2'] = (None, now-1)
    Safety.tick(fake)
    assert reports[-1]['robot1']['reason'] == '其他机器人定位或里程计不可用'
    assert outputs['robot1'][-1].linear.x == 0
    progress['odom']['robot2'] = (None, now)
    poses.pop('robot2')
    Safety.tick(fake)
    assert reports[-1]['robot1']['reason'] == '其他机器人定位或里程计不可用'
    assert outputs['robot1'][-1].linear.x == 0
