"""Offline assertions over independent actuator, guard and physical samples."""
import math
import json
import threading
import time
from collections import deque


def source_steady(info, received, wall_ns):
    """DDS uses host wall time, independently of ROS /clock and callback delay."""
    source = info.get('source_timestamp', 0)
    if not isinstance(source, int) or source <= 0:
        raise ValueError('DDS 未提供有效的源时间戳')
    stamp = received+(source-wall_ns)/1e9
    if stamp > received+.02 or received-stamp > 10.:
        raise ValueError('DDS 源时间戳与本机观测时钟不一致或消息积压超过 10 秒')
    return stamp


class MotionRecorder:
    """Drain an unspun observation node independently of blocking CLI calls.

    Humble's executor discards MessageInfo. Its subscription handle exposes
    take_message() with the DDS source timestamp, which we retain here. The
    node is never added to an executor: this is its sole message consumer.
    """
    def __init__(self, context, members):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        self.node = Node('fault_motion_recorder', context=context)
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.error = None
        self.commands = {n: deque(maxlen=4000) for n in members}
        self.guards = {n: deque(maxlen=4000) for n in members}
        self.raw = {n: deque(maxlen=8000) for n in members}
        self.subscriptions = []
        for n in members:
            for kind, typ, topic, qos in (
                    ('command', Twist, f'/{n}/diffdrive_controller/cmd_vel_unstamped', 200),
                    ('guard', String, f'/{n}/safety_state',
                     QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL))):
                sub = self.node.create_subscription(typ, topic, lambda m: None, qos)
                self.subscriptions.append((n, kind, typ, sub))
        self.thread = threading.Thread(target=self.collect, daemon=True)
        self.thread.start()

    def collect(self):
        try:
            while not self.stop.is_set():
                for name, kind, typ, sub in self.subscriptions:
                    for _ in range(200):
                        with sub.handle:
                            packet = sub.handle.take_message(typ, False)
                        if packet is None:
                            break
                        message, info = packet
                        received, wall = time.monotonic(), time.time_ns()
                        stamp = source_steady(info, received, wall)
                        payload = ([message.linear.x, message.angular.z] if kind == 'command'
                                   else json.loads(message.data))
                        with self.lock:
                            self.raw[name].append(dict(kind=kind, source_timestamp=info['source_timestamp'],
                                steady_time=stamp, received_steady=received, received_wall_ns=wall, payload=payload))
                            if kind == 'command':
                                self.commands[name].append((stamp, *payload))
                            else:
                                self.guards[name].append((stamp, payload))
                self.stop.wait(.005)
        except Exception as error:
            self.error = error

    def snapshot(self, name):
        if self.error:
            raise RuntimeError('独立运动采集失败') from self.error
        with self.lock:
            return list(self.commands[name]), list(self.guards[name]), list(self.raw[name])

    def latest(self, name):
        _, guards, _ = self.snapshot(name)
        return guards[-1] if guards else None

    def close(self):
        self.stop.set()
        self.thread.join()
        self.node.destroy_node()


def save_audit(path, commands, guards, truth, robot, start, end, *, raw=(), **options):
    """Write the actual bounded inputs before asserting, including on failure."""
    evidence = dict(robot=robot, start=start, end=end, options=options,
        commands=[r for r in commands if start-.2 <= r[0] <= end+.2],
        guards=[r for r in guards if start-.2 <= r[0] <= end+.2],
        truth=[r for r in truth if start-.2 <= r.get('steady_time', -math.inf) <= end+.2],
        raw=[r for r in raw if start-.2 <= r['steady_time'] <= end+.2], passed=False)
    path.parent.mkdir(exist_ok=True)
    with path.open('x') as stream:
        json.dump(evidence, stream, ensure_ascii=False, indent=2)
    try:
        result = audit_motion(commands, guards, truth, robot, start, end, **options)
    except Exception as error:
        evidence['error'] = str(error)
        raise
    else:
        evidence.update(passed=True, result=result)
        return dict(result, evidence=str(path))
    finally:
        path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2))


def audit_motion(commands, guards, truth, robot, start, end, *, require_motion=False, stationary=False):
    cmd = [(t, v, w) for t, v, w in commands if start <= t <= end]
    status = [(t, s) for t, s in guards if start-.15 <= t <= end+.15]
    poses = [(s['steady_time'], s['robots'][robot]) for s in truth
             if start <= s.get('steady_time', -math.inf) <= end and robot in s['robots']]
    if len(cmd) < 5 or len(status) < 5 or len(poses) < 5 or poses[-1][0]-poses[0][0] < (end-start)*.7:
        raise AssertionError('网络故障缺少连续的执行器、安全状态或独立真值证据')
    nonzero = [(t, v, w) for t, v, w in cmd if abs(v) > .001 or abs(w) > .001]
    for t, v, w in nonzero:
        if not any(abs(t-stamp) <= .15 and s.get('moving_allowed') and s.get('lease_valid')
                   and abs(v-s.get('v', math.inf)) < .001 and abs(w-s.get('w', math.inf)) < .001
                   for stamp, s in status):
            nearest = sorted(status, key=lambda row: abs(row[0]-t))[:3]
            raise AssertionError('执行器非零速度缺少相邻的有效多数授权证据：'+
                                 repr(dict(command=(t, v, w), nearest_guards=nearest)))
    distance = max(math.dist(p[:2], poses[0][1][:2]) for _, p in poses)
    yaw = max(abs(math.atan2(math.sin(p[2]-poses[0][1][2]), math.cos(p[2]-poses[0][1][2]))) for _, p in poses)
    if require_motion and (not nonzero or distance < .05):
        raise AssertionError('受影响成员没有真实平移证据')
    if stationary and (nonzero or distance > .03 or yaw > .04):
        raise AssertionError('恢复通信或迟到消息使已中止任务重新运动')
    return dict(robot=robot, start=start, end=end, command_samples=len(cmd),
                guard_samples=len(status), truth_samples=len(poses), nonzero_samples=len(nonzero),
                displacement=distance, yaw_displacement=yaw)
