"""Independent, steady-clock safety for exactly one robot's actuator."""
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from fleet_interfaces.msg import (PeerState, MappingState, CoordinationEvent,
                                   LocalAuthorization, MissionCommand)
from fleet_mapping.registration import compose
from .health import scan_health, valid_odom, stamp_fresh, lidar_motion_check, peer_motion_clear
from .peer_protocol import (MEMBERS, ReplayWindow, GrantTracker, lost_peer_radius, finite_pose,
                            command_targets_boot, command_is_current)
from .peer_common import (STATE_QOS, EVENT_QOS, load_config, loads, dumps,
                          peer_topic, pose_list, local_frame)
from .peer_persistence import Journal
from .peer_transport import PeerInbox

SENSOR_TTL = .65  # Leave scheduling margin inside the 0.8 s stop requirement.


def inside_region(name, point, limits, margin):
    if name not in MEMBERS or not finite_pose(point) or not math.isfinite(margin) or margin < 0:
        return False
    if not limits.get('region_enabled'):
        return True
    regions = limits.get('regions')
    if regions:
        if (not isinstance(regions, list) or len(regions) != 3
                or any(not isinstance(b, list) or len(b) != 4
                       or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in b)
                       or b[0] >= b[2] or b[1] >= b[3] for b in regions)):
            return False
        b = regions[MEMBERS.index(name)]
        return b[0]+margin <= point[0] <= b[2]-margin and b[1]+margin <= point[1] <= b[3]-margin
    seeds = limits.get('seeds', {})
    if not isinstance(seeds, dict) or set(seeds) != set(MEMBERS) or not all(finite_pose(p) for p in seeds.values()):
        return False
    a = seeds[name]
    own = math.dist(point[:2], a[:2])**2
    for n, b in seeds.items():
        if n != name:
            separation = math.dist(a[:2], b[:2])
            if separation < .1 or (math.dist(point[:2], b[:2])**2-own)/(2*separation) < margin:
                return False
    return True


class LocalSafety(Node):
    def __init__(self):
        super().__init__('local_safety')
        self.cfg = load_config(self)
        self.name, self.boot, self.session = self.cfg['robot_id'], self.cfg['boot_id'], self.cfg['session_id']
        self.radius, self.speed = self.cfg['robot_radius'], self.cfg['max_speed']
        self.inbox = PeerInbox(self, self.cfg)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan_data, self.odom_data, self.command_data = None, None, None
        self.progress = {}
        self.peers, self.mapping, self.authorization = {}, None, None
        self.peer_last = {}
        self.state_replay, self.event_replay = ReplayWindow(), ReplayWindow()
        self.grants = GrantTracker()
        self.auth_sequence = 0
        self.estop = False
        self.stop_revision = 0
        self.command_order, self.command_id = (0, '', ''), ''
        self.pending_commands = {}
        self.clear_command = None
        self.mission_votes = {}
        self.journal = Journal(self.cfg['run_dir'], self.name, 'safety_fence', self.session)
        self.estop = bool(self.journal.data.get('estop', False))
        self.command_order = tuple(self.journal.data.get('command_order', [0, '', '']))
        self.command_id = self.command_order[2]
        self.stop_revision = self.journal.data.get('stop_revision', 0)
        self.pub = self.create_publisher(Twist, f'/{self.name}/diffdrive_controller/cmd_vel_unstamped', 1)
        self.statuspub = self.create_publisher(String, f'/{self.name}/safety_state', STATE_QOS)
        self.create_subscription(LaserScan, f'/{self.name}/scan', lambda m: self.sensor('scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{self.name}/odom', lambda m: self.sensor('odom', m), qos_profile_sensor_data)
        self.create_subscription(Twist, f'/{self.name}/cmd_vel_nav', self.command, 1)
        self.create_subscription(MappingState, peer_topic(self.name, 'mapping'), self.mapping_cb, STATE_QOS)
        self.create_subscription(LocalAuthorization, f'/{self.name}/authorization', self.authorize, 10)
        self.create_subscription(MissionCommand, '/fleet/peer/mission', self.mission, EVENT_QOS)
        for robot in MEMBERS:
            self.create_subscription(PeerState, peer_topic(robot, 'state'),
                                     lambda m, n=robot: self.inbox.receive(n, 'state', self.peer_state, n, m), STATE_QOS)
            self.create_subscription(CoordinationEvent, peer_topic(robot, 'event'),
                                     lambda m, n=robot: self.inbox.receive(n, 'event', self.event, n, m), EVENT_QOS)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def sensor(self, kind, message):
        now = time.monotonic()
        if kind == 'scan':
            self.scan_data = now, message
        else:
            self.odom_data = now, message
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        previous = self.progress.get(kind)
        if previous is None or previous[0] != stamp:
            self.progress[kind] = stamp, now

    def command(self, message):
        self.command_data = time.monotonic(), message

    def mapping_cb(self, message):
        if (message.session_id == self.session and message.robot_id == self.name
                and message.boot_id == self.boot):
            try:
                details = loads(message.details_json)
                ages = details.get('map_ages')
                transforms = details.get('transforms')
                if (not isinstance(ages, dict) or self.name not in ages
                        or not isinstance(transforms, dict)
                        or any(n not in MEMBERS or not isinstance(v, (float, int))
                               or isinstance(v, bool) or not math.isfinite(v) or v < 0
                               for n, v in ages.items())):
                    return
                for name, transform in transforms.items():
                    if (name not in MEMBERS or not isinstance(transform, dict)
                            or not isinstance(transform.get('boot_id'), str)
                            or not transform['boot_id'] or not finite_pose(transform.get('transform'))
                            or not isinstance(transform.get('sigma'), (float, int))
                            or isinstance(transform['sigma'], bool)
                            or not math.isfinite(transform['sigma']) or transform['sigma'] < 0):
                        return
            except ValueError:
                return
            if self.mapping and self.mapping[1].frame_epoch != message.frame_epoch:
                self.peer_last = {}
            self.mapping = time.monotonic(), message, details

    def peer_state(self, robot, message):
        if (message.session_id == self.session and message.robot_id == robot
                and self.state_replay.accept(robot, message.boot_id, message.sequence, message.map_epoch)):
            self.peers[robot] = time.monotonic(), message

    def event(self, robot, message):
        if (message.session_id != self.session or message.robot_id != robot
                or robot not in self.peers or self.peers[robot][1].boot_id != message.boot_id
                or not self.event_replay.accept(robot, message.boot_id, message.sequence)):
            return
        if message.phase == CoordinationEvent.MISSION_RELAY:
            if message.task.command_id == message.mission_id:
                self.mission(message.task)
            return
        if (message.phase == message.REQUEST and message.owner_id == self.name
                and message.owner_boot == self.boot and robot == self.name):
            self.grants.requested(message.request_id, message.mission_id, message.resource,
                message.request_sequence, message.sent_steady_ns*1e-9, time.monotonic())
        if (message.phase == message.GRANT and message.owner_id == self.name
                and message.owner_boot == self.boot):
            self.grants.granted(robot, message.boot_id, message.request_id, message.mission_id,
                                message.resource, message.request_sequence)
        if message.phase == message.MISSION_VOTE:
            try:
                accepted = loads(message.payload_json).get('accepted') is True
            except ValueError:
                accepted = False
            if accepted:
                self.mission_votes.setdefault(message.mission_id, {})[robot] = message.boot_id

    def mission(self, message):
        if (message.session_id != self.session or not command_targets_boot(message, self.name, self.boot)
                or not command_is_current(message, time.monotonic(), self.peers)):
            return
        order = (message.revision, message.issuer_id, message.command_id)
        if order < self.command_order:
            return
        if message.kind in ('explore', 'pause', 'estop', 'clear-estop', 'formation', 'regions', 'patrol'):
            self.pending_commands[message.command_id] = order
            if len(self.pending_commands) > 64:
                self.pending_commands.pop(next(iter(self.pending_commands)))
            if message.kind in ('pause', 'estop') and order > self.command_order:
                self.command_order, self.command_id = order, message.command_id
                self.journal.update(command_order=list(order))
        if message.kind in ('pause', 'estop') and message.revision >= self.stop_revision:
            self.stop_revision = message.revision
            self.command_data = None
            if message.kind == 'estop':
                self.estop = True
            self.journal.update(estop=self.estop, stop_revision=self.stop_revision)
        if message.kind == 'clear-estop' and message.revision > self.stop_revision:
            self.clear_command = message.command_id

    def authorize(self, message):
        if (message.session_id != self.session or message.robot_id != self.name
                or message.boot_id != self.boot or message.sequence <= self.auth_sequence):
            return
        self.auth_sequence = message.sequence
        old = self.authorization[1] if self.authorization else None
        if old is None or (old.request_id, old.mission_id) != (message.request_id, message.mission_id):
            self.command_data = None
        self.authorization = time.monotonic(), message
        if message.estop:
            if not self.estop:
                self.estop = True
                self.journal.update(estop=True)

    def tick(self):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
        out = Twist()
        reason = ''
        scan_valid, odom_valid, localization_ok = False, False, False
        pose = None
        scan_age = now-self.scan_data[0] if self.scan_data else None
        odom_age = now-self.odom_data[0] if self.odom_data else None
        boots = {n: p.boot_id for n, (received, p) in self.peers.items()
                 if now-received < .8}
        if self.mapping:
            try:
                transforms = loads(self.mapping[1].details_json)['transforms']
                for other, (received, peer) in self.peers.items():
                    tr = transforms.get(other)
                    if other != self.name and tr and tr['boot_id'] == peer.boot_id and peer.local_ready:
                        self.peer_last[other] = (compose(tr['transform'], pose_list(peer.pose)), received, tr.get('sigma', .05))
            except (ValueError, KeyError, TypeError):
                pass
        if self.scan_data:
            scan_valid = (scan_health(self.scan_data[1])[0] and scan_age < SENSOR_TTL
                          and stamp_fresh(self.scan_data[1].header.stamp, ros_now, max_age=SENSOR_TTL)
                          and now-self.progress.get('scan', (None, 0))[1] < SENSOR_TTL)
        if self.odom_data:
            odom_valid = (valid_odom(self.odom_data[1]) and odom_age < SENSOR_TTL
                          and stamp_fresh(self.odom_data[1].header.stamp, ros_now, max_age=SENSOR_TTL)
                          and now-self.progress.get('odom', (None, 0))[1] < SENSOR_TTL)
        try:
            t = self.buffer.lookup_transform(local_frame(self.name, self.cfg['map_epoch']), self.name+'/base_link', rclpy.time.Time())
            p, q = t.transform.translation, t.transform.rotation
            pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            localization_ok = all(math.isfinite(v) for v in pose) and stamp_fresh(t.header.stamp, ros_now)
        except TransformException:
            pass
        auth = self.authorization[1] if self.authorization else None
        if auth and auth.mission_id in self.pending_commands:
            order = self.pending_commands[auth.mission_id]
            votes = self.mission_votes.get(auth.mission_id, {})
            if order > self.command_order and sum(boots.get(n) == boot for n, boot in votes.items()) >= 2:
                self.command_order, self.command_id = order, auth.mission_id
                self.journal.update(command_order=list(order))
        if self.estop and auth and not auth.estop and auth.mission_id == self.clear_command:
            votes = self.mission_votes.get(self.clear_command, {})
            if sum(boots.get(n) == boot for n, boot in votes.items()) >= 2:
                self.estop = False
                self.journal.update(estop=False)
        if self.estop:
            reason = '急停锁定'
        elif auth is None or now-self.authorization[0] > .5:
            reason = '本车授权心跳中断'
        elif not auth.enabled:
            reason = '待命'
        elif auth.mission_id != self.command_id:
            reason = '任务已被新命令中止'
        elif not self.grants.valid(auth.request_id, auth.mission_id, auth.resource, now, boots):
            reason = '缺少有效多数通行授权'
        elif not scan_valid:
            reason = '雷达超时或无效'
        elif not odom_valid:
            reason = '里程计超时或无效'
        elif not localization_ok:
            reason = '本车定位超时或无效'
        elif self.mapping is None or now-self.mapping[0] > 1.5:
            reason = '本车地图状态超时'
        elif not auth.bootstrap and not self.mapping[1].aligned:
            reason = '本车尚未重新定位'
        elif self.mapping[2].get('map_ages', {}).get(self.name, math.inf) > 3.:
            reason = '本车 SLAM 地图超时'
        elif not self.command_data or now-self.command_data[0] > .5:
            reason = '等待新速度'
        else:
            cmd = self.command_data[1]
            if not all(math.isfinite(v) for v in (cmd.linear.x, cmd.linear.y, cmd.linear.z,
                                                   cmd.angular.x, cmd.angular.y, cmd.angular.z)):
                reason = '非法速度'
            elif (auth.bootstrap and (auth.resource != 'team_motion' or auth.maximum_speed > .080001)
                  or not 0 < auth.maximum_speed <= self.speed):
                reason = '非法运动模式或限速'
            elif auth.frame_epoch != self.mapping[1].frame_epoch:
                reason = '地图参考系已改变'
            else:
                out.linear.x = max(-auth.maximum_speed, min(auth.maximum_speed, cmd.linear.x))
                out.angular.z = max(-.6, min(.6, cmd.angular.z))
                if abs(out.linear.x) <= .001:
                    out.linear.x = 0.
                if abs(out.angular.z) <= .001:
                    out.angular.z = 0.
                if out.linear.x or out.angular.z:
                    try:
                        scan = self.scan_data[1]
                        laser = self.buffer.lookup_transform(self.name+'/base_link', scan.header.frame_id,
                                                           rclpy.time.Time.from_msg(scan.header.stamp))
                        covered, blocked = lidar_motion_check(scan, laser.transform, out.linear.x,
                                                              self.radius, out.angular.z)
                        if out.angular.z:
                            back, back_blocked = lidar_motion_check(scan, laser.transform, 0., self.radius,
                                                                   check_backward=out.linear.x >= 0)
                            covered, blocked = covered and back, blocked or back_blocked
                        if not covered:
                            reason = '运动方向雷达数据无效'
                        elif blocked:
                            reason = '近障停车'
                    except (TransformException, ValueError, OverflowError):
                        reason = '雷达坐标变换不可用'
                if not reason and (out.linear.x or out.angular.z) and not auth.bootstrap:
                    try:
                        details = loads(self.mapping[1].details_json)
                        transforms = details['transforms']
                        team_from_local = pose_list(self.mapping[1].team_from_local)
                        from fleet_mapping.registration import inverse
                        local_from_team = inverse(team_from_local)
                        fresh = []
                        for other in MEMBERS:
                            if other == self.name:
                                continue
                            value, tr = self.peers.get(other), transforms.get(other)
                            localized = bool(value and tr and tr['boot_id'] == value[1].boot_id and value[1].local_ready)
                            if localized:
                                received, peer = value
                                team_pose = compose(tr['transform'], pose_list(peer.pose))
                                self.peer_last[other] = (team_pose, received, tr.get('sigma', .05))
                            if other not in self.peer_last:
                                raise ValueError('同伴尚未定位，不能确认运动区域安全')
                            team_pose, received, sigma = self.peer_last[other]
                            other_pose = compose(local_from_team, team_pose)
                            age = now-received
                            if age <= .8 and localized:
                                fresh.append((*other_pose[:2], age))
                            else:
                                radius = lost_peer_radius(self.radius, self.speed, age, sigma)
                                for step in range(7):
                                    dt = step*.1
                                    predicted = (pose[0]+math.cos(pose[2])*out.linear.x*dt,
                                                 pose[1]+math.sin(pose[2])*out.linear.x*dt)
                                    if math.dist(predicted, other_pose[:2]) < self.radius+radius:
                                        raise ValueError('失联机器人可能占用当前区域')
                        if not peer_motion_clear(pose[:2], pose[2], out.linear.x, out.angular.z,
                                                 fresh, self.radius, self.speed):
                            reason = '机器人间距保护'
                        limits = loads(auth.limits_json) if auth.limits_json else {}
                        predicted = (pose[0]+math.cos(pose[2])*out.linear.x*.6,
                                     pose[1]+math.sin(pose[2])*out.linear.x*.6, pose[2])
                        if not inside_region(self.name, compose(team_from_local, predicted), limits, self.radius+.02):
                            reason = '分区边界保护'
                    except (ValueError, KeyError, TypeError) as error:
                        reason = str(error)
        if reason:
            out = Twist()
        self.pub.publish(out)
        report = dict(reason=reason, moving_allowed=not bool(reason), v=out.linear.x, w=out.angular.z,
                      lease_valid=bool(auth and self.grants.valid(auth.request_id, auth.mission_id,
                                                                 auth.resource, now, boots)),
                      mission_id=auth.mission_id if auth else '',
                      scan_valid=bool(scan_valid), odom_valid=bool(odom_valid),
                      localization_ok=bool(localization_ok), scan_age=scan_age, odom_age=odom_age,
                      estop=self.estop, command_epoch=auth.request_id if auth else '')
        self.statuspub.publish(String(data=dumps(report)))


def main():
    from .ros_runtime import RosRuntime
    with RosRuntime() as runtime:
        node = LocalSafety()
        try:
            runtime.spin(node)
        finally:
            if rclpy.ok():
                node.pub.publish(Twist())
            node.destroy_node()
