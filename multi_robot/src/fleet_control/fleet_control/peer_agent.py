"""A robot's task state machine and peer voter; never drives an actuator."""
from concurrent.futures import ThreadPoolExecutor
import copy
import math
from pathlib import Path
import threading
import time
import uuid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from cartographer_ros_msgs.srv import WriteState
from tf2_ros import Buffer, TransformListener, TransformException
from fleet_interfaces.msg import (PeerState, MappingState, CoordinationEvent,
                                   MissionCommand, LocalAuthorization)
from fleet_mapping.grid import valid_grid_message, save_map
from fleet_mapping.registration import (angle, compose, inverse, dynamic_fusion,
                                        transform_points, voronoi_mask)
from .geometry import (FORMATION_POSITION_TOLERANCE, FORMATION_YAW_TOLERANCE,
                       Grid, formation, plan_formation, formation_pass, region_return,
                       region_entry_margin, private_observation_grid, yaw)
from .health import scan_health, valid_odom, stamp_fresh
from .peer_protocol import (MEMBERS, ReplayWindow, LeaseVoter, GrantTracker, digest,
                            finite_pose, choose_motion_owner, command_targets_boot, command_is_current)
from .peer_common import (STATE_QOS, EVENT_QOS, load_config, dumps, loads, peer_topic,
                          pose_list, set_pose, local_frame)
from .local_safety import inside_region
from .peer_persistence import Journal
from .peer_plans import validate_formation_plan, agreed_formation_plan
from .peer_transport import PeerInbox
from .peer_auction import valid_offer, quote_frontiers, frontier_winner

FORMATION_LOCALIZATION_SETTLE_SECONDS = 1.2
FORMATION_HEADING_CORRECTION_LIMIT = 2
FORMATION_HEADING_CORRECTION_TIMEOUT = 30.


class PeerAgent(Node):
    def __init__(self):
        super().__init__('peer_agent')
        self.cfg = load_config(self)
        self.name, self.boot, self.session = self.cfg['robot_id'], self.cfg['boot_id'], self.cfg['session_id']
        self.epoch = self.cfg['map_epoch']
        self.frame = local_frame(self.name, self.epoch)
        self.inbox = PeerInbox(self, self.cfg)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.closed = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.voter = LeaseVoter(self.name, time.monotonic())
        self.grants, self.states_replay, self.events_replay = GrantTracker(), ReplayWindow(), ReplayWindow()
        self.peers, self.mapping, self.grid, self.scan, self.odom_data = {}, None, None, None, None
        self.raw_grid = None
        self.odom_epoch = ''
        self.safety, self.pose, self.home, self.initial_team = {}, None, None, None
        self.pose_sim_time, self.pose_steady_time = None, None
        self.alignment_verified, self.alignment_hold, self.verified_at = False, None, None
        self.alignment_ever_verified = False
        self.team_ready_since = None
        self.sequence, self.event_sequence, self.auth_sequence = 0, 0, 0
        self.nav_states, self.nav_futures, self.last_nav_check = {}, {}, 0.
        self.nav_ready, self.local_ready = False, False
        self.pending_mission, self.mission_votes = None, {}
        self.active_command, self.last_command_relay = None, 0.
        self.last_command_order, self.last_command, self.mission_id = (0, '', ''), None, ''
        self.mode, self.message, self.estop = 'BOOTSTRAP', '等待本车 SLAM、传感器和导航', False
        self.team_initialized = False
        self.work, self.goal_handle, self.goal_sending, self.goal_generation = None, None, False, 0
        self.cancel_dispatch_failed = False
        self.request_id, self.request_sequence, self.resource, self.release_pending = '', 0, '', None
        self.stopped_since = None
        self.seeds, self.regions, self.unrestricted = {}, None, False
        self.travel, self.distance, self.trail, self.last_odom_pose = 0., 0., [], None
        self.regional_travel = 0.
        self.blacklist, self.visits, self.bid, self.peer_bids = [], [], None, {}
        self.explore_done, self.empty_since, self.release_votes = False, None, {}
        self.quote_future, self.local_quotes, self.frontier_quotes = None, {}, {}
        self.last_quote, self.last_quote_emit = 0., 0.
        self.formation_plan, self.formation_plans, self.formation_votes = None, {}, {}
        self.formation_validations, self.formation_emit_at = {}, 0.
        self.formation_step, self.completed_step, self.formation_stable = 0, -1, None
        self.formation_arrival = None
        self.formation_heading_corrections = 0
        self.formation_settled = False
        self.patrol, self.patrol_index, self.patrol_cycle = None, 0, 0
        self.patrol_dwell_until, self.goal_pending_since = 0., 0.
        self.events, self.plan_points = [], []
        self.save_jobs, self.command_outcomes = set(), {}
        self.journal = Journal(self.cfg['run_dir'], self.name, 'agent_fence', self.session)
        self.estop = bool(self.journal.data.get('estop', False))
        self.team_initialized = bool(self.journal.data.get('team_initialized', False))
        self.last_command_order = tuple(self.journal.data.get('last_command_order', [0, '', '']))
        self.last_command = self.last_command_order[2] or None
        self.pool = ThreadPoolExecutor(max_workers=2)
        self.planner_future, self.last_plan = None, 0.
        self.statepub = self.create_publisher(PeerState, peer_topic(self.name, 'state'), STATE_QOS)
        self.eventpub = self.create_publisher(CoordinationEvent, peer_topic(self.name, 'event'), EVENT_QOS)
        self.authpub = self.create_publisher(LocalAuthorization, f'/{self.name}/authorization', 10)
        self.create_subscription(MappingState, peer_topic(self.name, 'mapping'), self.mapping_cb, STATE_QOS)
        self.create_subscription(OccupancyGrid, f'/{self.name}/shared_map', self.grid_cb, STATE_QOS)
        self.create_subscription(OccupancyGrid, f'/{self.name}/local_map', self.raw_grid_cb, STATE_QOS)
        self.create_subscription(LaserScan, f'/{self.name}/scan', lambda m: self.sensor('scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{self.name}/odom', lambda m: self.sensor('odom', m), qos_profile_sensor_data)
        self.create_subscription(String, f'/{self.name}/odometry_identity',
                                 lambda m: setattr(self, 'odom_epoch', m.data[:64]), STATE_QOS)
        self.create_subscription(String, f'/{self.name}/safety_state', self.safety_cb, STATE_QOS)
        self.create_subscription(NavPath, f'/{self.name}/plan', self.plan_cb, 1)
        self.create_subscription(MissionCommand, '/fleet/peer/mission', self.mission, EVENT_QOS)
        for robot in MEMBERS:
            self.create_subscription(PeerState, peer_topic(robot, 'state'),
                lambda m, n=robot: self.inbox.receive(n, 'state', self.peer_state, n, m), STATE_QOS)
            self.create_subscription(CoordinationEvent, peer_topic(robot, 'event'),
                lambda m, n=robot: self.inbox.receive(n, 'event', self.event_cb, n, m), EVENT_QOS)
        self.action = ActionClient(self, NavigateToPose, f'/{self.name}/navigate_to_pose', callback_group=self.group)
        self.nav_clients = {n: self.create_client(GetState, f'/{self.name}/{n}/get_state', callback_group=self.group)
                            for n in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')}
        self.write_state = self.create_client(WriteState, f'/{self.name}/write_state', callback_group=self.group)
        self.create_timer(.1, self.tick, callback_group=self.group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.log_dir = Path(self.cfg['run_dir'])/self.name
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log = (self.log_dir/'events.jsonl').open('a', buffering=1)

    def record(self, event, **values):
        if self.closed:
            return
        item = dict(event=event, robot=self.name, wall_time=time.time(), steady_time=time.monotonic(),
                    boot_id=self.boot, map_epoch=self.epoch, mission_id=self.mission_id,
                    sim_time=self.get_clock().now().nanoseconds*1e-9, **values)
        self.events.append(item)
        self.events = self.events[-12:]
        self.log.write(dumps(item)+'\n')

    def mapping_cb(self, message):
        if (message.session_id != self.session or message.boot_id != self.boot
                or message.robot_id != self.name or message.map_epoch != self.epoch):
            return
        try:
            details = loads(message.details_json)
        except ValueError:
            return
        with self.lock:
            old_frame = self.mapping[1].frame_epoch if self.mapping else ''
            self.mapping = time.monotonic(), message, details
            if old_frame and old_frame != message.frame_epoch:
                self.cancel()
                self.alignment_verified, self.alignment_hold, self.verified_at = False, None, None
                self.alignment_ever_verified = False
                self.team_ready_since = None
                self.initial_team = None
                self.seeds = {}
                self.regions = None

    def grid_cb(self, message):
        if valid_grid_message(message) and message.header.frame_id == self.frame:
            with self.lock:
                data = np.asarray(message.data, np.int16).reshape(message.info.height, message.info.width)
                old = self.grid
                origin = (message.info.origin.position.x, message.info.origin.position.y)
                if old is None or old.res != message.info.resolution or old.origin != origin or not np.array_equal(data, old.data):
                    self.grid = Grid(data, message.info.resolution, origin,
                                     self.cfg['robot_radius'], self.cfg['region_margin'])

    def raw_grid_cb(self, message):
        if valid_grid_message(message) and message.header.frame_id == self.frame:
            self.raw_grid = message

    def sensor(self, kind, message):
        with self.lock:
            now = time.monotonic()
            if kind == 'scan':
                self.scan = now, message
                return
            if valid_odom(message):
                p = message.pose.pose.position
                position = (p.x, p.y)
                if self.last_odom_pose:
                    ds = math.dist(position, self.last_odom_pose)
                    if ds < .15:
                        self.distance += ds
                        if self.mode in ('EXPLORING', 'ALIGNING'):
                            self.travel += ds
                        if self.mode == 'EXPLORING' and self.resource.startswith('region/'):
                            self.regional_travel += ds
                self.last_odom_pose = position
            self.odom_data = now, message

    def safety_cb(self, message):
        try:
            self.safety = loads(message.data)
        except ValueError:
            self.safety = {}

    def plan_cb(self, message):
        if message.header.frame_id == self.frame and len(message.poses) <= 20000:
            self.plan_points = [(p.pose.position.x, p.pose.position.y) for p in message.poses][::5][:500]

    def peer_state(self, robot, message):
        if (message.session_id == self.session and message.robot_id == robot
                and self.states_replay.accept(robot, message.boot_id, message.sequence, message.map_epoch)):
            try:
                details = loads(message.details_json)
                pose_list(message.pose)
            except ValueError:
                return
            with self.lock:
                self.peers[robot] = time.monotonic(), message, details

    def emit(self, *args, **kwargs):
        with self.lock:
            return self._emit(*args, **kwargs)

    def _emit(self, phase, *, mission=None, resource='', owner=None, owner_boot=None,
             request='', request_sequence=0, sent=0, stopped=False, payload=None, task=None):
        if self.closed or not rclpy.ok():
            return
        self.event_sequence += 1
        msg = CoordinationEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id, msg.robot_id, msg.boot_id = self.session, self.name, self.boot
        msg.sequence, msg.phase = self.event_sequence, phase
        msg.mission_id = self.mission_id if mission is None else mission
        msg.resource, msg.owner_id, msg.owner_boot = resource, owner or self.name, owner_boot or self.boot
        msg.request_id, msg.request_sequence, msg.sent_steady_ns = request, request_sequence, sent
        msg.stopped, msg.payload_json = stopped, dumps(payload or {})
        if task is not None:
            msg.task = copy.deepcopy(task)
        self.eventpub.publish(msg)
        self.event_cb(self.name, msg)

    def live(self):
        now = time.monotonic()
        return {n: data for n, data in self.peers.items() if now-data[0] < .8}

    def boots(self):
        return {n: value[1].boot_id for n, value in self.live().items()}

    def map_aligned(self):
        return bool(self.mapping and time.monotonic()-self.mapping[0] < 1.5 and self.mapping[1].aligned)

    def aligned(self):
        return self.map_aligned() and self.alignment_verified

    def verify_alignment(self, now):
        if not self.map_aligned() or not self.local_ready:
            self.alignment_verified, self.alignment_hold, self.verified_at = False, None, None
        elif not self.alignment_verified:
            if not self.stopped(now):
                self.alignment_hold = None
            else:
                hold = self.alignment_hold
                team_pose = compose(self.team_transform(), self.pose)
                revision = self.mapping[1].revision
                confirmed = self.mapping[2].get('confirmed_local_revision', 0)
                registered = self.mapping[2].get('registered_local_revision', 0)
                # During the original team's startup, a stationary continuity
                # check can confirm an existing two-witness registration.
                # A restarted identity still needs a new post-stop match.
                first_ready = (self.alignment_ever_verified
                               or registered >= (hold[3] if hold else revision)
                               or (registered > 0 and self.epoch == 1 and not self.team_initialized))
                if (hold is None or math.dist(hold[1][:2], self.pose[:2]) > .02
                        or abs(angle(hold[1][2]-self.pose[2])) > .01
                        or math.dist(hold[2][:2], team_pose[:2]) > .02
                        or abs(angle(hold[2][2]-team_pose[2])) > .01):
                    self.alignment_hold = now, self.pose, team_pose, revision
                elif now-hold[0] >= 3. and confirmed >= hold[3] and first_ready:
                    self.alignment_verified, self.verified_at = True, now
                    self.alignment_ever_verified = True
                    self.record('local_alignment_verified', pose=self.pose,
                                team_pose=team_pose, confirmed_local_revision=confirmed,
                                registered_local_revision=registered,
                                pose_sim_time=self.pose_sim_time, pose_steady_time=self.pose_steady_time)

    def team_transform(self):
        return pose_list(self.mapping[1].team_from_local) if self.mapping else (0., 0., 0.)

    def team_poses(self):
        result = {}
        if not self.aligned():
            return result
        transforms = self.mapping[2].get('transforms', {})
        for n, (_, peer, _) in self.live().items():
            tr = transforms.get(n)
            if tr and tr['boot_id'] == peer.boot_id and peer.local_ready:
                result[n] = compose(tr['transform'], pose_list(peer.pose))
        if self.pose:
            result[self.name] = compose(self.team_transform(), self.pose)
        return result

    def all_aligned(self):
        frame = self.mapping[1].frame_epoch if self.mapping else ''
        return len(self.team_poses()) == 3 and all(p[1].aligned and p[1].frame_epoch == frame
                                                   for p in self.live().values())

    def limits(self):
        region = (self.mode == 'EXPLORING' and self.all_aligned() and not self.unrestricted
                  and self.work is not None and self.work.get('kind') != 'return')
        return dict(region_enabled=region, seeds=self.seeds, regions=self.regions)

    def regions_exhausted(self):
        # Keep productive region holders parallel. One empty region is not a
        # reason to turn every other robot's exploration into a serial queue.
        live = self.live()
        return (len(live) >= 2 and self.explore_done
                and all(p[2].get('explore_done') for p in live.values()))

    def release_regions(self):
        self.cancel()
        self.unrestricted = True
        self.explore_done, self.empty_since = False, None
        self.bid, self.peer_bids, self.local_quotes, self.frontier_quotes = None, {}, {}, {}
        self.last_plan = 0.
        self.record('exploration_regions_released', reason='对等确认分区阻断或前沿耗尽')

    def exploration_finished(self, now):
        live = self.live()
        return bool(self.unrestricted and self.explore_done and not self.work and self.empty_since
            and now-self.empty_since > 10 and len(live) >= 2
            and all(peer.mission_id == self.mission_id and details.get('unrestricted')
                    and details.get('explore_done') for _, peer, details in live.values()))

    def homing_pending(self):
        if self.mode != 'EXPLORING' or self.unrestricted or not self.all_aligned() or len(self.seeds) != 3:
            return False
        limits = dict(region_enabled=True, seeds=self.seeds, regions=self.regions)
        margin = region_entry_margin(self.cfg['robot_radius'], self.cfg.get('region_margin', .35),
                                     self.cfg.get('resolution', .05))
        return any(not inside_region(n, p, limits, margin)
                   for n, p in self.team_poses().items())

    def motion_turn(self, now):
        if self.work and self.resource.startswith('region/') and self.homing_pending():
            return False
        if not self.work or self.resource != 'team_motion':
            return True
        held = self.grants.valid(self.request_id, self.mission_id, self.resource, now, self.boots())
        if held:
            return True
        contenders = {self.name: dict(kind=self.work['kind'], score=self.work.get('score', 0.),
            completed=self.patrol_cycle*len(self.patrol.get('routes', {}).get(self.name, []))+self.patrol_index if self.patrol else 0,
            held=held)}
        for name, (_, peer, details) in self.live().items():
            if (name != self.name and peer.mission_id == self.mission_id and peer.local_ready and peer.nav_ready
                    and details.get('work_resource') == 'team_motion'):
                contenders[name] = dict(kind=details.get('work_kind'), score=details.get('work_score', 0.),
                    completed=details.get('patrol_completed', 0), held=details.get('motion_lease_held', False))
        return choose_motion_owner(contenders) == self.name

    def validate_command(self, message):
        if message.kind not in ('explore', 'pause', 'estop', 'clear-estop', 'save', 'formation', 'regions', 'patrol'):
            raise ValueError('未知任务类型')
        if len(message.argument.encode()) > 65536:
            raise ValueError('任务过大')
        if self.estop and message.kind not in ('pause', 'estop', 'clear-estop', 'save'):
            raise ValueError('请先解除急停')
        if message.kind in ('formation', 'regions') and not self.all_aligned():
            raise ValueError('需要三车对齐后使用共同相对坐标')
        if message.kind == 'formation':
            if len(self.live()) != 3 or not all(p[1].nav_ready for p in self.live().values()):
                raise ValueError('三车队形需要全部成员及本车导航就绪')
            args = loads(message.argument)
            slots = formation(args['shape'], float(args['x']), float(args['y']), float(args['yaw']), float(args['spacing']))
            if self.grid is None:
                raise ValueError('本车地图不可用')
            allowed = self.grid.allowed()
            for slot in slots:
                local = compose(inverse(self.team_transform()), slot)
                cell = self.grid.cell(local)
                if not self.grid.inside(cell) or not allowed[cell]:
                    raise ValueError('队形槽位未知、受阻或超出观测地图')
        elif message.kind == 'regions':
            if self.mode not in ('IDLE', 'EXPLORED', 'FORMATION_READY', 'PATROL_COMPLETE'):
                raise ValueError('请先暂停运动任务再调整分区')
            rows = loads(message.argument)['regions']
            if len(rows) != 3 or any(len(r) != 4 or not all(math.isfinite(v) for v in r)
                                     or r[0] >= r[2] or r[1] >= r[3] for r in rows):
                raise ValueError('需要三个有效矩形')
            for i, a in enumerate(rows):
                seed = self.seeds.get(MEMBERS[i])
                margin = self.cfg['robot_radius']+self.cfg['region_margin']+.03
                if seed is None or not (a[0]+margin <= seed[0] <= a[2]-margin
                                         and a[1]+margin <= seed[1] <= a[3]-margin):
                    raise ValueError('每个手动分区须以安全余量包含该车固定的初始种子')
                for b in rows[:i]:
                    if min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1]):
                        raise ValueError('分区不能重叠')
        elif message.kind == 'patrol':
            if not self.aligned() or len(self.live()) < 2 or not self.nav_ready or not self.local_ready:
                raise ValueError('巡逻需要本车定位、导航及两个连通成员')
            plan = loads(message.argument)
            if plan.get('coordinate_frame') != self.mapping[1].frame_epoch:
                raise ValueError('巡逻点坐标系与本轮共同相对坐标不符；须显式转换或重新输入')
            routes = plan.get('routes')
            if (not isinstance(routes, dict) or set(routes)-set(MEMBERS)
                    or not isinstance(plan.get('cycles', 1), int) or isinstance(plan.get('cycles', 1), bool)
                    or not 1 <= plan.get('cycles', 1) <= 20
                    or not isinstance(plan.get('dwell', 0), (float, int)) or not 0 <= plan.get('dwell', 0) <= 30
                    or not isinstance(plan.get('goal_timeout', 120), (float, int)) or not 5 <= plan.get('goal_timeout', 120) <= 240):
                raise ValueError('巡逻格式错误')
            if not routes or not set(routes) <= set(self.live()):
                raise ValueError('巡逻路线包含失联成员')
            for n, route in routes.items():
                if not 1 <= len(route) <= 20 or not all(finite_pose(p) for p in route):
                    raise ValueError('巡逻航点无效')
            if self.name in routes:
                previous = self.pose
                local_from_team = inverse(self.team_transform())
                for p in routes[self.name]:
                    goal = compose(local_from_team, p)
                    if self.grid.astar(previous, goal, self.grid.allowed()) is None:
                        raise ValueError('巡逻航点未知、受阻或不可达')
                    previous = goal
        elif message.kind == 'explore' and not self.local_ready:
            raise ValueError('本车尚未就绪')

    def mission(self, message):
        if message.session_id != self.session or not message.command_id or not message.issuer_id:
            return
        with self.lock:
            if (not command_targets_boot(message, self.name, self.boot)
                    or not command_is_current(message, time.monotonic(), self.peers)):
                return
            if message.kind == 'save':
                if message.command_id not in self.save_jobs:
                    self.save_jobs.add(message.command_id)
                    self.pool.submit(self.save, copy.deepcopy(message))
                return
            order = (message.revision, message.issuer_id, message.command_id)
            if order < self.last_command_order:
                return
            if self.last_command == message.command_id:
                self.emit(CoordinationEvent.MISSION_VOTE, mission=message.command_id,
                          payload={'accepted': True, 'reason': '本启动实例已提交'})
                return
            try:
                self.validate_command(message)
                valid, reason = True, '已验证'
            except (ValueError, KeyError, TypeError) as error:
                valid, reason = False, str(error)
            if order > self.last_command_order:
                self.last_command_order = order
                self.pending_mission = copy.deepcopy(message) if valid else None
                if message.kind in ('pause', 'estop'):
                    self.cancel()
                    self.mode = 'ESTOP' if message.kind == 'estop' else 'IDLE'
                    self.estop = self.estop or message.kind == 'estop'
                self.journal.update(estop=self.estop, last_command_order=list(order))
            if valid:
                # A transient local rejection must not poison this order for
                # the rest of the same boot. A reboot has a different target ID.
                self.pending_mission = copy.deepcopy(message)
            self.emit(CoordinationEvent.MISSION_VOTE, mission=message.command_id,
                      payload={'accepted': valid, 'reason': reason})

    def event_cb(self, robot, message):
        if (message.session_id != self.session or message.robot_id != robot
                or robot not in self.peers or self.peers[robot][1].boot_id != message.boot_id
                or not self.events_replay.accept(robot, message.boot_id, message.sequence)):
            return
        try:
            payload = loads(message.payload_json, 65536)
        except ValueError:
            return
        with self.lock:
            if message.phase == CoordinationEvent.MISSION_RELAY:
                if message.task.command_id == message.mission_id:
                    self.mission(message.task)
                return
            if message.phase == message.MISSION_VOTE:
                if payload.get('accepted'):
                    self.mission_votes.setdefault(message.mission_id, {})[robot] = message.boot_id
                return
            if message.phase == message.REQUEST:
                if (message.mission_id != self.mission_id or self.estop
                        or message.owner_id != robot or message.owner_boot != message.boot_id
                        or robot not in self.live() or self.live()[robot][1].boot_id != message.boot_id):
                    return
                if message.resource.startswith('region/') and (
                        not self.all_aligned() or self.unrestricted
                        or payload.get('region_hash') != digest([self.seeds, self.regions])):
                    return
                if (not payload.get('bootstrap') and self.mapping
                        and payload.get('frame_epoch') != self.mapping[1].frame_epoch):
                    return
                if not payload.get('bootstrap') and not self.live()[robot][1].aligned:
                    return
                if robot == self.name:
                    self.grants.requested(message.request_id, message.mission_id, message.resource,
                        message.request_sequence, message.sent_steady_ns*1e-9, time.monotonic())
                if self.voter.grant(message.resource, robot, message.boot_id, message.request_id,
                                    message.mission_id, message.request_sequence, time.monotonic()):
                    self.emit(message.GRANT, resource=message.resource, owner=robot,
                        owner_boot=message.boot_id, request=message.request_id,
                        request_sequence=message.request_sequence)
                return
            if message.phase == message.GRANT and message.owner_id == self.name and message.owner_boot == self.boot:
                self.grants.granted(robot, message.boot_id, message.request_id, message.mission_id,
                                    message.resource, message.request_sequence)
            elif message.phase == message.RELEASE and message.owner_id == robot:
                self.voter.release(robot, message.owner_boot, message.request_id, message.stopped)
            elif message.phase == message.BID and message.mission_id == self.mission_id:
                if payload.get('operation') == 'frontier_quotes':
                    if (self.mapping and payload.get('frame_epoch') == self.mapping[1].frame_epoch
                            and self.boots().get(robot) == message.boot_id
                            and isinstance(payload.get('quotes'), dict) and len(payload['quotes']) <= 3):
                        for frontier_id, cost in payload['quotes'].items():
                            if isinstance(frontier_id, str) and len(frontier_id) <= 64:
                                self.frontier_quotes.setdefault(frontier_id, {})[robot] = dict(
                                    boot_id=message.boot_id, cost=cost, sent=payload.get('sent'))
                else:
                    self.peer_bids[robot] = time.monotonic(), payload
            elif message.phase == message.PLAN_VOTE and message.mission_id == self.mission_id:
                op = payload.get('operation')
                if op == 'release_regions':
                    if self.mode not in ('EXPLORING', 'EXPLORED'):
                        return
                    # The voters already checked exhaustion before emitting.
                    # Accept their majority certificate even after an earlier
                    # adopter resets its region flags for the new global scan.
                    self.release_votes[robot] = message.boot_id
                    if (sum(self.boots().get(n) == boot for n, boot in self.release_votes.items()) >= 2
                            and not self.unrestricted):
                        self.release_regions()
                elif op == 'formation_proposal':
                    plan = payload.get('plan')
                    if ((plan is None or isinstance(plan, dict)) and self.mode == 'FORMING'
                            and self.boots().get(robot) == message.boot_id):
                        self.formation_plans.setdefault(robot, plan)
                elif op == 'formation_vote':
                    plan_id = payload.get('plan_id')
                    if isinstance(plan_id, str) and isinstance(payload.get('accepted'), bool):
                        self.formation_votes.setdefault(plan_id, {})[robot] = (message.boot_id, payload['accepted'])

    def commit(self, message):
        if self.last_command == message.command_id:
            return
        self.cancel()
        self.last_command, self.mission_id = message.command_id, message.command_id
        self.active_command = copy.deepcopy(message)
        self.pending_mission = None
        self.explore_done, self.empty_since, self.bid, self.peer_bids = False, None, None, {}
        self.formation_plan, self.formation_plans, self.formation_votes = None, {}, {}
        self.formation_validations, self.formation_emit_at = {}, 0.
        self.release_votes = {}
        self.quote_future, self.local_quotes, self.frontier_quotes = None, {}, {}
        self.formation_step, self.completed_step, self.formation_stable = 0, -1, None
        self.formation_arrival = None
        self.formation_heading_corrections = 0
        self.formation_settled = False
        self.record('command_committed', command=message.kind, command_id=message.command_id)
        if message.kind == 'clear-estop':
            self.estop, self.mode = False, 'IDLE'
            self.message = '急停已解除，等待新任务'
            self.journal.update(estop=False)
        elif message.kind in ('pause', 'estop'):
            self.mode = 'ESTOP' if self.estop else 'IDLE'
            self.message = '急停已锁定' if self.estop else '已暂停，等待新任务'
        elif message.kind == 'explore':
            self.mode = 'EXPLORING' if self.team_initialized and self.aligned() else 'ALIGNING'
            self.message = '三车对等探索' if self.mode == 'EXPLORING' else '未知起点：受控局部探索与配准'
        elif message.kind == 'formation':
            self.mode, self.formation_command = 'FORMING', loads(message.argument)
            self.message = '等待三车确认同一队形计划'
        elif message.kind == 'regions':
            self.regions, self.unrestricted, self.mode = loads(message.argument)['regions'], False, 'IDLE'
            self.journal.update(regions=self.regions)
            self.message = '分区已更新，等待新任务'
        elif message.kind == 'patrol':
            self.patrol = loads(message.argument)
            self.patrol_index, self.patrol_cycle, self.mode = 0, 0, 'PATROLLING'
            self.patrol_dwell_until = 0.
            self.message = '执行航点巡逻'

    def _cancel_goal_handle(self, handle):
        try:
            handle.cancel_goal_async()
        except Exception as error:
            # An unconfirmed old goal may still be publishing anonymous Twist
            # commands. Do not issue another motion authorization in this boot.
            self.cancel_dispatch_failed = True
            self.nav_ready = False
            self.mode, self.message = 'FAULT', 'Nav2 取消请求未发出，保持停车'
            self.record('goal_action_error', stage='cancel', error=str(error))

    def cancel(self):
        self.goal_generation += 1
        handle = self.goal_handle
        self.goal_handle, self.goal_sending, self.work = None, False, None
        self.bid = None
        if self.request_id:
            self.release_pending = (self.request_id, self.resource, self.mission_id)
        self.request_id, self.resource = '', ''
        self.plan_points = []
        if handle:
            self._cancel_goal_handle(handle)

    def stopped(self, now):
        valid = (self.odom_data and now-self.odom_data[0] < .8 and valid_odom(self.odom_data[1])
                 and abs(self.odom_data[1].twist.twist.linear.x) < .015
                 and abs(self.odom_data[1].twist.twist.angular.z) < .04)
        self.stopped_since = (self.stopped_since or now) if valid else None
        return self.stopped_since is not None and now-self.stopped_since > .6

    def finish_work(self, success):
        work = self.work
        if work:
            if success:
                self.visits.append(work['target'])
                if work['kind'] == 'formation':
                    self.formation_arrival = (time.monotonic(), self.pose_sim_time)
                    if not work.get('heading_correction'):
                        self.completed_step = self.formation_step
                elif work['kind'] == 'patrol':
                    self.patrol_index += 1
                    self.patrol_dwell_until = time.monotonic()+self.patrol.get('dwell', 0)
            else:
                self.blacklist.append((*work['target'], time.monotonic()+45))
                if work['kind'] in ('formation', 'patrol'):
                    self.mode, self.message = 'FAULT', '本车导航失败，保持停车'
            self.record('goal_result', success=success, kind=work['kind'], target=work['target'])
            if work.get('heading_correction'):
                self.record('formation_heading_correction_result', success=success,
                            attempt=self.formation_heading_corrections)
        self.cancel()

    def send_goal(self):
        if (self.cancel_dispatch_failed or self.goal_sending or self.goal_handle
                or not self.work or not self.action.server_is_ready()):
            return
        generation, mission = self.goal_generation, self.mission_id
        work = self.work
        if work.get('heading_correction'):
            target = self.formation_heading_goal(self.formation_plan)
            if target is None:
                self.cancel()
                return
            work['target'] = target
        target = work['target']
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id, goal.pose.header.stamp = self.frame, self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(target[0]), float(target[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(target[2]/2), math.cos(target[2]/2)
        self.goal_sending = True
        self.goal_pending_since = time.monotonic()
        try:
            future = self.action.send_goal_async(goal)
        except Exception as error:
            self.record('goal_action_error', stage='send', error=str(error))
            self.finish_work(False)
            return

        def accepted(f):
            with self.lock:
                try:
                    handle = f.result()
                except Exception:
                    handle = None
                if generation != self.goal_generation or mission != self.mission_id or self.work is not work:
                    if handle and handle.accepted:
                        self._cancel_goal_handle(handle)
                    return
                self.goal_sending = False
                if not handle or not handle.accepted:
                    self.finish_work(False)
                    return
                self.goal_handle = handle
                work['navigation_started'] = time.monotonic()
                try:
                    result_future = handle.get_result_async()
                except Exception as error:
                    self.record('goal_action_error', stage='result_request', error=str(error))
                    self.finish_work(False)
                else:
                    result_future.add_done_callback(completed)

        def completed(f):
            with self.lock:
                if generation == self.goal_generation and mission == self.mission_id and self.work is work:
                    try:
                        result = f.result()
                    except Exception as error:
                        self.record('goal_action_error', stage='result', error=str(error))
                        result = None
                    if result is not None and result.status in (4, 5, 6):
                        # The action server reports a terminal GoalStatus.
                        self.goal_handle = None
                    self.finish_work(result is not None and result.status == 4)

        self.record('goal_sent', kind=work['kind'], target=target)
        future.add_done_callback(accepted)

    def candidate(self):
        with self.lock:
            grid, pose, mode = self.grid, self.pose, self.mode
            raw_grid = self.raw_grid
            aligned, full = self.aligned(), self.all_aligned()
            limits = self.limits()
            seeds, regions, unrestricted = dict(self.seeds), self.regions, self.unrestricted
            travel, blacklist, visits = self.travel, list(self.blacklist), list(self.visits)
            if aligned and full and not unrestricted:
                travel = self.regional_travel
            # A stationary 360-degree scan at an attained viewpoint has already
            # observed its visible surface. Do not cycle forever over raster
            # cracks just because the historical demo used truth to stop it.
            blacklist.extend(visits)
            team_from_local = self.team_transform()
            if aligned and unrestricted:
                frame = self.mapping[1].frame_epoch
                for _, peer, details in self.live().values():
                    if peer.robot_id != self.name and peer.frame_epoch == frame:
                        blacklist.extend(compose(inverse(team_from_local), p)
                                         for p in details.get('observed_viewpoints', []) if finite_pose(p))
                        if finite_pose(details.get('active_frontier_target')):
                            blacklist.append(compose(inverse(team_from_local), details['active_frontier_target']))
        if grid is None or pose is None or mode not in ('ALIGNING', 'EXPLORING'):
            return None
        local_from_team = inverse(team_from_local)
        if aligned and full and not unrestricted and len(seeds) == 3:
            yy, xx = np.indices(grid.data.shape)
            points = np.column_stack([grid.world_x.ravel(), grid.world_y.ravel()])
            team = transform_points(team_from_local, points)
            if regions:
                b = regions[MEMBERS.index(self.name)]
                mask = ((team[:, 0] >= b[0]+grid.region_clearance) & (team[:, 0] <= b[2]-grid.region_clearance)
                        & (team[:, 1] >= b[1]+grid.region_clearance) & (team[:, 1] <= b[3]-grid.region_clearance)).reshape(grid.data.shape)
            else:
                local_seeds = {n: compose(local_from_team, p) for n, p in seeds.items()}
                mask = voronoi_mask(grid.data.shape, grid.res, grid.origin, local_seeds, self.name, grid.region_clearance)
            margin = region_entry_margin(self.cfg['robot_radius'], self.cfg['region_margin'], grid.res)
            if not inside_region(self.name, compose(team_from_local, pose),
                                 dict(region_enabled=True, seeds=seeds, regions=regions), margin):
                target = region_return(grid, pose, mask)
                if target:
                    return dict(kind='return', target=list(target), score=100., bootstrap=False, resource='team_motion')
                proposal = grid.frontiers(pose, None, blacklist=blacklist, visits=visits)
                if proposal:
                    point = proposal[1]
                    return dict(kind='return', target=[point[0], point[1], math.atan2(point[1]-pose[1], point[0]-pose[0])],
                                score=100., bootstrap=False, resource='team_motion')
                return dict(empty=True, regional=False)
            regional_data = grid.data.copy()
            regional_data[~mask] = 100
            regional = Grid(regional_data, grid.res, grid.origin, self.cfg['robot_radius'], self.cfg['region_margin'])
            proposal = regional.frontiers(pose, None, blacklist=blacklist, visits=visits,
                                           min_travel=travel < 3.2, observation_grid=grid)
            if raw_grid and (proposal is None or proposal[3]['visible_unknown_m2'] < .10):
                info = raw_grid.info
                local = private_observation_grid(grid,
                    np.asarray(raw_grid.data, np.int16).reshape(info.height, info.width), info.resolution,
                    (info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation)))
                verification = regional.frontiers(pose, None, blacklist=blacklist, visits=visits,
                                                   min_travel=travel < 3.2, observation_grid=local)
                if verification is not None:
                    proposal = verification
            resource = 'region/'+self.name
        else:
            proposal = grid.frontiers(pose, None, blacklist=blacklist, visits=visits, min_travel=travel < 3.2)
            resource = 'team_motion'
        if proposal is None or (travel >= 3.2 and proposal[3]['visible_unknown_m2'] < .10):
            return dict(empty=True, regional=aligned and full and not unrestricted)
        p = proposal[1]
        bootstrap = not self.team_initialized or not aligned
        if bootstrap:
            path = proposal[2][1]
            previous, distance = pose[:2], 0.
            for point in path:
                distance += math.dist(previous, point)
                previous = point
                p = point
                if distance >= 2.:
                    break
        target = [p[0], p[1], math.atan2(p[1]-pose[1], p[0]-pose[0])]
        result = dict(kind='explore', target=target, resource=resource, bootstrap=bootstrap,
                    score=float(proposal[0])+(5. if travel < 3.2 else 0.)+(100. if not aligned else 0.),
                    cost=proposal[2][0], visible_unknown_m2=proposal[3]['visible_unknown_m2'])
        if aligned and unrestricted:
            team_target = list(compose(team_from_local, target))
            result.update(team_target=team_target, frame_epoch=self.mapping[1].frame_epoch,
                frontier_id=digest([self.boot, [round(v, 3) for v in team_target]]), offered_at=time.monotonic())
        return result

    def frontier_tick(self, now):
        # Locally completed members are still available auction participants.
        # Dropping their quotes while frontier_winner requires every ready
        # member strands the last explorer. Paused/stopped missions stay out.
        if not (self.mode in ('EXPLORING', 'EXPLORED') and self.unrestricted
                and self.local_ready and self.nav_ready and self.aligned()):
            return
        frame = self.mapping[1].frame_epoch
        participants = {n: p.boot_id for n, (_, p, _) in self.live().items()
                        if p.aligned and p.local_ready and p.nav_ready and p.frame_epoch == frame}
        if len(participants) < 2:
            return
        claimed = {d.get('active_frontier_id') for _, p, d in self.live().values()
                   if p.mission_id == self.mission_id and d.get('active_frontier_id')}
        if self.bid and (self.bid.get('frontier_id') in claimed
                         or now-self.bid.get('offered_at', now) > 8.):
            self.bid = None
            self.peer_bids.pop(self.name, None)
            self.last_plan = 0.
        offers = {n: d for n, (received, d) in self.peer_bids.items() if now-received < 2
                  and n in participants and d.get('frame_epoch') == frame and valid_offer(d)
                  and d['frontier_id'] not in claimed}
        if self.quote_future and self.quote_future.done():
            try:
                mission, quote_frame, result = self.quote_future.result()
                if mission == self.mission_id and quote_frame == frame:
                    self.local_quotes = dict(operation='frontier_quotes', frame_epoch=frame,
                                             sent=now, quotes=result)
            except Exception as error:
                self.message = '前沿路径报价暂不可用：'+str(error)
            self.quote_future = None
        if not self.quote_future and offers and self.grid and self.pose and now-self.last_quote > 1.5:
            mission, grid, pose, transform = self.mission_id, self.grid, self.pose, self.team_transform()
            peers = [p for n, p in self.team_poses().items() if n != self.name]
            proposals = copy.deepcopy(offers)
            self.quote_future = self.pool.submit(lambda: (mission, frame,
                quote_frontiers(grid, pose, transform, proposals, peers)))
            self.last_quote = now
        if self.local_quotes and now-self.last_quote_emit > .5:
            self.emit(CoordinationEvent.BID, payload=self.local_quotes)
            self.last_quote_emit = now
        winner = frontier_winner(offers, self.frontier_quotes, participants, now)
        if winner and winner[0] == self.name and not self.work and not self.release_pending:
            offer = winner[1]
            resumed = self.mode == 'EXPLORED'
            self.work = dict(offer, target=list(compose(inverse(self.team_transform()), offer['team_target'])), started=now)
            self.request_id, self.request_sequence, self.resource = uuid.uuid4().hex, 0, 'team_motion'
            self.mode = 'EXPLORING'
            self.explore_done, self.empty_since = False, None
            self.message = '按可行路径代价认领跨区补盲前沿'
            self.record('frontier_claim', frontier_id=offer['frontier_id'], team_target=offer['team_target'],
                        feasible_quotes=self.frontier_quotes[offer['frontier_id']],
                        resumed_from_completed=resumed)
        for frontier_id, values in list(self.frontier_quotes.items()):
            if all(not isinstance(v.get('sent'), (int, float)) or now-v['sent'] > 10 for v in values.values()):
                del self.frontier_quotes[frontier_id]

    def make_formation(self):
        poses = self.team_poses()
        if len(poses) != 3 or self.grid is None:
            return None
        args = self.formation_command
        slots = formation(args['shape'], args['x'], args['y'], args['yaw'], args['spacing'])
        local = self.grid
        fused, origin = dynamic_fusion([(local.data, local.res, *local.origin, 0., self.team_transform())], local.res)
        # Leave room for grid resampling and the independently estimated local
        # maps. A path that only just fits one raster often fails another
        # member's unchanged validation; propose with extra clearance instead.
        grid = Grid(fused, local.res, origin, self.cfg['robot_radius']+.12, self.cfg['region_margin'])
        result = plan_formation(grid, [poses[n] for n in MEMBERS], slots, peer_clearance=1.0)
        if result is None:
            return None
        _, assignment, routes = result
        return dict(slots=[list(p) for p in slots], assignment=list(assignment),
                    routes=[[i, list(target), [list(p) for p in path]] for i, target, path in routes])

    def save(self, command):
        try:
            with self.lock:
                if self.grid is None:
                    raise ValueError('本车地图尚不可用')
                grid = self.grid
                aligned, transform = self.aligned(), self.team_transform()
                metadata = copy.deepcopy(self.mapping[2]) if self.mapping else {}
                frame_epoch = self.mapping[1].frame_epoch if self.mapping else ''
                if self.mapping:
                    metadata.update(anchor_id=self.mapping[1].anchor_id, anchor_boot=self.mapping[1].anchor_boot,
                                    map_epoch=self.epoch, revision=self.mapping[1].revision)
                participants = sorted(self.live())
                aligned_participants = sorted(n for n, p in self.live().items()
                    if p[1].aligned and p[1].frame_epoch == frame_epoch)
                raw = self.raw_grid
            dest = Path(command.argument).expanduser().resolve() if command.argument else Path(self.cfg['run_dir'])/'results/final'
            dest.mkdir(parents=True, exist_ok=True)
            if raw is None:
                raise ValueError('本车原始局部地图尚不可用')
            save_map(dest/(self.name+'_local_map'), np.asarray(raw.data).reshape(raw.info.height, raw.info.width),
                     raw.info.resolution, (raw.info.origin.position.x, raw.info.origin.position.y))
            if aligned:
                data, origin = dynamic_fusion([(grid.data, grid.res, *grid.origin, 0., transform)], grid.res)
                save_map(dest/('replica_'+self.name), data, grid.res, origin)
                if aligned_participants and self.name == min(aligned_participants):
                    save_map(dest/'shared_map', data, grid.res, origin)
                    (dest/'frame.json').write_text(dumps(dict(session_id=self.session,
                        frame_epoch=frame_epoch, source_robot=self.name, team_from_source=transform,
                        sim_time=self.get_clock().now().nanoseconds*1e-9, save_command_id=command.command_id,
                        partial=len(metadata.get('transforms', {})) != 3, **metadata)))
            request = WriteState.Request()
            request.filename, request.include_unfinished_submaps = str(dest/(self.name+'.pbstream')), True
            if not self.write_state.service_is_ready():
                raise RuntimeError('本车 PBStream 服务尚未就绪')
            future = self.write_state.call_async(request)
            end = time.monotonic()+15
            while rclpy.ok() and not future.done() and time.monotonic() < end:
                time.sleep(.02)
            if not future.done() or future.result() is None or future.result().status.code != 0:
                raise RuntimeError('本车 PBStream 保存失败或超时')
            self.emit(CoordinationEvent.PLAN_VOTE, mission=command.command_id,
                      payload=dict(operation='save', success=True, aligned=aligned,
                                   shared_writer=bool(aligned and aligned_participants
                                                      and self.name == min(aligned_participants)),
                                   frame_epoch=frame_epoch,
                                   partial=len(participants) != 3, destination=str(dest)))
        except Exception as error:
            self.emit(CoordinationEvent.PLAN_VOTE, mission=command.command_id,
                      payload=dict(operation='save', success=False, reason=str(error)))

    def tick(self):
        with self.lock:
            now = time.monotonic()
            self.pose_sim_time, self.pose_steady_time = None, None
            try:
                tf = self.buffer.lookup_transform(self.frame, self.name+'/base_link', rclpy.time.Time())
                p, q = tf.transform.translation, tf.transform.rotation
                self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                if not stamp_fresh(tf.header.stamp, self.get_clock().now().nanoseconds):
                    self.pose = None
                if self.pose:
                    self.pose_sim_time = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                    self.pose_steady_time = now+self.pose_sim_time-self.get_clock().now().nanoseconds*1e-9
            except TransformException:
                self.pose = None
            if self.pose and self.home is None:
                self.home = self.pose
            self.local_ready = bool(self.pose and self.grid and self.scan and now-self.scan[0] < .8
                                    and scan_health(self.scan[1])[0] and self.odom_data
                                    and now-self.odom_data[0] < .8 and valid_odom(self.odom_data[1]))
            for name, (sent, future) in list(self.nav_futures.items()):
                if not future.done() and now-sent >= .3:
                    self.nav_states[name] = False
                    self.nav_futures.pop(name, None)
                    future.cancel()
            if now-self.last_nav_check >= .25:
                for name, client in self.nav_clients.items():
                    old = self.nav_futures.get(name)
                    if not client.service_is_ready():
                        self.nav_states[name] = False
                    if client.service_is_ready() and (not old or old[1].done()):
                        future = client.call_async(GetState.Request())
                        self.nav_futures[name] = now, future
                        def received(f, n=name):
                            with self.lock:
                                if self.nav_futures.get(n, (None, None))[1] is not f:
                                    return
                                try:
                                    self.nav_states[n] = f.result() is not None and f.result().current_state.id == 3
                                except Exception:
                                    self.nav_states[n] = False
                        future.add_done_callback(received)
                self.last_nav_check = now
            self.nav_ready = (not self.cancel_dispatch_failed and len(self.nav_states) == 4
                              and all(self.nav_states.values()))
            self.verify_alignment(now)
            if self.mode == 'BOOTSTRAP' and self.local_ready and self.nav_ready:
                self.mode = 'IDLE'
                self.message = '本车就绪，等待任务；地图将自主对齐'
            if self.aligned() and self.pose and self.initial_team is None:
                saved = self.journal.data
                if saved.get('frame_epoch') == self.mapping[1].frame_epoch:
                    self.seeds = dict(saved.get('seeds', {}))
                    self.regions = saved.get('regions')
                self.initial_team = self.seeds.get(self.name) or compose(self.team_transform(), self.home or self.pose)
            for n, (_, peer, details) in self.live().items():
                if self.mapping and peer.aligned and details.get('initial_team_pose') and peer.frame_epoch == self.mapping[1].frame_epoch:
                    self.seeds.setdefault(n, details['initial_team_pose'])
            if self.initial_team:
                self.seeds[self.name] = list(self.initial_team)
            if (len(self.seeds) == 3 and self.mapping
                    and (self.journal.data.get('seeds') != self.seeds
                         or self.journal.data.get('frame_epoch') != self.mapping[1].frame_epoch)):
                self.journal.update(seeds=self.seeds, frame_epoch=self.mapping[1].frame_epoch)
            if self.all_aligned() and len(self.seeds) == 3:
                self.team_ready_since = self.team_ready_since or now
                if not self.team_initialized and now-self.team_ready_since >= 1.:
                    self.journal.update(team_initialized=True)
                    self.team_initialized = True
            else:
                self.team_ready_since = None
            alignment_wait = bool(self.map_aligned() and
                (not self.alignment_verified or not self.team_initialized
                 or self.verified_at is None or now-self.verified_at < 1.))
            if alignment_wait:
                self.cancel()
                self.bid = None
                self.message = '地图已配准，静止验证定位及共同坐标后开始分区探索'
            if (self.active_command and now-self.last_command_relay > .5
                    and (self.active_command.revision, self.active_command.issuer_id,
                         self.active_command.command_id) == self.last_command_order):
                self.emit(CoordinationEvent.MISSION_RELAY, task=self.active_command)
                self.last_command_relay = now
            if self.pending_mission and not command_is_current(self.pending_mission, now, self.peers):
                self.pending_mission = None
            if self.pending_mission:
                votes = self.mission_votes.get(self.pending_mission.command_id, {})
                needed = 3 if self.pending_mission.kind in ('formation', 'regions') else 2
                fresh_voters = {n for n, boot in votes.items() if self.boots().get(n) == boot}
                owners = set(loads(self.pending_mission.argument)['routes']) if self.pending_mission.kind == 'patrol' else set()
                if len(fresh_voters) >= needed and owners <= fresh_voters:
                    self.commit(self.pending_mission)
            stopped = self.stopped(now)
            if self.release_pending and stopped:
                request, resource, mission = self.release_pending
                self.emit(CoordinationEvent.RELEASE, mission=mission, request=request, resource=resource, stopped=True)
                self.release_pending = None
            if self.mode == 'ALIGNING' and self.team_initialized and self.aligned():
                self.cancel()
                self.mode = 'EXPLORING'
                self.record('alignment_ready', frame_epoch=self.mapping[1].frame_epoch)
            if self.work and (not self.local_ready or not self.nav_ready):
                self.cancel()
                self.message = '本车传感器或导航中断，保持停车'
            if self.work and self.resource.startswith('region/') and not self.all_aligned():
                self.cancel()
                self.bid = None
                self.last_plan = 0.
            if self.planner_future and self.planner_future.done():
                try:
                    mission, mode, result = self.planner_future.result()
                    if (mode == 'EXPLORING' and self.unrestricted and result and
                            (result.get('regional') or result.get('resource', '').startswith('region/'))):
                        result = None  # A pre-release worker cannot restore the old phase.
                    if mission == self.mission_id and mode == self.mode and not alignment_wait:
                        if mode in ('ALIGNING', 'EXPLORING'):
                            self.bid = result
                            if result and result.get('empty'):
                                self.explore_done = True
                                self.message = ('观测不足或存在配准歧义，保持停车等待新的共同观测' if mode == 'ALIGNING' else
                                                '本区没有可行的新观测视点，等待其他分区完成后跨区补盲'
                                                if result.get('regional') else '可达前沿已处理，等待同伴确认完成')
                                if result.get('regional') and self.regions_exhausted():
                                    self.emit(CoordinationEvent.PLAN_VOTE, payload={'operation': 'release_regions'})
                                else:
                                    self.empty_since = self.empty_since or now
                            else:
                                self.explore_done, self.empty_since = False, None
                                self.message = '分区并行探索' if result and result.get('resource', '').startswith('region/') else '补充观测或跨区通行，等待对等授权'
                        elif mode == 'FORMING':
                            self.formation_plans[self.name] = result
                            self.emit(CoordinationEvent.PLAN_VOTE, payload={'operation': 'formation_proposal', 'plan': result})
                except Exception as error:
                    self.message = '本车规划暂不可用：'+str(error)
                self.planner_future = None
            if (self.local_ready and self.nav_ready and not self.work and not self.planner_future
                    and not alignment_wait and not self.release_pending and now-self.last_plan > 2):
                mission, mode = self.mission_id, self.mode
                function = self.candidate if mode in ('ALIGNING', 'EXPLORING') and not (self.bid and self.bid.get('frontier_id')) else self.make_formation if mode == 'FORMING' and self.name not in self.formation_plans else None
                if function:
                    self.planner_future = self.pool.submit(lambda: (mission, mode, function()))
                    self.last_plan = now
            if not alignment_wait and self.mode in ('ALIGNING', 'EXPLORING') and self.bid and not self.bid.get('empty'):
                self.emit(CoordinationEvent.BID, payload=self.bid)
                peers = [(b[1].get('score', -1e9), n) for n, b in self.peer_bids.items()
                         if now-b[0] < 1 and not b[1].get('empty') and b[1].get('resource') == 'team_motion']
                owner = min([(-score, n) for score, n in peers] or [(0, self.name)])[1]
                if (self.work and self.work['kind'] == 'explore' and self.aligned() and not self.all_aligned()
                        and any(n != self.name and not self.live().get(n, (None, None, None))[1].aligned
                                for _, n in peers if n in self.live())):
                    self.cancel()
                    self.last_plan = 0.
                if self.work and self.resource.startswith('region/') and self.homing_pending():
                    self.cancel()
                if (not self.work and not self.release_pending
                        and not self.bid.get('frontier_id')
                        and not (self.bid['resource'].startswith('region/') and self.homing_pending()) and
                        (self.bid['resource'].startswith('region/') or owner == self.name)):
                    self.work = dict(self.bid, started=now)
                    self.request_id, self.request_sequence = uuid.uuid4().hex, 0
                    self.resource = self.work['resource']
            if self.mode == 'EXPLORING' and not self.unrestricted and self.regions_exhausted():
                self.emit(CoordinationEvent.PLAN_VOTE, payload={'operation': 'release_regions'})
            self.frontier_tick(now)
            if self.mode == 'EXPLORING' and self.exploration_finished(now):
                self.mode = 'EXPLORED'
                self.message = '本车可达探索任务已完成'
                self.record('exploration_complete')
            if self.mode == 'FORMING' and not alignment_wait:
                self.formation_tick(now)
            if self.mode == 'PATROLLING' and not alignment_wait:
                self.patrol_tick(now)
            timeout = self.patrol.get('goal_timeout', 120) if self.work and self.work['kind'] == 'patrol' and self.patrol else 160
            if self.work and ((self.goal_sending and now-self.goal_pending_since > 5)
                    or (self.work.get('navigation_started') and now-self.work['navigation_started'] > timeout)
                    or (self.work.get('heading_correction')
                        and now-self.work['started'] > FORMATION_HEADING_CORRECTION_TIMEOUT)):
                self.finish_work(False)
            if self.work and not self.estop and self.local_ready and self.nav_ready and self.motion_turn(now):
                self.request_sequence += 1
                sent = time.monotonic_ns()
                self.grants.requested(self.request_id, self.mission_id, self.resource,
                                       self.request_sequence, sent*1e-9, time.monotonic())
                self.emit(CoordinationEvent.REQUEST, resource=self.resource, request=self.request_id,
                    request_sequence=self.request_sequence, sent=sent,
                    payload=dict(frame_epoch=self.mapping[1].frame_epoch,
                                 region_hash=digest([self.seeds, self.regions]), bootstrap=self.work.get('bootstrap', False)))
                if self.grants.valid(self.request_id, self.mission_id, self.resource, now, self.boots()):
                    self.send_goal()
            self.publish_authorization()
            self.publish_state(now)

    def formation_tick(self, now):
        if (not self.all_aligned() or any(not peer.local_ready or not peer.nav_ready
                                         for _, peer, _ in self.live().values())):
            self.formation_stable = None
            self.formation_settled = False
            self.cancel()
            self.message = '三车队形需要全部成员在线、定位有效且导航可用'
            return
        if now >= self.formation_emit_at:
            self.formation_emit_at = now+1.
            if self.name in self.formation_plans:
                self.emit(CoordinationEvent.PLAN_VOTE, payload=dict(operation='formation_proposal',
                          plan=self.formation_plans[self.name]))
            for plan_id, accepted in self.formation_validations.items():
                self.emit(CoordinationEvent.PLAN_VOTE, payload=dict(operation='formation_vote',
                          plan_id=plan_id, accepted=accepted))
        if not self.formation_plan:
            for plan in self.formation_plans.values():
                if plan is not None:
                    plan_id = digest(plan)
                    if plan_id not in self.formation_validations:
                        accepted = validate_formation_plan(plan, self.formation_command, self.team_poses(),
                                                           self.grid, self.team_transform())
                        self.formation_validations[plan_id] = accepted
                        self.record('formation_validation', plan_id=plan_id, accepted=accepted)
            agreement = agreed_formation_plan(self.formation_plans, self.formation_votes, self.boots())
            if agreement:
                plan_id, plan = agreement
                self.formation_plan = copy.deepcopy(plan)
                self.formation_plan['plan_id'] = plan_id
                self.formation_plan['frame_epoch'] = self.mapping[1].frame_epoch
                self.record('formation_plan', plan_id=plan_id, assignment=plan['assignment'], slots=plan['slots'])
            else:
                self.message = '等待三车对同一静态队形计划完成可行性确认'
                if len(self.formation_plans) == 3:
                    boots = self.boots()
                    plans = [p for p in self.formation_plans.values() if p is not None]
                    decided = all(all(n in self.formation_votes.get(digest(p), {})
                        and self.formation_votes[digest(p)][n][0] == boots.get(n)
                        for n in MEMBERS) for p in plans)
                    if decided:
                        self.mode = 'FAULT'
                        self.message = '三车地图没有共同可行的队形路径，请补充观测或重新选择集合点'
                        self.record('formation_unavailable', reason=self.message)
        if not self.formation_plan:
            return
        plan = self.formation_plan
        if plan['frame_epoch'] != self.mapping[1].frame_epoch:
            self.formation_stable = None
            self.formation_settled = False
            self.cancel()
            self.mode, self.message = 'FAULT', '队形计划的共同坐标已变化，保持停车'
            return
        if any(name != self.name and (peer.mission_id != self.mission_id
                    or details.get('formation_plan_id') != plan['plan_id'])
               for name, (_, peer, details) in self.live().items()):
            self.formation_stable = None
            self.formation_settled = False
            self.message = '等待全部成员确认同一队形计划后开始通行'
            return
        routes = plan['routes']
        while self.formation_step < len(routes):
            owner = MEMBERS[routes[self.formation_step][0]]
            if owner == self.name:
                done = self.completed_step >= self.formation_step
            else:
                peer = self.live().get(owner)
                done = bool(peer and peer[2].get('formation_plan_id') == plan['plan_id']
                            and peer[2].get('completed_step', -1) >= self.formation_step)
            if not done:
                break
            self.formation_step += 1
        if self.formation_step < len(routes) and not self.work and not self.release_pending:
            owner_index, target, _ = routes[self.formation_step]
            if MEMBERS[owner_index] == self.name:
                self.work = dict(kind='formation', target=list(compose(inverse(self.team_transform()), target)),
                                 bootstrap=False, resource='team_motion', started=now)
                self.request_id, self.request_sequence, self.resource = uuid.uuid4().hex, 0, 'team_motion'
        if self.formation_step == len(routes):
            self.formation_finish_tick(plan, now)

    def formation_heading_goal(self, plan):
        if (not plan or self.estop or not self.all_aligned() or self.grid is None
                or plan['frame_epoch'] != self.mapping[1].frame_epoch):
            return None
        for name, (_, peer, details) in self.live().items():
            if name == self.name:
                continue
            if (peer.mission_id != self.mission_id or peer.estop or not peer.nav_ready
                    or peer.mode not in ('FORMING', 'FORMATION_READY')
                    or details.get('formation_plan_id') != plan['plan_id']
                    or details.get('formation_step') != len(plan['routes'])
                    or details.get('motion_lease_held') or not peer.odom_valid
                    or abs(peer.velocity.linear.x) >= .015 or abs(peer.velocity.angular.z) >= .04):
                return None
        poses = self.team_poses()
        if set(poses) != set(MEMBERS):
            return None
        slot = plan['slots'][plan['assignment'][MEMBERS.index(self.name)]]
        pose = poses[self.name]
        if (math.dist(pose[:2], slot[:2]) > FORMATION_POSITION_TOLERANCE
                or abs(angle(pose[2]-slot[2])) <= FORMATION_YAW_TOLERANCE):
            return None
        local_from_team = inverse(self.team_transform())
        goal = compose(local_from_team, slot)
        others = [compose(local_from_team, pose) for name, pose in poses.items() if name != self.name]
        allowed = self.grid.allowed(others=others, other_clearance=.85)
        # The small move back to the voted slot must remain safe in the live map.
        cell = self.grid.cell(self.pose)
        if not self.grid.inside(cell) or not allowed[cell] or not self.grid.astar(self.pose, goal, allowed):
            return None
        return list(goal)

    def formation_finish_tick(self, plan, now):
        self.formation_settled = False
        arrival = self.formation_arrival
        poses = self.team_poses()
        if (self.estop or self.work or self.release_pending or not arrival or not self.stopped(now)
                or self.pose_steady_time is None or self.pose_steady_time <= arrival[0]
                or self.pose_sim_time is None or (arrival[1] is not None and self.pose_sim_time <= arrival[1])
                or now-arrival[0] < FORMATION_LOCALIZATION_SETTLE_SECONDS or set(poses) != set(MEMBERS)):
            self.formation_stable = None
            return
        slot = plan['slots'][plan['assignment'][MEMBERS.index(self.name)]]
        pose = poses[self.name]
        position_error = math.dist(pose[:2], slot[:2])
        yaw_error = abs(angle(pose[2]-slot[2]))
        self.formation_settled = (position_error <= FORMATION_POSITION_TOLERANCE
                                  and yaw_error <= FORMATION_YAW_TOLERANCE)
        if position_error <= FORMATION_POSITION_TOLERANCE and yaw_error > FORMATION_YAW_TOLERANCE:
            self.message = '导航已结束，航向仍超出槽位容差，检查纠偏路径'
            goal = self.formation_heading_goal(plan)
            if goal is not None:
                if self.formation_heading_corrections >= FORMATION_HEADING_CORRECTION_LIMIT:
                    self.mode, self.message = 'FAULT', '队形航向纠偏次数已用完，保持停车'
                    self.record('formation_heading_correction_exhausted', yaw_error_deg=math.degrees(yaw_error))
                else:
                    self.formation_heading_corrections += 1
                    self.work = dict(kind='formation', target=goal, heading_correction=True,
                                     bootstrap=False, resource='team_motion', started=now)
                    self.request_id, self.request_sequence, self.resource = uuid.uuid4().hex, 0, 'team_motion'
                    self.message = '等待队形航向纠偏的对等授权'
                    self.record('formation_heading_correction', attempt=self.formation_heading_corrections,
                                yaw_error_deg=math.degrees(yaw_error), position_error_m=position_error, slot=slot)
        peers_settled = all(name == self.name or (peer.mission_id == self.mission_id
                    and details.get('formation_plan_id') == plan['plan_id']
                    and details.get('formation_settled') is True)
                    for name, (_, peer, details) in self.live().items())
        good = self.formation_settled and peers_settled and formation_pass(
            [poses[n] for n in MEMBERS], plan['slots'], plan['assignment'], self.cfg['robot_radius'])
        self.formation_stable = (self.formation_stable or now) if good else None
        if self.formation_stable and now-self.formation_stable >= 3:
            self.mode = 'FORMATION_READY'
            self.message = '静态队形已稳定到位'
            self.record('formation_ready', slots=plan['slots'], assignment=plan['assignment'])

    def patrol_tick(self, now):
        route = self.patrol.get('routes', {}).get(self.name, []) if self.patrol else []
        if not route:
            self.mode = 'PATROL_COMPLETE'
            self.message = '本车没有待执行巡逻航点'
            return
        if self.patrol_index >= len(route):
            self.patrol_cycle, self.patrol_index = self.patrol_cycle+1, 0
        if self.patrol_cycle >= self.patrol.get('cycles', 1):
            self.mode = 'PATROL_COMPLETE'
            self.message = '本车巡逻路线已完成'
            return
        if not self.work and not self.release_pending and now >= self.patrol_dwell_until:
            self.work = dict(kind='patrol', target=list(compose(inverse(self.team_transform()), route[self.patrol_index])),
                             bootstrap=False, resource='team_motion', started=now)
            self.request_id, self.request_sequence, self.resource = uuid.uuid4().hex, 0, 'team_motion'

    def publish_authorization(self):
        self.auth_sequence += 1
        msg = LocalAuthorization()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id, msg.robot_id, msg.boot_id = self.session, self.name, self.boot
        msg.sequence, msg.mission_id, msg.request_id = self.auth_sequence, self.mission_id, self.request_id
        msg.resource, msg.frame_epoch = self.resource, self.mapping[1].frame_epoch if self.mapping else ''
        msg.enabled, msg.estop = bool(self.work and self.local_ready and self.nav_ready), self.estop
        msg.bootstrap = bool(self.work and self.work.get('bootstrap'))
        msg.maximum_speed = .08 if msg.bootstrap else self.cfg['max_speed']
        msg.limits_json = dumps(self.limits())
        self.authpub.publish(msg)

    def publish_state(self, now):
        self.sequence += 1
        msg = PeerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.session_id, msg.robot_id, msg.boot_id = self.session, self.name, self.boot
        msg.sequence, msg.map_epoch = self.sequence, self.epoch
        set_pose(msg.pose, self.pose or (0., 0., 0.))
        if self.odom_data:
            msg.velocity = self.odom_data[1].twist.twist
            p, q = self.odom_data[1].pose.pose.position, self.odom_data[1].pose.pose.orientation
            set_pose(msg.odom_pose, (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
            msg.odom_epoch = self.odom_epoch
            msg.odom_valid = bool(self.odom_epoch and now-self.odom_data[0] < .8
                and valid_odom(self.odom_data[1]) and stamp_fresh(self.odom_data[1].header.stamp,
                                                               self.get_clock().now().nanoseconds))
        msg.local_ready, msg.nav_ready, msg.aligned, msg.estop = self.local_ready, self.nav_ready, self.aligned(), self.estop
        msg.frame_epoch = self.mapping[1].frame_epoch if self.mapping else ''
        msg.mission_id, msg.mode = self.mission_id, self.mode
        team_pose = compose(self.team_transform(), self.pose) if self.pose and self.aligned() else None
        if team_pose and (not self.trail or math.dist(team_pose[:2], self.trail[-1]) > .08):
            self.trail.append(team_pose[:2])
            self.trail = self.trail[-600:]
        goal = compose(self.team_transform(), self.work['target']) if self.work and self.aligned() else None
        msg.details_json = dumps(dict(team_pose=team_pose, home=self.home,
            pose_sim_time=self.pose_sim_time, pose_steady_time=self.pose_steady_time,
            regional_travel=self.regional_travel, alignment_verified=self.alignment_verified,
            alignment=dict(verified=self.alignment_verified, ever_verified=self.alignment_ever_verified,
                map_aligned=self.map_aligned(),
                hold_seconds=now-self.alignment_hold[0] if self.alignment_hold else 0.,
                target_revision=self.alignment_hold[3] if self.alignment_hold else None,
                current_revision=self.mapping[1].revision if self.mapping else None,
                confirmed_revision=self.mapping[2].get('confirmed_local_revision', 0) if self.mapping else 0,
                registered_revision=self.mapping[2].get('registered_local_revision', 0) if self.mapping else 0),
            initial_team_pose=self.initial_team, travel=self.travel, distance=self.distance, trail=self.trail,
            goal=goal, goal_local=self.work['target'] if self.work else None,
            work_resource=self.resource, bootstrap_motion=bool(self.work and self.work.get('bootstrap')),
            connected_members=sorted(self.live()),
            active_frontier_id=self.work.get('frontier_id') if self.work else None,
            active_frontier_target=self.work.get('team_target') if self.work else None,
            work_kind=self.work['kind'] if self.work else '', work_score=self.work.get('score', 0.) if self.work else 0.,
            motion_lease_held=self.grants.valid(self.request_id, self.mission_id, self.resource, now, self.boots()) if self.work else False,
            observed_viewpoints=[compose(self.team_transform(), p) for p in self.visits[-200:]] if self.aligned() else [],
            plan=[list(p) for p in transform_points(self.team_transform(), self.plan_points)] if self.plan_points and self.aligned() else [],
            safety=self.safety, local_ready=self.local_ready, nav_ready=self.nav_ready,
            message=('与同伴失联，保持停车等待通信恢复' if len(self.live()) < 2
                     and self.mode in ('ALIGNING', 'EXPLORING', 'PATROLLING', 'FORMING') else self.message),
            explore_done=self.explore_done, unrestricted=self.unrestricted,
            seeds=self.seeds, regions=self.regions, region_hash=digest([self.seeds, self.regions]),
            formation_plan_id=self.formation_plan.get('plan_id') if self.formation_plan else None,
            formation_slots=self.formation_plan['slots'] if self.formation_plan else [],
            assignment=self.formation_plan['assignment'] if self.formation_plan else None,
            completed_step=self.completed_step, formation_step=self.formation_step,
            formation_settled=self.formation_settled,
            patrol_index=self.patrol_index, patrol_cycle=self.patrol_cycle,
            patrol_total=len(self.patrol.get('routes', {}).get(self.name, []))*self.patrol.get('cycles', 1) if self.patrol else 0,
            patrol_completed=(self.patrol_cycle*len(self.patrol.get('routes', {}).get(self.name, []))+self.patrol_index) if self.patrol else 0,
            events=self.events[-5:]))
        self.statepub.publish(msg)
        self.peer_state(self.name, msg)

    def close(self):
        with self.lock:
            if rclpy.ok():
                self.cancel()
                self.publish_authorization()
            self.closed = True
            self.goal_generation += 1
            self.work = None
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.log.close()


def main():
    from .ros_runtime import RosRuntime
    with RosRuntime() as runtime:
        node = PeerAgent()
        try:
            runtime.spin(node, MultiThreadedExecutor(num_threads=4))
        finally:
            node.close()
            node.destroy_node()
