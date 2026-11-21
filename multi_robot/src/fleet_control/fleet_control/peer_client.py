"""Direct client for peer missions. No dashboard or coordinator is required."""
import math
from pathlib import Path
import time
import uuid
import rclpy
from rclpy.parameter import Parameter
from fleet_interfaces.msg import PeerState, MappingState, MissionCommand, CoordinationEvent
from .peer_common import STATE_QOS, EVENT_QOS, dumps, loads, peer_topic, pose_list
from .peer_protocol import MEMBERS, ReplayWindow


class PeerClient:
    def __init__(self, run, node=None):
        self.run = Path(run).resolve()
        metadata = loads((self.run/'architecture.json').read_text())
        if metadata.get('architecture') != 'decentralized':
            raise ValueError('not a decentralized invocation')
        self.session = metadata['session_id']
        self.owned = node is None
        self.node = node or rclpy.create_node('peer_client_'+uuid.uuid4().hex[:10])
        if self.owned:
            self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.states, self.mappings, self.replies = {}, {}, {}
        self.state_replay, self.event_replay = ReplayWindow(), ReplayWindow()
        self.issuer = uuid.uuid4().hex
        self.publisher = self.node.create_publisher(MissionCommand, '/fleet/peer/mission', EVENT_QOS)
        for name in MEMBERS:
            self.node.create_subscription(PeerState, peer_topic(name, 'state'),
                lambda m, n=name: self.state_cb(n, m), STATE_QOS)
            self.node.create_subscription(MappingState, peer_topic(name, 'mapping'),
                lambda m, n=name: self.mapping_cb(n, m), STATE_QOS)
            self.node.create_subscription(CoordinationEvent, peer_topic(name, 'event'),
                lambda m, n=name: self.event_cb(n, m), EVENT_QOS)

    def state_cb(self, name, msg):
        if (msg.session_id == self.session and msg.robot_id == name
                and self.state_replay.accept(name, msg.boot_id, msg.sequence, msg.map_epoch)):
            try:
                details = loads(msg.details_json)
            except ValueError:
                return
            self.states[name] = time.monotonic(), msg, details

    def mapping_cb(self, name, msg):
        if msg.session_id == self.session and msg.robot_id == name:
            current = self.states.get(name)
            if current and (current[1].boot_id != msg.boot_id or current[1].map_epoch != msg.map_epoch):
                return
            try:
                details = loads(msg.details_json)
            except ValueError:
                return
            self.mappings[name] = time.monotonic(), msg, details

    def event_cb(self, name, msg):
        if (msg.session_id != self.session or msg.robot_id != name
                or name not in self.states or self.states[name][1].boot_id != msg.boot_id
                or not self.event_replay.accept(name, msg.boot_id, msg.sequence)):
            return
        try:
            data = loads(msg.payload_json)
        except ValueError:
            return
        if msg.phase == msg.MISSION_VOTE or data.get('operation') == 'save':
            self.replies.setdefault(msg.mission_id, {})[name] = (msg.boot_id, data)

    def spin(self, seconds=.05):
        if self.owned:
            rclpy.spin_once(self.node, timeout_sec=seconds)
        else:
            time.sleep(seconds)

    def state(self, timeout=1.):
        end = time.monotonic()+timeout
        while self.owned and time.monotonic() < end:
            self.spin(.05)
        now = time.monotonic()
        live = {n: s for n, s in self.states.items() if now-s[0] < 1.5}
        members = [s[1] for s in live.values()]
        full = (len(live) == 3 and all(s.local_ready and s.nav_ready and s.aligned for s in members)
                and len({s.frame_epoch for s in members}) == 1
                and all(len(row[2].get('connected_members', MEMBERS)) == 3 for row in live.values()))
        local = sum(s.local_ready and s.nav_ready for s in members) >= 2
        cooperative = (sum(s.local_ready and s.nav_ready and s.aligned for s in members) >= 2
                       and len({s.frame_epoch for s in members if s.aligned}) == 1)
        modes = [s.mode for s in members]
        if any(s.estop for s in members):
            mode = 'ESTOP'
        elif not local:
            mode = 'BOOTSTRAP'
        elif 'FAULT' in modes:
            mode = 'DEGRADED' if len(live) >= 2 else 'FAULT'
        elif 'FORMING' in modes:
            mode = 'FORMING'
        elif len(live) == 3 and all(m == 'FORMATION_READY' for m in modes):
            mode = 'FORMATION_READY'
        elif 'PATROLLING' in modes:
            mode = 'PATROLLING'
        elif 'EXPLORING' in modes:
            mode = 'EXPLORING'
        elif 'ALIGNING' in modes:
            mode = 'ALIGNING'
        elif modes and all(m == 'EXPLORED' for m in modes):
            mode = 'EXPLORED'
        elif modes and all(m == 'PATROL_COMPLETE' for m in modes):
            mode = 'PATROL_COMPLETE'
        else:
            mode = 'IDLE'
        selected = min(live) if live else None
        mapping = self.mappings.get(selected)
        info = mapping[2] if mapping else {}
        robot_data = {}
        patrol_rows = {}
        seeds, regions, slots, assignment = {}, [], [], None
        for name in MEMBERS:
            value = self.states.get(name)
            if not value:
                robot_data[name] = {'ready': False, 'pose': None, 'travel': 0., 'distance': 0.,
                                    'goal': None, 'trail': [], 'plan': [], 'safety': {'reason': '未收到本车状态'}}
                continue
            received, state, d = value
            current_mapping = self.mappings.get(name)
            mapping_details = (current_mapping[2] if current_mapping
                and current_mapping[1].boot_id == state.boot_id
                and current_mapping[1].map_epoch == state.map_epoch else {})
            robot_data[name] = dict(ready=name in live and state.local_ready and state.nav_ready,
                mission_id=state.mission_id, alignment=d.get('alignment', {}),
                mapping_diagnostics={key: mapping_details.get(key) for key in
                    ('confirmed_local_revision', 'registered_local_revision', 'map_ages', 'geometry', 'worker',
                     'registration', 'continuity_rejections')},
                pose=d.get('team_pose'), travel=d.get('travel', 0.), distance=d.get('distance', 0.),
                regional_travel=d.get('regional_travel', 0.),
                pose_sim_time=d.get('pose_sim_time'), pose_steady_time=d.get('pose_steady_time'),
                goal=d.get('goal'), trail=d.get('trail', []), plan=d.get('plan', []),
                safety=d.get('safety', {}), mode=state.mode, state_age=now-received,
                message=d.get('message', ''), aligned=state.aligned, nav_ready=state.nav_ready,
                frame_epoch=state.frame_epoch,
                work_resource=d.get('work_resource', ''),
                connected_members=d.get('connected_members', []),
                boot_id=state.boot_id, map_epoch=state.map_epoch, local_pose=pose_list(state.pose))
            patrol_rows[name] = dict(completed=d.get('patrol_completed', 0), total=d.get('patrol_total', 0))
            seeds.update(d.get('seeds', {}))
            if d.get('regions'):
                regions = d['regions']
            if d.get('formation_slots'):
                slots, assignment = d['formation_slots'], d.get('assignment')
        errors = {}
        if slots and assignment:
            from .geometry import angle
            for i, name in enumerate(MEMBERS):
                p = robot_data[name].get('pose')
                if p:
                    target = slots[assignment[i]]
                    errors[name] = {'position': math.dist(p[:2], target[:2]), 'yaw': abs(angle(p[2]-target[2]))}
        unrestricted = any(v[2].get('unrestricted') for v in live.values())
        degraded = len(live) < 3 or any(len(row[2].get('connected_members', MEMBERS)) < 3 for row in live.values())
        frame = mapping[1].frame_epoch if mapping else ''
        return dict(architecture='decentralized', initialization='unknown', run_dir=str(self.run),
            sim_time=self.node.get_clock().now().nanoseconds*1e-9, wall_time=time.time(), steady_time=time.monotonic(),
            session_id=self.session, ready=bool(full), local_ready=bool(local), team_ready=bool(full),
            cooperative_ready=bool(cooperative),
            capabilities=dict(explore=bool(local), patrol=bool(cooperative), formation=bool(full), regions=bool(full)),
            mode=mode, message=('降级运行：仅在已确认安全的区域继续，三车队形暂不可用' if degraded and cooperative else
                '待对齐：补充共同观测，有歧义时保持等待' if mode == 'ALIGNING' else
                '三车对等自治' if full else '本车就绪，可开始局部探索以补充对齐观测' if local else '等待本车传感器、SLAM 与 Nav2'),
            coordinate_frame=frame,
            degraded=degraded,
            online_members=sorted(live), robots=robot_data,
            known_fraction=info.get('known_fraction', 0.), direct_observed_fraction=info.get('observed_fraction', 0.),
            observed_fraction=info.get('observed_fraction', 0.),
            inferred_obstacle_fraction=info.get('inferred_obstacle_fraction', 0.),
            boundary_obstacle_fraction=info.get('boundary_obstacle_fraction', 0.),
            regions=regions, starts=seeds, region_seeds=seeds,
            region_enabled=not unrestricted,
            regions_locked=any(s.mode not in ('IDLE', 'EXPLORED', 'FORMATION_READY', 'PATROL_COMPLETE') for s in members),
            exploration_unrestricted=unrestricted, region_release_reason='对等协商跨区补盲' if unrestricted else '',
            formation_slots=slots, formation_errors=errors,
            formation_assignment=assignment,
            patrol=dict(status='running' if mode == 'PATROLLING' else 'completed' if mode == 'PATROL_COMPLETE' else 'cancelled',
                        robots=patrol_rows) if any(p['total'] for p in patrol_rows.values()) else None,
            mapping={'aligned': [n for n, s in live.items() if s[1].aligned],
                     'anchor_id': mapping[1].anchor_id if mapping else '',
                     'anchor_boot': mapping[1].anchor_boot if mapping else '', **info},
            estop=any(s.estop for s in members), saved=None)

    def request(self, kind, data, timeout=15.):
        stopping = getattr(self.node, '_fleet_stopping', lambda: False)
        if kind == 'command':
            operation, argument = str(data['command']), str(data.get('argument', ''))
        elif kind in ('formation', 'regions'):
            operation, argument = kind, dumps(data)
        else:
            raise ValueError('unknown peer command')
        required_members = set()
        if operation == 'patrol':
            routes = loads(argument).get('routes')
            if isinstance(routes, dict):
                required_members = set(routes) & set(MEMBERS)
        msg = MissionCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.session_id, msg.issuer_id, msg.command_id = self.session, self.issuer, uuid.uuid4().hex
        msg.revision, msg.kind, msg.argument = time.monotonic_ns(), operation, argument
        membership_deadline = time.monotonic()+min(2., timeout/4)
        while (operation not in ('pause', 'estop') and time.monotonic() < membership_deadline
               and not stopping()):
            self.spin(.05)
            if sum(time.monotonic()-r[0] < 1.5 for r in self.states.values()) == 3:
                break
        captured = time.monotonic()
        msg.participant_boot_ids = [self.states[n][1].boot_id
            if n in self.states and captured-self.states[n][0] < 1.5 else '' for n in MEMBERS]
        end, published = time.monotonic()+timeout, 0.
        msg.expires_at = end
        while time.monotonic() < end and not stopping():
            now = time.monotonic()
            if now-published > .2:
                self.publisher.publish(msg)
                published = now
            self.spin(.02)
            current = {n: s for n, s in self.states.items() if now-s[0] < 1.5}
            replies = {n: value[1] for n, value in self.replies.get(msg.command_id, {}).items()
                       if n in current and current[n][1].boot_id == value[0]}
            need = 3 if operation in ('formation', 'regions') else 2
            if operation == 'save':
                need = max(2, len(current))
            field = 'success' if operation == 'save' else 'accepted'
            good = [n for n, r in replies.items() if r.get(field) is True]
            shared_saved = (operation != 'save' or not any(s[1].aligned for s in current.values())
                            or any(replies[n].get('shared_writer') for n in good))
            committed = (good if operation == 'save' else
                         [n for n in good if current[n][1].mission_id == msg.command_id])
            if len(committed) >= need and shared_saved and required_members <= set(committed):
                return {'success': True, 'message': '对等成员已保存结果' if operation == 'save' else '任务已获对等确认',
                        'command_id': msg.command_id, 'members': committed,
                        'partial': len(committed) < 3, 'results': replies}
            rejected = [r.get('reason', '成员拒绝') for r in replies.values() if r.get(field) is False]
            owner_rejections = [replies[n].get('reason', n+' 拒绝执行') for n in required_members
                                if n in replies and replies[n].get(field) is False]
            if owner_rejections:
                return {'success': False, 'message': '；'.join(owner_rejections), 'results': replies}
            if rejected and len(current)-len(rejected) < need:
                return {'success': False, 'message': '；'.join(rejected), 'results': replies}
        return {'success': False, 'message': '未取得足够的新鲜成员确认，任务未获准',
                'command_id': msg.command_id, 'results': self.replies.get(msg.command_id, {})}

    def close(self):
        if self.owned:
            self.node.destroy_node()
