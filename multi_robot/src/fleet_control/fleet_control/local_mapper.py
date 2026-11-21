"""One robot's scan filter, unknown-origin registration and map replica.

Only this robot's TF is subscribed. All remote observations arrive as raw,
source-qualified snapshots. Registration runs in a separate spawned process.
"""
import copy
from collections import OrderedDict
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException
from fleet_interfaces.msg import (MapSnapshot, RelativeConstraint, MapContinuity,
                                  MappingState, PeerState, LocalAuthorization)
from fleet_mapping.grid import valid_grid_message, mask_robot_returns
from fleet_mapping.observed_completion import ObservedCompletion
from fleet_mapping.peer_echoes import filter_peer_echoes
from fleet_mapping.continuity import retained_geometry_job
from fleet_mapping.registration import (ConstraintGraph, angle, assess, compose,
                                        dynamic_fusion, inverse, registration_job, transform_points, voronoi_mask, anchor_key,
                                        MIN_MATCH_SCORE, MAX_MATCH_RMS, initialize_registration_worker)
from .geometry import peer_obstacle_grid, navigation_peer_radius
from .ros_runtime import RosRuntime
from .peer_protocol import MEMBERS, ReplayWindow, digest, lost_peer_radius
from .peer_transport import PeerInbox
from .peer_continuity import constraint_consistent_with_odometry, component_bodies_consistent
from .peer_geometry_certificates import GeometryCertificates
from .peer_common import (STATE_QOS, CONSTRAINT_QOS, dumps, loads, load_config,
                          pose_list, set_pose, grid_message, snapshot_from_message, peer_topic, local_frame)


class LocalMapper(Node):
    def __init__(self):
        super().__init__('local_mapper')
        self.cfg = load_config(self)
        self.name, self.boot = self.cfg['robot_id'], self.cfg['boot_id']
        self.session, self.epoch = self.cfg['session_id'], self.cfg['map_epoch']
        self.frame = local_frame(self.name, self.epoch)
        self.inbox = PeerInbox(self, self.cfg)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.maps, self.received, self.peers, self.mapper_states = {}, {}, {}, {}
        self.last_positions = {}
        self.peer_history = {}
        self.authorization = None
        self.auth_sequence = 0
        self.graph = ConstraintGraph()
        self.odom_priors, self.last_prior_save, self.continuity_rejections = {}, 0., 0
        self.frozen_anchor = None
        self.constraints, self.candidates = {}, {}
        self.confirmed_local_revision = 0
        self.registered_local_revision = 0
        self.admitted = False
        self.local_history = OrderedDict()
        self.geometry = GeometryCertificates(self.session, self.name, self.boot, self.epoch)
        self.last_geometry_submit = 0.
        self.completion = ObservedCompletion()
        self.replay = ReplayWindow()
        self.sequence, self.revision, self.last_payload = 0, 0, None
        self.last_submit, self.last_relay, self.pair_index = 0., 0., 0
        process_context = get_context('spawn')
        self.registration_stop = process_context.Event()
        self.pool = ProcessPoolExecutor(max_workers=1, mp_context=process_context,
            initializer=initialize_registration_worker, initargs=(self.registration_stop,))
        self.future, self.future_pair = None, None
        self.future_kind = 'registration'
        self.future_started, self.last_worker = 0., {}
        self.last_registration = {}
        self.last_geometry_checks = []
        self.graph_file = Path(self.cfg['run_dir'])/self.name/'mapping_graph.json'
        self.graph_file.parent.mkdir(parents=True, exist_ok=True)
        if self.graph_file.exists():
            saved = loads(self.graph_file.read_text())
            if saved.get('session_id') == self.session:
                self.odom_priors = saved.get('odom_priors', {})
                for edge in saved.get('edges', []):
                    self.graph.add(edge['source'], edge['target'], edge['transform'], edge['score'],
                                   edge.get('sigma', .05), edge.get('yaw_sigma', math.radians(1)))
                if saved.get('anchor'):
                    self.frozen_anchor = tuple(saved['anchor'])
                    self.graph.anchor = self.frozen_anchor
        self.rawpub = self.create_publisher(MapSnapshot, peer_topic(self.name, 'map'), STATE_QOS)
        self.constraintpub = self.create_publisher(RelativeConstraint, peer_topic(self.name, 'constraint'), CONSTRAINT_QOS)
        self.continuitypub = self.create_publisher(MapContinuity, peer_topic(self.name, 'continuity'), CONSTRAINT_QOS)
        self.statepub = self.create_publisher(MappingState, peer_topic(self.name, 'mapping'), STATE_QOS)
        self.sharedpub = self.create_publisher(OccupancyGrid, f'/{self.name}/shared_map', STATE_QOS)
        self.navpub = self.create_publisher(OccupancyGrid, f'/{self.name}/navigation_map', STATE_QOS)
        self.scanpub = self.create_publisher(LaserScan, f'/{self.name}/scan_slam', qos_profile_sensor_data)
        self.create_subscription(LaserScan, f'/{self.name}/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, f'/{self.name}/local_map', self.local_map, STATE_QOS)
        self.create_subscription(LocalAuthorization, f'/{self.name}/authorization', self.authorization_cb, 10)
        for robot in MEMBERS:
            self.create_subscription(MapSnapshot, peer_topic(robot, 'map'),
                                     lambda m, n=robot: self.inbox.receive(n, 'map', self.remote_map, n, m), STATE_QOS)
            self.create_subscription(RelativeConstraint, peer_topic(robot, 'constraint'),
                                     lambda m, n=robot: self.inbox.receive(n, 'constraint', self.constraint, n, m), CONSTRAINT_QOS)
            self.create_subscription(MapContinuity, peer_topic(robot, 'continuity'),
                                     lambda m, n=robot: self.inbox.receive(n, 'continuity', self.continuity, n, m), CONSTRAINT_QOS)
            self.create_subscription(MappingState, peer_topic(robot, 'mapping'),
                                     lambda m, n=robot: self.inbox.receive(n, 'mapping', self.mapping_state, n, m), STATE_QOS)
            self.create_subscription(PeerState, peer_topic(robot, 'state'),
                                     lambda m, n=robot: self.inbox.receive(n, 'state', self.peer_state, n, m), STATE_QOS)
        self.create_timer(.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def own_key(self):
        return (self.name, self.boot, self.epoch)

    def authorization_cb(self, message):
        if (message.session_id == self.session and message.boot_id == self.boot
                and message.sequence > self.auth_sequence):
            self.auth_sequence = message.sequence
            self.authorization = time.monotonic(), message

    def local_map(self, message):
        if not valid_grid_message(message) or message.header.frame_id != self.frame:
            return
        p, q = message.info.origin.position, message.info.origin.orientation
        payload = (message.info.width, message.info.height, message.info.resolution,
                   p.x, p.y, q.z, q.w, bytes(np.asarray(message.data, dtype=np.int8)))
        if payload != self.last_payload:
            self.last_payload = payload
            self.revision += 1
        snap = MapSnapshot()
        snap.header = message.header
        snap.session_id, snap.robot_id, snap.boot_id = self.session, self.name, self.boot
        snap.map_epoch, snap.revision, snap.grid = self.epoch, self.revision, message
        self.rawpub.publish(snap)
        self.remote_map(self.name, snap)

    def remote_map(self, robot, message):
        if (message.session_id != self.session or message.robot_id != robot
                or not message.boot_id or message.map_epoch < 1 or message.revision < 1
                or not valid_grid_message(message.grid)
                or abs(message.grid.info.resolution-self.cfg['resolution']) > 1e-6
                or message.grid.header.frame_id != local_frame(robot, message.map_epoch)):
            return
        old = self.maps.get(robot)
        if old and old.boot == message.boot_id and message.revision < old.revision:
            return
        peer = self.peers.get(robot)
        if peer and peer[1].boot_id != message.boot_id:
            return
        try:
            snapshot = snapshot_from_message(message)
        except ValueError:
            return
        if old and old.boot == snapshot.boot and snapshot.epoch < old.epoch:
            return
        self.maps[robot] = snapshot
        self.received[robot] = time.monotonic()
        if robot == self.name and (snapshot.boot, snapshot.epoch) == (self.boot, self.epoch):
            self.local_history[snapshot.revision] = snapshot
            while len(self.local_history) > 32:
                self.local_history.popitem(last=False)

    def geometry_members(self, now):
        return {n: (p.boot_id, p.map_epoch) for n, (received, p) in self.peers.items()
                if now-received < .8 and p.local_ready and n in self.maps
                and (self.maps[n].boot, self.maps[n].epoch) == (p.boot_id, p.map_epoch)
                and now-self.received.get(n, 0.) < 3.}

    def observation_samples(self):
        return [(n, boot, epoch, pose, received, peer.local_ready)
                for n, (received, peer) in self.peers.items()
                for boot, epoch, pose in self.peer_history.get(n, [])
                if boot == peer.boot_id and epoch == peer.map_epoch]

    def continuity(self, sender, message):
        now = time.monotonic()
        members = self.geometry_members(now)
        self.geometry.receive(sender, message, members,
                              {n: s.revision for n, s in self.maps.items()}, now)
        self.confirmed_local_revision = max(self.confirmed_local_revision,
                                            self.geometry.confirmed_revision(members, now))

    def peer_state(self, robot, message):
        if message.session_id != self.session or message.robot_id != robot:
            return
        try:
            pose_list(message.pose)
            pose_list(message.odom_pose)
        except ValueError:
            return
        if self.replay.accept(robot, message.boot_id, message.sequence, message.map_epoch):
            self.peers[robot] = (time.monotonic(), message)
            if message.local_ready:
                history = self.peer_history.setdefault(robot, [])
                pose = pose_list(message.pose)
                if not history or history[-1][:2] != (message.boot_id, message.map_epoch) or math.dist(history[-1][2][:2], pose[:2]) > .15:
                    history.append((message.boot_id, message.map_epoch, pose))
                    del history[:-600]

    def mapping_state(self, robot, message):
        if (message.session_id == self.session and message.robot_id == robot
                and robot in self.peers and self.peers[robot][1].boot_id == message.boot_id
                and self.peers[robot][1].map_epoch == message.map_epoch):
            self.mapper_states[robot] = (time.monotonic(), message)

    def constraint(self, sender, message):
        if (message.session_id != self.session or message.robot_id != sender
                or sender not in self.peers or self.peers[sender][1].boot_id != message.boot_id
                or message.source_robot not in MEMBERS or message.target_robot not in MEMBERS
                or message.source_robot == message.target_robot
                or not message.constraint_id or not MIN_MATCH_SCORE <= message.score <= 1.0
                or not .2 <= message.overlap <= 1.0
                or not .025 <= message.translation_sigma <= MAX_MATCH_RMS
                or not 0 < message.rotation_sigma <= math.radians(3)):
            return
        try:
            pose = pose_list(message.transform)
        except ValueError:
            return
        endpoints = {message.source_robot, message.target_robot}
        source = (message.source_robot, message.source_boot, message.source_epoch)
        target = (message.target_robot, message.target_boot, message.target_epoch)
        now = time.monotonic()
        historical = {key for edge in self.graph.edges for key in edge} | {self.own_key()}
        for key in (source, target):
            current = self.peers.get(key[0])
            if key not in historical and (not current or now-current[0] >= .8
                    or (current[1].boot_id, current[1].map_epoch) != key[1:] or not current[1].local_ready):
                return
        peers = {n: dict(boot_id=p.boot_id, map_epoch=p.map_epoch, pose=pose_list(p.pose),
                        odom_pose=pose_list(p.odom_pose), odom_epoch=p.odom_epoch,
                        odom_valid=p.odom_valid and time.monotonic()-received < .8,
                        local_ready=p.local_ready, received=received) for n, (received, p) in self.peers.items()}
        if not component_bodies_consistent(self.graph, source, target, pose, peers, now,
                                           self.cfg['robot_radius']):
            self.continuity_rejections += 1
            return
        if not constraint_consistent_with_odometry(source, target, pose,
                self.graph.poses(self.frozen_anchor) if self.frozen_anchor else {}, peers,
                self.odom_priors, time.monotonic(), self.cfg['max_speed'], self.cfg['robot_radius']):
            self.continuity_rejections += 1
            return
        if not set(message.witnesses) <= endpoints:
            return
        witnesses = set(message.witnesses)
        if self.name in endpoints:
            a, b = self.maps.get(message.source_robot), self.maps.get(message.target_robot)
            if (a and b and a.boot == message.source_boot and b.boot == message.target_boot
                    and (self.boot == (a.boot if a.robot == self.name else b.boot))):
                candidate = self.candidates.get((a.robot, a.boot, a.epoch, b.robot, b.boot, b.epoch))
                if (candidate and candidate[1] >= 3 and now-candidate[2] >= 2
                        and now-candidate[3] < 10
                        and math.dist(candidate[0][:2], pose[:2]) < .08
                        and abs(angle(candidate[0][2]-pose[2])) < math.radians(2)):
                    witnesses.add(self.name)
        previous = self.constraints.get(message.constraint_id)
        if previous:
            witnesses |= set(previous.witnesses)
        message = copy.deepcopy(message)
        message.witnesses = sorted(witnesses)
        family = (message.source_robot, message.source_boot, message.target_robot, message.target_boot)
        if witnesses == endpoints:
            for old_id, old in list(self.constraints.items()):
                if (old.source_robot, old.source_boot, old.target_robot, old.target_boot) == family and len(old.witnesses) == 2:
                    if (old.source_revision, old.target_revision) > (message.source_revision, message.target_revision):
                        return
                    if old_id != message.constraint_id:
                        del self.constraints[old_id]
        if len(self.constraints) >= 24 and message.constraint_id not in self.constraints:
            pending = next((k for k, m in self.constraints.items() if len(m.witnesses) < 2), None)
            if pending is None:
                pending = next((k for k, m in self.constraints.items()
                    if any(n in self.peers and self.peers[n][1].boot_id != boot
                           for n, boot in ((m.source_robot, m.source_boot), (m.target_robot, m.target_boot)))), None)
            if pending:
                del self.constraints[pending]
            else:
                return
        self.constraints[message.constraint_id] = message
        if witnesses == endpoints:
            a = (message.source_robot, message.source_boot, message.source_epoch)
            b = (message.target_robot, message.target_boot, message.target_epoch)
            if self.graph.add(a, b, pose, message.score, message.translation_sigma, message.rotation_sigma):
                revision = (message.source_revision if a == self.own_key()
                            else message.target_revision if b == self.own_key() else 0)
                self.confirmed_local_revision = max(self.confirmed_local_revision, revision)
                self.registered_local_revision = max(self.registered_local_revision, revision)
                self.geometry.remember(message, self.local_history.get(revision))
                self.persist()
        if self.name in witnesses and (not previous or set(previous.witnesses) != witnesses):
            message.robot_id, message.boot_id = self.name, self.boot
            self.constraintpub.publish(message)

    def persist(self):
        value = {'session_id': self.session, 'anchor': self.frozen_anchor, 'odom_priors': self.odom_priors,
                 'edges': [dict(source=a, target=b, transform=p, score=s,
                                sigma=self.graph.sigmas.get((a, b), (.05, math.radians(1)))[0],
                                yaw_sigma=self.graph.sigmas.get((a, b), (.05, math.radians(1)))[1])
                           for (a, b), (p, s) in self.graph.edges.items()]}
        temp = self.graph_file.with_suffix('.tmp')
        temp.write_text(dumps(value))
        temp.replace(self.graph_file)

    def confirm_pending(self, a, b):
        """Use a newly completed independent match to revisit a peer proposal.

        Receiving the proposal and finishing the local worker need not happen
        in that order. Re-run the existing confirmation checks; do not refresh
        or extend the lifetime of any candidate or manufacture a witness.
        """
        key = (a.robot, a.boot, a.epoch, b.robot, b.boot, b.epoch)
        confirmed = 0
        for message in reversed(list(self.constraints.values())):
            if (len(message.witnesses) != 1 or self.name in message.witnesses
                    or (message.source_robot, message.source_boot, message.source_epoch,
                        message.target_robot, message.target_boot, message.target_epoch) != key):
                continue
            if a.revision < message.source_revision or b.revision < message.target_revision:
                continue
            revision = message.source_revision if a.robot == self.name else message.target_revision
            if revision not in self.local_history:
                continue
            self.constraint(message.robot_id, message)
            result = self.constraints.get(message.constraint_id)
            if result and len(result.witnesses) == 2:
                confirmed += 1
        return confirmed

    def propose_match(self, a, b, match, diagnostics=None):
        key = (a.robot, a.boot, a.epoch, b.robot, b.boot, b.epoch)
        now = time.monotonic()
        self.last_registration = dict(completed_steady=now,
            source=dict(robot=a.robot, boot_id=a.boot, epoch=a.epoch, revision=a.revision),
            target=dict(robot=b.robot, boot_id=b.boot, epoch=b.epoch, revision=b.revision),
            match=None if match is None else dict(transform=list(match.transform),
                score=match.score, overlap=match.overlap, rms=match.rms))
        if diagnostics is not None:
            self.last_registration['assessment'] = diagnostics
        if any(not (current := self.maps.get(s.robot)) or
               (current.boot, current.epoch) != (s.boot, s.epoch) for s in (a, b)):
            self.last_registration['outcome'] = 'retired_identity'
            return
        if match is None:
            self.candidates.pop(key, None)
            self.last_registration['outcome'] = 'no_accepted_match'
            return
        previous = self.candidates.get(key)
        if (previous and math.dist(previous[0][:2], match.transform[:2]) < .08
                and abs(angle(previous[0][2]-match.transform[2])) < math.radians(2)):
            count, since = previous[1]+1, previous[2]
        else:
            count, since = 1, now
        self.candidates[key] = (match.transform, count, since, now)
        self.last_registration.update(stable_count=count, stable_seconds=now-since)
        if count < 3 or now-since < 2:
            self.last_registration['outcome'] = 'awaiting_stability'
            return
        self.last_registration.update(outcome='stable_match', pending_endorsed=self.confirm_pending(a, b))
        msg = RelativeConstraint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id, msg.robot_id, msg.boot_id = self.session, self.name, self.boot
        msg.source_robot, msg.source_boot, msg.source_epoch = a.robot, a.boot, a.epoch
        msg.target_robot, msg.target_boot, msg.target_epoch = b.robot, b.boot, b.epoch
        msg.source_revision, msg.target_revision = a.revision, b.revision
        set_pose(msg.transform, match.transform)
        msg.score, msg.overlap = match.score, match.overlap
        msg.translation_sigma, msg.rotation_sigma = match.sigma, match.yaw_sigma
        msg.constraint_id = digest([a.robot, a.boot, b.robot, b.boot,
                                    [round(v, 3) for v in match.transform]])
        msg.witnesses = [self.name]
        if not self.admitted:
            # Preserve exact worker inputs, not later raw-map revisions. These
            # are observed occupancy grids; the mapper never reads truth.
            from .peer_recording import atomic_npz
            folder = self.graph_file.parent/'registration-checkpoints'
            folder.mkdir(exist_ok=True)
            tag = f'e{self.epoch}-{msg.constraint_id}-{a.revision}-{b.revision}'
            for role, snap in (('source', a), ('target', b)):
                path = folder/f'{tag}-{role}.npz'
                if not path.exists():
                    atomic_npz(path, robot=snap.robot, boot=snap.boot, epoch=snap.epoch,
                        revision=snap.revision, data=snap.data, resolution=snap.resolution, origin=snap.origin)
            path = folder/f'{tag}.json'
            if not path.exists():
                path.write_text(dumps(dict(self.last_registration, constraint_id=msg.constraint_id)))
        self.constraint(self.name, msg)
        self.constraintpub.publish(msg)

    def peer_positions(self, now, root=None):
        poses = self.graph.poses(root or self.own_key())
        result = {}
        for robot, (received, peer) in self.peers.items():
            key = (robot, peer.boot_id, peer.map_epoch)
            if key in poses and peer.local_ready:
                position = compose(poses[key], pose_list(peer.pose))
                self.last_positions[robot] = position, received
                result[robot] = position, now-received
            elif robot in self.last_positions:
                position, observed = self.last_positions[robot]
                result[robot] = position, now-observed
        return result

    def scan(self, message):
        output = copy.deepcopy(message)
        positions = self.peer_positions(time.monotonic())
        graph = self.graph.poses(self.own_key())
        others = [p for n, (p, age) in positions.items() if n != self.name and age < .8
                  and self.peers[n][1].local_ready
                  and (n, self.peers[n][1].boot_id, self.peers[n][1].map_epoch) in graph]
        if others:
            try:
                tf = self.buffer.lookup_transform(self.frame, message.header.frame_id,
                                                 rclpy.time.Time.from_msg(message.header.stamp))
                p, q = tf.transform.translation, tf.transform.rotation
                sensor = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                output.ranges = mask_robot_returns(message.ranges, message.angle_min,
                    message.angle_increment, sensor, others, self.cfg['robot_radius']+.17).tolist()
            except TransformException:
                pass  # Unknown peers remain real obstacles; safety uses the raw scan.
        self.scanpub.publish(output)

    def tick(self):
        now = time.monotonic()
        if self.future and self.future.done():
            self.last_worker = dict(kind=self.future_kind, seconds=now-self.future_started)
            try:
                if self.future_kind == 'continuity':
                    result = self.future.result()
                    self.last_geometry_checks = result['checks']
                    self.geometry.completed(result['proofs'])
                else:
                    result = self.future.result()
                    self.propose_match(*self.future_pair, result['match'], result['diagnostics'])
            except Exception as error:
                self.last_worker['error'] = str(error)
                self.get_logger().warning('配准任务失败，继续等待观测：'+str(error))
            self.future, self.future_pair = None, None
        pending = {n for n, (received, peer) in self.peers.items()
                   if now-received < .8 and not peer.aligned}
        references = self.geometry.active_references(self.geometry_members(now), pending)
        if (not self.future and references and self.name in self.maps
                and self.last_worker.get('kind') != 'continuity'
                and now-self.last_geometry_submit >= 2.):
            self.future_kind = 'continuity'
            self.future = self.pool.submit(retained_geometry_job, self.maps[self.name], references,
                copy.deepcopy(self.graph), self.observation_samples(), now,
                self.cfg['robot_radius']+.17, dict(self.maps), True)
            self.future_started = now
            self.last_geometry_submit = now
        elif not self.future and now-self.last_submit >= 1.0 and self.name in self.maps:
            others = [n for n in MEMBERS if n != self.name and n in self.maps
                      and now-self.received.get(n, 0) < 15]
            if others:
                other = others[self.pair_index % len(others)]
                self.pair_index += 1
                a, b = sorted([self.maps[self.name], self.maps[other]], key=lambda s: s.robot)
                self.future_pair = (a, b)
                self.future_kind = 'registration'
                self.future = self.pool.submit(registration_job, a, b)
                self.future_started = now
                self.last_submit = now
        for message in self.geometry.outgoing(now):
            message.header.stamp = self.get_clock().now().to_msg()
            self.continuity(self.name, message)
            self.continuitypub.publish(message)
        if now-self.last_relay > 2:
            relay_messages = list(self.constraints.values())+list(self.geometry.relayed_bases())
            for msg in relay_messages:
                if len(msg.witnesses) == 2:
                    relay = copy.deepcopy(msg)
                    relay.robot_id, relay.boot_id = self.name, self.boot
                    self.constraintpub.publish(relay)
            self.last_relay = now
        own_poses = self.graph.poses(self.own_key())
        candidate = min(own_poses) if len(own_poses) >= 2 else self.own_key()
        # Previously certified frame identities survive a robot process restart.
        certified = {}
        for robot, (received, state) in self.mapper_states.items():
            if now-received < 1.5 and state.aligned:
                root = anchor_key(own_poses, state.anchor_id, state.anchor_boot)
                if root:
                    certified.setdefault(root, set()).add(robot)
        for root, witnesses in certified.items():
            if len(witnesses) >= 2 and root in own_poses:
                if self.frozen_anchor != root:
                    self.frozen_anchor = root
                    self.persist()
        root = self.frozen_anchor or candidate
        epoch = digest([self.session, root])
        votes = {self.name}
        for robot, (received, state) in self.mapper_states.items():
            if now-received < 1.5 and state.frame_epoch == epoch:
                votes.add(robot)
        if not self.frozen_anchor and len(own_poses) >= 2 and len(votes) >= 2:
            self.frozen_anchor = root
            self.persist()
        self.graph.anchor = self.frozen_anchor or candidate
        team_poses = self.graph.poses()
        uncertainties = self.graph.uncertainties()
        if self.frozen_anchor:
            for n, (received, peer) in self.peers.items():
                key = (n, peer.boot_id, peer.map_epoch)
                if (now-received < .8 and peer.aligned and peer.frame_epoch == epoch
                        and peer.local_ready and peer.odom_valid and peer.odom_epoch and key in team_poses):
                    team_pose = compose(team_poses[key], pose_list(peer.pose))
                    self.odom_priors[n] = dict(boot_id=peer.boot_id, odom_epoch=peer.odom_epoch,
                        team_from_odom=compose(team_pose, inverse(pose_list(peer.odom_pose))),
                        last_team_pose=team_pose, last_seen=received, sigma=uncertainties[key][0])
            if now-self.last_prior_save > 2:
                self.persist()
                self.last_prior_save = now
        local_from_team = inverse(team_poses[self.own_key()]) if self.own_key() in team_poses else None
        layers = []
        transforms = {}
        samples = self.observation_samples()
        for n, snap in self.maps.items():
            key = (n, snap.boot, snap.epoch)
            if key in own_poses:
                derived = filter_peer_echoes(snap, self.graph, samples, now,
                                             self.cfg['robot_radius']+.17, self.maps)
                layers.append((derived, snap.resolution, *snap.origin, own_poses[key]))
            if key in team_poses:
                transforms[n] = {'boot_id': snap.boot, 'map_epoch': snap.epoch,
                                 'transform': team_poses[key], 'sigma': max(.025, uncertainties[key][0]),
                                 'yaw_sigma': uncertainties[key][1]}
        fusion_error = ''
        try:
            fused = dynamic_fusion(layers, self.cfg['resolution']) if layers else None
        except ValueError as error:
            fused, fusion_error = None, str(error)
        state = MappingState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.frame
        state.session_id, state.robot_id, state.boot_id = self.session, self.name, self.boot
        state.map_epoch, state.revision = self.epoch, self.revision
        state.frame_epoch, state.anchor_id, state.anchor_boot = epoch, root[0], root[1]
        state.aligned = bool(self.frozen_anchor and local_from_team and len(transforms) >= 2 and not fusion_error)
        if local_from_team:
            set_pose(state.team_from_local, team_poses[self.own_key()])
        details = {'transforms': transforms, 'map_ages': {n: now-t for n, t in self.received.items()},
                   'constraints': len(self.graph.edges), 'observed_only': True, 'fusion_error': fusion_error,
                   'continuity_rejections': self.continuity_rejections,
                   'peer_echo_filter': 'confirmed_current_footprints_only'}
        details['confirmed_local_revision'] = self.confirmed_local_revision
        details['registered_local_revision'] = self.registered_local_revision
        own_peer = self.peers.get(self.name)
        if not self.admitted and own_peer and own_peer[1].aligned:
            self.admitted = True
            self.persist()
            checkpoint = self.graph_file.parent/f'first-admitted-e{self.epoch}.json'
            if not checkpoint.exists():
                checkpoint.write_text(dumps(dict(steady_time=now, graph=loads(self.graph_file.read_text()),
                    mapping=details, team_from_local=pose_list(state.team_from_local))))
        details['geometry'] = self.geometry.diagnostics(self.geometry_members(now), now)
        details['geometry']['checks'] = self.last_geometry_checks
        details['worker'] = dict(active_kind=self.future_kind if self.future else None,
            active_age=now-self.future_started if self.future else 0., last=dict(self.last_worker))
        details['registration'] = dict(last=dict(self.last_registration),
            candidates=[dict(source=list(key[:3]), target=list(key[3:]), stable_count=value[1],
                             stable_seconds=now-value[2], age=now-value[3])
                        for key, value in list(self.candidates.items())[-4:]],
            pending_count=sum(len(m.witnesses) < 2 for m in self.constraints.values()),
            pending=[dict(constraint=m.constraint_id, source=m.source_robot, source_boot=m.source_boot,
                          source_epoch=m.source_epoch, source_revision=m.source_revision,
                          target=m.target_robot, target_boot=m.target_boot, target_epoch=m.target_epoch,
                          target_revision=m.target_revision, witnesses=list(m.witnesses))
                     for m in [m for m in self.constraints.values() if len(m.witnesses) < 2][-4:]])
        if fused:
            data, origin = fused
            versions = tuple(sorted((n, s.boot, s.revision) for n, s in self.maps.items()))
            data, completion = self.completion.update(data, self.cfg['resolution'], origin, versions, now)
            details.update(completion)
            details.update(origin=list(origin), shape=list(data.shape),
                           known_fraction=float(np.mean(data >= 0)))
            message = grid_message(data, self.cfg['resolution'], origin, self.frame, state.header.stamp)
            self.sharedpub.publish(message)
            nav = data.copy()
            exclusive = bool(self.authorization and now-self.authorization[0] < .5
                             and self.authorization[1].enabled
                             and self.authorization[1].resource == 'team_motion'
                             and not self.authorization[1].bootstrap)
            for n, (pose, age) in self.peer_positions(now).items():
                if n == self.name:
                    continue
                radius = (navigation_peer_radius(self.cfg['robot_radius'], self.cfg['max_speed'], exclusive)
                          if age <= .8 else lost_peer_radius(self.cfg['robot_radius'],
                                                            self.cfg['max_speed'], age))
                nav = peer_obstacle_grid(nav.shape, self.cfg['resolution'], origin, [pose[:2]], radius, nav)
            if self.authorization and now-self.authorization[0] < 1:
                try:
                    limits = loads(self.authorization[1].limits_json)
                    if limits.get('region_enabled') and local_from_team:
                        margin = self.cfg['region_margin']+self.cfg['robot_radius']+.03
                        seeds = {n: compose(local_from_team, p) for n, p in limits['seeds'].items()}
                        if limits.get('regions'):
                            yy, xx = np.indices(nav.shape)
                            pts = np.column_stack([origin[0]+(xx.ravel()+.5)*self.cfg['resolution'],
                                                   origin[1]+(yy.ravel()+.5)*self.cfg['resolution']])
                            pts = transform_points(team_poses[self.own_key()], pts)
                            b = limits['regions'][MEMBERS.index(self.name)]
                            mask = ((pts[:, 0] >= b[0]+margin) & (pts[:, 0] <= b[2]-margin)
                                    & (pts[:, 1] >= b[1]+margin) & (pts[:, 1] <= b[3]-margin)).reshape(nav.shape)
                        else:
                            mask = voronoi_mask(nav.shape, self.cfg['resolution'], origin, seeds, self.name, margin)
                        nav[~mask] = 100
                except (ValueError, KeyError, TypeError):
                    nav[:] = 100
            self.navpub.publish(grid_message(nav, self.cfg['resolution'], origin, self.frame, state.header.stamp))
        state.details_json = dumps(details)
        self.statepub.publish(state)

    def close(self):
        self.registration_stop.set()
        self.pool.shutdown(wait=True, cancel_futures=True)


def main():
    with RosRuntime() as runtime:
        node = LocalMapper()
        try:
            runtime.spin(node)
        finally:
            node.close()
            node.destroy_node()
