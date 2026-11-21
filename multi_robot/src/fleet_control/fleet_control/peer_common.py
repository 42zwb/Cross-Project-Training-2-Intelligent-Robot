"""Shared, bounded serialization and ROS helpers for independent robot nodes."""
import json
import math
import os
from pathlib import Path
import uuid
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from fleet_mapping.registration import Snapshot
from .peer_protocol import MEMBERS

STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
EVENT_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
CONSTRAINT_QOS = QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)


def dumps(value):
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def loads(value, maximum=131072):
    if not isinstance(value, str) or len(value.encode()) > maximum:
        raise ValueError('oversized peer payload')

    def invalid(text):
        raise ValueError('nonfinite JSON')
    result = json.loads(value, parse_constant=invalid)
    if not isinstance(result, dict):
        raise ValueError('peer payload must be an object')
    return result


def load_config(node):
    node.declare_parameter('config', '')
    path = Path(node.get_parameter('config').value)
    cfg = yaml.safe_load(path.read_text())
    allowed = {'architecture', 'robot_id', 'members', 'session_id', 'resolution',
               'robot_radius', 'max_speed', 'region_margin', 'run_dir',
               'dashboard_port', 'assets', 'boot_id', 'map_epoch', 'no_dashboard'}
    if (not isinstance(cfg, dict) or set(cfg)-allowed or cfg.get('architecture') != 'decentralized'
            or cfg.get('robot_id') not in MEMBERS or cfg.get('members') != list(MEMBERS)
            or not cfg.get('session_id') or not cfg.get('boot_id')):
        raise ValueError('invalid/simulation-contaminated agent configuration')
    return cfg


def pose_list(message):
    values = [float(message.x), float(message.y), float(message.theta)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('invalid pose')
    return values


def set_pose(message, values):
    message.x, message.y, message.theta = map(float, values)


def grid_message(data, resolution, origin, frame, stamp):
    message = OccupancyGrid()
    message.header.frame_id, message.header.stamp = frame, stamp
    message.info.height, message.info.width = np.asarray(data).shape
    message.info.resolution = float(resolution)
    message.info.origin.position.x, message.info.origin.position.y = map(float, origin[:2])
    message.info.origin.orientation.w = 1.
    message.data = np.asarray(data, dtype=np.int8).ravel().tolist()
    return message


def snapshot_from_message(message):
    g = message.grid
    q = g.info.origin.orientation
    heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return Snapshot(message.robot_id, message.boot_id, message.map_epoch, message.revision,
                    np.asarray(g.data, np.int16).reshape(g.info.height, g.info.width),
                    g.info.resolution, (g.info.origin.position.x, g.info.origin.position.y, heading))


def peer_topic(robot, channel):
    return f'/{robot}/peer/{channel}'


def local_frame(robot, epoch):
    if robot not in MEMBERS or not isinstance(epoch, int) or epoch < 1:
        raise ValueError('invalid local frame identity')
    return robot+'/map' if epoch == 1 else f'{robot}/map_e{epoch}'


def component_parameters(cfg):
    return {'config': str(Path(cfg['assets'])/'agents'/f'{cfg["robot_id"]}.yaml'),
            'use_sim_time': True}
