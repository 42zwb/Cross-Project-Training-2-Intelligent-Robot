"""Optional UI compatibility projection. Has no navigation or drive authority."""
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from fleet_interfaces.srv import FleetCommand, SetFormation, SetRegions
from fleet_mapping.grid import valid_grid_message
from fleet_mapping.registration import dynamic_fusion
from .peer_common import load_config, STATE_QOS, dumps, grid_message, pose_list, local_frame
from .peer_client import PeerClient
from .peer_protocol import MEMBERS


class PeerGateway(Node):
    def __init__(self):
        super().__init__('peer_gateway')
        self.cfg = load_config(self)
        self.client = PeerClient(self.cfg['run_dir'], node=self)
        self.maps = {}
        self.group = ReentrantCallbackGroup()
        self.statepub = self.create_publisher(String, '/fleet/state', STATE_QOS)
        self.mappub = self.create_publisher(OccupancyGrid, '/map', STATE_QOS)
        for name in MEMBERS:
            self.create_subscription(OccupancyGrid, f'/{name}/shared_map',
                                     lambda m, n=name: self.map_cb(n, m), STATE_QOS)
        self.create_service(FleetCommand, '/fleet/command', self.command, callback_group=self.group)
        self.create_service(SetFormation, '/fleet/set_formation', self.formation, callback_group=self.group)
        self.create_service(SetRegions, '/fleet/set_regions', self.regions, callback_group=self.group)
        self.create_timer(.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.log_dir = Path(self.cfg['run_dir'])
        self.telemetry = (self.log_dir/'telemetry.jsonl').open('a', buffering=1)
        self.events = (self.log_dir/'events.jsonl').open('a', buffering=1)
        self.seen = set()
        self.arena = [-10., -10., 10., 10.]

    def map_cb(self, name, message):
        if valid_grid_message(message):
            self.maps[name] = message

    def service(self, kind, data, response):
        try:
            result = self.client.request(kind, data, timeout=10)
            response.success, response.message = result['success'], result['message']
        except (ValueError, KeyError, TypeError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def command(self, request, response):
        return self.service('command', {'command': request.command, 'argument': request.argument}, response)

    def formation(self, request, response):
        return self.service('formation', dict(shape=request.shape, x=request.x, y=request.y,
                                              yaw=request.yaw, spacing=request.spacing), response)

    def regions(self, request, response):
        return self.service('regions', {'regions': [list(request.bounds[i:i+4]) for i in range(0, 12, 4)]}, response)

    def tick(self):
        state = self.client.state(timeout=0)
        live = state['online_members']
        candidates = [n for n in live if n in self.maps and n in self.client.mappings
                      and self.client.mappings[n][1].aligned
                      and time.monotonic()-self.client.mappings[n][0] < 2.
                      and self.client.states[n][1].boot_id == self.client.mappings[n][1].boot_id
                      and self.client.mappings[n][1].frame_epoch == state['coordinate_frame']
                      and self.maps[n].header.frame_id == local_frame(n, self.client.mappings[n][1].map_epoch)]
        if candidates:
            name = min(candidates)
            grid = self.maps[name]
            mapping = self.client.mappings[name][1]
            data = np.asarray(grid.data, np.int16).reshape(grid.info.height, grid.info.width)
            q = grid.info.origin.orientation
            import math
            oyaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            fused, origin = dynamic_fusion([(data, grid.info.resolution, grid.info.origin.position.x,
                grid.info.origin.position.y, oyaw, pose_list(mapping.team_from_local))], grid.info.resolution)
            self.mappub.publish(grid_message(fused, grid.info.resolution, origin, state['coordinate_frame'],
                                             self.get_clock().now().to_msg()))
            self.arena = [origin[0], origin[1], origin[0]+fused.shape[1]*grid.info.resolution,
                          origin[1]+fused.shape[0]*grid.info.resolution]
        state['arena'] = self.arena
        state['sim_time'] = self.get_clock().now().nanoseconds*1e-9
        state['wall_time'] = time.time()
        self.statepub.publish(String(data=dumps(state)))
        self.telemetry.write(dumps(state)+'\n')
        for _, _, details in list(self.client.states.values()):
            for item in details.get('events', []):
                key = (item['robot'], item['wall_time'], item['event'])
                if key not in self.seen:
                    self.events.write(dumps(item)+'\n')
                    self.seen.add(key)


def main():
    from .ros_runtime import RosRuntime
    with RosRuntime() as runtime:
        node = PeerGateway()
        try:
            runtime.spin(node, MultiThreadedExecutor(num_threads=4))
        finally:
            node.telemetry.close()
            node.events.close()
            node.destroy_node()
