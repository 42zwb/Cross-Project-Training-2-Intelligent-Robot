#!/usr/bin/env python3
"""Independent acceptance telemetry; publishes no missions or control data."""
import json
from pathlib import Path
import sys
import time
import rclpy
from fleet_control.peer_client import PeerClient
from fleet_control.peer_recording import RawMapRecorder, NavigationRecorder
from fleet_control.peer_common import STATE_QOS
from fleet_control.ros_runtime import RosRuntime
from fleet_interfaces.msg import MapSnapshot
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data


def main():
    run = Path(sys.argv[1]).resolve()
    end = time.monotonic()+30
    while not (run/'architecture.json').exists() and time.monotonic() < end:
        time.sleep(.1)
    with RosRuntime() as runtime:
        client = PeerClient(run)
        maps = RawMapRecorder(run, client.session)
        navigation = NavigationRecorder(run)
        for name in ('robot1', 'robot2', 'robot3'):
            client.node.create_subscription(MapSnapshot, f'/{name}/peer/map',
                lambda message, n=name: maps.receive(n, message), STATE_QOS)
            for channel, topic in (('shared_map', 'shared_map'),
                                   ('navigation_map', 'navigation_map'),
                                   ('costmap', 'local_costmap/costmap'),
                                   ('global_costmap', 'global_costmap/costmap')):
                client.node.create_subscription(OccupancyGrid, f'/{name}/{topic}',
                    lambda m, n=name, c=channel: navigation.grid(n, c, m), STATE_QOS)
            client.node.create_subscription(LaserScan, f'/{name}/scan',
                lambda m, n=name: navigation.scan(n, m), qos_profile_sensor_data)
            for channel, qos in (('tf', qos_profile_sensor_data), ('tf_static', STATE_QOS)):
                client.node.create_subscription(TFMessage, f'/{name}/{channel}',
                    lambda m, n=name, c=channel: navigation.tf(n, c, m), qos)
        try:
            with (run/'peer-observations.jsonl').open('a', buffering=1) as stream:
                while runtime.ok():
                    state = client.state(.5)
                    maps.checkpoint(state)
                    navigation.checkpoint(state, client.states)
                    stream.write(json.dumps(state, ensure_ascii=False, allow_nan=False)+'\n')
        finally:
            client.close()


if __name__ == '__main__':
    main()
