#!/usr/bin/env python3
import argparse
import os

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

from .common import parse_cli_args
from .map_tools import validate_map


def main():
    parser = argparse.ArgumentParser(
        description='启动 Nav2 map_server 并自动切换到 active 状态。')
    parser.add_argument('map', help='地图 YAML 文件的绝对路径')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parse_cli_args(parser)

    map_path = os.path.abspath(os.path.expanduser(args.map))
    if not os.path.isfile(map_path):
        parser.error(f'地图不存在: {map_path}')
    try:
        map_path = validate_map(map_path)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))

    description = LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_path,
                'use_sim_time': args.use_sim_time,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
                'use_sim_time': args.use_sim_time,
            }],
        ),
    ])
    service = LaunchService()
    service.include_launch_description(description)
    return service.run()
