"""Attach one autonomy tree to an existing world; never run world preflight."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = Path(LaunchConfiguration('robot_config').perform(context)).resolve()
    cfg = yaml.safe_load(config.read_text())
    name, assets = cfg['robot_id'], Path(cfg['assets'])
    params = [{'config': str(config), 'use_sim_time': True}]
    remaps = [('/tf', f'/{name}/tf'), ('/tf_static', f'/{name}/tf_static')]
    actions = [SetEnvironmentVariable('TB4_SIM_ROBOT', name),
               SetEnvironmentVariable('TB4_SIM_COMPONENT', 'autonomy')]
    actions += [Node(package='cartographer_ros', executable='cartographer_node', namespace=name,
        parameters=[{'use_sim_time': True}], arguments=['-configuration_directory', str(assets),
            '-configuration_basename', f'{name}.lua'],
        remappings=remaps+[('scan', 'scan_slam')], output='log'),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', namespace=name,
            parameters=[{'use_sim_time': True}], arguments=['-resolution', str(cfg['resolution']),
                '-publish_period_sec', '1.0'], remappings=remaps+[('map', 'local_map')], output='log')]
    for executable in ['local_mapper', 'peer_agent', 'local_safety']:
        actions.append(Node(package='fleet_control', executable=executable, namespace=name,
                            parameters=params, remappings=remaps, output='log'))
    actions.append(Node(package='fleet_bringup', executable='local_nav_ready', namespace=name,
                        parameters=params, remappings=remaps, output='log'))
    return [GroupAction(actions=actions)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_config'), OpaqueFunction(function=setup)])
