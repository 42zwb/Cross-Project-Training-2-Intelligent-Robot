from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    cfg = yaml.safe_load(Path(LaunchConfiguration('robot_config').perform(context)).read_text())
    name = cfg['robot_id']
    executables = [('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
                   ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')]
    remaps = [('/tf', f'/{name}/tf'), ('/tf_static', f'/{name}/tf_static'), ('cmd_vel', 'cmd_vel_nav')]
    actions = [Node(package=pkg, executable=exe, name=exe, namespace=name,
        parameters=[str(Path(cfg['assets'])/f'{name}_nav.yaml'), {'use_sim_time': True}],
        remappings=remaps, output='screen') for pkg, exe in executables]
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=name,
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.,
                     'node_names': [exe for _, exe in executables]}], remappings=remaps, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_config'), OpaqueFunction(function=setup)])
