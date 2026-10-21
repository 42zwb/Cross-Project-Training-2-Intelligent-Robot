from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def create_cartographer(context):
    """Start a new trajectory, optionally restoring a saved SLAM state."""
    config_dir = get_package_share_directory('turtlebot4_cartographer') + '/config'
    arguments = [
        '-configuration_directory', config_dir,
        '-configuration_basename', 'turtlebot4_2d.lua',
    ]
    filename = LaunchConfiguration('load_state_filename').perform(context)
    if filename:
        path = Path(filename).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f'Cartographer 状态文件不存在: {path}')
        arguments.extend([
            '-load_state_filename', str(path),
            '-load_frozen_state=' + LaunchConfiguration('load_frozen_state').perform(context),
        ])
    return [Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node', output='screen', arguments=arguments,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument('load_frozen_state', default_value='false',
                              choices=['true', 'false']),
        GroupAction([
            PushRosNamespace(namespace),
            OpaqueFunction(function=create_cartographer),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                arguments=[
                    '-resolution', '0.05', '-publish_period_sec', '1.0'],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),
    ])
