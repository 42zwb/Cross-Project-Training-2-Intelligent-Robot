from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from turtlebot4_tutorial.launch_support import make_rviz


def generate_launch_description():
    tutorial = get_package_share_directory('turtlebot4_tutorial')
    cartographer = get_package_share_directory('turtlebot4_cartographer')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_sim = LaunchConfiguration('start_sim')
    gz_args = LaunchConfiguration('gz_args')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tutorial, 'launch', 'tutorial_simulation.launch.py'])),
        launch_arguments={
            'model': model,
            'world': world,
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'spawn_dock': LaunchConfiguration('spawn_dock'),
            'software_rendering': LaunchConfiguration('software_rendering'),
            'gui': LaunchConfiguration('gui'),
            'gz_args': gz_args,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
        condition=IfCondition(start_sim),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [cartographer, 'launch', 'cartographer.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'load_state_filename': LaunchConfiguration('load_state_filename'),
            'load_frozen_state': LaunchConfiguration('load_frozen_state'),
        }.items(),
    )
    rviz_node = OpaqueFunction(function=make_rviz)
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('software_rendering', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('spawn_dock', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument('load_frozen_state', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('model', default_value='lite', choices=['lite', 'standard']),
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gz_args', default_value=''),
        GroupAction(actions=[simulation]),
        TimerAction(period=6.0, actions=[slam, rviz_node]),
    ])
