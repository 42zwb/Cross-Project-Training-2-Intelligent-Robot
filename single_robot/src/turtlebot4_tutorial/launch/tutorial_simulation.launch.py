from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from turtlebot4_tutorial.launch_support import rendering_environment


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('rviz', default_value='false'),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('world', default_value='warehouse'),
    DeclareLaunchArgument('model', default_value='lite', choices=['standard', 'lite']),
    DeclareLaunchArgument('spawn_dock', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('software_rendering', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.0'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
    DeclareLaunchArgument('gz_args', default_value=''),
]


def launch_world(context):
    """Keep GPU sensors on the server and use software GL only in the GUI."""
    ignition_pkg = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    raw = LaunchConfiguration('gz_args').perform(context)
    arguments = shlex.split(raw) if raw else [world + '.sdf', '-r', '-v', '1']
    if '-g' in arguments or '--gui-only' in arguments:
        raise ValueError('仿真入口需要服务器；请使用 gui:=true 开启独立图形窗口')
    gui = (LaunchConfiguration('gui').perform(context) == 'true'
           and '-s' not in arguments and '--server-only' not in arguments)
    if '-s' not in arguments and '--server-only' not in arguments:
        arguments.append('-s')
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ignition_pkg / 'launch/ignition.launch.py')),
        launch_arguments={
            'world': world, 'model': model,
            'gz_args': shlex.join(arguments),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    actions = [server]
    if gui:
        actions.append(ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '--gui-config',
                 str(ignition_pkg / 'gui' / model / 'gui.config')],
            output='screen', additional_env=rendering_environment(context),
        ))
    return actions


def generate_launch_description():
    tutorial_pkg = get_package_share_directory('turtlebot4_tutorial')
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tutorial_pkg, 'launch', 'tutorial_spawn.launch.py'])),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in [
                'namespace', 'rviz', 'use_sim_time', 'model',
                'x', 'y', 'z', 'yaw', 'spawn_dock', 'software_rendering',
            ]
        }.items(),
    )
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_world), spawn])
