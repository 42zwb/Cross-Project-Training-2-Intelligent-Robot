from pathlib import Path
import importlib.util
import os
import shlex
import shutil
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from turtlebot4_tutorial.launch_support import rendering_environment
from turtlebot4_tutorial.simulation_preflight import launch_preflight


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('rviz', default_value='false'),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('world', default_value='warehouse'),
    DeclareLaunchArgument('model', default_value='lite', choices=['standard', 'lite']),
    DeclareLaunchArgument('spawn_dock', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('software_rendering', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('safety', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
    DeclareLaunchArgument('sensors_plugin', default_value='true',
                          choices=['true', 'false']),
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
    # The upstream ros_gz launch uses a shell and Ruby CLI; terminating the
    # CLI alone can leave the actual Gazebo server orphaned. Run each tree in
    # its own process group so shutdown reaches precisely our descendants.
    ros_gz = Path(get_package_share_directory('ros_gz_sim'))
    spec = importlib.util.spec_from_file_location(
        'tutorial_upstream_gz_launch', ros_gz / 'launch' / 'gz_sim.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model_paths, plugin_paths = module.GazeboRosPaths.get_paths()
    create_pkg = Path(get_package_share_directory('irobot_create_ignition_bringup'))
    tb_description = Path(get_package_share_directory('turtlebot4_description'))
    create_description = Path(get_package_share_directory('irobot_create_description'))
    resource_path = os.pathsep.join([
        str(ignition_pkg / 'worlds'), str(create_pkg / 'worlds'),
        str(tb_description.parent.resolve()),
        str(create_description.parent.resolve()), model_paths,
    ])
    gui_plugins = Path(get_package_share_directory('turtlebot4_ignition_gui_plugins'))
    create_plugins = Path(get_package_share_directory('irobot_create_ignition_plugins'))
    env = {
        'IGN_GAZEBO_RESOURCE_PATH': resource_path,
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''), model_paths]),
        'IGN_GUI_PLUGIN_PATH': os.pathsep.join([
            str(gui_plugins / 'lib'), str(create_plugins / 'lib')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''), plugin_paths]),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''), plugin_paths]),
    }
    ignition = shutil.which('ign')
    if not ignition:
        raise RuntimeError('无法找到 ign 命令')
    wrapper = [sys.executable, '-m', 'turtlebot4_tutorial.process_group']
    server = ExecuteProcess(
        cmd=wrapper + ['ruby', ignition, 'gazebo'] + arguments + ['--force-version', '6'],
        output='screen', additional_env=env, sigterm_timeout='10', sigkill_timeout='3')
    actions = [server, Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])]
    if gui:
        gui_env = dict(env)
        gui_env.update(rendering_environment(context))
        actions.append(ExecuteProcess(
            cmd=wrapper + ['ruby', ignition, 'gazebo', '-g', '--gui-config',
                           str(ignition_pkg / 'gui' / model / 'gui.config'),
                           '--force-version', '6'],
            output='screen', additional_env=gui_env, sigterm_timeout='10', sigkill_timeout='3'))
    return actions


def generate_launch_description():
    tutorial_pkg = get_package_share_directory('turtlebot4_tutorial')
    safety = LaunchConfiguration('safety')
    spawn_cmd_vel_topic = PythonExpression([
        "'cmd_vel_safe' if '", safety, "' == 'true' else '",
        LaunchConfiguration('cmd_vel_topic'), "'"
    ])
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tutorial_pkg, 'launch', 'tutorial_spawn.launch.py'])),
        launch_arguments=(
            {
                name: LaunchConfiguration(name)
                for name in [
                    'namespace', 'rviz', 'use_sim_time', 'model',
                    'x', 'y', 'z', 'yaw', 'spawn_dock',
                    'software_rendering', 'sensors_plugin',
                ]
            }
            | {'cmd_vel_topic': spawn_cmd_vel_topic}
        ).items(),
    )
    safety_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tutorial_pkg, 'launch', 'tutorial_safety.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(safety),
    )
    return LaunchDescription(
        ARGUMENTS + [OpaqueFunction(function=launch_preflight),
                     OpaqueFunction(function=launch_world), spawn, safety_nodes])
