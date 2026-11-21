"""World infrastructure plus separately scoped robot autonomy launch trees."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from ament_index_python.packages import get_package_prefix
from launch.actions import (ExecuteProcess, IncludeLaunchDescription, TimerAction,
                             SetEnvironmentVariable, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .simulation_preflight import launch_preflight, SESSION
from .peer_assets import prepare_agents


def setup(context):
    out = Path(LaunchConfiguration('assets').perform(context))
    launch_preflight(context, project='fleet', report_path=out.parent/'startup_preflight.json')
    no_dashboard = LaunchConfiguration('no_dashboard').perform(context) == 'true'
    cfg = prepare_agents(out, LaunchConfiguration('config').perform(context),
                         context.environment[SESSION], no_dashboard,
                         LaunchConfiguration('acceptance_scene').perform(context))
    full = LaunchConfiguration('mode').perform(context) == 'full'
    actions = [SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
        str(out)+os.pathsep+os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=[str(Path(get_package_prefix('fleet_bridge'))/'lib/fleet_bridge/fleet_sim_server'),
                            str(out/'arena.sdf')], output='log'),
        Node(package='fleet_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='log')]
    if LaunchConfiguration('gui').perform(context) == 'true':
        actions.append(TimerAction(period=4., actions=[ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '-v', '1', '--gui-config', str(Path(share('fleet_bringup'))/'config/gui.config')],
            output='log', additional_env={'LIBGL_ALWAYS_SOFTWARE': os.environ.get('FLEET_SOFTWARE_RENDERING', '0'),
                                         'LP_NUM_THREADS': '2'})]))
    for i, robot in enumerate(cfg['robots']):
        name = robot['name']
        x, y, yaw = robot['start']  # This module belongs to the simulator, never a robot.
        remaps = [('/tf', f'/{name}/tf'), ('/tf_static', f'/{name}/tf_static')]
        actions.extend([
            Node(package='fleet_bringup', executable='odom_tf', namespace=name,
                 parameters=[{'config': str(out/'agents'/f'{name}.yaml'), 'use_sim_time': True}],
                 remappings=remaps, output='log'),
            Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=name,
                 parameters=[{'use_sim_time': True, 'frame_prefix': name+'/',
                              'robot_description': (out/f'{name}.urdf').read_text(), 'publish_frequency': 10.}],
                 remappings=remaps, output='log'),
            Node(package='fleet_bridge', executable='parameter_bridge', namespace=name, name='scan_bridge',
                 arguments=[f'/{name}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                 parameters=[{'use_sim_time': True}], output='log'),
            TimerAction(period=2.+i*2., actions=[Node(package='ros_gz_sim', executable='create', namespace=name,
                arguments=['-world', 'fleet_arena', '-name', name, '-file', str(out/f'{name}.sdf'),
                           '-x', str(x), '-y', str(y), '-z', '.02', '-Y', str(yaw)], output='log')]),
            TimerAction(period=4.+i*2., actions=[Node(package='controller_manager', executable='spawner',
                namespace=name, arguments=['joint_state_broadcaster', 'diffdrive_controller',
                    '-c', f'/{name}/controller_manager', '--controller-manager-timeout', '120'], output='log')])])
        if full:
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(Path(share('fleet_bringup'))/'launch/robot.launch.py')),
                launch_arguments={'robot_config': str(out/'agents'/f'{name}.yaml')}.items()))
    if full and not no_dashboard:
        actions.append(Node(package='fleet_control', executable='peer_gateway', name='peer_gateway',
            parameters=[{'config': str(out/'agents/robot1.yaml'), 'use_sim_time': True}], output='log'))
        actions.append(Node(package='fleet_control', executable='dashboard', name='dashboard',
            parameters=[{'config': str(out/'agents/robot1.yaml'), 'run_dir': str(out.parent),
                         'use_sim_time': True}], output='log'))
    return actions
