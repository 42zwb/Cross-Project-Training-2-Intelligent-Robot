"""Launch the velocity arbiter and full-footprint collision filter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Start the two safety stages in the requested namespace."""
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='turtlebot4_tutorial',
                executable='velocity_arbiter',
                name='velocity_arbiter',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'nav_topic': 'cmd_vel',
                    'teleop_topic': 'cmd_vel_raw',
                    'aux_topic': 'cmd_vel_aux',
                    'output_topic': 'cmd_vel_arbitrated',
                    'scan_topic': 'scan',
                    'odom_topic': 'odom',
                    'require_odom_tf': True,
                    'require_nav_localization': True,
                }],
            ),
            Node(
                package='turtlebot4_tutorial',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': 'cmd_vel_arbitrated',
                    'output_topic': 'cmd_vel_safe',
                    'scan_topic': 'scan',
                    'base_frame': 'base_link',
                    'stop_distance': 0.35,
                    'slow_distance': 0.70,
                    'robot_radius': 0.175,
                    'reaction_time': 0.60,
                    'lateral_margin': 0.02,
                    'rotation_margin': 0.10,
                }],
            ),
        ]),
    ])
