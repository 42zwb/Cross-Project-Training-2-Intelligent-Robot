from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from turtlebot4_tutorial.launch_support import make_rviz


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'navigation', 'simulation']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('software_rendering', default_value='true',
                              choices=['true', 'false']),
        OpaqueFunction(function=lambda context: make_rviz(
            context, LaunchConfiguration('mode').perform(context))),
    ])
