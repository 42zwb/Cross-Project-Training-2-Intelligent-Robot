"""Plan or execute the MTC Panda pick-and-place task."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Run the installed MTC example, with execution disabled by default."""
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path='config/panda.urdf.xacro')
        .to_moveit_configs()
    )
    demo_config = get_package_share_directory('moveit_task_constructor_demo')
    task = Node(
        package='moveit_task_constructor_demo',
        executable='pick_place_demo',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            f'{demo_config}/config/panda_config.yaml',
            {'execute': ParameterValue(
                LaunchConfiguration('execute'), value_type=bool)},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'execute', default_value='false', choices=['true', 'false'],
            description='Execute the selected trajectory on Panda mock controllers'),
        task,
    ])
