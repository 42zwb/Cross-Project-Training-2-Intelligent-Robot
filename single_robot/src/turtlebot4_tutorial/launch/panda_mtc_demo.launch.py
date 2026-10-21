"""Start a Panda MoveIt Task Constructor demo with mock controllers."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# Some Humble binary builds interpret `capabilities` as a replacement for the
# normal MoveGroup plugin set. List the default capabilities explicitly so the
# planning-scene services/actions and MTC execution action are all available.
MOVE_GROUP_CAPABILITIES = ' '.join([
    'move_group/MoveGroupCartesianPathService',
    'move_group/MoveGroupExecuteTrajectoryAction',
    'move_group/MoveGroupKinematicsService',
    'move_group/MoveGroupMoveAction',
    'move_group/MoveGroupPlanService',
    'move_group/MoveGroupQueryPlannersService',
    'move_group/MoveGroupStateValidationService',
    'move_group/MoveGroupGetPlanningSceneService',
    'move_group/ApplyPlanningSceneService',
    'move_group/ClearOctomapService',
    'move_group/ExecuteTaskSolutionCapability',
])


def generate_launch_description():
    """Launch MoveGroup, Panda mock controllers, state publisher, and RViz."""
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .planning_pipelines(pipelines=['ompl'])
        .robot_description(file_path='config/panda.urdf.xacro')
        .trajectory_execution(file_path='config/gripper_moveit_controllers.yaml')
        .to_moveit_configs()
    )
    panda_config = get_package_share_directory('moveit_resources_panda_moveit_config')
    mtc_config = get_package_share_directory('moveit_task_constructor_demo')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {
            'capabilities': MOVE_GROUP_CAPABILITIES,
        }],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='panda_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0'],
        output='screen',
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            f'{panda_config}/config/ros2_controllers.yaml',
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='both',
    )
    controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )
        for name in [
            'panda_arm_controller', 'panda_hand_controller',
            'joint_state_broadcaster',
        ]
    ]
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', f'{mtc_config}/config/mtc.rviz'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        robot_state_publisher,
        static_tf,
        move_group,
        controller_manager,
        *controllers,
        rviz,
    ])
