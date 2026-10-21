from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from turtlebot4_tutorial.launch_support import check_navigation_map, make_rviz


def generate_launch_description():
    tb4_nav = get_package_share_directory('turtlebot4_navigation')
    tutorial = get_package_share_directory('turtlebot4_tutorial')

    start_sim = LaunchConfiguration('start_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    gz_args = LaunchConfiguration('gz_args')
    model = LaunchConfiguration('model')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tutorial, 'launch', 'tutorial_simulation.launch.py'])),
        launch_arguments={
            'model': model,
            'world': world,
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'software_rendering': LaunchConfiguration('software_rendering'),
            'gui': LaunchConfiguration('gui'),
            'gz_args': gz_args,
            'x': x,
            'y': y,
            'z': LaunchConfiguration('z'),
            'yaw': yaw,
            'spawn_dock': LaunchConfiguration('spawn_dock'),
        }.items(),
        condition=IfCondition(start_sim),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_nav, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'namespace': namespace,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_nav, 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'namespace': namespace,
        }.items(),
    )
    rviz_node = OpaqueFunction(function=lambda context: make_rviz(context, 'navigation'))
    initial_pose = Node(
        package='turtlebot4_tutorial',
        executable='initial_pose_publisher',
        name='tutorial_initial_pose',
        output='screen',
        arguments=[x, y, yaw, '--duration', '45.0', '--wait-for-amcl'],
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=namespace,
        condition=IfCondition(auto_initial_pose),
    )

    def start_navigation_after_localization(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='AMCL 已就绪，启动 Nav2。'), navigation, rviz_node]
        return [
            LogInfo(msg='AMCL 定位准备失败；停止启动，避免 Nav2 在缺少 map TF 时接收目标。'),
            EmitEvent(event=Shutdown(reason='AMCL did not confirm initial pose')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('software_rendering', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'auto_initial_pose', default_value='true',
            description='Publish the configured x, y and yaw start pose to AMCL'),
        DeclareLaunchArgument('model', default_value='lite', choices=['lite', 'standard']),
        # Direct forward navigation from the dock can spin the wheels against
        # its low collision box while wheel odometry falsely reports progress.
        DeclareLaunchArgument('spawn_dock', default_value='false',
                              choices=['true', 'false'],
                              description='Enable the charging dock for docking experiments'),
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gz_args', default_value=''),
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution([tb4_nav, 'maps', 'warehouse.yaml'])),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution(
                [tutorial, 'config', 'nav2_params.yaml'])),
        OpaqueFunction(function=check_navigation_map),
        GroupAction(actions=[simulation]),
        # Loading the 1006 x 1674 map and configuring every Nav2 server at the
        # same instant can make lifecycle service calls time out on modest PCs.
        TimerAction(period=6.0, actions=[localization]),
        RegisterEventHandler(OnProcessExit(
            target_action=initial_pose,
            on_exit=start_navigation_after_localization)),
        TimerAction(period=6.0, actions=[initial_pose]),
        # 手动初始位姿模式仍启动 Nav2，由用户在 RViz 中设置位姿。
        TimerAction(period=14.0, actions=[navigation, rviz_node],
                    condition=UnlessCondition(auto_initial_pose)),
    ])
