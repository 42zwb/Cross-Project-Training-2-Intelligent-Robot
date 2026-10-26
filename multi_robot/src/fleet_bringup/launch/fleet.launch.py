from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fleet_bringup.prepare import prepare
from ament_index_python.packages import get_package_share_directory as share


def setup(context):
    out = Path(LaunchConfiguration("assets").perform(context))
    cfg = prepare(out, LaunchConfiguration("config").perform(context))
    full = LaunchConfiguration("mode").perform(context) == "full"
    actions = [
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", str(out) + os.pathsep + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")),
        ExecuteProcess(
            cmd=["ign", "gazebo", str(out / "arena.sdf"), "-r", "-s", "-v", "2"],
            output="log",
        ),
        Node(
            package="fleet_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            output="log",
        ),
    ]
    if LaunchConfiguration("gui").perform(context) == "true":
        actions.append(
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ign",
                            "gazebo",
                            "-g",
                            "-v",
                            "1",
                            "--gui-config",
                            str(Path(share("fleet_bringup")) / "config/gui.config"),
                        ],
                        additional_env={
                            "LIBGL_ALWAYS_SOFTWARE": "1",
                            "LP_NUM_THREADS": "2",
                        },
                        output="log",
                    )
                ],
            )
        )
    for i, r in enumerate(cfg["robots"]):
        n = r["name"]
        x, y, yaw = r["start"]
        actions.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=n,
                    parameters=[
                        {
                            "use_sim_time": True,
                            "frame_prefix": n + "/",
                            "robot_description": (out / f"{n}.urdf").read_text(),
                        }
                    ],
                    output="log",
                ),
                TimerAction(
                    period=2.0 + i * 2.0,
                    actions=[
                        Node(
                            package="ros_gz_sim",
                            executable="create",
                            namespace=n,
                            arguments=[
                                "-world",
                                "fleet_arena",
                                "-name",
                                n,
                                "-topic",
                                f"/{n}/robot_description",
                                "-x",
                                str(x),
                                "-y",
                                str(y),
                                "-z",
                                ".02",
                                "-Y",
                                str(yaw),
                            ],
                            output="log",
                        )
                    ],
                ),
                Node(
                    package="fleet_bridge",
                    executable="parameter_bridge",
                    namespace=n,
                    name="scan_bridge",
                    arguments=[
                        f"/{n}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"
                    ],
                    parameters=[{"use_sim_time": True}],
                    output="log",
                ),
                TimerAction(
                    period=4.0 + i * 2.0,
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            namespace=n,
                            arguments=[
                                "joint_state_broadcaster",
                                "diffdrive_controller",
                                "-c",
                                f"/{n}/controller_manager",
                                "--controller-manager-timeout",
                                "120",
                            ],
                            output="log",
                        )
                    ],
                ),
            ]
        )
        if full:
            actions.extend(
                [
                    Node(
                        package="cartographer_ros",
                        executable="cartographer_node",
                        namespace=n,
                        arguments=[
                            "-configuration_directory",
                            str(out),
                            "-configuration_basename",
                            f"{n}.lua",
                        ],
                        parameters=[{"use_sim_time": True}],
                        remappings=[("scan", "scan_slam")],
                        output="log",
                    ),
                    Node(
                        package="cartographer_ros",
                        executable="cartographer_occupancy_grid_node",
                        namespace=n,
                        arguments=["-resolution", str(cfg["resolution"]), "-publish_period_sec", "1.0"],
                        parameters=[{"use_sim_time": True}],
                        remappings=[("map", "local_map")],
                        output="log",
                    ),
                ]
            )
    if full:
        actions.append(
            Node(
                package="fleet_bringup",
                executable="nav_ready",
                arguments=["--assets", str(out)],
                output="log",
            )
        )
        for pkg, exe in [
            ("fleet_mapping", "mapping"),
            ("fleet_control", "safety"),
            ("fleet_control", "coordinator"),
            ("fleet_control", "dashboard"),
        ]:
            actions.append(
                Node(
                    package=pkg,
                    executable=exe,
                    name=exe,
                    parameters=[
                        {
                            "use_sim_time": True,
                            "config": str(out / "fleet.yaml"),
                            "run_dir": str(out.parent),
                        }
                    ],
                    output="log",
                )
            )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("mode", default_value="full"),
            DeclareLaunchArgument("assets", default_value="/tmp/fleet_assets"),
            DeclareLaunchArgument(
                "config",
                default_value=str(Path(share("fleet_bringup")) / "config/fleet.yaml"),
            ),
            OpaqueFunction(function=setup),
        ]
    )
