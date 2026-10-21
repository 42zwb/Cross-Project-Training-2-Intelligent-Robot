from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml, os


def setup(context):
    out = Path(LaunchConfiguration("assets").perform(context))
    cfg = yaml.safe_load((out / "fleet.yaml").read_text())
    actions = []
    for r in cfg["robots"]:
        n = r["name"]
        for package, exe in [
            ("nav2_controller", "controller_server"),
            ("nav2_planner", "planner_server"),
            ("nav2_behaviors", "behavior_server"),
            ("nav2_bt_navigator", "bt_navigator"),
        ]:
            actions.append(
                Node(
                    package=package,
                    executable=exe,
                    namespace=n,
                    name=exe,
                    parameters=[str(out / f"{n}_nav.yaml"), {"use_sim_time": True}],
                    additional_env=(
                        {"LD_PRELOAD": os.environ["FLEET_DEBUG_PTRACE"]}
                        if os.environ.get("FLEET_DEBUG_PTRACE")
                        else None
                    ),
                    remappings=[
                        ("cmd_vel", "cmd_vel_nav"),
                        ("tf2_frames", f"/{n}/{exe}/tf2_frames"),
                    ],
                    output="log",
                )
            )
        actions.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=n,
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "bond_timeout": 10.0,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                        ],
                    }
                ],
                output="log",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("assets"), OpaqueFunction(function=setup)]
    )
