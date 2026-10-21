"""Generate deterministic simulation assets from system TurtleBot4 description.
Obstacle geometry is exported for the separate evaluator only. Runtime mapping
and control packages never import it.
"""

from pathlib import Path
import argparse
import copy
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory as share


def boxes():
    b = [
        ("west", -5.9, 0.0, 0.2, 12.0),
        ("east", 5.9, 0.0, 0.2, 12.0),
        ("south", 0.0, -5.9, 11.6, 0.2),
        ("north", 0.0, 5.9, 11.6, 0.2),
    ]
    b += [
        (f"partition_{x}_{y}", x, y, 0.15, 3.0)
        for x in [-2.0, 2.0]
        for y in [-3.5, 3.5]
    ]
    b += [
        (f"box_{i}", x, y, 0.8, 0.8)
        for i, (x, y) in enumerate(
            [(-4, 0), (-4, 3.5), (4, 0), (4, 3.5), (0, 4), (0, -3)]
        )
    ]
    return b


def validate_config(cfg):
    """Reject fleet settings that the fixed 12 m course cannot honor safely."""
    if not isinstance(cfg, dict):
        raise ValueError("fleet 配置必须是 YAML 映射。")
    arena = cfg.get("arena")
    if (
        not isinstance(arena, list)
        or len(arena) != 4
        or not all(math.isfinite(float(v)) for v in arena)
        or any(
            not math.isclose(float(v), expected, abs_tol=1e-9)
            for v, expected in zip(arena, [-6.0, -6.0, 6.0, 6.0])
        )
    ):
        raise ValueError("当前场景固定为 arena [-6, -6, 6, 6]。")
    cfg["arena"] = [float(v) for v in arena]

    for key, low, high in [
        ("resolution", 0.025, 0.1),
        ("region_margin", 0.2, 1.0),
        ("robot_radius", 0.18, 0.3),
        ("max_speed", 0.05, 0.2),
    ]:
        value = cfg.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"配置 {key} 必须是数值。")
        value = float(value)
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"配置 {key} 必须在 {low} 到 {high} 之间。")
        cfg[key] = value
    cells_per_axis = 12.0 / cfg["resolution"]
    if not math.isclose(cells_per_axis, round(cells_per_axis), abs_tol=1e-8):
        raise ValueError("resolution 必须能整除 12 m 场地边长。")

    port = cfg.get("dashboard_port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1024 <= port <= 65535:
        raise ValueError("dashboard_port 必须是 1024 到 65535 之间的整数。")

    robots = cfg.get("robots")
    if (
        not isinstance(robots, list)
        or len(robots) != 3
        or any(not isinstance(r, dict) for r in robots)
    ):
        raise ValueError("配置必须包含三台机器人。")
    names = [r.get("name") for r in robots]
    if names != ["robot1", "robot2", "robot3"]:
        raise ValueError("当前调度器要求 robots 按顺序命名为 robot1、robot2、robot3。")
    margin = cfg["region_margin"]
    radius = cfg["robot_radius"]
    regions = []
    starts = []
    for i, robot in enumerate(robots):
        start = robot.get("start")
        region = robot.get("region")
        if (
            not isinstance(start, list)
            or len(start) != 3
            or not isinstance(region, list)
            or len(region) != 4
        ):
            raise ValueError(f"robot{i+1} 的 start 或 region 格式错误。")
        if not all(
            isinstance(v, (int, float))
            and not isinstance(v, bool)
            and math.isfinite(float(v))
            for v in start + region
        ):
            raise ValueError(f"robot{i+1} 的 start 和 region 必须是有限数值。")
        start = [float(v) for v in start]
        region = [float(v) for v in region]
        robot["start"] = start
        robot["region"] = region
        x0, y0, x1, y1 = region
        if (
            x1 - x0 < 2.0 * (margin + radius + 0.03) + 0.2
            or y1 - y0 < 2.0 * (margin + radius + 0.03) + 0.2
            or x0 < -6.0
            or y0 < -6.0
            or x1 > 6.0
            or y1 > 6.0
            or not x0 + margin <= start[0] <= x1 - margin
            or not y0 + margin <= start[1] <= y1 - margin
        ):
            raise ValueError(f"robot{i+1} 区域越界、过窄或未安全包含起点。")
        if (
            not -6.0 + radius <= start[0] <= 6.0 - radius
            or not -6.0 + radius <= start[1] <= 6.0 - radius
        ):
            raise ValueError(f"robot{i+1} 起点超出场地安全边界。")
        starts.append(start)
        regions.append(region)

    area = 0.0
    for i, (x0, y0, x1, y1) in enumerate(regions):
        area += (x1 - x0) * (y1 - y0)
        for other in regions[:i]:
            if (
                min(x1, other[2]) - max(x0, other[0]) > 1e-6
                and min(y1, other[3]) - max(y0, other[1]) > 1e-6
            ):
                raise ValueError("三个默认区域不能重叠。")
    if not math.isclose(area, 144.0, abs_tol=1e-4):
        raise ValueError("三个默认区域必须覆盖整个 12 m × 12 m 场地。")
    if any(
        math.dist(a[:2], b[:2]) <= 2.0 * radius
        for i, a in enumerate(starts)
        for b in starts[:i]
    ):
        raise ValueError("机器人起点之间缺少车体安全间距。")
    return cfg


def world(out):
    w = ET.Element("sdf", version="1.8")
    root = ET.SubElement(w, "world", name="fleet_arena")
    ET.SubElement(root, "physics", name="physics", type="ignored")
    p = root.find("physics")
    ET.SubElement(p, "max_step_size").text = "0.005"
    ET.SubElement(p, "real_time_factor").text = "1.0"
    for filename, name in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("sensors", "Sensors"),
        ("contact", "Contact"),
    ]:
        plug = ET.SubElement(
            root,
            "plugin",
            filename=f"libignition-gazebo-{filename}-system.so",
            name=f"ignition::gazebo::systems::{name}",
        )
        if name == "Sensors":
            ET.SubElement(plug, "render_engine").text = "ogre2"
    light = ET.SubElement(root, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.85 0.85 0.85 1"
    ET.SubElement(light, "direction").text = "-0.2 0.1 -1"
    floor = ET.SubElement(root, "model", name="floor")
    ET.SubElement(floor, "static").text = "true"
    link = ET.SubElement(floor, "link", name="link")
    for tag in ["collision", "visual"]:
        elem = ET.SubElement(link, tag, name=tag)
        g = ET.SubElement(elem, "geometry")
        plane = ET.SubElement(g, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "12 12"
        if tag == "visual":
            ET.SubElement(ET.SubElement(elem, "material"), "diffuse").text = (
                "0.72 0.75 0.78 1"
            )
    for name, x, y, dx, dy in boxes():
        m = ET.SubElement(root, "model", name=name)
        ET.SubElement(m, "static").text = "true"
        ET.SubElement(m, "pose").text = f"{x} {y} .6 0 0 0"
        l = ET.SubElement(m, "link", name="link")
        for tag in ["collision", "visual"]:
            c = ET.SubElement(l, tag, name=tag)
            ET.SubElement(
                ET.SubElement(ET.SubElement(c, "geometry"), "box"), "size"
            ).text = f"{dx} {dy} 1.2"
            if tag == "visual":
                ET.SubElement(ET.SubElement(c, "material"), "diffuse").text = (
                    "0.35 0.45 0.6 1"
                )
    # Ground truth transport is intentionally not bridged into runtime ROS nodes.
    (out / "arena.sdf").write_text(ET.tostring(w, encoding="unicode"))
    (out / "ground_truth.yaml").write_text(yaml.safe_dump({"boxes": boxes()}))


def nav_config(name, cfg):
    base = f"{name}/base_link"
    odom = f"{name}/odom"
    d = yaml.safe_load(
        (Path(share("turtlebot4_navigation")) / "config/nav2.yaml").read_text()
    )
    d["bt_navigator"]["ros__parameters"].update(
        global_frame="map",
        robot_base_frame=base,
        odom_topic=f"/{name}/odom",
        transform_tolerance=0.5,
    )
    ctl = d["controller_server"]["ros__parameters"]
    ctl.update(controller_frequency=10.0, failure_tolerance=1.0)
    ctl["progress_checker"].update(
        required_movement_radius=0.1, movement_time_allowance=25.0
    )
    ctl["general_goal_checker"].update(
        xy_goal_tolerance=0.08, yaw_goal_tolerance=0.10, stateful=True
    )
    ctl["FollowPath"] = {
        "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "desired_linear_vel": cfg["max_speed"],
        "lookahead_dist": 0.5,
        "min_lookahead_dist": 0.3,
        "max_lookahead_dist": 0.7,
        "lookahead_time": 1.5,
        "rotate_to_heading_angular_vel": 0.5,
        "transform_tolerance": 0.5,
        "use_velocity_scaled_lookahead_dist": True,
        "min_approach_linear_velocity": 0.035,
        "approach_velocity_scaling_dist": 0.5,
        "use_collision_detection": True,
        "max_allowed_time_to_collision_up_to_carrot": 1.0,
        "use_regulated_linear_velocity_scaling": True,
        "use_cost_regulated_linear_velocity_scaling": True,
        "regulated_linear_scaling_min_radius": 0.6,
        "regulated_linear_scaling_min_speed": 0.04,
        "use_rotate_to_heading": True,
        "rotate_to_heading_min_angle": 0.5,
        "max_angular_accel": 1.5,
        "allow_reversing": False,
        "cost_scaling_dist": 0.4,
        "cost_scaling_gain": 1.0,
        "inflation_cost_scaling_factor": 5.0,
    }
    for key in ["local_costmap", "global_costmap"]:
        p = d[key][key]["ros__parameters"]
        p.update(
            global_frame="map",
            robot_base_frame=base,
            resolution=cfg["resolution"],
            robot_radius=cfg["robot_radius"],
            transform_tolerance=0.5,
            update_frequency=5.0 if key == "local_costmap" else 2.0,
            publish_frequency=1.0,
            plugins=["static_layer", "obstacle_layer", "inflation_layer"],
            filters=["keepout_filter"],
        )
        p.pop("voxel_layer", None)
        p["static_layer"] = {
            "plugin": "nav2_costmap_2d::StaticLayer",
            "map_topic": "/map",
            "map_subscribe_transient_local": True,
            "subscribe_to_updates": False,
        }
        p["obstacle_layer"] = {
            "plugin": "nav2_costmap_2d::ObstacleLayer",
            "enabled": True,
            "observation_sources": "scan",
            "scan": {
                "topic": f"/{name}/scan",
                "data_type": "LaserScan",
                "marking": True,
                "clearing": True,
                "inf_is_valid": True,
                "raytrace_max_range": 8.0,
                "obstacle_max_range": 7.5,
                "max_obstacle_height": 2.0,
            },
        }
        p["inflation_layer"].update(
            inflation_radius=max(0.45, cfg["robot_radius"] + 0.27),
            cost_scaling_factor=5.0,
        )
        p["keepout_filter"] = {
            "plugin": "nav2_costmap_2d::KeepoutFilter",
            "enabled": True,
            "filter_info_topic": f"/{name}/keepout_info",
            "transform_tolerance": 0.5,
        }
    d["planner_server"]["ros__parameters"].update(expected_planner_frequency=1.0)
    d["planner_server"]["ros__parameters"]["GridBased"].update(
        tolerance=0.10, allow_unknown=False, use_astar=True
    )
    d["behavior_server"]["ros__parameters"].update(
        global_frame="map",
        robot_base_frame=base,
        transform_tolerance=0.5,
        max_rotational_vel=0.5,
        min_rotational_vel=0.15,
        rotational_acc_lim=1.5,
    )
    return {name: d}


def prepare(out, config):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    cfg = validate_config(yaml.safe_load(Path(config).read_text()))
    (out / "fleet.yaml").write_text(yaml.safe_dump(cfg))
    world(out)
    for robot in cfg["robots"]:
        n = robot["name"]
        cmd = [
            "xacro",
            str(
                Path(share("turtlebot4_description"))
                / "urdf/lite/turtlebot4.urdf.xacro"
            ),
            "gazebo:=ignition",
            f"namespace:={n}",
        ]
        root = ET.fromstring(subprocess.check_output(cmd, text=True))
        for parent in list(root.iter()):
            for child in list(parent):
                if child.tag == "sensor" and child.attrib.get("name") != "rplidar":
                    parent.remove(child)
                if (
                    child.tag == "plugin"
                    and "filename" in child.attrib
                    and child.attrib.get("name")
                    != "ign_ros2_control::IgnitionROS2ControlPlugin"
                ):
                    parent.remove(child)
        # Publish the passive suspension joints from measured simulator state,
        # so every wheel TF is connected to base_link without fabricated joints.
        for side in ["left", "right"]:
            hardware = root.find(f"ros2_control[@name='{side}_wheel_controller']")
            joint = ET.SubElement(hardware, "joint", name=f"wheel_drop_{side}_joint")
            ET.SubElement(joint, "state_interface", name="position")
            ET.SubElement(joint, "state_interface", name="velocity")
        lidar = root.find(".//sensor[@name='rplidar']")
        lidar.find("update_rate").text = "10"
        lidar.find("visualize").text = "false"
        ET.SubElement(lidar, "topic").text = f"/{n}/scan"
        ET.SubElement(lidar, "ignition_frame_id").text = f"{n}/rplidar_link"
        plug = root.find(
            ".//plugin[@name='ign_ros2_control::IgnitionROS2ControlPlugin']"
        )
        plug.find("parameters").text = str(out / "controllers.yaml")
        ros = plug.find("ros")
        for remap in list(ros.findall("remapping")):
            ros.remove(remap)
        for s in ["/tf:=/tf", "/tf_static:=/tf_static", "~/odom:=odom"]:
            ET.SubElement(ros, "remapping").text = s
        ET.SubElement(plug, "robot_param_node").text = "robot_state_publisher"
        # The command timeout remains enforced by native ros2_control.
        control = {
            "controller_manager": {
                "ros__parameters": {
                    "update_rate": 100,
                    "use_sim_time": True,
                    "joint_state_broadcaster": {
                        "type": "joint_state_broadcaster/JointStateBroadcaster"
                    },
                    "diffdrive_controller": {
                        "type": "diff_drive_controller/DiffDriveController"
                    },
                }
            },
            "diffdrive_controller": {
                "ros__parameters": {
                    "use_sim_time": True,
                    "left_wheel_names": ["left_wheel_joint"],
                    "right_wheel_names": ["right_wheel_joint"],
                    "wheel_separation": 0.233,
                    "wheel_radius": 0.03575,
                    "publish_rate": 30.0,
                    "odom_frame_id": f"{n}/odom",
                    "base_frame_id": f"{n}/base_link",
                    "tf_frame_prefix_enable": False,
                    "enable_odom_tf": True,
                    "open_loop": False,
                    "cmd_vel_timeout": 0.5,
                    "use_stamped_vel": False,
                    "linear.x.has_velocity_limits": True,
                    "linear.x.max_velocity": cfg["max_speed"],
                    "linear.x.min_velocity": -cfg["max_speed"],
                    "linear.x.has_acceleration_limits": True,
                    "linear.x.max_acceleration": 0.5,
                    "angular.z.has_velocity_limits": True,
                    "angular.z.max_velocity": 0.6,
                    "angular.z.min_velocity": -0.6,
                }
            },
        }
        (out / f"{n}_control.yaml").write_text(
            yaml.safe_dump({f"/{n}/{k}": v for k, v in control.items()})
        )
        (out / f"{n}.urdf").write_text(ET.tostring(root, encoding="unicode"))
        (out / f"{n}_nav.yaml").write_text(yaml.safe_dump(nav_config(n, cfg)))
        (out / f"{n}.lua").write_text(
            """include "map_builder.lua"
include "trajectory_builder.lua"
options = {
 map_builder=MAP_BUILDER, trajectory_builder=TRAJECTORY_BUILDER,
 map_frame="N/map",tracking_frame="N/base_link",published_frame="N/odom",odom_frame="N/odom",
 provide_odom_frame=false,publish_frame_projected_to_2d=true,use_pose_extrapolator=true,
 use_odometry=true,use_nav_sat=false,use_landmarks=false,num_laser_scans=1,
 num_multi_echo_laser_scans=0,num_subdivisions_per_laser_scan=1,num_point_clouds=0,
 lookup_transform_timeout_sec=.2,submap_publish_period_sec=.5,pose_publish_period_sec=.05,
 trajectory_publish_period_sec=.1,rangefinder_sampling_ratio=1.,odometry_sampling_ratio=1.,
 fixed_frame_pose_sampling_ratio=1.,imu_sampling_ratio=1.,landmarks_sampling_ratio=1.
}
MAP_BUILDER.use_trajectory_builder_2d=true
MAP_BUILDER.num_background_threads=2
TRAJECTORY_BUILDER_2D.use_imu_data=false
TRAJECTORY_BUILDER_2D.min_range=.18
TRAJECTORY_BUILDER_2D.max_range=12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length=12.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching=true
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds=.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters=.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians=.05
POSE_GRAPH.optimize_every_n_nodes=50
POSE_GRAPH.constraint_builder.min_score=.65
return options
""".replace(
                "N/", n + "/"
            )
        )
    combined = {}
    for r in cfg["robots"]:
        combined.update(
            yaml.safe_load((out / (r["name"] + "_control.yaml")).read_text())
        )
    (out / "controllers.yaml").write_text(yaml.safe_dump(combined))
    return cfg


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", required=True)
    p.add_argument(
        "--config", default=str(Path(share("fleet_bringup")) / "config/fleet.yaml")
    )
    a = p.parse_args()
    prepare(a.output, a.config)
