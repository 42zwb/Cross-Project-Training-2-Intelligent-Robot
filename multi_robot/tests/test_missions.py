import copy
import json
import math
import threading
from concurrent.futures import Future
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rclpy.clock import Clock

from fleet_control.coordinator import Coordinator
from fleet_control.geometry import Grid, formation_pass
from fleet_control.health import scan_health, valid_odom
from fleet_control.patrol import parse_patrol, validate_paths
from fleet_control.safety import Safety
from fleet_mapping.grid import valid_grid_message


NAMES = ["robot1", "robot2", "robot3"]
REGIONS = [[-6, -6, -2, 6], [-2, -6, 2, 6], [2, -6, 6, 6]]
POSES = dict(zip(NAMES, [(-4, -4, 0), (0, -4, 0), (4, -4, 0)]))


def make_plan():
    return {"routes": {"robot1": [[-4.8, -4, 0], [-4, -4, 0]]}, "cycles": 2, "dwell": 0.5}


def test_patrol_validates_paths_and_bounded_cycles():
    grid = Grid(np.zeros((120, 120)), 0.1, (-6, -6))
    plan = parse_patrol(json.dumps(make_plan()), NAMES, REGIONS, grid, POSES)
    validate_paths(plan, NAMES, REGIONS, grid, POSES)
    assert plan["cycles"] == 2
    assert plan["goal_timeout"] == 120


@pytest.mark.parametrize("change", [
    {"cycles": True}, {"cycles": 0}, {"cycles": 21}, {"dwell": -1},
    {"goal_timeout": 0}, {"unexpected": 1}, {"routes": {}},
    {"routes": {"robot4": [[0, 0, 0]]}},
    {"routes": {"robot1": [[float("nan"), -4, 0]]}},
    {"routes": {"robot1": [[True, -4, 0]]}},
    {"routes": {"robot1": [[0, -4, 0]]}},
    {"routes": {"robot1": [[-4, -4]]}},
])
def test_patrol_rejects_invalid_or_cross_region_requests(change):
    data = make_plan()
    data.update(change)
    with pytest.raises(ValueError):
        parse_patrol(json.dumps(data), NAMES, REGIONS, Grid(np.zeros((120, 120)), 0.1), POSES)


def test_patrol_rejects_unknown_obstacle_and_disconnected_legs():
    data = np.zeros((120, 120), dtype=np.int16)
    route = {"routes": {"robot1": [[-4, -2, 0]]}}
    data[30, :] = 100
    grid = Grid(data, 0.1)
    plan = parse_patrol(json.dumps(route), NAMES, REGIONS, grid, POSES)
    with pytest.raises(ValueError, match="不连通"):
        validate_paths(plan, NAMES, REGIONS, grid, POSES)
    for value in [-1, 100]:
        data[40, 20] = value
        with pytest.raises(ValueError):
            parse_patrol(json.dumps(route), NAMES, REGIONS, Grid(data, 0.1), POSES)


@pytest.mark.parametrize("cancelled", [False, True])
def test_late_goal_acceptance_is_cancelled_after_timeout_or_task_switch(cancelled):
    response = Future()
    valid = [True]
    cancellations = []
    handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: cancellations.append(True))
    def wait(*args):
        valid[0] = not cancelled
        return None
    fake = SimpleNamespace(
        lock=threading.RLock(), active=set(), actions={"robot1": SimpleNamespace(send_goal_async=lambda req: response)},
        valid=lambda token: valid[0], get_clock=lambda: Clock(), wait=wait,
    )
    assert Coordinator.send_goal(fake, "robot1", (-4, -3, 0), 1) is None
    response.set_result(handle)
    assert cancellations
    assert not fake.active


def test_repeated_explore_preserves_active_goal_generation():
    fake = SimpleNamespace(event=lambda *args, **kw: None, lock=threading.RLock(),
        ready=True, estop=False, mode="EXPLORING", current_task="explore", generation=5)
    result = Coordinator.command(fake, SimpleNamespace(command="explore"), SimpleNamespace(success=False, message=""))
    assert result.success
    assert fake.generation == 5


def test_map_envelope_rejects_nonfinite_resolution_and_bad_dimensions():
    msg = OccupancyGrid()
    msg.info.width = 2
    msg.info.height = 2
    msg.info.resolution = 0.05
    msg.info.origin.orientation.w = 1.0
    msg.data = [-1, 0, 100, 0]
    assert valid_grid_message(msg)
    for resolution in [0.0, float("nan"), float("inf")]:
        bad = copy.deepcopy(msg)
        bad.info.resolution = resolution
        assert not valid_grid_message(bad)
    msg.data = [0]
    assert not valid_grid_message(msg)


def test_scan_range_limits_and_nonfinite_odom_fail_closed():
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 12.0
    scan.angle_increment = 0.01
    scan.ranges = [1.0] * 40
    assert scan_health(scan) == (True, 1.0)
    for ranges in [[20.0] * 40, [float("nan")] * 40, [-1.0] * 40]:
        scan.ranges = ranges
        assert not scan_health(scan)[0]
    scan.ranges = [1.0] * 40
    scan.angle_increment = float("nan")
    assert not scan_health(scan)[0]
    odom = Odometry()
    assert valid_odom(odom)
    odom.twist.twist.linear.x = float("nan")
    assert not valid_odom(odom)


def test_grid_edge_and_nan_formation_are_unsafe():
    grid = Grid(np.zeros((60, 60)), 0.1, (0, 0))
    assert not grid.allowed()[0, 30]
    assert grid.allowed()[30, 30]
    assert not formation_pass([(float("nan"), 0, 0)] * 3, [(0, 0, 0)] * 3, [0, 1, 2], 0.18)


def test_blind_motion_direction_stops_despite_valid_side_returns():
    import time
    from geometry_msgs.msg import Twist, TransformStamped
    from rclpy.clock import ClockType
    clock = Clock(clock_type=ClockType.ROS_TIME)
    now = time.monotonic()
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 12.0
    scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 640
    scan.ranges = [float("nan") if abs(-math.pi + i * 2 * math.pi / 640) < 0.45 else 1.0 for i in range(640)]
    command = Twist()
    command.linear.x = 0.1
    transform = TransformStamped()
    transform.header.stamp = clock.now().to_msg()
    transform.transform.rotation.w = 1.0
    outputs, reports = [], []
    fake = SimpleNamespace(cfg={"robots":[{"name":"robot1"}]}, lease=now,
        state={"active":["robot1"],"estop":False,"region_enabled":False},
        scans={"robot1":(now,scan)}, odoms={"robot1":(now,Odometry())},
        commands={"robot1":(now,command)}, max_speed=0.2, robot_radius=0.18,
        buffer=SimpleNamespace(lookup_transform=lambda *args: transform), get_clock=lambda: clock,
        pubs={"robot1":SimpleNamespace(publish=outputs.append)},
        status=SimpleNamespace(publish=lambda msg: reports.append(json.loads(msg.data))), last_diagnostics=now)
    assert scan_health(scan)[0]
    Safety.tick(fake)
    assert outputs[0].linear.x == 0
    assert outputs[0].angular.z == 0
    assert reports[0]["robot1"]["reason"] == "运动方向雷达数据无效"


def test_custom_spawn_cannot_overlap_a_wall_or_box():
    import yaml
    from pathlib import Path
    from fleet_bringup.prepare import validate_config
    config = yaml.safe_load((Path(__file__).resolve().parents[1] / "src/fleet_bringup/config/fleet.yaml").read_text())
    assert validate_config(copy.deepcopy(config))["planner"] == "navfn"
    config["robots"][0]["start"] = [-4.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="静态障碍"):
        validate_config(config)


def test_coverage_uses_configured_start_components():
    from fleet_control.evaluate import coverage
    data = np.zeros((5, 5), dtype=np.int16)
    data[:, 2], data[:, 3:] = 100, -1
    boxes = [("wall", 2.5, 2.5, 1.0, 5.0)]
    regions = [[0, 0, 5, 5]]
    left = coverage(data, 1.0, (0, 0), boxes, regions, starts=[(0.5, 0.5)])
    both = coverage(data, 1.0, (0, 0), boxes, regions, starts=[(0.5, 0.5), (4.5, 0.5)])
    assert left["coverage"] == 1.0
    assert both["coverage"] == 0.5


def test_region_start_margin_matches_planner_clearance():
    from fleet_control.geometry import validate_regions
    with pytest.raises(ValueError, match="安全边界"):
        validate_regions(REGIONS, [(-5.5, -4), (0, -4), (4, -4)], [-6, -6, 6, 6])


def test_expired_save_releases_completed_and_pending_clients():
    from fleet_mapping.node import Mapping
    complete, pending = Future(), Future()
    complete.set_result(None)
    clients = [object(), object()]
    destroyed = []
    fake = SimpleNamespace(save_pending={"r": {"deadline": 0, "errors": [],
        "clients": list(zip(clients, [complete, pending]))}}, destroy_client=destroyed.append)
    fake.finish_save = lambda request_id: fake.save_pending.pop(request_id)
    Mapping.expire_saves(fake)
    assert not fake.save_pending
    assert destroyed == clients
    assert pending.cancelled()


def test_shutdown_matches_ruby_process_title_and_exact_world_path(monkeypatch):
    from pathlib import Path
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
    from shutdown_context import simulator_command_matches
    world = "/tmp/中文 目录/assets/arena.sdf".encode()
    assert simulator_command_matches(b"ign\0gazebo\0" + world + b"\0-r\0-s\0", world)
    assert simulator_command_matches(b"ign gazebo " + world + b" -r -s\0\0", world)
    assert not simulator_command_matches(b"ign gazebo " + world + b".other -s\0", world)
    assert not simulator_command_matches(b"other_tool " + world + b" -s\0", world)
