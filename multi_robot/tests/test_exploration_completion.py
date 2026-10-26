import threading
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from fleet_control.geometry import Grid
from fleet_control.coordinator import Coordinator
from fleet_mapping.completion import ObstacleCompletion, enclosed_obstacles, map_metrics


def closed_box():
    data = np.zeros((100, 100), dtype=np.int16)
    data[30:61, 30:61] = 100
    data[31:60, 31:60] = -1
    return data


def test_complete_only_closed_unknown_solid_interior():
    data = closed_box()
    original = data.copy()
    mask = enclosed_obstacles(data, 0.05)
    assert mask[40, 40]
    assert np.all(data[mask] < 0)
    assert np.array_equal(original, data)


@pytest.mark.parametrize("opening", [0, -1])
def test_open_contour_and_free_doorway_are_not_filled(opening):
    data = closed_box()
    data[30, 40:48] = opening
    assert not enclosed_obstacles(data, 0.05).any()


def test_measured_free_contour_pixels_are_never_overwritten():
    data = closed_box()
    data[30, 40] = 0
    mask = enclosed_obstacles(data, 0.05)
    assert not mask[30, 40]
    assert np.all(data[mask] < 0)


def test_room_with_observed_free_interior_is_not_solid():
    data = closed_box()
    data[43:48, 43:48] = 0
    assert not enclosed_obstacles(data, 0.05).any()


def test_large_enclosed_arena_is_not_inferred_as_an_obstacle():
    data = np.full((150, 150), -1)
    data[[0, -1], :] = 100
    data[:, [0, -1]] = 100
    assert not enclosed_obstacles(data, 0.1).any()


def test_moderate_contour_can_connect_a_strongly_supported_obstacle():
    raw = closed_box()
    boundary = raw.copy()
    raw[30:61, 30] = -1  # moderate edge lost by trinary fusion
    assert not enclosed_obstacles(raw, 0.05).any()
    mask = enclosed_obstacles(raw, 0.05, boundary=boundary)
    assert mask[40, 40]
    assert np.all(raw[mask] < 0)


def test_moderate_contour_alone_cannot_establish_a_solid_obstacle():
    boundary = closed_box()
    raw = boundary.copy()
    raw[raw >= 65] = -1
    assert not enclosed_obstacles(raw, 0.05, boundary=boundary).any()


def test_moderate_evidence_cannot_hide_observed_free_interior():
    boundary = closed_box()
    raw = boundary.copy()
    raw[42:48, 42:48] = 0
    assert not enclosed_obstacles(raw, 0.05, boundary=boundary).any()


def test_moderate_evidence_cannot_hide_a_measured_passable_doorway():
    boundary = closed_box()
    raw = boundary.copy()
    raw[30, 40:48] = 0
    assert not enclosed_obstacles(raw, 0.05, boundary=boundary).any()


def test_completion_needs_multiple_new_updates_and_time_then_retracts():
    data = closed_box()
    completion = ObstacleCompletion()
    for moment in [0, 1, 2, 3]:
        result, mask = completion.update(data, 0.05, evidence=1, now=moment)
        assert not mask.any()  # repeated timer calls are not new observations
    completion.update(data, 0.05, evidence=2, now=3.5)
    result, mask = completion.update(data, 0.05, evidence=3, now=4)
    assert mask[40, 40] and result[40, 40] == 100
    data[40, 40] = 0
    result, mask = completion.update(data, 0.05, evidence=4, now=4.5)
    assert not mask.any() and result[40, 40] == 0


def test_inference_raises_known_fraction_without_claiming_direct_observation():
    data = closed_box()
    mask = enclosed_obstacles(data, 0.05)
    metrics = map_metrics(data, mask)
    assert metrics["known_fraction"] > metrics["observed_fraction"]
    assert metrics["known_fraction"] == pytest.approx(
        metrics["observed_fraction"] + metrics["inferred_obstacle_fraction"])
    assert metrics["known_fraction"] + metrics["unknown_fraction"] == pytest.approx(1)


def test_confirmed_inference_survives_brief_missing_contour_then_expires():
    data = closed_box()
    completion = ObstacleCompletion()
    for evidence, moment in enumerate([0, 1, 2]):
        result, mask = completion.update(data, 0.05, evidence=evidence, now=moment)
    assert mask[40, 40]
    data[30, 40:48] = -1
    result, mask = completion.update(data, 0.05, evidence=3, now=3)
    assert mask[40, 40]
    result, mask = completion.update(data, 0.05, evidence=4, now=8)
    assert not mask.any()


def test_unknown_behind_a_wall_provides_no_visible_information_gain():
    data = np.zeros((100, 100), dtype=np.int16)
    data[:, 50] = 100
    data[:, 51:] = -1
    grid = Grid(data, 0.1, (0, 0))
    assert grid.visible_unknown(grid.cell((4, 5))) == 0
    assert grid.frontiers((2, 5, 0), None) is None


def test_small_start_corner_is_selected_instead_of_large_unreachable_unknown():
    data = np.zeros((100, 100), dtype=np.int16)
    data[:, 60] = 100
    data[70:, 65:] = -1
    data[3:12, 3:12] = -1
    grid = Grid(data, 0.1, (0, 0))
    result = grid.frontiers((3, 3, 0), None)
    assert result is not None
    assert result[1][0] < 3 and result[1][1] < 3
    assert result[3]["visible_unknown_m2"] > 0
    assert all(grid.data[grid.cell(point)] == 0 for point in result[2][1])


def test_thin_frontier_not_aligned_to_sampling_lattice_is_retained():
    data = np.zeros((80, 80), dtype=np.int16)
    data[50:, :] = -1
    grid = Grid(data, 0.1, (0, 0))
    mask = np.zeros(data.shape, bool)
    mask[43, 41] = True
    result = grid.frontiers((2, 2, 0), None, target_mask=mask)
    assert result is not None
    assert grid.cell(result[1]) == (43, 41)


def test_failed_target_becomes_available_after_cooldown(monkeypatch):
    data = np.zeros((80, 80), dtype=np.int16)
    data[50:, :] = -1
    grid = Grid(data, 0.1, (0, 0))
    mask = np.zeros(data.shape, bool)
    mask[43, 41] = True
    target = grid.point((43, 41)) + (0, 100)
    monkeypatch.setattr("fleet_control.geometry.time.monotonic", lambda: 99)
    assert grid.frontiers((2, 2, 0), None, blacklist=[target], target_mask=mask) is None
    monkeypatch.setattr("fleet_control.geometry.time.monotonic", lambda: 101)
    assert grid.frontiers((2, 2, 0), None, blacklist=[target], target_mask=mask) is not None


def test_cooldown_is_not_mistaken_for_exploration_complete(monkeypatch):
    monkeypatch.setattr("fleet_control.coordinator.time.monotonic", lambda: 10)
    target = (2, 2, 0, 50)
    def frontiers(*args, **kwargs):
        return None if args[3] else (1, (2, 2), (1, []))
    fake = SimpleNamespace(names=["robot1"], valid=lambda token: True,
        grid=SimpleNamespace(frontiers=frontiers), poses={"robot1": (1, 1, 0)},
        regions=[None], exploration_unrestricted=True, cross_region_next=0,
        exploring={}, goals={}, handles={}, lock=threading.RLock(), active=set(),
        blacklist={"robot1": [target]}, visits={"robot1": []}, travel={"robot1": 5},
        explore_done={"robot1": False}, region_stall_since={}, mode="EXPLORING")
    Coordinator.explore_tick(fake, 1)
    assert fake.mode == "EXPLORING"
    assert "重试" in fake.message


def test_nav2_plan_display_requires_active_matching_goal_and_finite_map_path():
    fake = SimpleNamespace(goals={"robot1": (2, 3, 0)}, nav_plans={}, lock=threading.RLock())
    message = Path()
    message.header.frame_id = "map"
    point = PoseStamped()
    point.pose.position.x, point.pose.position.y = 2.0, 3.0
    message.poses = [point]
    Coordinator.plan_cb(fake, "robot1", message)
    assert fake.nav_plans["robot1"][-1] == (2, 3)
    fake.nav_plans.clear()
    point.pose.position.x = float("nan")
    Coordinator.plan_cb(fake, "robot1", message)
    assert not fake.nav_plans


def test_exploration_completion_requires_a_stable_frontier_recheck(monkeypatch):
    moment = [10.0]
    monkeypatch.setattr("fleet_control.coordinator.time.monotonic", lambda: moment[0])
    fake = SimpleNamespace(names=["robot1"], valid=lambda token: True,
        grid=SimpleNamespace(frontiers=lambda *a: None, region_escape=lambda *a: None),
        poses={"robot1": (1, 1, 0)}, regions=[None], exploration_unrestricted=False,
        exploring={}, goals={}, handles={}, lock=threading.RLock(), active=set(),
        blacklist={"robot1": []}, visits={"robot1": []}, travel={"robot1": 5},
        explore_done={"robot1": False}, region_stall_since={}, mode="EXPLORING",
        last_boundary_check=10, boundary_blocked_since={}, event=lambda *a, **kw: None)
    Coordinator.explore_tick(fake, 1)
    moment[0] = 18.1
    Coordinator.explore_tick(fake, 1)
    assert fake.mode == "EXPLORING" and "复查" in fake.message
    moment[0] = 28.2
    Coordinator.explore_tick(fake, 1)
    assert fake.mode == "EXPLORED"
