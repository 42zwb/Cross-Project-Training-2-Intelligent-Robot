import math
import numpy as np
import pytest
from fleet_control.geometry import (
    Grid,
    formation,
    formation_pass,
    compose,
    inverse,
    validate_regions,
    plan_formation,
)
from fleet_mapping.grid import fuse, save_map, load_map


def test_frame_calibration():
    start = (-4.0, -4.0, 0.7)
    local = (0.1, -0.2, 0.3)
    align = compose(start, inverse(local))
    assert compose(align, local) == pytest.approx(start)


def test_three_shapes():
    for shape in ["row", "column", "triangle"]:
        pts = formation(shape, 1.0, 2.0, 0.7, 1.2)
        assert np.mean([p[0] for p in pts]) == pytest.approx(1.0)
        assert np.mean([p[1] for p in pts]) == pytest.approx(2.0)
        distances = sorted(
            math.dist(a[:2], b[:2])
            for a, b in [(pts[0], pts[1]), (pts[0], pts[2]), (pts[1], pts[2])]
        )
        assert distances[:2] == pytest.approx([1.2, 1.2])
        if shape == "triangle":
            assert distances == pytest.approx([1.2] * 3)
    with pytest.raises(ValueError):
        formation("triangle", 0, 0, 0, float("nan"))


def test_formation_acceptance_matches_error_and_separation_limits():
    slots = formation("triangle", 0, 0, 0, 1.2)
    poses = [list(point) for point in slots]
    poses[0][0] += 0.18
    assert formation_pass(poses, slots, [0, 1, 2], 0.18)

    poses[0][0] = slots[0][0] + 0.26
    assert not formation_pass(poses, slots, [0, 1, 2], 0.18)

    poses[0] = list(slots[0])
    poses[0][2] = math.radians(16)
    assert not formation_pass(poses, slots, [0, 1, 2], 0.18)

    close_slots = formation("row", 0, 0, 0, 0.7)
    close_poses = [[0, -0.48, 0], [0, 0, 0], [0, 0.48, 0]]
    assert not formation_pass(close_poses, close_slots, [0, 1, 2], 0.18)


def test_regions_cover_overlap_start():
    regions = [[-6, -6, -2, 6], [-2, -6, 2, 6], [2, -6, 6, 6]]
    starts = [(-4, -4), (0, -4), (4, -4)]
    assert validate_regions(regions, starts, [-6, -6, 6, 6])
    with pytest.raises(ValueError):
        validate_regions([[-6, -6, -1, 6], *regions[1:]], starts, [-6, -6, 6, 6])
    with pytest.raises(ValueError):
        validate_regions([[-6, -6, -3, 6], *regions[1:]], starts, [-6, -6, 6, 6])


def test_fusion_conflict_and_unknown():
    a = np.array([[0, 100], [-1, 0]])
    b = np.array([[100, 0], [-1, -1]])
    layer = lambda data: (data, 1.0, 0.0, 0.0, 0.0, (0, 0, 0))
    result = fuse([layer(a), layer(b)], (2, 2), 1.0, (0, 0))
    assert result.tolist() == [[100, 100], [-1, 0]]
    assert np.array_equal(result, fuse([layer(b), layer(a)], (2, 2), 1.0, (0, 0)))


def test_fusion_rotated_origin():
    a = np.array([[100]])
    result = fuse([(a, 1.0, 1.0, 1.0, math.pi / 2, (0, 0, 0))], (3, 3), 1.0, (0, 0))
    assert result[1, 0] == 100


def test_unknown_pgm_roundtrip(tmp_path):
    a = np.array([[-1, 0, 100], [100, -1, 0]], dtype=np.int8)
    save_map(tmp_path / "map", a, 0.05, (-6, -6))
    b, meta = load_map(tmp_path / "map.yaml")
    assert np.array_equal(a, b)
    assert meta["free_thresh"] == 0.196


def test_planning_obstacle_unknown_and_corner_cut():
    a = np.zeros((60, 60))
    a[10:50, 29:31] = 100
    g = Grid(a, 0.1, (0, 0))
    route = g.astar((1, 3), (5, 3), g.allowed())
    assert route and route[0] > 6
    a[:, 29:31] = 100
    g = Grid(a, 0.1, (0, 0))
    assert g.astar((1, 3), (5, 3), g.allowed()) is None
    a[:] = 0
    a[:, 29:31] = -1
    g = Grid(a, 0.1, (0, 0))
    assert g.astar((1, 3), (5, 3), g.allowed()) is None


def test_frontiers_from_observed_map_only():
    a = np.full((120, 120), -1)
    a[10:85, 10:110] = 0
    g = Grid(a, 0.1, (-6, -6))
    r = g.frontiers((-3, -3, 0), [-6, -6, 6, 6])
    assert r is not None
    p = r[1]
    assert g.data[g.cell(p)] == 0


def test_formation_assignment_and_routes():
    a = np.zeros((100, 100))
    g = Grid(a, 0.1, (-5, -5))
    poses = [(-3, -3, 0), (0, -3, 0), (3, -3, 0)]
    slots = formation("triangle", 0, 0, 0, 1.2)
    r = plan_formation(g, poses, slots)
    assert r and len(r[2]) == 3 and sorted(r[1]) == [0, 1, 2]


def test_region_limits_apply_to_full_path():
    a = np.zeros((100, 100))
    g = Grid(a, 0.1, (-5, -5))
    region = [-5, -5, 0, 5]
    route = g.astar((-4, -4), (-1, 4), g.allowed(region))
    assert route
    assert all(-4.65 <= p[0] <= -0.35 and -4.65 <= p[1] <= 4.65 for p in route[1])


def test_other_robot_returns_are_ignored_without_clearing_space():
    from fleet_mapping.grid import mask_robot_returns

    original = np.array([2.0, 3.0, 4.0])
    filtered = mask_robot_returns(
        original, 0.0, math.pi / 2, (1.0, 1.0, 0.0), [(3.0, 1.0, 0.0)]
    )
    assert math.isnan(filtered[0])
    assert filtered[1:].tolist() == [3.0, 4.0]
    assert original.tolist() == [2.0, 3.0, 4.0]
    # Sensor yaw and translation must be applied, not just raw polar coordinates.
    rotated = mask_robot_returns(
        [2.0], 0.0, 1.0, (1.0, 1.0, math.pi / 2), [(1.0, 3.0, 0.0)]
    )
    assert math.isnan(rotated[0])


def test_nav_bond_requires_both_node_and_manager_heartbeats():
    from types import SimpleNamespace
    from fleet_control.coordinator import Coordinator

    names = ["robot1", "robot2", "robot3"]
    roles = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
    records = {
        (name, role, side): 10.0
        for name in names
        for role in roles
        for side in ["node", "manager"]
    }
    fake = SimpleNamespace(names=names, bond_instances=records)
    assert Coordinator.bonds_fresh(fake, 10.5)
    records[("robot1", "bt_navigator", "node")] = 7.0
    assert not Coordinator.bonds_fresh(fake, 10.5)
