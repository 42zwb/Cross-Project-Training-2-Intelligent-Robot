import copy
import json
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from fleet_bringup.prepare import validate_config, world
from fleet_bringup.scene import generate_scene, navigation_space, box_gap
from fleet_control.geometry import Grid
from fleet_control.coordinator import Coordinator


STARTS = [(-5, -5, 0), (0, -5, 0), (5, -5, 0)]


def test_scene_seed_and_export_are_reproducible(tmp_path):
    geometry, metadata = generate_scene(42, STARTS)
    again, meta_again = generate_scene(42, STARTS)
    assert geometry == again and metadata == meta_again
    assert geometry != generate_scene(43, STARTS)[0]
    first, second = tmp_path / "first", tmp_path / "second"
    first.mkdir(); second.mkdir()
    world(first, geometry, metadata); world(second, again, meta_again)
    for name in ["arena.sdf", "ground_truth.yaml", "scene.json"]:
        assert (first / name).read_bytes() == (second / name).read_bytes()
    assert json.loads((first / "scene.json").read_text())["seed"] == 42


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 7, 42, 99, 2026, 12345, 2**63-1])
def test_scene_has_large_separated_obstacles_and_connected_navigation(seed):
    geometry, metadata = generate_scene(seed, STARTS, generator="rectangles-v1")
    assert len(geometry) == 14
    assert navigation_space(geometry, STARTS)["connected"]
    assert metadata["navigation_check"]["free_fraction"] >= 0.5
    for i, box in enumerate(geometry[4:]):
        assert min(box[3:]) >= 1.0
        assert all(box_gap(box, other) >= 1.2 for other in geometry[:i+4])
        assert all(box_gap(box, ("start", p[0], p[1], 1.8, 1.8)) >= 0.1 for p in STARTS)


@pytest.mark.parametrize("seed", [True, -1, 2**63, "42", float("nan")])
def test_scene_rejects_invalid_seed(seed):
    with pytest.raises(ValueError):
        generate_scene(seed, STARTS)


def test_random_config_validates_size_seed_and_spawn():
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "src/fleet_bringup/config/fleet.yaml").read_text())
    assert validate_config(copy.deepcopy(cfg))["arena"] == [-7.5,-7.5,7.5,7.5]
    for change in [{"arena": [-6,-6,6,6]}, {"scene": {"seed": True}}, {"scene": {"obstacle_count": 100}}]:
        with pytest.raises(ValueError):
            validate_config(copy.deepcopy(cfg) | change)


def test_region_escape_requires_safe_observed_path():
    data = np.zeros((150,150), dtype=np.int16)
    # An obstacle spans the left task rectangle. Reaching its upper frontier
    # requires a detour through the middle task rectangle, which is safe globally.
    data[70:85, :55] = 100
    data[120:, :50] = -1
    grid = Grid(data, 0.1, (-7.5,-7.5))
    region = [-7.5,-7.5,-2.5,7.5]
    assert grid.frontiers((-5,-5,0), region) is None
    escape = grid.region_escape((-5,-5,0), region)
    assert escape is not None
    assert all(grid.data[grid.cell(p)] == 0 for p in escape[2][1])
    data[:, 55:65] = 100
    assert Grid(data, 0.1, (-7.5,-7.5)).region_escape((-5,-5,0), region) is None


def test_region_release_stops_before_changing_filters_and_records_reason():
    calls = []
    fake = SimpleNamespace(names=["robot1","robot2","robot3"], lock=threading.RLock(),
        stopped=lambda token: calls.append("stop") or True,
        filters=lambda enabled, token: calls.append(("filter", enabled)), valid=lambda token: True,
        event=lambda event, **kw: calls.append(event))
    Coordinator.release_exploration_regions(fake, 1, "robot1", (-1,2))
    assert calls == ["stop", ("filter", False), "exploration_regions_released"]
    assert fake.exploration_unrestricted
    assert fake.exploring == {}
    assert "robot1" in fake.region_release_reason


def test_region_release_waits_for_persistent_blockage(monkeypatch):
    now, releases = [10.0], []
    monkeypatch.setattr("fleet_control.coordinator.time.monotonic", lambda: now[0])
    fake = SimpleNamespace(names=["robot1"], valid=lambda token: True,
        grid=SimpleNamespace(frontiers=lambda *a: None, region_escape=lambda *a: (1, (0,2), (1,[]))),
        poses={"robot1":(-5,-5,0)}, regions=[[-7.5,-7.5,-2.5,7.5]],
        exploring={}, goals={}, lock=threading.RLock(), exploration_unrestricted=False,
        blacklist={"robot1":[]}, visits={"robot1":[]}, travel={"robot1":4.0},
        explore_done={"robot1":False}, region_stall_since={},
        last_boundary_check=10.0, boundary_blocked_since={},
        release_exploration_regions=lambda *args: releases.append(args))
    for moment in [10.0, 17.9]:
        now[0] = moment
        Coordinator.explore_tick(fake, 1)
        assert not releases
    now[0] = 18.0
    Coordinator.explore_tick(fake, 1)
    assert releases == [(1,"robot1",(0,2))]


def test_region_release_detects_detour_while_local_goal_is_still_active(monkeypatch):
    now, releases = [10.0], []
    monkeypatch.setattr("fleet_control.coordinator.time.monotonic", lambda: now[0])
    fake = SimpleNamespace(names=["robot1"], valid=lambda token: True,
        grid=SimpleNamespace(region_escape=lambda *a: (1,(0,2),(1,[]))),
        poses={"robot1":(-5,-5,0)}, regions=[[-7.5,-7.5,-2.5,7.5]],
        exploring={"robot1":(Future(),0,(-5,-1,0))}, goals={"robot1":(-5,-1,0)},
        lock=threading.RLock(), exploration_unrestricted=False,
        blacklist={"robot1":[]}, visits={"robot1":[]}, explore_done={"robot1":False},
        last_boundary_check=0.0, boundary_blocked_since={},
        release_exploration_regions=lambda *args: releases.append(args))
    Coordinator.explore_tick(fake,1)
    assert not releases
    now[0]=22.0
    Coordinator.explore_tick(fake,1)
    assert releases == [(1,"robot1",(0,2))]


def test_region_release_does_not_disable_filter_if_motion_cannot_stop():
    changes=[]
    fake=SimpleNamespace(stopped=lambda token: False, valid=lambda token: True,
                         filters=lambda *args: changes.append(args))
    with pytest.raises(RuntimeError,match="停车"):
        Coordinator.release_exploration_regions(fake,1,"robot1",(0,2))
    assert not changes


def test_cross_region_goals_are_serialized_and_rotate_between_robots():
    names=["robot1","robot2","robot3"]
    sent, blocked_poses, futures=[],[],[]
    def frontiers(*args,**kwargs):
        blocked_poses.append(kwargs["others"])
        return (1,(0,2),(1,[]))
    fake=SimpleNamespace(names=names, valid=lambda token:True, grid=SimpleNamespace(frontiers=frontiers),
        lock=threading.RLock(), exploration_unrestricted=True, cross_region_next=0,
        poses=dict(zip(names,STARTS)), regions=[None]*3, exploring={},goals={},handles={},active=set(),
        blacklist={n:[] for n in names},visits={n:[] for n in names},travel={n:4 for n in names},
        region_stall_since={},explore_done={n:False for n in names},event=lambda *a,**kw:None)
    def send(name,target,token):
        sent.append(name);future=Future();futures.append(future);return future
    fake.send_goal=send
    Coordinator.explore_tick(fake,1)
    Coordinator.explore_tick(fake,1)
    assert sent==["robot1"] and len(fake.exploring)==1
    assert blocked_poses[0]==STARTS[1:]
    futures[0].set_result(SimpleNamespace(status=4))
    Coordinator.explore_tick(fake,1)
    Coordinator.explore_tick(fake,1)
    assert sent==["robot1","robot2"] and len(fake.exploring)==1


def test_cross_region_route_stays_clear_of_parked_robots():
    data=np.zeros((150,150),dtype=np.int16);data[120:,:]=-1
    grid=Grid(data,.1,(-7.5,-7.5))
    other=(-5,0,0)
    result=grid.frontiers((-5,-5,0),None,others=[other])
    assert result
    assert all(np.linalg.norm(np.array(point)-np.array(other[:2]))>0.8 for point in result[2][1])
