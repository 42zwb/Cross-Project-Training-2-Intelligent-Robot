import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid

from fleet_control.geometry import Grid
from fleet_control.coordinator import Coordinator
from fleet_mapping.completion import ObstacleCompletion


def obstacle():
    data = np.zeros((100,100),np.int16)
    data[25:65,25:65] = 100
    data[26:64,26:64] = -1
    return data


def establish(data):
    memory = ObstacleCompletion()
    for revision in range(3):
        memory.update(data,.05,revision,now=revision)
    return memory


def test_unknown_never_expires_confirmed_obstacle_or_measured_edge():
    data = obstacle();memory = establish(data)
    data[:] = -1
    for revision,moment in [(3,3),(4,20),(5,3600),(6,100000)]:
        output,inferred = memory.update(data,.05,revision,now=moment)
        assert output[40,40] == 100 and inferred[40,40]
        assert output[25,40] == 100  # retained direct measurement
    assert memory.retractions == 0


def test_known_occupied_transition_does_not_destroy_inference_memory():
    data = obstacle();memory = establish(data)
    data[26:64,26:64] = 100
    memory.update(data,.05,3,now=3)
    data[26:64,26:64] = -1
    result,_ = memory.update(data,.05,4,now=100)
    assert result[40,40] == 100


def test_stable_free_patch_retracts_object_and_cannot_become_unknown_obstacle_again():
    data = obstacle();memory = establish(data)
    data[40:46,40:46] = 0
    result,_ = memory.update(data,.05,3,now=3)
    assert np.all(result[40:46,40:46] == 0) and result[50,50] == 100
    memory.update(data,.05,4,now=4)
    result,_ = memory.update(data,.05,5,now=5)
    assert result[50,50] == -1 and memory.retractions == 1
    data[:] = -1
    result,_ = memory.update(data,.05,6,now=300)
    assert result[50,50] == -1


def test_repeated_timer_and_one_free_speckle_cannot_retract_object():
    data = obstacle();memory = establish(data)
    data[40:46,40:46] = 0
    for moment in [3,4,50,1000]:
        result,_ = memory.update(data,.05,3,now=moment)
        assert result[50,50] == 100
    data = obstacle();data[40,40] = 0
    for revision in [4,5,6]:
        result,_ = memory.update(data,.05,revision,now=1000+revision)
    assert result[50,50] == 100 and result[40,40] == 0


def test_stable_free_revokes_retained_direct_measurement():
    data = np.zeros((30,30),np.int16);data[15,15] = 100
    memory = establish(data)
    data[15,15] = -1
    assert memory.update(data,.05,3,now=3)[0][15,15] == 100
    data[15,15] = 0
    for revision in range(4,7):memory.update(data,.05,revision,now=revision)
    data[15,15] = -1
    assert memory.update(data,.05,7,now=20)[0][15,15] == -1


def test_geometry_reset_does_not_reuse_old_candidate():
    memory = establish(obstacle())
    output,mask = memory.update(np.full((20,20),-1),.1,None,now=10)
    assert not mask.any() and output.shape == (20,20)


@pytest.mark.parametrize("seed",range(8))
def test_compiled_shortest_routes_match_astar_and_do_not_cut_corners(seed):
    rng = np.random.default_rng(seed)
    data = np.zeros((90,90),np.int16)
    data[20:70,43:47] = 100
    for _ in range(3):
        y,x = rng.integers(10,70,2);data[y:y+5,x:x+5] = 100
    grid = Grid(data,.1,(0,0));start = (.85,.85,0)
    field = grid.route_field(start)
    for target in [(8.15,8.15),(1.15,7.15),(7.15,1.15),(4.55,4.55)]:
        a = grid.astar(start,target,grid.allowed())
        b = grid.field_route(field,target)
        assert (a is None) == (b is None)
        if a:
            assert a[0] == pytest.approx(b[0])
            assert all(grid.data[grid.cell(p)] == 0 for p in b[1])
    assert grid.route_field(start) is field


def fixed_target(grid,target):
    def propose(*args,**kwargs):
        return (1,target,(1,[]),dict(visible_unknown_m2=2.,route_length=1.,viewpoints=1))
    grid.frontiers = propose


def test_global_assignment_chooses_closest_safe_robot_and_records_all_costs():
    grid = Grid(np.zeros((120,120),np.int16),.1,(0,0))
    target = (8.05,8.05);fixed_target(grid,target)
    poses = dict(robot1=(1.05,1.05,0),robot2=(9.05,7.05,0),robot3=(2.05,9.05,0))
    choice = grid.assign_frontier(poses,{n:[] for n in poses},{n:[] for n in poses})
    assert choice["robot"] == "robot2"
    info = choice["result"][3]
    assert info["route_costs_m"]["robot2"] == min(v for v in info["route_costs_m"].values() if v is not None)


def test_euclidean_nearest_robot_can_have_a_longer_safe_detour():
    data = np.zeros((120,120),np.int16);data[:90,59:61] = 100
    grid = Grid(data,.1,(0,0));target = (7.05,3.05);fixed_target(grid,target)
    poses = dict(robot1=(5.05,3.05,0),robot2=(10.05,3.05,0),robot3=(1.05,9.05,0))
    assert math.dist(poses['robot1'][:2],target) < math.dist(poses['robot2'][:2],target)
    choice = grid.assign_frontier(poses,{n:[] for n in poses},{n:[] for n in poses})
    assert choice["robot"] == "robot2"
    assert choice["result"][3]["route_costs_m"]["robot1"] > 12


def test_blacklisted_closest_robot_is_excluded_from_assignment():
    grid = Grid(np.zeros((120,120),np.int16),.1,(0,0));target = (8.05,8.05)
    fixed_target(grid,target)
    poses = dict(robot1=(1.05,1.05,0),robot2=(9.05,7.05,0),robot3=(2.05,9.05,0))
    bans = {n:[] for n in poses};bans['robot2'] = [(*target,0)]
    choice = grid.assign_frontier(poses,bans,{n:[] for n in poses})
    assert choice['robot'] == 'robot3'


def test_identical_map_heartbeat_reuses_grid_and_new_grid_invalidates_it():
    msg = OccupancyGrid();msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 30;msg.info.resolution = .1
    msg.info.origin.orientation.w = 1.;msg.data = [0]*900
    fake = SimpleNamespace(grid=None,lock=threading.RLock(),cfg=dict(robot_radius=.18,region_margin=.35))
    Coordinator.map_cb(fake,msg);first = fake.grid
    Coordinator.map_cb(fake,msg);assert fake.grid is first
    msg.data[400] = 100
    Coordinator.map_cb(fake,msg);assert fake.grid is not first
