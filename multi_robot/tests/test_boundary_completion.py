import numpy as np
import pytest

from fleet_mapping.completion import ObstacleCompletion, boundary_wall_candidates, map_metrics


def box_map(size=100, depth=4):
    data = np.zeros((size,size),np.int16)
    data[:depth,:] = data[-depth:,:] = -1
    data[:,:depth] = data[:,-depth:] = -1
    data[depth,depth:-depth] = data[-depth-1,depth:-depth] = 100
    data[depth:-depth,depth] = data[depth:-depth,-depth-1] = 100
    return data


def mature(data,boundary=None):
    memory = ObstacleCompletion()
    for revision in range(3):
        output,inferred = memory.update(data,.05,revision,now=revision,boundary=boundary)
    return memory,output,inferred


def test_confirmed_perimeter_is_filled_to_all_grid_edges_and_corners():
    data = box_map();original = data.copy()
    model,output,inferred = mature(data)
    assert np.all(output[data<0] == 100)
    assert output[0,0] == output[-1,-1] == 100
    assert np.array_equal(data,original)
    assert np.array_equal(output[data>=0],data[data>=0])
    assert model.boundary_inferred.sum() == (data<0).sum()
    metrics = map_metrics(data,inferred,boundary=model.boundary_inferred)
    assert metrics['known_fraction'] == 1


@pytest.mark.parametrize('side',range(4))
def test_doorway_and_measured_free_path_to_edge_remain_open(side):
    data = box_map();data[:5,40:52] = 0
    data = np.rot90(data,side)
    model,output,_ = mature(data)
    assert np.array_equal(output[data==0],data[data==0])
    unknown_door = box_map();unknown_door[:5,40:52] = -1
    unknown_door = np.rot90(unknown_door,side)
    _,output,_ = mature(unknown_door)
    probe = np.rot90(np.indices(data.shape)[0]<5,side) & np.rot90((np.indices(data.shape)[1]>=40)&(np.indices(data.shape)[1]<52),side)
    assert np.all(output[probe] == -1)


def test_inner_wall_is_not_projected_halfway_across_the_map():
    data = np.full((100,100),-1,np.int16)
    data[45:55,:] = 100;data[55:,:] = 0
    assert not boundary_wall_candidates(data,.05).any()


def test_short_edge_object_and_weak_only_contour_are_not_enough():
    data = np.zeros((100,100),np.int16);data[:4,:] = -1;data[4,40:45] = 100
    assert not boundary_wall_candidates(data,.05).any()
    contour = box_map();raw = contour.copy();raw[raw>=65] = -1
    assert not boundary_wall_candidates(raw,.05,contour).any()


def test_unobserved_boundary_without_wall_evidence_is_never_filled():
    data = np.full((100,100),-1,np.int16)
    assert not boundary_wall_candidates(data,.05).any()
    data[[0,-1],:] = data[:,[0,-1]] = 100
    # No measured free interior side: these could be unrelated occupied points.
    assert not boundary_wall_candidates(data,.05).any()


def test_small_unknown_raster_crack_can_be_bridged_but_free_crack_is_not():
    data = box_map();data[4,40:42] = -1
    _,output,_ = mature(data)
    assert np.all(output[:5,40:42] == 100)
    data = box_map();data[:5,40:42] = 0
    _,output,_ = mature(data)
    assert np.all(output[:5,40:42] == 0)


def test_boundary_completion_requires_new_updates_and_preserves_memory():
    data = box_map();model = ObstacleCompletion()
    for moment in [0,1,2,10]:
        output,_ = model.update(data,.05,1,now=moment)
        assert output[0,50] == -1
    model.update(data,.05,2,now=11)
    output,_ = model.update(data,.05,3,now=12)
    assert output[0,50] == 100
    data[4,:] = -1
    output,_ = model.update(data,.05,4,now=3600)
    assert output[0,50] == 100


def test_stable_free_doorway_retracts_only_cells_with_positive_free_evidence():
    data = box_map();model,_,_ = mature(data)
    data[3:5,40:52] = 0
    for revision in [3,4,5]:output,_ = model.update(data,.05,revision,now=revision)
    assert np.all(output[:3,40:52] == 100)
    assert np.all(output[3:5,40:52] == 0)
    data[:5,40:52] = 0
    for revision in [6,7,8]:output,_ = model.update(data,.05,revision,now=revision)
    data[:5,40:52] = -1
    output,_ = model.update(data,.05,9,now=20)
    assert np.all(output[:5,40:52] == -1)
    assert output[0,25] == output[0,75] == 100
    assert model.boundary_retractions > 0


@pytest.mark.parametrize('side',range(4))
def test_refined_wall_face_does_not_erase_still_confirmed_outer_prefix(side):
    data = box_map();model,_,_ = mature(data)
    data[4,10:90] = 0
    data[3,10:90] = 100
    outer = np.zeros(data.shape,bool);outer[:3,20:80] = True
    free = np.zeros(data.shape,bool);free[4,20:80] = True
    data,outer,free = [np.rot90(a,side) for a in [data,outer,free]]
    for revision in [3,4,5,6]:
        output,_ = model.update(data,.05,revision,now=revision)
        assert np.all(output[outer] == 100)
        assert np.all(output[free] == 0)


@pytest.mark.parametrize('resolution',[.025,.05,.1])
def test_completion_uses_metric_limits_at_different_resolutions(resolution):
    n = int(5/resolution);depth = int(.2/resolution)
    data = box_map(n,depth)
    assert boundary_wall_candidates(data,resolution).any()
    inferred = boundary_wall_candidates(data,resolution).any(axis=0)
    assert not np.any(inferred & (data==0))
