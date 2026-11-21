import math
import cv2
import numpy as np
import pytest
from fleet_mapping.registration import (ConstraintGraph, Snapshot, angle, compose,
                                        dynamic_fusion, inverse, register, voronoi_mask)


def observed_grid():
    data = np.zeros((300, 300), np.int16)
    data[:4] = data[-4:] = 100
    data[:, :4] = data[:, -4:] = 100
    cv2.rectangle(data, (34, 39), (69, 87), 100, -1)
    cv2.rectangle(data, (83, 140), (128, 169), 100, -1)
    cv2.circle(data, (229, 87), 23, 100, -1)
    cv2.fillPoly(data, [np.array([[165, 184], [180, 182], [205, 242], [167, 231]])], 100)
    cv2.rectangle(data, (210, 200), (260, 214), 100, -1)
    cv2.rectangle(data, (245, 210), (260, 268), 100, -1)
    return data


@pytest.mark.parametrize('yaw,shift', [(23, (21, -17)), (-57, (-5, 12)), (137, (11, 3))])
def test_unknown_translation_rotation_with_observed_overlap(yaw, shift):
    a = observed_grid()
    matrix = cv2.getRotationMatrix2D((150, 150), yaw, 1)
    matrix[:, 2] += shift
    b = cv2.warpAffine(a, matrix, (300, 300), flags=cv2.INTER_NEAREST, borderValue=-1)
    a[:, 270:] = -1
    b[:, :20] = -1
    source = Snapshot('robot1', 'b1', 1, 1, a, .05, (-2, -1, 0))
    target = Snapshot('robot2', 'b2', 1, 1, b, .05, (3, -4, 0))
    match = register(source, target)
    assert match is not None
    # Affine image coordinates have the same index handedness as OccupancyGrid.
    theta = math.atan2(matrix[1, 0], matrix[0, 0])
    b_from_a = compose((3, -4, 0), compose((matrix[0, 2]*.05, matrix[1, 2]*.05, theta),
                                       inverse((-2, -1, 0))))
    truth = inverse(b_from_a)
    assert math.dist(match.transform[:2], truth[:2]) < .10
    assert abs(angle(match.transform[2]-truth[2])) < math.radians(2)


def test_empty_or_symmetric_map_cannot_initialize_team():
    empty = np.full((120, 120), -1, np.int16)
    a = Snapshot('robot1', 'a', 1, 1, empty, .05, (0, 0, 0))
    assert register(a, a) is None
    square = np.zeros((200, 200), np.int16)
    square[:4] = square[-4:] = 100
    square[:, :4] = square[:, -4:] = 100
    b = Snapshot('robot2', 'b', 1, 1, square, .05, (0, 0, 0))
    assert register(b, b) is None


@pytest.mark.parametrize('alteration', ['mirror', 'scale', 'unrelated'])
def test_rigid_registration_rejects_incompatible_geometry(alteration):
    a = observed_grid()
    if alteration == 'mirror':
        b = np.fliplr(a).copy()
    elif alteration == 'scale':
        matrix = cv2.getRotationMatrix2D((150, 150), 17, 1.3)
        b = cv2.warpAffine(a, matrix, (300, 300), flags=cv2.INTER_NEAREST, borderValue=-1)
    else:
        b = np.zeros_like(a)
        for x, y in ((61, 65), (118, 214), (232, 193), (165, 80)):
            cv2.circle(b, (x, y), 12, 100, -1)
    source = Snapshot('robot1', 'a', 1, 1, a, .05, (0, 0, 0))
    target = Snapshot('robot2', 'b', 1, 1, b, .05, (0, 0, 0))
    assert register(source, target) is None


def test_graph_rejects_inconsistent_cycle_and_preserves_anchor():
    graph = ConstraintGraph()
    a, b, c = ('robot1', 'a', 1), ('robot2', 'b', 1), ('robot3', 'c', 1)
    assert graph.add(a, b, (4, 0, .1))
    assert graph.add(b, c, (2, 1, -.2))
    assert not graph.add(a, c, (0, 0, 0))
    assert graph.add(a, c, compose((4, 0, .1), (2, 1, -.2)))
    old_anchor = graph.anchor
    graph.poses(b)
    assert graph.anchor == old_anchor


def test_fusion_uses_observed_extent_without_world_spawn_information():
    data = np.full((20, 30), -1, np.int16)
    data[3:17, 4:25] = 0
    data[8:11, 12:16] = 100
    layer = (data, .05, 0., 0., 0., (10., -30., .4))
    combined, origin = dynamic_fusion([layer], .05)
    assert origin[0] > 8 and origin[1] < -29
    assert np.any(combined == 100) and np.any(combined < 0)
    assert np.array_equal(dynamic_fusion([layer, layer], .05)[0], combined)


def test_voronoi_regions_are_disjoint_with_a_clearance_gap():
    seeds = {'robot1': (-2, 0), 'robot2': (0, 0), 'robot3': (2, 0)}
    masks = [voronoi_mask((100, 120), .05, (-3, -2.5), seeds, n) for n in seeds]
    assert np.max(np.sum(masks, axis=0)) <= 1
    assert all(np.any(m) for m in masks)
