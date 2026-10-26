import collections
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy import ndimage

from fleet_bringup.meshes import write_prism
from fleet_bringup.prepare import world
from fleet_bringup.scene import generate_scene, navigation_space
from fleet_mapping.shapes import (bounds, is_convex, local_vertices, polygon_area,
                                  raycast, shape_distance, shape_gap, triangulate, vertices)
from fleet_mapping.completion import enclosed_obstacles


STARTS = [(-5, -5, 0), (0, -5, 0), (5, -5, 0)]
L = np.array([[0, 0], [2, 0], [2, .7], [.7, .7], [.7, 2], [0, 2]])


def shape(kind="concave_polygon", yaw=0, **kw):
    return dict(name="test", kind=kind, x=0., y=0., yaw=yaw, vertices=L.tolist()) | kw


@pytest.fixture(scope="module")
def scene():
    return generate_scene(24, STARTS)


def test_l_shape_notch_and_rotation_are_preserved():
    original = shape()
    assert shape_distance(.3, 1.5, original) == 0
    assert shape_distance(1.5, 1.5, original) == pytest.approx(.8)
    rotated = shape(yaw=math.pi/2)
    assert shape_distance(-1.5, .3, rotated) == 0
    assert shape_distance(-1.5, 1.5, rotated) == pytest.approx(.8)
    assert bounds(rotated) == pytest.approx((-2, 0, 0, 2))


@pytest.mark.parametrize("yaw", [0, .3, 1.6, -2.8])
def test_concave_triangulation_has_correct_union_and_area(yaw):
    points = vertices(shape(yaw=yaw))
    triangles = [points[t] for t in triangulate(points)]
    assert len(triangles) == len(points)-2
    assert all(polygon_area(t) > 0 for t in triangles)
    assert sum(polygon_area(t) for t in triangles) == pytest.approx(polygon_area(points))
    yy, xx = np.mgrid[-3:3:.07, -3:3:.07]
    body = shape_distance(xx, yy, shape(yaw=yaw)) == 0
    pieces = np.zeros(body.shape, bool)
    for triangle in triangles:
        pieces |= shape_distance(xx, yy, shape(vertices=triangle.tolist(), yaw=0)) == 0
    assert np.array_equal(body, pieces)


def test_crossed_rectangles_and_collinear_disjoint_edges():
    a = shape("rectangle", size=[4, .2])
    b = shape("rectangle", yaw=math.pi/2, size=[4, .2])
    assert shape_gap(a, b) == 0
    b.update(yaw=0, x=5)
    assert shape_gap(a, b) == pytest.approx(1)


def test_circle_and_polygon_distances():
    circle = shape("circle", radius=.25, x=1.5, y=1.5)
    assert shape_gap(circle, shape()) == pytest.approx(.55)
    assert shape_gap(shape(), circle) == pytest.approx(.55)
    assert shape_distance([1.5, 2.5], [1.5, 1.5], circle).tolist() == [0., .75]


def test_rays_see_the_concave_notch_and_analytic_circle():
    ranges, hits = raycast([1.5, 1.5], [0, math.pi, -math.pi/2], [shape()], 12)
    assert ranges == pytest.approx([12, .8, .8])
    assert hits.tolist() == [-1, 0, 0]
    ranges, _ = raycast([0, 0], [0, math.pi/2], [shape("circle", radius=1, x=3)])
    assert ranges == pytest.approx([2, 12])


@pytest.mark.parametrize("points", [[], [[0,0],[1,1]], [[0,0],[1,1],[2,2]],
                                    [[0,0],[2,2],[0,2],[2,0]], [[0,0],[1,float("nan")],[2,0]]])
def test_invalid_polygon_is_rejected(points):
    with pytest.raises(ValueError):
        triangulate(points)


@pytest.mark.parametrize("clockwise", [False, True])
def test_stl_is_closed_outward_and_has_expected_volume(tmp_path, clockwise):
    points = L[::-1] if clockwise else L
    path = tmp_path / "test.stl"
    write_prism(path, points)
    coords = [[float(v) for v in line.split()[1:]] for line in path.read_text().splitlines()
              if line.strip().startswith("vertex ")]
    facets = np.array(coords).reshape(-1, 3, 3)
    edges = collections.Counter()
    for facet in facets:
        for a, b in zip(facet, np.roll(facet, -1, axis=0)):
            edges[(tuple(a), tuple(b))] += 1
    assert all(count == 1 and edges[(b, a)] == 1 for (a, b), count in edges.items())
    volume = sum(np.dot(a, np.cross(b, c))/6 for a, b, c in facets)
    assert volume == pytest.approx(abs(polygon_area(points))*1.2)


def test_mixed_scene_contains_all_kinds_rotated_safe_and_connected(scene):
    geometry, meta = scene
    assert meta["generator"] == "mixed-shapes-v2"
    assert set(meta["shape_counts"].values()) == {2}
    assert navigation_space(geometry, STARTS)["connected"]
    for i, obstacle in enumerate(geometry[4:]):
        assert abs(math.sin(obstacle["yaw"]*2)) > .01
        assert all(shape_gap(obstacle, other) >= 1.2-1e-6 for other in geometry[:i+4])
        if obstacle["kind"] == "concave_polygon":
            assert not is_convex(local_vertices(obstacle))
            assert obstacle["minimum_arm_width"] >= .65
        if obstacle["kind"] == "convex_polygon":
            assert is_convex(local_vertices(obstacle))


def test_export_meshes_and_sdf_are_reproducible_and_concave_collisions_split(tmp_path, scene):
    first, second = tmp_path/"a", tmp_path/"b"
    first.mkdir(); second.mkdir()
    world(first, *scene); world(second, *scene)
    files = sorted(p.relative_to(first) for p in first.rglob("*") if p.is_file())
    assert len(files) > 10
    for relative in files:
        assert (first/relative).read_bytes() == (second/relative).read_bytes()
    root = ET.parse(first/"arena.sdf").getroot()
    for obstacle in scene[0][4:]:
        model = root.find(f".//model[@name='{obstacle['name']}']")
        collisions = model.findall("./link/collision")
        if obstacle["kind"] == "concave_polygon":
            assert len(collisions) == len(obstacle["vertices"])-2
        if obstacle["kind"] == "circle":
            assert collisions[0].find("./geometry/cylinder") is not None
    assert all((first/uri.text).is_file() for uri in root.findall(".//mesh/uri"))


@pytest.mark.parametrize("yaw", [0, .3, 1.2, -2.7])
def test_solid_concave_completion_preserves_notch_and_free_observations(yaw):
    yy, xx = np.mgrid[-3:3:.05, -3:3:.05]
    body = shape_distance(xx, yy, shape(yaw=yaw)) < 1e-9
    interior = ndimage.binary_erosion(body)
    data = np.zeros(body.shape, np.int16)
    data[body] = 100
    data[interior] = -1
    inferred = enclosed_obstacles(data, .05, boundary=data)
    assert inferred.sum() > .7*interior.sum()
    assert not np.any(inferred & ~body)
    assert np.all(data[inferred] < 0)


def test_concave_free_door_and_observed_interior_are_not_filled():
    yy, xx = np.mgrid[-1:3:.05, -1:3:.05]
    body = shape_distance(xx, yy, shape()) < 1e-9
    data = np.zeros(body.shape, np.int16)
    data[body] = 100
    data[ndimage.binary_erosion(body)] = -1
    data[(xx < .7) & (yy > .9) & (yy < 1.4)] = 0
    inferred = enclosed_obstacles(data, .05, boundary=data)
    assert not inferred[(yy > 1.4)].any()
    data[(xx > .2) & (xx < .5) & (yy > .2) & (yy < .5)] = 0
    assert not enclosed_obstacles(data, .05, boundary=data).any()
