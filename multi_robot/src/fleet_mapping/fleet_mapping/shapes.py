"""Pure 2-D geometry shared by scene generation and independent evaluation.

Runtime SLAM and navigation never receive generated obstacle geometry.
Legacy axis-aligned box records remain readable for historical evaluations.
"""

import math
import numpy as np


def as_shape(value):
    if isinstance(value, dict):
        return value
    name, x, y, width, height = value
    return {"name": name, "kind": "rectangle", "x": x, "y": y,
            "yaw": 0.0, "size": [width, height]}


def local_vertices(value):
    shape = as_shape(value)
    if shape["kind"] == "circle":
        angles = np.linspace(0, 2 * math.pi, 96, endpoint=False)
        return np.column_stack((np.cos(angles), np.sin(angles))) * shape["radius"]
    if shape["kind"] == "rectangle":
        x, y = np.asarray(shape["size"], float) / 2
        return np.array([[-x, -y], [x, -y], [x, y], [-x, y]])
    return np.asarray(shape["vertices"], dtype=float)


def vertices(value):
    shape = as_shape(value)
    c, s = math.cos(shape["yaw"]), math.sin(shape["yaw"])
    return local_vertices(shape) @ np.array([[c, s], [-s, c]]) + [shape["x"], shape["y"]]


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def polygon_area(points):
    points = np.asarray(points)
    return float(np.sum(cross(points, np.roll(points, -1, axis=0))) / 2)


def point_polygon_distance(x, y, points):
    x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
    inside = np.zeros(x.shape, bool)
    distance2 = np.full(x.shape, np.inf)
    for a, b in zip(points, np.roll(points, -1, axis=0)):
        edge = b - a
        length2 = float(np.dot(edge, edge))
        if length2 < 1e-20:
            raise ValueError("多边形包含重复顶点")
        projection = np.clip(((x-a[0])*edge[0] + (y-a[1])*edge[1]) / length2, 0, 1)
        distance2 = np.minimum(distance2, (x-a[0]-projection*edge[0])**2
                               + (y-a[1]-projection*edge[1])**2)
        denominator = edge[1] if abs(edge[1]) > 1e-15 else 1.0
        inside ^= ((a[1] > y) != (b[1] > y)) & (x < a[0] + (y-a[1])*edge[0]/denominator)
    return np.where(inside, 0.0, np.sqrt(distance2))


def shape_distance(x, y, value):
    shape = as_shape(value)
    if shape["kind"] == "circle":
        return np.maximum(np.hypot(np.asarray(x)-shape["x"], np.asarray(y)-shape["y"])
                          - shape["radius"], 0.0)
    return point_polygon_distance(x, y, vertices(shape))


def bounds(value):
    shape = as_shape(value)
    if shape["kind"] == "circle":
        radius = shape["radius"]
        return (shape["x"]-radius, shape["y"]-radius,
                shape["x"]+radius, shape["y"]+radius)
    points = vertices(shape)
    return tuple(float(v) for v in (*points.min(axis=0), *points.max(axis=0)))


def shape_gap(first, second):
    a, b = as_shape(first), as_shape(second)
    if a["kind"] == "circle":
        return max(0.0, float(shape_distance(a["x"], a["y"], b)) - a["radius"])
    if b["kind"] == "circle":
        return shape_gap(b, a)
    aa, bb = vertices(a), vertices(b)
    a0, a1 = aa[:, None, :], np.roll(aa, -1, axis=0)[:, None, :]
    b0, b1 = bb[None, :, :], np.roll(bb, -1, axis=0)[None, :, :]
    # Crossing edges can intersect without containing any opposing vertex.
    intersects = ((cross(a1-a0, b0-a0) * cross(a1-a0, b1-a0) <= 1e-14)
                  & (cross(b1-b0, a0-b0) * cross(b1-b0, a1-b0) <= 1e-14)
                  & np.all(np.maximum(np.minimum(a0, a1), np.minimum(b0, b1))
                           <= np.minimum(np.maximum(a0, a1), np.maximum(b0, b1)) + 1e-10, axis=-1))
    if np.any(intersects):
        return 0.0
    return float(min(point_polygon_distance(aa[:, 0], aa[:, 1], bb).min(),
                     point_polygon_distance(bb[:, 0], bb[:, 1], aa).min()))


def is_convex(points):
    points = np.asarray(points)
    edges = np.roll(points, -1, axis=0) - points
    turns = cross(edges, np.roll(edges, -1, axis=0))
    return bool(np.all(turns >= -1e-10) or np.all(turns <= 1e-10))


def triangulate(points):
    """Ear clipping for a simple polygon, preserving concave empty bays."""
    points = np.asarray(points, float)
    if len(points) < 3 or not np.isfinite(points).all() or abs(polygon_area(points)) < 1e-10:
        raise ValueError("多边形必须具有有限顶点及非零面积")
    remaining = list(range(len(points)))
    if polygon_area(points) < 0:
        remaining.reverse()
    triangles = []
    while len(remaining) > 3:
        for offset, middle in enumerate(remaining):
            left, right = remaining[offset-1], remaining[(offset+1) % len(remaining)]
            a, b, c = points[[left, middle, right]]
            if cross(b-a, c-b) <= 1e-12:
                continue
            others = points[[i for i in remaining if i not in (left, middle, right)]]
            if len(others) and np.any((cross(b-a, others-a) >= -1e-12)
                                      & (cross(c-b, others-b) >= -1e-12)
                                      & (cross(a-c, others-c) >= -1e-12)):
                continue
            triangles.append([left, middle, right])
            remaining.pop(offset)
            break
        else:
            raise ValueError("多边形自交、退化或无法三角剖分")
    triangles.append(remaining)
    return triangles


def raycast(origin, angles, geometry, max_range=12.0):
    """Analytic first intersections, for independent lidar validation only."""
    angles = np.asarray(angles)
    direction = np.column_stack((np.cos(angles), np.sin(angles)))
    ranges = np.full(len(angles), max_range, float)
    hits = np.full(len(angles), -1, int)
    for index, value in enumerate(geometry):
        shape = as_shape(value)
        if shape["kind"] == "circle":
            delta = np.array([shape["x"], shape["y"]]) - origin[:2]
            projection = direction @ delta
            discriminant = projection**2 - np.dot(delta, delta) + shape["radius"]**2
            root = np.sqrt(np.maximum(discriminant, 0))
            near, far = projection-root, projection+root
            distance = np.where(near >= 0, near, far)
            distance = np.where((discriminant >= 0) & (distance >= 0), distance, np.inf)
        else:
            points = vertices(shape)
            edge = np.roll(points, -1, axis=0)-points
            delta = points-np.asarray(origin[:2])
            denominator = cross(direction[:, None, :], edge[None, :, :])
            safe = np.where(abs(denominator) > 1e-12, denominator, 1)
            t = cross(delta, edge)[None, :] / safe
            u = cross(delta[None, :, :], direction[:, None, :]) / safe
            distance = np.min(np.where((abs(denominator) > 1e-12) & (t >= 0)
                                       & (u >= -1e-10) & (u <= 1+1e-10), t, np.inf), axis=1)
        closer = distance < ranges
        ranges[closer], hits[closer] = distance[closer], index
    return ranges, hits
