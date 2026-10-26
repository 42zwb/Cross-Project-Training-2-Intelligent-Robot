"""Seeded scene generation; geometry is used only by Gazebo and offline evaluation.

Version the generator so a recorded seed has a precise reproducibility contract.
All rejection sampling uses a private RNG, never process-global random state.
"""

import hashlib
import json
import math
import random

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull
from fleet_mapping.shapes import as_shape, bounds, shape_distance, shape_gap


GENERATOR = "mixed-shapes-v2"
ARENA = [-7.5, -7.5, 7.5, 7.5]
MIN_GAP = 1.2
CHECK_RESOLUTION = 0.1
NAV_CLEARANCE = 0.5


def walls():
    return [
        ("west", -7.4, 0.0, 0.2, 15.0),
        ("east", 7.4, 0.0, 0.2, 15.0),
        ("south", 0.0, -7.4, 14.6, 0.2),
        ("north", 0.0, 7.4, 14.6, 0.2),
    ]


def box_gap(a, b):
    """Euclidean distance between the surfaces of two axis-aligned rectangles."""
    return math.hypot(max(abs(a[1] - b[1]) - (a[3] + b[3]) / 2, 0),
                      max(abs(a[2] - b[2]) - (a[4] + b[4]) / 2, 0))


def navigation_space(geometry, starts):
    """Check conservative centre clearance and connectivity on a 10 cm grid."""
    yy, xx = np.indices((150, 150))
    x, y = -7.5 + (xx + 0.5) * CHECK_RESOLUTION, -7.5 + (yy + 0.5) * CHECK_RESOLUTION
    allowed = np.ones(x.shape, dtype=bool)
    for item in geometry:
        if isinstance(item, dict):
            distance = shape_distance(x, y, item)
        else:
            _, bx, by, dx, dy = item
            distance = np.hypot(np.maximum(abs(x - bx) - dx / 2, 0),
                                np.maximum(abs(y - by) - dy / 2, 0))
        allowed &= distance >= NAV_CLEARANCE
    labels, count = ndimage.label(allowed)
    start_labels = [int(labels[int((p[1] + 7.5) / CHECK_RESOLUTION),
                              int((p[0] + 7.5) / CHECK_RESOLUTION)]) for p in starts]
    return {
        "connected": count == 1 and all(label == 1 for label in start_labels),
        "components": int(count),
        "start_labels": start_labels,
        "free_fraction": float(np.mean(allowed)),
        "checked_resolution": CHECK_RESOLUTION,
        "centre_clearance": NAV_CLEARANCE,
    }


def generate_rectangles(seed, starts, obstacle_count=10):
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**63:
        raise ValueError("场景种子必须是 0 到 2^63-1 的整数。")
    if isinstance(obstacle_count, bool) or not isinstance(obstacle_count, int) or not 4 <= obstacle_count <= 16:
        raise ValueError("obstacle_count 必须是 4 到 16 的整数。")
    if len(starts) != 3 or any(len(p) < 2 or not all(math.isfinite(v) and -6.8 <= v <= 6.8 for v in p[:2]) for p in starts):
        raise ValueError("随机场景要求三个起点位于 [-6.8, 6.8] 范围内。")
    rng = random.Random(seed)
    base = walls()
    # Preserve a central gathering area and clear space around all spawn poses.
    reserved = [("gathering", 0.0, 0.0, 3.4, 3.4)]
    reserved += [("spawn", p[0], p[1], 1.8, 1.8) for p in starts]
    geometry = None
    attempts = 0
    # Restarting packing avoids allowing an unlucky early placement to starve it.
    for _ in range(32):
        placed = list(base)
        for _ in range(1500):
            attempts += 1
            if rng.random() < 0.3:
                dx, dy = rng.uniform(2.8, 4.0), rng.uniform(1.0, 1.3)
            else:
                dx, dy = rng.uniform(1.0, 2.3), rng.uniform(1.0, 2.3)
            if rng.random() < 0.5:
                dx, dy = dy, dx
            dx, dy = round(dx, 3), round(dy, 3)
            x = round(rng.uniform(-6.1 + dx / 2, 6.1 - dx / 2), 3)
            y = round(rng.uniform(-6.1 + dy / 2, 6.1 - dy / 2), 3)
            box = (f"obstacle_{len(placed) - 4:02d}", x, y, dx, dy)
            if any(box_gap(box, other) < MIN_GAP for other in placed):
                continue
            if any(box_gap(box, other) < 0.1 for other in reserved):
                continue
            placed.append(box)
            if len(placed) == obstacle_count + 4:
                checked = navigation_space(placed, starts)
                if checked["connected"] and checked["free_fraction"] >= 0.5:
                    geometry = placed
                break
        if geometry is not None:
            break
    if geometry is None:
        raise ValueError("在采样预算内无法生成安全场景，请降低 obstacle_count 或调整起点。")
    digest = hashlib.sha256(json.dumps(geometry, separators=(",", ":")).encode()).hexdigest()
    return geometry, {
        "generator": "rectangles-v1",
        "seed": seed,
        "arena": ARENA,
        "obstacle_count": obstacle_count,
        "minimum_obstacle_side": 1.0,
        "minimum_surface_gap": MIN_GAP,
        "geometry_sha256": digest,
        "sampling_attempts": attempts,
        "navigation_check": checked,
    }


def _shape(rng, kind):
    shape = {"kind": kind, "x": 0.0, "y": 0.0,
             "yaw": round(rng.uniform(-math.pi, math.pi), 8)}
    if kind == "rectangle":
        shape["size"] = [round(rng.uniform(1.1, 2.1), 4), round(rng.uniform(1.1, 2.3), 4)]
    elif kind == "circle":
        shape["radius"] = round(rng.uniform(0.65, 1.05), 4)
    elif kind == "convex_polygon":
        count = rng.randint(5, 8)
        angles = np.linspace(0, 2*math.pi, count, endpoint=False)
        radii = np.array([rng.uniform(0.9, 1.1) for _ in angles])
        points = np.column_stack((np.cos(angles), np.sin(angles))) * radii[:, None]
        points *= [rng.uniform(0.8, 1.15), rng.uniform(0.8, 1.15)]
        shape["vertices"] = np.round(points[ConvexHull(points).vertices], 6).tolist()
    elif kind == "concave_polygon":
        width, height = rng.uniform(1.8, 2.4), rng.uniform(1.8, 2.4)
        arm_x, arm_y = rng.uniform(0.65, 0.85), rng.uniform(0.65, 0.85)
        points = [[0, 0], [width, 0], [width, arm_y], [arm_x, arm_y],
                  [arm_x, height], [0, height]]
        shape["vertices"] = np.round(np.asarray(points)-[width/2, height/2], 6).tolist()
        shape["minimum_arm_width"] = min(arm_x, arm_y)
    elif kind == "sector":
        radius = rng.uniform(1.0, 1.35)
        span = math.radians(rng.uniform(100, 280))
        angles = np.linspace(-span/2, span/2, int(math.ceil(span/0.08))+1)
        points = np.vstack(([0, 0], np.column_stack((np.cos(angles), np.sin(angles)))*radius))
        shape.update(radius=round(radius, 6), angle=round(span, 8),
                     vertices=np.round(points, 6).tolist())
    else:
        raise ValueError("未知障碍形状")
    return shape


def _too_close(shape, others, gap):
    """Skip exact polygon distances when bounding boxes already prove a gap."""
    ax0, ay0, ax1, ay1 = bounds(shape)
    for other, (bx0, by0, bx1, by1) in others:
        lower_bound = math.hypot(max(bx0-ax1, ax0-bx1, 0),
                                 max(by0-ay1, ay0-by1, 0))
        if lower_bound < gap and shape_gap(shape, other) < gap:
            return True
    return False


def generate_scene(seed, starts, obstacle_count=10, generator=GENERATOR):
    if generator == "rectangles-v1":
        return generate_rectangles(seed, starts, obstacle_count)
    if generator != GENERATOR:
        raise ValueError("未知场景生成器")
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**63:
        raise ValueError("场景种子必须是 0 到 2^63-1 的整数。")
    if isinstance(obstacle_count, bool) or not isinstance(obstacle_count, int) or not 4 <= obstacle_count <= 16:
        raise ValueError("obstacle_count 必须是 4 到 16 的整数。")
    if len(starts) != 3 or any(len(p) < 2 or not all(math.isfinite(v) and -6.8 <= v <= 6.8 for v in p[:2]) for p in starts):
        raise ValueError("随机场景要求三个起点位于 [-6.8, 6.8] 范围内。")
    rng = random.Random(seed)
    kinds = ["rectangle", "convex_polygon", "concave_polygon", "circle", "sector"]
    order = kinds * (obstacle_count // len(kinds)) + rng.sample(kinds, obstacle_count % len(kinds))
    rng.shuffle(order)
    reserved = [("gathering", 0, 0, 3.4, 3.4)] + [("spawn", p[0], p[1], 1.8, 1.8) for p in starts]
    reserved_bounds = [(item, bounds(item)) for item in reserved]
    attempts = 0
    for restart in range(32):
        placed = [as_shape(wall) for wall in walls()]
        placed_bounds = []
        for _ in range(1800):
            attempts += 1
            proposal = _shape(rng, order[len(placed)-4])
            x0, y0, x1, y1 = bounds(proposal)
            proposal.update(name=f"obstacle_{len(placed)-4:02d}",
                            x=round(rng.uniform(-6.1-x0, 6.1-x1), 6),
                            y=round(rng.uniform(-6.1-y0, 6.1-y1), 6))
            # The fitting box [-6.1, 6.1] already leaves >=1.2 m to walls.
            if _too_close(proposal, placed_bounds, MIN_GAP):
                continue
            if _too_close(proposal, reserved_bounds, 0.1):
                continue
            placed.append(proposal)
            placed_bounds.append((proposal, bounds(proposal)))
            if len(placed) == obstacle_count+4:
                checked = navigation_space(placed, starts)
                if checked["connected"] and checked["free_fraction"] >= 0.5:
                    digest = hashlib.sha256(json.dumps(placed, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
                    return placed, {"generator": GENERATOR, "seed": seed, "arena": ARENA,
                        "obstacle_count": obstacle_count, "shape_counts": {kind: order.count(kind) for kind in kinds},
                        "minimum_surface_gap": MIN_GAP, "minimum_concave_arm_width": 0.65,
                        "rotation_range_rad": [-math.pi, math.pi], "maximum_arc_chord_error_m": 0.0011,
                        "geometry_sha256": digest, "sampling_attempts": attempts,
                        "navigation_check": checked}
                break
    raise ValueError("在采样预算内无法生成安全混合形状场景，请降低 obstacle_count 或调整起点。")
