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


GENERATOR = "rectangles-v1"
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
    for _, bx, by, dx, dy in geometry:
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


def generate_scene(seed, starts, obstacle_count=10):
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
        "generator": GENERATOR,
        "seed": seed,
        "arena": ARENA,
        "obstacle_count": obstacle_count,
        "minimum_obstacle_side": 1.0,
        "minimum_surface_gap": MIN_GAP,
        "geometry_sha256": digest,
        "sampling_attempts": attempts,
        "navigation_check": checked,
    }
