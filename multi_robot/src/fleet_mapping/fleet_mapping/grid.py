"""Occupancy fusion and Nav2-compatible trinary map encoding."""

from pathlib import Path
import math
import numpy as np
import yaml
from PIL import Image


def fuse(layers, shape=(240, 240), resolution=0.05, origin=(-6.0, -6.0)):
    result = np.full(shape, -1, dtype=np.int8)
    for data, res, ox, oy, oyaw, transform in layers:
        a = np.asarray(data)
        yy, xx = np.where((a >= 0) & ((a <= 25) | (a >= 65)))
        x = ox + (xx + 0.5) * res
        y = oy + (yy + 0.5) * res
        c, s = math.cos(oyaw), math.sin(oyaw)
        # Origin yaw rotates the grid around its origin, not the map frame origin.
        x, y = ox + c * ((xx + 0.5) * res) - s * ((yy + 0.5) * res), oy + s * (
            (xx + 0.5) * res
        ) + c * ((yy + 0.5) * res)
        tx, ty, th = transform
        c, s = math.cos(th), math.sin(th)
        gx = tx + c * x - s * y
        gy = ty + s * x + c * y
        ix = np.floor((gx - origin[0]) / resolution).astype(int)
        iy = np.floor((gy - origin[1]) / resolution).astype(int)
        valid = (ix >= 0) & (iy >= 0) & (ix < shape[1]) & (iy < shape[0])
        ix = ix[valid]
        iy = iy[valid]
        v = a[yy[valid], xx[valid]]
        free = v <= 25
        occ = v >= 65
        # Preserve occupied cells regardless of duplicate resampling indices/order.
        result[iy[free], ix[free]] = np.maximum(result[iy[free], ix[free]], 0)
        result[iy[occ], ix[occ]] = 100
    return result


def save_map(prefix, data, resolution, origin):
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    a = np.asarray(data)
    pgm = np.full(a.shape, 205, dtype=np.uint8)
    pgm[a == 0] = 254
    pgm[a >= 65] = 0
    Image.fromarray(np.flipud(pgm)).save(str(prefix) + ".pgm")
    meta = {
        "image": prefix.name + ".pgm",
        "mode": "trinary",
        "resolution": float(resolution),
        "origin": [float(origin[0]), float(origin[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    Path(str(prefix) + ".yaml").write_text(yaml.safe_dump(meta, sort_keys=False))


def load_map(path):
    p = Path(path)
    m = yaml.safe_load(p.read_text())
    a = np.flipud(np.asarray(Image.open(p.parent / m["image"]).convert("L")))
    prob = (255 - a.astype(float)) / 255
    out = np.full(a.shape, -1, dtype=np.int8)
    out[prob < m["free_thresh"]] = 0
    out[prob > m["occupied_thresh"]] = 100
    return out, m


def mask_robot_returns(
    ranges, angle_min, angle_increment, sensor_pose, other_poses, radius=0.35
):
    """Ignore endpoints on another robot; never clear the space behind its body."""
    values = np.array(ranges, dtype=float, copy=True)
    angles = sensor_pose[2] + angle_min + np.arange(len(values)) * angle_increment
    x = sensor_pose[0] + values * np.cos(angles)
    y = sensor_pose[1] + values * np.sin(angles)
    for p in other_poses:
        values[(x - p[0]) ** 2 + (y - p[1]) ** 2 < radius**2] = np.nan
    return values
