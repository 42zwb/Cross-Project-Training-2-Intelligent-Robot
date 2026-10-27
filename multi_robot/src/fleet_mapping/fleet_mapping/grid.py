"""Occupancy fusion and Nav2-compatible trinary map encoding."""

from pathlib import Path
import math
import numpy as np
import yaml
from PIL import Image


def valid_grid_message(message):
    """Reject malformed, oversized or nonplanar grids before allocating arrays."""
    info = message.info
    p, q = info.origin.position, info.origin.orientation
    return (
        0 < info.width and 0 < info.height and info.width * info.height <= 4_000_000
        and len(message.data) == info.width * info.height
        and math.isfinite(info.resolution) and info.resolution > 0
        and all(math.isfinite(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        and abs(q.x) < 1e-6 and abs(q.y) < 1e-6
        and abs(q.z * q.z + q.w * q.w - 1.0) < 0.01
        and np.min(message.data) >= -1 and np.max(message.data) <= 100
    )


def fuse(layers, shape=(240, 240), resolution=0.05, origin=(-6.0, -6.0)):
    result = np.full(shape, -1, dtype=np.int8)
    yy, xx = np.indices(shape)
    world_x = origin[0] + (xx + 0.5) * resolution
    world_y = origin[1] + (yy + 0.5) * resolution
    for data, res, ox, oy, oyaw, transform in layers:
        a = np.asarray(data)
        tx, ty, th = transform
        c, s = math.cos(th), math.sin(th)
        # Pull each output centre into the source grid. Forward centre splatting
        # leaves holes when upsampling or rotating, fragmenting safe free space.
        local_x = c * (world_x - tx) + s * (world_y - ty) - ox
        local_y = -s * (world_x - tx) + c * (world_y - ty) - oy
        origin_cos, origin_sin = math.cos(oyaw), math.sin(oyaw)
        sx = np.floor((origin_cos * local_x + origin_sin * local_y) / res).astype(int)
        sy = np.floor((-origin_sin * local_x + origin_cos * local_y) / res).astype(int)
        inside = (sx >= 0) & (sy >= 0) & (sx < a.shape[1]) & (sy < a.shape[0])
        values = np.full(shape, -1, dtype=np.int16)
        values[inside] = a[sy[inside], sx[inside]]
        result[(values >= 0) & (values <= 25) & (result != 100)] = 0
        result[values >= 65] = 100

        # Also forward-project occupied source samples. A small obstacle must
        # survive downsampling even if no coarse output centre lands inside it.
        by, bx = np.where(a >= 65)
        x = ox + origin_cos * ((bx + 0.5) * res) - origin_sin * ((by + 0.5) * res)
        y = oy + origin_sin * ((bx + 0.5) * res) + origin_cos * ((by + 0.5) * res)
        ix = np.floor((tx + c * x - s * y - origin[0]) / resolution).astype(int)
        iy = np.floor((ty + s * x + c * y - origin[1]) / resolution).astype(int)
        valid = (ix >= 0) & (iy >= 0) & (ix < shape[1]) & (iy < shape[0])
        result[iy[valid], ix[valid]] = 100
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
