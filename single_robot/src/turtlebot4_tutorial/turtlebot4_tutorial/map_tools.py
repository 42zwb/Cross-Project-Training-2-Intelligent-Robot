"""Validate saved map metadata and preserve trinary unknown cells."""

from collections import Counter
from datetime import datetime
import math
from pathlib import Path
import shutil

from PIL import Image
import yaml


FREE_THRESHOLD = 0.196


def read_map(path):
    """Resolve a map and reject invalid metadata before starting Nav2."""
    path = Path(path).expanduser().resolve()
    with path.open() as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError(f'地图 YAML 必须是字典: {path}')
    image = Path(metadata['image']).expanduser()
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError(f'地图图像不存在: {image}')
    free = float(metadata['free_thresh'])
    occupied = float(metadata['occupied_thresh'])
    resolution = float(metadata['resolution'])
    if not (0.0 < free < occupied < 1.0):
        raise ValueError('地图阈值必须满足 0 < free_thresh < occupied_thresh < 1')
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('地图 resolution 必须是大于 0 的有限数值')
    origin = metadata.get('origin', [])
    if len(origin) != 3 or not all(math.isfinite(float(v)) for v in origin):
        raise ValueError('地图 origin 必须包含三个有限数值')
    if metadata.get('mode', 'trinary') not in ('trinary', 'scale', 'raw'):
        raise ValueError('地图 mode 必须是 trinary、scale 或 raw')
    return path, metadata, image


def has_unsafe_unknowns(metadata, image):
    """Detect canonical map_saver images whose unknown gray becomes free."""
    if metadata.get('mode', 'trinary') != 'trinary' or metadata.get('negate', 0):
        return False
    with Image.open(image) as source:
        counts = Counter(source.convert('L').getdata())
    canonical = set(counts).issubset({0, 205, 254, 255})
    return (canonical and counts.get(205, 0) > 0
            and float(metadata['free_thresh']) >= (255.0 - 205.0) / 255.0)


def validate_map(path):
    """Reject the known unknown-to-free conversion instead of hiding it."""
    path, metadata, image = read_map(path)
    if has_unsafe_unknowns(metadata, image):
        raise ValueError(
            f'{path}: free_thresh 会把灰度 205 的未知区域当作空闲；'
            '请先备份并将 free_thresh 改为 0.196，或重新使用教程 map_saver 保存')
    return str(path)


def repair_map(path):
    """Back up and repair only canonical maps affected by the known issue."""
    path, metadata, image = read_map(path)
    if not has_unsafe_unknowns(metadata, image):
        return None
    backup = path.with_name(path.name + '.bak-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    shutil.copy2(path, backup)
    metadata['free_thresh'] = FREE_THRESHOLD
    path.write_text(yaml.safe_dump(metadata, sort_keys=False))
    validate_map(path)
    return backup
