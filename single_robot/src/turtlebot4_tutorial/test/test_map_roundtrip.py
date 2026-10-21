from collections import Counter
from pathlib import Path

from PIL import Image
import pytest
import yaml

from turtlebot4_tutorial.map_tools import repair_map, validate_map


def make_map(directory, free=0.25):
    image = Image.new('L', (3, 1))
    image.putdata([0, 205, 254])
    image.save(directory / 'map.pgm')
    path = directory / 'map.yaml'
    path.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': 0.05, 'origin': [0, 0, 0],
        'negate': 0, 'mode': 'trinary', 'free_thresh': free, 'occupied_thresh': 0.65,
    }))
    return path


def test_unsafe_map_is_rejected_and_backup_repair_preserves_unknown(tmp_path):
    path = make_map(tmp_path)
    before = path.read_bytes()
    with pytest.raises(ValueError, match='未知'):
        validate_map(path)
    backup = repair_map(path)
    assert backup.read_bytes() == before
    assert validate_map(path) == str(path)
    metadata = yaml.safe_load(path.read_text())
    with Image.open(tmp_path / metadata['image']) as image:
        cells = []
        for gray in image.getdata():
            probability = (255 - gray) / 255.0
            cells.append(100 if probability > metadata['occupied_thresh'] else (
                0 if probability < metadata['free_thresh'] else -1))
    assert Counter(cells) == {-1: 1, 0: 1, 100: 1}
    assert repair_map(path) is None


def test_map_relative_image_path_and_missing_image(tmp_path):
    path = make_map(tmp_path, free=0.196)
    assert Path(validate_map(path)).is_absolute()
    (tmp_path / 'map.pgm').unlink()
    with pytest.raises(ValueError, match='不存在'):
        validate_map(path)
