"""Remove confirmed current peer echoes from derived layers, never raw maps."""
import math
import numpy as np
from .registration import compose, inverse, transform_points


def filter_peer_echoes(snapshot, graph, peer_samples, now, radius, raw_sources=None):
    """A robot's own layer always retains its own obstacle observations.

    At unknown startup other robots really are unclassified obstacles. After
    alignment, a current confirmed footprint may be removed from another
    source's historical occupancy. Removed pixels become unknown, not free;
    free space still needs an actual free observation from a raw source.
    """
    poses = graph.poses((snapshot.robot, snapshot.boot, snapshot.epoch))
    data = snapshot.data
    result = None
    for robot, boot, epoch, pose, received, valid in peer_samples:
        if robot == snapshot.robot or not valid or not 0 <= now-received < .8:
            continue
        key = (robot, boot, epoch)
        if key not in poses:
            continue
        centre = compose(inverse(snapshot.origin), compose(poses[key], pose))
        res = snapshot.resolution
        lo_x, hi_x = max(0, int(math.floor((centre[0]-radius)/res))), min(data.shape[1], int(math.ceil((centre[0]+radius)/res)))
        lo_y, hi_y = max(0, int(math.floor((centre[1]-radius)/res))), min(data.shape[0], int(math.ceil((centre[1]+radius)/res)))
        if hi_x <= lo_x or hi_y <= lo_y:
            continue
        yy, xx = np.ogrid[lo_y:hi_y, lo_x:hi_x]
        footprint = (((xx+.5)*res-centre[0])**2+((yy+.5)*res-centre[1])**2) <= radius**2
        if result is None:
            result = data.copy()
        patch = result[lo_y:hi_y, lo_x:hi_x]
        selected = footprint & (patch >= 65)
        if not selected.any():
            continue
        if raw_sources is not None:
            own = raw_sources.get(robot)
            if own is None or (own.robot, own.boot, own.epoch) != key:
                continue
            sy, sx = np.where(selected)
            raw_points = transform_points(snapshot.origin, np.column_stack([
                (sx+lo_x+.5)*res, (sy+lo_y+.5)*res]))
            peer_points = transform_points(inverse(poses[key]), raw_points)
            free, _, _ = own.samples(peer_points)
            selected[sy, sx] = (free >= 0) & (free <= 25)
        patch[selected] = -1
    return data if result is None else result
