"""Completion in arbitrary local coordinates, with no canvas-boundary prior."""
import math
import cv2
import numpy as np
from .completion import enclosed_obstacles
from .registration import angle, transform_points


def observed_enclosure(data, resolution, origin):
    occupied = np.asarray(data) >= 65
    lines = cv2.HoughLinesP(occupied.astype(np.uint8)*255, 1, np.pi/720,
                            threshold=30, minLineLength=max(20, int(2/resolution)), maxLineGap=4)
    if lines is None:
        return None
    bins = {}
    for x1, y1, x2, y2 in lines[:, 0]:
        theta = (math.atan2(float(y2-y1), float(x2-x1))+math.pi/4) % (math.pi/2)-math.pi/4
        bucket = round(theta/math.radians(.5))
        bins[bucket] = bins.get(bucket, 0)+math.hypot(float(x2-x1), float(y2-y1))
    theta = max(bins, key=bins.get)*math.radians(.5)
    yy, xx = np.where(occupied)
    if len(xx) < 200:
        return None
    points = np.column_stack([origin[0]+(xx+.5)*resolution, origin[1]+(yy+.5)*resolution])
    flat = transform_points((0., 0., -theta), points)
    low, high = np.quantile(flat, [.001, .999], axis=0)
    extent = high-low
    if min(extent) < 5.:
        return None
    # Four observed, long, mutually enclosing wall faces are all required.
    # A canvas edge, short object or unobserved fourth wall provides no support.
    for axis in (0, 1):
        other = 1-axis
        for value in (low[axis], high[axis]):
            support = flat[np.abs(flat[:, axis]-value) < .13, other]
            count = int(math.ceil(extent[other]/.30))
            bins_present = np.zeros(count, bool)
            cells = np.floor((support-low[other])/.30).astype(int)
            bins_present[np.clip(cells, 0, count-1)] = True
            missing = np.flatnonzero(np.diff(np.r_[False, ~bins_present, False]))
            maximum_gap = max((b-a for a, b in zip(missing[::2], missing[1::2])), default=0)*.30
            if bins_present.mean() < .75 or maximum_gap > .61:
                return None
    fy, fx = np.where(np.asarray(data) == 0)
    free = transform_points((0., 0., -theta), np.column_stack([
        origin[0]+(fx+.5)*resolution, origin[1]+(fy+.5)*resolution]))
    contradicted = np.any((free < low-.12) | (free > high+.12), axis=1)
    if len(free) < 100 or np.mean(contradicted) > .001:
        return None
    return float(theta), low, high


class ObservedCompletion:
    def __init__(self):
        self.boundary, self.pending, self.count, self.since = None, None, 0, None
        self.evidence = None
        self.interior_support = {}

    def update(self, data, resolution, origin, evidence, now):
        data = np.asarray(data)
        candidate = observed_enclosure(data, resolution, origin)
        fresh = self.evidence != evidence
        self.evidence = evidence
        if candidate is not None:
            theta, low, high = candidate
            previous = self.pending
            same = (previous is not None and abs(angle(theta-previous[0])) < math.radians(1)
                    and np.max(np.abs(low-previous[1])) < .10 and np.max(np.abs(high-previous[2])) < .10)
            if not same:
                self.pending, self.count, self.since = candidate, 1, now
            elif fresh:
                self.count += 1
            if self.count >= 3 and now-self.since >= 2.:
                self.boundary = candidate
        inferred = np.zeros(data.shape, bool)
        # Unknown obstacle interiors need repeated independent map revisions.
        interior = enclosed_obstacles(data, resolution)
        iy, ix = np.where(interior)
        keys = np.column_stack([np.rint((origin[0]+(ix+.5)*resolution)/resolution).astype(int),
                                np.rint((origin[1]+(iy+.5)*resolution)/resolution).astype(int)])
        current = set()
        for y, x, key in zip(iy, ix, map(tuple, keys)):
            current.add(key)
            count, since = self.interior_support.get(key, (0, now))
            count += int(fresh)
            self.interior_support[key] = count, since
            inferred[y, x] = count >= 3 and now-since >= 2.
        self.interior_support = {k: v for k, v in self.interior_support.items() if k in current}
        boundary_mask = np.zeros(data.shape, bool)
        if self.boundary is not None and candidate is not None:
            theta, low, high = self.boundary
            yy, xx = np.indices(data.shape)
            points = transform_points((0., 0., -theta), np.column_stack([
                origin[0]+(xx.ravel()+.5)*resolution, origin[1]+(yy.ravel()+.5)*resolution]))
            boundary_mask = np.any((points < low) | (points > high), axis=1).reshape(data.shape) & (data < 0)
            inferred |= boundary_mask
        output = data.copy()
        output[inferred] = 100
        details = dict(observed_fraction=float(np.mean(data >= 0)),
            inferred_obstacle_fraction=float(np.mean(inferred)),
            boundary_obstacle_fraction=float(np.mean(boundary_mask)),
            boundary_confirmed=bool(self.boundary is not None and candidate is not None),
            boundary_source='four_observed_wall_faces', observed_only=not bool(inferred.any()))
        if self.boundary is not None:
            theta, low, high = self.boundary
            details['observed_enclosure'] = dict(yaw=theta, low=low.tolist(), high=high.tolist())
        return output, details
