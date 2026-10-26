"""Conservative completion of observed closed, compact solid obstacles.

This is a solid-obstacle prior for the course scene, not a measurement of the
interior. Never reads scene geometry, fills open contours, or overwrites free
observations. Every update is derived anew from the raw SLAM fusion.
"""

import math
import time

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull, QhullError


def enclosed_obstacles(data, resolution, boundary=None):
    data = np.asarray(data)
    occupied = data >= 65 if boundary is None else ((np.asarray(boundary) >= 65) & (data != 0))
    # Bridge at most 0.20 m of raster contour fragmentation. This is below a
    # navigable doorway's width; no measured free cell is changed in the output.
    steps = int(math.floor(0.10 / resolution + 1e-6))
    barrier = occupied.copy()
    if steps:
        barrier |= ndimage.binary_closing(occupied, iterations=steps,
                                         structure=np.ones((3, 3), bool))
    holes = ndimage.binary_fill_holes(barrier) & ~barrier
    labels, _ = ndimage.label(holes)
    inferred = np.zeros(data.shape, bool)
    edge_width = max(1, int(math.ceil(0.15 / resolution)))
    for label, bounds in enumerate(ndimage.find_objects(labels), 1):
        if bounds is None:
            continue
        hole = labels[bounds] == label
        area = hole.sum() * resolution**2
        sides = np.asarray(hole.shape) * resolution
        # Reject large rooms, thin wall slivers and arena-border regions.
        if not 0.15 <= area <= 10.0 or sides.min() < 0.45 or sides.max() > 4.8:
            continue
        core = ndimage.binary_erosion(hole, iterations=edge_width, border_value=0)
        if not core.any() or np.any(core & (data[bounds] == 0)):
            continue
        points = np.argwhere(hole)
        try:
            convex_area = ConvexHull(points).volume
        except QhullError:
            continue
        solidity = hole.sum() / max(1.0, convex_area)
        if solidity < 0.55:
            continue
        if boundary is not None:
            # Moderate occupancy may connect a measured edge, but cannot alone
            # establish an object: >=50% (>=65% for concave contours) of the
            # rim needs strong support in the original fused observations.
            padded = np.pad(hole, 1)
            rim = ndimage.binary_dilation(padded) & ~padded
            patch = data[bounds[0].start - 1:bounds[0].stop + 1,
                         bounds[1].start - 1:bounds[1].stop + 1]
            required_support = 0.65 if solidity < 0.90 else 0.5
            if np.sum(rim & (patch >= 65)) < required_support * np.sum(rim):
                continue
        # Known free cells near a noisy contour remain free as well.
        inferred[bounds] |= hole & (data[bounds] < 0)
    return inferred


class ObstacleCompletion:
    def __init__(self):
        self.signature = None
        self.last_evidence = None

    def update(self, data, resolution, evidence, now=None, boundary=None):
        now = time.monotonic() if now is None else now
        signature = (data.shape, resolution)
        if signature != self.signature:
            self.signature = signature
            self.support = np.zeros(data.shape, np.uint8)
            self.since = np.full(data.shape, np.inf)
            self.confirmed = np.zeros(data.shape, bool)
            self.last_supported = np.full(data.shape, -np.inf)
            self.last_evidence = None
        candidate = enclosed_obstacles(data, resolution, boundary)
        self.last_supported[candidate] = now
        self.support[~candidate] = 0
        self.since[~candidate] = np.inf
        first = candidate & (self.support == 0)
        self.since[first] = now
        if evidence != self.last_evidence:
            self.support[candidate] = np.minimum(self.support[candidate].astype(int) + 1, 255)
            self.last_evidence = evidence
        fresh = candidate & (self.support >= 3) & (now - self.since >= 2.0)
        # Briefly missing contour pixels are not positive evidence of an open
        # interior. Hold a confirmed inference for <= 5 s, but immediately
        # retract the whole component when free observations contradict its core.
        confirmed = fresh | self.confirmed
        labels, _ = ndimage.label(confirmed)
        edge_width = max(1, int(math.ceil(0.15 / resolution)))
        for label, bounds in enumerate(ndimage.find_objects(labels), 1):
            component = labels[bounds] == label
            core = ndimage.binary_erosion(component, iterations=edge_width)
            if np.any(core & (data[bounds] == 0)):
                confirmed[bounds] &= ~component
        inferred = confirmed & (now - self.last_supported <= 5.0) & (data < 0)
        self.confirmed = inferred
        result = np.array(data, copy=True)
        result[inferred] = 100
        return result, inferred


def map_metrics(observed, inferred):
    total = observed.size
    return {
        "known_fraction": float(np.mean((observed >= 0) | inferred)),
        "observed_fraction": float(np.mean(observed >= 0)),
        "inferred_obstacle_fraction": float(np.mean(inferred)),
        "unknown_fraction": float(np.mean((observed < 0) & ~inferred)),
        "observed_cells": int(np.sum(observed >= 0)),
        "inferred_obstacle_cells": int(inferred.sum()),
        "total_cells": int(total),
        "completion_method": "closed-compact-solid-v4",
        "completion_policy": {"direct_occupied_threshold": 65,
                              "contour_probability_threshold": 55,
                              "minimum_strong_contour_fraction": 0.5,
                              "minimum_concave_strong_contour_fraction": 0.65,
                              "minimum_solidity": 0.55,
                              "maximum_contour_gap_m": 0.20,
                              "new_updates_required": 3, "support_seconds": 2.0,
                              "hold_seconds": 5.0},
    }
