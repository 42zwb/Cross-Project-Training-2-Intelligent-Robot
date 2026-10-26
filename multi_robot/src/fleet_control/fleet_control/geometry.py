"""Pure SE(2), occupancy-grid planning and formation routines (no simulator truth)."""

import heapq
import itertools
import math
import time
import numpy as np
from scipy import ndimage


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


FORMATION_POSITION_TOLERANCE = 0.25
FORMATION_YAW_TOLERANCE = math.radians(15.0)
FORMATION_CLEARANCE_MARGIN = 0.14


def formation_pass(poses, slots, assignment, robot_radius):
    """Check formation error and body separation using the shared acceptance policy."""
    if len(poses) != 3 or len(slots) != 3 or len(assignment) != 3:
        return False
    if sorted(assignment) != [0, 1, 2] or any(
        len(p) != 3 or not all(math.isfinite(v) for v in p) for p in list(poses) + list(slots)
    ):
        return False
    for robot_index, pose in enumerate(poses):
        target = slots[assignment[robot_index]]
        if (
            math.dist(pose[:2], target[:2]) > FORMATION_POSITION_TOLERANCE
            or abs(angle(pose[2] - target[2])) > FORMATION_YAW_TOLERANCE
        ):
            return False
    minimum_separation = 2.0 * robot_radius + FORMATION_CLEARANCE_MARGIN
    return all(
        math.dist(poses[i][:2], poses[j][:2]) >= minimum_separation
        for i in range(3)
        for j in range(i)
    )


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], angle(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def formation(shape, x, y, heading, spacing):
    if (
        not all(math.isfinite(v) for v in [x, y, heading, spacing])
        or not 0.7 <= spacing <= 3.0
    ):
        raise ValueError("间距必须在 0.7–3 m，所有数值必须有限。")
    if shape == "row":
        pts = [(0, -spacing), (0, 0), (0, spacing)]
    elif shape == "column":
        pts = [(-spacing, 0), (0, 0), (spacing, 0)]
    elif shape == "triangle":
        pts = [
            (spacing / math.sqrt(3), 0),
            (-spacing / (2 * math.sqrt(3)), -spacing / 2),
            (-spacing / (2 * math.sqrt(3)), spacing / 2),
        ]
    else:
        raise ValueError("队形仅支持 triangle、row、column。")
    return [compose((x, y, heading), (a, b, 0.0)) for a, b in pts]


def validate_regions(regions, starts, arena, margin=0.35, robot_radius=0.18):
    if len(regions) != 3:
        raise ValueError("需要三个矩形。")
    area = 0
    for i, r in enumerate(regions):
        if len(r) != 4 or not all(math.isfinite(v) for v in r):
            raise ValueError("区域必须是四个有限数值。")
        x0, y0, x1, y1 = r
        if (
            x1 - x0 < 2.0 * (margin + robot_radius + 0.03) + 0.2
            or y1 - y0 < 2.0 * (margin + robot_radius + 0.03) + 0.2
            or x0 < arena[0]
            or y0 < arena[1]
            or x1 > arena[2]
            or y1 > arena[3]
        ):
            raise ValueError("区域越界或过窄。")
        start_margin = margin + robot_radius + 0.03
        if (
            not x0 + start_margin <= starts[i][0] <= x1 - start_margin
            or not y0 + start_margin <= starts[i][1] <= y1 - start_margin
        ):
            raise ValueError(f"robot{i+1} 起点必须在区域安全边界内。")
        area += (x1 - x0) * (y1 - y0)
        for other in regions[:i]:
            if (
                min(x1, other[2]) - max(x0, other[0]) > 1e-6
                and min(y1, other[3]) - max(y0, other[1]) > 1e-6
            ):
                raise ValueError("区域不能重叠。")
    if abs(area - (arena[2] - arena[0]) * (arena[3] - arena[1])) > 1e-4:
        raise ValueError("三个区域必须覆盖整个场地。")
    return True


class Grid:
    def __init__(
        self,
        data,
        res=0.05,
        origin=(-6.0, -6.0),
        robot_radius=0.18,
        region_margin=0.35,
    ):
        self.data = np.asarray(data, dtype=np.int16)
        self.res = res
        self.origin = origin
        self.robot_radius = robot_radius
        self.region_margin = region_margin
        self.clearance_radius = robot_radius + 0.10
        self.region_clearance = region_margin + robot_radius + 0.03
        self.inter_robot_clearance = 2.0 * robot_radius + 0.14
        self.h, self.w = self.data.shape
        self.clearance = ndimage.distance_transform_edt(
            np.pad(self.data < 65, 1, constant_values=False)
        )[1:-1, 1:-1] * res
        self.free_clearance = ndimage.distance_transform_edt(
            np.pad(self.data == 0, 1, constant_values=False)
        )[1:-1, 1:-1] * res

    def cell(self, p):
        return (
            int(math.floor((p[1] - self.origin[1]) / self.res)),
            int(math.floor((p[0] - self.origin[0]) / self.res)),
        )

    def point(self, c):
        return (
            self.origin[0] + (c[1] + 0.5) * self.res,
            self.origin[1] + (c[0] + 0.5) * self.res,
        )

    def inside(self, c):
        return 0 <= c[0] < self.h and 0 <= c[1] < self.w

    def allowed(self, region=None, others=(), radius=None, other_clearance=None):
        if radius is None:
            radius = self.clearance_radius
        ok = (self.data == 0) & (self.free_clearance >= radius)
        yy, xx = np.indices(ok.shape)
        x = self.origin[0] + (xx + 0.5) * self.res
        y = self.origin[1] + (yy + 0.5) * self.res
        if region is not None:
            ok &= (
                (x >= region[0] + self.region_clearance)
                & (x <= region[2] - self.region_clearance)
                & (y >= region[1] + self.region_clearance)
                & (y <= region[3] - self.region_clearance)
            )
        for p in others:
            ok &= (
                (x - p[0]) ** 2 + (y - p[1]) ** 2
                > (self.inter_robot_clearance if other_clearance is None else other_clearance)**2
            )
        return ok

    def astar(self, start, goal, ok):
        s, t = self.cell(start), self.cell(goal)
        if not self.inside(s) or not self.inside(t) or not ok[t]:
            return None
        if not ok[s]:
            # Only permit a small initial clearance relaxation, never tunnel through a wall.
            if self.data[s] != 0 or self.clearance[s] < 0.19:
                return None
        queue = [(0.0, 0.0, s)]
        cost = {s: 0.0}
        parent = {}
        seen = set()
        while queue:
            _, g, u = heapq.heappop(queue)
            if u in seen:
                continue
            seen.add(u)
            if u == t:
                path = [u]
                while u != s:
                    u = parent[u]
                    path.append(u)
                return g * self.res, [self.point(c) for c in path[::-1]]
            for dy, dx in [
                (0, 1),
                (0, -1),
                (1, 0),
                (-1, 0),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]:
                v = (u[0] + dy, u[1] + dx)
                if not self.inside(v) or not ok[v]:
                    continue
                if dy and dx and (not ok[u[0], v[1]] or not ok[v[0], u[1]]):
                    continue
                ng = g + (1.41421356237 if dx and dy else 1.0)
                if ng < cost.get(v, float("inf")):
                    cost[v] = ng
                    parent[v] = u
                    heapq.heappush(
                        queue, (ng + math.hypot(v[0] - t[0], v[1] - t[1]), ng, v)
                    )
        return None

    def visible_unknown(self, cell, sensor_range=3.0):
        """Expected unknown area visible before the first mapped obstacle.

        Ray marching is used only to rank viewpoints; navigation still requires
        an entirely observed safe path. Unknown cells never become traversable.
        """
        if getattr(self, "_view_range", None) != sensor_range:
            self._view_range = sensor_range
            angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
            steps = np.arange(1, int(math.ceil(sensor_range / self.res)) + 1)
            self._view_rays = (np.rint(np.sin(angles[:, None]) * steps).astype(int),
                               np.rint(np.cos(angles[:, None]) * steps).astype(int))
        dy, dx = self._view_rays
        yy, xx = cell[0] + dy, cell[1] + dx
        inside = (yy >= 0) & (yy < self.h) & (xx >= 0) & (xx < self.w)
        values = self.data[np.clip(yy, 0, self.h - 1), np.clip(xx, 0, self.w - 1)]
        blocked = ~inside | (values >= 65)
        visible = ~np.maximum.accumulate(blocked, axis=1)
        ids = np.unique((yy * self.w + xx)[visible & (values < 0)])
        return len(ids) * self.res**2

    def frontiers(
        self, start, region, reserved=(), blacklist=(), visits=(), min_travel=False,
        target_mask=None, others=(),
    ):
        ok = self.allowed(region, others=others,
                          other_clearance=max(0.8, self.inter_robot_clearance))
        start_cell = self.cell(start)
        if not self.inside(start_cell):
            return None
        # Filter connectivity BEFORE ranking. A large unreachable pocket must
        # not occupy every slot in the old top-12 candidate shortlist.
        labels, _ = ndimage.label(ok)
        roots = {int(labels[start_cell])} - {0}
        if not roots and self.data[start_cell] == 0 and self.clearance[start_cell] >= 0.19:
            for dy, dx in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                cell = (start_cell[0] + dy, start_cell[1] + dx)
                if self.inside(cell) and labels[cell]:
                    roots.add(int(labels[cell]))
        ok &= np.isin(labels, list(roots))
        unknown = self.data < 0
        if not unknown.any():
            return None
        near = ndimage.distance_transform_edt(~unknown) * self.res
        band = ok & (near < 1.0)
        if target_mask is not None:
            band &= target_mask
        stride = max(1, int(round(0.30 / self.res)))
        yy, xx = np.indices(band.shape)
        sample = band & (yy % stride == 0) & (xx % stride == 0)
        # Also retain a representative of every thin/small frontier component;
        # a fixed sampling lattice alone can miss a corner entirely.
        pieces, _ = ndimage.label(band)
        for label, bounds in enumerate(ndimage.find_objects(pieces), 1):
            if bounds is None or np.any(sample[bounds] & (pieces[bounds] == label)):
                continue
            cells = np.argwhere(pieces[bounds] == label)
            if len(cells):
                cell = cells[len(cells) // 2] + [bounds[0].start, bounds[1].start]
                sample[tuple(cell)] = True
        coords = np.argwhere(sample)
        active_blacklist = [q for q in blacklist if len(q) < 4 or q[3] > time.monotonic()]
        candidates = []
        for cy, cx in coords:
            p = self.point((cy, cx))
            dist = math.dist(start[:2], p)
            if (
                dist < 0.35
                or any(
                    math.dist(p, q[:2]) < max(0.65, self.inter_robot_clearance)
                    for q in reserved
                )
                or any(math.dist(p, q[:2]) < 0.55 for q in active_blacklist)
            ):
                continue
            gain = self.visible_unknown((cy, cx))
            if gain < 0.015:
                continue
            novelty = min([math.dist(p, q[:2]) for q in visits[-16:]] or [3.0])
            score = (3.0 * math.log1p(gain) + min(novelty, 2.0) * 0.5
                     - 1.5 * math.exp(-(novelty / 0.5)**2) - dist * 0.10)
            candidates.append((score, p, gain))
        # If frontiers are exhausted before a robot has participated, choose a new
        # sensor viewpoint in its *observed* connected free area, never a fixed patrol.
        if not candidates and min_travel:
            for cy in range(3, self.h, 12):
                for cx in range(3, self.w, 12):
                    if not ok[cy, cx]:
                        continue
                    p = self.point((cy, cx))
                    dist = math.dist(start[:2], p)
                    if dist < 1.2 or any(math.dist(p, q[:2]) < 0.55 for q in active_blacklist):
                        continue
                    novelty = min([math.dist(p, q[:2]) for q in visits] or [3.0])
                    candidates.append(
                        (
                            min(novelty, 3.0) * 0.3
                            - dist * 0.03,
                            p,
                            0.0,
                        )
                    )
        ranked = sorted(candidates, reverse=True)
        selected = []
        for score, p, gain in ranked:
            if any(
                math.dist(p, q[1]) < max(0.8, self.inter_robot_clearance + 0.2)
                for q in selected
            ):
                continue
            selected.append((score, p, gain))
            if len(selected) >= 16:
                break
        best = None
        for score, p, gain in selected:
            route = self.astar(start, p, ok)
            if route and (best is None or score - route[0] * 0.08 > best[0]):
                best = (score - route[0] * 0.08, p, route,
                        {"visible_unknown_m2": gain, "route_length": route[0],
                         "viewpoints": len(candidates)})
        return best

    def region_escape(self, start, region, reserved=(), blacklist=(), visits=()):
        """Find a live-map frontier whose known-safe route needs a region exit."""
        cell = self.cell(start)
        if not self.inside(cell):
            return None
        labels, _ = ndimage.label(self.allowed(region))
        component = labels[cell]
        locally_reachable = labels == component if component else np.zeros(labels.shape, bool)
        yy, xx = np.indices(labels.shape)
        x = self.origin[0] + (xx + 0.5) * self.res
        y = self.origin[1] + (yy + 0.5) * self.res
        blocked = (~locally_reachable & (x >= region[0]) & (x <= region[2])
                   & (y >= region[1]) & (y <= region[3]))
        return self.frontiers(start, None, reserved, blacklist, visits, target_mask=blocked)


def plan_formation(grid, poses, slots):
    best = None
    for assign in itertools.permutations(range(3)):
        for order in itertools.permutations(range(3)):
            current = [tuple(p) for p in poses]
            routes = []
            total = 0.0
            for i in order:
                target = slots[assign[i]]
                if math.dist(current[i][:2], target[:2]) < 0.08:
                    routes.append((i, target, []))
                    current[i] = target
                    continue
                result = grid.astar(
                    current[i],
                    target,
                    grid.allowed(others=[p for j, p in enumerate(current) if j != i]),
                )
                if result is None:
                    break
                total += result[0]
                routes.append((i, target, result[1]))
                current[i] = target
            if len(routes) == 3 and (best is None or total < best[0]):
                best = (total, assign, routes)
    if best:
        return best
    # Resolve a cycle with one observed, safe temporary holding point.
    ok = grid.allowed()
    yy, xx = np.where(ok)
    samples = [grid.point((y, x)) for y, x in zip(yy[::160], xx[::160])]
    for mover in range(3):
        others = [p for j, p in enumerate(poses) if j != mover]
        for p in sorted(samples, key=lambda p: math.dist(p, poses[mover][:2]))[:40]:
            if (
                min(math.dist(p, q[:2]) for q in list(poses) + list(slots))
                < max(0.8, grid.inter_robot_clearance + 0.2)
            ):
                continue
            r = grid.astar(poses[mover], p, grid.allowed(others=others))
            if not r:
                continue
            moved = list(poses)
            moved[mover] = (p[0], p[1], poses[mover][2])
            # Enumerate directly here to bound staging to exactly one temporary move.
            for assign in itertools.permutations(range(3)):
                for order in itertools.permutations(range(3)):
                    current = list(moved)
                    routes = [(mover, moved[mover], r[1])]
                    total = r[0]
                    for i in order:
                        target = slots[assign[i]]
                        path = grid.astar(
                            current[i],
                            target,
                            grid.allowed(
                                others=[q for j, q in enumerate(current) if j != i]
                            ),
                        )
                        if path is None:
                            break
                        routes.append((i, target, path[1]))
                        total += path[0]
                        current[i] = target
                    if len(routes) == 4:
                        return total, assign, routes
    return None
