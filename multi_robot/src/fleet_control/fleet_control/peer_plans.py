"""Independent validation of another member's static formation proposal."""
import math
import numpy as np
from fleet_mapping.registration import compose, inverse, transform_points
from .geometry import formation
from .peer_protocol import MEMBERS, finite_pose, digest


def agreed_formation_plan(proposals, votes, boots):
    """Choose one immutable, unanimously validated plan in a common order.

    A later proposal cannot win while an earlier proposal still lacks ballots.
    Validation ballots may approve several feasible plans; they grant no motion.
    The separate intersecting motion leases still serialize route execution.
    """
    if set(proposals) != set(MEMBERS) or set(boots) != set(MEMBERS):
        return None
    for proposer in MEMBERS:
        plan = proposals[proposer]
        if plan is None:
            continue
        plan_id = digest(plan)
        ballots = votes.get(plan_id, {})
        if any(n not in ballots or ballots[n][0] != boots[n] for n in MEMBERS):
            return None
        if all(ballots[n][1] for n in MEMBERS):
            return plan_id, plan
    return None


def validate_formation_plan(plan, command, poses, grid, team_from_local, clearance=.85):
    try:
        expected = formation(command['shape'], command['x'], command['y'],
                             command['yaw'], command['spacing'])
        slots, assignment, routes = plan['slots'], plan['assignment'], plan['routes']
        if (set(poses) != set(MEMBERS) or len(slots) != 3 or sorted(assignment) != [0, 1, 2]
                or not 3 <= len(routes) <= 4 or not all(finite_pose(p) for p in slots)
                or any(math.dist(p, q) > 1e-6 for p, q in zip(slots, expected))):
            return False
        current = [poses[n] for n in MEMBERS]
        allowed = grid.allowed()
        local_from_team = inverse(team_from_local)
        for row in routes:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                return False
            owner, target, path = row
            if (not isinstance(owner, int) or isinstance(owner, bool) or not 0 <= owner < 3
                    or not finite_pose(target) or not isinstance(path, list) or len(path) > 20000):
                return False
            points = np.asarray(path, dtype=float) if path else np.empty((0, 2))
            if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
                return False
            points = np.vstack([np.asarray(current[owner][:2]), points, np.asarray(target[:2])])
            samples = []
            for start, end in zip(points, points[1:]):
                count = max(1, int(math.ceil(np.linalg.norm(end-start)/grid.res)))
                if count > 2000:
                    return False
                samples.extend(np.linspace(start, end, count+1))
            if len(samples) > 50000:
                return False
            sample = np.asarray(samples)
            for i, peer in enumerate(current):
                if i != owner and np.min(np.linalg.norm(sample-peer[:2], axis=1)) < clearance:
                    return False
            local = transform_points(local_from_team, sample)
            for point in local:
                cell = grid.cell(point)
                if not grid.inside(cell) or not allowed[cell]:
                    return False
            current[owner] = target
        return all(math.dist(current[i], slots[assignment[i]]) < .01 for i in range(3))
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return False
