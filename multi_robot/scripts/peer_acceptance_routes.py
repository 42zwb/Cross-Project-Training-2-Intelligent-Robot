"""Select demonstrated routes and formation targets from the robots' maps."""
import math
import numpy as np
from fleet_control.geometry import Grid, formation, plan_formation
from fleet_control.peer_plans import validate_formation_plan
from fleet_control.peer_protocol import MEMBERS, finite_pose


def short_patrol(grid, pose, others=(), *, length=.6, other_clearance=.85):
    others = list(others)
    if (not finite_pose(pose) or any(not finite_pose(p) for p in others)
            or not math.isfinite(length) or length <= 0.):
        return None
    allowed = grid.allowed(others=others, other_clearance=other_clearance)
    # A* may escape a start just below its ordinary clearance requirement.
    # That exception does not make the same point a legal return goal. Prefer
    # endpoints two raster cells inside observed free space, then check every
    # leg using the unchanged path mask and start-escape rule.
    endpoints = allowed & (grid.free_clearance >= grid.clearance_radius+2*grid.res)
    start = grid.cell(pose)
    if not grid.inside(start):
        return None
    anchors = []
    if endpoints[start]:
        anchors.append(list(pose))
    yy, xx = np.where(endpoints & ((grid.world_x-pose[0])**2+(grid.world_y-pose[1])**2 <= 1.))
    order = np.argsort((grid.world_x[yy, xx]-pose[0])**2+(grid.world_y[yy, xx]-pose[1])**2,
                      kind='stable')
    anchors.extend([*grid.point((yy[i], xx[i])), pose[2]] for i in order[:64])
    for anchor in anchors:
        if grid.astar(pose, anchor, allowed) is None:
            continue
        for theta in np.linspace(0., 2*math.pi, 24, endpoint=False):
            target = [anchor[0]+length*math.cos(theta), anchor[1]+length*math.sin(theta), pose[2]]
            cell = grid.cell(target)
            if (grid.inside(cell) and endpoints[cell]
                    and grid.astar(pose, target, allowed) is not None
                    and grid.astar(target, anchor, allowed) is not None):
                return [target, anchor]
    return None


def formation_target(replicas, poses, shape):
    """Choose a request with a feasible route across the current map replicas."""
    reference = replicas[MEMBERS[0]]
    start = [poses[name] for name in MEMBERS]
    centre = np.mean([pose[:2] for pose in start], axis=0)
    stride = max(1, round(.25/reference.res))
    cells = np.argwhere(reference.allowed()[::stride, ::stride])*stride
    # Match the extra .12 m used by each robot when proposing a formation path.
    planning = [Grid(grid.data, grid.res, grid.origin, grid.robot_radius+.12, grid.region_margin)
                for grid in replicas.values()]
    candidates = []
    for cell in cells:
        x, y = reference.point(cell)
        slots = formation(shape, x, y, 0., 1.2)
        endpoints = [(grid, grid.cell(slot)) for grid in planning for slot in slots]
        if not all(grid.inside(c) and grid.data[c] == 0
                   and grid.free_clearance[c] >= grid.clearance_radius for grid, c in endpoints):
            continue
        room = min(grid.free_clearance[c]-grid.clearance_radius for grid, c in endpoints)
        candidates.append((room < .10, math.dist((x, y), centre), x, y, slots))
    # Prefer a little room around parked vehicles before the next transition.
    # The robot-side checks still decide whether to accept the actual request.
    for _, _, x, y, slots in sorted(candidates)[:50]:
        command = dict(shape=shape, x=x, y=y, yaw=0., spacing=1.2)
        for grid in planning:
            result = plan_formation(grid, start, slots, peer_clearance=1.0)
            if result is None:
                continue
            _, assignment, routes = result
            plan = dict(slots=slots, assignment=assignment, routes=routes)
            if all(validate_formation_plan(plan, command, poses, replica, (0., 0., 0.))
                   for replica in replicas.values()):
                return command
    return None
