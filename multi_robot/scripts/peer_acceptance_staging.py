"""Observed goals near fixed seeds for the fault harness, never agent control."""
import itertools
import math
import numpy as np
from scipy import ndimage
from fleet_control.peer_protocol import MEMBERS, finite_pose


def staging_plan(grids, poses, seeds, *, radius=.75, separation=3., peer_clearance=.9):
    """Return a safe ordered list of three owner/goal pairs, or no plan.

    Seeds remain unchanged. Every goal and route uses its executing owner's
    map in the common relative frame. Earlier arrivals become peer obstacles
    for the remaining legs; a member occupying another seed can leave first.
    """
    if (any(set(value) != set(MEMBERS) for value in (grids, poses, seeds))
            or any(not finite_pose(p) for value in (poses, seeds) for p in value.values())
            or any(not math.isfinite(v) or v <= 0. for v in (radius, separation, peer_clearance))):
        return None
    candidates = {}
    for owner in MEMBERS:
        grid, seed = grids[owner], seeds[owner]
        endpoints = grid.allowed() & (grid.free_clearance >= grid.clearance_radius+2*grid.res)
        near = (grid.world_x-seed[0])**2+(grid.world_y-seed[1])**2
        yy, xx = np.where(endpoints & (near <= radius**2))
        order = np.argsort(near[yy, xx], kind='stable')
        points = [[*grid.point((yy[i], xx[i])), seed[2]] for i in order]
        cell = grid.cell(seed)
        if grid.inside(cell) and endpoints[cell]:
            points.insert(0, list(seed))
        if not points:
            return None
        candidates[owner] = points
    for order in itertools.permutations(MEMBERS):
        current, targets, legs = dict(poses), {}, []
        for owner in order:
            grid = grids[owner]
            allowed = grid.allowed(others=[p for n, p in current.items() if n != owner],
                                   other_clearance=peer_clearance)
            root = grid.cell(current[owner])
            if (not grid.inside(root) or (not allowed[root]
                    and (grid.data[root] != 0 or grid.clearance[root] < .19))):
                break
            # Four-connected reachability rejects disconnected candidates in
            # one pass. A* still validates each selected leg and its exact
            # start-escape/corner rules; this mask grants no navigation right.
            connected = allowed.copy()
            connected[root] = True
            labels, _ = ndimage.label(connected)
            component = labels[root]
            for target in candidates[owner]:
                cell = grid.cell(target)
                if not allowed[cell] or labels[cell] != component:
                    continue
                if any(math.dist(target[:2], p[:2]) < separation for p in targets.values()):
                    continue
                if grid.astar(current[owner], target, allowed) is not None:
                    targets[owner] = target
                    current[owner] = target
                    legs.append((owner, target))
                    break
            else:
                break
        if len(legs) == len(MEMBERS):
            return legs
    return None


def release_segment_clear(grid, start, target, others, *, peer_clearance=.9, maximum_length=1.):
    """A bounded departure from an already-close pair, never through geometry.

    The ordinary peer mask remains unchanged. Only the initial close pair may
    start within that conservative mask: its separation cannot decrease along
    this straight segment. Once outside, the original staging planner applies.
    """
    others = list(others)
    if (not finite_pose(start) or not finite_pose(target)
            or any(not finite_pose(p) for p in others)
            or any(not math.isfinite(v) or v <= 0. for v in (peer_clearance, maximum_length))):
        return False
    delta = np.asarray(target[:2], float)-start[:2]
    length = float(np.linalg.norm(delta))
    if length < .05 or length > maximum_length+1e-9:
        return False
    close = False
    for peer in others:
        offset = np.asarray(start[:2], float)-peer[:2]
        distance = float(np.linalg.norm(offset))
        if (distance < grid.inter_robot_clearance+.10
                or math.dist(target[:2], peer[:2]) < peer_clearance+.30):
            return False
        projection = float(np.dot(offset, delta))
        if distance < peer_clearance:
            close = True
            if projection < -1e-9:
                return False
        else:
            t = max(0., min(1., -projection/(length*length)))
            if float(np.linalg.norm(offset+t*delta)) < peer_clearance:
                return False
    if not close:
        return False
    allowed = grid.allowed()
    end = grid.cell(target)
    if (not grid.inside(end)
            or grid.free_clearance[end] < grid.clearance_radius+2*grid.res):
        return False
    for t in np.linspace(0., 1., int(math.ceil(length/(grid.res*.5)))+1):
        cell = grid.cell(np.asarray(start[:2])+t*delta)
        if not grid.inside(cell) or not allowed[cell]:
            return False
    return True


def staging_release(grids, poses, seeds, *, peer_clearance=.9):
    """At most one observed retreat, with a strict three-leg plan afterwards."""
    if (any(set(value) != set(MEMBERS) for value in (grids, poses, seeds))
            or any(not finite_pose(p) for value in (poses, seeds) for p in value.values())
            or not math.isfinite(peer_clearance) or peer_clearance <= 0.):
        return None
    if staging_plan(grids, poses, seeds, peer_clearance=peer_clearance) is not None:
        return None
    for owner in MEMBERS:
        grid, start = grids[owner], poses[owner]
        others = {n: p for n, p in poses.items() if n != owner}
        if not any(math.dist(start[:2], p[:2]) < peer_clearance for p in others.values()):
            continue
        cell = grid.cell(start)
        if not grid.inside(cell) or not grid.allowed()[cell]:
            continue
        endpoints = grid.allowed() & (grid.free_clearance >= grid.clearance_radius+2*grid.res)
        near = (grid.world_x-start[0])**2+(grid.world_y-start[1])**2
        yy, xx = np.where(endpoints & (near <= 1.))
        for i in np.argsort(near[yy, xx], kind='stable'):
            x, y = grid.point((yy[i], xx[i]))
            target = [x, y, math.atan2(y-start[1], x-start[0])]
            if any(math.dist(target[:2], p[:2]) < peer_clearance+.35 for p in others.values()):
                continue
            if not release_segment_clear(grid, start, target, others.values(), peer_clearance=peer_clearance):
                continue
            moved = dict(poses)
            moved[owner] = target
            plan = staging_plan(grids, moved, seeds, peer_clearance=peer_clearance)
            if plan is not None:
                return dict(owner=owner, start=list(start), target=target, others=others,
                    initial_separations={n: math.dist(start[:2], p[:2]) for n, p in others.items()},
                    peer_clearance=peer_clearance, maximum_length=1., normal_plan=plan)
    return None


def release_observation_valid(poses, release):
    """Bound actual reported departure; the driver pauses on any violation."""
    owner, start, target = release['owner'], release['start'], release['target']
    if set(poses) != set(MEMBERS) or any(not finite_pose(p) for p in poses.values()):
        return False
    delta = np.asarray(target[:2])-start[:2]
    squared = float(np.dot(delta, delta))
    if squared <= 0.:
        return False
    displacement = np.asarray(poses[owner][:2])-start[:2]
    along = float(np.dot(displacement, delta)/squared)
    nearest = np.asarray(start[:2])+max(0., min(1., along))*delta
    if np.linalg.norm(np.asarray(poses[owner][:2])-nearest) > .12:
        return False
    for name, before in release['others'].items():
        if math.dist(poses[name][:2], before[:2]) > .05:
            return False
        minimum = min(release['peer_clearance'], release['initial_separations'][name])
        if math.dist(poses[owner][:2], poses[name][:2]) < minimum-.03:
            return False
    return True


def release_state_valid(state, release, identities, frame, now, *, state_elapsed=0.):
    """A retreat needs current stationary peers in the captured frame/cohort."""
    if (state.get('coordinate_frame') != frame or state.get('estop') or not math.isfinite(now)
            or not math.isfinite(state_elapsed) or state_elapsed < 0.):
        return False
    robots = state.get('robots', {})
    if set(robots) != set(MEMBERS) or set(identities) != set(MEMBERS):
        return False
    for name, robot in robots.items():
        if ((robot.get('boot_id'), robot.get('map_epoch')) != tuple(identities[name])
                or robot.get('frame_epoch') != frame
                or not all(robot.get(k) for k in ('ready', 'aligned', 'nav_ready'))):
            return False
        age, stamp = robot.get('state_age'), robot.get('pose_steady_time')
        if (any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v)
                for v in (age, stamp)) or not 0. <= age+state_elapsed <= .8 or not 0. <= now-stamp <= .8):
            return False
        if name != release['owner']:
            safety = robot.get('safety', {})
            if (robot.get('goal') or any(not isinstance(safety.get(k), (float, int)) or isinstance(safety[k], bool)
                    or not math.isfinite(safety[k]) or abs(safety[k]) > .001 for k in ('v', 'w'))):
                return False
    return release_observation_valid({n: r.get('pose') for n, r in robots.items()}, release)


def release_clock_delay(state, release, identities, frame, now):
    """Wait out at most 50 ms of TF/clock skew, retaining the original guard.

    The reported pose time is projected from independently delivered TF and
    simulation clocks, so it can lead the observer slightly. Only suggest a
    wait if this exact snapshot will satisfy every unchanged guard afterwards.
    The caller must recheck with actual monotonic time after one bounded wait.
    """
    robots = state.get('robots', {})
    if (set(robots) != set(MEMBERS) or not isinstance(now, (int, float))
            or isinstance(now, bool) or not math.isfinite(now)):
        return 0.
    stamps = [r.get('pose_steady_time') for r in robots.values()]
    if any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v)
           for v in stamps):
        return 0.
    newest = max(stamps)
    if (now < newest <= now+.05 and release_state_valid(state, release, identities, frame, newest,
                                                      state_elapsed=newest-now)):
        return newest-now
    return 0.
