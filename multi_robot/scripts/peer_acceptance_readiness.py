"""A test precondition, never a replacement for robot motion authorization."""
import math
import time
from fleet_control.peer_protocol import MEMBERS, finite_pose


class ReadinessHold:
    def __init__(self, owners, *, all_members=False, stationary=True, seconds=1.):
        self.owners = set(owners)
        self.all_members, self.stationary, self.seconds = all_members, stationary, seconds
        self.since, self.identity, self.details = None, None, {}

    def update(self, state, now):
        robots, frame = state.get('robots', {}), state.get('coordinate_frame', '')
        required = {'robot1', 'robot2', 'robot3'} if self.all_members else self.owners
        good, missing = {}, []
        for name, robot in robots.items():
            pose = robot.get('pose')
            if (robot.get('ready') and robot.get('aligned') and robot.get('boot_id')
                    and robot.get('map_epoch', 0) > 0 and 0 <= robot.get('state_age', math.inf) < .8
                    and frame and robot.get('frame_epoch') == frame
                    and len(robot.get('connected_members', [])) >= 2
                    and isinstance(pose, (list, tuple)) and len(pose) == 3
                    and all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose)):
                good[name] = robot
        missing.extend(name+':not_ready_aligned_fresh' for name in sorted(required-set(good)))
        if len(good) < 2:
            missing.append('no_ready_quorum')
        if state.get('estop'):
            missing.append('estop')
        if self.stationary:
            for name in sorted(required & set(robots)):
                robot, safety = robots[name], robots[name].get('safety', {})
                speeds = [safety.get('v'), safety.get('w')]
                if (robot.get('goal') is not None or any(not isinstance(v, (int, float))
                        or not math.isfinite(v) or abs(v) >= .001 for v in speeds)):
                    missing.append(name+':not_stationary')
        identity = (frame, tuple(sorted((n, r['boot_id'], r['map_epoch']) for n, r in good.items())))
        if missing:
            self.since, self.identity = None, None
        elif identity != self.identity:
            self.since, self.identity = now, identity
        held = now-self.since if self.since is not None else 0.
        self.details = dict(missing=missing, ready_members=sorted(good), owners=sorted(self.owners),
                            all_members=self.all_members, coordinate_frame=frame, held_seconds=held)
        return not missing and held >= self.seconds


class RecoverySpacingHold(ReadinessHold):
    """Pause an existing recovery mission only with room for later staging.

    This grants no motion and chooses no goal. Independent guards remain in
    charge while exploration continues. Recheck the original maps, 0.90 m
    routes and bounded-retreat rules after the pause has actually completed.
    """
    def __init__(self):
        super().__init__(MEMBERS, all_members=True, stationary=False, seconds=1.)

    def update(self, state, now):
        ready = super().update(state, now)
        robots = state.get('robots', {})
        valid = (set(robots) == set(MEMBERS) and all(r.get('nav_ready') and finite_pose(r.get('pose'))
            and isinstance(r.get('pose_steady_time'), (int, float))
            and not isinstance(r.get('pose_steady_time'), bool)
            and math.isfinite(r['pose_steady_time']) and 0. <= now-r['pose_steady_time'] <= .8
            for r in robots.values()))
        minimum = (min(math.dist(robots[a]['pose'][:2], robots[b]['pose'][:2])
            for i, a in enumerate(MEMBERS) for b in MEMBERS[:i]) if valid else None)
        if minimum is None or minimum < 1.5:
            self.since, self.identity = None, None
            self.details['missing'].append('no_fresh_recovery_spacing')
            ready = False
        self.details.update(minimum_separation=minimum, required_separation=1.5)
        return ready


def stationary_patrol_state(api, *, timeout=60., clock=time.monotonic):
    """A cold client must receive all current poses before filling UI routes."""
    hold = ReadinessHold(MEMBERS, all_members=True)
    deadline = clock()+timeout
    while clock() < deadline:
        state = api.state(.2)
        now = clock()
        if now >= deadline:
            break
        robots = state.get('robots', {})
        if (not state.get('team_ready') or set(robots) != set(MEMBERS)
                or not all(finite_pose(r.get('pose')) for r in robots.values())):
            hold.update({}, now)
            continue
        if hold.update(state, now):
            return state
    raise TimeoutError('三车当前相对位姿未在时限内连续静止就绪：'+str(hold.details))
