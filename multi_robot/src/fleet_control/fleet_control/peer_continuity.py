"""Veto map aliases using previously aligned encoder observations, never truth.

    These checks do not create a map constraint or establish a common origin.
    A feature/geometric match and both endpoint confirmations remain required.
"""
import math
from fleet_mapping.registration import compose, inverse, angle
from .peer_protocol import finite_pose


def component_bodies_consistent(graph, source, target, transform, peers, now, robot_radius=.18):
    """Veto a proposed join against every currently observed body in both components.

    This is a negative physical check, never a source of alignment. Stale poses
    cannot prove a collision. Boot and epoch fences exclude retired local maps.
    """
    component = graph.poses(source)
    other = graph.poses(target)
    for key, pose in other.items():
        component.setdefault(key, compose(transform, pose))
    bodies = []
    for (robot, boot, epoch), pose in component.items():
        state = peers.get(robot)
        if (state and (state['boot_id'], state['map_epoch']) == (boot, epoch)
                and state.get('local_ready') and now-state.get('received', -math.inf) < .4
                and finite_pose(state.get('pose'))):
            bodies.append((robot, compose(pose, state['pose'])))
    return all(a == b or math.dist(pa[:2], pb[:2]) >= 2*robot_radius-.03
               for i, (a, pa) in enumerate(bodies) for b, pb in bodies[i+1:])


def constraint_consistent_with_odometry(source, target, transform, known, peers, priors, now, maximum_speed, robot_radius=.18):
    a, b = peers.get(source[0]), peers.get(target[0])
    if (a and b and (a['boot_id'], a['map_epoch']) == source[1:]
            and (b['boot_id'], b['map_epoch']) == target[1:]
            and a.get('local_ready') and b.get('local_ready')):
        if not finite_pose(a.get('pose')) or not finite_pose(b.get('pose')):
            return False
        if math.dist(a['pose'][:2], compose(transform, b['pose'])[:2]) < 2*robot_radius+.05:
            return False  # Identical rooms cannot put two physical bodies together.
    expected, active = {}, {}
    for key in (source, target):
        robot, boot, epoch = key
        state, prior = peers.get(robot), priors.get(robot)
        if not state or not prior or (state['boot_id'], state['map_epoch']) != (boot, epoch):
            continue
        active[key] = prior, state
        fresh = state.get('odom_valid') and state.get('local_ready')
        if not fresh:
            if prior['boot_id'] != boot:
                return False
            continue
        # A rigid encoder-to-team calibration is only fresh while alignment
        # observations keep updating it. Wheel slip accumulates during a long
        # unknown-origin observation run; an old calibration is not a timeless
        # 6-degree certificate. Keep the physical reachable-region veto below.
        age = max(0., now-prior.get('last_seen', now))
        if state.get('odom_epoch') == prior.get('odom_epoch') and age <= .8:
            if not finite_pose(state.get('odom_pose')) or not finite_pose(state.get('pose')):
                return False
            body = compose(prior['team_from_odom'], state['odom_pose'])
            expected[key] = compose(body, inverse(state['pose']))
    available = dict(known)
    available.update(expected)
    for own, other, relation in ((source, target, transform), (target, source, inverse(transform))):
        if own not in available or other not in active:
            continue
        prior, state = active[other]
        if not finite_pose(state.get('pose')):
            return False
        predicted = compose(compose(available[own], relation), state['pose'])
        if other in expected:
            required = compose(expected[other], state['pose'])
            tolerance = .15+min(.30, 3*max(0., prior.get('sigma', .05)))
            if (math.dist(predicted[:2], required[:2]) > tolerance
                    or abs(angle(predicted[2]-required[2])) > math.radians(6)):
                return False
        elif prior['boot_id'] != state['boot_id']:
            # A stale calibration or replaced encoder has no rigid certificate. Retain
            # the last reachable region as a weaker veto, without treating it
            # as an alignment or assuming the unobserved robot stayed stopped.
            radius = (.3+3*max(0., prior.get('sigma', .05))
                      +maximum_speed*max(0., now-prior.get('last_seen', now)))
            if math.dist(predicted[:2], prior['last_team_pose'][:2]) > radius:
                return False
    return True
