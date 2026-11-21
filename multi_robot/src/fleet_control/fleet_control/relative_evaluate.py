"""Offline-only truth evaluation; never imported by a robot control node.

The denominator is the complete physical arena. The gauge is established once
from the first initialized anchor sample, never an optimized map/trajectory fit.
"""
import argparse
from itertools import combinations
import json
import math
from pathlib import Path
import numpy as np
from scipy import ndimage
import yaml
from fleet_mapping.grid import fuse, load_map
from fleet_mapping.registration import angle, compose, inverse, transform_points
from fleet_mapping.shapes import shape_distance
from .geometry import formation_pass
from .peer_protocol import MEMBERS, digest

COLLISION_CLEARANCE_LIMIT = -.001
COLLISION_SAMPLE_GAP_LIMIT = .25


def full_arena_coverage(data, resolution, origin, world_from_team, arena, obstacles,
                        starts, seeds, regions=None, robot_radius=.18):
    x0, y0, x1, y1 = arena
    shape = (int(round((y1-y0)/resolution)), int(round((x1-x0)/resolution)))
    observed = fuse([(data, resolution, origin[0], origin[1], 0., world_from_team)],
                    shape, resolution, (x0, y0))
    yy, xx = np.indices(shape)
    x, y = x0+(xx+.5)*resolution, y0+(yy+.5)*resolution
    blocked = np.zeros(shape, bool)
    for obstacle in obstacles:
        blocked |= shape_distance(x, y, obstacle) <= 0.
    # Padding prevents the raster distance transform from treating outside the
    # physical arena as open space, including scenes whose walls are thin.
    distance = ndimage.distance_transform_edt(np.pad(~blocked, 1))[1:-1, 1:-1]*resolution
    allowed = distance >= robot_radius+.02
    labels, _ = ndimage.label(allowed)
    reachable = np.zeros(shape, bool)
    for start in starts:
        cx, cy = int((start[0]-x0)/resolution), int((start[1]-y0)/resolution)
        if not (0 <= cy < shape[0] and 0 <= cx < shape[1]):
            raise ValueError('独立真值起点位于完整场地之外')
        label = labels[cy, cx]
        if label:
            reachable |= labels == label
    if not reachable.any():
        raise ValueError('完整场地中没有从真实起点可达的自由空间')
    team = transform_points(inverse(world_from_team), np.column_stack([x.ravel(), y.ravel()]))
    masks = []
    if regions:
        for b in regions:
            masks.append(((team[:, 0] >= b[0]) & (team[:, 0] <= b[2])
                          & (team[:, 1] >= b[1]) & (team[:, 1] <= b[3])).reshape(shape))
    else:
        if set(seeds) != set(MEMBERS):
            raise ValueError('缺少初始化时固定的三车分区种子')
        distances = np.stack([np.sum((team-seeds[n][:2])**2, axis=1) for n in MEMBERS])
        owner = np.argmin(distances, axis=0).reshape(shape)
        masks = [owner == i for i in range(3)]
    regional = [reachable & m for m in masks]
    ratios = [float(np.sum(m & (observed == 0))/max(1, m.sum())) for m in regional]
    ratio = float(np.sum(reachable & (observed == 0))/reachable.sum())
    return dict(coverage=ratio, region_coverage=ratios, reachable_cells=int(reachable.sum()),
                region_reachable_cells=[int(m.sum()) for m in regional],
                coverage_pass=ratio >= .90 and min(ratios) >= .80,
                denominator='complete_physical_arena', arena=list(arena))


def rows(path):
    lines = Path(path).read_text().splitlines(keepends=True)
    # A recorder may currently be writing its final line. A malformed completed
    # line is still an error; only an unfinished EOF record is omitted.
    return [json.loads(line) for line in lines if line.strip() and line.endswith('\n')]


def identified_formation(slots):
    if len(slots) != 3:
        return None
    xy = np.asarray(slots, float)[:, :2]
    yaw = slots[0][2]
    relative = transform_points((0., 0., -yaw), xy-xy.mean(axis=0))
    spread = np.ptp(relative, axis=0)
    if spread[0] < .001 and spread[1] > .1:
        return 'row'
    if spread[1] < .001 and spread[0] > .1:
        return 'column'
    distances = [math.dist(xy[i], xy[j]) for i in range(3) for j in range(i)]
    return 'triangle' if min(distances) > .1 and max(distances)-min(distances) < .001 else None


def association_clock(telemetry, truth):
    clock = 'steady_time' if all('steady_time' in r for r in telemetry+truth) else 'sim_time'
    for stream in (telemetry, truth):
        values = [row[clock] for row in stream]
        if not np.isfinite(values).all() or np.any(np.diff(values) < 0):
            raise ValueError('评估时间不单调，不能按重复仿真时间关联不同的物理时刻；需要单调时钟记录')
    return clock


def formation_completions(run, telemetry, clock='sim_time'):
    result = {}
    for i, sample in enumerate(telemetry):
        if (sample.get('mode') != 'FORMATION_READY' or not sample.get('formation_slots')
                or (i and telemetry[i-1].get('mode') == 'FORMATION_READY')):
            continue
        key = digest([sample['formation_slots'], sample.get('formation_assignment')])
        started = next((telemetry[j][clock] for j in range(i-1, -1, -1)
                        if telemetry[j].get('mode') != 'FORMING'), sample[clock])
        result[key] = dict(sample, formation_seconds=sample[clock]-started,
                           completion_evidence='three_member_telemetry')
    # A brief ready state can fall between two telemetry samples. Each agent's
    # durable task event contains the same slots and an exact simulation time;
    # require all three independent completion records, then still verify the
    # entire physical holding window against truth below.
    groups = {}
    for name in MEMBERS:
        path = Path(run)/name/'events.jsonl'
        if not path.exists():
            continue
        started = None
        for event in rows(path):
            if clock not in event:
                continue
            if event.get('event') == 'command_committed' and event.get('command') == 'formation':
                started = event[clock]
            if event.get('event') == 'formation_ready' and started is not None:
                key = digest([event['slots'], event['assignment']])
                groups.setdefault(key, {})[name] = dict(event, seconds=event[clock]-started)
    for key, events in groups.items():
        if set(events) == set(MEMBERS):
            latest = max(events.values(), key=lambda e: e[clock])
            result[key] = dict(sim_time=latest['sim_time'], formation_slots=latest['slots'],
                formation_assignment=latest['assignment'], formation_seconds=max(e['seconds'] for e in events.values()),
                completion_evidence='three_agent_durable_events')
            result[key][clock] = latest[clock]
    return sorted(result.values(), key=lambda value: value[clock])


def durable_exploration_completion(run, telemetry, clock='sim_time', frame_epoch=None, mission_id=None):
    """Recover a missed joint state from three overlapping completion intervals.

    Only the initial common exploration mission and its original boot cohort
    count. New work invalidates a local completion, and any subsequent command
    ends the interval. A later recovery task cannot complete an earlier one.
    """
    def finite(value):
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))

    ready = next((s for s in telemetry if s.get('team_ready')), None)
    if ready is None or not finite(ready.get(clock)):
        return None
    frame = frame_epoch or ready.get('coordinate_frame')
    if not frame or ready.get('coordinate_frame') != frame:
        return None
    records, starts = {}, {}
    for name in MEMBERS:
        path = Path(run)/name/'events.jsonl'
        if not path.exists():
            return None
        records[name] = rows(path)
        if any(not finite(e.get(clock)) for e in records[name]):
            return None
        # Different callback threads may append out of order. The validated
        # observation clock, rather than file append order, defines intervals.
        records[name].sort(key=lambda e: e[clock])
        start = next((e for e in records[name] if e.get('event') == 'command_committed'
                      and e.get('command') == 'explore'
                      and (mission_id is None or e.get('mission_id') == mission_id)), None)
        if (not start or not finite(start.get(clock)) or start.get('robot') != name
                or not start.get('mission_id') or start.get('command_id') != start['mission_id']):
            return None
        peer = ready.get('robots', {}).get(name, {})
        identity = (start.get('boot_id'), start.get('map_epoch'))
        if not identity[0] or identity != (peer.get('boot_id'), peer.get('map_epoch')):
            return None
        starts[name] = start
    missions = {s['mission_id'] for s in starts.values()}
    if len(missions) != 1 or not telemetry or not finite(telemetry[-1].get(clock)):
        return None
    mission = missions.pop()
    end = telemetry[-1][clock]
    transitions = []
    for name, events in records.items():
        start = starts[name]
        for event in events[events.index(start)+1:]:
            stamp = event[clock]
            if ((event.get('boot_id'), event.get('map_epoch')) !=
                    (start['boot_id'], start['map_epoch'])
                    or event.get('mission_id') != mission
                    or event.get('event') == 'command_committed'):
                end = min(end, stamp)
                break
            kind = event.get('event')
            if kind == 'exploration_complete':
                transitions.append((stamp, name, True))
            elif kind in ('frontier_claim', 'goal_sent', 'alignment_ready', 'exploration_regions_released'):
                transitions.append((stamp, name, False))
    completed = {}
    # If an invalidation shares a clock tick with completion, retain the safer
    # interpretation: it invalidates the completion rather than authorizing it.
    for stamp, name, done in sorted(transitions, key=lambda e: (e[0], 0 if e[2] else 1)):
        if stamp >= end:
            continue
        if done:
            completed[name] = stamp
        else:
            completed.pop(name, None)
    if set(completed) != set(MEMBERS) or min(completed.values()) < ready[clock]:
        return None
    for name, stamp in completed.items():
        def context(sample):
            peer = sample.get('robots', {}).get(name, {})
            safety = peer.get('safety', {})
            return (finite(sample.get(clock)) and abs(sample[clock]-stamp) <= .8
                and sample.get('coordinate_frame') == frame
                and peer.get('frame_epoch') == frame and peer.get('ready') and peer.get('aligned')
                and (peer.get('boot_id'), peer.get('map_epoch')) ==
                    (starts[name]['boot_id'], starts[name]['map_epoch'])
                and peer.get('mission_id') == mission and peer.get('goal') is None
                and all(finite(safety.get(k)) and abs(safety[k]) < .001 for k in ('v', 'w')))
        if not any(context(sample) for sample in telemetry):
            return None
    return dict(time=max(completed.values()), method='three_agent_durable_events',
                mission_id=mission, frame_epoch=frame, member_completions=completed,
                participants={n: [s['boot_id'], s['map_epoch']] for n, s in starts.items()},
                context_end=end, joint_completion_window=end-max(completed.values()))


def pose_observation_time(sample, robot, clock):
    stamp = robot.get('pose_'+clock)
    if stamp is None:
        return sample[clock]  # Legacy evidence explicitly lacks pose timestamps.
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ValueError('invalid pose observation time')
    return stamp


def fixed_gauge(telemetry, truth, frame_epoch, clock='sim_time'):
    for sample in telemetry:
        mapping = sample.get('mapping', {})
        anchor = mapping.get('anchor_id')
        if (sample.get('coordinate_frame') != frame_epoch or not anchor
                or anchor not in mapping.get('aligned', [])):
            continue
        robot = sample.get('robots', {}).get(anchor, {})
        if not robot.get('pose') or robot.get('boot_id') != mapping.get('anchor_boot'):
            continue
        observed_at = pose_observation_time(sample, robot, clock)
        actual = min(truth, key=lambda t: abs(t[clock]-observed_at))
        if abs(actual[clock]-observed_at) > .3 or anchor not in actual['robots']:
            continue
        return (compose(actual['robots'][anchor][:3], inverse(robot['pose'])),
                dict(anchor_id=anchor, anchor_boot=robot['boot_id'],
                     sim_time=sample['sim_time'], world_pose=actual['robots'][anchor][:3],
                     observation_clock=clock, observation_time=observed_at,
                     pose_timestamp_recorded=robot.get('pose_'+clock) is not None,
                     estimated_pose=robot['pose'], method='one_initial_anchor_sample'))
    raise ValueError('没有初始化时锚定机器人的同时间真值/估计样本；禁止事后拟合')


def first_boot_alignment_errors(telemetry, truth, frame_epoch, world_from_team, clock):
    """Keep the one original gauge across every subsequent boot; never refit."""
    result, seen = [], set()
    if not truth:
        return result
    truth_times = np.asarray([row[clock] for row in truth])
    for sample in telemetry:
        for name, robot in sample.get('robots', {}).items():
            key = (name, robot.get('boot_id'), robot.get('map_epoch'))
            if (key in seen or not robot.get('aligned') or not robot.get('ready')
                    or robot.get('frame_epoch', sample.get('coordinate_frame')) != frame_epoch
                    or robot.get('pose') is None):
                continue
            stamp = pose_observation_time(sample, robot, clock)
            actual = truth[int(np.argmin(np.abs(truth_times-stamp)))]
            if abs(actual[clock]-stamp) > .3 or name not in actual['robots']:
                continue
            predicted, real = compose(world_from_team, robot['pose']), actual['robots'][name]
            result.append(dict(robot=name, boot_id=key[1], map_epoch=key[2], observation_time=stamp,
                position=math.dist(predicted[:2], real[:2]), yaw=abs(angle(predicted[2]-real[2]))))
            seen.add(key)
    return result


def alignment_identity_audit(run, telemetry, history):
    """Require independent evidence for every accepted identity, including gaps."""
    expected = set()

    def add(name, boot, epoch):
        if name not in MEMBERS or not isinstance(boot, str) or not boot or type(epoch) is not int or epoch < 1:
            raise ValueError('已接纳配准身份无效：'+repr((name, boot, epoch)))
        expected.add((name, boot, epoch))

    for sample in telemetry:
        for name, robot in sample.get('robots', {}).items():
            if robot.get('ready') and robot.get('aligned'):
                add(name, robot.get('boot_id'), robot.get('map_epoch'))
    for name in MEMBERS:
        path = Path(run)/name/'events.jsonl'
        for event in rows(path) if path.exists() else []:
            if event.get('event') == 'alignment_ready':
                if event.get('robot') != name:
                    raise ValueError('配准事件机器人身份不匹配')
                add(name, event.get('boot_id'), event.get('map_epoch'))
        for path in (Path(run)/'raw-map-checkpoints').glob(f'{name}-e*-first-aligned-{name}.npz'):
            with np.load(path, allow_pickle=False) as raw:
                add(str(raw['robot'].item()), str(raw['boot'].item()), int(raw['epoch'].item()))
    evaluated = {(r['robot'], r['boot_id'], r['map_epoch']) for r in history}
    missing = expected-evaluated
    def identities(values):
        return [dict(robot=n, boot_id=b, map_epoch=e) for n, b, e in sorted(values)]
    return dict(expected=identities(expected), evaluated=identities(evaluated),
        missing=identities(missing), unexpected=identities(evaluated-expected),
        passed=bool(expected) and {n for n, _, _ in expected} == set(MEMBERS)
            and expected == evaluated and all(math.isfinite(r['position']) and math.isfinite(r['yaw'])
                and r['position'] <= .1 and r['yaw'] <= math.radians(2) for r in history))


def initial_exploration_window(run, telemetry, clock, frame_epoch):
    """End initial participation at completion or the first task/identity fence."""
    start = next((s[clock] for s in telemetry if s.get('mode') in ('ALIGNING', 'EXPLORING')), None)
    if start is None:
        return None, None, None
    boundary = telemetry[-1][clock]
    # The functional harness deliberately tests pause/resume once during its
    # initial exploration. Only its exact recorded command IDs are allowed to
    # cross that fence; later recovery missions cannot extend this phase.
    transcript = Path(run)/'peer-demo.jsonl'
    demo_commands, authorized, resumed_mission = [], set(), None
    for event in rows(transcript) if transcript.exists() else []:
        if event.get('event') in ('exploration_complete', 'development_continuation', 'failure_pause'):
            break
        if event.get('event') == 'command':
            command = event.get('data', {}).get('command')
            result = event.get('result', {})
            if event.get('kind') != 'command' or not result.get('success') or not result.get('command_id'):
                break
            demo_commands.append((command, result['command_id']))
    if [c for c, _ in demo_commands] == ['explore', 'pause', 'explore']:
        authorized = {cid for _, cid in demo_commands}
        resumed_mission = demo_commands[-1][1]
    for name in MEMBERS:
        path = Path(run)/name/'events.jsonl'
        events = sorted(rows(path), key=lambda e: e[clock]) if path.exists() else []
        first = next((i for i, e in enumerate(events) if e.get('event') == 'command_committed'
                      and e.get('command') == 'explore'), None)
        if first is not None:
            identity = (events[first].get('boot_id'), events[first].get('map_epoch'))
            for event in events[first+1:]:
                if ((event.get('event') == 'command_committed'
                     and event.get('command_id') not in authorized) or
                        (event.get('boot_id'), event.get('map_epoch')) != identity):
                    boundary = min(boundary, event[clock])
                    break
    # Telemetry is also a fence when an event stream is unavailable.
    ready = next((s for s in telemetry if s.get('team_ready') and s[clock] >= start), None)
    if ready:
        cohort = {n: (r.get('boot_id'), r.get('map_epoch')) for n, r in ready['robots'].items()}
        for sample in telemetry:
            if sample[clock] <= ready[clock]:
                continue
            if any(n in cohort and r.get('boot_id') and
                   (r.get('boot_id'), r.get('map_epoch')) != cohort[n]
                   for n, r in sample.get('robots', {}).items()):
                boundary = min(boundary, sample[clock])
                break
    completed = next((s[clock] for s in telemetry if start <= s[clock] <= boundary
                      and s.get('coordinate_frame') == frame_epoch and s.get('mode') == 'EXPLORED'), None)
    completion = dict(time=completed, method='three_member_telemetry') if completed is not None else None
    if completion is None:
        completion = durable_exploration_completion(run, telemetry, clock, frame_epoch,
                                                    mission_id=resumed_mission)
        if completion and not start <= completion['time'] <= boundary:
            completion = None
    return start, min(boundary, completion['time']) if completion else boundary, completion


def collision_metrics(truth, obstacles, robot_radius):
    """Measure all recorded Gazebo poses and audit full-team simulation-time coverage."""
    def finite_number(value):
        return type(value) in (int, float) and math.isfinite(value)

    positions, complete_times = [], []
    minimum_pair = math.inf
    paired_samples = 0
    for sample in truth:
        centers = []
        for name in MEMBERS:
            pose = sample['robots'].get(name)
            if isinstance(pose, (list, tuple)) and len(pose) >= 2:
                x, y = pose[:2]
                if finite_number(x) and finite_number(y):
                    centers.append((x, y))
        positions.extend(centers)
        if len(centers) >= 2:
            paired_samples += 1
            for a, b in combinations(centers, 2):
                minimum_pair = min(minimum_pair, math.dist(a, b)-2*robot_radius)
        if len(centers) == len(MEMBERS):
            complete_times.append(sample.get('sim_time'))

    minimum_static = None
    if positions and obstacles:
        # Vectorized obstacle distances keep large recorded runs cheap.
        x, y = np.asarray(positions, dtype=float).T
        measured = min(float(np.min(shape_distance(x, y, obstacle)))-robot_radius
                       for obstacle in obstacles)
        minimum_static = measured if math.isfinite(measured) else None
    minimum_pair = minimum_pair if math.isfinite(minimum_pair) else None

    # Incomplete leading/trailing rows are expected while models are spawned
    # or removed. Inside the first and last complete rows, missing poses
    # lengthen the interval between full samples. Gazebo time advances only
    # with the physics, including when the wall clock stalls or Gazebo pauses.
    times = [t for t in complete_times if finite_number(t)]
    gaps = [b-a for a, b in zip(times, times[1:])]
    distinct_times = len(set(times))
    maximum_gap = max((gap for gap in gaps if gap > 0), default=None)
    issues = []
    if distinct_times < 2:
        issues.append('insufficient_complete_samples')
    if len(times) != len(complete_times) or any(gap < 0 for gap in gaps):
        issues.append('invalid_sim_time')
    if maximum_gap is not None and maximum_gap > COLLISION_SAMPLE_GAP_LIMIT:
        issues.append('complete_sample_gap')
    if minimum_pair is None:
        issues.append('missing_interrobot_clearance')
    if obstacles and minimum_static is None:
        issues.append('missing_static_clearance')

    detected = any(value is not None and value < COLLISION_CLEARANCE_LIMIT
                   for value in (minimum_static, minimum_pair))
    return dict(minimum_static_clearance=minimum_static, minimum_interrobot_clearance=minimum_pair,
        collision_detected=detected, collision_evidence_pass=not issues,
        collision_pass=not detected and not issues,
        collision_evidence=dict(clock='sim_time', truth_samples=len(truth), paired_samples=paired_samples,
            complete_samples=len(complete_times), distinct_complete_times=distinct_times,
            first_complete_sim_time=times[0] if times else None, last_complete_sim_time=times[-1] if times else None,
            max_sample_gap_s=maximum_gap, gap_limit_s=COLLISION_SAMPLE_GAP_LIMIT,
            static_obstacles=len(obstacles), issues=issues))


def evaluate(run, map_path=None, output=None):
    run = Path(run)
    architecture = json.loads((run/'architecture.json').read_text())
    if architecture.get('architecture') != 'decentralized':
        raise ValueError('该评估器仅接受去中心化运行')
    cfg = yaml.safe_load((run/'assets/fleet.yaml').read_text())
    gt = yaml.safe_load((run/'assets/ground_truth.yaml').read_text())
    path = run/'peer-observations.jsonl'
    telemetry = rows(path if path.exists() else run/'telemetry.jsonl')
    truth = rows(run/'truth.jsonl')
    if not telemetry or not truth:
        raise ValueError('缺少遥测或独立真值样本')
    if map_path is None:
        map_path = run/'results/final/shared_map.yaml'
    map_path = Path(map_path)
    frame = json.loads((map_path.parent/'frame.json').read_text())
    data, meta = load_map(map_path)
    clock = association_clock(telemetry, truth)
    world_from_team, gauge = fixed_gauge(telemetry, truth, frame['frame_epoch'], clock)
    last = next((s for s in telemetry if s.get('coordinate_frame') == frame['frame_epoch']
                 and s.get('mode') == 'EXPLORED'),
                next(s for s in reversed(telemetry) if s.get('coordinate_frame') == frame['frame_epoch']))
    obstacles = gt.get('obstacles', gt.get('boxes', []))
    report = full_arena_coverage(data, meta['resolution'], meta['origin'], world_from_team,
        cfg['arena'], obstacles, [r['start'] for r in cfg['robots']],
        last.get('region_seeds', {}), last.get('regions'), cfg['robot_radius'])
    report.update(architecture='decentralized', initialization='unknown', fixed_gauge=gauge,
                  world_from_team=world_from_team, map=str(map_path), association_clock=clock)
    report.update(collision_metrics(truth, obstacles, cfg['robot_radius']))
    times = np.asarray([s[clock] for s in telemetry])
    true_times = np.asarray([s[clock] for s in truth])
    actual_travel = dict.fromkeys(MEMBERS, 0.)
    recovery_travel = dict.fromkeys(MEMBERS, 0.)
    initial_motion_violations, bootstrap_limit_violations, bootstrap_parallel_violations = 0, 0, 0
    exploration_samples, concurrency, parallel_translating = 0, dict.fromkeys(range(4), 0), 0
    first_explore_time, initial_end, completion = initial_exploration_window(
        run, telemetry, clock, frame['frame_epoch'])
    first_ready = next((s for s in telemetry if s.get('team_ready')), None)
    registration_errors = {}
    if first_ready:
        for name in MEMBERS:
            robot = first_ready['robots'][name]
            estimate = robot.get('pose')
            observed_at = pose_observation_time(first_ready, robot, clock)
            actual = truth[int(np.argmin(np.abs(true_times-observed_at)))]
            if estimate and name in actual['robots'] and abs(actual[clock]-observed_at) <= .3:
                prediction = compose(world_from_team, estimate)
                real = actual['robots'][name]
                registration_errors[name] = dict(position=math.dist(prediction[:2], real[:2]),
                                                  yaw=abs(angle(prediction[2]-real[2])))
    # Trajectory metrics use the nearest observation in the association clock.
    right = np.clip(np.searchsorted(times, true_times), 0, len(times)-1)
    left = np.maximum(right-1, 0)
    nearest = np.where(np.abs(times[left]-true_times) <= np.abs(times[right]-true_times), left, right)
    for k, actual in enumerate(truth):
        si = int(nearest[k])
        sample = telemetry[si]
        if abs(sample[clock]-actual[clock]) > 1.:
            continue
        moving, translating = 0, 0
        initial = (k > 0 and first_explore_time is not None
                   and first_explore_time <= truth[k-1][clock] <= actual[clock] <= initial_end)
        for name, pose in actual['robots'].items():
            if k and name in truth[k-1]['robots']:
                dt = actual[clock]-truth[k-1][clock]
                ds = math.dist(pose[:2], truth[k-1]['robots'][name][:2])
                dyaw = abs(angle(pose[2]-truth[k-1]['robots'][name][2]))
                moving += int(dt > 0 and (ds/dt > .015 or dyaw/dt > .05))
                translating += int(dt > 0 and ds/dt > .015)
                if first_explore_time is not None and actual[clock] < first_explore_time and ds > .01:
                    initial_motion_violations += 1
                if sample.get('mode') in ('ALIGNING', 'EXPLORING'):
                    if initial:
                        actual_travel[name] += ds
                    elif initial_end is not None and truth[k-1][clock] >= initial_end:
                        recovery_travel[name] += ds
                if sample.get('mode') == 'ALIGNING' and dt > 0 and ds/dt > .095:
                    bootstrap_limit_violations += 1
        if initial and sample.get('mode') == 'EXPLORING' and sample.get('team_ready') and sample.get('region_enabled'):
            exploration_samples += 1
            concurrency[moving] += 1
            parallel_translating += int(translating == 3)
        if sample.get('mode') == 'ALIGNING' and moving > 1:
            bootstrap_parallel_violations += 1
    formations = []
    for sample in formation_completions(run, telemetry, clock):
        slots = sample.get('formation_slots', [])
        assignment = sample.get('formation_assignment')
        if not assignment:
            continue
        start = max(0, int(np.searchsorted(true_times, sample[clock]-3, side='right'))-1)
        end = min(len(truth)-1, int(np.searchsorted(true_times, sample[clock], side='left')))
        window = truth[start:end+1]
        actual_slots = [compose(world_from_team, p) for p in slots]
        good = (len(window) >= 10 and window[-1][clock]-window[0][clock] >= 3.
                and np.max(np.diff([t[clock] for t in window])) <= .25)
        for actual in window:
            good = good and all(n in actual['robots'] for n in MEMBERS) and formation_pass(
                [actual['robots'][n][:3] for n in MEMBERS], actual_slots, assignment, cfg['robot_radius'])
        formations.append(dict(sim_time=sample['sim_time'], stable_window_samples=len(window),
            shape=identified_formation(slots), seconds=sample['formation_seconds'],
            stable_seconds=window[-1][clock]-window[0][clock] if window else 0.,
            completion_evidence=sample['completion_evidence'],
            actual_pass=bool(good) and sample['formation_seconds'] <= 500))
    ready_time = first_ready[clock] if first_ready else None
    explored_time = completion['time'] if completion else None
    report.update(registration_errors=registration_errors,
        initialization_seconds=ready_time-first_explore_time if ready_time is not None and first_explore_time is not None else None,
        registration_pass=len(registration_errors) == 3 and all(e['position'] <= .1 and e['yaw'] <= math.radians(2) for e in registration_errors.values()),
        initialization_pass=ready_time is not None and first_explore_time is not None and ready_time-first_explore_time <= 300,
        actual_explore_distance=actual_travel, actual_distance_pass=min(actual_travel.values()) >= 3,
        recovery_explore_distance=recovery_travel,
        initial_exploration_window=dict(clock=clock, start=first_explore_time, end=initial_end),
        initial_motion_violations=initial_motion_violations, bootstrap_limit_violations=bootstrap_limit_violations,
        bootstrap_parallel_violations=bootstrap_parallel_violations,
        concurrency_truth_samples=concurrency, regional_exploration_truth_samples=exploration_samples,
        parallel_translating_truth_samples=parallel_translating,
        regional_vehicle_utilization=sum(k*v for k, v in concurrency.items())/max(1, 3*exploration_samples),
        parallel_exploration_observed=parallel_translating > 0,
        autonomous_exploration_seconds=explored_time-first_explore_time
            if explored_time is not None and first_explore_time is not None else None,
        autonomous_exploration_completion_evidence=completion,
        autonomous_exploration_pass=explored_time is not None and first_explore_time is not None
            and explored_time-first_explore_time <= 1200,
        formations=formations, three_formations_pass={f['shape'] for f in formations} == {'triangle', 'row', 'column'}
            and all(f['actual_pass'] for f in formations),
        unknown_cells=int((data < 0).sum()), unknown_encoding_pass=meta.get('free_thresh') == .196)
    resources = rows(run/'resources.jsonl')
    report['peak_project_rss_mb'] = max((r['rss_mb'] for r in resources), default=math.inf)
    report['system_oom_kills_since_start'] = max((r.get('system_oom_kills_since_start', 0) for r in resources), default=0)
    report['resource_pass'] = report['peak_project_rss_mb'] <= 12*1024 and not report['system_oom_kills_since_start']
    history = first_boot_alignment_errors(telemetry, truth, frame['frame_epoch'], world_from_team, clock)
    report['alignment_history'] = history
    report['alignment_identity_audit'] = alignment_identity_audit(run, telemetry, history)
    report['alignment_history_pass'] = report['alignment_identity_audit']['passed']
    report['functional_pass'] = all(report[k] for k in ('coverage_pass', 'actual_distance_pass', 'collision_pass',
        'registration_pass', 'initialization_pass', 'three_formations_pass', 'unknown_encoding_pass',
        'resource_pass', 'parallel_exploration_observed', 'autonomous_exploration_pass',
        'alignment_history_pass')) and not initial_motion_violations and not bootstrap_limit_violations and not bootstrap_parallel_violations
    destination = Path(output) if output is not None else run/'acceptance.json'
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run_dir')
    p.add_argument('--map')
    p.add_argument('--output', help='独立评估结果路径；默认仍为运行目录下 acceptance.json')
    p.add_argument('--report-only', action='store_true', help='记录全部指标，不因完整评估的精度或性能指标退出失败')
    a = p.parse_args()
    report = evaluate(a.run_dir, a.map, a.output)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not a.report_only and not report['functional_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
