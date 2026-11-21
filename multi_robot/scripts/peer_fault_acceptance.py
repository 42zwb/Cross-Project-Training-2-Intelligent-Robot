#!/usr/bin/env python3
"""Real process faults and receive-boundary link faults in one peer session."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import numpy as np
import yaml
from rosgraph_msgs.msg import Clock
from runtime_context import RuntimeClient
from fleet_bringup.simulation_preflight import snapshot, _send, SESSION
from fleet_control.geometry import Grid
from fleet_control.peer_protocol import MEMBERS
from fleet_mapping.grid import load_map
from fleet_control.relative_evaluate import rows
from acceptance_progress import AcceptanceProgress
from peer_network_evidence import MotionRecorder, save_audit
from peer_acceptance_readiness import ReadinessHold, RecoverySpacingHold
from peer_acceptance_routes import short_patrol
from peer_acceptance_staging import (staging_plan, staging_release, release_segment_clear,
                                     release_state_valid, release_clock_delay)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--network-only', action='store_true')
    parser.add_argument('--robots', nargs='+', choices=MEMBERS, default=list(MEMBERS))
    parser.add_argument('--components', nargs='+', choices=['peer_agent', 'local_safety', 'controller_server', 'scan_bridge', 'odom_tf'],
                        default=['peer_agent', 'local_safety', 'controller_server', 'scan_bridge', 'odom_tf'])
    parser.add_argument('--actions', nargs='+', choices=['freeze', 'exit'], default=['freeze', 'exit'])
    parser.add_argument('--skip-network', action='store_true')
    parser.add_argument('--skip-restart-estop', action='store_true')
    parser.add_argument('--skip-clock', action='store_true')
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    api = RuntimeClient(run)
    session = json.loads((run/'architecture.json').read_text())['session_id']
    progress = AcceptanceProgress(run, 'peer-fault-run', api)
    profile_path = run/'peer_network_faults.json'
    report, resumed = [], []
    full_matrix = (not args.network_only and not args.skip_network and not args.skip_restart_estop and not args.skip_clock
                   and len(set(args.robots)) == 3 and len(set(args.components)) == 5 and len(set(args.actions)) == 2)
    monotonic_truth = any('steady_time' in item for item in rows(run/'truth.jsonl')[-5:])
    if full_matrix and not monotonic_truth:
        raise RuntimeError('完整故障验收需要带单调观测时间的独立真值，请重新冷启动生成新的记录')
    (run/'peer-fault-manifest.json').write_text(json.dumps(dict(full_matrix=full_matrix,
        robots=args.robots, components=args.components, actions=args.actions,
        network_only=args.network_only, skip_network=args.skip_network, skip_restart_estop=args.skip_restart_estop,
        skip_clock=args.skip_clock), indent=2))
    peer = api.peer()
    recorder = MotionRecorder(peer.node.context, MEMBERS)

    def record(value):
        report.append(value)
        progress.report['completed_cases'] = len(report)
        (run/'peer-fault-acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(value, flush=True)

    def send(command, argument=''):
        progress.check()
        result = api.request('command', dict(command=command, argument=argument), timeout=20)
        progress.check()
        if not result['success']:
            progress.report['rejected_command'] = dict(command=command, argument=argument, response=result)
            raise RuntimeError(result['message'])
        return result

    def wait(check, timeout=40):
        return progress.wait(check, timeout)

    def settle(owners, *, all_members=False, stage='ready_before_motion', timeout=60):
        progress.stage(stage, timeout, owners=list(owners), all_members=all_members)
        hold = ReadinessHold(owners, all_members=all_members)
        def check(state):
            ready = hold.update(state, time.monotonic())
            progress.report['readiness'] = hold.details
            return ready
        return wait(check, timeout)

    def profile(links):
        temp = profile_path.with_suffix('.tmp')
        temp.write_text(json.dumps(dict(session_id=session, links=links)))
        temp.replace(profile_path)
        time.sleep(.3)

    def patrol(name, timeout=60, length=1., all_members=False):
        progress.stage('map_export_before_fault_motion', 20, owner=name)
        send('save', str(run/'results/fault_view'))
        # PBStream export can briefly make Cartographer TF unavailable. A
        # successful save reply is not a fresh localization certificate.
        state = settle([name], all_members=all_members, stage='ready_after_map_export')
        data, meta = load_map(run/'results/fault_view'/('replica_'+name+'.yaml'))
        grid = Grid(data, meta['resolution'], meta['origin'][:2], .18, .35)
        pose = state['robots'][name]['pose']
        peers = [r['pose'] for n, r in state['robots'].items() if n != name and r.get('pose')]
        points = short_patrol(grid, pose, peers, length=length, other_clearance=.9)
        if points is None:
            raise RuntimeError(name+' 没有足够净空的故障测试路线')
        route = dict(routes={name: points}, coordinate_frame=state['coordinate_frame'],
                     cycles=1, dwell=0, goal_timeout=120)
        send('patrol', json.dumps(route))
        progress.stage('await_fault_test_motion', timeout, owner=name)
        return wait(lambda s: abs(s['robots'][name]['safety'].get('v', 0.)) > .04, timeout)

    def spread():
        send('pause')
        settle(MEMBERS, all_members=True, stage='ready_before_safe_region_staging')
        progress.stage('staging_map_export', 20)
        directory = run/'results/staging_view'
        send('save', str(directory))
        state = settle(MEMBERS, all_members=True, stage='ready_after_staging_map_export')
        seeds = state.get('region_seeds', {})
        if set(seeds) != set(MEMBERS):
            raise RuntimeError('缺少固定分区种子，不能建立故障验收安全区域')
        frame = json.loads((directory/'frame.json').read_text())
        identities = {n: (r['boot_id'], r['map_epoch']) for n, r in state['robots'].items()}
        if (frame['frame_epoch'] != state['coordinate_frame'] or any(
                (frame['transforms'].get(n, {}).get('boot_id'),
                 frame['transforms'].get(n, {}).get('map_epoch')) != identities[n] for n in MEMBERS)):
            raise RuntimeError('安全分散地图的坐标或启动身份已变化')
        grids = {}
        for n in MEMBERS:
            data, meta = load_map(directory/('replica_'+n+'.yaml'))
            grids[n] = Grid(data, meta['resolution'], meta['origin'][:2], .18, .35)
        poses = {n: state['robots'][n]['pose'] for n in MEMBERS}
        evidence = dict(coordinate_frame=state['coordinate_frame'], identities=identities,
            seeds=seeds, poses=poses, maps=str(directory), radius=.75, planned_separation=3.,
            peer_clearance=.9, plan=None)
        plan_path = run/'peer-fault-staging-plan.json'
        plan_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
        plan = staging_plan(grids, poses, seeds)
        evidence['plan'] = plan
        plan_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
        deadline = time.monotonic()+500.

        def budget():
            remaining = deadline-time.monotonic()
            if remaining <= 0.:
                raise TimeoutError('安全分散的总通行时间超过 500 秒')
            return remaining

        release_path = None
        if plan is None:
            release = staging_release(grids, poses, seeds)
            if release is None:
                raise RuntimeError('当前各车观测图中没有经过验证的分散目标与通行顺序')
            folder = run/'results/staging_releases'/str(len(report))
            shutil.copytree(directory, folder/'before')
            release_path = folder/'release.json'
            release_evidence = dict(plan=release, coordinate_frame=state['coordinate_frame'],
                identities=identities, original_poses=poses, before_maps=str(folder/'before'),
                passed=False, outcome='planned', minimum_separations=dict(release['initial_separations']))
            release_path.write_text(json.dumps(release_evidence, ensure_ascii=False, indent=2))
            evidence['release_evidence'] = str(release_path)
            plan_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
            owner, target = release['owner'], release['target']

            def guarded_departure(value):
                sampled_at = time.monotonic()
                delay = release_clock_delay(value, release, identities, state['coordinate_frame'], sampled_at)
                if delay:
                    time.sleep(delay)
                    waits = release_evidence.setdefault('clock_waits', dict(count=0, maximum_requested=0.))
                    waits['count'] += 1
                    waits['maximum_requested'] = max(waits['maximum_requested'], delay)
                checked_at = time.monotonic()
                if not release_state_valid(value, release, identities, state['coordinate_frame'], checked_at,
                                           state_elapsed=checked_at-sampled_at):
                    release_evidence['invalid_state'] = dict(sampled_steady=sampled_at,
                        checked_steady=checked_at, clock_wait_requested=delay,
                        coordinate_frame=value.get('coordinate_frame'), estop=value.get('estop'),
                        robots={n: {k: r.get(k) for k in ('ready', 'aligned', 'nav_ready',
                            'boot_id', 'map_epoch', 'frame_epoch', 'state_age', 'pose_steady_time',
                            'pose', 'goal', 'safety')} for n, r in value.get('robots', {}).items()})
                    raise RuntimeError('临时退让的定位、身份、静止同伴、间距或已验证走廊不再有效')
                current_poses = {n: value['robots'][n].get('pose') for n in MEMBERS}
                for n in release['others']:
                    release_evidence['minimum_separations'][n] = min(
                        release_evidence['minimum_separations'][n],
                        math.dist(current_poses[owner][:2], current_poses[n][:2]))
                return value['robots'][owner]['mode'] == 'PATROL_COMPLETE'

            try:
                current = settle(MEMBERS, all_members=True, stage='ready_before_staging_release',
                                 timeout=min(60., budget()))
                guarded_departure(current)
                if not release_segment_clear(grids[owner], current['robots'][owner]['pose'], target,
                        [current['robots'][n]['pose'] for n in release['others']]):
                    raise RuntimeError('临时退让出发前观测位置不再满足已验证的直线净空')
                send('patrol', json.dumps(dict(routes={owner: [target]},
                    coordinate_frame=state['coordinate_frame'], cycles=1, dwell=0, goal_timeout=120)))
                progress.stage('staging_peer_release', budget(), owner=owner, target=target)
                release_evidence['outcome'] = 'moving'
                release_path.write_text(json.dumps(release_evidence, ensure_ascii=False, indent=2))
                wait(guarded_departure, budget())
                send('pause')
                current = settle(MEMBERS, all_members=True, stage='staging_release_stationary',
                                 timeout=min(60., budget()))
                guarded_departure(current)
                if any(math.dist(current['robots'][owner]['pose'][:2], current['robots'][n]['pose'][:2])
                       < 1. for n in release['others']):
                    raise RuntimeError('临时退让后尚未离开普通同伴排除区')
                release_evidence.update(passed=True, outcome='passed',
                    final_poses={n: current['robots'][n]['pose'] for n in MEMBERS})
            except Exception as error:
                release_evidence.update(outcome='failed', error=str(error))
                try:
                    send('pause')
                finally:
                    raise
            finally:
                release_path.write_text(json.dumps(release_evidence, ensure_ascii=False, indent=2))
            # No second retreat: independently export and solve the original
            # 0.90 m three-leg problem using the actual reached positions.
            directory = folder/'after'
            progress.stage('staging_map_export_after_release', min(20., budget()))
            send('save', str(directory))
            current = settle(MEMBERS, all_members=True, stage='ready_after_staging_release_export',
                             timeout=min(60., budget()))
            frame = json.loads((directory/'frame.json').read_text())
            if (current['coordinate_frame'] != state['coordinate_frame']
                    or frame['frame_epoch'] != state['coordinate_frame'] or any(
                    (current['robots'][n]['boot_id'], current['robots'][n]['map_epoch']) != identities[n]
                    or (frame['transforms'].get(n, {}).get('boot_id'),
                        frame['transforms'].get(n, {}).get('map_epoch')) != identities[n] for n in MEMBERS)):
                raise RuntimeError('临时退让后的地图坐标或身份已变化')
            for n in MEMBERS:
                data, meta = load_map(directory/('replica_'+n+'.yaml'))
                grids[n] = Grid(data, meta['resolution'], meta['origin'][:2], .18, .35)
            poses = {n: current['robots'][n]['pose'] for n in MEMBERS}
            plan = staging_plan(grids, poses, seeds)
            evidence.update(poses=poses, maps=str(directory), plan=plan)
            plan_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
            if plan is None:
                raise RuntimeError('一次临时退让后的真实观测仍不支持原有净空的三段分散路线')
        targets = dict(plan)

        for index, (owner, target) in enumerate(plan):
            current = settle(MEMBERS, all_members=True, stage='ready_before_staging_leg',
                             timeout=min(60., budget()))
            if (current['coordinate_frame'] != state['coordinate_frame'] or any(
                    (current['robots'][n]['boot_id'], current['robots'][n]['map_epoch']) != identities[n]
                    for n in MEMBERS)):
                raise RuntimeError('安全分散执行前坐标或启动身份已变化')
            others = [r['pose'] for n, r in current['robots'].items() if n != owner]
            allowed = grids[owner].allowed(others=others, other_clearance=.9)
            if grids[owner].astar(current['robots'][owner]['pose'], target, allowed) is None:
                raise RuntimeError(owner+' 当前起点或同伴位置不再支持已选分散路线')
            send('patrol', json.dumps(dict(routes={owner: [target]},
                 coordinate_frame=state['coordinate_frame'], cycles=1, dwell=0, goal_timeout=240)))
            remaining = budget()
            progress.stage('safe_region_staging', remaining, owner=owner, leg=index)
            wait(lambda s: s['robots'][owner]['mode'] == 'PATROL_COMPLETE', remaining)
        staged = settle(MEMBERS, all_members=True, stage='safe_staging_stationary',
                        timeout=min(60., budget()))
        poses = {n: staged['robots'][n]['pose'] for n in MEMBERS}
        if (any(math.dist(poses[n][:2], targets[n][:2]) > .25
                or math.dist(poses[n][:2], seeds[n][:2]) > 1. for n in MEMBERS)
                or min(math.dist(poses[a][:2], poses[b][:2])
                       for i, a in enumerate(MEMBERS) for b in MEMBERS[:i]) < 2.5):
            raise AssertionError('未到达相互分离的故障验收安全区域')
        with (run/'peer-fault-staging.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(steady_time=time.monotonic(), seeds=seeds,
                targets=targets, order=[n for n, _ in plan], poses=poses,
                coordinate_frame=staged['coordinate_frame'],
                release_evidence=str(release_path) if release_path else None))+'\n')

    def pair_continues(isolated, failure, last_connected):
        survivors = [n for n in MEMBERS if n != isolated]
        state = settle(survivors, stage='two_connected_members_ready')
        if state['coordinate_frame'] != last_connected['coordinate_frame']:
            raise RuntimeError('失联续行前共同坐标发生变化')
        poses = {n: state['robots'][n]['pose'] if n in survivors else
                 last_connected['robots'][n]['pose'] for n in MEMBERS}
        # patrol() exported this fully observed map before isolation. Do not
        # ask the isolated member to export a partial competing map, or require
        # its common-frame pose to remain verified while disconnected.
        data, meta = load_map(run/'results/fault_view/shared_map.yaml')
        grid = Grid(data, meta['resolution'], meta['origin'][:2], .18, .35)
        routes = {}
        for n in survivors:
            p = poses[n]
            for yaw in [p[2], p[2]+math.pi/4, p[2]-math.pi/4, p[2]+math.pi]:
                target = [p[0]+.30*math.cos(yaw), p[1]+.30*math.sin(yaw), p[2]]
                if (min(math.dist(target[:2], poses[q][:2]) for q in MEMBERS if q != n) > 2.5
                        and grid.astar(p, target, grid.allowed()) is not None):
                    routes[n] = [target]
                    break
            if n not in routes:
                raise RuntimeError(n+' 缺少失联禁止区域之外的短程路线')
        continuation_started = time.monotonic()
        send('patrol', json.dumps(dict(routes=routes, coordinate_frame=state['coordinate_frame'],
                                     cycles=1, dwell=0, goal_timeout=120)))
        moved, start = set(), time.monotonic()
        while time.monotonic()-start < 14:
            value = api.state(timeout=.05)
            for n in survivors:
                current = value['robots'][n].get('pose')
                if current and math.dist(poses[n][:2], current[:2]) > .12:
                    moved.add(n)
            if moved == set(survivors):
                break
        if moved != set(survivors):
            raise AssertionError(dict(action='two_connected_members_continue', moved=sorted(moved)))
        # Independently bound both initial body disks and check every physical
        # survivor sample outside the expanding lost-peer region. The estimate
        # selects tasks; truth is read only after movement for acceptance.
        samples = rows(run/'truth.jsonl')
        initial = min(samples, key=lambda t: abs(t['steady_time']-continuation_started))
        displacements = {n: max((math.dist(t['robots'][n][:2], initial['robots'][n][:2])
            for t in samples if t['steady_time'] >= continuation_started and n in t['robots']), default=0.)
            for n in MEMBERS}
        progress.report['isolation_motion'] = dict(isolated=isolated,
            started_steady=continuation_started, physical_displacements=displacements)
        if displacements[isolated] > .03:
            raise AssertionError('孤立成员在其他两车执行新任务时恢复了运动')
        if any(displacements[n] < .08 for n in survivors):
            raise AssertionError('估计位移没有通过两车真实续行检查：'+str(displacements))
        before = min((t for t in samples if isolated in t['robots']),
                     key=lambda t: abs(t['steady_time']-failure))
        last_pose = before['robots'][isolated]
        cfg = yaml.safe_load((run/'assets/fleet.yaml').read_text())
        sigma = max(.05, last_connected['mapping'].get('transforms', {}).get(isolated, {}).get('sigma', .05))
        minimum_margin = math.inf
        for actual in samples:
            age = actual.get('steady_time', -math.inf)-failure
            if age < .8 or actual.get('steady_time', 0) > time.monotonic():
                continue
            # The pre-isolation pose is a slightly more conservative center
            # than the last received heartbeat. Include both body radii.
            forbidden = 2*cfg['robot_radius']+.10+cfg['max_speed']*age+3*sigma
            for name in survivors:
                if name in actual['robots']:
                    minimum_margin = min(minimum_margin,
                        math.dist(actual['robots'][name][:2], last_pose[:2])-forbidden)
        if minimum_margin < -.03:
            raise AssertionError(dict(action='lost_peer_forbidden_region', minimum_margin=minimum_margin))
        return sorted(moved), minimum_margin

    def find(name, component):
        found = []
        for process in snapshot().values():
            if process.env.get(SESSION) != session:
                continue
            namespace = '__ns:=/'+name
            if component in ('peer_agent', 'local_safety', 'controller_server', 'odom_tf'):
                match = any(Path(arg).name == component for arg in process.argv[:2])
            else:
                match = (any(Path(arg).name == 'parameter_bridge' for arg in process.argv[:2])
                         and '__node:=scan_bridge' in process.argv)
            if match and namespace in process.argv:
                found.append(process)
        if len(found) != 1:
            raise RuntimeError('不能唯一定位本轮本车 '+component+': '+str([p.pid for p in found]))
        return found[0]

    def stopping_sample(name, failure_time, failure_sim):
        # Observe the actuator topic independently. A frozen agent cannot make
        # its stale relayed status count as stopping. If the guard itself dies,
        # the native controller expires its last command in 0.5 wall seconds.
        end = failure_time+2.2
        while time.monotonic() < end:
            peer.spin(.01)
        history = recorder.snapshot(name)[0]
        nonzero = [row for row in history if row[0] >= failure_time-.6
                   and (abs(row[1]) >= .001 or abs(row[2]) >= .001)]
        if not nonzero:
            raise AssertionError(name+' 故障前没有独立速度证据')
        last = nonzero[-1][0]
        zeros = [row[0] for row in history if row[0] > max(last, failure_time)
                 and abs(row[1]) < .001 and abs(row[2]) < .001]
        authority = max(0., (min(zeros) if zeros else last+.5)-failure_time)
        # Read truth only AFTER stopping observation, never to select a target
        # or decide when to terminate the exploration mission.
        samples = rows(run/'truth.jsonl')
        clock, failed = ('steady_time', failure_time) if monotonic_truth else ('sim_time', failure_sim)
        window = [r for r in samples if failed <= r[clock] <= failed+2.0 and name in r['robots']]
        stopped_at = None
        for i in range(1, len(window)-2):
            current = window[i]
            tail = window[i:i+3]
            p = current['robots'][name]
            if (tail[-1][clock]-tail[0][clock] >= .18
                and all(math.dist(t['robots'][name][:2], p[:2]) < .003
                   and abs(math.atan2(math.sin(t['robots'][name][2]-p[2]), math.cos(t['robots'][name][2]-p[2]))) < .008 for t in tail)):
                stopped_at = current[clock]-failed
                break
        if stopped_at is None or stopped_at > 1.5 or authority is not None and authority > .8:
            raise AssertionError(dict(robot=name, authorization_stop=authority, physical_stop=stopped_at))
        return dict(authorization_stop_seconds=authority, physical_stop_seconds=stopped_at,
                    physical_stop_clock=clock)

    def restore(process, name, component):
        log = (run/name/('fault-restore-'+component+'.log')).open('a')
        env = dict(process.env, TB4_SIM_COMPONENT='recovered_sensor', TB4_SIM_ROBOT=name)
        subprocess.Popen(list(process.argv), env=env, cwd=process.cwd, stdin=subprocess.DEVNULL,
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log.close()

    def observe(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            progress.check()
            peer.spin(.02)

    def audit(name, start, end, **options):
        # Allow asynchronous recorder/subscriber callbacks to finish; this does
        # not change the bounded observation interval or select motion targets.
        observe(.2)
        commands, guards, raw = recorder.snapshot(name)
        action = progress.report['case']['action']
        path = run/'network-evidence'/f'{action}-{name}-{start:.6f}.json'
        return save_audit(path, commands, guards, rows(run/'truth.jsonl'),
                          name, start, end, raw=raw, **options)

    def execute():
        try:
            send('pause')
            if not args.network_only:
                for name in args.robots:
                    for component in args.components:
                        for action in args.actions:
                            progress.report['case'] = dict(robot=name, component=component, action=action)
                            progress.report['fault_injected'] = False
                            # Every process fault starts from the separated observed
                            # seeds, including after a previous bootstrap recovery.
                            spread()
                            patrol(name, all_members=True)
                            process = find(name, component)
                            progress.stage('process_fault_stop', 5, robot=name, component=component, action=action)
                            state = api.state(timeout=.05)
                            failed, sim = time.monotonic(), state['sim_time']
                            _send(process, signal.SIGSTOP if action == 'freeze' else signal.SIGINT)
                            progress.report['fault_injected'] = True
                            if action == 'freeze':
                                resumed.append(process)
                            timing = stopping_sample(name, failed, sim)
                            send('pause')
                            if action == 'freeze':
                                _send(process, signal.SIGCONT)
                                resumed.remove(process)
                            elif component in ('scan_bridge', 'odom_tf'):
                                restore(process, name, component)
                            restart_started = time.monotonic()
                            progress.stage('restart_autonomy', 30)
                            subprocess.run(['ros2', 'run', 'fleet_bringup', 'peer_restart', name,
                                            '--run-dir', str(run)], check=True, timeout=30)
                            # Restart never receives an implicit motion command.
                            new_boot = yaml.safe_load((run/'assets/agents'/f'{name}.yaml').read_text())['boot_id']
                            progress.stage('restart_local_ready', 120, boot_id=new_boot)
                            target = wait(lambda s: s['robots'][name].get('ready')
                                          and s['robots'][name].get('boot_id') == new_boot, 120)
                            assert target['robots'][name].get('goal') is None
                            if not target['robots'][name].get('aligned'):
                                remaining = 300.-(time.monotonic()-restart_started)
                                if remaining <= 0.:
                                    raise TimeoutError('重启重新配准超过 300 秒')
                                progress.stage('restart_unknown_origin_alignment', remaining, boot_id=new_boot)
                                send('explore')
                                target = wait(lambda s: s.get('team_ready'),
                                              max(0., 300.-(time.monotonic()-restart_started)))
                                alignment_seconds = time.monotonic()-restart_started
                                if alignment_seconds > 300.:
                                    raise TimeoutError('重启重新配准超过 300 秒')
                                # A just-aligned member may still be close to
                                # another explorer. Do not freeze that legal
                                # transient pose into an impossible 0.90 m
                                # staging start or weaken the retreat guard.
                                progress.stage('recovery_spacing_before_pause', 60, boot_id=new_boot)
                                hold = RecoverySpacingHold()
                                def prepared(state):
                                    if (state.get('coordinate_frame') != target['coordinate_frame'] or any(
                                            (state['robots'][n].get('boot_id'), state['robots'][n].get('map_epoch')) !=
                                            (r['boot_id'], r['map_epoch']) for n, r in target['robots'].items())):
                                        raise RuntimeError('恢复站位准备期间成员身份或共同坐标改变')
                                    good = hold.update(state, time.monotonic())
                                    progress.report['recovery_spacing'] = hold.details
                                    return good
                                prepared_at = time.monotonic()
                                prepared_state = wait(prepared, 60)
                                send('pause')
                                with (run/'recovery-preparation.jsonl').open('a') as stream:
                                    stream.write(json.dumps(dict(robot=name, boot_id=new_boot,
                                        alignment_seconds=alignment_seconds,
                                        preparation_seconds=time.monotonic()-prepared_at,
                                        observed_spacing=hold.details, coordinate_frame=prepared_state['coordinate_frame'],
                                        poses={n: r['pose'] for n, r in prepared_state['robots'].items()}))+'\n')
                            else:
                                alignment_seconds = time.monotonic()-restart_started
                            record(dict(robot=name, component=component, action=action, passed=True,
                                boot_id=new_boot, map_epoch=target['robots'][name]['map_epoch'],
                                restart_alignment_seconds=alignment_seconds, **timing))
            channels = ['state', 'event', 'map', 'constraint', 'mapping', 'continuity']
            for name in [] if args.skip_network else args.robots:
                progress.report.update(case=dict(robot=name, action='bidirectional_isolation'), fault_injected=False)
                spread()
                patrol(name, all_members=True)
                links = [dict(source=source, target=target, channels=channels, drop=1.)
                         for source in MEMBERS for target in MEMBERS if source != target
                         and (source == name or target == name)]
                progress.stage('bidirectional_isolation_stop_and_continuation', 90, robot=name)
                state = api.state(timeout=.05)
                failure, sim = time.monotonic(), state['sim_time']
                profile(links)
                progress.report['fault_injected'] = True
                timing = stopping_sample(name, failure, sim)
                continued, margin = pair_continues(name, failure, state)
                send('pause')
                profile([])
                record(dict(robot=name, action='bidirectional_isolation', continued=continued,
                            minimum_forbidden_region_margin=margin,
                            physical_continuation=progress.report.get('isolation_motion'), passed=True, **timing))
                # Reconnection alone does not create a new mission.
                wait(lambda s: all(not r.get('goal') for r in s['robots'].values()), 10)
            scenarios = [dict(name='one_way_loss', drop=1., delay=0., reorder=0., duplicates=0),
                         dict(name='loss_reorder_duplicates', drop=.25, delay=.04, reorder=.12, duplicates=2),
                         dict(name='late_votes', drop=0., delay=.65, reorder=.3, duplicates=1)]
            for case in [] if args.skip_network else scenarios:
                progress.report.update(case=dict(action=case['name']), fault_injected=False)
                spread()
                patrol('robot2', all_members=True)
                progress.stage('network_scenario', 180, action=case['name'], affected='robot2')
                sources = ('robot1', 'robot3') if case['name'] == 'late_votes' else ('robot1',)
                rules = [dict(source=source, target='robot2',
                              channels=['event'] if case['name'] == 'late_votes' else channels,
                              **{k: case[k] for k in ('drop', 'delay', 'reorder', 'duplicates')})
                         for source in sources]
                before = api.state(.05)
                started = time.monotonic()
                profile(rules)
                progress.report['fault_injected'] = True
                evidence = {}
                if case['name'] == 'late_votes':
                    evidence.update(stopping_sample('robot2', started, before['sim_time']))
                    evidence['authorization_window'] = audit('robot2', started, time.monotonic())
                else:
                    observe(3.)
                    evidence['affected_motion'] = audit('robot2', started, time.monotonic(), require_motion=True)
                send('pause')
                profile([])
                wait(lambda s: all(not r.get('goal') for r in s['robots'].values()), 10)
                observe(1.5)
                quiet = time.monotonic()
                observe(1.3)
                evidence['no_old_task_resumption'] = audit('robot2', quiet, time.monotonic(), stationary=True)
                patrol('robot2', all_members=True)
                recovered = time.monotonic()
                observe(3.)
                evidence['new_task_recovery'] = audit('robot2', recovered, time.monotonic(), require_motion=True)
                send('pause')
                record(dict(action=case['name'], affected='robot2', rules=rules, passed=True, **evidence))
            if not args.skip_network:
                progress.report.update(case=dict(action='expired_delayed_majority'), fault_injected=False)
                patrol('robot1')
                progress.stage('expired_delayed_majority_stop', 10)
                state = api.state(.05)
                failure, sim = time.monotonic(), state['sim_time']
                profile([dict(source=n, target='robot1', channels=['event'], delay=.70, duplicates=2)
                         for n in ('robot2', 'robot3')])
                progress.report['fault_injected'] = True
                timing = stopping_sample('robot1', failure, sim)
                send('pause')
                profile([])
                wait(lambda s: all(not r.get('goal') for r in s['robots'].values()), 5)
                # Delayed messages remain queued after the rule is removed. They
                # must not resurrect the fenced mission while being flushed.
                until = time.monotonic()+1.2
                while time.monotonic() < until:
                    peer.spin(.02)
                    if any(abs(v) > .001 or abs(w) > .001 for t, v, w in recorder.snapshot('robot1')[0]
                           if t > until-1.1):
                        raise AssertionError('迟到多数授权恢复了已中止任务的速度')
                record(dict(action='expired_delayed_majority_does_not_restore_motion', passed=True, **timing))
            for jump in [] if args.skip_clock else (0., 120., -120.):
                progress.report.update(case=dict(action='clock_fault', jump_seconds=jump), fault_injected=False)
                if not monotonic_truth:
                    raise RuntimeError('时钟故障必须使用带 steady_time 的真值，不允许仿真时间代替墙钟停车期限')
                patrol('robot1')
                bridges = [p for p in snapshot().values() if p.env.get(SESSION) == session
                           and '__node:=clock_bridge' in p.argv]
                if len(bridges) != 1:
                    raise RuntimeError('本轮 clock_bridge 无法唯一定位')
                bridge = bridges[0]
                progress.stage('clock_fault_stop', 30, jump_seconds=jump)
                before = api.state(.05)
                failed, sim = time.monotonic(), before['sim_time']
                _send(bridge, signal.SIGSTOP)
                progress.report['fault_injected'] = True
                resumed.append(bridge)
                publisher = peer.node.create_publisher(Clock, '/clock', 10)
                timer = None
                try:
                    if jump:
                        value = max(0., sim+jump)
                        message = Clock()
                        message.clock.sec = int(value)
                        message.clock.nanosec = int((value-int(value))*1e9)
                        from rclpy.clock import Clock as ROSClock, ClockType
                        timer = peer.node.create_timer(.02, lambda: publisher.publish(message),
                                                      clock=ROSClock(clock_type=ClockType.STEADY_TIME))
                        publisher.publish(message)
                    timing = stopping_sample('robot1', failed, sim)
                    send('pause')  # Fence the old task before restoring time.
                finally:
                    if timer:
                        peer.node.destroy_timer(timer)
                    peer.node.destroy_publisher(publisher)
                    _send(bridge, signal.SIGCONT)
                    resumed.remove(bridge)
                wait(lambda s: all(r.get('ready') for r in s['robots'].values()), 120)
                wait(lambda s: all(not r.get('goal') for r in s['robots'].values()), 10)
                record(dict(action='clock_freeze' if jump == 0 else 'clock_jump', jump_seconds=jump,
                            passed=True, **timing))
            if not args.skip_clock:
                progress.report.update(case=dict(action='world_pause_and_resume'), fault_injected=False)
                patrol('robot1')
                latest = rows(run/'truth.jsonl')[-1]
                command = ['ign', 'service', '-s', '/world/fleet_arena/control',
                           '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
                           '--timeout', '1500', '--req']
                progress.stage('world_pause_and_resume', 120)
                try:
                    failed = time.monotonic()
                    response = subprocess.run(command+['pause: true'], capture_output=True, text=True,
                                              check=True, timeout=5)
                    progress.report['fault_injected'] = True
                    if 'true' not in response.stdout:
                        raise RuntimeError('Gazebo 未确认暂停物理仿真：'+response.stdout)
                    until = failed+2.2
                    while time.monotonic() < until:
                        peer.spin(.02)
                    history = recorder.snapshot('robot1')[0]
                    nonzero = [t for t, v, w in history if t >= failed-.6 and (abs(v) > .001 or abs(w) > .001)]
                    zeros = [t for t, v, w in history if t > max([failed]+nonzero)
                             and abs(v) < .001 and abs(w) < .001]
                    authority = min(zeros)-failed if zeros else math.inf
                    if authority > .8:
                        raise AssertionError(dict(action='world_pause', authorization_stop=authority))
                    send('pause')
                finally:
                    resumed_at = time.monotonic()
                    subprocess.run(command+['pause: false'], capture_output=True, text=True, check=True, timeout=5)
                wait(lambda s: all(r.get('ready') and not r.get('goal') for r in s['robots'].values()), 120)
                after = next(t for t in rows(run/'truth.jsonl')
                             if t['steady_time'] > resumed_at and 'robot1' in t['robots'])
                displacement = math.dist(latest['robots']['robot1'][:2], after['robots']['robot1'][:2])
                if displacement > .05:
                    raise AssertionError('物理暂停/恢复后出现未授权位移：'+str(displacement))
                record(dict(action='world_pause_and_resume', authorization_stop_seconds=authority,
                            resume_displacement=displacement, passed=True,
                            physical_observation='physics pause acknowledged; first resumed truth pose checked'))
            if not args.skip_restart_estop:
                progress.report.update(case=dict(action='restart_preserves_estop'), fault_injected=False)
                progress.stage('restart_preserves_estop', 180)
                send('estop')
                for name in args.robots:
                    boot = api.state(timeout=.2)['robots'][name]['boot_id']
                    subprocess.run(['ros2', 'run', 'fleet_bringup', 'peer_restart', name, '--run-dir', str(run)], check=True)
                    state = wait(lambda s: s['robots'][name].get('ready') and s['robots'][name].get('boot_id') != boot, 120)
                    assert peer.states[name][1].estop and state['robots'][name].get('goal') is None
                    wait(lambda s: (guard := recorder.latest(name)) and time.monotonic()-guard[0] < .5
                         and guard[1].get('estop'), 5)
                send('clear-estop')
                send('pause')
                record(dict(action='restart_preserves_estop_and_discards_goals', passed=True))
            (run/'peer-fault-summary.json').write_text(json.dumps(dict(passed=True, full_matrix=full_matrix,
                 completed_cases=len(report), process_cases=sum(r.get('component') is not None for r in report)), indent=2))
        finally:
            for process in resumed:
                _send(process, signal.SIGCONT)
            profile([])
            try:
                progress.check()
                send('pause')
            except Exception:
                pass
            recorder.close()
            peer.close()


    with progress:
        execute()
        progress.succeed(full_matrix=full_matrix, completed_cases=len(report),
            process_cases=sum(r.get('component') is not None for r in report))


if __name__ == '__main__':
    main()
