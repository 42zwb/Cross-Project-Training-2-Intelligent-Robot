#!/usr/bin/env python3
"""Stop the entire optional monitor, then command robots through fleetctl."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
from runtime_context import RuntimeClient
from fleet_bringup.simulation_preflight import snapshot, _send, still_alive, SESSION
from fleet_control.geometry import Grid
from fleet_mapping.grid import load_map
from fleet_control.peer_protocol import MEMBERS
from peer_acceptance_readiness import ReadinessHold, stationary_patrol_state
from peer_acceptance_routes import short_patrol


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    api = RuntimeClient(run)
    session = json.loads((run/'architecture.json').read_text())['session_id']
    report = dict(passed=False, monitor_processes=[])

    def cli(*arguments):
        result = subprocess.run(['ros2', 'run', 'fleet_control', 'fleetctl', '--run-dir', str(run), *arguments],
                                capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        return result.stdout

    def wait(check, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            state = api.state(.1)
            if check(state):
                return state
        raise TimeoutError('关闭监控后的任务或命令条件未满足')

    try:
        cli('pause')
        state = stationary_patrol_state(api)
        assert state['team_ready']
        cli('save', '--output', str(run/'results/monitor_view'))
        ready = ReadinessHold(MEMBERS, all_members=True)
        state = wait(lambda s: ready.update(s, time.monotonic()), 60)
        route = None
        for name, robot in state['robots'].items():
            p = robot['pose']
            peers = [r['pose'] for n, r in state['robots'].items() if n != name]
            data, meta = load_map(run/'results/monitor_view'/('replica_'+name+'.yaml'))
            grid = Grid(data, meta['resolution'], meta['origin'][:2], .18, .35)
            points = short_patrol(grid, p, peers, length=.8)
            if points is not None:
                route = dict(routes={name: points}, coordinate_frame=state['coordinate_frame'],
                             cycles=10, dwell=0, goal_timeout=120)
            if route:
                break
        if not route:
            raise RuntimeError('观测地图没有监控退出测试所需的短程路线')
        plan = run/'monitor-patrol.json'
        plan.write_text(json.dumps(route))
        cli('patrol', '--file', str(plan))
        wait(lambda s: abs(s['robots'][name]['safety'].get('v', 0.)) > .04, 60)
        processes = [p for p in snapshot().values() if p.env.get(SESSION) == session
                     and any(Path(a).name in ('peer_gateway', 'dashboard') for a in p.argv[:2])]
        assert len(processes) == 2, [(p.pid, p.argv) for p in processes]
        before = api.state(.1)['robots'][name]['distance']
        for process in processes:
            report['monitor_processes'].append(dict(pid=process.pid, argv=process.argv))
            _send(process, signal.SIGINT)
        end = time.monotonic()+8
        while time.monotonic() < end and any(still_alive(p) for p in processes):
            api.state(.1)
        assert not any(still_alive(p) for p in processes)
        state = wait(lambda s: s['robots'][name]['distance']-before > .25, 20)
        report['continued_distance'] = state['robots'][name]['distance']-before
        cli('pause')
        wait(lambda s: all(not r.get('goal') and abs(r['safety'].get('v', 1.)) < .001
                          and abs(r['safety'].get('w', 1.)) < .001 for r in s['robots'].values()), 5)
        cli('patrol', '--file', str(plan))
        wait(lambda s: abs(s['robots'][name]['safety'].get('v', 0.)) > .04, 60)
        cli('estop')
        wait(lambda s: s['estop'] and all(abs(r['safety'].get('v', 1.)) < .001 for r in s['robots'].values()), 5)
        cli('clear-estop')
        wait(lambda s: not s['estop'] and all(not r.get('goal') for r in s['robots'].values()), 5)
        report.update(passed=True, task_continued=True, direct_new_task=True, direct_pause=True,
                      direct_estop=True, direct_clear_does_not_resume=True)
    except Exception as error:
        report['failure'] = type(error).__name__+': '+str(error)
        raise
    finally:
        try:
            cli('pause')
        finally:
            (run/'peer-monitor-acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            if api._peer:
                api._peer.close()
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
