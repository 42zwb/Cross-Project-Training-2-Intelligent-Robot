#!/usr/bin/env python3
"""Peer-mode mission acceptance. No simulator truth is read during control."""
import argparse
import json
from pathlib import Path
import time
from runtime_context import RuntimeClient
from fleet_control.geometry import Grid
from fleet_mapping.grid import load_map
from fleet_control.peer_protocol import MEMBERS
from peer_acceptance_readiness import ReadinessHold
from peer_acceptance_routes import formation_target, short_patrol


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--explore-timeout', type=float, default=1200)
    parser.add_argument('--formation-timeout', type=float, default=500)
    parser.add_argument('--resume-after-exploration', action='store_true',
                        help='Development continuation only; does not establish initialization/exploration acceptance')
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    api = RuntimeClient(run)
    log = (run/'peer-demo.jsonl').open('a', buffering=1)

    def record(event, **values):
        item = dict(event=event, wall_time=time.time(), steady_time=time.monotonic(), **values)
        log.write(json.dumps(item, ensure_ascii=False)+'\n')
        print(item, flush=True)

    def send(kind, data):
        result = api.request(kind, data, timeout=25)
        record('command', kind=kind, data=data, result=result)
        if not result['success']:
            raise RuntimeError(result['message'])
        return result

    def wait(check, seconds, context):
        end, last_report = time.monotonic()+seconds, 0.
        while time.monotonic() < end:
            state = api.state(timeout=.3)
            if check(state):
                return state
            failed = [n+': '+r.get('message', '本车任务失败') for n, r in state.get('robots', {}).items()
                      if r.get('mode') == 'FAULT']
            if state.get('mode') in ('FAULT', 'ESTOP') or failed:
                raise RuntimeError('；'.join(failed) if failed else state.get('message', context))
            if time.monotonic()-last_report > 15:
                record('progress', context=context, mode=state.get('mode'), ready=state.get('team_ready'),
                       robots={n: {k: r.get(k) for k in ('pose', 'travel', 'goal', 'safety')} for n, r in state['robots'].items()})
                last_report = time.monotonic()
            time.sleep(.3)
        raise TimeoutError(context+'超时')

    def saved_replicas(directory):
        send('command', {'command': 'save', 'argument': str(directory)})
        ready = ReadinessHold(MEMBERS, all_members=True)
        state = wait(lambda s: ready.update(s, time.monotonic()), 60, '保存后定位及停车确认')
        frame = json.loads((directory/'frame.json').read_text())
        if frame['frame_epoch'] != state['coordinate_frame']:
            raise RuntimeError('保存地图和当前位姿的共同坐标不一致')
        replicas = {}
        for name in MEMBERS:
            data, meta = load_map(directory/f'replica_{name}.yaml')
            replicas[name] = Grid(data, meta['resolution'], meta['origin'][:2], .18, .35)
        return state, replicas

    completed = False
    try:
        wait(lambda s: s.get('local_ready'), 180, '本车链路初始化')
        if args.resume_after_exploration:
            send('command', {'command': 'pause'})
            wait(lambda s: s.get('team_ready'), 10, '续测前共同坐标检查')
            record('development_continuation', initialization_acceptance=False,
                   exploration_acceptance=False)
        else:
            send('command', {'command': 'explore'})
            started = time.monotonic()
            wait(lambda s: s.get('team_ready'), 300, '未知起点对齐')
            record('team_initialized', seconds=time.monotonic()-started)
            # Pause/resume uses only reported control status; physical stopping is
            # checked later by the offline truth evaluator.
            wait(lambda s: min(r['travel'] for r in s['robots'].values()) >= 3, 250, '三车探索参与')
            send('command', {'command': 'pause'})
            wait(lambda s: s['mode'] == 'IDLE' and all(abs(r['safety'].get('v', 1)) < .001
                 and abs(r['safety'].get('w', 1)) < .001 for r in s['robots'].values()), 8, '暂停停车')
            send('command', {'command': 'explore'})
            wait(lambda s: s['mode'] == 'EXPLORED', max(1, args.explore_timeout-(time.monotonic()-started)), '自治探索完成')
            record('exploration_complete', seconds=time.monotonic()-started)
        state, replicas = saved_replicas(run/'results/explored')
        poses = {name: state['robots'][name]['pose'] for name in MEMBERS}
        routes = {}
        for name in MEMBERS:
            route = short_patrol(replicas[name], poses[name], [poses[n] for n in MEMBERS if n != name])
            if route is None:
                raise RuntimeError(name+' 没有可验证的短程巡逻路线')
            routes[name] = route
        send('command', {'command': 'patrol', 'argument': json.dumps(dict(routes=routes,
            coordinate_frame=state['coordinate_frame'], cycles=1, dwell=0, goal_timeout=120))})
        wait(lambda s: s['mode'] == 'PATROL_COMPLETE', 300, '航点巡逻')
        for shape in ('triangle', 'row', 'column'):
            directory = run/'results'/f'formation-{shape}-input'
            state, replicas = saved_replicas(directory)
            poses = {name: state['robots'][name]['pose'] for name in MEMBERS}
            target = formation_target(replicas, poses, shape)
            record('formation_target', shape=shape, target=target, poses=poses, maps=str(directory))
            if target is None:
                raise RuntimeError('当前三个地图副本中没有可共同通过的'+shape+'队形路线')
            send('formation', target)
            wait(lambda s: s['mode'] == 'FORMATION_READY', args.formation_timeout, '静态队形 '+shape)
            record('formation_ready', shape=shape)
            time.sleep(1.)  # Make the ready state observable by independent recorders.
        send('command', {'command': 'estop'})
        time.sleep(1.)
        if not api.state(timeout=.2).get('estop'):
            raise RuntimeError('急停未锁定')
        send('command', {'command': 'clear-estop'})
        wait(lambda s: s['mode'] == 'IDLE' and not s['estop'], 10, '解除急停')
        send('command', {'command': 'save', 'argument': str(run/'results/final')})
        record('missions_complete')
        completed = True
    finally:
        if not completed:
            try:
                record('failure_pause', result=api.request('command', {'command': 'pause'}, timeout=8))
            except Exception as error:
                record('failure_pause_error', message=str(error))
        if api._peer:
            api._peer.close()
        log.close()


if __name__ == '__main__':
    main()
