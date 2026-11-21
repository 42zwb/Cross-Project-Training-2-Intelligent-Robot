"""Restart one autonomy tree without touching world infrastructure or peers."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid
import yaml
from .simulation_preflight import (SESSION, PROJECT, OWNER, START, read_process,
                                   snapshot, still_alive, _send)


def selected(processes, session, robot):
    return [p for p in processes.values()
            if p.env.get(SESSION) == session and p.env.get(PROJECT) == 'fleet'
            and p.env.get('TB4_SIM_ROBOT') == robot
            and p.env.get('TB4_SIM_COMPONENT') == 'autonomy']


def stop_robot(session, robot, exclude_group=None):
    report = {'session_id': session, 'robot_id': robot, 'signals': [], 'targets': []}
    def candidates():
        result = selected(snapshot(), session, robot)
        if exclude_group is not None:
            filtered = []
            for p in result:
                try:
                    if os.getpgid(p.pid) != exclude_group:
                        filtered.append(p)
                except ProcessLookupError:
                    pass
            result = filtered
        return result
    targets = {p.pid: p for p in candidates()}
    report['targets'] = [{'pid': p.pid, 'start': p.start, 'argv': list(p.argv)} for p in targets.values()]
    for sig, timeout in ((signal.SIGINT, 6.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
        # A closing launch may spawn or reparent a child. Selection always uses
        # all three exact environment tags, never names or process groups.
        targets.update({p.pid: p for p in candidates()})
        alive = [p for p in targets.values() if still_alive(p)]
        if not alive:
            break
        live_pids = {p.pid for p in alive}
        roots = [p for p in alive if p.ppid not in live_pids]
        for p in roots if sig == signal.SIGINT else alive:
            _send(p, sig)
            report['signals'].append({'pid': p.pid, 'signal': int(sig)})
        end = time.monotonic()+timeout
        while time.monotonic() < end and any(still_alive(p) for p in alive):
            time.sleep(.05)
    report['remaining'] = [p.pid for p in candidates()]
    if report['remaining']:
        raise RuntimeError('本车旧进程未退出，拒绝叠加启动：'+str(report['remaining']))
    return report


def advance_identity(path):
    cfg = yaml.safe_load(path.read_text())
    name, assets = cfg['robot_id'], Path(cfg['assets'])
    old_epoch = cfg['map_epoch']
    cfg['map_epoch'], cfg['boot_id'] = old_epoch+1, uuid.uuid4().hex
    old_frame = name+'/map' if old_epoch == 1 else f'{name}/map_e{old_epoch}'
    frame = f'{name}/map_e{cfg["map_epoch"]}'
    lua = assets/f'{name}.lua'
    lua.write_text(re.sub(r'map_frame="[^"]+"', 'map_frame="'+frame+'"', lua.read_text(), count=1))
    navpath = assets/f'{name}_nav.yaml'
    nav = yaml.safe_load(navpath.read_text())

    def update(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'global_frame' and child == old_frame:
                    value[key] = frame
                else:
                    update(child)
        elif isinstance(value, list):
            for child in value:
                update(child)
    update(nav)
    navpath.write_text(yaml.safe_dump(nav))
    temp = path.with_suffix('.tmp')
    temp.write_text(yaml.safe_dump(cfg))
    temp.replace(path)
    return cfg


def stop_detached_session(session, main_group):
    """Close restarted autonomy, restored sensors and audit children as well."""
    report = dict(session_id=session, signals=[], remaining=[])
    targets = {}
    def candidates():
        result = []
        for p in snapshot().values():
            if p.pid == os.getpid() or p.env.get(SESSION) != session or p.env.get(PROJECT) != 'fleet':
                continue
            try:
                if os.getpgid(p.pid) != main_group:
                    result.append(p)
            except ProcessLookupError:
                pass
        return result
    for sig, timeout in ((signal.SIGINT, 12.), (signal.SIGTERM, 3.), (signal.SIGKILL, 2.)):
        targets.update({p.pid: p for p in candidates()})
        alive = [p for p in targets.values() if still_alive(p)]
        if not alive:
            break
        ids = {p.pid for p in alive}
        roots = [p for p in alive if p.ppid not in ids]
        for p in roots if sig == signal.SIGINT else alive:
            _send(p, sig)
            report['signals'].append(dict(pid=p.pid, signal=int(sig)))
        end = time.monotonic()+timeout
        while time.monotonic() < end and any(still_alive(p) for p in alive):
            time.sleep(.05)
    report['remaining'] = [p.pid for p in candidates()]
    return report


def main():
    p = argparse.ArgumentParser(description='在同一世界中重启一台车的自治链，不清理其他机器人')
    p.add_argument('robot', choices=['robot1', 'robot2', 'robot3'])
    p.add_argument('--run-dir', required=True)
    a = p.parse_args()
    run = Path(a.run_dir).expanduser().resolve()
    metadata = json.loads((run/'architecture.json').read_text())
    if metadata.get('architecture') != 'decentralized':
        raise ValueError('仅支持去中心化运行')
    manifest = json.loads((run/'manifest.json').read_text())
    for key in ('ROS_DOMAIN_ID', 'IGN_PARTITION'):
        if os.environ.get(key) != manifest['environment'][key]:
            raise ValueError('环境不属于这次仿真：'+key)
    supervisor = read_process(int((run/'supervisor.pid').read_text()))
    if not supervisor or not still_alive(supervisor):
        raise RuntimeError('本轮世界监督进程已退出')
    infrastructure = [p for p in snapshot().values() if p.env.get(SESSION) == metadata['session_id']
                      and p.env.get(PROJECT) == 'fleet'
                      and p.env.get(OWNER) == str(supervisor.pid) and p.env.get(START) == supervisor.start
                      and p.env.get('TB4_SIM_COMPONENT') != 'autonomy']
    if not infrastructure:
        raise RuntimeError('未发现本轮基础设施，拒绝启动孤立自治进程')
    lock = open(run/(a.robot+'-restart.lock'), 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    report = stop_robot(metadata['session_id'], a.robot)
    cfg = advance_identity(run/'assets/agents'/f'{a.robot}.yaml')
    env = dict(os.environ)
    for key in (SESSION, PROJECT, OWNER, START):
        env[key] = infrastructure[0].env[key]
    env.update(TB4_SIM_ROBOT=a.robot, TB4_SIM_COMPONENT='autonomy')
    logpath = run/a.robot/f'restart-{cfg["map_epoch"]}.log'
    with logpath.open('w') as log:
        proc = subprocess.Popen(['ros2', 'launch', 'fleet_bringup', 'robot.launch.py',
            'robot_config:='+str(run/'assets/agents'/f'{a.robot}.yaml')], env=env,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
    report.update(new_pid=proc.pid, new_boot=cfg['boot_id'], new_map_epoch=cfg['map_epoch'], log=str(logpath))
    (run/a.robot/f'restart-{cfg["map_epoch"]}.json').write_text(json.dumps(report, indent=2))
    print(f'{a.robot} 自治链已重新启动；等待本车就绪及重新配准。PID={proc.pid}，日志：{logpath}')


if __name__ == '__main__':
    main()
