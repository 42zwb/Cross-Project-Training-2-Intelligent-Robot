#!/usr/bin/env python3
"""Fresh-world recovery and two staged faults; explicitly not a full matrix."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
from acceptance_progress import AcceptanceProgress
from runtime_context import RuntimeClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--fully-explored', action='store_true',
                        help='先由机器人自主完成探索，再检查完整地图条件下的重启配准')
    args = parser.parse_args()
    root, run = Path(__file__).resolve().parents[1], Path(args.run_dir).resolve()
    if run.exists() and any(run.iterdir()):
        parser.error('请使用新的空运行目录')
    api = RuntimeClient(run)
    child = subprocess.Popen([str(root/'run.sh'), '--architecture', 'decentralized',
        '--headless', '--no-dashboard', '--record-truth', '--seed', '24', '--spawn-layout', '0',
        '--run-dir', str(run)], cwd=root, start_new_session=True)
    progress, active = None, None
    try:
        # Do not populate the directory until supervisor has accepted it.
        deadline = time.monotonic()+30
        while not (run/'manifest.json').exists():
            if child.poll() is not None:
                raise RuntimeError('后端启动退出：'+str(child.returncode))
            if time.monotonic() >= deadline:
                raise TimeoutError('后端运行清单未生成')
            time.sleep(.1)
        progress = AcceptanceProgress(run, 'peer-recovery-acceptance', api,
            backend_status=lambda: None if child.poll() is None else '本轮后端退出：'+str(child.returncode))
        progress.report.update(full_matrix=False, full_map_before_restart=args.fully_explored,
            scope='same-boot recovery and robot1 peer_agent freeze/exit')
        with progress:
            try:
                progress.stage('local_ready', 180)
                progress.wait(lambda s: s.get('local_ready'), 180, startup=True)
                exploration_started = time.monotonic()
                progress.request('explore')
                progress.stage('unknown_origin_alignment', 300)
                progress.wait(lambda s: s.get('team_ready'), 300)
                progress.stage('regional_participation', 250)
                progress.wait(lambda s: all(v.get('regional_travel', 0) >= 3.2
                    for v in s['robots'].values()), 250)
                if args.fully_explored:
                    remaining = max(1., 1200.-(time.monotonic()-exploration_started))
                    progress.stage('autonomous_full_exploration', remaining)
                    progress.wait(lambda s: s.get('mode') == 'EXPLORED', remaining)
                progress.request('pause')
                progress.stage('stationary_ready', 60)
                progress.wait(lambda s: s.get('team_ready') and all(
                    abs(v['safety'].get('v', 1)) < .001 and abs(v['safety'].get('w', 1)) < .001
                    for v in s['robots'].values()), 60)
                jobs = [
                    ('same_boot_recovery', 180, ['peer_task_delivery_acceptance.py'],
                     'peer-task-delivery-acceptance.json'),
                    ('two_staged_agent_faults', 1800, ['peer_fault_acceptance.py', '--robots', 'robot1',
                     '--components', 'peer_agent', '--actions', 'freeze', 'exit',
                     '--skip-network', '--skip-clock', '--skip-restart-estop'], 'peer-fault-summary.json')]
                for stage, timeout, arguments, result in jobs:
                    progress.stage(stage, timeout)
                    active = subprocess.Popen([sys.executable, str(root/'scripts'/arguments[0]),
                        '--run-dir', str(run), *arguments[1:]], cwd=root, start_new_session=True)
                    deadline = time.monotonic()+timeout
                    while active.poll() is None:
                        progress.state()
                        if time.monotonic() >= deadline:
                            raise TimeoutError(stage+' 子流程超时')
                    if active.returncode:
                        raise RuntimeError(stage+' 子流程退出：'+str(active.returncode))
                    value = json.loads((run/result).read_text())
                    assert value['passed'], value
                    progress.report[stage] = value
                faults = progress.report['two_staged_agent_faults']
                assert not faults['full_matrix'] and faults['process_cases'] == 2
                progress.succeed()
            finally:
                if active and active.poll() is None:
                    active.send_signal(signal.SIGINT)
                    try:
                        active.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        active.terminate()
                        active.wait(timeout=10)
                if api._peer:
                    api._peer.close()
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    child.wait(timeout=60)
                progress.report['backend_exit'] = child.returncode
                if child.returncode:
                    raise RuntimeError('后端未正常退出：'+str(child.returncode))
    finally:
        # This fallback only targets this harness's own supervisor.
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=60)
    print(progress.report, flush=True)


if __name__ == '__main__':
    main()
