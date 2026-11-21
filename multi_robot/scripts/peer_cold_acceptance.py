#!/usr/bin/env python3
"""Five peer cold starts, scoped member restart and orphan recovery."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from runtime_context import RuntimeClient
from fleet_bringup.simulation_preflight import snapshot, SESSION
from fleet_control.relative_evaluate import rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--count', type=int, default=5)
    args = parser.parse_args()
    if args.count < 5:
        parser.error('完整生命周期验收至少需要五次冷启动')
    root, out = Path(__file__).resolve().parents[1], Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    results, orphan_session = [], None
    for index in range(args.count):
        run = out/f'cold-{index+1}'
        if run.exists():
            raise RuntimeError('请使用新的空验收目录，保留历史记录：'+str(run))
        with (out/f'cold-{index+1}.log').open('w') as log:
            child = subprocess.Popen([str(root/'run.sh'), '--architecture', 'decentralized', '--headless',
                '--no-dashboard', '--record-truth', '--seed', '24', '--run-dir', str(run)],
                cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        api, force, session = RuntimeClient(run), index == 2, None
        try:
            end = time.monotonic()+180
            while time.monotonic() < end:
                if child.poll() is not None:
                    raise RuntimeError('冷启动监督进程提前退出：'+str(child.returncode))
                if (run/'architecture.json').exists():
                    session = json.loads((run/'architecture.json').read_text())['session_id']
                    state = api.state(.2)
                    if all(r.get('ready') for r in state['robots'].values()):
                        break
                time.sleep(.2)
            else:
                raise TimeoutError('冷启动本车链路未就绪')
            if orphan_session:
                assert not any(p.env.get(SESSION) == orphan_session for p in snapshot().values())
            subprocess.run([sys.executable, str(root/'scripts/peer_runtime_acceptance.py'),
                            '--run-dir', str(run)], check=True, timeout=25)
            if index == 1:
                original = {n: r['boot_id'] for n, r in state['robots'].items()}
                subprocess.run(['ros2', 'run', 'fleet_bringup', 'peer_restart', 'robot2', '--run-dir', str(run)],
                               check=True, timeout=25)
                end = time.monotonic()+120
                while time.monotonic() < end:
                    state = api.state(.2)
                    r = state['robots']['robot2']
                    if r.get('ready') and r['boot_id'] != original['robot2']:
                        break
                else:
                    raise TimeoutError('冷启动中的单车重启未就绪')
                assert state['robots']['robot1']['boot_id'] == original['robot1']
                assert state['robots']['robot3']['boot_id'] == original['robot3']
                assert all(not r.get('goal') for r in state['robots'].values())
            first, last = {}, {}
            for item in rows(run/'truth.jsonl'):
                for name, pose in item['robots'].items():
                    first.setdefault(name, pose)
                    last[name] = pose
            assert len(first) == 3 and all(math.dist(first[n][:2], last[n][:2]) < .015
                                          for n in first), (first, last)
            if index == args.count-1:
                result = api.request('command', dict(command='explore'), 20)
                assert result['success'], result
                end = time.monotonic()+120
                while time.monotonic() < end:
                    state = api.state(.2)
                    if any(abs(r.get('safety', {}).get('v', 0.)) > .04
                           for r in state['robots'].values()):
                        break
                else:
                    raise TimeoutError('最后一次冷启动未产生运动，不能验证运动中的正常关闭')
            if force:
                child.kill()  # Leave the world and tagged descendants for next startup preflight.
                child.wait(timeout=5)
                assert any(p.env.get(SESSION) == session for p in snapshot().values())
                orphan_session = session
        finally:
            if api._peer:
                api._peer.close()
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        if not force:
            cleanup = json.loads((run/'cleanup.json').read_text())
            assert child.returncode == 0 and not cleanup['abnormal_process_exits'], cleanup
            assert not any(p.env.get(SESSION) == session for p in snapshot().values())
        results.append(dict(run=str(run), passed=True, forced_exit=force,
            supervisor_exit=child.returncode, previous_orphans_removed=bool(orphan_session and not force),
            shutdown_during_motion=index == args.count-1))
        (out/'peer-cold-acceptance.json').write_text(json.dumps(dict(passed=len(results) == args.count,
            completed=len(results), required=args.count, runs=results), ensure_ascii=False, indent=2))
        print(results[-1], flush=True)


if __name__ == '__main__':
    main()
