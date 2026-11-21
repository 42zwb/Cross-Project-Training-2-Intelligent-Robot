#!/usr/bin/env python3
"""Own a complete demo invocation, phase evidence, backend and cleanup."""
import argparse
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from runtime_context import RuntimeClient
from acceptance_progress import AcceptanceProgress
from acceptance_process import run_command, stop_child, browser_environment


def main():
    p = argparse.ArgumentParser()
    display = p.add_mutually_exclusive_group()
    display.add_argument('--gui', action='store_true', help='打开 Gazebo 三维窗口')
    display.add_argument('--headless', action='store_true', help='仅运行仿真后台和网页（默认）')
    p.add_argument('--with-faults', action='store_true', help='另做故障与精度验收')
    p.add_argument('--seed', type=int)
    p.add_argument('--architecture', choices=['centralized', 'decentralized'],
                   help='三车控制架构，默认 decentralized')
    p.add_argument('--spawn-layout', type=int, choices=[0, 1, 2], default=0)
    p.add_argument('--no-dashboard', action='store_true')
    p.add_argument('--with-ui-checks', action='store_true')
    p.add_argument('--run-dir', help='新的空运行目录')
    a = p.parse_args()
    architecture_explicit = a.architecture is not None
    a.architecture = a.architecture or 'decentralized'
    if a.with_ui_checks and (a.architecture != 'decentralized' or a.no_dashboard):
        p.error('--with-ui-checks 需要启用网页的去中心化模式')
    root = Path(__file__).resolve().parents[1]
    run = Path(a.run_dir).resolve() if a.run_dir else root/'runs'/datetime.datetime.now().strftime('demo-%Y%m%d-%H%M%S')
    if run.exists() and any(run.iterdir()):
        p.error('运行目录已存在且非空')
    browser_env, browser_report = browser_environment(root) if a.with_ui_checks else (None, None)
    run.mkdir(parents=True, exist_ok=True)
    args = [str(root/'run.sh'), '--record-truth', '--run-dir', str(run),
            '--spawn-layout', str(a.spawn_layout)]
    if architecture_explicit:
        args += ['--architecture', a.architecture]
    if a.no_dashboard:
        args.append('--no-dashboard')
    if a.seed is not None:
        args += ['--seed', str(a.seed)]
    if a.gui:
        args.append('--gui')
    elif a.headless:
        args.append('--headless')
    backend = subprocess.Popen(args, start_new_session=True)
    api = RuntimeClient(run)
    progress = AcceptanceProgress(run, 'demo-run', api,
        backend_status=lambda: None if backend.poll() is None else '本轮仿真后台提前退出')

    def step(name, command, timeout, env=None):
        progress.stage(name, timeout)
        run_command(command, cwd=root, log=run/(name+'.log'), progress=progress, timeout=timeout, env=env)

    def script(name, timeout):
        step(name, [sys.executable, str(root/'scripts'/(name+'.py')), '--run-dir', str(run)], timeout,
             browser_env if name == 'peer_browser_acceptance' else None)

    def evaluate_phase(name):
        output = run/(name+'.json')
        command = [sys.executable, '-m', 'fleet_control.relative_evaluate', str(run), '--output', str(output)]
        if not a.with_faults:
            command.append('--report-only')
        step(name, command, 180)
        if not a.with_faults:
            report = json.loads(output.read_text())
            if not report['collision_pass']:
                message = ('独立仿真记录检测到碰撞' if report['collision_detected']
                           else '独立仿真真值不足以通过碰撞验收')
                raise RuntimeError(message+'，见 '+str(output))
            if not math.isfinite(report['peak_project_rss_mb']) or report['system_oom_kills_since_start']:
                raise RuntimeError('资源记录缺失或检测到内存不足退出，见 '+str(output))

    try:
        # The supervisor owns the empty-directory check. Do not race it by
        # publishing progress before it has created its first owned artifact.
        deadline = time.monotonic()+180
        while not (run/'startup_preflight.json').exists():
            progress.check()
            if time.monotonic() >= deadline:
                raise TimeoutError('监督进程未完成启动前检查')
            time.sleep(.1)
        if browser_report:
            (run/'browser-preflight.json').write_text(json.dumps(browser_report, indent=2))
        with progress:
            progress.stage('startup', 180)
            progress.wait(lambda s: s.get('local_ready') if a.architecture == 'decentralized'
                          else s.get('ready'), 180, startup=True)
            script('peer_demo' if a.architecture == 'decentralized' else 'demo', 3600)
            step('map-roundtrip', [sys.executable, str(root/'scripts/map_roundtrip.py'),
                                  str(run/'results/final/shared_map.yaml')], 120)
            if a.architecture == 'decentralized':
                script('peer_runtime_acceptance', 60)
                evaluate_phase('acceptance-functional-before-faults')
                if a.with_faults:
                    script('peer_task_delivery_acceptance', 900)
            if a.with_ui_checks:
                script('peer_browser_acceptance', 900)
                script('peer_monitor_acceptance', 600)
            if a.with_faults:
                script('peer_fault_acceptance' if a.architecture == 'decentralized' else 'safety_acceptance', 36000)
                if a.architecture == 'decentralized':
                    evaluate_phase('acceptance-after-faults')
            progress.stage('backend_cleanup', 90)
            stop_child(backend)
            progress.backend_status = lambda: None
            if backend.returncode != 0:
                raise RuntimeError('仿真后台退出异常，见 cleanup.json 和 launch.log')
            if a.architecture == 'decentralized':
                evaluate_phase('acceptance')
            progress.succeed(architecture=a.architecture, architecture_explicit=architecture_explicit,
                             invocation=sys.argv[1:], backend_exit=backend.returncode)
    except BaseException:
        stop_child(backend)
        # Preserve a partial-run assessment without replacing either the
        # original exception or the immutable functional checkpoint.
        if a.architecture == 'decentralized' and (run/'results/final/frame.json').exists():
            try:
                with (run/'acceptance-after-interruption.log').open('x') as log:
                    subprocess.run([sys.executable, '-m', 'fleet_control.relative_evaluate', str(run),
                        '--output', str(run/'acceptance-after-interruption.json')], cwd=root,
                        stdout=log, stderr=subprocess.STDOUT, timeout=180)
            except (OSError, subprocess.TimeoutExpired):
                pass
        raise
    finally:
        if api._peer:
            api._peer.close()
        stop_child(backend)
    print('演示及验收完成。结果：'+str(run), flush=True)


if __name__ == '__main__':
    main()
