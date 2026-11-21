#!/usr/bin/env python3
"""Close the operator after two commits; the recovered same-boot peer must join."""
import argparse
import json
from pathlib import Path
import signal
import time
from runtime_context import RuntimeClient
from fleet_bringup.simulation_preflight import snapshot, SESSION, _send
from acceptance_progress import AcceptanceProgress


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    run = Path(parser.parse_args().run_dir).resolve()
    api = RuntimeClient(run)
    session = json.loads((run/'architecture.json').read_text())['session_id']
    targets = [p for p in snapshot().values() if p.env.get(SESSION) == session
               and '__node:=scan_bridge' in p.argv and '__ns:=/robot3' in p.argv]
    if len(targets) != 1:
        raise RuntimeError('不能唯一确定本轮 robot3 雷达桥')
    process, frozen = targets[0], False
    progress = AcceptanceProgress(run, 'peer-task-delivery-acceptance', api)
    with progress:
        try:
            progress.stage('initial_local_ready', 30)
            state = progress.wait(lambda s: all(r.get('ready') for r in s['robots'].values()), 30)
            boot = state['robots']['robot3']['boot_id']
            progress.stage('sensor_frozen', 5, robot='robot3', boot_id=boot)
            _send(process, signal.SIGSTOP)
            frozen = True
            progress.wait(lambda s: not s['robots']['robot3'].get('ready'), 5)
            progress.stage('two_members_commit', 20)
            result = progress.request('explore')
            assert result['success'] and 'robot3' not in result['members'], result
            command = result['command_id']
            api._peer.close()  # No operator publisher remains to retransmit.
            api = RuntimeClient(run)
            api.stopping = lambda: progress.stop_reason is not None
            progress.api = api
            progress.stage('operator_closed_past_proposal_deadline', 21, command_id=command)
            held_seconds = progress.hold(21.)
            progress.stage('sensor_and_task_rejoin', 12, boot_id=boot, command_id=command)
            _send(process, signal.SIGCONT)
            frozen = False
            started = time.monotonic()
            state = progress.wait(lambda s: s['robots']['robot3'].get('ready')
                         and s['robots']['robot3'].get('mission_id') == command, 12)
            assert state['robots']['robot3']['boot_id'] == boot
            joined_seconds = time.monotonic()-started
            # Delivery can recover before the stationary localization certificate.
            # Pause and require that independent condition before the next suite.
            progress.stage('pause_and_stationary_map_confirmation', 60)
            progress.request('pause', 15)
            recovery_started = time.monotonic()
            state = progress.wait(lambda s: s.get('team_ready'), 60)
            assert state['robots']['robot3']['boot_id'] == boot
            progress.succeed(initial_members=result['members'], recovered_member='robot3',
                same_boot=True, join_seconds=joined_seconds, operator_publisher_closed=True,
                cooperative_ready_recovered=True,
                cooperative_recovery_seconds=time.monotonic()-recovery_started,
                recovered_after_proposal_deadline=held_seconds >= 21., held_seconds=held_seconds)
        finally:
            if frozen:
                _send(process, signal.SIGCONT)
                progress.report['frozen_process_resume_requested'] = True
            try:
                progress.check()
            except RuntimeError:
                pass  # Backend shutdown must not turn cancellation into another long wait.
            else:
                try:
                    progress.report['cleanup_pause'] = api.request('command', dict(command='pause'), 15)
                    if not progress.report['cleanup_pause'].get('success'):
                        progress.report.update(passed=False, outcome='failed')
                except (RuntimeError, OSError) as error:
                    progress.report['cleanup_error'] = str(error)
                    progress.report.update(passed=False, outcome='failed')
            finally:
                if api._peer:
                    api._peer.close()
    print(progress.report, flush=True)
    if not progress.report['passed']:
        raise RuntimeError('恢复验收收尾失败，见阶段报告')


if __name__ == '__main__':
    main()
