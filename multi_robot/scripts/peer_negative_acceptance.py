#!/usr/bin/env python3
"""Observe an intentionally unidentifiable physical scene; never force fusion."""
import argparse
import json
import math
from pathlib import Path
import time
import yaml
from runtime_context import RuntimeClient
from fleet_control.relative_evaluate import rows
from fleet_control.evaluate import clearance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--seconds', type=float, default=180.)
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    deadline = time.monotonic()+180
    while not all((run/name).exists() for name in ('manifest.json', 'architecture.json')):
        if time.monotonic() >= deadline:
            raise TimeoutError('负例运行元数据未生成')
        time.sleep(.1)
    manifest = json.loads((run/'manifest.json').read_text())
    assert manifest.get('acceptance_scene') in ('separated-rooms', 'symmetric-square')
    api, report = RuntimeClient(run), dict(passed=False, case=manifest['acceptance_scene'])
    def pump_until_ready():
        end = time.monotonic()+180
        while time.monotonic() < end:
            try:
                state = api.state(.2)
            except (OSError, ValueError, RuntimeError):
                time.sleep(.2)
                continue
            if all(r.get('ready') for r in state['robots'].values()):
                return state
        raise TimeoutError('负例的本车链路未就绪')
    try:
        state = pump_until_ready()
        result = api.request('command', dict(command='explore'), 20)
        assert result['success'], result
        end, maximum_aligned = time.monotonic()+args.seconds, 0
        while time.monotonic() < end:
            state = api.state(.2)
            maximum_aligned = max(maximum_aligned, sum(r.get('aligned', False) for r in state['robots'].values()))
            if maximum_aligned or state.get('cooperative_ready'):
                raise AssertionError('不可辨识负例被误接纳为已对齐：'+json.dumps(state['mapping']))
        result = api.request('command', dict(command='pause'), 20)
        assert result['success'], result
        truth = rows(run/'truth.jsonl')
        gt = yaml.safe_load((run/'assets/ground_truth.yaml').read_text())
        obstacles = gt.get('obstacles', gt.get('boxes', []))
        moving_violations, speed_violations, minimum = 0, 0, math.inf
        for previous, current in zip(truth, truth[1:]):
            dt = current['sim_time']-previous['sim_time']
            if dt <= 0:
                continue
            moving = 0
            for name, pose in current['robots'].items():
                minimum = min(minimum, clearance(pose, obstacles, .18))
                if name in previous['robots']:
                    old = previous['robots'][name]
                    speed = math.dist(pose[:2], old[:2])/dt
                    yaw_rate = abs(math.atan2(math.sin(pose[2]-old[2]), math.cos(pose[2]-old[2])))/dt
                    moving += int(speed > .015 or yaw_rate > .05)
                    speed_violations += int(speed > .095)
            moving_violations += int(moving > 1)
        report.update(passed=not moving_violations and not speed_violations and minimum >= -.001,
            maximum_aligned=maximum_aligned, bootstrap_parallel_violations=moving_violations,
            bootstrap_speed_violations=speed_violations, minimum_static_clearance=minimum,
            observed_seconds=args.seconds)
        assert report['passed'], report
    finally:
        try:
            api.request('command', dict(command='pause'), timeout=8)
        finally:
            (run/'peer-negative-acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            if api._peer:
                api._peer.close()
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
