#!/usr/bin/env python3
"""Sequential frozen-source 3-seed by 3-layout functional acceptance."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--reuse-first-run', help='复用同一冻结源码、正常结束且通过完整功能的种子24布局0运行')
    parser.add_argument('--reuse-runs', help='前一矩阵目录；只复用当前版本已通过的场景')
    args = parser.parse_args()
    out, root = Path(args.output).resolve(), Path(__file__).resolve().parents[1]
    if out.exists() and any(out.iterdir()):
        parser.error('请使用新的空矩阵目录，保留已有验收证据')
    out.mkdir(parents=True, exist_ok=True)
    results, frozen = [], None
    for seed in (24, 42, 2026):
        for layout in (0, 1, 2):
            reused = bool(args.reuse_first_run and (seed, layout) == (24, 0))
            run = Path(args.reuse_first_run).resolve() if reused else out/f'seed-{seed}-layout-{layout}'
            if not reused and args.reuse_runs:
                candidate = Path(args.reuse_runs).resolve()/f'seed-{seed}-layout-{layout}'
                if (candidate/'acceptance.json').exists():
                    from peer_release_evidence import freeze, functional_run
                    try:
                        functional_run(candidate, freeze(root))
                    except (AssertionError, OSError, KeyError, ValueError):
                        pass
                    else:
                        run, reused = candidate, True
            if not reused:
                with (out/(run.name+'.log')).open('w') as stream:
                    child = subprocess.run([sys.executable, str(root/'scripts/one_click_demo.py'), '--headless',
                        '--architecture', 'decentralized', '--seed', str(seed), '--spawn-layout', str(layout),
                        '--run-dir', str(run)], cwd=root, stdout=stream, stderr=subprocess.STDOUT)
            manifest = json.loads((run/'manifest.json').read_text())
            assert manifest['architecture'] == 'decentralized' and manifest['initialization'] == 'unknown'
            assert manifest['spawn_layout'] == layout and manifest.get('acceptance_scene') in (None, 'none')
            assert yaml.safe_load((run/'assets/fleet.yaml').read_text())['scene']['seed'] == seed
            assert all((root/name).is_file() and hashlib.sha256((root/name).read_bytes()).hexdigest() == sha
                       for name, sha in manifest['source_sha256'].items()), '运行源码与当前工作区不一致'
            if frozen is None:
                frozen = manifest['source_sha256']
                (out/'source-sha256.json').write_text(json.dumps(frozen, indent=2))
            assert manifest['source_sha256'] == frozen, '矩阵运行中的源码发生改变，不能作为冻结版本验收'
            returncode = 0 if reused else child.returncode
            passed = returncode == 0
            if passed:
                acceptance = json.loads((run/'acceptance.json').read_text())
                cleanup = json.loads((run/'cleanup.json').read_text())
                passed = bool(acceptance['functional_pass'] and not cleanup['abnormal_process_exits']
                              and not cleanup['residual_before_final_kill'] and cleanup['launch_exit'] == 0)
                assert json.loads((run/'results/final/map_roundtrip.json').read_text())['passed']
                assert json.loads((run/'peer-runtime-acceptance.json').read_text())['passed']
                assert not json.loads((run/'detached_cleanup.json').read_text())['remaining']
            result = dict(seed=seed, layout=layout, run=str(run), returncode=returncode, passed=passed, reused=reused)
            results.append(result)
            (out/'peer-matrix-acceptance.json').write_text(json.dumps(dict(passed=len(results) == 9 and all(
                r['passed'] for r in results), completed=len(results), required=9, cases=results), ensure_ascii=False, indent=2))
            print(result, flush=True)
            if not passed:
                raise RuntimeError('本场景门禁失败，停止矩阵并保留日志；修复后重新运行受影响的冻结版本门禁')


if __name__ == '__main__':
    main()
