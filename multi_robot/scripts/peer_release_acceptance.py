#!/usr/bin/env python3
"""Own the frozen-version release gates; publish success only after default smoke."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from acceptance_progress import AcceptanceProgress, AcceptanceInterrupted
from acceptance_process import run_command, stop_child
from peer_release_evidence import (DEFAULT_FILES, freeze, read, check_manifest, clean_run,
                                   functional_run, full_run, evidence_index, sha)

GATES = ('full', 'matrix', 'separated-rooms', 'symmetric-square', 'cold', 'default')


def save(out, report):
    target = out/'release-acceptance.json'
    temp = target.with_suffix('.tmp')
    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    temp.replace(target)


def finalize_default(root, out, run):
    report, frozen = read(out/'release-acceptance.json'), read(out/'source-freeze.json')
    assert report['prerequisites_ready'] and all(report['gates'][n]['status'] == 'passed' for n in GATES[:-1])
    assert report['gates']['default']['status'] == 'not_run'
    current = freeze(root)
    changes = {}
    if current != frozen:
        # Older reports may have frozen the centralized defaults before migration.
        expected = dict(frozen['source_sha256'])
        for name in DEFAULT_FILES:
            old = (out/'frozen-defaults'/name).read_text()
            new = old.replace("default='centralized'", "default='decentralized'").replace(
                              "default_value='centralized'", "default_value='decentralized'")
            assert old != new and (root/name).read_text() == new, '默认切换包含额外源码变化：'+name
            expected[name] = sha(root/name)
            changes[name] = dict(before=frozen['source_sha256'][name], after=expected[name])
        assert current == dict(frozen, source_sha256=expected), '默认值之外的冻结内容发生变化'
    functional_run(run, current)
    demo = read(run/'demo-run.json')
    assert demo['architecture'] == 'decentralized' and demo['architecture_explicit'] is False
    report['gates']['default'] = dict(status='passed', run=str(run), changes=changes,
                                    evidence_sha256=evidence_index(run))
    report.update(passed=True, outcome='passed', final_freeze=current)
    save(out, report)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--reuse-first-run')
    p.add_argument('--reuse-matrix', help='仅复用该目录内同版本、证据完整的已通过场景')
    p.add_argument('--finalize-default-run', help='专项检查完成后，核对同版本无架构参数的实际运行')
    args = p.parse_args()
    root, out = Path(__file__).resolve().parents[1], Path(args.output).resolve()
    if args.finalize_default_run:
        finalize_default(root, out, Path(args.finalize_default_run).resolve())
        return
    if out.exists() and any(out.iterdir()):
        p.error('请使用新的空发布目录；历史报告保持不变')
    out.mkdir(parents=True, exist_ok=True)
    frozen = freeze(root)
    (out/'source-freeze.json').write_text(json.dumps(frozen, indent=2))
    for name in DEFAULT_FILES:
        destination = out/'frozen-defaults'/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((root/name).read_bytes())
    report = dict(passed=False, outcome='running', prerequisites_ready=False,
                  gates={n: dict(status='not_run') for n in GATES})
    save(out, report)
    progress = AcceptanceProgress(out, 'release-progress', SimpleNamespace(), backend_status=lambda: None)
    active = None

    def command(name, argv, timeout):
        # A demo first reaps its fault tool, then its supervisor, then writes an
        # offline interrupted-run assessment. Do not kill it at the same 90 s
        # deadline as its own first child's cleanup.
        run_command(argv, cwd=root, log=out/(name+'.log'), progress=progress, timeout=timeout,
                    shutdown_grace=420)

    def gate(name, action):
        nonlocal active
        assert freeze(root) == frozen, '验收过程中冻结内容发生变化'
        active = name
        report['gates'][name]['status'] = 'running'
        save(out, report)
        progress.stage(name, 86400)
        evidence = action()
        assert freeze(root) == frozen, '验收结束时冻结内容发生变化'
        report['gates'][name] = dict(status='passed', **evidence)
        save(out, report)
        active = None

    first = Path(args.reuse_first_run).resolve() if args.reuse_first_run else out/'full-seed24-layout0'

    def first_gate():
        if not args.reuse_first_run:
            command('full', [sys.executable, str(root/'scripts/one_click_demo.py'), '--headless',
                '--architecture', 'decentralized', '--seed', '24', '--spawn-layout', '0',
                '--with-faults', '--with-ui-checks', '--run-dir', str(first)], 43200)
        full_run(first, frozen)
        return dict(run=str(first), reused=bool(args.reuse_first_run), evidence_sha256=evidence_index(first))

    def matrix_gate():
        argv = [sys.executable, str(root/'scripts/peer_matrix_acceptance.py'), '--output', str(out/'matrix'),
                '--reuse-first-run', str(first)]
        if args.reuse_matrix:
            argv += ['--reuse-runs', str(Path(args.reuse_matrix).resolve())]
        command('matrix', argv, 43200)
        result = read(out/'matrix/peer-matrix-acceptance.json')
        assert result['passed'] and result['completed'] == result['required'] == 9
        assert {(c['seed'], c['layout']) for c in result['cases']} == {
            (s, l) for s in (24, 42, 2026) for l in (0, 1, 2)}
        for case in result['cases']:
            assert case['passed']
            functional_run(case['run'], frozen)
        return dict(result=str(out/'matrix/peer-matrix-acceptance.json'),
                    runs=[dict(run=c['run'], evidence_sha256=evidence_index(c['run'])) for c in result['cases']])

    def negative(scene):
        run = out/scene
        with (out/(scene+'-backend.log')).open('x') as log:
            child = subprocess.Popen([str(root/'run.sh'), '--headless', '--architecture', 'decentralized',
                '--record-truth', '--no-dashboard', '--seed', '24', '--acceptance-scene', scene,
                '--run-dir', str(run)], cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            command(scene, [sys.executable, str(root/'scripts/peer_negative_acceptance.py'),
                           '--run-dir', str(run)], 600)
        finally:
            stop_child(child)
        assert child.returncode == 0
        check_manifest(run, frozen)
        clean_run(run)
        result = read(run/'peer-negative-acceptance.json')
        assert result['passed'] and result['observed_seconds'] >= 180 and result['case'] == scene
        return dict(run=str(run))

    def cold_gate():
        command('cold', [sys.executable, str(root/'scripts/peer_cold_acceptance.py'),
                        '--output', str(out/'cold'), '--count', '5'], 2400)
        result = read(out/'cold/peer-cold-acceptance.json')
        assert result['passed'] and result['completed'] == result['required'] == 5
        assert len(result['runs']) == 5 and all(r['passed'] for r in result['runs'])
        for case in result['runs']:
            check_manifest(case['run'], frozen)
            if not case['forced_exit']:
                clean_run(case['run'])
        assert any(r['forced_exit'] for r in result['runs'])
        assert any(r['previous_orphans_removed'] for r in result['runs'])
        assert result['runs'][-1]['shutdown_during_motion']
        return dict(result=str(out/'cold/peer-cold-acceptance.json'))

    try:
        with progress:
            gate('full', first_gate)
            gate('matrix', matrix_gate)
            for scene in ('separated-rooms', 'symmetric-square'):
                gate(scene, lambda scene=scene: negative(scene))
            gate('cold', cold_gate)
            report.update(prerequisites_ready=True, outcome='awaiting_default_verification')
            progress.succeed(prerequisites_ready=True)
    except BaseException as error:
        outcome = 'interrupted' if isinstance(error, (KeyboardInterrupt, AcceptanceInterrupted)) else 'failed'
        report.update(outcome=outcome, error=repr(error))
        if active:
            report['gates'][active].update(status=outcome, error=repr(error))
        raise
    finally:
        save(out, report)
        (out/'release-evidence-sha256.json').write_text(json.dumps(evidence_index(out), indent=2))
    print('专项检查通过；尚需核对本冻结版本的默认入口运行。', flush=True)


if __name__ == '__main__':
    main()
