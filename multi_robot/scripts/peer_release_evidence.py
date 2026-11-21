"""Strict release evidence validation; no simulation or controller imports."""
import hashlib
import json
import math
from pathlib import Path

MEMBERS = ('robot1', 'robot2', 'robot3')
COMPONENTS = ('peer_agent', 'local_safety', 'controller_server', 'scan_bridge', 'odom_tf')
DEFAULT_FILES = ('scripts/supervise.py', 'scripts/one_click_demo.py',
                 'src/fleet_bringup/launch/fleet.launch.py')
EXTENSIONS = {'.py', '.sh', '.yaml', '.lua', '.config', '.cfg', '.srv', '.msg', '.action',
              '.cpp', '.h', '.hpp', '.cmake', '.html', '.js', '.css', '.xml'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def freeze(root):
    root = Path(root)
    files = [p for directory in ('src', 'scripts', 'tools', 'vendor/tf2_compat')
             for p in (root/directory).rglob('*') if p.is_file()
             and (p.name == 'CMakeLists.txt' or p.suffix in EXTENSIONS)]
    files += [root/n for n in ('env.sh', 'build.sh', 'run.sh', 'demo.sh')]
    artifacts = [p for p in (root/'install/fleet_bridge/lib/fleet_bridge').iterdir() if p.is_file()]
    artifacts += [root/'vendor/runtime/lib/libtf2.so', root/'build/tools/truth_probe']
    return dict(source_sha256={str(p.relative_to(root)): sha(p) for p in sorted(files)},
                artifacts_sha256={str(p.relative_to(root)): sha(p) for p in sorted(artifacts)},
                tests_sha256={str(p.relative_to(root)): sha(p) for p in sorted((root/'tests').rglob('*'))
                              if p.is_file() and '__pycache__' not in p.parts})


def check_manifest(run, frozen):
    manifest = read(Path(run)/'manifest.json')
    assert manifest['architecture'] == 'decentralized' and manifest['initialization'] == 'unknown'
    assert manifest['source_sha256'] == frozen['source_sha256'], '运行源码不属于冻结版本'
    assert manifest['simulator_binary_sha256'] == frozen['artifacts_sha256'][
        'install/fleet_bridge/lib/fleet_bridge/fleet_sim_server'], '仿真二进制不属于冻结版本'


def clean_run(run):
    run = Path(run)
    cleanup = read(run/'cleanup.json')
    assert cleanup['launch_exit'] == 0 and not cleanup['abnormal_process_exits']
    assert not cleanup['residual_before_final_kill']
    assert not read(run/'detached_cleanup.json')['remaining']


def functional_run(run, frozen):
    run = Path(run)
    check_manifest(run, frozen)
    clean_run(run)
    assert read(run/'demo-run.json')['outcome'] == 'passed'
    for name in ('acceptance-functional-before-faults.json', 'acceptance.json'):
        value = read(run/name)
        assert value['functional_pass'] and value['alignment_identity_audit']['passed']
        assert not value['alignment_identity_audit']['missing']
    for name in ('peer-runtime-acceptance.json', 'results/final/map_roundtrip.json'):
        assert read(run/name)['passed']


def fault_key(case):
    return (case.get('robot'), case.get('component'), case['action'], case.get('jump_seconds'))


def expected_faults():
    values = {(n, c, a, None) for n in MEMBERS for c in COMPONENTS for a in ('freeze', 'exit')}
    values |= {(n, None, 'bidirectional_isolation', None) for n in MEMBERS}
    values |= {(None, None, a, None) for a in ('one_way_loss', 'loss_reorder_duplicates', 'late_votes',
        'expired_delayed_majority_does_not_restore_motion', 'world_pause_and_resume',
        'restart_preserves_estop_and_discards_goals')}
    values |= {(None, None, 'clock_freeze' if t == 0 else 'clock_jump', t) for t in (0., 120., -120.)}
    return values


def complete_faults(run):
    run = Path(run)
    cases, required = read(run/'peer-fault-acceptance.json'), expected_faults()
    assert len(cases) == len(required) and {fault_key(c) for c in cases} == required
    assert all(c.get('passed') is True for c in cases)
    assert read(run/'peer-fault-manifest.json')['full_matrix']
    for name in ('peer-fault-run.json', 'peer-fault-summary.json'):
        value = read(run/name)
        assert value['passed'] and value['full_matrix'] and value['completed_cases'] == len(required)
        assert value['process_cases'] == 30
    assert read(run/'peer-fault-run.json')['outcome'] == 'passed'
    for c in cases:
        if c.get('component'):
            assert c['boot_id'] and type(c['map_epoch']) is int and c['map_epoch'] > 1
            assert math.isfinite(c['restart_alignment_seconds']) and 0 <= c['restart_alignment_seconds'] <= 300.
        if c['action'] in ('one_way_loss', 'loss_reorder_duplicates', 'late_votes'):
            assert c['affected'] == 'robot2'
            assert c['no_old_task_resumption']['nonzero_samples'] == 0
            assert c['new_task_recovery']['displacement'] >= .05
            if c['action'] != 'late_votes':
                assert c['affected_motion']['displacement'] >= .05
            phases = ('no_old_task_resumption', 'new_task_recovery',
                      'authorization_window' if c['action'] == 'late_votes' else 'affected_motion')
            from peer_network_evidence import audit_motion
            for phase in phases:
                path = Path(c[phase]['evidence']).resolve()
                assert path.is_relative_to((run/'network-evidence').resolve())
                raw = read(path)
                assert raw['passed'] and raw['robot'] == 'robot2' and raw['raw']
                actual = audit_motion(raw['commands'], raw['guards'], raw['truth'], raw['robot'],
                                      raw['start'], raw['end'], **raw['options'])
                assert actual == raw['result'] == {k: v for k, v in c[phase].items() if k != 'evidence'}


def full_run(run, frozen):
    run = Path(run)
    functional_run(run, frozen)
    complete_faults(run)
    after = read(run/'acceptance-after-faults.json')
    assert after['functional_pass']
    evaluated = {(r['robot'], r['boot_id'], r['map_epoch']) for r in after['alignment_history']}
    for case in read(run/'peer-fault-acceptance.json'):
        if case.get('component'):
            assert (case['robot'], case['boot_id'], case['map_epoch']) in evaluated
    for name in ('peer-task-delivery-acceptance.json', 'peer-browser-acceptance.json', 'peer-monitor-acceptance.json'):
        assert read(run/name)['passed']


def evidence_index(directory):
    directory = Path(directory)
    return {str(p.relative_to(directory)): sha(p) for p in sorted(directory.rglob('*'))
            if p.is_file() and p.suffix in {'.json', '.jsonl', '.yaml', '.pgm', '.npz', '.png', '.log'}
            and p.name not in ('release-acceptance.json', 'release-evidence-sha256.json', 'release-progress.json')}
