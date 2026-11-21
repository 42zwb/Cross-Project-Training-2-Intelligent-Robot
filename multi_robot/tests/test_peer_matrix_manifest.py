"""Exercise matrix bookkeeping at a stubbed launch boundary, with no physics."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml


@pytest.fixture
def matrix_run(tmp_path, monkeypatch):
    project = Path(__file__).resolve().parents[1]
    script = project/'scripts/peer_matrix_acceptance.py'
    spec = importlib.util.spec_from_file_location('matrix_manifest_under_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = {'scripts/peer_matrix_acceptance.py': hashlib.sha256(script.read_bytes()).hexdigest()}

    def execute(scene, *, reuse=False):
        launched = []

        def write_run(run, seed, layout):
            (run/'assets').mkdir(parents=True)
            (run/'results/final').mkdir(parents=True)
            manifest = dict(architecture='decentralized', initialization='unknown',
                            spawn_layout=layout, source_sha256=source)
            if scene != 'omitted':
                manifest['acceptance_scene'] = scene
            records = {'manifest.json': manifest, 'acceptance.json': dict(functional_pass=True),
                'cleanup.json': dict(launch_exit=0, abnormal_process_exits=[], residual_before_final_kill=[]),
                'detached_cleanup.json': dict(remaining=[]),
                'peer-runtime-acceptance.json': dict(passed=True),
                'results/final/map_roundtrip.json': dict(passed=True)}
            for name, value in records.items():
                (run/name).write_text(json.dumps(value))
            (run/'assets/fleet.yaml').write_text(yaml.safe_dump(dict(scene=dict(seed=seed))))

        def launch(command, **kwargs):
            seed = int(command[command.index('--seed')+1])
            layout = int(command[command.index('--spawn-layout')+1])
            run = Path(command[command.index('--run-dir')+1])
            launched.append((seed, layout))
            write_run(run, seed, layout)
            return SimpleNamespace(returncode=0)

        out = tmp_path/'matrix'
        arguments = [str(script), '--output', str(out)]
        if reuse:
            first = tmp_path/'already_completed_first_run'
            write_run(first, 24, 0)
            arguments += ['--reuse-first-run', str(first)]
        monkeypatch.setattr(sys, 'argv', arguments)
        monkeypatch.setattr(module.subprocess, 'run', launch)
        module.main()
        return json.loads((out/'peer-matrix-acceptance.json').read_text()), launched

    return execute


@pytest.mark.parametrize('scene', [None, 'none', 'omitted'])
def test_nine_ordinary_manifests_complete_the_matrix_for_all_supported_encodings(matrix_run, scene):
    result, launched = matrix_run(scene)
    assert result['passed'] and result['completed'] == result['required'] == 9
    assert launched == [(seed, layout) for seed in (24, 42, 2026) for layout in (0, 1, 2)]
    assert len(result['cases']) == 9 and all(case['passed'] and not case['reused'] for case in result['cases'])


def test_reusing_the_first_ordinary_null_scene_launches_only_the_remaining_eight(matrix_run):
    result, launched = matrix_run(None, reuse=True)
    assert result['passed'] and result['completed'] == 9
    assert result['cases'][0]['reused'] and result['cases'][0]['seed'] == 24
    assert result['cases'][0]['layout'] == 0
    assert launched == [(seed, layout) for seed in (24, 42, 2026) for layout in (0, 1, 2)][1:]


@pytest.mark.parametrize('scene', ['separated-rooms', 'symmetric-square', ''])
def test_negative_or_malformed_scenes_cannot_count_as_an_ordinary_matrix_case(matrix_run, scene):
    with pytest.raises(AssertionError):
        matrix_run(scene)
