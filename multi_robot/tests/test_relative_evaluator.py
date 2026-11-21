import math
import numpy as np
import pytest
import json
from fleet_control.relative_evaluate import (
    collision_metrics, evaluate, fixed_gauge, formation_completions, full_arena_coverage,
    identified_formation, rows,
)
from fleet_control.geometry import formation


def test_three_formation_types_are_verified_from_slot_geometry():
    for shape in ('triangle', 'row', 'column'):
        assert identified_formation(formation(shape, 4., -2., 1.1, 1.2)) == shape
    assert identified_formation([[0., 0., 0.], [1., 0., 0.], [2., 1., 0.]]) is None


def test_brief_ready_transition_requires_all_three_durable_completion_events(tmp_path):
    slots = formation('column', 0., 0., 0., 1.2)
    for name in ('robot1', 'robot2', 'robot3'):
        path = tmp_path/name
        path.mkdir()
        events = [dict(event='command_committed', command='formation', sim_time=10.),
                  dict(event='formation_ready', sim_time=20., slots=slots, assignment=[0, 1, 2])]
        (path/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    assert len(formation_completions(tmp_path, [])) == 1
    (tmp_path/'robot3/events.jsonl').write_text('')
    assert formation_completions(tmp_path, []) == []


def test_reducing_exported_map_never_reduces_truth_coverage_denominator():
    args = dict(resolution=.1, world_from_team=(0., 0., 0.), arena=(-5., -5., 5., 5.),
                obstacles=[], starts=[(-3., 0.), (0., 0.), (3., 0.)],
                seeds={'robot1': [-3., 0., 0.], 'robot2': [0., 0., 0.], 'robot3': [3., 0., 0.]})
    full = full_arena_coverage(np.zeros((100, 100), np.int8), origin=(-5., -5.), **args)
    small = full_arena_coverage(np.zeros((10, 10), np.int8), origin=(-.5, -.5), **args)
    assert full['reachable_cells'] == small['reachable_cells']
    assert full['coverage'] == 1. and small['coverage'] < .02
    assert not small['coverage_pass']


def test_gauge_is_initial_anchor_sample_and_does_not_best_fit_later_error():
    def sample(t, x):
        return dict(sim_time=t, coordinate_frame='frozen', mapping=dict(anchor_id='robot1', anchor_boot='a', aligned=['robot1']),
                    robots={'robot1': {'pose': [x, 0., 0.], 'boot_id': 'a'}})
    telemetry = [sample(1., 0.), sample(2., 20.)]
    truth = [dict(sim_time=1., robots={'robot1': [5., 5., math.pi/2]}),
             dict(sim_time=2., robots={'robot1': [5., 6., math.pi/2]})]
    gauge, details = fixed_gauge(telemetry, truth, 'frozen')
    assert gauge == pytest.approx((5., 5., math.pi/2))
    assert details['sim_time'] == 1.
    with pytest.raises(ValueError, match='禁止事后拟合'):
        fixed_gauge(telemetry, truth, 'wrong-epoch')
def test_clock_rewind_associates_by_monotonic_observation_instead_of_duplicate_sim_time():
    from fleet_control.relative_evaluate import association_clock, fixed_gauge
    telemetry = [dict(sim_time=10., steady_time=100., coordinate_frame='f',
        mapping=dict(anchor_id='robot1', anchor_boot='a', aligned=['robot1']),
        robots=dict(robot1=dict(pose=(0., 0., 0.), boot_id='a')))]
    truth = [dict(sim_time=10., steady_time=90., robots=dict(robot1=(8., 0., 0.))),
             dict(sim_time=1., steady_time=100., robots=dict(robot1=(2., 0., 0.)))]
    clock = association_clock(telemetry, truth)
    assert clock == 'steady_time'
    transform, _ = fixed_gauge(telemetry, truth, 'f', clock)
    assert transform == (2., 0., 0.)


def test_legacy_clock_rewind_is_rejected_instead_of_silently_misassociating():
    from fleet_control.relative_evaluate import association_clock
    with pytest.raises(ValueError, match='时间不单调'):
        association_clock([dict(sim_time=10.), dict(sim_time=1.)], [dict(sim_time=1.)])


def test_initial_gauge_uses_the_pose_timestamp_not_the_later_aggregate_poll_time():
    telemetry = [dict(sim_time=10., steady_time=100., coordinate_frame='f',
        mapping=dict(anchor_id='robot1', anchor_boot='a', aligned=['robot1']),
        robots=dict(robot1=dict(pose=(0., 0., 0.), boot_id='a', pose_steady_time=90.)))]
    truth = [dict(sim_time=1., steady_time=90., robots=dict(robot1=(1., 2., .1))),
             dict(sim_time=10., steady_time=100., robots=dict(robot1=(5., 6., 1.)))]
    transform, evidence = fixed_gauge(telemetry, truth, 'f', 'steady_time')
    assert transform == pytest.approx((1., 2., .1))
    assert evidence['observation_time'] == 90. and evidence['pose_timestamp_recorded']


def test_restarted_robot_is_checked_in_the_original_gauge_and_cannot_be_refitted():
    from fleet_control.relative_evaluate import first_boot_alignment_errors
    samples = [dict(steady_time=float(i), coordinate_frame='f', robots=dict(robot1=dict(
        boot_id=boot, map_epoch=i, ready=True, aligned=True, pose=pose)))
        for i, boot, pose in [(1, 'original', (0., 0., 0.)), (2, 'restart', (1., 0., 0.))]]
    truth = [dict(steady_time=float(i), robots=dict(robot1=(5., 0., 0.))) for i in (1, 2)]
    errors = first_boot_alignment_errors(samples, truth, 'f', (5., 0., 0.), 'steady_time')
    assert [x['position'] for x in errors] == [0., 1.]


def test_missing_restart_truth_is_reported_instead_of_skipped(tmp_path):
    from fleet_control.relative_evaluate import first_boot_alignment_errors, alignment_identity_audit
    names = ('robot1', 'robot2', 'robot3')
    robots = {n: dict(boot_id=n, map_epoch=1, ready=True, aligned=True, pose=(0., 0., 0.)) for n in names}
    samples = [dict(steady_time=1., coordinate_frame='f', robots=robots),
               dict(steady_time=2., coordinate_frame='f', robots=dict(robot1=dict(
                   boot_id='restart', map_epoch=2, ready=True, aligned=True, pose=(0., 0., 0.))))]
    truth = [dict(steady_time=1., robots={n: (0., 0., 0.) for n in names})]
    history = first_boot_alignment_errors(samples, truth, 'f', (0., 0., 0.), 'steady_time')
    result = alignment_identity_audit(tmp_path, samples, history)
    assert len(history) == 3 and not result['passed']
    assert result['missing'] == [dict(robot='robot1', boot_id='restart', map_epoch=2)]


@pytest.mark.parametrize('source', ['event', 'raw'])
def test_accepted_identity_absent_from_telemetry_is_still_required(tmp_path, source):
    from fleet_control.relative_evaluate import alignment_identity_audit
    if source == 'event':
        (tmp_path/'robot1').mkdir()
        (tmp_path/'robot1/events.jsonl').write_text(json.dumps(dict(event='alignment_ready',
            robot='robot1', boot_id='lost', map_epoch=4))+'\n')
    else:
        (tmp_path/'raw-map-checkpoints').mkdir()
        np.savez(tmp_path/'raw-map-checkpoints/robot1-e4-first-aligned-robot1.npz',
                 robot='robot1', boot='lost', epoch=4)
    result = alignment_identity_audit(tmp_path, [], [])
    assert not result['passed']
    assert result['missing'] == [dict(robot='robot1', boot_id='lost', map_epoch=4)]


def write_rows(path, items):
    path.write_text(''.join(json.dumps(item)+'\n' for item in items))


@pytest.fixture
def evaluator_run(tmp_path, monkeypatch):
    import fleet_control.relative_evaluate as ev
    import yaml
    names = ('robot1', 'robot2', 'robot3')
    run = tmp_path/'run'
    (run/'assets').mkdir(parents=True)
    (run/'results/final').mkdir(parents=True)
    (run/'architecture.json').write_text(json.dumps(dict(architecture='decentralized')))
    (run/'assets/fleet.yaml').write_text(yaml.safe_dump(dict(arena=[-5, -5, 5, 5], robot_radius=.18,
        robots=[dict(start=[i*2., 0.]) for i in range(3)])))
    (run/'assets/ground_truth.yaml').write_text(
        yaml.safe_dump(dict(obstacles=[['test-block', 0., 3., 1., 1.]])))
    (run/'results/final/frame.json').write_text(json.dumps(dict(frame_epoch='f')))
    (run/'resources.jsonl').write_text(json.dumps(dict(rss_mb=100.))+'\n')
    monkeypatch.setattr(ev, 'load_map', lambda path: (np.zeros((100, 100), np.int8),
        dict(resolution=.1, origin=[-5., -5., 0.], free_thresh=.196)))
    for name in names:
        (run/name).mkdir()
        events = [dict(event='command_committed', command=c, steady_time=t, sim_time=t,
                       robot=name, boot_id=name, map_epoch=1, mission_id=m, command_id=m)
                  for t, c, m in [(1., 'explore', 'initial'), (6.1, 'pause', 'pause'), (8., 'explore', 'recovery')]]
        (run/name/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))

    def execute(*, rotate=False, recovery=False, output=None):
        def poses(t):
            return {n: [i*2.+(0. if rotate else t*.03)+(3. if t >= 8 else 0.), 0., t*.1 if rotate else 0.]
                    for i, n in enumerate(names)}

        telemetry = []
        end = 11 if recovery else 6
        for t in range(end+1):
            mode = 'EXPLORING' if 1 <= t <= 5 or t >= 8 else 'EXPLORED' if t == 6 else 'IDLE'
            telemetry.append(dict(steady_time=float(t), sim_time=float(t), coordinate_frame='f', mode=mode,
                team_ready=t >= 1, region_enabled=True, region_seeds={n: [i*2., 0.] for i, n in enumerate(names)},
                mapping=dict(anchor_id='robot1', anchor_boot='robot1', aligned=list(names)),
                robots={n: dict(boot_id=n, map_epoch=1, ready=True, aligned=True, pose=p)
                        for n, p in poses(t).items()}))
        truth_times = [i/8 for i in range(end*8+1)]
        write_rows(run/'peer-observations.jsonl', telemetry)
        write_rows(run/'truth.jsonl', [dict(steady_time=t, sim_time=t, robots=poses(t)) for t in truth_times])
        return ev.evaluate(run, output=output)
    return run, execute


def test_three_spinning_robots_do_not_pass_parallel_translation(evaluator_run):
    _, execute = evaluator_run
    result = execute(rotate=True)
    assert result['concurrency_truth_samples'][3] > 0
    assert result['parallel_translating_truth_samples'] == 0
    assert not result['parallel_exploration_observed']


def test_recovery_does_not_inflate_initial_exploration_metrics(evaluator_run):
    _, execute = evaluator_run
    before, after = execute(), execute(recovery=True)
    assert before['parallel_exploration_observed']
    assert after['actual_explore_distance'] == before['actual_explore_distance']
    assert after['parallel_translating_truth_samples'] == before['parallel_translating_truth_samples']
    assert all(v > 0 for v in after['recovery_explore_distance'].values())
    assert after['initial_exploration_window']['end'] == 6.


def test_explicit_output_preserves_prior_acceptance_file(evaluator_run):
    run, execute = evaluator_run
    (run/'acceptance.json').write_text('historical evidence')
    output = run/'new-evaluation.json'
    execute(output=output)
    assert output.is_file() and (run/'acceptance.json').read_text() == 'historical evidence'


@pytest.mark.parametrize('contact', ['static', 'interrobot'])
def test_partial_truth_contact_survives_telemetry_gap(evaluator_run, contact):
    run, execute = evaluator_run
    assert execute()['collision_pass']
    truth = rows(run/'truth.jsonl')
    sample = next(row for row in truth if row['sim_time'] == 3.)
    if contact == 'static':
        sample['robots'] = dict(robot2=[0., 3., 0.])
        expected_clearance = -.18
    else:
        pose = sample['robots']['robot2']
        sample['robots'] = dict(robot2=pose, robot3=pose)
        expected_clearance = -.36
    write_rows(run/'truth.jsonl', truth)
    control = evaluate(run)

    log = run/'peer-observations.jsonl'
    write_rows(log, [row for row in rows(log) if abs(row['steady_time']-3.) > 1.])
    masked = evaluate(run)
    for report in (control, masked):
        assert report[f'minimum_{contact}_clearance'] == pytest.approx(expected_clearance)
        assert report['collision_detected'] and report['collision_evidence_pass']
        assert not report['collision_pass'] and not report['functional_pass']
        assert report['collision_evidence']['max_sample_gap_s'] == .25


def test_no_paired_truth_is_an_evidence_failure_with_null_clearance(evaluator_run):
    run, execute = evaluator_run
    execute()
    path = run/'truth.jsonl'
    write_rows(path, [dict(row, robots={'robot1': row['robots']['robot1']}) for row in rows(path)])
    report = evaluate(run)
    assert report['minimum_static_clearance'] > 0.
    assert report['minimum_interrobot_clearance'] is None
    assert json.loads((run/'acceptance.json').read_text())['minimum_interrobot_clearance'] is None
    assert not report['collision_detected'] and not report['collision_evidence_pass']
    assert not report['collision_pass'] and not report['functional_pass']
    assert report['collision_evidence']['paired_samples'] == 0
    assert 'missing_interrobot_clearance' in report['collision_evidence']['issues']


@pytest.mark.parametrize(('complete_times', 'max_gap', 'issues'), [
    ([1., 1.125, 1.25], .125, []),
    ([1., 1.], None, ['insufficient_complete_samples']),
    ([1., 1.375], .375, ['complete_sample_gap']),
    ([1.125, 1., 1.125], .125, ['invalid_sim_time']),
])
def test_collision_coverage_uses_complete_distinct_gazebo_times(complete_times, max_gap, issues):
    full_team = {f'robot{i}': [float(i), 0., 0.] for i in (1, 2, 3)}
    partial_team = {n: full_team[n] for n in ('robot1', 'robot2')}
    records = [dict(sim_time=t, robots=full_team) for t in complete_times]
    # Partial startup/teardown does not extend full-team coverage. Wall
    # delays, and a partial row inside the window, cannot fill a Gazebo gap.
    records.insert(1, dict(sim_time=1.125, robots=partial_team))
    records.insert(0, dict(sim_time=0., robots=partial_team))
    records.append(dict(sim_time=5., robots=partial_team))
    for i, record in enumerate(records):
        record['steady_time'] = i*.6
    report = collision_metrics(records, [], .18)
    evidence = report['collision_evidence']
    assert evidence['max_sample_gap_s'] == max_gap and evidence['issues'] == issues
    assert evidence['complete_samples'] == len(complete_times)
    assert evidence['paired_samples'] == len(records)
    assert report['minimum_interrobot_clearance'] == pytest.approx(.64)
    assert report['minimum_static_clearance'] is None
    assert not report['collision_detected']
    assert report['collision_evidence_pass'] == report['collision_pass'] == (not issues)


def test_a_later_mission_cannot_complete_the_initial_exploration(tmp_path):
    from fleet_control.relative_evaluate import initial_exploration_window
    folder = tmp_path/'robot1'
    folder.mkdir()
    events = [dict(steady_time=t, event='command_committed', command='explore', boot_id='b', map_epoch=1)
              for t in (1., 3.)]
    (folder/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    telemetry = [dict(steady_time=t, mode=m, coordinate_frame='f')
                 for t, m in [(1., 'EXPLORING'), (3., 'EXPLORING'), (5., 'EXPLORED')]]
    start, end, completion = initial_exploration_window(tmp_path, telemetry, 'steady_time', 'f')
    assert start == 1. and end == 3. and completion is None


@pytest.mark.parametrize('foreign_resume', [False, True])
def test_captured_initial_pause_resume_is_bound_to_exact_harness_command_ids(tmp_path, foreign_resume):
    from pathlib import Path
    from fleet_control.relative_evaluate import initial_exploration_window
    data = json.loads((Path(__file__).parent/'data/initial_exploration_pause_resume.json').read_text())
    for name, events in data['events'].items():
        (tmp_path/name).mkdir()
        (tmp_path/name/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    if foreign_resume:
        data['transcript'][2]['result']['command_id'] = 'different-recovery-command'
    (tmp_path/'peer-demo.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in data['transcript']))
    events = data['events']['robot1']
    start = events[0]['steady_time']+.1
    completion = max(e['steady_time'] for events in data['events'].values() for e in events
                     if e['event'] == 'exploration_complete')+.1
    samples = [dict(steady_time=start, mode='EXPLORING', coordinate_frame='f'),
               dict(steady_time=completion, mode='EXPLORED', coordinate_frame='f')]
    _, end, done = initial_exploration_window(tmp_path, samples, 'steady_time', 'f')
    assert (done is None) == foreign_resume
    assert (end == completion) != foreign_resume
