"""A missed aggregate sample needs three valid overlapping local intervals."""
import copy
import json
import math
from pathlib import Path

import pytest

from fleet_control.relative_evaluate import durable_exploration_completion


def captured():
    return json.loads((Path(__file__).parent/'data/exploration_completion.json').read_text())


def evaluate(tmp_path, value, clock='steady_time'):
    for name, events in value['events'].items():
        directory = tmp_path/name
        directory.mkdir(exist_ok=True)
        (directory/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    return durable_exploration_completion(tmp_path, value['telemetry'], clock)


@pytest.mark.parametrize('clock', ['steady_time', 'sim_time'])
def test_real_joint_completion_shorter_than_a_recording_period_is_preserved(tmp_path, clock):
    value = captured()
    assert not any(s['mode'] == 'EXPLORED' for s in value['telemetry'])
    result = evaluate(tmp_path, value, clock)
    assert result is not None and result['method'] == 'three_agent_durable_events'
    expected = max(e[clock] for events in value['events'].values()
                   for e in events if e['event'] == 'exploration_complete')
    assert result['time'] == expected
    assert 0. < result['joint_completion_window'] < .5
    assert set(result['participants']) == {'robot1', 'robot2', 'robot3'}


def test_callback_append_order_does_not_replace_the_observation_clock(tmp_path):
    value = captured()
    for events in value['events'].values():
        events.reverse()
    assert evaluate(tmp_path, value) is not None


@pytest.mark.parametrize('invalid', ['missing_completion', 'wrong_mission', 'retired_boot',
    'changed_epoch', 'resumed', 'after_pause', 'wrong_frame', 'later_mission',
    'nonfinite_time', 'boolean_time', 'moving_context'])
def test_incomplete_or_invalid_history_cannot_manufacture_joint_completion(tmp_path, invalid):
    value = captured()
    events = value['events']
    third = next(e for e in events['robot3'] if e['event'] == 'exploration_complete')
    finished = max(e['steady_time'] for rows in events.values()
                   for e in rows if e['event'] == 'exploration_complete')
    stopped = min(e['steady_time'] for rows in events.values()
                  for e in rows if e.get('command') == 'pause')
    if invalid in ('missing_completion', 'later_mission'):
        events['robot3'].remove(third)
    elif invalid == 'wrong_mission':
        third['mission_id'] = 'unrelated-task'
    elif invalid == 'retired_boot':
        third['boot_id'] = 'retired'
    elif invalid == 'changed_epoch':
        third['map_epoch'] += 1
    elif invalid == 'resumed':
        claim = copy.deepcopy(next(e for e in events['robot1'] if e['event'] == 'exploration_complete'))
        claim.update(event='frontier_claim', steady_time=finished+.01, resumed_from_completed=True)
        events['robot1'].append(claim)
    elif invalid == 'after_pause':
        third['steady_time'] = stopped+.005
    elif invalid == 'wrong_frame':
        for sample in value['telemetry']:
            sample['robots']['robot3']['frame_epoch'] = 'other-frame'
    elif invalid == 'moving_context':
        for sample in value['telemetry']:
            sample['robots']['robot3']['safety']['v'] = .2
    elif invalid == 'nonfinite_time':
        third['steady_time'] = math.inf
    elif invalid == 'boolean_time':
        third['steady_time'] = True
    if invalid == 'later_mission':
        for name, records in events.items():
            start = copy.deepcopy(records[0])
            start.update(mission_id='later-recovery', command_id='later-recovery',
                         steady_time=stopped+100., sim_time=1000.)
            done = copy.deepcopy(start)
            done.update(event='exploration_complete', steady_time=stopped+110., sim_time=1010.)
            del done['command'], done['command_id']
            records.extend([start, done])
        context = copy.deepcopy(next(s for s in value['telemetry'] if s['team_ready']))
        context.update(steady_time=stopped+110., sim_time=1010., mode='EXPLORING')
        for robot in context['robots'].values():
            robot.update(mission_id='later-recovery', goal=None)
            robot['safety'].update(v=0., w=0.)
        value['telemetry'].append(context)
    assert evaluate(tmp_path, value) is None
