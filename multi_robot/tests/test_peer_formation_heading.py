import math
from types import SimpleNamespace

import numpy as np
import pytest

from fleet_control.geometry import Grid
from fleet_control.peer_agent import PeerAgent, FORMATION_HEADING_CORRECTION_LIMIT


@pytest.fixture
def row():
    slots = [(0., -1.2, 0.), (0., 0., 0.), (0., 1.2, 0.)]
    plan = dict(plan_id='row', frame_epoch='team', slots=slots,
                assignment=[0, 1, 2], routes=[(i, slot, []) for i, slot in enumerate(slots)])
    velocity = SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=0.))
    peer = SimpleNamespace(mission_id='mission', nav_ready=True, estop=False, odom_valid=True,
                           mode='FORMING', velocity=velocity)
    peers = {name: (100., peer, dict(formation_plan_id='row', formation_step=3, formation_settled=True))
             for name in ('robot1', 'robot2')}
    events = []
    agent = SimpleNamespace(
        name='robot3', mode='FORMING', mission_id='mission', estop=False, work=None,
        release_pending=None, formation_arrival=(100., 744.), pose=(.06, 1.24, math.radians(26.5)),
        pose_steady_time=101.5, pose_sim_time=745., formation_heading_corrections=0,
        formation_stable=None, formation_settled=False, completed_step=2,
        grid=Grid(np.zeros((120, 120), dtype=np.int8), .05, (-3., -3.), .18, .35),
        mapping=(0., SimpleNamespace(frame_epoch='team')), cfg={'robot_radius': .18},
        stopped=lambda now: True, team_transform=lambda: (0., 0., 0.), all_aligned=lambda: True,
        live=lambda: peers, record=lambda event, **fields: events.append((event, fields)))
    agent.team_poses = lambda: dict(robot1=slots[0], robot2=slots[1], robot3=agent.pose)
    agent.formation_heading_goal = PeerAgent.formation_heading_goal.__get__(agent)
    agent.finish_formation = lambda now: PeerAgent.formation_finish_tick(agent, plan, now)
    return agent, plan, peers, events


def test_late_heading_error_queues_a_new_leased_goal_after_stop_and_fresh_pose(row):
    agent, plan, _, events = row
    # The first estimate would have passed at the Nav2 callback.
    agent.pose = (.06, 1.24, math.radians(1.45))
    agent.finish_formation(100.4)
    assert agent.work is None and agent.formation_stable is None
    agent.pose = (.06, 1.24, math.radians(26.5))
    agent.release_pending = ('old', 'team_motion', 'mission')
    agent.finish_formation(101.6)
    assert agent.work is None
    agent.release_pending = None
    agent.pose_sim_time = 744.
    agent.finish_formation(101.6)
    assert agent.work is None

    agent.pose_sim_time = 745.
    agent.finish_formation(101.6)
    assert agent.work['target'] == list(plan['slots'][2])
    assert agent.work['kind'] == 'formation' and agent.work['heading_correction']
    assert agent.resource == 'team_motion' and agent.request_id
    assert agent.completed_step == 2
    assert not agent.formation_settled and agent.formation_stable is None
    assert events[-1][0] == 'formation_heading_correction'


def test_good_pose_must_be_held_for_three_seconds_after_all_peers_settle(row):
    agent, plan, peers, events = row
    agent.pose = plan['slots'][2]
    peers['robot2'][2]['formation_settled'] = False
    agent.finish_formation(102.)
    assert agent.formation_settled and agent.formation_stable is None
    peers['robot2'][2]['formation_settled'] = True
    agent.finish_formation(103.)
    assert agent.formation_stable == 103.
    peers['robot2'][2]['formation_settled'] = False
    agent.finish_formation(104.)
    peers['robot2'][2]['formation_settled'] = True
    agent.finish_formation(107.)
    agent.finish_formation(109.99)
    assert agent.mode == 'FORMING' and agent.work is None
    agent.finish_formation(110.)
    assert agent.mode == 'FORMATION_READY' and events[-1][0] == 'formation_ready'


def test_heading_retry_respects_motion_and_map_clearance(row):
    agent, plan, peers, _ = row
    peers['robot2'][2]['motion_lease_held'] = True
    assert agent.formation_heading_goal(plan) is None
    peers['robot2'][2]['motion_lease_held'] = False
    agent.grid = Grid(np.full((120, 120), -1, dtype=np.int8), .05, (-3., -3.), .18, .35)
    assert agent.formation_heading_goal(plan) is None


def test_bounded_corrections_finish_in_a_stationary_fault(row):
    agent, _, _, events = row
    agent.formation_heading_corrections = FORMATION_HEADING_CORRECTION_LIMIT
    agent.finish_formation(101.6)
    assert agent.mode == 'FAULT' and agent.work is None
    assert events[-1][0] == 'formation_heading_correction_exhausted'
