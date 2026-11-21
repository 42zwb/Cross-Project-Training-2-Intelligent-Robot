import copy
import numpy as np
from fleet_control.geometry import Grid
from fleet_control.peer_auction import quote_frontiers, frontier_winner


def test_same_frontier_is_assigned_by_each_members_feasible_path_cost_then_id():
    offers = {'robot1': dict(frontier_id='frontier', team_target=[0., 0., 0.], visible_unknown_m2=2.)}
    boots = {'robot1': 'a', 'robot2': 'b', 'robot3': 'c'}
    quotes = {'frontier': {n: dict(boot_id=b, cost=c, sent=10.) for (n, b), c in zip(boots.items(), (5., 2., 3.))}}
    assert frontier_winner(offers, quotes, boots, 10.1)[0] == 'robot2'
    quotes['frontier']['robot1']['cost'] = 2.
    assert frontier_winner(offers, quotes, boots, 10.1)[0] == 'robot1'
    quotes['frontier']['robot1']['cost'] = None
    assert frontier_winner(offers, quotes, boots, 10.1)[0] == 'robot2'
    assert frontier_winner(offers, quotes, boots, 16.1) is None
    quotes['frontier']['robot3']['boot_id'] = 'retired'
    assert frontier_winner(offers, quotes, boots, 10.1) is None


def test_quotes_use_observed_routes_and_preserve_relative_coordinate_transform():
    data = np.zeros((120, 120), np.int16)
    data[:, 55:65] = 100
    grid = Grid(data, .05, (-3., -3.), .18, .35)
    offers = {'robot2': dict(frontier_id='blocked', team_target=[3., 1., 0.], visible_unknown_m2=2.),
              'robot3': dict(frontier_id='free', team_target=[0., 1., 0.], visible_unknown_m2=1.)}
    before = copy.deepcopy(offers)
    result = quote_frontiers(grid, (-2., 0., 0.), (2., 1., 0.), offers, [])
    assert result['blocked'] is None
    assert result['free'] == 0.
    assert offers == before
