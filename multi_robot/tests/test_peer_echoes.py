import numpy as np
from fleet_mapping.registration import ConstraintGraph, Snapshot, dynamic_fusion
from fleet_mapping.peer_echoes import filter_peer_echoes


def test_aligned_peer_history_cannot_imprison_robot_or_turn_unknown_into_free():
    own = np.zeros((80, 80), np.int16)
    own[:, 65] = 100  # A real obstacle in this robot's own observations.
    foreign = own.copy()
    foreign[36:44, 36:44] = 100  # Startup return on robot1, before alignment.
    a = Snapshot('robot1', 'a', 1, 1, own, .05, (-2., -2., 0.))
    b = Snapshot('robot2', 'b', 1, 1, foreign, .05, (-2., -2., 0.))
    graph = ConstraintGraph()
    peers = [('robot1', 'a', 1, (0., 0., 0.), 10., True),
             ('robot2', 'b', 1, (1., 0., 0.), 10., True)]
    assert np.array_equal(filter_peer_echoes(b, graph, peers, 10.1, .35), foreign)
    assert graph.add(('robot1', 'a', 1), ('robot2', 'b', 1), (0., 0., 0.))
    filtered = filter_peer_echoes(b, graph, peers, 10.1, .35)
    assert np.all(filtered[36:44, 36:44] == -1)
    assert np.all(filtered[:, 65] == 100)
    assert np.array_equal(b.data, foreign), 'raw source was mutated'
    # Only robot1's actual free observation can make its footprint free.
    fused, origin = dynamic_fusion([(a.data, .05, -2., -2., 0., (0., 0., 0.)),
                                    (filtered, .05, -2., -2., 0., (0., 0., 0.))], .05)
    cell = (int(-origin[1]/.05), int(-origin[0]/.05))
    assert fused[cell] == 0
    assert np.array_equal(filter_peer_echoes(b, graph, peers, 12., .35), foreign), 'stale peer was filtered'


def test_historical_footprints_require_original_free_evidence_for_every_erased_cell():
    own = np.zeros((80, 80), np.int16)
    own[40, 44] = 100  # Static object close to a historical body return.
    foreign = own.copy()
    foreign[37:43, 37:43] = 100
    a = Snapshot('robot1', 'a', 1, 1, own, .05, (-2., -2., 0.))
    b = Snapshot('robot2', 'b', 1, 1, foreign, .05, (-2., -2., 0.))
    graph = ConstraintGraph()
    graph.add(('robot1', 'a', 1), ('robot2', 'b', 1), (0., 0., 0.))
    samples = [('robot1', 'a', 1, (0., 0., 0.), 10., True)]
    result = filter_peer_echoes(b, graph, samples, 10.1, .35, {'robot1': a, 'robot2': b})
    assert np.all(result[37:43, 37:43] == -1)
    assert result[40, 44] == 100
    assert not np.any((result == 0) & (foreign != 0))
