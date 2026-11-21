import math
from fleet_mapping.registration import compose, inverse, ConstraintGraph
from fleet_control.peer_continuity import constraint_consistent_with_odometry, component_bodies_consistent


def test_join_cannot_put_new_robot_on_an_existing_third_member():
    a, b, c = ('robot1', 'a', 1), ('robot2', 'b', 1), ('robot3', 'c', 1)
    graph = ConstraintGraph()
    graph.add(a, b, (5., 0., 0.))
    peers = {name: dict(boot_id=boot, map_epoch=1, pose=(0., 0., 0.),
                       local_ready=True, received=10.) for name, boot, _ in (a, b, c)}
    assert not component_bodies_consistent(graph, b, c, (-5., 0., -math.pi/2), peers, 10.1)
    assert component_bodies_consistent(graph, b, c, (5., 0., 0.), peers, 10.1)
    peers['robot1']['received'] = 9.
    assert component_bodies_consistent(graph, b, c, (-5., 0., -math.pi/2), peers, 10.1)
    peers['robot1'].update(received=10., boot_id='retired')
    assert component_bodies_consistent(graph, b, c, (-5., 0., -math.pi/2), peers, 10.1)


def test_identical_nonoverlapping_rooms_cannot_claim_collocated_robot_bodies():
    a, b = ('robot1', 'a', 1), ('robot2', 'b', 1)
    peers = {'robot1': dict(boot_id='a', map_epoch=1, local_ready=True, pose=[0., 0., 0.]),
             'robot2': dict(boot_id='b', map_epoch=1, local_ready=True, pose=[0., 0., 0.])}
    assert not constraint_consistent_with_odometry(a, b, (0., 0., 0.), {}, peers, {}, 1., .2)


def test_restart_corner_alias_is_vetoed_without_using_truth_or_forcing_alignment():
    a, b = ('robot2', 'a', 1), ('robot3', 'new', 2)
    known = {a: (5., 0., 0.)}
    odom = (1.2, -.4, -.45)
    team_from_odom = (6.8, -1.2, 0.)
    body = compose(team_from_odom, odom)
    peer = dict(boot_id='new', map_epoch=2, pose=[0., 0., 0.], odom_pose=odom,
                odom_epoch='encoder-process', odom_valid=True, local_ready=True)
    priors = {'robot3': dict(boot_id='old', odom_epoch='encoder-process',
                             team_from_odom=team_from_odom, sigma=.05,
                             last_team_pose=body, last_seen=10.)}
    true_relative = compose(inverse(known[a]), body)
    assert constraint_consistent_with_odometry(a, b, true_relative, known, {'robot3': peer}, priors, 20., .2)
    assert not constraint_consistent_with_odometry(a, b, (-6.6, 1.9, -2.02), known, {'robot3': peer}, priors, 20., .2)
    peer['odom_valid'] = False
    assert not constraint_consistent_with_odometry(a, b, true_relative, known, {'robot3': peer}, priors, 20., .2)
    # With no prior correspondence, odometry does not manufacture one.
    assert constraint_consistent_with_odometry(a, b, true_relative, {}, {'robot3': peer}, {}, 20., .2)
    peer.update(odom_valid=True, odom_epoch='replacement-encoder-process')
    assert not constraint_consistent_with_odometry(a, b, (-6.6, 1.9, -2.02), known, {'robot3': peer}, priors, 20., .2)


def test_stale_encoder_calibration_cannot_permanently_veto_a_fresh_map_match():
    a, b = ('robot2', 'a', 1), ('robot3', 'new', 2)
    known = {a: (5., 0., 0.)}
    state = dict(boot_id='new', map_epoch=2, pose=(0., 0., 0.),
                 odom_pose=(12., 0., .5), odom_epoch='encoder',
                 odom_valid=True, local_ready=True)
    prior = dict(boot_id='old', odom_epoch='encoder', team_from_odom=(0., 0., 0.),
                 sigma=.05, last_team_pose=(8., 0., 0.), last_seen=0.)
    # The unique geometric candidate puts the body 2 m from its last position
    # after 20 s. The accumulated wheel heading/position drift is not a veto.
    assert constraint_consistent_with_odometry(a, b, (5., 0., 0.), known,
        {'robot3': state}, {'robot3': prior}, 20., .2)
    assert not constraint_consistent_with_odometry(a, b, (-8., 0., 0.), known,
        {'robot3': state}, {'robot3': prior}, 20., .2)
    prior['last_seen'] = 19.8
    assert not constraint_consistent_with_odometry(a, b, (5., 0., 0.), known,
        {'robot3': state}, {'robot3': prior}, 20., .2)
