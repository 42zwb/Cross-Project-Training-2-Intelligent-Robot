import numpy as np
import pytest
from fleet_mapping.observed_completion import ObservedCompletion, observed_enclosure
from fleet_mapping.registration import dynamic_fusion


def enclosure():
    data = np.full((150, 160), -1, np.int16)
    data[20:121, 25:126] = 0
    data[20, 25:126] = data[120, 25:126] = 100
    data[20:121, 25] = data[20:121, 125] = 100
    return data


@pytest.mark.parametrize('yaw', [0., .43, -1.2])
def test_boundary_comes_from_four_observed_faces_in_arbitrary_relative_coordinates(yaw):
    data, origin = dynamic_fusion([(enclosure(), .1, -5., -5., 0., (11., -23., yaw))], .1, padding=1.3)
    model = ObservedCompletion()
    for i in range(4):
        output, details = model.update(data, .1, origin, i, float(i))
    assert details['boundary_confirmed']
    assert details['boundary_obstacle_fraction'] > 0
    assert np.array_equal(output[data == 0], data[data == 0])


def test_three_walls_or_a_measured_doorway_cannot_confirm_a_closed_boundary():
    data = enclosure()
    data[120, 26:125] = -1
    assert observed_enclosure(data, .1, (-5., -5.)) is None
    data = enclosure()
    data[20, 65:76] = 0
    assert observed_enclosure(data, .1, (-5., -5.)) is None


def test_repeated_same_map_revision_does_not_mature_wall_inference():
    data = enclosure()
    model = ObservedCompletion()
    for i in range(4):
        _, details = model.update(data, .1, (-5., -5.), 1, float(i))
    assert not details['boundary_confirmed']
