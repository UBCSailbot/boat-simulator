"""Test the functions in boat_simulator/common/utils.py"""

from boat_simulator.common import utils
import pytest
import math


@pytest.mark.parametrize(
    "test_input, expected_output",
    [
        (math.pi, 180),
        (math.pi / 2, 90),
        (math.pi / 4, 45),
        (0, 0),
        (math.pi / 12, 180 / 12),
        (-math.pi / 4.5, -180 / 4.5),
    ],
)
def test_rad_to_degrees(test_input, expected_output):
    actual_output = utils.rad_to_degrees(test_input)
    assert math.isclose(actual_output, expected_output)


@pytest.mark.parametrize(
    "test_input, expected_output",
    [
        (180, math.pi),
        (90, math.pi / 2),
        (45, math.pi / 4),
        (0, 0),
        (180 / 12, math.pi / 12),
        (-180 / 4.5, -math.pi / 4.5),
    ],
)
def test_degrees_to_rad(test_input, expected_output):
    actual_output = utils.degrees_to_rad(test_input)
    assert math.isclose(actual_output, expected_output)
