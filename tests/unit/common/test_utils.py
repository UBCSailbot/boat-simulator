"""Test the functions in boat_simulator/common/utils.py"""

import math

import pytest

from boat_simulator.common import utils


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


@pytest.mark.parametrize(
    "test_input1, test_input2, expected_output",
    [
        ([0, 2 * math.pi], True, [0, 0]),
        ([3 / 2 * math.pi, -2.5 * math.pi], True, [-0.5 * math.pi, -0.5 * math.pi]),
        ([3 * math.pi, -3 * math.pi], True, [-math.pi, -math.pi]),
        ([4.44 * math.pi, -5.68 * math.pi], True, [0.44 * math.pi, 0.32 * math.pi]),
        ([1 / 36 * math.pi, -0.334 * math.pi], True, [1 / 36 * math.pi, -0.334 * math.pi]),
        ([0, 270, -450, 360], False, [0, -90, -90, 0]),
        ([540, -540, 899, -899, 5, -30], False, [-180, -180, 179, -179, 5, -30]),
    ],
)
def test_bound_to_180(test_input1, test_input2, expected_output):
    actual_output = utils.bound_to_180(test_input1, test_input2)
    for actual_angle, expected_angle in zip(actual_output, expected_output):
        assert math.isclose(actual_angle, expected_angle)
