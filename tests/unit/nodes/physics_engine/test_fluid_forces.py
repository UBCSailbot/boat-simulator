import numpy as np
import pytest

from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation


class TestMediumForceComputation:
    @pytest.fixture
    def medium_force_computation(self):
        lift_coefficients = np.array([[0, 0], [10, 1], [20, 2]])
        drag_coefficients = np.array([[0, 0], [10, 1], [20, 2]])
        areas = np.array([[0, 1], [10, 2], [20, 3]])
        fluid_density = 1.0
        return MediumForceComputation(lift_coefficients, drag_coefficients, areas, fluid_density)

    @pytest.mark.parametrize(
        ["apparent_velocity", "orientation", "expected_attack_angle"],
        [
            (np.array([0, 0]), 0, 0),
            (np.array([3, 4]), 90, -36.86989764584402),
            (np.array([3, 4]), 180, -126.86989764584402),
            (np.array([3, 4]), 270, -216.86989764584402),
        ],
    )
    def test_compute(self, medium_force_computation):
        apparent_velocity = np.array([3, 4])
        orientation = 0
        lift_force, drag_force = medium_force_computation.compute(apparent_velocity, orientation)
        assert lift_force == 0
        assert drag_force == 0
