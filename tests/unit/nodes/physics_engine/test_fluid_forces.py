import numpy as np
import pytest
from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation

@pytest.fixture
def medium_force_setup():
    lift_coefficients = np.array([[0, -1.08], [45, -1.09], [90, 0.0],  [135, 1.09], [180, 1.08]])
    drag_coefficients = np.array([[0, 0.225], [45, 0.140], [90, 0.012], [135, 0.143], [180, 0.143]])
    areas = np.array([[0, 100.0], [45, 100.0], [90, 100.0], [135, 100.0], [180, 100.0]])
    fluid_density = 1.226 
 
    computation = MediumForceComputation(lift_coefficients, drag_coefficients, areas, fluid_density)
    return computation

def test_initialization(medium_force_setup):
    assert isinstance(medium_force_setup.lift_coefficients, np.ndarray)
    assert isinstance(medium_force_setup.drag_coefficients, np.ndarray)
    assert isinstance(medium_force_setup.areas, np.ndarray)
    assert isinstance(medium_force_setup.fluid_density, (int, float))

@pytest.mark.parametrize("apparent_velocity, orientation, expected_angle", [
    # Tests if apparent velocity is 0
    (np.array([0, 0]), 0, 0),
    (np.array([0, 0]), 45, 45),
    (np.array([0, 0]), 90, 90),
    (np.array([0, 0]), 180, 180),
    (np.array([0, 0]), 270, 90),
    (np.array([0, 0]), 360, 0),
    (np.array([0, 0]), 450, 90),

    # Tests if apparent velocity is not 0
    (np.array([1, 0]), 0, 0),
    (np.array([0, 1]), 0, 90),
    (np.array([-1, 0]), 0, 180),
    (np.array([0, -1]), 0, 90),
    (np.array([1, 1]), 0, 45),
    (np.array([1, -1]), 0, 45),
    (np.array([-1, 1]), 0, 135),
    (np.array([-1, -1]), 0, 135),

    # Tests for apparent velocity other than unit vectors
    (np.array([2, 0]), 0, 0),
    (np.array([0, 2]), 0, 90),
    (np.array([-2, 0]), 0, 180),
    (np.array([0, -2]), 0, 90),
    (np.array([2, 2]), 0, 45),
    (np.array([2, -2]), 0, 45),
    (np.array([-2, 2]), 0, 135),
    (np.array([-2, -2]), 0, 135),

    # Tests for orientation other than 0
    (np.array([1, 0]), 45, 45),
    (np.array([0, 1]), 45, 45),
    (np.array([-1, 0]), 45, 135),
    (np.array([0, -1]), 45, 135),
    (np.array([1, 1]), 45, 0),
    (np.array([1, -1]), 45, 90),
    (np.array([-1, 1]), 45, 90),
    (np.array([-1, -1]), 45, 180),
])
def test_calculate_attack_angle(apparent_velocity, orientation, expected_angle, medium_force_setup):
    attack_angle = medium_force_setup.calculate_attack_angle(apparent_velocity, orientation)
    assert attack_angle == expected_angle


@pytest.mark.parametrize("attack_angle, expected_lift, expected_drag, apparent_velocity", [
    # Tests for attack angle 0
    (0, -2756, 575, np.array([80, 80])),
    # Tests for attack angle 45
    (45, -2794, 359, np.array([80, 80])),
    # Tests for attack angle 90
    (90, 0, 31, np.array([80, 80])),
    # Tests for attack angle 135
    (135, 2794, 367, np.array([80, 80])),
    # Tests for attack angle 180
    (180, 2756, 773, np.array([80, 80])),
])
def test_compute_forces(medium_force_setup, attack_angle, expected_lift, expected_drag, apparent_velocity):
    lift_force, drag_force = medium_force_setup.compute(apparent_velocity, attack_angle)
    calculate_magnitude = lambda force: np.sqrt(force[0]**2 + force[1]**2)
    print(lift_force, drag_force)
    assert np.isclose(calculate_magnitude(lift_force), expected_lift, rtol=0.01)
    assert np.isclose(calculate_magnitude(drag_force), expected_drag, rtol=0.01)



def test_interpolation(medium_force_setup):
    attack_angle = 45  # Directly between two defined points
    apparent_velocity = np.array([5, 5])
    lift_force, drag_force = medium_force_setup.compute(apparent_velocity, 0)
    lift_coefficient, drag_coefficient, area = medium_force_setup.interpolate(attack_angle)
    medium_force_setup.visualize_forces(apparent_velocity, lift_force, drag_force)
    # Check that interpolation falls between known values
    assert 1.0 < lift_coefficient < 1.5
    assert 0.1 < drag_coefficient < 0.2
    assert 10.0 < area < 10.5

    
