"""This module provides a generator for fluid vectors used within the physics engine."""

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.generators import VectorGenerator
from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180


class FluidGenerator:
    """This class provides functionality to generate velocity vectors representing fluid movements.

    Attributes:
        `generator` (VectorGenerator): The vector generator used to generate fluid velocities.
        `velocity` (NDArray): The most recently generated fluid velocity vector, expressed in
            meters per second (m/s).
    """

    def __init__(self, generator: VectorGenerator):
        self.__generator = generator
        self.__velocity = np.array(self.__generator.next())

    def next(self) -> NDArray:
        """Generates the next velocity vector for the fluid simulation.

        Returns:
            NDArray: An array representing the updated velocity vector for the fluid simulation.
        """

        # TODO: Implement this method
        return np.array([])

    @property
    def velocity(self) -> NDArray:
        """Returns the fluid's current velocity vector.

        Returns:
            NDArray: The velocity vector of the fluid, expressed in meters per second (m/s) and
                ranging from negative infinity to positive infinity.
        """
        return self.__velocity

    @property
    def speed(self) -> Scalar:
        """Calculates the current speed of the fluid.

        Returns:
            Scalar: The speed of the fluid, expressed in meters per second (m/s) and within the
                range of 0 to positive infinity.
        """
        return np.linalg.norm(self.__velocity)

    @property
    def direction(self) -> Scalar:
        """Calculates the current direction of the fluid on the x-y plane.

        Returns:
            Scalar: The direction of the fluid, expressed in degrees and bounded between
                [-180, 180).
        """
        angle_rad = np.arctan2(self.__velocity[1], self.__velocity[0])
        angle_deg = np.degrees(angle_rad)
        return bound_to_180(angle_deg)
