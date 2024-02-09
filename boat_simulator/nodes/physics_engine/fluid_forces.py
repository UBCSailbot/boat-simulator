"""This module provides functionality for computing the lift and drag forces acting on a medium."""

from typing import Tuple

from numpy.typing import NDArray

from boat_simulator.common.types import Scalar


class MediumForceComputation:
    """This class calculates the lift and drag forces experienced by a medium when subjected to
    fluid flow.

    Attributes:
        `lift_coefficients` (NDArray): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding lift
            coefficient.
        `drag_coefficients` (NDArray): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding drag
            coefficient.
        `areas` (NDArray): An array of shape (n, 2) where each row contains a pair (x, y),
            representing an angle of attack, in degrees, and its corresponding area, in square
            meters (m^2).
        `fluid_density` (Scalar): The density of the fluid acting on the medium, expressed in
            kilograms per cubic meter (kg/m^3).
    """

    def __init__(
        self,
        lift_coefficients: NDArray,
        drag_coefficients: NDArray,
        areas: NDArray,
        fluid_density: Scalar,
    ):
        self.__lift_coefficients = lift_coefficients
        self.__drag_coefficients = drag_coefficients
        self.__areas = areas
        self.__fluid_density = fluid_density

    def compute(self, apparent_velocity: NDArray, orientation: Scalar) -> Tuple[NDArray, NDArray]:
        """Computes the lift and drag forces experienced by a medium immersed in a fluid.

        Args:
            apparent_velocity (NDArray): The apparent (relative) velocity between the fluid and the
                medium, calculated as the difference between the fluid velocity and the medium
                velocity (fluid_velocity - medium_velocity), expressed in meters per second (m/s).
            orientation (Scalar): The orientation angle of the medium, expressed in degrees,
                indicating its alignment in a particular reference frame.

        Returns:
            Tuple[NDArray, NDArray]: A tuple containing the lift force and drag force experienced
                by the medium, both expressed in newtons (N).
        """

        # TODO: Implement this method.

        raise NotImplementedError()

    # TODO: Create a simple linear interpolation function that uses `np.interp` to compute the
    # coefficients and area given an angle of attack.

    @property
    def lift_coefficients(self) -> NDArray:
        return self.__lift_coefficients

    @property
    def drag_coefficients(self) -> NDArray:
        return self.__drag_coefficients

    @property
    def areas(self) -> NDArray:
        return self.__areas

    @property
    def fluid_density(self) -> Scalar:
        return self.__fluid_density
