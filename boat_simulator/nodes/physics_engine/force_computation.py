"""Class to compute lift and drag forces acting on a medium."""

from typing import Tuple

from numpy.typing import NDArray

from boat_simulator.common.types import Scalar


class ForceComputation:
    def __init__(self, lift_coefficient: Scalar, area: Scalar, fluid_density: Scalar):
        self.__lift_coefficient = lift_coefficient
        self.__area = area
        self.__fluid_density = fluid_density

    def compute(self, apparent_velocity: NDArray) -> Tuple[NDArray, NDArray]:
        """Computes the lift and drag forces acting on a medium.

        Args:
            apparent_velocity (NDArray): Apparent (relative) velocity between the fluid and the
                medium, given in meters per second (m/s).

        Returns:
            Tuple[NDArray, NDArray]: A tuple where the first element represents lift force and the
                second element represents the drag force, both of which are expressed in
                newtons (N).
        """

    # TODO: Implement this method
    pass

    @property
    def lift_coefficient(self) -> Scalar:
        return self.__lift_coefficient

    @property
    def area(self) -> Scalar:
        return self.__area

    @property
    def fluid_density(self) -> Scalar:
        return self.__fluid_density
