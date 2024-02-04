"""Generator for fluid vectors used in the physics engine."""

from numpy.typing import NDArray

from boat_simulator.common.generators import VectorGenerator
from boat_simulator.common.types import Scalar


class FluidGenerator:
    def __init__(self, generator: VectorGenerator):
        self.__generator = generator
        self.__velocity = 0
        self.__speed = 0
        self.__direction = 0

    def next(self) -> NDArray:
        """Updates the fluid simulator by generating the next velocity vector.

        Returns:
            NDArray: An array representing the updated velocity vector for the fluid simulation.
        """

        # TODO: Implement this method
        pass

    @property
    def velocity(self) -> NDArray:
        return self.__velocity

    @property
    def speed(self) -> Scalar:
        return self.__speed

    @property
    def direction(self) -> Scalar:
        return self.__direction
