"""Generators for wind and current kinematics used in the physics engine."""

from abc import ABC, abstractmethod

from numpy.typing import ArrayLike


class FluidGenerator(ABC):
    @abstractmethod
    def next(self) -> ArrayLike:
        pass


class WindGenerator(FluidGenerator):
    def __init__(self):
        super().__init__()

    def next(self) -> ArrayLike:
        pass
