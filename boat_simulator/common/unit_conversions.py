from boat_simulator.common.types import Scalar
from enum import Enum
from typing import Dict


class ConversionFactor:

    def __init__(self, factor: Scalar):
        self.__factor = factor

    def forward_convert(self, value: Scalar) -> Scalar:
        # TODO Implement this function
        raise NotImplementedError()

    def backward_convert(self, value: Scalar) -> Scalar:
        # TODO Implement this function
        raise NotImplementedError()

    @property
    def factor(self) -> Scalar:
        return self.__factor


class ConversionFactors(Enum):
    sec2min = ConversionFactor(factor=1/60)
    min2sec = ConversionFactor(factor=60)


class UnitConverter:

    def __init__(self, **kwargs):
        # TODO Implement this function to accept keys as class attributes and conversion enums
        pass

    def convert(self, **kwargs) -> Dict[str, Scalar]:
        # TODO Implement this function
        raise NotImplementedError()
