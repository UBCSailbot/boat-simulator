"""Unit conversion logic, mostly contained in the `UnitConverter` class."""

from __future__ import annotations
from boat_simulator.common.types import EnumAttr, Scalar
from enum import Enum
from typing import Dict


class ConversionFactor:
    """Performs unit conversions from one unit to another. Both directions of unit conversion are
    supported by this class.

    Attributes:
        `factor` (Scalar): The conversion factor to go from unit A to B.
        `inverse_factor` (Scalar): The conversion factor to go from unit B to A.
    """

    def __init__(self, factor: Scalar):
        """Initializes an instance of `ConversionFactor`.

        Args:
            factor (Scalar): Conversion factor from unit A to B.
        """
        self.__factor = factor

    def forward_convert(self, value: Scalar) -> Scalar:
        """Convert from unit A to B.

        Args:
            value (Scalar): Value with unit A to be converted.

        Returns:
            Scalar: Converted value with unit B.
        """
        return value * self.factor

    def backward_convert(self, value: Scalar) -> Scalar:
        """Convert from unit B to A.

        Args:
            value (Scalar): Value with unit B to be converted.

        Returns:
            Scalar: Converted value with unit A.
        """
        return value * self.inverse_factor

    def inverse(self) -> ConversionFactor:
        """
        Get the inverse of this class containing the inverse conversion factor.

        Returns:
            ConversionFactor: The inverse of this class.
        """
        return ConversionFactor(factor=self.inverse_factor)

    def __mul__(self, other: ConversionFactor) -> ConversionFactor:
        """Multiplication operator between two `ConversionFactor` objects (A * B).

        Args:
            other (ConversionFactor): Other conversion factor being multiplied.

        Returns:
            ConversionFactor: Multiplied conversion factor.
        """
        mul_conversion_factor = self.factor * other.factor
        return ConversionFactor(factor=mul_conversion_factor)

    def __rmul__(self, other: ConversionFactor) -> ConversionFactor:
        """Multiplication operator between two `ConversionFactor` objects (B * A).

        Args:
            other (ConversionFactor): Other conversion factor being multiplied.

        Returns:
            ConversionFactor: Multiplied conversion factor.
        """
        return self.__mul__(other)

    @property
    def factor(self) -> Scalar:
        return self.__factor

    @property
    def inverse_factor(self) -> Scalar:
        return 1/self.factor


class ConversionFactors(Enum):
    """Predefined conversion factors commonly used in that boat simulator. This class is meant
    to be used in conjunction with the `UnitConverter` class to specify the unit conversions
    that will be performed.

    Attributes:
        <UNIT A>2<UNIT B> (EnumAttr): `ConversionFactor` classes to perform unit
            conversions going from unit A to B.

        Attributes in this class must follow the above naming convention.
    """
    sec2min = ConversionFactor(factor=1/60)
    sec2hr = sec2min * ConversionFactor(factor=1/60)
    min2sec = sec2min.inverse()


class UnitConverter:
    """Performs multiple unit conversions at once.

    Attributes:
        <ATTR NAME> (EnumAttr): Attribute names of this class depend on what is passed into
            the __init__ function of this class. All attributes are of type `EnumAttr`, which
            should come from the `ConversionFactors` class.
    """

    def __init__(self, **kwargs: EnumAttr):
        """Initializes an instance of `UnitConverter`.

        Args:
            kwargs (Dict[str, EnumAttr]): Dictionary keys are class attribute names, and dictionary
                values are class attribute values. Dictionary values are strictly class attributes
                belonging to `ConversionFactors`.
        """
        # TODO Implement this function to accept keys as class attributes and conversion enums
        pass

    def convert(self, **kwargs: Scalar) -> Dict[str, Scalar]:
        """Perform unit conversions for multiple specified values.

        Args:
            kwargs (Dict[str, Scalar]): Dictionary keys are strictly names of attributes belonging
                to this class. Dictionary values are the values to be converted, using the
                conversion factor corresponding to the class attribute.

        Returns:
            Dict[str, Scalar]: Converted values. Dictionary keys are class attribute names
                corresponding to the converted value. Dictionary values are the converted values.
        """
        # TODO Implement this function
        raise NotImplementedError()
