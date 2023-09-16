"""Useful functions that could be used anywhere in the boat simulator package."""

import math
from numpy.typing import ArrayLike
from boat_simulator.common.types import Scalar


def rad_to_degrees(angle: Scalar) -> Scalar:
    """Converts an angle from radians to degrees.

    Args:
        `angle` (Scalar): Angle in radians.

    Returns:
        Scalar: Angle in degrees.
    """
    return angle * (180 / math.pi)


def degrees_to_rad(angle: Scalar) -> Scalar:
    """Converts an angle from degrees to radians.

    Args:
        `angle` (Scalar): Angle in degrees.

    Returns:
        Scalar: Angle in radians.
    """
    return angle * (math.pi / 180)


def bound_to_180(angles: ArrayLike, isRadians: bool = True) -> ArrayLike:
    """Converts an angle to be in the range [-π, π) radians or [-180, 180) degrees.

    Args:
        `angles` (ArrayLike): Angles to be bound.
        `isRadians` (bool, optional): True if the input is in radians, and false for degrees.
            Defaults to True.

    Returns:
        ArrayLike: Bounded angles. Output unit matches `isRadians`.
    """
    bound = math.pi if isRadians else 180
    for i in range(len(angles)):
        angles[i] = angles[i] - 2 * bound * ((angles[i] + bound) // (2 * bound))
    return angles


def bound_to_360(angle: Scalar, isDegrees: bool = True) -> Scalar:
    """Converts an angle to be in the range [0, 360) degrees.

    Args:
        `angle` (Scalar): Angle to be bound.
        `isDegrees` (bool, optional): True if the input is in degrees, and false for radians.
            Defaults to True.

    Returns:
        Scalar: Bounded angle. Output units matches `isDegrees`.
    """
    raise NotImplementedError()


def enu_heading_to_ned_heading(enu_heading: Scalar) -> Scalar:
    """Converts a heading from ENU to NED.

    Args:
        `enu_heading` (Scalar): Heading in ENU.

    Returns:
        Scalar: Heading in NED.
    """
    raise NotImplementedError()


def ned_heading_to_enu_heading(ned_heading: Scalar) -> Scalar:
    """Converts a heading from NED to ENU.

    Args:
        `ned_heading` (Scalar): Heading in NED.

    Returns:
        Scalar: Heading in ENU.
    """
    raise NotImplementedError()
