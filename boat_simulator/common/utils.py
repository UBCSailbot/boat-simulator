"""Useful functions that could be used anywhere in the boat simulator package."""

from boat_simulator.common.types import Scalar
import math


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


def bound_to_180(angle: Scalar, isDegrees: bool = True) -> Scalar:
    """Converts an angle to be in the range [-180, 180) degrees.

    Args:
        `angle` (Scalar): Angle to be bound.
        `isDegrees` (bool, optional): True if the input is in degrees, and false for radians.
            Defaults to True.

    Returns:
        Scalar: Bounded angle. Output unit matches `isDegrees`.
    """
    raise NotImplementedError()


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


def symmetric_clockwise_angle_to_true_bearing(angle: Scalar, isDegrees: bool = True) -> Scalar:
    """Converts an angle in a symmetric clockwise-positive system (angles range from [-180, 180))
    to a true bearing value (angles range from [0, 360), and is also a clockwise-positive system).

    Args:
        `angle` (Scalar): Angle to be converted.
        `isDegrees` (bool, optional): True if the input is in degrees, and false for radians.
            Defaults to True.

    Returns:
        Scalar: The true bearing equivalent of the input angle.
    """
    raise NotImplementedError()


def true_bearing_to_symmetric_clockwise_angle(bearing: Scalar, isDegrees: bool = True) -> Scalar:
    """Converts a true bearing value (angles range from [0, 360), and is also a clockwise-positive
    system) to a symmetric clockwise-positive system (angles range from [-180, 180)).

    Args:
        `bearing` (Scalar): The true bearing to be converted.
        `isDegrees` (bool, optional): True if the input is in degrees, and false for radians.
            Defaults to True.

    Returns:
        Scalar: The symmetric angle equivalent of the input bearing.
    """
    raise NotImplementedError()
