"""This module provides functionality for computing the lift and drag forces acting on a medium."""

from typing import Dict, Tuple

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.types import Scalar


class MediumForceComputation:
    """This class calculates the lift and drag forces experienced by a medium when subjected to
    fluid flow.

    Attributes:
        `lift_coefficients` (Dict[Scalar, Scalar]): A dictionary mapping angles of attack to lift
            coefficients. Keys represent different angles of attack in degrees, and values
            represent corresponding lift coefficients.
        `drag_coefficients` (Dict[Scalar, Scalar]): A dictionary mapping angles of attack to drag
            coefficients. Keys represent different angles of attack in degrees, and values
            represent corresponding drag coefficients.
        `area` (Scalar): The area of the medium exposed to fluid flow, expressed in square meters
            (m^2).
        `fluid_density` (Scalar): The density of the fluid acting on the medium, expressed in
            kilograms per cubic meter (kg/m^3).
    """

    def __init__(
        self,
        lift_coefficients: Dict[Scalar, Scalar],
        drag_coefficients: Dict[Scalar, Scalar],
        area: Scalar,
        fluid_density: Scalar,
    ):
        self.__lift_coefficients = lift_coefficients
        self.__drag_coefficients = drag_coefficients
        self.__area = area
        self.__fluid_density = fluid_density

    def compute(self, apparent_velocity: NDArray, attack_angle: Scalar) -> Tuple[NDArray, NDArray]:
        """Calculates the lift and drag forces experienced by a medium due to fluid flow.

        Args:
            apparent_velocity (NDArray): The apparent (relative) velocity between the fluid and the
                medium, calculated as the difference between the fluid velocity and the medium
                velocity (fluid_velocity - medium_velocity), expressed in meters per second (m/s).
            attack_angle (Scalar):The angle of attack formed between the apparent velocity and the
                reference line of the medium, given in degrees.

        Returns:
            Tuple[NDArray, NDArray]: A tuple containing the lift force and drag force experienced
                by the medium, both expressed in newtons (N).
        """

        # TODO: Implement this method
        return (np.array([]), np.array([]))

    @property
    def lift_coefficients(self) -> Dict[Scalar, Scalar]:
        return self.__lift_coefficients

    @property
    def drag_coefficients(self) -> Dict[Scalar, Scalar]:
        return self.__drag_coefficients

    @property
    def area(self) -> Scalar:
        return self.__area

    @property
    def fluid_density(self) -> Scalar:
        return self.__fluid_density
