"""This module provides functionality for computing the lift and drag forces acting on a medium."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.utils import Scalar


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

    def calculate_attack_angle(self, apparent_velocity: NDArray, orientation: Scalar) -> Scalar:
        """Calculates the angle of attack formed between the orientation angle of the medium
            and the direction of the apparent velocity.

        Args:
            apparent_velocity (NDArray): The apparent (relative) velocity between the fluid and
                the medium, calculated as the difference between the fluid velocity and the
                medium velocity (fluid_velocity - medium_velocity), expressed in meters per
                second (m/s).
            orientation (Scalar): The orientation angle of the medium in degrees, where 0
                degrees corresponds to the positive x-axis, and angles increase
                counter-clockwise (CCW).

        Returns:
            Scalar: The angle of attack formed between the orientation angle of the medium and
                the direction of the apparent velocity, expressed in degrees.
        """

        return (
            np.rad2deg(
                np.arctan2(apparent_velocity[1], apparent_velocity[0]) - np.deg2rad(orientation)
            )
        ) % 360

    def compute(self, apparent_velocity: NDArray, orientation: Scalar) -> Tuple[NDArray, NDArray]:
        """Computes the lift and drag forces experienced by a medium immersed in a fluid.

        Args:
            apparent_velocity (NDArray): The apparent (relative) velocity between the fluid and the
                medium, calculated as the difference between the fluid velocity and the medium
                velocity (fluid_velocity - medium_velocity), expressed in meters per second (m/s).
            orientation (Scalar): The orientation angle of the medium in degrees, where 0 degrees
                corresponds to the positive x-axis, and angles increase counter-clockwise (CCW).

        Returns:
            Tuple[NDArray, NDArray]: A tuple containing the lift force and drag force experienced
                by the medium, both expressed in newtons (N).
                Bound to 360 degrees before using.
        """

        attack_angle = self.calculate_attack_angle(apparent_velocity, orientation)
        lift_coefficient, drag_coefficient, area = self.interpolate(attack_angle)
        lift_force_magnitude = (
            0.5
            * self.__fluid_density
            * lift_coefficient
            * area
            * (np.linalg.norm(apparent_velocity) ** 2)
        )

        drag_force_magnitude = (
            0.5
            * self.__fluid_density
            * drag_coefficient
            * area
            * (np.linalg.norm(apparent_velocity) ** 2)
        )

        # Rotate the lift and drag forces by 90 degrees to obtain the lift and drag forces
        # Rotate counter clockwise in 1st and 3rd quadrant, and clockwise in 2nd and 4th quadrant
        # 0 otherwise
        drag_force_direction = (apparent_velocity) / np.linalg.norm(apparent_velocity)
        if (drag_force_direction[0] > 0 and drag_force_direction[1] > 0) or (
            drag_force_direction[0] < 0 and drag_force_direction[1] < 0
        ):
            lift_force_direction = np.array([-drag_force_direction[1], drag_force_direction[0]])
        elif (drag_force_direction[0] > 0 and drag_force_direction[1] < 0) or (
            drag_force_direction[0] < 0 and drag_force_direction[1] > 0
        ):
            lift_force_direction = np.array([drag_force_direction[1], -drag_force_direction[0]])
        else:
            lift_force_direction = np.array([0, 0])

        lift_force = lift_force_magnitude * lift_force_direction
        drag_force = drag_force_magnitude * drag_force_direction
        return lift_force, drag_force

    def interpolate(self, attack_angle: Scalar) -> Tuple[Scalar, Scalar, Scalar]:
        """Performs linear interpolation to estimate the lift and drag coefficients, as well as the
        associated area upon which the fluid acts, based on the provided angle of attack.

            Args:
                attack_angle (Scalar): The angle of attack formed between the orientation angle of
                    the medium and the direction of the apparent velocity, expressed in degrees.

            Returns:
                Tuple[Scalar, Scalar, Scalar]: A tuple representing the computed parameters. The
                    first scalar denotes the lift coefficient, the second scalar represents the
                    drag coefficient, and the third scalar indicates the surface area upon which
                    the fluid acts. Both lift and drag coefficients are unitless, while the
                    area is expressed in square meters (m^2).
        """

        lift_coefficient = np.interp(
            attack_angle, self.__lift_coefficients[:, 0], self.__lift_coefficients[:, 1]
        )
        drag_coefficient = np.interp(
            attack_angle, self.__drag_coefficients[:, 0], self.__drag_coefficients[:, 1]
        )
        area = np.interp(attack_angle, self.__areas[:, 0], self.__areas[:, 1])
        return lift_coefficient, drag_coefficient, area

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
