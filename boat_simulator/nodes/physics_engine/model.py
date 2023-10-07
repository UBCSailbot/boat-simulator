"""This module represents the state of the boat at a given step in time."""

from typing import Tuple

import numpy as np
from numpy.typing import ArrayLike

from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData


class BoatState:
    """Represents the state of the boat at a specific point in time, including kinematic data
    in both relative and global reference frames.

    Attributes:
        `kinematics_computation` (BoatKinematics): The kinematic data for the boat in both
        the relative and global reference frames, used for computing future kinematic data.
    """

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (Scalar): The time interval for calculations.
            mass (Scalar): The mass of the boat.
            inertia (ArrayLike): The inertia matrix of the boat.
        """
        self.__kinematics_computation = BoatKinematics(timestep, mass, inertia)

    def compute_net_force_and_torque(self, wind_vel: ArrayLike) -> Tuple[ArrayLike, ArrayLike]:
        """Calculates the net force and net torque acting on the boat due to the wind.

        Args:
            wind_vel (ArrayLike): The velocity of the wind.

        Returns:
            Tuple[ArrayLike, ArrayLike]: A tuple containing the net force in the relative reference
            frame (position 0) and the net torque (position 1).
        """
        raise NotImplementedError()

    def step(
        self, rel_net_force: ArrayLike, net_torque: ArrayLike
    ) -> Tuple[KinematicsData, KinematicsData]:
        """Updates the boat's kinematic data based on applied forces and torques, and returns
        the updated kinematic data in both relative and global reference frames.

        Args:
            rel_net_force (ArrayLike): The net force acting on the boat in the relative reference
            frame.
            net_torque (ArrayLike): The net torque acting on the boat.

        Returns:
            Tuple[KinematicsData, KinematicsData]: A tuple containing updated kinematic data, with
            the first element representing data in the relative reference frame and the second
            element representing data in the global reference frame.
        """
        return self.__kinematics_computation.step(rel_net_force, net_torque)

    @property
    def global_position(self) -> ArrayLike:
        return self.__kinematics_computation.global_data.linear_position

    @property
    def global_velocity(self) -> ArrayLike:
        return self.__kinematics_computation.global_data.linear_velocity

    @property
    def global_acceleration(self) -> ArrayLike:
        self.__kinematics_computation.global_data.linear_acceleration

    @property
    def relative_velocity(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.linear_velocity

    @property
    def relative_acceleration(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.linear_acceleration

    @property
    def angular_position(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.angular_position

    @property
    def angular_velocity(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.angular_velocity

    @property
    def angular_acceleration(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.angular_position

    @property
    def inertia(self) -> ArrayLike:
        return self.__kinematics_computation.inertia

    @property
    def inertia_inverse(self) -> ArrayLike:
        return self.__kinematics_computation.inertia_inverse

    @property
    def boat_mass(self) -> Scalar:
        return self.__kinematics_computation.boat_mass

    @property
    def timestep(self) -> Scalar:
        return self.__kinematics_computation.timestep

    @property
    def speed(self) -> Scalar:
        return np.linalg.norm(x=self.relative_velocity, ord=2)

    @property
    def true_bearing(self) -> Scalar:
        # TODO: Implement this function
        return 0
