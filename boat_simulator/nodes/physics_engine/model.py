"""This module represents the state of the boat at a given step in time."""

from typing import Tuple

import numpy as np
from numpy.typing import ArrayLike

from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData


class BoatState:
    # Private class member defaults
    __kinematics_computation = BoatKinematics(
        timestep=1.0, mass=1.0, inertia=np.identity(n=3, dtype=np.float32)
    )

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike):
        self.__kinematics_computation = BoatKinematics(timestep, mass, inertia)

    def compute_net_force_and_torque(self, wind_vel: ArrayLike) -> Tuple[ArrayLike, ArrayLike]:
        raise NotImplementedError()

    # Updates the kinematics data and returns the kinematics data objects
    def step(
        self, net_force: ArrayLike, net_torque: ArrayLike
    ) -> Tuple[KinematicsData, KinematicsData]:
        return self.__kinematics_computation.step(net_force, net_torque)

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
