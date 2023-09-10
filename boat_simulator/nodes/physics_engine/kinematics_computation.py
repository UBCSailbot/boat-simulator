"""This module contains the kinematics computations for the boat."""

from typing import Tuple
from numpy.typing import ArrayLike
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
import numpy as np
import boat_simulator.common.constants as constants


class BoatKinematics:
    # Private class member default
    __relative_data = KinematicsData()
    __global_data = KinematicsData()

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike) -> None:
        self.__timestep = timestep
        self.__boat_mass = mass
        self.__inertia = np.array(inertia, dtype=np.float32)
        self.__inertia_inverse = np.linalg.inv(self.__inertia)

    # Updates the kinematics data and returns the kinematics data objects
    def step(self, net_force: ArrayLike, net_torque: ArrayLike
             ) -> Tuple[KinematicsData, KinematicsData]:

        yaw_radians = self.__update_ang_data(net_torque)

        next_relative_acceleration, next_relative_velocity = \
            self.__update_linear_relative_data(net_force)

        self.__update_linear_global_data(
            next_relative_acceleration, next_relative_velocity, yaw_radians
            )

        return (self.relative_data, self.global_data)

    def __update_ang_data(self, net_torque: ArrayLike) -> Scalar:
        next_ang_acceleration = self.__compute_next_ang_acceleration(net_torque)
        next_ang_velocity = self.__compute_next_velocity(
            self.global_data.angular_velocity, self.global_data.angular_acceleration
            )
        next_ang_position = self.__compute_next_position(
            self.global_data.angular_position,
            self.global_data.angular_velocity,
            self.global_data.angular_acceleration
            )

        self.__relative_data.angular_acceleration = next_ang_acceleration
        self.__relative_data.angular_velocity = next_ang_velocity
        self.__relative_data.angular_position[:] = 0  # relative angular position is unused

        self.__global_data.angular_acceleration = next_ang_acceleration
        self.__global_data.angular_velocity = next_ang_velocity
        self.__global_data.angular_position = next_ang_position

        yaw_radians = next_ang_position[constants.ORIENTATION_INDICES.YAW.value]

        return yaw_radians

    def __update_linear_relative_data(self, net_force: ArrayLike) -> Tuple[ArrayLike, ArrayLike]:
        next_relative_acceleration = self.__compute_next_relative_acceleration(
            self.boat_mass, net_force
            )
        next_relative_velocity = self.__compute_next_velocity(
            self.relative_data.linear_velocity, self.relative_data.linear_acceleration
            )

        self.__relative_data.linear_acceleration = next_relative_acceleration
        self.__relative_data.linear_velocity = next_relative_velocity
        self.__relative_data.linear_position[:] = 0  # linear position is unused

        return (next_relative_acceleration, next_relative_velocity)

    def __update_linear_global_data(self, next_relative_acceleration: ArrayLike,
                                    next_relative_velocity: ArrayLike,
                                    yaw_radians: Scalar) -> None:

        # z-directional acceleration and velocity are neglected
        next_global_acceleration = next_relative_acceleration * \
            np.array([np.cos(yaw_radians), np.sin(yaw_radians), 0])
        next_global_velocity = next_relative_velocity * \
            np.array([np.cos(yaw_radians), np.sin(yaw_radians), 0])
        next_global_position = self.__compute_next_position(
            self.global_data.linear_position,
            self.global_data.linear_velocity,
            self.global_data.linear_acceleration
            )

        self.__global_data.linear_acceleration = next_global_acceleration
        self.__global_data.linear_velocity = next_global_velocity
        self.__global_data.linear_position = next_global_position

    def __compute_next_position(self, pos: ArrayLike, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        return pos + (vel * self.timestep) + (acc * (self.timestep**2 / 2))

    def __compute_next_velocity(self, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        return vel + (acc * self.timestep)

    def __compute_next_relative_acceleration(self, mass: float, net_force: ArrayLike) -> ArrayLike:
        return net_force / mass

    def __compute_next_ang_acceleration(self, net_torque: ArrayLike) -> ArrayLike:
        return self.inertia_inverse @ net_torque

    @property
    def relative_data(self) -> KinematicsData:
        return self.__relative_data

    @property
    def global_data(self) -> KinematicsData:
        return self.__global_data

    @property
    def timestep(self) -> Scalar:
        return self.__timestep

    @property
    def inertia(self) -> ArrayLike:
        return self.__inertia

    @property
    def inertia_inverse(self) -> ArrayLike:
        return self.__inertia_inverse

    @property
    def boat_mass(self) -> Scalar:
        return self.__boat_mass
