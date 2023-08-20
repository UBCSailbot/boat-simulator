"""This module contains the kinematics computations for the boat."""

from typing import Tuple
from numpy.typing import ArrayLike
from boat_simulator.common import utils
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
import numpy as np
import boat_simulator.common.constants as constants


class BoatKinematics:
    # Private class member default
    __timestep = 1.0
    __relative_data = KinematicsData()
    __global_data = KinematicsData()

    def __init__(self, timestep: Scalar) -> None:
        self.__timestep = timestep

    # Updates the kinematics data and returns the kinematics data objects
    def step(self, net_force: ArrayLike, net_torque: ArrayLike
             ) -> Tuple[KinematicsData, KinematicsData]:

        next_relative_acceleration = self.__compute_next_relative_acceleration(
            self.boat_mass, net_force
        )
        next_relative_velocity = self.__compute_next_relative_velocity(
            self.relative_data.cartesian_velocity, next_relative_acceleration
        )
        next_relative_position = self.__compute_next_position(
            self.relative_data.cartesian_position,
            next_relative_velocity,
            next_relative_acceleration
        )

        next_ang_acceleration = self.__compute_next_ang_acceleration(
            self.inertia_inverse, net_torque
        )
        next_ang_velocity = self.__compute_next_ang_velocity(
            self.relative_data.angular_velocity, next_ang_acceleration
        )
        next_ang_position = self.__compute_next_position(
            self.relative_data.angular_position, next_ang_velocity, next_ang_acceleration
        )

        yaw_radians = utils.degrees_to_rad(
            next_ang_position[constants.ORIENTATION_INDICES.YAW.value]
        )

        next_global_acceleration = next_relative_acceleration * \
            np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])
        next_global_velocity = next_relative_velocity * \
            np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])
        next_global_position = self.__kinematics_computation.compute_next_position(
            self.global_data.cartesian_position, next_global_velocity, next_global_acceleration
        )

        self.__relative_data.cartesian_acceleration = next_relative_acceleration
        self.__relative_data.cartesian_velocity = next_relative_velocity
        self.__relative_data.cartesian_position = next_relative_position
        self.__relative_data.angular_acceleration = next_ang_acceleration
        self.__relative_data.angular_velocity = next_ang_velocity
        self.__relative_data.angular_position = next_ang_position

        self.__global_data.cartesian_acceleration = next_global_acceleration
        self.__global_data.cartesian_velocity = next_global_velocity
        self.__global_data.cartesian_position = next_global_position
        self.__global_data.angular_acceleration = next_ang_acceleration
        self.__global_data.angular_velocity = next_ang_velocity
        self.__global_data.angular_position = next_ang_position

        return (self.relative_data, self.global_data)

    def __compute_next_position(self, pos: ArrayLike, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        return pos + (vel / self.__timestep) + (acc / (2 * self.__timestep**2))

    def __compute_next_relative_velocity(self, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        return vel + (acc / self.__timestep)

    def __compute_next_relative_acceleration(
        self, mass: float, net_force: ArrayLike
    ) -> ArrayLike:
        return net_force / mass

    def __compute_next_ang_velocity(self, ang_vel: ArrayLike, ang_acc: ArrayLike) -> ArrayLike:
        return ang_vel + (ang_acc / self.__timestep)

    def __compute_next_ang_acceleration(
        self, inertia_inverse: ArrayLike, net_torque: ArrayLike
    ) -> ArrayLike:
        return net_torque * inertia_inverse

    @property
    def relative_data(self):
        return self.__relative_data

    @property
    def global_data(self):
        return self.__global_data
