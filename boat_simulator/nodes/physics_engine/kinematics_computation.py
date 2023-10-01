"""This module contains the kinematics computations for the boat."""

from typing import Tuple

import numpy as np
from numpy.typing import ArrayLike

import boat_simulator.common.constants as constants
import boat_simulator.common.utils as utils
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.kinematics_formulas import KinematicsFormulas


class BoatKinematics:
    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike) -> None:
        self.__timestep = timestep
        self.__boat_mass = mass
        self.__inertia = np.array(inertia, dtype=np.float32)
        self.__inertia_inverse = np.linalg.inv(self.__inertia)
        self.__relative_data = KinematicsData()
        self.__global_data = KinematicsData()

    # Updates the kinematics data and returns the kinematics data objects
    def step(
        self, rel_net_force: ArrayLike, net_torque: ArrayLike
    ) -> Tuple[KinematicsData, KinematicsData]:
        yaw_radians = self.__update_ang_data(np.array(net_torque, dtype=np.float32))

        self.__update_linear_relative_data(np.array(rel_net_force, dtype=np.float32))

        # z-directional acceleration and velocity are neglected
        glo_net_force = rel_net_force * np.array([np.cos(yaw_radians), np.sin(yaw_radians), 0])
        self.__update_linear_global_data(glo_net_force)

        return (self.relative_data, self.global_data)

    def __update_ang_data(self, net_torque: ArrayLike) -> Scalar:
        next_ang_acceleration = utils.bound_to_180(
            KinematicsFormulas.next_ang_acceleration(net_torque, self.inertia_inverse)
        )
        next_ang_velocity = utils.bound_to_180(
            KinematicsFormulas.next_velocity(
                self.global_data.angular_velocity,
                self.global_data.angular_acceleration,
                self.timestep,
            )
        )
        next_ang_position = utils.bound_to_180(
            KinematicsFormulas.next_position(
                self.global_data.angular_position,
                self.global_data.angular_velocity,
                self.global_data.angular_acceleration,
                self.timestep,
            )
        )

        self.__relative_data.angular_acceleration = next_ang_acceleration
        self.__relative_data.angular_velocity = next_ang_velocity
        self.__relative_data.angular_position[:] = 0  # relative angular position is unused

        self.__global_data.angular_acceleration = next_ang_acceleration
        self.__global_data.angular_velocity = next_ang_velocity
        self.__global_data.angular_position = next_ang_position

        yaw_radians = next_ang_position[constants.ORIENTATION_INDICES.YAW.value]

        return yaw_radians

    def __update_linear_relative_data(self, net_force: ArrayLike) -> None:
        next_relative_acceleration = KinematicsFormulas.next_lin_acceleration(
            self.boat_mass, net_force
        )
        next_relative_velocity = KinematicsFormulas.next_velocity(
            self.relative_data.linear_velocity,
            self.relative_data.linear_acceleration,
            self.timestep,
        )

        self.__relative_data.linear_acceleration = next_relative_acceleration
        self.__relative_data.linear_velocity = next_relative_velocity
        self.__relative_data.linear_position[:] = 0  # linear position is unused

    def __update_linear_global_data(self, net_force: ArrayLike) -> None:
        next_global_acceleration = KinematicsFormulas.next_lin_acceleration(
            self.boat_mass, net_force
        )
        next_global_velocity = KinematicsFormulas.next_velocity(
            self.global_data.linear_velocity, self.global_data.linear_acceleration, self.timestep
        )
        next_global_position = KinematicsFormulas.next_position(
            self.global_data.linear_position,
            self.global_data.linear_velocity,
            self.global_data.linear_acceleration,
            self.timestep,
        )

        self.__global_data.linear_acceleration = next_global_acceleration
        self.__global_data.linear_velocity = next_global_velocity
        self.__global_data.linear_position = next_global_position

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
