"""This module represents the state of the boat at a given step in time."""

from typing import Tuple
from numpy.typing import ArrayLike
from boat_simulator.boat_simulator.common.types import Scalar
from boat_simulator.boat_simulator.nodes.physics_engine.kinematics_computation \
    import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
import numpy as np


class BoatState:
    # Private class member defaults
    __inertia = np.identity(n=3, dtype=np.float32)
    __inertia_inverse = np.identity(n=3, dtype=np.float32)
    __boat_mass = 1.0
    __timestep = 1.0
    __kinematics_computation = BoatKinematics(__timestep)

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike):
        self.__timestep = timestep
        self.__boat_mass = mass
        self.__inertia = np.array(inertia, dtype=np.float32)
        self.__inertia_inverse = np.linalg.inv(self.__inertia)
        self.__kinematics_computation = BoatKinematics(self.__timestep)

    def compute_net_force_and_torque(self, wind_vel: ArrayLike) -> Tuple[ArrayLike, ArrayLike]:
        raise NotImplementedError()

    # Updates the kinematics data and returns the kinematics data objects
    def step(self, net_force: ArrayLike, net_torque: ArrayLike
             ) -> Tuple[KinematicsData, KinematicsData]:
        return self.__kinematics_computation.step(net_force, net_torque)

    @property
    def global_position(self) -> ArrayLike:
        return self.__kinematics_computation.global_data.cartesian_position

    @property
    def global_velocity(self) -> ArrayLike:
        return self.__kinematics_computation.global_data.cartesian_velocity

    @property
    def global_acceleration(self) -> ArrayLike:
        self.__kinematics_computation.global_data.cartesian_acceleration

    @property
    def relative_velocity(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.cartesian_velocity

    @property
    def relative_acceleration(self) -> ArrayLike:
        return self.__kinematics_computation.relative_data.cartesian_acceleration

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
        return self.__inertia

    @property
    def inertia_inverse(self) -> ArrayLike:
        return self.__inertia_inverse

    @property
    def boat_mass(self) -> Scalar:
        return self.__boat_mass

    @property
    def timestep(self) -> Scalar:
        return self.__timestep

    @property
    def speed(self) -> Scalar:
        return np.linalg.norm(x=self.relative_velocity, ord=2)

    @property
    def true_bearing(self) -> Scalar:
        # TODO: Implement this function
        return 0
