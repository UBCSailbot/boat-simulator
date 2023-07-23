from boat_simulator.common.types import Number
from typing import Tuple
import numpy as np
import boat_simulator.common.constants as constants


class BoatState:

    # Private class member defaults
    __global_position = np.zeros(shape=(3,), dtype=np.float32)
    __relative_velocity = np.zeros(shape=(3,), dtype=np.float32)
    __relative_acceleration = np.zeros(shape=(3,), dtype=np.float32)
    __angular_position = np.zeros(shape=(3,), dtype=np.float32)
    __angular_velocity = np.zeros(shape=(3,), dtype=np.float32)
    __angular_acceleration = np.zeros(shape=(3,), dtype=np.float32)
    __inertia = np.identity(n=3, dtype=np.float32)
    __boat_mass = 1.0
    __timestep = 1.0

    def __init__(self, timestep: Number, mass: Number, inertia: np.array):
        pass

    def step(self, wind_velocity: np.array):
        pass

    def __compute_net_force_and_torque(self, wind_velocity: np.array) -> Tuple[np.array, np.array]:
        raise NotImplementedError()

    def __compute_next_position(self, pos: np.array, vel: np.array, acc: np.array) -> np.array:
        raise NotImplementedError()

    def __compute_next_relative_velocity(self, vel: np.array, acc: np.array) -> np.array:
        raise NotImplementedError()

    def __compute_next_relative_acceleration(self, mass: Number, net_force: np.array) -> np.array:
        raise NotImplementedError()

    def __compute_next_ang_velocity(self, ang_vel: np.array, ang_acc: np.array) -> np.array:
        raise NotImplementedError()

    def __compute_next_ang_acceleration(self, inertia: np.array, net_torque: np.array) -> np.array:
        raise NotImplementedError()

    @property
    def global_position(self) -> np.array:
        return self.__global_position

    @property
    def global_velocity(self) -> np.array:
        yaw_radians = self.__angular_position(constants.ORIENTATION_INDICES.YAW.value)
        return self.relative_velocity * np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])

    @property
    def global_acceleration(self) -> np.array:
        yaw_radians = self.__angular_position(constants.ORIENTATION_INDICES.YAW.value)
        return self.relative_acceleration * np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])

    @property
    def relative_velocity(self) -> np.array:
        return self.__relative_velocity

    @property
    def relative_acceleration(self) -> np.array:
        return self.__relative_acceleration

    @property
    def angular_position(self) -> np.array:
        return self.__angular_position

    @property
    def angular_velocity(self) -> np.array:
        return self.__angular_velocity

    @property
    def angular_acceleration(self) -> np.array:
        return self.__angular_acceleration

    @property
    def inertia(self) -> np.array:
        return self.__inertia

    @property
    def boat_mass(self) -> Number:
        return self.__boat_mass

    @property
    def timestep(self) -> Number:
        return self.__timestep

    @property
    def speed(self) -> Number:
        return np.linalg.norm(x=self.relative_velocity, ord=2)

    @property
    def true_bearing(self) -> Number:
        raise NotImplementedError()
