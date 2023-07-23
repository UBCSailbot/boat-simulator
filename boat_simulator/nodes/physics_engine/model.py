from boat_simulator.common.types import Scalar
from typing import Tuple
from numpy.typing import ArrayLike
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

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike):
        pass

    def step(self, wind_vel: ArrayLike):
        pass

    def __compute_net_force_and_torque(self, wind_vel: ArrayLike) -> Tuple[ArrayLike, ArrayLike]:
        raise NotImplementedError()

    def __compute_next_position(self, pos: ArrayLike, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        raise NotImplementedError()

    def __compute_next_relative_velocity(self, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        raise NotImplementedError()

    def __compute_next_relative_acceleration(self, mass: Scalar,
                                             net_force: ArrayLike) -> ArrayLike:
        raise NotImplementedError()

    def __compute_next_ang_velocity(self, ang_vel: ArrayLike, ang_acc: ArrayLike) -> ArrayLike:
        raise NotImplementedError()

    def __compute_next_ang_acceleration(self, inertia: ArrayLike,
                                        net_torque: ArrayLike) -> ArrayLike:
        raise NotImplementedError()

    @property
    def global_position(self) -> ArrayLike:
        return self.__global_position

    @property
    def global_velocity(self) -> ArrayLike:
        yaw_radians = self.__angular_position(constants.ORIENTATION_INDICES.YAW.value)
        return self.relative_velocity * np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])

    @property
    def global_acceleration(self) -> ArrayLike:
        yaw_radians = self.__angular_position(constants.ORIENTATION_INDICES.YAW.value)
        return self.relative_acceleration * np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])

    @property
    def relative_velocity(self) -> ArrayLike:
        return self.__relative_velocity

    @property
    def relative_acceleration(self) -> ArrayLike:
        return self.__relative_acceleration

    @property
    def angular_position(self) -> ArrayLike:
        return self.__angular_position

    @property
    def angular_velocity(self) -> ArrayLike:
        return self.__angular_velocity

    @property
    def angular_acceleration(self) -> ArrayLike:
        return self.__angular_acceleration

    @property
    def inertia(self) -> ArrayLike:
        return self.__inertia

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
