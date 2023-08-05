from boat_simulator.common import utils
from boat_simulator.boat_simulator.nodes.physics_engine.computation import BoatComputation
from boat_simulator.common.types import Scalar
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
    __inertia_inverse = np.identity(n=3, dtype=np.float32)
    __boat_mass = 1.0
    __timestep = 1.0
    __boat_computation = BoatComputation(__timestep)

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: ArrayLike):
        self.__timestep = timestep
        self.__boat_mass = mass
        self.__inertia = np.array(inertia, dtype=np.float32)
        self.__inertia_inverse = np.linalg.inv(self.__inertia)
        self.__boat_computation = BoatComputation(self.__timestep)

    def step(self, wind_vel: ArrayLike) -> None:
        net_force, net_torque = self.__boat_computation.compute_net_force_and_torque(wind_vel)

        next_relative_acceleration = self.__boat_computation.compute_next_relative_acceleration(
            self.boat_mass, net_force
        )
        next_ang_acceleration = self.__boat_computation.compute_next_ang_acceleration(self.inertia_inverse, net_torque)

        next_relative_velocity = self.__boat_computation.compute_next_relative_velocity(
            self.relative_velocity, next_relative_acceleration
        )
        next_ang_velocity = self.__boat_computation.compute_next_ang_velocity(
            self.angular_velocity, next_ang_acceleration
        )

        next_global_position = self.__boat_computation.compute_next_position(
            self.global_position, self.global_velocity, self.global_acceleration
        )

        self.__relative_velocity = next_relative_velocity
        self.__relative_acceleration = next_relative_acceleration
        self.__angular_velocity = next_ang_velocity
        self.__angular_acceleration = next_ang_acceleration
        self.__global_position = next_global_position

    @property
    def global_position(self) -> ArrayLike:
        return self.__global_position

    @property
    def global_velocity(self) -> ArrayLike:
        yaw_radians = utils.degrees_to_rad(
            self.__angular_position[constants.ORIENTATION_INDICES.YAW.value]
        )
        return self.relative_velocity * np.array([np.sin(yaw_radians), np.cos(yaw_radians), 0])

    @property
    def global_acceleration(self) -> ArrayLike:
        yaw_radians = utils.degrees_to_rad(
            self.__angular_position[constants.ORIENTATION_INDICES.YAW.value]
        )
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
