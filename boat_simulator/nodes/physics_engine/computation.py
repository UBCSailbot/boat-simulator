from typing import Tuple
from numpy.typing import ArrayLike
from boat_simulator.boat_simulator.common.types import Scalar


class BoatComputation:
    # Private class member default
    __timestep = 1.0

    def __init__(self, timestep: Scalar) -> None:
        self.__timestep = timestep

    def compute_net_force_and_torque(self, wind_vel: ArrayLike) -> Tuple[ArrayLike, ArrayLike]:
        raise NotImplementedError()

    def compute_next_position(self, pos: ArrayLike, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        return pos + (vel / self.__timestep) + (acc / (2 * self.__timestep**2))

    def compute_next_relative_velocity(self, vel: ArrayLike, acc: ArrayLike) -> ArrayLike:
        return vel + (acc / self.__timestep)

    def compute_next_relative_acceleration(
        self, mass: float, net_force: ArrayLike
    ) -> ArrayLike:
        return net_force / mass

    def compute_next_ang_velocity(self, ang_vel: ArrayLike, ang_acc: ArrayLike) -> ArrayLike:
        return ang_vel + (ang_acc / self.__timestep)

    def compute_next_ang_acceleration(
        self, inertia_inverse: ArrayLike, net_torque: ArrayLike
    ) -> ArrayLike:
        return net_torque * inertia_inverse
