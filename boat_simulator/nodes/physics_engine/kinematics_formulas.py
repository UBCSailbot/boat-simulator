"""Spec goes here."""

from numpy.typing import ArrayLike

from boat_simulator.common.types import Scalar


class KinematicsFormulas:
    @staticmethod
    def next_position(
        pos: ArrayLike, vel: ArrayLike, acc: ArrayLike, timestep: Scalar
    ) -> ArrayLike:
        return pos + (vel * timestep) + (acc * (timestep**2 / 2))

    @staticmethod
    def next_velocity(vel: ArrayLike, acc: ArrayLike, timestep: Scalar) -> ArrayLike:
        return vel + (acc * timestep)

    @staticmethod
    def next_lin_acceleration(mass: float, net_force: ArrayLike) -> ArrayLike:
        return net_force / mass

    @staticmethod
    def next_ang_acceleration(net_torque: ArrayLike, inertia_inverse: ArrayLike) -> ArrayLike:
        return inertia_inverse @ net_torque
