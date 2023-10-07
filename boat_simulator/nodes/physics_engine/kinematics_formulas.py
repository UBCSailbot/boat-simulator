"""Formulas for estimating the next position, velocity, and acceleration."""

from numpy.typing import ArrayLike

from boat_simulator.common.types import Scalar


class KinematicsFormulas:
    """Contains formulas for calculating the boat's next position, velocity, and acceleration based
    on previous kinematic data."""

    @staticmethod
    def next_position(
        pos: ArrayLike, vel: ArrayLike, acc: ArrayLike, timestep: Scalar
    ) -> ArrayLike:
        """Calculates the boat's next position based on previous data and time step.

        Args:
            pos (ArrayLike): The last recorded boat position prior to the current time step.
            vel (ArrayLike): The last recorded boat velocity prior to the current time step.
            acc (ArrayLike): The last recorded boat acceleration prior to the current time step.
            timestep (Scalar): The time interval on which the calculation is based.

        Returns:
            ArrayLike: The calculated next position of the boat.
        """
        return pos + (vel * timestep) + (acc * (timestep**2 / 2))

    @staticmethod
    def next_velocity(vel: ArrayLike, acc: ArrayLike, timestep: Scalar) -> ArrayLike:
        """Calculates the boat's next velocity based on previous velocity, acceleration, and time
        step.

        Args:
            vel (ArrayLike): The last recorded boat velocity prior to the current time step.
            acc (ArrayLike): The last recorded boat acceleration prior to the current time step.
            timestep (Scalar): The time interval on which the calculation is based.

        Returns:
            ArrayLike: The calculated next velocity of the boat.
        """
        return vel + (acc * timestep)

    @staticmethod
    def next_lin_acceleration(mass: float, net_force: ArrayLike) -> ArrayLike:
        """Calculates the boat's next linear acceleration based on its mass and net force.

        Args:
            mass (float): The mass of the boat.
            net_force (ArrayLike): The net force acting on the boat.

        Returns:
            ArrayLike: The calculated next linear acceleration of the boat.
        """
        return net_force / mass

    @staticmethod
    def next_ang_acceleration(net_torque: ArrayLike, inertia_inverse: ArrayLike) -> ArrayLike:
        """Calculates the boat's next angular acceleration based on net torque and inverse of
        inertia.

        Args:
            net_torque (ArrayLike): The net torque acting on the boat.
            inertia_inverse (ArrayLike): The inverse of the boat's inertia.

        Returns:
            ArrayLike: The calculated next angular acceleration of the boat.
        """
        return inertia_inverse @ net_torque
