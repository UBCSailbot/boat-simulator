"""This module contains the kinematics data for the boat."""

from dataclasses import Field, dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass
class KinematicsData:
    """Stores both linear and angular kinematic information pertaining to the boat.

    Attributes:
        `linear_position` (Field[NDArray]): Linear position of the boat, expressed in meters (m).
        `linear_velocity` (Field[NDArray]): Linear velocity of the boat, expressed in meters per
            second (m/s).
        `linear_acceleration` (Field[NDArray]): Linear acceleration of the boat, expressed in
            meters per second squared (m/s^2).
        `angular_position` (Field[NDArray]): Angular position of the boat, expressed in radians
            (rad).
        `angular_velocity` (Field[NDArray]): Angular velocity of the boat, expressed in radians
            per second (rad/s).
        `angular_acceleration` (Field[NDArray]): Angular acceleration of the boat, expressed in
            radians per second squared (rad/s^2).
    """

    # TODO: Ensure position is always set to 0 for relative reference frame
    linear_position: Field[NDArray] = field(default=np.zeros(3, dtype=np.float32))
    linear_velocity: Field[NDArray] = field(default=np.zeros(3, dtype=np.float32))
    linear_acceleration: Field[NDArray] = field(default=np.zeros(3, dtype=np.float32))
    angular_position: Field[NDArray] = field(default=np.zeros(3, dtype=np.float32))
    angular_velocity: Field[NDArray] = field(default=np.zeros(3, dtype=np.float32))
    angular_acceleration: Field[NDArray] = field(default=np.zeros(3, dtype=np.float32))
