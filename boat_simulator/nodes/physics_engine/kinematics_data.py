"""This module contains the kinematics data for the boat."""

from dataclasses import Field, dataclass, field
from numpy.typing import ArrayLike
import numpy as np


@dataclass
class KinematicsData:
    # TODO: Ensure position is always set to 0 for relative reference frame
    linear_position: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    linear_velocity: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    linear_acceleration: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    angular_position: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    angular_velocity: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
    angular_acceleration: Field[ArrayLike] = field(default=np.zeros(3, dtype=np.float32))
