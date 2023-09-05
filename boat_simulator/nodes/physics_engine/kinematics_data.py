"""This module contains the kinematics data for the boat."""

from dataclasses import dataclass, field
from numpy.typing import ArrayLike
import numpy as np


@dataclass
class KinematicsData:
    linear_position: ArrayLike = field(default=np.zeros(3, dtype=np.float32))
    linear_velocity: ArrayLike = field(default=np.zeros(3, dtype=np.float32))
    linear_acceleration: ArrayLike = field(default=np.zeros(3, dtype=np.float32))
    angular_position: ArrayLike = field(default=np.zeros(3, dtype=np.float32))
    angular_velocity: ArrayLike = field(default=np.zeros(3, dtype=np.float32))
    angular_acceleration: ArrayLike = field(default=np.zeros(3, dtype=np.float32))
