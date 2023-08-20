"""This module contains the kinematics data for the boat."""

from dataclasses import dataclass, field
from numpy.typing import ArrayLike
import numpy as np


@dataclass
class KinematicsData:
    cartesian_position: ArrayLike = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    cartesian_velocity: ArrayLike = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    cartesian_acceleration: ArrayLike = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )
    angular_position: ArrayLike = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    angular_velocity: ArrayLike = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    angular_acceleration: ArrayLike = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
