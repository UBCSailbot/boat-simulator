"""Low level control logic for actuating the rudder and the sail."""

from abc import ABC, abstractmethod
from math import atan2, cos, pi, sin
from typing import List  # Any

import numpy as np

from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180


class ActuatorController(ABC):
    """Abstract class for rudder and sail actuation mechanisms.

    Attributes:
        `current_heading` (Scalar): Current boat heading direction in radians
        `desired_heading` (Scalar): Target boating heading direction in radians
        `time_step` (Scalar): Amount of time for one iteration of heading change
        `time_period` (Scalar): Constant time period between error samples.
        `kp` (Scalar): Proportional constant when calculating error
        `cp` (Scalar): Timeseries of error values computed over time.
        `last_error` (float): The previous error calculated
        `integral_sum` (int): The running total of integral sum

    """

    def __init__(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
    ):
        self.current_heading = current_heading
        self.desired_heading = desired_heading
        self.time_step = time_step
        self.kp = kp
        self.cp = cp
        self.prev_heading: List[Scalar] = list()

    def append_heading(self, past_heading) -> None:
        """Appends the previous heading to heading list

        Args:
            `past_heading` (Scalar): The previous heading to be added

        """
        self.prev_heading.append(past_heading)

    def reset_desired_heading(self, target_angle) -> None:
        """Resets a new desired heading angle and clears all previous
        headings from heading list

        Args:
            `target_angle` (Scalar): New desired heading

        """
        self.desired_heading = target_angle
        self.prev_heading.clear()

    def update(self):

        if self.desired_heading is not None:
            self.adjust_angle()

    @abstractmethod
    def compute_error(self, desired, current):
        pass

    @abstractmethod
    def adjust_angle(self, intervals, error):
        pass


class RudderController(ActuatorController):
    def __innit__(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        max_angle_range: Scalar,
    ):
        super().__init__(current_heading, desired_heading, time_step, kp, cp)

        self.setpoint = 0
        self.max_angle_range = np.deg2rad(max_angle_range)  # convert to radians

    def compute_error(self, desired, current):
        error = atan2(sin(desired - current), cos(desired - current))
        current_bound = bound_to_180(current)
        desired_bound = bound_to_180(desired)

        if current_bound == desired_bound:
            error = 0

        return error

    def rudder_change(self):
        error = self.compute_error(self.current_heading, self.desired_heading)

        if abs(error) > pi:
            raise ValueError("heading_error must be between -pi and pi")

        rudder_change = (self.kp * error) / (1 + (self.cp * error))

        return rudder_change

    def adjust_angle(self, intervals, error):
        pass

    def update_current_heading(self):
        self.current_heading += self.adjust_angle
        if self.current_heading > self.max_angle_range:
            raise ValueError("Heading exceeds max heading range")
        return self.current_heading


class SailController(ActuatorController):
    pass
