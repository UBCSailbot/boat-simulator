"""Low level control logic for actuating the rudder and the sail."""

from abc import ABC, abstractmethod
from math import atan2, cos, pi, sin
from typing import List

import numpy as np

from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180


class ActuatorController(ABC):
    """Abstract class for rudder and sail actuation mechanisms.

    Attributes:
        `current_heading` (Scalar): Current boat heading direction in degrees.
        `desired_heading` (Scalar): Target boating heading direction in degrees.
        `current_control_ang` (Scalar): Current control mechanism angle in degrees.
        `time_step` (Scalar): Time per iteration given in degrees/second.
        `kp` (Scalar): Proportional constant when calculating error.
        `cp` (Scalar): Timeseries of error values computed over time.
        `prev_control` (list): Timeseries of control angles computed over time.
        `time_series` (list): Timeseries of running time values corresponding to control angles

    """

    def __init__(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        max_angle_range: tuple,
    ):
        """Initializes the class attributes. Note that this class cannot be directly instantiated.

        Args:
            `current_heading` (Scalar): Current boat heading direction in degrees.
            `desired_heading` (Scalar): Target boating heading direction in degrees.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in degrees/second.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Timeseries of error values computed over time.
            `prev_control` (list): Timeseries of control angles computed over time.
            `time_series` (list): Timeseries of running time values related to control angles.
            `max_angle_range` (tuple): Tuple of minimum to maximum control angle range
        """
        self.current_heading = current_heading
        self.desired_heading = desired_heading
        self.current_control_ang = current_control_ang
        self.time_step = time_step
        self.kp = kp
        self.cp = cp
        self.max_angle_range = max_angle_range
        self.prev_control: List[Scalar] = list()
        self.time_series: List[Scalar] = list()

        self.time_series.append(0)  # append initial starting time 0
        self.prev_control.append(self.current_control_ang)  # append starting control angle

    def append_heading(self, past_heading: Scalar) -> None:
        """Appends the previous heading to heading list and updates the current
        time taken to iterate to the current rudder position

        Args:
            `past_heading` (Scalar): The previous heading to be added

        """
        self.prev_control.append(past_heading)
        self.time_series.append(self.time_series[-1] + self.time_step)

    def reset_desired_heading(self, target_angle: Scalar) -> None:
        """Resets a new desired heading angle and clears all previous
        headings from current_control_ang list and running time for time_series

        Args:
            `target_angle` (Scalar): New desired heading

        """
        self.desired_heading = target_angle
        self.prev_control.clear()
        self.time_series.clear()

    def get_heading(self) -> tuple:
        """Returns the most recent heading and current running time"""
        return (self.prev_control[-1], self.time_series[-1])

    @abstractmethod
    def update_state(self, speed: Scalar) -> None:
        """Updates the controller position based on target heading

        Args:
            `speed` (Scalar): Iterative speed in degrees / second

        """

    @abstractmethod
    def compute_feedback_angle(self) -> Scalar:
        """Computes the corresponding feedback based on error and desired heading"""
        pass


class RudderController(ActuatorController):
    """General Class for the Actuator Controller.

    RudderController Extends: ActuatorController
    """

    def __innit__(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        max_angle_range: tuple,
    ):
        """Initializes the class attributes.

        Args:
            `current_heading` (Scalar): Current boat heading direction in degrees.
            `desired_heading` (Scalar): Target boating heading direction in degrees.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in degrees/second.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Timeseries of error values computed over time.
            `max_angle_range` (tuple): Tuple of minimum to maximum control angle range

        """
        super().__init__(current_heading, desired_heading, current_control_ang, time_step)

        self.kp = kp
        self.cp = cp
        self.max_angle_range = max_angle_range  # passed in degrees
        self.running_error = 0

    def compute_error(
        self, desired: Scalar, current: Scalar
    ) -> Scalar:  # angle passed in as radians
        """Computes the error between desired and current heading
        implementation taken from: https://stackoverflow.com/a/2007279

        Args:
            `desired` (Scalar): New desired heading
            `current` (Scalar): New current heading

        Returns:
            Scalar: The error between the given headings in radians.
        """
        current_bound = bound_to_180(current)
        desired_bound = bound_to_180(desired)
        desired_rad = np.deg2rad(desired)
        current_rad = np.deg2rad(current)
        error = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))

        if current_bound == desired_bound:
            error = 0

        return error

    def compute_feedback_angle(self) -> Scalar:
        """Computes the feedback angle from the given error (in radians)
        Calculations done using Raye's implementation
            https://github.com/UBCSailbot/raye-boat-controller/blob/master/python/tack_controller.py

        Returns:
            Scalar: Difference between current and target rudder angle in degrees.
        """
        error = self.compute_error(self.current_heading, self.desired_heading)

        if abs(error) > pi:
            raise ValueError("heading_error must be between -pi and pi")

        rudder_change = (self.kp * error) / (1 + (self.cp * error))
        self.running_error = np.rad2deg(rudder_change)

        return np.rad2deg(rudder_change)

    def compute_setpoint(self) -> Scalar:
        """Calculates the rudder setpoint angle. Only called at the very beginning once
        to determine the setpoint angle

        Returns:
            The rudder setpoint angle in degrees
        """
        return self.prev_control[0] + self.compute_feedback_angle()

    def update_state(self, speed=2) -> None:  # default speed set to 2 degrees / sec
        """Updates the rudder angle iteratively towards the target rudder setpoint

        Args:
            `speed` (Scalar): Speed of rudder angle change in degrees per second
        """
        change = speed * self.time_step
        if self.running_error > change:
            next_control = self.current_control_ang + change
            if next_control > self.max_angle_range[0]:
                next_control = self.max_angle_range[0]
            elif next_control < self.max_angle_range[1]:
                next_control = self.max_angle_range[1]
            else:
                next_control = next_control
            self.running_error -= change
            self.current_control_ang = next_control

        else:
            next_control = self.current_control_ang + self.running_error
            if next_control > self.max_angle_range[0]:
                next_control = self.max_angle_range[0]
            elif next_control < self.max_angle_range[1]:
                next_control = self.max_angle_range[1]
            else:
                next_control = next_control
            self.running_error = 0
            self.current_control_ang = next_control


class SailController(ActuatorController):
    """General Class for the Actuator Controller.

    SailController Extends: ActuatorController
    """

    def __innit__(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp=0.7,
        cp=0.34,
        max_angle_range=(7, -7),
    ):
        """Initializes the class attributes.

        Args:
            `current_heading` (Scalar): Current boat heading direction in degrees.
            `desired_heading` (Scalar): Target boating heading direction in degrees.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in degrees/second.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Timeseries of error values computed over time.
            `max_angle_range` (tuple): Tuple of minimum to maximum control angle range

        """
        super().__init__(current_heading, desired_heading, current_control_ang, time_step)

        self.kp = kp
        self.cp = cp
        self.setpoint = 0
        self.max_angle_range = max_angle_range  # convert to radians
        self.running_error = 0

    def compute_feedback_angle(self, target_angle):
        """Computes the feedback angle between setpoint and current sail angles

        Args:
            `target_angle` (Scalar): Target sail angle in degrees
        """
        return target_angle - self.current_control_ang

    def update_state(self, speed=1):  # default speed set to 1 degree / sec
        """Updates the sail angle iteratively towards the target sail setpoint

        Args:
            `speed` (Scalar): Speed of sail angle change in degrees per second
        """
        change = speed * self.time_step
        if self.running_error > change:
            next_control = self.current_control_ang + change
            if next_control > self.max_angle_range[0]:
                next_control = self.max_angle_range[0]
            elif next_control < self.max_angle_range[1]:
                next_control = self.max_angle_range[1]
            else:
                next_control = next_control
            self.running_error -= change
            self.current_control_ang = next_control

        else:
            next_control = self.current_control_ang + self.running_error
            if next_control > self.max_angle_range[0]:
                next_control = self.max_angle_range[0]
            elif next_control < self.max_angle_range[1]:
                next_control = self.max_angle_range[1]
            else:
                next_control = next_control
            self.running_error = 0
            self.current_control_ang = next_control
