"""Low level control logic for actuating the rudder and the sail."""

from abc import ABC, abstractmethod
from math import atan2, cos, sin
from typing import Any, List

from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180


class PID(ABC):

    """Abstract class for a PID controller.

    Attributes:
        `kp` (Scalar): The proportional component tuning constant.
        `ki` (Scalar): The integral component tuning constant.
        `kd` (Scalar): The derivative component tuning constant.
        `time_period` (Scalar): Constant time period between error samples.
        `buf_size` (int): The max number of error samples to store for integral component.
        `error_timeseries` (List[Scalar]): Timeseries of error values computed over time.
        `last_error` (float): The previous error calculated
        `integral_sum` (int): The running total of integral sum

    """

    def __init__(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        sum_threshold: Scalar,
    ):
        """Initializes the class attributes. Note that this class cannot be directly instantiated.

        Args:
            `kp` (Scalar): The proportional component tuning constant.
            `ki` (Scalar): The integral component tuning constant.
            `kd` (Scalar): The derivative component tuning constant.
            `time_period` (Scalar): Time period between error samples.
            `buf_size` (int): The max number of error samples to store for integral component.
            `last_error` (float): The error calculated in the previous iteration
            `integral_sum` (int): The running total of integral sum from integral response
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.buf_size = buf_size
        self.time_period = time_period
        self.error_timeseries: List[Scalar] = list()
        self.integral_sum: Scalar = 0
        self.sum_threshold = sum_threshold

    def step(self, current: Any, target: Any) -> Scalar:
        """Computes the correction factor.

        Args:
            `current` (Any): Current state of the system.
            `target` (Any): Target state of the system.

        Returns:
            Scalar: Correction factor.
        """

        error = self._compute_error(current, target)
        feedback = (
            self._compute_derivative_response(error)
            + self._compute_integral_response(error)
            + self._compute_proportional_response(error)
        )
        self.append_error(error)
        return feedback

    def reset(self, is_latest_error_kept: bool = False):
        """Empties the error timeseries of the PID controller, effectively starting a new
        control iteration.

        Args:
            is_latest_error_kept (bool, optional): True if the latest error is kept in the error
                timeseries to avoid starting from scratch if the target remains the same. False
                if the timeseries should be completely emptied. Defaults to False.
        """
        self.error_timeseries.clear

    def append_error(self, error: Scalar) -> None:
        """Appends the latest error to the error timeseries attribute. If the timeseries is at
        the maximum buffer size, the least recently computed error is evicted from the timeseries
        and the new one is appended.

        Args:
            `error` (Scalar): The latest error.
        """
        if len(self.error_timeseries) < self.buf_size:
            self.error_timeseries.append(error)
        else:
            self.integral_sum -= self.error_timeseries[0] * self.time_period
            self.error_timeseries.pop(0)
            self.error_timeseries.append(error)

    @abstractmethod
    def _compute_error(self, current: Any, target: Any) -> Scalar:
        """Computes the currently observed error.

        Args:
            current (Any): Current state of the system.
            target (Any): Target state of the system.

        Returns:
            Scalar: Current error between the current and target states.
        """
        pass

    @abstractmethod
    def _compute_proportional_response(self, error: Any) -> Scalar:
        """
        Args:
            error (Any): Current calculated error for present iteration

        Returns:
            Scalar: The proportional component of the correction factor.
        """
        pass

    @abstractmethod
    def _compute_integral_response(self, error: Any) -> Scalar:
        """
        Args:
            error (Any): Current calculated error for present iteration
            integral_sum (int): The running total of integral sum from integral response

        Returns:
            Scalar: The integral component of the correction factor.
        """
        pass

    @abstractmethod
    def _compute_derivative_response(self, error: Any) -> Scalar:
        """
         Args:
            error (Any): Current calculated error for present iteration
            last_error (float): The error calculated in the previous iteration

        Returns:
            Scalar: The derivative component of the correction factor.
        """
        pass

    @property
    def last_error(self):
        return self.error_timeseries[-1]


class VanilaPID(PID):
    """General Class for the PID controller.

    Extends: PID
    """

    def __init__(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        sum_threshold: Scalar,
    ):
        """Initializes the class attributes.

        Args:
            `kp` (Scalar): The proportional component tuning constant.
            `ki` (Scalar): The integral component tuning constant.
            `kd` (Scalar): The derivative component tuning constant.
            `time_period` (Scalar): Time period between error samples.
            `buf_size` (int): The max number of error samples to store for integral component.
            `last_error` (float): The error calculated in the previous iteration
            `integral_sum` (Scalar): The running total of integral sum from integral response
        """
        super().__init__(
            kp,
            ki,
            kd,
            time_period,
            buf_size,
            sum_threshold,
        )

    def _compute_proportional_response(self, error: Scalar) -> Scalar:
        return self.kp * error

    def _compute_integral_response(self, error: Scalar) -> Scalar:
        current_sum = self.integral_sum + (self.time_period * error)

        if abs(current_sum) < self.sum_threshold:
            self.integral_sum = current_sum
        else:
            self.integral_sum = self.sum_threshold
        return self.ki * self.integral_sum

    def _compute_derivative_response(self, error: Scalar) -> Scalar:
        if not self.error_timeseries:
            return 0
        else:
            derivative_response = (error - self.last_error) / self.time_period
            return self.kd * derivative_response


class RudderPID(VanilaPID):
    """Individual Class for the rudder PID controller.

    Extends: VanilaPID
    """

    def _compute_error(self, current: Scalar, target: Scalar) -> Scalar:
        error = atan2(sin(target - current), cos(target - current))
        current_bound = bound_to_180(current)
        target_bound = bound_to_180(target)
        if current_bound == target_bound:
            error = 0
        return error


class RobotPID(VanilaPID):
    """Individual Class for the Model Robot Arm PID controller.

    Extends: VanilaPID
    """

    def _compute_error(self, current: Scalar, target: Scalar) -> Scalar:
        return target - current
