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

    # Private class member defaults

    # TODO Remove these and add as defualts to the init function
    __kp: Scalar = 9.691
    __ki: Scalar = 13.78
    __kd: Scalar = 0.6658
    __time_period: Scalar = 1
    __buf_size: int = 50
    __error_timeseries: List[Scalar] = list()
    __last_error: Any = 0
    __integral_sum: Any = 0

    def __init__(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        error_timeseries: list,
        last_error: Any,
        integral_sum: Any,
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
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
        self.__buf_size = buf_size
        self.__time_period = time_period
        self.__error_timeseries = list()

        # TODO Just use the error timeseries to get the latest error
        self.__last_error = last_error

        # TODO You can store this value, but don't pass the initial value in the init function.
        # Just initialize as zero.
        self.__integral_sum = integral_sum

    def step(self, current: Any, target: Any) -> Scalar:
        """Computes the correction factor.

        Args:
            `current` (Any): Current state of the system.
            `target` (Any): Target state of the system.

        Returns:
            Scalar: Correction factor.
        """

        error = self._compute_error(current, target)

        # TODO Use the append_error function that you wrote. Also, error should be appended
        # after the feedback is computed
        self.__error_timeseries.append(error)
        feedback = (
            self._compute_derivative_response(error, self.__last_error)
            + self._compute_integral_response(error, self.__integral_sum)
            + self._compute_proportional_response(error)
        )
        self.__last_error = error
        return feedback

    def reset(self, is_latest_error_kept: bool = False) -> None:
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
            # TODO for the case when you have to remove an entry, remember to update the
            # integral sum by subtracting off the oldest error. Otherwise, the integral sum
            # will explode
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
    def _compute_integral_response(self, error: Any, integral_sum: Any) -> Scalar:
        """
        Args:
            error (Any): Current calculated error for present iteration
            integral_sum (int): The running total of integral sum from integral response

        Returns:
            Scalar: The integral component of the correction factor.
        """
        pass

    @abstractmethod
    def _compute_derivative_response(self, error: Any, last_error: Any) -> Scalar:
        """
         Args:
            error (Any): Current calculated error for present iteration
            last_error (float): The error calculated in the previous iteration

        Returns:
            Scalar: The derivative component of the correction factor.
        """
        pass

    # TODO Remove these
    @property
    def kp(self) -> Scalar:
        return self.__kp

    @property
    def ki(self) -> Scalar:
        return self.__ki

    @property
    def kd(self) -> Scalar:
        return self.__kd

    @property
    def buf_size(self) -> Scalar:
        return self.__buf_size

    @property
    def time_period(self) -> Scalar:
        return self.__time_period

    @property
    def last_error(self) -> Scalar:
        return self.__last_error

    @property
    def integral_sum(self) -> Scalar:
        return self.__integral_sum

    @property
    def error_timeseries(self) -> List[Scalar]:
        return self.__error_timeseries


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
        error_timeseries: list,
        integral_sum: Any,
        last_error: Any,
    ):
        """Initializes the class attributes.

        Args:
            `kp` (Scalar): The proportional component tuning constant.
            `ki` (Scalar): The integral component tuning constant.
            `kd` (Scalar): The derivative component tuning constant.
            `time_period` (Scalar): Time period between error samples.
            `buf_size` (int): The max number of error samples to store for integral component.
            `last_error` (float): The error calculated in the previous iteration
            `integral_sum` (int): The running total of integral sum from integral response
        """
        super().__init__(
            kp, ki, kd, time_period, buf_size, error_timeseries, integral_sum, last_error
        )

    def _compute_proportional_response(self, error: Any) -> Scalar:
        return self.kp * error  # return proportional response

    def _compute_integral_response(self, error: Any, integral_sum: Any) -> Scalar:
        integral_sum += self.time_period * error  # adds new integral response to running total

        # TODO Multiply by ki
        # TODO You should also bound the integral sum to prevent it from exploding
        # This threshold could be specified in the init function
        return integral_sum

    # TODO Remove the last_error argument and just extract the last error from the error_timeseries
    # If you have no last error (i.e. the error timeseries is empty), then just return zero
    def _compute_derivative_response(self, error, last_error: Any) -> Scalar:
        derivative_response = (error - last_error) / self.time_period  # calculates change in error

        # TODO Multiply by kd
        return derivative_response


class RudderPID(VanilaPID):
    """Individual Class for the rudder PID controller.

    Extends: VanilaPID
    """

    def _compute_error(
        self, current: Scalar, target: Scalar
    ) -> Scalar:  # target and current in degrees
        error = atan2(
            sin(target - current), cos(target - current)
        )  # return error in radians between pi and -pi
        current_bound = bound_to_180(current)
        target_bound = bound_to_180(target)
        if current_bound == target_bound:
            error = 0
        return error


class RobotPID(VanilaPID):
    """Individual Class for the Model Robot Arm PID controller.

    Extends: VanilaPID
    """

    # TODO Change the current - target
    def _compute_error(
        self, current: Scalar, target: Scalar
    ) -> Scalar:  # target and current positions
        return target - current
