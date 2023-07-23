from boat_simulator.common.types import Scalar
from typing import List
from abc import ABC, abstractmethod


class PID(ABC):

    # Private class member defaults
    __kp = 0
    __ki = 0
    __kd = 0
    __buf_size = 50
    __time_period = 1
    __error_timeseries = list()

    def __init__(self, kp: Scalar, ki: Scalar, kd: Scalar, time_period: Scalar, buf_size: int):
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
        self.__buf_size = buf_size
        self.__time_period = time_period
        self.__error_timeseries = list()

    def step(self, current: Scalar, target: Scalar) -> Scalar:
        raise NotImplementedError()

    def __append_error(self, error: Scalar):
        raise NotImplementedError()

    @abstractmethod
    def _compute_error(self, current: Scalar, target: Scalar) -> Scalar:
        pass

    @abstractmethod
    def _compute_proportional_response(self) -> Scalar:
        pass

    @abstractmethod
    def _compute_integral_response(self) -> Scalar:
        pass

    @abstractmethod
    def _compute_derivative_response(self) -> Scalar:
        pass

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
    def error_timeseries(self) -> List[Scalar]:
        return self.__error_timeseries


class RudderPID(PID):

    def __init__(self, kp: Scalar, ki: Scalar, kd: Scalar, time_period: Scalar, buf_size: int):
        super().__init__(kp, ki, kd, time_period, buf_size)

    def _compute_error(self, current: Scalar, target: Scalar) -> Scalar:
        raise NotImplementedError()

    def _compute_proportional_response(self) -> Scalar:
        raise NotImplementedError()

    def _compute_integral_response(self) -> Scalar:
        raise NotImplementedError()

    def _compute_derivative_response(self) -> Scalar:
        raise NotImplementedError()
