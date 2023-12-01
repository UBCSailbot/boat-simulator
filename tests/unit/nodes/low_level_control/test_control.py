"""Tests classes and functions in boat_simulator/nodes/controls.py"""
from math import cos, sin
from typing import Any

import numpy as np
import pytest

from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180
from boat_simulator.nodes.low_level_control.control import RudderPID


class TestRudderPID:
    @pytest.mark.parametrize(
        "kp, ki, kd, time_period, buf_size, error_timeseries, integral_sum, last_error, error",
        [
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 0),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 5),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 25.58),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 258.7),
        ],
    )
    def test_compute_proportional_response(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        error_timeseries: list,
        integral_sum: Any,
        last_error: Any,
        error: Any,
    ):
        """This test compares if the calculated proportional
        response is equal to the expected response

        Args:
            kp (Scalar): The proportional component tuning constant.
            ki (Scalar): The integral component tuning constant.
            kd (Scalar): The derivative component tuning constant.
            time_period (Scalar): Time period between error samples.
            buf_size (int): The max number of error samples to store for integral component.
            last_error (float): The error calculated in the previous iteration
            integral_sum (int): The running total of integral sum from integral response
            error (Any): The current positional error

        """
        pid = RudderPID(
            kp=kp,
            ki=ki,
            kd=kd,
            time_period=time_period,
            buf_size=buf_size,
            error_timeseries=error_timeseries,
            integral_sum=integral_sum,
            last_error=last_error,
        )
        expected_error = pid.kp * error
        assert np.equal(expected_error, pid._compute_proportional_response(error))

    @pytest.mark.parametrize(
        "kp, ki, kd, time_period, buf_size, error_timeseries, integral_sum, last_error, error",
        [
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 0),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 5),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 25.58),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 258.7),
        ],
    )
    def test_compute_integral_response(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        error_timeseries: list,
        integral_sum: Any,
        last_error: Any,
        error: Any,
    ):
        """This test compares if the calculated integral
        response is equal to the expected response

        Args:
            kp (Scalar): The proportional component tuning constant.
            ki (Scalar): The integral component tuning constant.
            kd (Scalar): The derivative component tuning constant.
            time_period (Scalar): Time period between error samples.
            buf_size (int): The max number of error samples to store for integral component.
            last_error (float): The error calculated in the previous iteration
            integral_sum (int): The running total of integral sum from integral response
            error (Any): The current positional error
        """
        pid = RudderPID(
            kp=kp,
            ki=ki,
            kd=kd,
            time_period=time_period,
            buf_size=buf_size,
            error_timeseries=error_timeseries,
            integral_sum=integral_sum,
            last_error=last_error,
        )
        current_sum = 0.0
        test_sum = pid._compute_integral_response(error, current_sum)
        current_sum += test_sum
        test_sum = pid._compute_integral_response(error / 2, current_sum)
        expected_error = (pid.time_period * error) + (pid.time_period * error / 2)
        assert np.equal(expected_error, test_sum)

    @pytest.mark.parametrize(
        "kp, ki, kd, time_period, buf_size, error_timeseries, integral_sum, last_error, error",
        [
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 0),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 5),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 25.58),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 258.7),
        ],
    )
    def test_compute_derivative_response(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        error_timeseries: list,
        integral_sum: Any,
        last_error: Any,
        error: Any,
    ):
        """This test compares if the calculated derivative
        response is equal to the expected response

        Args:
            ckp (Scalar): The proportional component tuning constant.
            ki (Scalar): The integral component tuning constant.
            kd (Scalar): The derivative component tuning constant.
            time_period (Scalar): Time period between error samples.
            buf_size (int): The max number of error samples to store for integral component.
            last_error (float): The error calculated in the previous iteration
            integral_sum (int): The running total of integral sum from integral response
            error (Any): The current positional error
        """
        pid = RudderPID(
            kp=kp,
            ki=ki,
            kd=kd,
            time_period=time_period,
            buf_size=buf_size,
            error_timeseries=error_timeseries,
            integral_sum=integral_sum,
            last_error=last_error,
        )
        current_error = pid._compute_derivative_response(error, pid.last_error)
        last_error = error
        stored_error = current_error
        current_error = pid._compute_derivative_response(current_error, last_error)
        assert np.equal((stored_error - last_error) / time_period, current_error)

    @pytest.mark.parametrize(
        "kp,ki,kd,time_period,buf_size,error_timeseries,integral_sum,last_error,current,target",
        [
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 5, 0),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 190, 220),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, -62.4, 120),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, 180, -180),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, -360, 360),
            (50.22, 9.678, 0.6658, 1, 50, [], 0, 0, -157.45, -158.67),
        ],
    )
    def test_compute_error(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        error_timeseries: list,
        integral_sum: Any,
        last_error: Any,
        current: Scalar,
        target: Scalar,
    ):
        """This test compares the computed error to the expected
        error in radians, difference bounded between -pi and pi

        Args:
            ckp (Scalar): The proportional component tuning constant.
            ki (Scalar): The integral component tuning constant.
            kd (Scalar): The derivative component tuning constant.
            time_period (Scalar): Time period between error samples.
            buf_size (int): The max number of error samples to store for integral component.
            last_error (float): The error calculated in the previous iteration
            integral_sum (int): The running total of integral sum from integral response
            current (Scalar): The current heading given in degrees
            target (Scalar): The target heading given in degrees
        """
        pid = RudderPID(
            kp=kp,
            ki=ki,
            kd=kd,
            time_period=time_period,
            buf_size=buf_size,
            error_timeseries=error_timeseries,
            integral_sum=integral_sum,
            last_error=last_error,
        )
        current_error = pid._compute_error(current, target)
        expected_error = np.arctan(sin(target - current) / cos(target - current))

        current_bound = bound_to_180(current)
        target_bound = bound_to_180(target)
        if current_bound == target_bound:
            current_error = 0
            expected_error = 0
        assert np.equal(current_error, expected_error)

    @pytest.mark.parametrize(
        "kp,ki,kd,time_period,buf_size,error_timeseries,integral_sum,last_error,current,target",
        [
            (50.22, 9.678, 0.6658, 1, 50, [], 8, 2, 5, 0),
            (50.22, 9.678, 0.6658, 1, 50, [], 16, 5, 190, 220),
            (50.22, 9.678, 0.6658, 1, 50, [], 25.2, 6, -62.4, 120),
            (50.22, 9.678, 0.6658, 1, 50, [], 19, 9.3, 180, -180),
            (50.22, 9.678, 0.6658, 1, 50, [], 278, -8.9, -360, 360),
            (50.22, 9.678, 0.6658, 1, 50, [], -2, 0, -157.45, -158.67),
        ],
    )
    def test_step(
        self,
        kp: Scalar,
        ki: Scalar,
        kd: Scalar,
        time_period: Scalar,
        buf_size: int,
        error_timeseries: list,
        integral_sum: Any,
        last_error: Any,
        current: Scalar,
        target: Scalar,
    ):
        """This test compares the computed error to the expected
        error in radians, difference bounded between -pi and pi

        Args:
            ckp (Scalar): The proportional component tuning constant.
            ki (Scalar): The integral component tuning constant.
            kd (Scalar): The derivative component tuning constant.
            time_period (Scalar): Time period between error samples.
            buf_size (int): The max number of error samples to store for integral component.
            last_error (float): The error calculated in the previous iteration
            integral_sum (int): The running total of integral sum from integral response
            current (Scalar): The current heading given in degrees
            target (Scalar): The target heading given in degrees
        """
        pid = RudderPID(
            kp=kp,
            ki=ki,
            kd=kd,
            time_period=time_period,
            buf_size=buf_size,
            error_timeseries=error_timeseries,
            integral_sum=integral_sum,
            last_error=last_error,
        )

        error = pid._compute_error(current, target)
        timeseries = [error]
        feedback = (
            pid._compute_derivative_response(error, pid.last_error)
            + pid._compute_integral_response(error, pid.integral_sum)
            + pid._compute_proportional_response(error)
        )
        np.equal(feedback, pid.step(current, target))
        np.equal(timeseries, pid.error_timeseries)

        error = pid._compute_error(current - 1, target - 1)
        timeseries.append(error)
        feedback = (
            pid._compute_derivative_response(error, pid.last_error)
            + pid._compute_integral_response(error, pid.integral_sum)
            + pid._compute_proportional_response(error)
        )
        np.equal(feedback, pid.step(current - 1, target - 1))
        np.equal(timeseries, pid.error_timeseries)
