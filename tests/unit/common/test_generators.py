"""Tests classes and functions in boat_simulator/common/generators.py"""
from boat_simulator.common.generators import MVGaussianGenerator
import numpy as np
import pytest
import math


class TestGaussianGenerator:
    pass


class TestMVGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, cov, num_samples",
        [(60, 1, 60), (0.5, 2, 0.5 * 2), (-10.2, 14.22, -10.2 * 14.22)],
    )
    def test_multivariate_vector_generation(self, mean, cov, num_samples): # did not use cov
        samples = list()
        vector_total = 0
        for i in range(1, num_samples): 
            sample = MVGaussianGenerator() # will this generate random vectors?
            vector_total += sample
            samples.insert(i, sample)
        
        actual_mean = vector_total / i
        assert math.isclose(actual_mean, mean)

class TestConstantGenerator:
    pass
