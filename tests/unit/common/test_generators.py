"""Tests classes and functions in boat_simulator/common/generators.py"""
import math

import numpy as np
import pytest

from boat_simulator.common.generators import MVGaussianGenerator


class TestGaussianGenerator:
    pass


class TestMVGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, cov",
        [
            (np.array([1, 1]), np.eye(2)),
            (np.array([1, 1, 1]), np.eye(3)),
            (np.array([1, 1, 1, 1]), np.eye(4)),
        ],
    )
    def test_multivariate_vector_generation_2d(self, mean, cov):
        """This test compares the calculated cov/mean of generated vectors against
        expected mean/cov arrays

        Args:
            mean (array): Input array of size n with average 1
            cov (array): Identity matrix based on mean array size (I_n)
        """
        NUM_SAMPLES = 5000
        samples = np.zeros(shape=(NUM_SAMPLES, mean.size))
        generator = MVGaussianGenerator(mean=mean, cov=cov)
        for i in range(NUM_SAMPLES):
            samples[i, :] = generator.next()
        sample_mean = np.sum(samples, axis=0) / NUM_SAMPLES
        sample_cov = np.cov(samples, rowvar=False)

        assert np.allclose(sample_cov, cov, atol=0.1)
        assert np.isclose(sample_mean, mean, 0.1).all()


class TestConstantGenerator:
    pass
