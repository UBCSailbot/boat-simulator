"""Tests classes and functions in boat_simulator/common/generators.py"""

import numpy as np
import pytest

from boat_simulator.common.generators import GaussianGenerator, MVGaussianGenerator


class TestGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, stdev, threshold",
        [(1, 1, 0.2), (1, 0, 0.2), (-1, 0, 0.2), (4, 5, 0.2), (120, 120, 1)],
    )
    def test_gaussian_generator(self, mean, stdev, threshold):
        NUM_SAMPLES = 50000

        samples = np.zeros(NUM_SAMPLES)

        generator = GaussianGenerator(mean=mean, stdev=stdev)

        for i in range(NUM_SAMPLES):
            samples[i] = generator.next()

        sample_mean = np.mean(samples)
        sample_std = np.std(samples)

        assert np.allclose(sample_std, stdev, atol=threshold)
        assert np.allclose(sample_mean, mean, atol=threshold)


class TestMVGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, cov",
        [
            (np.array([1, 1]), np.eye(2)),
            (np.array([1, 2]), np.array([[2, 1], [1, 2]])),
            (np.array([4, 5]), np.array([[3, 1], [1, 3]])),
            (np.array([100, 50]), np.array([[10, 5], [5, 10]])),
            (np.array([120, 130]), np.array([[10, 5], [5, 10]])),
            (np.array([1, 1, 1]), np.eye(3)),
            (np.array([1, 2, 3]), np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]])),
            (np.array([4, 5, 6]), np.array([[3, 2, 2], [2, 3, 2], [2, 2, 3]])),
        ],
    )
    def test_multivariate_vector_generation_2d(self, mean, cov):
        """This test compares the calculated cov/mean of generated vectors against
        expected mean/cov arrays

        Args:
            mean (array): Input array of size n with average 1
            cov (array): Identity matrix based on mean array size (I_n)
        """
        NUM_SAMPLES = 10000
        samples = np.zeros(shape=(NUM_SAMPLES, mean.size))
        generator = MVGaussianGenerator(mean=mean, cov=cov)
        for i in range(NUM_SAMPLES):
            samples[i, :] = generator.next()
        sample_mean = np.mean(samples, axis=0)
        sample_cov = np.cov(samples, rowvar=False)

        assert np.allclose(sample_cov, cov, atol=0.2)
        assert np.isclose(sample_mean, mean, 0.1).all()


class TestConstantGenerator:
    pass
