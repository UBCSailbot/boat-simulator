"""Tests classes and functions in boat_simulator/common/generators.py"""

import numpy as np
import pytest


class TestGaussianGenerator:
    pass


class TestMVGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, cov, num_samples",
        [(60, 1, 60), (0.5, 2, 0.5 * 2), (-10.2, 14.22, -10.2 * 14.22)],
    )
    def test_multivariate_vector_generation(self, mean, cov, num_samples):
        pass


class TestConstantGenerator:
    pass
