"""Tests classes and functions in boat_simulator/common/unit_conversions.py"""

from boat_simulator.common.unit_conversions import ConversionFactor
import pytest
import math


class TestConversionFactor:
    @pytest.mark.parametrize(
        "factor, initial_value, expected_converted_value",
        [(60, 1, 60), (0.5, 2, 0.5 * 2), (-10.2, 14.22, -10.2 * 14.22)],
    )
    def test_forward_convert(self, factor, initial_value, expected_converted_value):
        conversion_factor = ConversionFactor(factor=factor)
        actual_converted_value = conversion_factor.forward_convert(initial_value)
        assert math.isclose(actual_converted_value, expected_converted_value)

    @pytest.mark.parametrize(
        "factor, initial_value, expected_converted_value",
        [(60, 60, 1), (0.5, 1, 1 / 0.5), (-10.2, -10.2 * 14.22, 14.22)],
    )
    def test_backward_convert(self, factor, initial_value, expected_converted_value):
        conversion_factor = ConversionFactor(factor=factor)
        actual_converted_value = conversion_factor.backward_convert(initial_value)
        assert math.isclose(actual_converted_value, expected_converted_value)

    @pytest.mark.parametrize("factor", [1, 2, 5, 10, 0.5, -18.9, -17, -13.33])
    def test_inverse(self, factor):
        conversion_factor = ConversionFactor(factor=factor)
        inverse_conversion_factor = conversion_factor.inverse()
        assert math.isclose(inverse_conversion_factor.factor, 1 / factor)
        assert math.isclose(inverse_conversion_factor.factor, conversion_factor.inverse_factor)

    @pytest.mark.parametrize(
        "factor1, factor2, expected_product_factor",
        [(1, 1, 1), (2, 5, 10), (1 / 2, 2, 1), (0, 1, 0)],
    )
    def test_multiplication(self, factor1, factor2, expected_product_factor):
        conversion_factor1 = ConversionFactor(factor=factor1)
        conversion_factor2 = ConversionFactor(factor=factor2)
        product_conversion_factor = conversion_factor1 * conversion_factor2
        reverse_product_conversion_factor = conversion_factor2 * conversion_factor1
        assert math.isclose(product_conversion_factor.factor, expected_product_factor)
        assert math.isclose(reverse_product_conversion_factor.factor, expected_product_factor)
        assert math.isclose(
            product_conversion_factor.factor, reverse_product_conversion_factor.factor
        )


class TestUnitConverter:
    pass
