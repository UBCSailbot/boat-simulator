"""Tests classes and functions in boat_simulator/common/unit_conversions.py"""

import math

import pytest

from boat_simulator.common.unit_conversions import (
    ConversionFactor,
    ConversionFactors,
    UnitConverter,
)


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
    def test_init(self):
        """
        Test instance attribute assignment using attributes or using kwargs into constructor
        """
        unit_converter1 = UnitConverter(
            prop1=ConversionFactors.sec_to_min, prop2=ConversionFactors.sec_to_hr
        )
        assert unit_converter1.prop1 == ConversionFactors.sec_to_min
        assert unit_converter1.prop2 == ConversionFactors.sec_to_hr

        converted_values1 = unit_converter1.convert(prop1=120, prop2=3600)

        assert converted_values1["prop1"] == 2.0
        assert converted_values1["prop2"] == 1.0

        conversion_factors = {
            "prop1": ConversionFactors.sec_to_min,
            "prop2": ConversionFactors.sec_to_hr,
        }
        values = {"prop1": 120, "prop2": 3600}
        unit_converter2 = UnitConverter(**conversion_factors)

        assert unit_converter2.prop1 == ConversionFactors.sec_to_min
        assert unit_converter2.prop2 == ConversionFactors.sec_to_hr

        converted_values2 = unit_converter2.convert(**values)

        assert converted_values2["prop1"] == 2.0
        assert converted_values2["prop2"] == 1.0

    def test_convert_m_km(self):
        unit_converter = UnitConverter(
            m_to_km=ConversionFactors.m_to_km, km_to_m=ConversionFactors.km_to_m
        )

        converted_values = unit_converter.convert(m_to_km=1000, km_to_m=1)

        assert converted_values["m_to_km"] == 1
        assert converted_values["km_to_m"] == 1000

    def test_convert_cm_m(self):
        unit_converter = UnitConverter(
            cm_to_m=ConversionFactors.cm_to_m, m_to_cm=ConversionFactors.m_to_cm
        )

        converted_values = unit_converter.convert(cm_to_m=100, m_to_cm=1)

        assert converted_values["cm_to_m"] == 1
        assert converted_values["m_to_cm"] == 100

    def test_convert_cm_km(self):
        unit_converter = UnitConverter(
            km_to_cm=ConversionFactors.km_to_cm, cm_to_km=ConversionFactors.cm_to_km
        )

        converted_values = unit_converter.convert(km_to_cm=1, cm_to_km=1e5)

        assert converted_values["km_to_cm"] == 1e5
        assert converted_values["cm_to_km"] == 1

    def test_convert_ft_m(self):
        unit_converter = UnitConverter(
            m_to_ft=ConversionFactors.m_to_ft, ft_to_m=ConversionFactors.ft_to_m
        )

        converted_values = unit_converter.convert(m_to_ft=100.0, ft_to_m=10)

        assert math.isclose(converted_values["m_to_ft"], 328.084, abs_tol=1e-6)
        assert math.isclose(converted_values["ft_to_m"], 3.048, abs_tol=1e-6)

    def test_convert_ft_mi(self):
        unit_converter = UnitConverter(
            mi_to_ft=ConversionFactors.mi_to_ft, ft_to_mi=ConversionFactors.ft_to_mi
        )

        converted_values = unit_converter.convert(mi_to_ft=1, ft_to_mi=1000)

        assert converted_values["mi_to_ft"] == 5280
        assert math.isclose(converted_values["ft_to_mi"], 0.189394, abs_tol=1e-6)

    def test_convert_m_mi(self):
        unit_converter = UnitConverter(
            mi_to_m=ConversionFactors.mi_to_m, m_to_mi=ConversionFactors.m_to_mi
        )

        converted_values = unit_converter.convert(mi_to_m=10, m_to_mi=1e4)

        assert math.isclose(converted_values["mi_to_m"], 16093.4, abs_tol=1e-6)
        assert math.isclose(converted_values["m_to_mi"], 6.213727, abs_tol=1e-6)

    def test_convert_km_mi(self):
        unit_converter = UnitConverter(
            km_to_mi=ConversionFactors.km_to_mi, mi_to_km=ConversionFactors.mi_to_km
        )

        converted_values = unit_converter.convert(km_to_mi=1.0, mi_to_km=10.0)

        assert math.isclose(converted_values["km_to_mi"], 0.6213727, abs_tol=1e-6)
        assert converted_values["mi_to_km"] == 16.0934

    def test_convert_mi_nautical_mi(self):
        unit_converter = UnitConverter(
            nat_mi_to_mi=ConversionFactors.nautical_mi_to_mi,
            mi_to_nat_mi=ConversionFactors.mi_to_nautical_mi,
        )

        converted_values = unit_converter.convert(nat_mi_to_mi=1.0, mi_to_nat_mi=1.0)

        assert converted_values["nat_mi_to_mi"] == 1.15078
        assert math.isclose(converted_values["mi_to_nat_mi"], 0.868976, abs_tol=1e-6)

    def test_convert_km_nautical_mi(self):
        unit_converter = UnitConverter(
            nat_mi_to_km=ConversionFactors.nautical_mi_to_km,
            km_to_nat_mi=ConversionFactors.km_to_nautical_mi,
        )

        converted_values = unit_converter.convert(nat_mi_to_km=1.0, km_to_nat_mi=1.0)

        assert converted_values["nat_mi_to_km"] == 1.852
        assert math.isclose(converted_values["km_to_nat_mi"], 0.539957, abs_tol=1e-6)

    def test_convert_sec_min(self):
        unit_converter = UnitConverter(
            sec_to_min=ConversionFactors.sec_to_min,
            min_to_sec=ConversionFactors.min_to_sec,
        )

        converted_values = unit_converter.convert(sec_to_min=120.0, min_to_sec=10.0)

        assert converted_values["sec_to_min"] == 2.0
        assert converted_values["min_to_sec"] == 600.0

    def test_convert_sec_h(self):
        unit_converter = UnitConverter(
            sec_to_h=ConversionFactors.sec_to_h,
            h_to_sec=ConversionFactors.h_to_sec,
        )

        converted_values = unit_converter.convert(sec_to_h=3600, h_to_sec=1)

        assert converted_values["sec_to_h"] == 1
        assert converted_values["h_to_sec"] == 3600

    def test_convert_min_h(self):
        unit_converter = UnitConverter(
            min_to_h=ConversionFactors.min_to_h,
            h_to_min=ConversionFactors.h_to_min,
        )

        converted_values = unit_converter.convert(min_to_h=90, h_to_min=2)

        assert converted_values["min_to_h"] == 1.5
        assert converted_values["h_to_min"] == 120

    def test_convert_miPh_kmPh(self):
        pass

    def test_convert_mPs_kmPh(self):
        pass

    def test_convert_knots_kmPh(self):
        pass
