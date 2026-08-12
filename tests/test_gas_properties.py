import unittest

from douglas_dart.gas_properties import real_gas_gamma, real_gas_specific_heat_j_per_kg_k


class GasPropertiesTests(unittest.TestCase):
    def test_gamma_near_cold_air_value_at_sea_level_temperature(self):
        self.assertAlmostEqual(real_gas_gamma(288.15), 1.4, places=2)

    def test_gamma_drops_at_combustion_temperature(self):
        cold = real_gas_gamma(288.15)
        hot = real_gas_gamma(1900.0)
        self.assertLess(hot, cold)
        self.assertGreater(hot, 1.2)
        self.assertLess(hot, 1.35)

    def test_cp_near_standard_air_value_at_sea_level_temperature(self):
        self.assertAlmostEqual(real_gas_specific_heat_j_per_kg_k(288.15), 1005.0, delta=25.0)

    def test_cp_rises_at_combustion_temperature(self):
        cold = real_gas_specific_heat_j_per_kg_k(288.15)
        hot = real_gas_specific_heat_j_per_kg_k(1900.0)
        self.assertGreater(hot, cold)

    def test_rejects_non_positive_temperature(self):
        with self.assertRaises(ValueError):
            real_gas_gamma(0.0)
        with self.assertRaises(ValueError):
            real_gas_specific_heat_j_per_kg_k(-10.0)


if __name__ == "__main__":
    unittest.main()
