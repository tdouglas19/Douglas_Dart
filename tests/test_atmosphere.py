import unittest

from douglas_dart.atmosphere import standard_atmosphere


class AtmosphereTests(unittest.TestCase):
    def test_sea_level_reference(self):
        atmosphere = standard_atmosphere(0.0)
        self.assertAlmostEqual(atmosphere.temperature_k, 288.15, places=6)
        self.assertAlmostEqual(atmosphere.pressure_pa, 101_325.0, places=3)
        self.assertAlmostEqual(atmosphere.density_kg_per_m3, 1.2250, places=3)

    def test_pressure_decreases_with_altitude(self):
        self.assertLess(
            standard_atmosphere(10_000.0).pressure_pa,
            standard_atmosphere(0.0).pressure_pa,
        )

    def test_domain_is_explicit(self):
        with self.assertRaises(ValueError):
            standard_atmosphere(25_000.0)


if __name__ == "__main__":
    unittest.main()
