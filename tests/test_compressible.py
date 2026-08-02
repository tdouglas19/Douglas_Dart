import unittest

from douglas_dart.compressible import (
    area_ratio_from_mach,
    compressible_orifice_mass_flow,
    fixed_cd_nozzle,
    supersonic_mach_from_area_ratio,
)


class CompressibleFlowTests(unittest.TestCase):
    def test_no_flow_against_pressure_gradient(self):
        mass_flow, choked = compressible_orifice_mass_flow(
            100_000.0, 300.0, 110_000.0, 0.01, 0.8, 1.4, 287.05
        )
        self.assertEqual(mass_flow, 0.0)
        self.assertFalse(choked)

    def test_choked_flow_independent_of_lower_back_pressure(self):
        first, first_choked = compressible_orifice_mass_flow(
            300_000.0, 500.0, 100_000.0, 0.01, 0.9, 1.4, 287.05
        )
        second, second_choked = compressible_orifice_mass_flow(
            300_000.0, 500.0, 50_000.0, 0.01, 0.9, 1.4, 287.05
        )
        self.assertTrue(first_choked and second_choked)
        self.assertAlmostEqual(first, second, places=10)

    def test_area_mach_inverse_supersonic_branch(self):
        target_ratio = 2.25
        mach = supersonic_mach_from_area_ratio(target_ratio, 1.33)
        self.assertGreater(mach, 1.0)
        self.assertAlmostEqual(area_ratio_from_mach(mach, 1.33), target_ratio, places=9)

    def test_nozzle_is_zero_without_pressure_head(self):
        result = fixed_cd_nozzle(
            100_000.0, 500.0, 101_325.0, 0.01, 0.02, 0.95, 1.33, 287.05
        )
        self.assertEqual(result.mass_flow_kg_per_s, 0.0)
        self.assertEqual(result.gross_thrust_n, 0.0)


if __name__ == "__main__":
    unittest.main()
