import unittest

from douglas_dart.compressible import (
    area_ratio_from_mach,
    compressible_orifice_mass_flow,
    fixed_cd_nozzle,
    isentropic_static_pressure_ratio,
    normal_shock_downstream_mach,
    normal_shock_static_pressure_ratio,
    normal_shock_total_pressure_ratio,
    subsonic_mach_from_area_ratio,
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

    def test_area_mach_inverse_subsonic_branch(self):
        target_ratio = 2.25
        mach = subsonic_mach_from_area_ratio(target_ratio, 1.33)
        self.assertGreater(mach, 0.0)
        self.assertLess(mach, 1.0)
        self.assertAlmostEqual(area_ratio_from_mach(mach, 1.33), target_ratio, places=9)

    def test_nozzle_is_zero_without_pressure_head(self):
        result = fixed_cd_nozzle(
            100_000.0, 500.0, 101_325.0, 0.01, 0.02, 0.95, 1.33, 287.05
        )
        self.assertEqual(result.mass_flow_kg_per_s, 0.0)
        self.assertEqual(result.gross_thrust_n, 0.0)

    def test_cd_nozzle_resolves_internal_normal_shock(self):
        result = fixed_cd_nozzle(
            180_000.0, 900.0, 101_325.0, 0.01, 0.0225, 0.95, 1.33, 287.05
        )
        self.assertTrue(result.choked)
        self.assertEqual(result.regime, "choked_internal_normal_shock")
        self.assertIsNotNone(result.shock_to_throat_area_ratio)
        self.assertAlmostEqual(result.exit_pressure_pa, 101_325.0)

    def test_normal_shock_reduces_to_no_shock_at_mach_one(self):
        self.assertAlmostEqual(normal_shock_downstream_mach(1.0, 1.33), 1.0)
        self.assertAlmostEqual(normal_shock_static_pressure_ratio(1.0, 1.33), 1.0)
        self.assertAlmostEqual(normal_shock_total_pressure_ratio(1.0, 1.33), 1.0)

    def test_cd_nozzle_is_continuous_across_choking_onset(self):
        total_pressure_pa = 180_000.0
        total_temperature_k = 900.0
        throat_area_m2 = 0.01
        exit_area_m2 = 0.0225
        gamma = 1.33
        exit_mach_at_choking = subsonic_mach_from_area_ratio(2.25, gamma)
        pressure_ratio = (
            1.0 + 0.5 * (gamma - 1.0) * exit_mach_at_choking**2
        ) ** (-gamma / (gamma - 1.0))
        just_unchoked = fixed_cd_nozzle(
            total_pressure_pa,
            total_temperature_k,
            total_pressure_pa * pressure_ratio * (1.0 + 1e-7),
            throat_area_m2,
            exit_area_m2,
            0.95,
            gamma,
            287.05,
        )
        just_choked = fixed_cd_nozzle(
            total_pressure_pa,
            total_temperature_k,
            total_pressure_pa * pressure_ratio * (1.0 - 1e-7),
            throat_area_m2,
            exit_area_m2,
            0.95,
            gamma,
            287.05,
        )
        self.assertFalse(just_unchoked.choked)
        self.assertTrue(just_choked.choked)
        self.assertAlmostEqual(
            just_unchoked.mass_flow_kg_per_s,
            just_choked.mass_flow_kg_per_s,
            delta=1e-5 * just_choked.mass_flow_kg_per_s,
        )
        self.assertAlmostEqual(
            just_unchoked.gross_thrust_n,
            just_choked.gross_thrust_n,
            delta=2e-4 * just_choked.gross_thrust_n,
        )

    def test_cd_nozzle_is_continuous_as_normal_shock_reaches_exit(self):
        total_pressure_pa = 300_000.0
        total_temperature_k = 900.0
        throat_area_m2 = 0.01
        exit_area_m2 = 0.0225
        gamma = 1.33
        exit_mach = supersonic_mach_from_area_ratio(2.25, gamma)
        shock_at_exit_back_pressure_ratio = (
            isentropic_static_pressure_ratio(exit_mach, gamma)
            * normal_shock_static_pressure_ratio(exit_mach, gamma)
        )
        shock_inside = fixed_cd_nozzle(
            total_pressure_pa,
            total_temperature_k,
            total_pressure_pa * shock_at_exit_back_pressure_ratio * (1.0 + 1e-7),
            throat_area_m2,
            exit_area_m2,
            0.95,
            gamma,
            287.05,
        )
        shock_outside = fixed_cd_nozzle(
            total_pressure_pa,
            total_temperature_k,
            total_pressure_pa * shock_at_exit_back_pressure_ratio * (1.0 - 1e-7),
            throat_area_m2,
            exit_area_m2,
            0.95,
            gamma,
            287.05,
        )
        self.assertEqual(shock_inside.regime, "choked_internal_normal_shock")
        self.assertEqual(shock_outside.regime, "choked_supersonic_exit")
        self.assertAlmostEqual(
            shock_inside.gross_thrust_n,
            shock_outside.gross_thrust_n,
            delta=2e-4 * shock_inside.gross_thrust_n,
        )


if __name__ == "__main__":
    unittest.main()
