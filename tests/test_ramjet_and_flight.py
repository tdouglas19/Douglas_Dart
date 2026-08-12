import unittest
from pathlib import Path

from douglas_dart.atmosphere import standard_atmosphere
from douglas_dart.compressible import stagnation_pressure
from douglas_dart.config import load_reference_case
from douglas_dart.flight import PointMassState, longitudinal_derivative
from douglas_dart.ramjet import evaluate_ramjet


ROOT = Path(__file__).resolve().parents[1]


class RamjetAndFlightTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "reference_case.yaml")

    def test_mach_point_eight_is_lightoff_test_not_self_sustaining(self):
        result = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            0.0,
            0.8,
        )
        self.assertIn("below_configured_self_sustaining_mach", result.status)
        self.assertNotIn("below_configured_lightoff_test_mach", result.status)
        self.assertFalse(result.self_sustaining_candidate)
        self.assertGreater(result.inlet_spillage_fraction, 0.0)
        self.assertAlmostEqual(
            result.air_mass_flow_kg_per_s + result.fuel_mass_flow_kg_per_s,
            result.nozzle_capacity_kg_per_s,
            places=12,
        )

    def test_below_lightoff_mach_reports_zero_thrust_not_negative(self):
        lightoff_mach = self.case.ramjet.minimum_lightoff_test_mach
        result = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.mission.speed_run_altitude_msl_m,
            lightoff_mach - 0.35,
        )
        self.assertEqual(result.net_thrust_n, 0.0)
        self.assertEqual(result.gross_thrust_n, 0.0)
        self.assertEqual(result.inlet_momentum_drag_n, 0.0)
        self.assertEqual(result.fuel_mass_flow_kg_per_s, 0.0)
        self.assertEqual(result.air_mass_flow_kg_per_s, 0.0)
        self.assertFalse(result.self_sustaining_candidate)
        self.assertIn("below_configured_lightoff_test_mach", result.status)
        self.assertIn("ramjet_not_attempted_below_lightoff_mach", result.status)
        self.assertGreater(result.potential_captured_air_mass_flow_kg_per_s, 0.0)

    def test_thrust_is_continuous_at_the_lightoff_mach_boundary(self):
        lightoff_mach = self.case.ramjet.minimum_lightoff_test_mach
        altitude_m = self.case.mission.speed_run_altitude_msl_m
        just_below = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            altitude_m,
            lightoff_mach - 1e-6,
        )
        at_threshold = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            altitude_m,
            lightoff_mach,
        )
        self.assertEqual(just_below.net_thrust_n, 0.0)
        self.assertGreaterEqual(at_threshold.net_thrust_n, 0.0)

    def test_flight_mass_rate_is_negative(self):
        derivative = longitudinal_derivative(
            PointMassState(0.0, 0.0, 100.0, 0.0, self.case.flight.initial_mass_kg),
            self.case.flight,
            thrust_n=500.0,
            fuel_mass_flow_kg_per_s=0.2,
            angle_of_attack_rad=0.05,
        )
        self.assertEqual(derivative.mass_rate_kg_per_s, -0.2)
        self.assertGreater(derivative.drag_n, 0.0)

    def test_throat_limited_ramjet_reports_spillage_without_underfed_failure(self):
        result = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.mission.speed_run_altitude_msl_m,
            self.case.mission.peak_mach,
        )
        self.assertGreater(result.inlet_spillage_fraction, 0.5)
        self.assertIn("fixed_nozzle_requires_inlet_spillage_coupling", result.status)
        self.assertNotIn("fixed_nozzle_underfed_pressure_match_required", result.status)
        self.assertAlmostEqual(
            result.net_thrust_n,
            result.gross_thrust_n - result.inlet_momentum_drag_n,
        )

    def test_ramjet_recovery_is_ideal_shock_times_installed_efficiency(self):
        altitude_m = self.case.mission.speed_run_altitude_msl_m
        mach = self.case.mission.peak_mach
        atmosphere = standard_atmosphere(altitude_m)
        ideal_total_pressure_pa = stagnation_pressure(atmosphere.pressure_pa, mach)
        result = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            altitude_m,
            mach,
        )
        expected_recovery = (
            result.ideal_inlet_shock_recovery * self.case.selector.ramjet_total_pressure_recovery
        )
        self.assertAlmostEqual(
            result.combustor_inlet_total_pressure_pa / ideal_total_pressure_pa,
            expected_recovery,
            places=12,
        )
        self.assertAlmostEqual(
            result.installed_total_pressure_recovery, expected_recovery, places=12
        )

    def test_ideal_inlet_shock_recovery_is_lossless_below_mach_one(self):
        from douglas_dart.ramjet import ideal_inlet_shock_recovery

        self.assertEqual(ideal_inlet_shock_recovery(0.0), 1.0)
        self.assertEqual(ideal_inlet_shock_recovery(0.99), 1.0)

    def test_ideal_inlet_shock_recovery_matches_mil_e_5008b_above_mach_one(self):
        from douglas_dart.ramjet import ideal_inlet_shock_recovery

        for mach in (1.05, 1.10, 1.30, 2.0):
            self.assertAlmostEqual(
                ideal_inlet_shock_recovery(mach),
                1.0 - 0.075 * (mach - 1.0) ** 1.35,
                places=12,
            )

    def test_ideal_inlet_shock_recovery_decreases_monotonically_above_mach_one(self):
        from douglas_dart.ramjet import ideal_inlet_shock_recovery

        machs = [1.0, 1.05, 1.10, 1.30, 1.60, 2.0]
        recoveries = [ideal_inlet_shock_recovery(m) for m in machs]
        self.assertTrue(all(a >= b for a, b in zip(recoveries, recoveries[1:])))


if __name__ == "__main__":
    unittest.main()
