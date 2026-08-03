import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
