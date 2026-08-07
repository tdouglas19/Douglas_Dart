import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.trajectory import (
    ADVERSE_SCENARIO,
    MissionScenario,
    NOMINAL_SCENARIO,
    _stall_speed_m_per_s,
    simulate_mission,
)

ROOT = Path(__file__).resolve().parents[1]


class MissionScenarioTests(unittest.TestCase):
    def test_rejects_nonpositive_multipliers(self):
        with self.assertRaises(ValueError):
            MissionScenario("bad", thrust_multiplier=0.0)
        with self.assertRaises(ValueError):
            MissionScenario("bad", drag_multiplier=-1.0)

    def test_rejects_invalid_recovery(self):
        with self.assertRaises(ValueError):
            MissionScenario("bad", ramjet_total_pressure_recovery=1.5)


class TrajectoryIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
        )

    def test_stall_margin_and_dynamic_pressure_are_reported(self):
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        self.assertIsInstance(result.minimum_stall_margin_fraction, float)
        self.assertGreater(result.peak_dynamic_pressure_pa, 0.0)
        self.assertEqual(
            result.stall_margin_violated, result.minimum_stall_margin_fraction < 0.0
        )

    def test_stall_margin_violation_is_recorded_in_status_when_it_occurs(self):
        # Candidate B's configured sled-release speed is below its own 1g
        # stall speed at max mass (docs/level0_feasibility_bounds.md) -- this
        # test documents that the trajectory-level check independently
        # surfaces the same finding, not that the finding is desirable.
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        if result.stall_margin_violated:
            self.assertIn("stall_margin_violated_1g_level_flight_bound", result.final_status)

    def test_nominal_mission_produces_monotonic_time_and_bounded_fuel(self):
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        self.assertTrue(result.points)
        times = [point.time_s for point in result.points]
        self.assertEqual(times, sorted(times))
        for point in result.points:
            self.assertGreaterEqual(point.pulsejet_fuel_remaining_kg, 0.0)
            self.assertGreaterEqual(point.ramjet_fuel_remaining_kg, 0.0)
            self.assertGreater(point.mass_kg, 0.0)
        # mass is monotonically nonincreasing (fuel burn only, no staging mass loss)
        masses = [point.mass_kg for point in result.points]
        self.assertTrue(all(a >= b - 1e-6 for a, b in zip(masses, masses[1:])))

    def test_phases_are_contiguous_and_named(self):
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        self.assertTrue(result.phases)
        for earlier, later in zip(result.phases, result.phases[1:]):
            self.assertLessEqual(earlier.end_time_s, later.start_time_s + 1e-9)

    def test_time_above_mach_one_flag_matches_requirement(self):
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        expected = (
            result.time_above_mach_one_s
            >= result.minimum_time_above_mach_one_requirement_s
        )
        self.assertEqual(result.meets_minimum_time_above_mach_one, expected)

    def test_adverse_scenario_does_not_out_perform_nominal(self):
        nominal = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        adverse = simulate_mission(
            self.case, ADVERSE_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        self.assertLessEqual(
            adverse.peak_mach_reached, nominal.peak_mach_reached + 1e-6
        )

    def test_no_altitude_loss_during_transonic_regime(self):
        """Boom Supersonic Prize rule: level or climbing only from Mach 0.8+."""
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        self.assertTrue(result.transonic_no_altitude_loss_rule_satisfied)
        self.assertNotIn(
            "transonic_no_altitude_loss_rule_violated", result.final_status
        )
        transonic_start_mach = self.case.requirements.transonic_regime_start_mach
        regulated_points = [
            point
            for point in result.points
            if point.mach >= transonic_start_mach
            and point.phase in {"dive", "ramjet_accel", "mach_hold"}
        ]
        for earlier, later in zip(regulated_points, regulated_points[1:]):
            self.assertGreaterEqual(later.altitude_m, earlier.altitude_m - 1e-6)

    def test_stall_speed_scales_with_mass_and_air_density(self):
        # V_stall = sqrt(2W/(rho S CLmax)): heavier -> faster, thinner air -> faster.
        light = _stall_speed_m_per_s(self.case, mass_kg=15.0, altitude_m=0.0)
        heavy = _stall_speed_m_per_s(self.case, mass_kg=25.0, altitude_m=0.0)
        self.assertGreater(heavy, light)
        sea_level = _stall_speed_m_per_s(self.case, mass_kg=20.0, altitude_m=0.0)
        high_altitude = _stall_speed_m_per_s(self.case, mass_kg=20.0, altitude_m=6000.0)
        self.assertGreater(high_altitude, sea_level)

    def test_zoom_climb_never_descends(self):
        result = simulate_mission(
            self.case, NOMINAL_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        zoom_points = [p for p in result.points if p.phase == "zoom_climb"]
        for earlier, later in zip(zoom_points, zoom_points[1:]):
            self.assertGreaterEqual(later.altitude_m, earlier.altitude_m - 1e-6)

    def test_adverse_scenario_is_reported_not_hidden(self):
        result = simulate_mission(
            self.case, ADVERSE_SCENARIO, time_step_s=0.05, max_time_s=120.0
        )
        self.assertTrue(result.numerical_reference_only)
        self.assertIn(
            "energy_state_model_no_lift_trim_stability_or_control_solved",
            result.final_status,
        )


if __name__ == "__main__":
    unittest.main()
