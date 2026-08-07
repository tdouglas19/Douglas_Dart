import unittest
from pathlib import Path

from douglas_dart.atmosphere import standard_atmosphere
from douglas_dart.config import load_reference_case
from douglas_dart.feasibility import (
    dive_energy_benefit,
    evaluate_level0_feasibility,
    fuel_energy_and_endurance_bounds,
    packaging_bounds,
    required_lift_area,
    sled_launch_feasibility,
    stall_speed_bounds,
    thrust_to_weight_sweep,
)

ROOT = Path(__file__).resolve().parents[1]


class Level0FeasibilityTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")

    def test_sled_launch_feasibility_matches_hand_calculation(self):
        result = sled_launch_feasibility(self.case, release_speed_m_per_s=60.0)
        expected_a_m_per_s2 = 60.0**2 / (2.0 * self.case.mission.sled_rail_length_m)
        self.assertAlmostEqual(result.required_acceleration_m_per_s2, expected_a_m_per_s2, places=6)
        self.assertAlmostEqual(result.required_acceleration_g, expected_a_m_per_s2 / 9.80665, places=6)

    def test_sled_launch_feasibility_default_speed_is_within_configured_ceiling(self):
        # Candidate B's current 42 m/s release speed over a 75 m rail needs far
        # less than the 10 g placeholder ceiling -- this is the case that should
        # NOT be flagged; test_sled_launch_feasibility_flags_high_speed below
        # covers the case that should be.
        result = sled_launch_feasibility(self.case)
        self.assertTrue(result.within_configured_ceiling)

    def test_sled_launch_feasibility_flags_speed_exceeding_ceiling(self):
        result = sled_launch_feasibility(self.case, release_speed_m_per_s=200.0)
        self.assertFalse(result.within_configured_ceiling)
        self.assertGreater(result.required_acceleration_g, result.configured_acceleration_ceiling_g)

    def test_stall_speed_bound_matches_hand_calculation(self):
        result = stall_speed_bounds(self.case)
        atmosphere = standard_atmosphere(self.case.mission.field_elevation_msl_m)
        weight_n = self.case.requirements.maximum_takeoff_mass_kg * 9.80665
        expected = (
            2.0
            * weight_n
            / (
                atmosphere.density_kg_per_m3
                * self.case.flight.reference_area_m2
                * self.case.flight.maximum_lift_coefficient
            )
        ) ** 0.5
        self.assertAlmostEqual(result.stall_speed_m_per_s, expected, places=6)

    def test_stall_speed_bound_flags_current_reference_area_as_insufficient(self):
        # Documents a real, current finding (not a bug): candidate B's configured
        # reference area cannot support flight at the configured sled-release
        # speed at maximum takeoff mass and configured CL_max. This is exactly
        # the kind of thing Gate 1 is supposed to catch -- see
        # docs/level0_feasibility_bounds.md.
        result = stall_speed_bounds(self.case)
        self.assertFalse(result.passes)
        self.assertGreater(result.stall_speed_m_per_s, result.sled_release_speed_max_m_per_s)

    def test_required_lift_area_is_positive_and_consistent_with_stall_bound(self):
        result = required_lift_area(self.case)
        self.assertGreater(result.required_area_m2, 0.0)
        self.assertFalse(result.passes)

    def test_thrust_to_weight_sweep_selects_pulsejet_below_and_ramjet_at_self_sustaining_mach(self):
        # 0.5, not 0.2: thrust_to_weight_sweep constructs PulsejetSimulator
        # directly with a short fixed measurement window (propulsion_map.py's
        # module docstring explains why this call site bypasses the adaptive
        # window _run_pulsejet_simulation uses). With the configured "side"
        # inlet_type, real cycle period lengthens sharply below roughly
        # Mach 0.3-0.35 (refill is driven by a weak pressure differential
        # instead of ram pressure -- a side inlet is credited only
        # SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION of the ram-pressure rise a
        # straight inlet gets), so a short fixed window there reads as
        # near-zero thrust even though the engine is still genuinely firing.
        # 0.5 sits safely above where that fixed-window measurement is valid
        # and below the lightoff gate.
        points = thrust_to_weight_sweep(self.case, mach_values=(0.5, 1.1))
        modes = {p.mach: p.mode for p in points}
        self.assertEqual(modes[0.5], "pulsejet")
        self.assertEqual(modes[1.1], "ramjet")
        for point in points:
            self.assertGreater(point.thrust_to_weight, 0.0)

    def test_dive_energy_benefit_increases_speed(self):
        result = dive_energy_benefit(self.case, dive_height_m=1000.0, entry_speed_m_per_s=40.0)
        self.assertGreater(result.exit_speed_m_per_s, result.entry_speed_m_per_s)

    def test_dive_energy_benefit_rejects_negative_height(self):
        with self.assertRaises(ValueError):
            dive_energy_benefit(self.case, dive_height_m=-100.0, entry_speed_m_per_s=40.0)

    def test_fuel_energy_bounds_are_positive_and_ramjet_budget_within_loaded(self):
        result = fuel_energy_and_endurance_bounds(self.case)
        self.assertGreater(result.loaded_chemical_energy_j, 0.0)
        self.assertLessEqual(result.ramjet_fuel_budget_kg, result.loaded_fuel_mass_kg)
        self.assertIsNotNone(result.pulsejet_endurance_s_bound)
        self.assertIsNotNone(result.ramjet_endurance_s_bound)

    def test_packaging_bounds_returns_named_checks(self):
        checks = packaging_bounds(self.case)
        self.assertGreaterEqual(len(checks), 4)
        for check in checks:
            self.assertTrue(check.name)
            self.assertTrue(check.detail)

    def test_evaluate_level0_feasibility_rolls_up_failing_checks(self):
        report = evaluate_level0_feasibility(self.case)
        self.assertEqual(report.case_name, self.case.name)
        self.assertFalse(report.all_pass)
        self.assertIn("stall speed bound", report.failing_checks)


if __name__ == "__main__":
    unittest.main()
