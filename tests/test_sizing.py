import unittest
from dataclasses import replace
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.sizing import (
    evaluate_peak_mach_diameter_trade,
    evaluate_ramjet_handoff_sizing,
    geometrically_scaled_drag_area_target_m2,
    peak_mach_diameter_trade_sweep,
    ramjet_handoff_sweep,
)


ROOT = Path(__file__).resolve().parents[1]


class RamjetSizingTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "reference_case.yaml")

    def test_required_throat_matches_captured_mass_flow(self):
        point = evaluate_ramjet_handoff_sizing(self.case, altitude_m=0.0, mach=1.2)
        self.assertAlmostEqual(
            point.matched_nozzle_mass_flow_residual_fraction,
            0.0,
            places=12,
        )
        self.assertEqual(
            point.required_throat_within_nominal_body,
            point.required_throat_to_body_diameter_ratio <= 1.0,
        )

    def test_reference_handoff_exposes_packaging_conflict(self):
        point = evaluate_ramjet_handoff_sizing(self.case, altitude_m=0.0, mach=1.1)
        self.assertGreater(point.required_throat_to_body_diameter_ratio, 1.0)
        self.assertFalse(point.required_throat_within_nominal_body)
        self.assertGreater(point.current_inlet_spillage_fraction, 0.5)

    def test_sweep_includes_requested_endpoints(self):
        points = ramjet_handoff_sweep(self.case, 0.0, 0.8, 1.3, 0.1)
        self.assertEqual(len(points), 6)
        self.assertAlmostEqual(points[0].mach, 0.8)
        self.assertAlmostEqual(points[-1].mach, 1.3)

    def test_outer_body_can_grow_without_changing_fixed_intake_capture(self):
        larger_body_case = replace(
            self.case,
            vehicle=replace(self.case.vehicle, body_diameter_m=0.250),
        )
        original = evaluate_ramjet_handoff_sizing(self.case, altitude_m=4500.0, mach=1.1)
        larger = evaluate_ramjet_handoff_sizing(
            larger_body_case,
            altitude_m=4500.0,
            mach=1.1,
        )
        self.assertEqual(original.intake_diameter_m, larger.intake_diameter_m)
        self.assertAlmostEqual(
            original.required_matched_throat_diameter_m,
            larger.required_matched_throat_diameter_m,
        )
        self.assertFalse(original.required_throat_within_nominal_body)
        self.assertTrue(larger.required_throat_within_nominal_body)

    def test_geometric_similarity_doubles_diameter_and_quadruples_drag_area(self):
        small = geometrically_scaled_drag_area_target_m2(self.case, 0.200)
        large = geometrically_scaled_drag_area_target_m2(self.case, 0.400)
        self.assertAlmostEqual(large, 4.0 * small)

    def test_peak_mach_hold_duration_is_derived_from_fuel_budget(self):
        low_drag_case = replace(
            self.case,
            vehicle=replace(
                self.case.vehicle,
                peak_mach_drag_area_ceiling_m2=0.003,
            ),
        )
        one_budget = evaluate_peak_mach_diameter_trade(low_drag_case, 0.225)
        two_budget_case = replace(
            low_drag_case,
            mission=replace(
                low_drag_case.mission,
                ramjet_speed_run_fuel_budget_kg=2.80,
            ),
        )
        two_budget = evaluate_peak_mach_diameter_trade(two_budget_case, 0.225)
        self.assertTrue(one_budget.can_hold_peak_mach_against_scaled_drag_target)
        self.assertIsNotNone(one_budget.fuel_limited_peak_mach_hold_duration_s)
        self.assertAlmostEqual(
            two_budget.fuel_limited_peak_mach_hold_duration_s,
            2.0 * one_budget.fuel_limited_peak_mach_hold_duration_s,
        )

    def test_reference_trade_reports_packaging_and_drag_separately(self):
        points = peak_mach_diameter_trade_sweep(self.case, 0.195, 0.225, 0.005)
        self.assertFalse(points[0].throat_packageable_without_radial_allowance)
        self.assertTrue(points[-1].throat_packageable_without_radial_allowance)
        self.assertGreater(points[-1].available_radial_clearance_m, 0.0)
        self.assertFalse(points[-1].can_hold_peak_mach_against_scaled_drag_target)
        self.assertGreater(points[-1].drag_area_reduction_required_fraction, 0.0)
        self.assertAlmostEqual(
            points[-1].full_throttle_fuel_endurance_s,
            self.case.mission.ramjet_speed_run_fuel_budget_kg
            / points[-1].full_throttle_ramjet_fuel_mass_flow_kg_per_s,
        )


if __name__ == "__main__":
    unittest.main()
