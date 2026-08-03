import unittest
from dataclasses import replace
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.sizing import (
    evaluate_shared_nozzle_trade,
    evaluate_peak_mach_diameter_trade,
    evaluate_ramjet_handoff_sizing,
    geometrically_scaled_drag_area_target_m2,
    peak_mach_altitude_trade_sweep,
    peak_mach_diameter_trade_sweep,
    ramjet_handoff_sweep,
    select_minimum_feasible_shared_nozzle,
    shared_nozzle_feasibility_bounds,
    shared_nozzle_trade_sweep,
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

    def test_shared_nozzle_candidate_closes_explicit_reserve_and_duration(self):
        candidate = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        point = evaluate_shared_nozzle_trade(
            candidate,
            candidate.vehicle.body_diameter_m,
            candidate.nozzle.throat_diameter_m,
            candidate.nozzle.exit_to_throat_area_ratio,
            pulsejet_warmup_s=0.05,
            pulsejet_measurement_s=0.05,
            pulsejet_time_step_s=0.00004,
        )
        self.assertTrue(point.packageable_with_configured_allowances)
        self.assertTrue(point.can_hold_peak_mach_with_derate)
        self.assertTrue(
            point.static_fuel_hold_exceeds_minimum_supersonic_duration
        )
        self.assertTrue(point.configured_loaded_mass_within_requirement)
        self.assertGreater(point.ramjet_derated_thrust_margin_n, 0.0)
        self.assertGreater(point.pulsejet_mean_net_thrust_n, 0.0)
        self.assertEqual(point.pulsejet_warmup_duration_s, 0.05)
        self.assertEqual(point.pulsejet_measurement_duration_s, 0.05)

    def test_minimum_feasible_selector_uses_explicit_lexicographic_rule(self):
        candidate = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        points = shared_nozzle_trade_sweep(
            candidate,
            body_diameters_m=(0.205,),
            throat_diameters_m=(0.120, 0.130),
            exit_to_throat_area_ratios=(1.05,),
            pulsejet_warmup_s=0.05,
            pulsejet_measurement_s=0.05,
            pulsejet_time_step_s=0.00004,
        )
        selected = select_minimum_feasible_shared_nozzle(points)
        self.assertIsNotNone(selected)
        self.assertEqual(selected.throat_diameter_m, 0.130)

    def test_candidate_has_narrow_local_body_and_throat_feasibility_margins(self):
        candidate = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        bounds = shared_nozzle_feasibility_bounds(candidate)
        self.assertTrue(bounds.fixed_architecture_has_body_feasibility_interval)
        self.assertAlmostEqual(bounds.minimum_packageable_body_diameter_m, 0.205)
        self.assertGreater(
            bounds.maximum_body_diameter_for_derated_drag_budget_m,
            candidate.vehicle.body_diameter_m,
        )
        self.assertLess(
            bounds.maximum_body_diameter_for_derated_drag_budget_m,
            0.211,
        )
        self.assertIsNotNone(
            bounds.minimum_throat_diameter_for_derated_drag_budget_m
        )
        self.assertGreater(
            bounds.minimum_throat_diameter_for_derated_drag_budget_m,
            0.126,
        )
        self.assertLess(
            bounds.minimum_throat_diameter_for_derated_drag_budget_m,
            0.128,
        )

    def test_altitude_trade_includes_endpoints_and_exposes_static_endurance(self):
        candidate = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        points = peak_mach_altitude_trade_sweep(
            candidate,
            3000.0,
            6500.0,
            500.0,
        )
        self.assertEqual(points[0].altitude_m, 3000.0)
        self.assertEqual(points[-1].altitude_m, 6500.0)
        self.assertTrue(all(point.can_hold_peak_mach_with_derate for point in points))
        self.assertLess(
            points[-1].ramjet_fuel_mass_flow_kg_per_s,
            points[0].ramjet_fuel_mass_flow_kg_per_s,
        )
        self.assertGreater(
            points[-1].ramjet_fuel_limited_hold_duration_s,
            points[0].ramjet_fuel_limited_hold_duration_s,
        )


if __name__ == "__main__":
    unittest.main()
