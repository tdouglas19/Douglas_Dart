import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.drag import (
    drag_rise_ratio,
    evaluate_total_drag,
    mach_indexed_zero_lift_drag_area_m2,
)
from douglas_dart.sizing import geometrically_scaled_drag_area_target_m2

ROOT = Path(__file__).resolve().parents[1]


class DragModelTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")

    def test_peak_mach_anchor_matches_scaled_drag_area_budget(self):
        anchored_area_m2 = mach_indexed_zero_lift_drag_area_m2(self.case, 1.10)
        budget_area_m2 = geometrically_scaled_drag_area_target_m2(
            self.case, self.case.vehicle.body_diameter_m
        )
        self.assertAlmostEqual(anchored_area_m2, budget_area_m2, places=9)

    def test_drag_rise_ratio_is_flat_subsonic_and_peaks_near_mach_one(self):
        self.assertAlmostEqual(drag_rise_ratio(0.0), drag_rise_ratio(0.3))
        self.assertGreater(drag_rise_ratio(1.0), drag_rise_ratio(0.6))
        self.assertGreaterEqual(drag_rise_ratio(1.0), drag_rise_ratio(1.30))

    def test_drag_rise_ratio_clamps_outside_table(self):
        self.assertEqual(drag_rise_ratio(-1.0), drag_rise_ratio(0.0))
        self.assertEqual(drag_rise_ratio(5.0), drag_rise_ratio(1.50))

    def test_total_drag_breakdown_sums_to_total(self):
        breakdown = evaluate_total_drag(
            self.case,
            self.case.flight,
            self.case.mission.speed_run_altitude_msl_m,
            self.case.mission.peak_mach,
            lift_coefficient=0.15,
        )
        self.assertAlmostEqual(
            breakdown.zero_lift_drag_n + breakdown.induced_drag_n,
            breakdown.total_drag_n,
            places=6,
        )
        self.assertGreater(breakdown.total_drag_n, 0.0)

    def test_drag_multiplier_scales_total_drag_linearly(self):
        base = evaluate_total_drag(
            self.case, self.case.flight, 4500.0, 1.10, lift_coefficient=0.0
        )
        scaled = evaluate_total_drag(
            self.case,
            self.case.flight,
            4500.0,
            1.10,
            lift_coefficient=0.0,
            drag_multiplier=1.20,
        )
        self.assertAlmostEqual(scaled.total_drag_n, 1.20 * base.total_drag_n, places=6)


if __name__ == "__main__":
    unittest.main()
