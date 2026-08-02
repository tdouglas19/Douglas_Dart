import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.sizing import evaluate_ramjet_handoff_sizing, ramjet_handoff_sweep


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


if __name__ == "__main__":
    unittest.main()
