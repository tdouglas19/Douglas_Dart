import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import PULSEJET_FIDELITY_FAST
from douglas_dart.sensitivity import (
    pulsejet_local_sensitivities,
    ramjet_local_sensitivities,
    rank_by_net_thrust_sensitivity,
)


ROOT = Path(__file__).resolve().parents[1]


class SensitivityTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )

    def test_pulsejet_sensitivity_preserves_ordered_input_bounds(self):
        point = pulsejet_local_sensitivities(
            self.case,
            variables=("throat_diameter_m",),
            pulsejet_fidelity=PULSEJET_FIDELITY_FAST,
        )[0]
        self.assertLess(point.low_input, point.baseline_input)
        self.assertLess(point.baseline_input, point.high_input)
        self.assertGreater(point.baseline_net_thrust_n, 0.0)
        self.assertTrue(point.numerical_reference_only)

    def test_ramjet_capture_coefficient_is_inactive_while_throat_limited(self):
        point = ramjet_local_sensitivities(
            self.case,
            variables=("mass_capture_coefficient",),
        )[0]
        self.assertGreater(point.baseline_inlet_spillage_fraction, 0.5)
        self.assertAlmostEqual(point.net_thrust_normalized_slope, 0.0, places=10)

    def test_ranking_uses_absolute_normalized_thrust_slope(self):
        points = ramjet_local_sensitivities(
            self.case,
            variables=("throat_diameter_m", "mass_capture_coefficient"),
        )
        ranked = rank_by_net_thrust_sensitivity(points)
        self.assertEqual(ranked[0].variable, "throat_diameter_m")


if __name__ == "__main__":
    unittest.main()
