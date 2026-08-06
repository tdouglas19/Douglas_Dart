import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.robustness import run_robustness_trade, summarize_mass_budget


ROOT = Path(__file__).resolve().parents[1]


class RobustnessTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
        )
        self.robustness_path = ROOT / "configs" / "robustness_candidate_b.yaml"

    def test_mass_budget_reconciles_and_retains_high_case_margin(self):
        summary = summarize_mass_budget(self.case, self.robustness_path)
        self.assertAlmostEqual(summary.current_total_mass_kg, 21.0)
        self.assertAlmostEqual(summary.uncertainty_plus_total_kg, 2.9)
        self.assertTrue(summary.current_mass_matches_case)
        self.assertAlmostEqual(summary.high_mass_margin_to_requirement_kg, 1.1)

    def test_trade_selects_candidate_b_and_does_not_hide_adverse_failure(self):
        # 170 mm, not 160 mm: decomposing the flat ramjet recovery into an idealized
        # Mach-dependent shock term times an installed-efficiency factor reduces
        # recovery at Mach 1.10 by ~0.1%, which is enough to flip the 160 mm
        # candidate's conservative-scenario excess thrust negative (it was a 1.5 N
        # razor's-edge pass under the old flat-recovery model). See
        # docs/design_convergence.md, "Model correction that moved the throat from
        # 160 to 170 mm".
        result = run_robustness_trade(self.case, self.robustness_path)
        selected = result.selected_candidate
        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected.body_diameter_m, 0.210)
        self.assertAlmostEqual(selected.throat_diameter_m, 0.170)
        self.assertAlmostEqual(selected.exit_to_throat_area_ratio, 1.05)
        scenarios = {item.scenario: item for item in selected.scenarios}
        self.assertGreaterEqual(scenarios["conservative"].excess_thrust_n, 50.0)
        self.assertLess(scenarios["adverse"].excess_thrust_n, 0.0)
        self.assertIn(
            "candidate_fails_informational_adverse_scenario",
            selected.status,
        )


if __name__ == "__main__":
    unittest.main()
