import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import PULSEJET_FIDELITY_FAST
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
        # 210/170 mm: the ramjet capture-area fix (ramjet.py reading the full
        # circular intake instead of selector.available_area_m2's 50% split
        # -- docs/assumptions.md) moved this to 205/160 mm by itself. The
        # pulsejet fluid-inertance inlet model added afterward (pulsejet.py,
        # replacing the instantaneous compressible_orifice_mass_flow inlet
        # calculation) substantially raised pulsejet thrust in the
        # M=0.35-0.75 band too, shifting the minimum-feasible trade search's
        # balance back to the larger throat/body (210/170 mm). The ramjet
        # nozzle near-critical onset smoothing fix
        # (compressible.py's _NOZZLE_ONSET_SMOOTHING_PRESSURE_RATIO_MARGIN,
        # docs/design_convergence.md) moved this back down to 205/160 mm on
        # its own -- but the side-inlet ram-recovery fix (pulsejet.py's
        # side_inlet_ram_recovery_ratio, replacing the flat 0.15 credit with
        # Hall & Frank's mass-flow-coefficient correlation whose *floor* is
        # 0.50, over 3x the old flat value) raised pulsejet thrust broadly
        # enough to move this back to 210/170 mm again -- confirmed by direct
        # re-run, not assumed.
        result = run_robustness_trade(
            self.case, self.robustness_path, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
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
