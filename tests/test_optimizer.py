import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.optimizer import (
    DEFAULT_BOUNDS,
    DesignVariables,
    apply_design_variables,
    evaluate_design,
    run_differential_evolution,
)

ROOT = Path(__file__).resolve().parents[1]


class OptimizerTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")
        self.baseline_variables = DesignVariables(
            body_diameter_m=self.case.vehicle.body_diameter_m,
            throat_diameter_m=self.case.nozzle.throat_diameter_m,
            exit_to_throat_area_ratio=self.case.nozzle.exit_to_throat_area_ratio,
            loaded_fuel_mass_kg=self.case.mission.loaded_fuel_mass_kg,
            ramjet_fuel_fraction=(
                self.case.mission.ramjet_speed_run_fuel_budget_kg
                / self.case.mission.loaded_fuel_mass_kg
            ),
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
        )

    def test_apply_design_variables_preserves_empty_mass(self):
        candidate = apply_design_variables(self.case, self.baseline_variables)
        self.assertAlmostEqual(
            candidate.flight.initial_mass_kg - candidate.mission.loaded_fuel_mass_kg,
            self.case.flight.initial_mass_kg - self.case.mission.loaded_fuel_mass_kg,
        )
        self.assertAlmostEqual(candidate.vehicle.body_diameter_m, self.case.vehicle.body_diameter_m)

    def test_apply_design_variables_rejects_infeasible_combination(self):
        bad_variables = DesignVariables(
            body_diameter_m=0.05,  # smaller than the fixed 0.195 m intake diameter
            throat_diameter_m=0.16,
            exit_to_throat_area_ratio=1.05,
            loaded_fuel_mass_kg=3.8,
            ramjet_fuel_fraction=0.36,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
        )
        with self.assertRaises(ValueError):
            apply_design_variables(self.case, bad_variables)

    def test_evaluate_design_reports_infeasible_without_raising(self):
        bad_variables = DesignVariables(
            body_diameter_m=0.05,
            throat_diameter_m=0.16,
            exit_to_throat_area_ratio=1.05,
            loaded_fuel_mass_kg=3.8,
            ramjet_fuel_fraction=0.36,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
        )
        evaluation = evaluate_design(self.case, bad_variables)
        self.assertFalse(evaluation.feasible)
        self.assertIsNotNone(evaluation.infeasibility_reason)
        self.assertLess(evaluation.score, -1e8)

    def test_evaluate_design_feasible_candidate_wires_all_variables(self):
        evaluation = evaluate_design(self.case, self.baseline_variables)
        self.assertTrue(evaluation.feasible)
        self.assertIsNotNone(evaluation.nominal_peak_mach)
        self.assertIsNotNone(evaluation.adverse_peak_mach)
        self.assertIsNotNone(evaluation.mass_margin_kg)

    def test_tiny_differential_evolution_run_improves_or_holds_best_score(self):
        records = run_differential_evolution(
            self.case,
            population_size=4,
            generations=2,
            seed=1,
        )
        self.assertEqual(len(records), 2)
        self.assertGreaterEqual(records[-1].best_score, records[0].best_score)
        for record in records:
            self.assertGreaterEqual(record.feasible_count, 0)
            self.assertLessEqual(record.feasible_count, record.population_size)


if __name__ == "__main__":
    unittest.main()
