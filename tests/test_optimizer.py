import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.mass_model import calibrate_mass_model
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
        self.mass_calibration = calibrate_mass_model(
            self.case, ROOT / "configs" / "robustness_candidate_b.yaml"
        )
        self.baseline_variables = DesignVariables(
            body_diameter_m=self.case.vehicle.body_diameter_m,
            body_length_m=self.case.vehicle.body_length_m,
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
            sled_release_speed_m_per_s=self.case.mission.sled_release_speed_max_m_per_s,
            wing_area_scale_factor=1.0,
        )

    def test_apply_design_variables_reproduces_baseline_empty_mass_at_baseline_geometry(self):
        candidate = apply_design_variables(self.case, self.baseline_variables, self.mass_calibration)
        self.assertAlmostEqual(
            candidate.flight.initial_mass_kg - candidate.mission.loaded_fuel_mass_kg,
            self.case.flight.initial_mass_kg - self.case.mission.loaded_fuel_mass_kg,
            places=6,
        )
        self.assertAlmostEqual(candidate.vehicle.body_diameter_m, self.case.vehicle.body_diameter_m)

    def test_apply_design_variables_grows_empty_mass_with_body_geometry(self):
        larger = DesignVariables(
            body_diameter_m=self.case.vehicle.body_diameter_m * 1.15,
            body_length_m=self.case.vehicle.body_length_m * 1.15,
            throat_diameter_m=self.case.nozzle.throat_diameter_m,
            exit_to_throat_area_ratio=self.case.nozzle.exit_to_throat_area_ratio,
            loaded_fuel_mass_kg=self.case.mission.loaded_fuel_mass_kg,
            ramjet_fuel_fraction=0.3,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
            sled_release_speed_m_per_s=self.case.mission.sled_release_speed_max_m_per_s,
            wing_area_scale_factor=1.0,
        )
        baseline_candidate = apply_design_variables(
            self.case, self.baseline_variables, self.mass_calibration
        )
        larger_candidate = apply_design_variables(self.case, larger, self.mass_calibration)
        baseline_empty_mass_kg = (
            baseline_candidate.flight.initial_mass_kg
            - baseline_candidate.mission.loaded_fuel_mass_kg
        )
        larger_empty_mass_kg = (
            larger_candidate.flight.initial_mass_kg - larger_candidate.mission.loaded_fuel_mass_kg
        )
        self.assertGreater(larger_empty_mass_kg, baseline_empty_mass_kg)

    def test_apply_design_variables_wires_sled_release_speed_into_min_and_max(self):
        variables = DesignVariables(
            body_diameter_m=self.case.vehicle.body_diameter_m,
            body_length_m=self.case.vehicle.body_length_m,
            throat_diameter_m=self.case.nozzle.throat_diameter_m,
            exit_to_throat_area_ratio=self.case.nozzle.exit_to_throat_area_ratio,
            loaded_fuel_mass_kg=self.case.mission.loaded_fuel_mass_kg,
            ramjet_fuel_fraction=0.3,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
            sled_release_speed_m_per_s=55.0,
            wing_area_scale_factor=1.0,
        )
        candidate = apply_design_variables(self.case, variables, self.mass_calibration)
        self.assertAlmostEqual(candidate.mission.sled_release_speed_min_m_per_s, 55.0)
        self.assertAlmostEqual(candidate.mission.sled_release_speed_max_m_per_s, 55.0)

    def test_apply_design_variables_rejects_infeasible_combination(self):
        bad_variables = DesignVariables(
            body_diameter_m=0.05,  # smaller than the fixed 0.195 m intake diameter
            body_length_m=self.case.vehicle.body_length_m,
            throat_diameter_m=0.16,
            exit_to_throat_area_ratio=1.05,
            loaded_fuel_mass_kg=3.8,
            ramjet_fuel_fraction=0.36,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
            sled_release_speed_m_per_s=40.0,
            wing_area_scale_factor=1.0,
        )
        with self.assertRaises(ValueError):
            apply_design_variables(self.case, bad_variables, self.mass_calibration)

    def test_evaluate_design_reports_infeasible_without_raising(self):
        bad_variables = DesignVariables(
            body_diameter_m=0.05,
            body_length_m=self.case.vehicle.body_length_m,
            throat_diameter_m=0.16,
            exit_to_throat_area_ratio=1.05,
            loaded_fuel_mass_kg=3.8,
            ramjet_fuel_fraction=0.36,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
            sled_release_speed_m_per_s=40.0,
            wing_area_scale_factor=1.0,
        )
        evaluation = evaluate_design(self.case, bad_variables, self.mass_calibration)
        self.assertFalse(evaluation.feasible)
        self.assertIsNotNone(evaluation.infeasibility_reason)
        self.assertLess(evaluation.score, -1e8)

    def test_evaluate_design_reports_infeasible_without_raising_when_simulate_mission_rejects(self):
        # Distinct from test_evaluate_design_reports_infeasible_without_raising
        # above: that test only exercises the exception path around
        # apply_design_variables. ramjet_fuel_fraction=1.0 passes
        # apply_design_variables (a valid ReferenceCase gets built) but
        # leaves zero pulsejet-phase fuel, which simulate_mission itself
        # rejects -- a second, separate except block in evaluate_design that
        # this test suite had never exercised (a DE search sampling
        # ramjet_fuel_fraction near its own 1.0 upper bound hit exactly this
        # and crashed the whole run with an unhandled TypeError from a stale
        # CandidateEvaluation(...) call missing required fields).
        bad_variables = DesignVariables(
            body_diameter_m=self.case.vehicle.body_diameter_m,
            body_length_m=self.case.vehicle.body_length_m,
            throat_diameter_m=self.case.nozzle.throat_diameter_m,
            exit_to_throat_area_ratio=self.case.nozzle.exit_to_throat_area_ratio,
            loaded_fuel_mass_kg=self.case.mission.loaded_fuel_mass_kg,
            ramjet_fuel_fraction=1.0,
            climb_angle_deg=8.0,
            dive_angle_deg=-10.0,
            dive_entry_mach=0.45,
            sled_release_speed_m_per_s=self.case.mission.sled_release_speed_max_m_per_s,
            wing_area_scale_factor=1.0,
        )
        evaluation = evaluate_design(self.case, bad_variables, self.mass_calibration)
        self.assertFalse(evaluation.feasible)
        self.assertIsNotNone(evaluation.infeasibility_reason)
        self.assertLess(evaluation.score, -1e8)
        self.assertIsNone(evaluation.packaging_failures)

    def test_evaluate_design_feasible_candidate_wires_all_variables(self):
        evaluation = evaluate_design(self.case, self.baseline_variables, self.mass_calibration)
        self.assertTrue(evaluation.feasible)
        self.assertIsNotNone(evaluation.nominal_peak_mach)
        self.assertIsNotNone(evaluation.adverse_peak_mach)
        self.assertIsNotNone(evaluation.mass_margin_kg)

    def test_evaluate_design_scores_packaging_failures_it_does_not_hide(self):
        # docs/design_convergence.md: the baseline candidate B geometry fails
        # Level 0 packaging (pulsejet chamber needs far more forebody length
        # than configured) even before any search touches it -- this is not
        # a search-found failure, it's the starting point's own known gap.
        # evaluate_design must surface and penalize this, not silently score
        # a candidate that packaging_bounds says cannot be built.
        evaluation = evaluate_design(self.case, self.baseline_variables, self.mass_calibration)
        self.assertTrue(evaluation.feasible)
        self.assertIsNotNone(evaluation.packaging_failures)
        self.assertGreaterEqual(evaluation.packaging_failures, 1)
        self.assertIn(
            "pulsejet chamber volume fits within forebody length at full body cross-section",
            evaluation.packaging_failure_names,
        )

    def test_tiny_differential_evolution_run_improves_or_holds_best_score(self):
        records = run_differential_evolution(
            self.case,
            mass_budget_path=ROOT / "configs" / "robustness_candidate_b.yaml",
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
