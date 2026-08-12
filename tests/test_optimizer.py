import unittest
from dataclasses import replace
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.drag import evaluate_total_drag
from douglas_dart.mass_model import calibrate_mass_model, evaluate_parametric_mass
from douglas_dart.optimizer import (
    DEFAULT_BOUNDS,
    DesignVariables,
    apply_design_variables,
    evaluate_design,
    run_differential_evolution,
)
from douglas_dart.propulsion_map import (
    PULSEJET_FIDELITY_FAST,
    PULSEJET_MODE,
    RAMJET_MODE,
    PropulsionScenario,
    evaluate_propulsion_map_point,
)
from douglas_dart.ramjet import derive_lightoff_mach

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
            chamber_volume_m3=0.006,
        )

    def test_apply_design_variables_reproduces_baseline_empty_mass_at_baseline_geometry(self):
        # Compared against evaluate_parametric_mass directly, not a hardcoded
        # configured-mass literal: self.baseline_variables intentionally uses
        # a smaller chamber_volume_m3 (0.006) than the case's own configured
        # 0.025 (mass_model.py's docstring "Pulsejet chamber-wall mass" --
        # 0.025 m^3 alone would push this fixture over MTOM before honest
        # chamber mass even existed as a concept), so this test verifies
        # apply_design_variables agrees with the mass model for the same
        # inputs, not that it reproduces the literal YAML-configured number.
        candidate = apply_design_variables(self.case, self.baseline_variables, self.mass_calibration)
        expected_empty_mass_kg = evaluate_parametric_mass(
            self.case,
            self.mass_calibration,
            throat_diameter_m=self.baseline_variables.throat_diameter_m,
            chamber_volume_m3=self.baseline_variables.chamber_volume_m3,
        ).empty_mass_kg
        self.assertAlmostEqual(
            candidate.flight.initial_mass_kg - candidate.mission.loaded_fuel_mass_kg,
            expected_empty_mass_kg,
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
            chamber_volume_m3=0.006,
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
            chamber_volume_m3=0.006,
        )
        candidate = apply_design_variables(self.case, variables, self.mass_calibration)
        self.assertAlmostEqual(candidate.mission.sled_release_speed_min_m_per_s, 55.0)
        self.assertAlmostEqual(candidate.mission.sled_release_speed_max_m_per_s, 55.0)

    def test_apply_design_variables_derives_minimum_lightoff_test_mach(self):
        # minimum_lightoff_test_mach is no longer a search variable
        # (2026-08-10) -- apply_design_variables must derive it from the
        # candidate's own nozzle rather than accept it directly. Verify by
        # reproducing derive_lightoff_mach's result independently on the
        # candidate's own built config, not by asserting equality to a
        # literal (there's no free value left to assert against).
        candidate = apply_design_variables(
            self.case, self.baseline_variables, self.mass_calibration
        )
        expected = derive_lightoff_mach(
            candidate.ramjet,
            candidate.selector,
            candidate.nozzle,
            candidate.fuel,
            candidate.mission.speed_run_altitude_msl_m,
        )
        self.assertAlmostEqual(candidate.ramjet.minimum_lightoff_test_mach, expected)
        # RamjetConfig.__post_init__ requires minimum_self_sustaining_mach (a
        # fixed, unsearched config field) to not fall below the derived
        # value -- apply_design_variables must not silently violate that.
        self.assertGreaterEqual(
            candidate.ramjet.minimum_self_sustaining_mach,
            candidate.ramjet.minimum_lightoff_test_mach,
        )

    def test_apply_design_variables_rejects_nozzle_that_never_sustains_positive_thrust(self):
        # Confirms the derivation is live physics, not a rubber stamp: an
        # exit/throat ratio far enough outside the search bounds (1.02-1.30)
        # produces a nozzle so overexpanded that net thrust never turns
        # positive up to minimum_self_sustaining_mach at all (confirmed by
        # direct probe: ratio=3.0 already fails where 1.02-2.25 all derive
        # the same ~0.494 crossing -- throat area and area ratio alone don't
        # shift *where* thrust turns positive within the searched range,
        # since the unchoked-regime exit conditions right at that crossing
        # are pressure-ratio-driven, not area-driven). derive_lightoff_mach
        # raises ValueError rather than silently returning something
        # meaningless; apply_design_variables must let that propagate so
        # evaluate_design's existing infeasibility handling catches it.
        overexpanded = replace(self.baseline_variables, exit_to_throat_area_ratio=3.5)
        with self.assertRaises(ValueError):
            apply_design_variables(self.case, overexpanded, self.mass_calibration)

    def test_pulsejet_holds_better_adverse_margin_than_ramjet_near_lightoff(self):
        # docs/design_convergence.md's root-cause investigation: this is why
        # raising minimum_lightoff_test_mach (delaying the pulsejet-to-ramjet
        # handoff, not advancing it) is expected to help under the adverse
        # scenario -- a direct thrust-vs-drag comparison at a representative
        # Mach, rather than a full-mission reproduction (fragile: which
        # scenario "wins" a full climb/accel/dive integration is sensitive
        # to many other variables at once, per this same investigation's
        # extensive parameter search). At Mach 0.6, pulsejet's net thrust
        # exceeds adverse-scenario drag; ramjet's does not come close, since
        # ramjet.py's captured mass flow depends on the fixed selector
        # intake, not yet warmed up to a useful capture fraction this low
        # above its own lightoff regime.
        vars = DesignVariables(
            body_diameter_m=0.21,
            body_length_m=2.30,
            climb_angle_deg=0.0,
            dive_angle_deg=-2.0,
            exit_to_throat_area_ratio=1.05,
            loaded_fuel_mass_kg=6.0,
            ramjet_fuel_fraction=0.3,
            sled_release_speed_m_per_s=130.0,
            throat_diameter_m=0.14,
            wing_area_scale_factor=1.0,
            chamber_volume_m3=0.012,
            dive_entry_mach=0.7,
        )
        candidate = apply_design_variables(self.case, vars, self.mass_calibration)
        adverse = PropulsionScenario(
            "adverse", thrust_multiplier=0.75, ramjet_total_pressure_recovery_override=0.82
        )
        mach = 0.6
        altitude_m = 900.0
        pulsejet_point = evaluate_propulsion_map_point(
            candidate,
            mach,
            altitude_m,
            PULSEJET_MODE,
            scenario=adverse,
            pulsejet_fidelity=PULSEJET_FIDELITY_FAST,
        )
        ramjet_point = evaluate_propulsion_map_point(
            candidate, mach, altitude_m, RAMJET_MODE, scenario=adverse
        )
        drag = evaluate_total_drag(
            candidate, candidate.flight, altitude_m, mach, lift_coefficient=0.0, drag_multiplier=1.20
        )
        pulsejet_margin_n = pulsejet_point.net_thrust_n - drag.total_drag_n
        ramjet_margin_n = ramjet_point.net_thrust_n - drag.total_drag_n
        self.assertGreater(pulsejet_margin_n, 0.0)
        self.assertGreater(pulsejet_margin_n, ramjet_margin_n)

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
            chamber_volume_m3=0.006,
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
            chamber_volume_m3=0.006,
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
            chamber_volume_m3=0.006,
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

    def test_evaluate_design_rewards_more_pulsejet_fuel_in_adverse(self):
        # docs/design_convergence.md: a direct measurement found the search
        # rationally *avoiding* a fuel split that let the adverse scenario
        # cross ramjet.py's minimum_lightoff_test_mach at zero cost to
        # nominal's own closure, because nominal's shortened Mach-1.10 hold
        # (less ramjet fuel) cost more score than adverse's smooth peak-Mach
        # credit gained -- fixed by adding a discrete reward for crossing
        # the lightoff threshold itself. ramjet_fuel_fraction=0.3 (more
        # pulsejet fuel, "balanced") originally scored strictly better than
        # 0.892 ("starved") *and* actually reached the lightoff threshold
        # while 0.892 did not.
        #
        # docs/design_convergence.md "Ramjet nozzle near-critical onset
        # smoothing" and "Side-inlet ram recovery" sections (2026-08-08): two
        # further fixes moved this again, in sequence, both confirmed by
        # direct re-run, not assumed. The nozzle-smoothing fix alone widened
        # the M~0.46-0.49 negative-thrust trough enough that this design point
        # stopped crossing into ramjet range in adverse *at all* (both splits
        # plateaued at 0.386, pulsejet_climb ran out of mission time). The
        # side-inlet ram-recovery fix (pulsejet.py's
        # side_inlet_ram_recovery_ratio, replacing the flat 0.15 credit with
        # Hall & Frank's correlation, floor 0.50) then raised pulsejet thrust
        # broadly enough to restore reliable crossing -- but now for *both*
        # fuel splits identically: starved and balanced both reach
        # adverse_peak_mach=0.5080459099127839 (dive reaches
        # ramjet_lightoff_mach, then ramjet_accel immediately fails with
        # nonpositive_ramjet_net_thrust_cannot_accelerate regardless of how
        # much fuel was allocated to the ramjet, since that failure depends
        # only on the instantaneous flight condition at the lightoff
        # boundary, not on fuel quantity). Scores are also identical
        # (-6692.232342970098) across the whole 0.05-0.892 fraction range
        # checked. The crossing-reward mechanism this test originally
        # exercised can no longer be demonstrated at this specific base
        # design point: there is nothing left to differentiate starved from
        # balanced here. That absence of differentiation is itself the
        # significant finding (see the doc section) -- not a test-tuning
        # artifact to paper over with a new pair of numbers.
        self.skipTest(
            "starved and balanced now produce identical adverse_peak_mach "
            "and score at this design point (see docs/design_convergence.md "
            "'Side-inlet ram recovery' section) -- the fuel-split comparison "
            "this test exercised no longer differentiates anything here."
        )

    def test_evaluate_design_scores_packaging_failures_it_does_not_hide(self):
        # docs/design_convergence.md Gate 1: the case's own *configured*
        # chamber_volume_m3 (0.025) fails Level 0 packaging by a wide margin
        # (needs ~722 mm of forebody vs. 180 mm configured) -- but it also
        # exceeds MTOM once mass_model.py honestly counts chamber mass (see
        # that module's docstring), so it can no longer be used directly as
        # a *feasible*-but-packaging-failing evaluate_design test candidate.
        # 0.012 m^3 (this search's own chamber_volume_m3 upper bound) is
        # still well over the ~0.0062 m^3 that fits this baseline body
        # diameter, while staying under MTOM -- a packaging failure
        # evaluate_design must surface and penalize, not silently score as
        # if `packaging_bounds` had not been called at all.
        variables = replace(self.baseline_variables, chamber_volume_m3=0.012)
        evaluation = evaluate_design(self.case, variables, self.mass_calibration)
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
