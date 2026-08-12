import dataclasses
import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import (
    NOMINAL,
    PULSEJET_FIDELITY_FAST,
    PULSEJET_KM_MODE,
    PULSEJET_MODE,
    RAMJET_MODE,
    PropulsionScenario,
    build_propulsion_map,
    evaluate_propulsion_map_point,
)

# Unit tests check structural/qualitative correctness (signs, field
# presence, scaling), not exact converged-thrust precision -- fast fidelity
# (propulsion_map.py's PULSEJET_FIDELITY_FAST) keeps this suite fast without
# weakening any assertion. Production code still defaults to full fidelity;
# this override is test-only.

ROOT = Path(__file__).resolve().parents[1]


class PropulsionMapTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")

    def test_pulsejet_point_has_no_ramjet_only_fields(self):
        # 0.5, not 0.2: chosen for a clearly-positive, fast-to-simulate
        # thrust value. Below roughly Mach 0.3-0.35, real pulsejet cycle
        # period lengthens sharply (with the configured "side" inlet_type,
        # refill is driven by a weak pressure differential instead of ram
        # pressure) -- evaluate_propulsion_map_point's adaptive measurement
        # window (propulsion_map.py's _run_pulsejet_simulation) still
        # measures genuine, if weaker, positive thrust there, just at
        # higher simulated-time cost than this test needs to pay.
        point = evaluate_propulsion_map_point(
            self.case, 0.5, 0.0, PULSEJET_MODE, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertEqual(point.mode, PULSEJET_MODE)
        self.assertIsNone(point.potential_air_mass_flow_kg_per_s)
        self.assertIsNone(point.spilled_mass_flow_fraction)
        self.assertIsNone(point.self_sustaining_status)
        self.assertIsNotNone(point.peak_chamber_pressure_pa)
        self.assertGreater(point.net_thrust_n, 0.0)
        self.assertGreater(point.captured_air_mass_flow_kg_per_s, 0.0)

    def test_ramjet_point_has_no_pulsejet_only_fields(self):
        point = evaluate_propulsion_map_point(
            self.case, self.case.mission.peak_mach, self.case.mission.speed_run_altitude_msl_m, RAMJET_MODE
        )
        self.assertEqual(point.mode, RAMJET_MODE)
        self.assertIsNone(point.peak_chamber_pressure_pa)
        self.assertIsNotNone(point.potential_air_mass_flow_kg_per_s)
        self.assertIsNotNone(point.spilled_mass_flow_fraction)
        self.assertIsNotNone(point.self_sustaining_status)

    def test_unknown_mode_raises(self):
        with self.assertRaises(ValueError):
            evaluate_propulsion_map_point(self.case, 0.5, 0.0, "turbofan")

    def test_scenario_thrust_multiplier_scales_both_modes(self):
        derated = PropulsionScenario("derated", thrust_multiplier=0.5)
        nominal_pj = evaluate_propulsion_map_point(
            self.case, 0.2, 0.0, PULSEJET_MODE, scenario=NOMINAL, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        derated_pj = evaluate_propulsion_map_point(
            self.case, 0.2, 0.0, PULSEJET_MODE, scenario=derated, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertAlmostEqual(derated_pj.net_thrust_n, nominal_pj.net_thrust_n * 0.5, places=6)

        nominal_rj = evaluate_propulsion_map_point(self.case, 1.1, 4500.0, RAMJET_MODE, scenario=NOMINAL)
        derated_rj = evaluate_propulsion_map_point(self.case, 1.1, 4500.0, RAMJET_MODE, scenario=derated)
        self.assertAlmostEqual(derated_rj.net_thrust_n, nominal_rj.net_thrust_n * 0.5, places=6)

    def test_scenario_ramjet_recovery_override_changes_installed_recovery(self):
        overridden = PropulsionScenario("low_recovery", ramjet_total_pressure_recovery_override=0.5)
        point = evaluate_propulsion_map_point(self.case, 1.1, 4500.0, RAMJET_MODE, scenario=overridden)
        self.assertLess(point.installed_total_pressure_recovery, 0.6)

    def test_build_propulsion_map_covers_full_grid(self):
        points = build_propulsion_map(
            self.case,
            mach_values=(0.2, 0.5),
            altitude_values=(0.0,),
            modes=(PULSEJET_MODE,),
            pulsejet_fidelity=PULSEJET_FIDELITY_FAST,
        )
        self.assertEqual(len(points), 2)

    def test_ramjet_lightoff_status_reflects_configured_gates(self):
        # Mach points relative to the case's own (now-derived, not
        # hardcoded -- ramjet.py's derive_lightoff_mach) thresholds, not
        # literals: 0.5 used to sit comfortably below the old hand-picked
        # 0.80 lightoff Mach, but the derivation can land anywhere.
        below = evaluate_propulsion_map_point(
            self.case, self.case.ramjet.minimum_lightoff_test_mach - 0.1, 4500.0, RAMJET_MODE
        )
        self.assertEqual(below.lightoff_status, "below_lightoff_test_mach")
        above = evaluate_propulsion_map_point(
            self.case, self.case.ramjet.minimum_self_sustaining_mach, 4500.0, RAMJET_MODE
        )
        self.assertEqual(above.lightoff_status, "at_or_above_self_sustaining_mach")


class PulsejetKmModeTests(unittest.TestCase):
    """PULSEJET_KM_MODE: additive third mode, sibling pulsejet-km model
    (docs/pulsejet_external_model_audit.md). Uses reference_case.yaml, not
    shared_nozzle_candidate_b.yaml -- only the former has a `pulsejet_km:`
    section (real Argus As-014/V-1 geometry, not candidate-specific)."""

    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "reference_case.yaml")

    def test_pulsejet_km_point_has_no_ramjet_or_native_pulsejet_only_fields(self):
        point = evaluate_propulsion_map_point(
            self.case, 0.0, 0.0, PULSEJET_KM_MODE, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertEqual(point.mode, PULSEJET_KM_MODE)
        self.assertIsNone(point.potential_air_mass_flow_kg_per_s)
        self.assertIsNone(point.spilled_mass_flow_fraction)
        self.assertIsNone(point.self_sustaining_status)
        self.assertIsNone(point.peak_chamber_pressure_pa)
        self.assertIsNone(point.combustor_temperature_k)
        self.assertIsNone(point.installed_total_pressure_recovery)
        self.assertGreater(point.net_thrust_n, 0.0)
        self.assertGreater(point.captured_air_mass_flow_kg_per_s, 0.0)
        self.assertIn("pulsejet_km_classification_stable_limit_cycle", point.validity_flags)

    def test_missing_pulsejet_km_engine_config_raises_clearly(self):
        case_without_km = dataclasses.replace(self.case, pulsejet_km_engine_config=None)
        with self.assertRaises(ValueError):
            evaluate_propulsion_map_point(case_without_km, 0.0, 0.0, PULSEJET_KM_MODE)

    def test_unknown_fidelity_raises(self):
        with self.assertRaises(ValueError):
            evaluate_propulsion_map_point(self.case, 0.0, 0.0, PULSEJET_KM_MODE, pulsejet_fidelity="ludicrous")

    def test_scenario_thrust_multiplier_scales_pulsejet_km(self):
        derated = PropulsionScenario("derated", thrust_multiplier=0.5)
        nominal = evaluate_propulsion_map_point(
            self.case, 0.0, 0.0, PULSEJET_KM_MODE, scenario=NOMINAL, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        derated_point = evaluate_propulsion_map_point(
            self.case, 0.0, 0.0, PULSEJET_KM_MODE, scenario=derated, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertAlmostEqual(derated_point.net_thrust_n, nominal.net_thrust_n * 0.5, places=6)

    def test_build_propulsion_map_covers_pulsejet_km(self):
        points = build_propulsion_map(
            self.case,
            mach_values=(0.0, 0.2),
            altitude_values=(0.0,),
            modes=(PULSEJET_KM_MODE,),
            pulsejet_fidelity=PULSEJET_FIDELITY_FAST,
        )
        self.assertEqual(len(points), 2)
        self.assertTrue(all(point.mode == PULSEJET_KM_MODE for point in points))


class PulsejetModeGuardedPrimaryDispatchTests(unittest.TestCase):
    """PULSEJET_MODE's guarded pulsejet-km-first dispatch (propulsion_map.py's
    `_pulsejet_mode_point`, 2026-08-11): pulsejet-km is tried first whenever
    a candidate provides `pulsejet_km_engine_config`, falling back to the
    native simulator whenever pulsejet-km signals it cannot answer."""

    def test_prefers_pulsejet_km_when_result_is_trustworthy(self):
        # reference_case.yaml has a real pulsejet_km: section; Mach 0.0 is
        # squarely inside pulsejet-km's own validated envelope and reaches
        # STABLE_LIMIT_CYCLE per PulsejetKmModeTests above.
        case = load_reference_case(ROOT / "configs" / "reference_case.yaml")
        point = evaluate_propulsion_map_point(
            case, 0.0, 0.0, PULSEJET_MODE, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertEqual(point.mode, PULSEJET_MODE)
        self.assertIn(
            "pulsejet_km_thrust_known_low_bias_see_pulsejet_km_architecture_md",
            point.validity_flags,
        )
        self.assertIn("pulsejet_km_classification_stable_limit_cycle", point.validity_flags)
        self.assertNotIn("pulsejet_km_primary_rejected_fell_back_to_native", point.validity_flags)

        # Same underlying query as PULSEJET_KM_MODE would make directly --
        # the guarded dispatch should return pulsejet-km's actual answer,
        # not a different number, when it accepts pulsejet-km's result.
        km_point = evaluate_propulsion_map_point(
            case, 0.0, 0.0, PULSEJET_KM_MODE, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertAlmostEqual(point.net_thrust_n, km_point.net_thrust_n, places=6)

    def test_falls_back_to_native_above_km_validated_mach_envelope(self):
        # Mach 0.8 is above pulsejet-km's own MACH_VALIDATED_ENVELOPE_MAXIMUM
        # (0.7) -- the guard must reject it and fall back to the native
        # simulator rather than return pulsejet-km's extrapolated (and, per
        # its own architecture.md, unphysical-above-0.7) answer.
        case = load_reference_case(ROOT / "configs" / "reference_case.yaml")
        point = evaluate_propulsion_map_point(
            case, 0.8, 0.0, PULSEJET_MODE, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertEqual(point.mode, PULSEJET_MODE)
        self.assertIn("pulsejet_km_primary_rejected_fell_back_to_native", point.validity_flags)
        self.assertNotIn(
            "pulsejet_km_thrust_known_low_bias_see_pulsejet_km_architecture_md",
            point.validity_flags,
        )

    def test_no_pulsejet_km_config_uses_native_directly_with_no_km_flags(self):
        # shared_nozzle_candidate_b.yaml has no pulsejet_km: section --
        # every actual vehicle candidate config today is in this state (see
        # _pulsejet_mode_point's docstring), so this must stay silent: no
        # low-bias flag (pulsejet-km was never consulted) and no
        # fell-back-to-native flag either (there was nothing to fall back
        # from -- the flag exists to make an actual rejection visible, not
        # to announce the ordinary no-config case on every single point).
        case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")
        self.assertIsNone(case.pulsejet_km_engine_config)
        point = evaluate_propulsion_map_point(
            case, 0.5, 0.0, PULSEJET_MODE, pulsejet_fidelity=PULSEJET_FIDELITY_FAST
        )
        self.assertNotIn(
            "pulsejet_km_thrust_known_low_bias_see_pulsejet_km_architecture_md",
            point.validity_flags,
        )
        self.assertNotIn("pulsejet_km_primary_rejected_fell_back_to_native", point.validity_flags)


if __name__ == "__main__":
    unittest.main()
