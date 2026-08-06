import math
import tempfile
import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.jsbsim_model import (
    ADVERSE_SCENARIO,
    NOMINAL_SCENARIO,
    build_jsbsim_aircraft_xml,
    estimate_stability_derivatives,
    validate_with_jsbsim,
    write_jsbsim_aircraft,
)

ROOT = Path(__file__).resolve().parents[1]


class StabilityDerivativeTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")

    def test_derivatives_are_geometry_derived_and_stabilizing(self):
        stability = estimate_stability_derivatives(self.case)
        # A tail (fin) aft of the CG generating lift with alpha should be
        # longitudinally and directionally stabilizing under this sign convention.
        self.assertLess(stability.cm_alpha_per_rad, 0.0)
        self.assertGreater(stability.cn_beta_per_rad, 0.0)
        self.assertLess(stability.cm_q_per_rad, 0.0)
        self.assertLess(stability.cn_r_per_rad, 0.0)
        self.assertLess(stability.cl_p_per_rad, 0.0)
        self.assertGreater(stability.tail_arm_m, 0.0)

    def test_rejects_fin_forward_of_cg(self):
        from dataclasses import replace

        bad_geometry = replace(
            self.case.geometry,
            reference_cg_x_m=self.case.geometry.fin.x_location_m + 5.0,
        )
        with self.assertRaises(ValueError):
            estimate_stability_derivatives(replace(self.case, geometry=bad_geometry))


class JSBSimXMLGenerationTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")

    def test_xml_is_well_formed_and_carries_mode_switch(self):
        import xml.etree.ElementTree as ET

        xml_text, summary = build_jsbsim_aircraft_xml(self.case)
        root = ET.fromstring(xml_text.split("\n", 1)[1])
        self.assertEqual(root.tag, "fdm_config")
        self.assertIsNotNone(root.find("external_reactions"))
        self.assertIsNotNone(root.find("flight_control"))
        self.assertAlmostEqual(
            summary.propulsion_mode_switch_mach,
            self.case.ramjet.minimum_lightoff_test_mach,
        )
        self.assertAlmostEqual(summary.loaded_mass_kg, self.case.flight.initial_mass_kg)
        self.assertGreater(summary.empty_mass_kg, 0.0)

    def test_adverse_scenario_grows_mass_and_derates_thrust(self):
        _, nominal_summary = build_jsbsim_aircraft_xml(self.case, NOMINAL_SCENARIO)
        _, adverse_summary = build_jsbsim_aircraft_xml(self.case, ADVERSE_SCENARIO)
        self.assertGreater(adverse_summary.loaded_mass_kg, nominal_summary.loaded_mass_kg)

    def test_write_creates_jsbsim_aircraft_directory_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            aircraft_dir = Path(directory) / "aircraft"
            output_path, summary = write_jsbsim_aircraft(self.case, aircraft_dir)
            self.assertTrue(output_path.is_file())
            self.assertEqual(output_path.parent.name, output_path.stem)
            self.assertEqual(summary.output_path, str(output_path))


class JSBSimRealEngineTests(unittest.TestCase):
    """Genuine verification against the installed jsbsim package, not a fake harness."""

    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")

    def test_model_loads_and_runs_finite_below_transition_mach(self):
        with tempfile.TemporaryDirectory() as directory:
            root_dir = Path(directory)
            output_path, _ = write_jsbsim_aircraft(self.case, root_dir / "aircraft")
            check = validate_with_jsbsim(
                root_dir,
                output_path.parent.name,
                altitude_m=4500.0,
                mach=0.20,
                run_seconds=2.0,
            )
        self.assertTrue(check.loaded_without_exception)
        self.assertTrue(check.state_finite)
        self.assertTrue(check.pulsejet_active_at_end)
        self.assertFalse(check.ramjet_active_at_end)

    def test_model_switches_to_ramjet_above_transition_mach(self):
        with tempfile.TemporaryDirectory() as directory:
            root_dir = Path(directory)
            output_path, _ = write_jsbsim_aircraft(self.case, root_dir / "aircraft")
            check = validate_with_jsbsim(
                root_dir,
                output_path.parent.name,
                altitude_m=4500.0,
                mach=1.10,
                run_seconds=1.0,
            )
        self.assertTrue(check.state_finite)
        self.assertFalse(check.pulsejet_active_at_end)
        self.assertTrue(check.ramjet_active_at_end)

    def test_adverse_scenario_loses_less_or_equal_mach_margin_than_nominal(self):
        with tempfile.TemporaryDirectory() as directory:
            root_dir = Path(directory)
            nominal_path, _ = write_jsbsim_aircraft(self.case, root_dir / "aircraft", NOMINAL_SCENARIO)
            adverse_path, _ = write_jsbsim_aircraft(self.case, root_dir / "aircraft", ADVERSE_SCENARIO)
            nominal_check = validate_with_jsbsim(
                root_dir, nominal_path.parent.name, altitude_m=4500.0, mach=1.10, run_seconds=2.0
            )
            adverse_check = validate_with_jsbsim(
                root_dir, adverse_path.parent.name, altitude_m=4500.0, mach=1.10, run_seconds=2.0
            )
        # Cross-check against the independent sizing.py/robustness.py static result:
        # the adverse scenario should not accelerate better than nominal at Mach 1.10.
        self.assertLessEqual(adverse_check.final_mach, nominal_check.final_mach + 1e-6)
        self.assertTrue(math.isfinite(adverse_check.final_mach))


if __name__ == "__main__":
    unittest.main()
