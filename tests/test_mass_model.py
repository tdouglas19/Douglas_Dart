import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.mass_model import calibrate_mass_model, evaluate_parametric_mass

ROOT = Path(__file__).resolve().parents[1]


class MassModelTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "shared_nozzle_candidate_b.yaml")
        self.calibration = calibrate_mass_model(
            self.case, ROOT / "configs" / "robustness_candidate_b.yaml"
        )

    def test_calibration_coefficients_are_positive(self):
        self.assertGreater(self.calibration.body_skin_areal_density_kg_per_m2, 0.0)
        self.assertGreater(self.calibration.lifting_surface_areal_density_kg_per_m2, 0.0)
        self.assertGreater(
            self.calibration.propulsion_hardware_mass_per_throat_area_kg_per_m2, 0.0
        )
        self.assertGreater(self.calibration.selector_mass_per_intake_area_kg_per_m2, 0.0)
        self.assertGreaterEqual(self.calibration.fixed_systems_mass_kg, 0.0)

    def test_calibrated_model_exactly_reproduces_configured_baseline_empty_mass(self):
        breakdown = evaluate_parametric_mass(self.case, self.calibration)
        configured_empty_mass_kg = (
            self.case.flight.initial_mass_kg - self.case.mission.loaded_fuel_mass_kg
        )
        self.assertAlmostEqual(breakdown.empty_mass_kg, configured_empty_mass_kg, places=6)

    def test_larger_body_increases_empty_mass(self):
        baseline = evaluate_parametric_mass(self.case, self.calibration)
        larger = evaluate_parametric_mass(
            self.case,
            self.calibration,
            body_diameter_m=self.case.vehicle.body_diameter_m * 1.2,
            body_length_m=self.case.vehicle.body_length_m * 1.2,
        )
        self.assertGreater(larger.empty_mass_kg, baseline.empty_mass_kg)
        self.assertGreater(larger.body_skin_mass_kg, baseline.body_skin_mass_kg)

    def test_larger_throat_increases_propulsion_hardware_mass_only(self):
        baseline = evaluate_parametric_mass(self.case, self.calibration)
        larger_throat = evaluate_parametric_mass(
            self.case,
            self.calibration,
            throat_diameter_m=self.case.nozzle.throat_diameter_m * 1.3,
        )
        self.assertGreater(
            larger_throat.propulsion_hardware_mass_kg, baseline.propulsion_hardware_mass_kg
        )
        self.assertAlmostEqual(
            larger_throat.body_skin_mass_kg, baseline.body_skin_mass_kg, places=9
        )
        self.assertAlmostEqual(
            larger_throat.lifting_surface_mass_kg, baseline.lifting_surface_mass_kg, places=9
        )

    def test_smaller_body_reduces_empty_mass_but_stays_positive(self):
        baseline = evaluate_parametric_mass(self.case, self.calibration)
        smaller = evaluate_parametric_mass(
            self.case,
            self.calibration,
            body_diameter_m=self.case.vehicle.body_diameter_m * 0.8,
            body_length_m=self.case.vehicle.body_length_m * 0.8,
        )
        self.assertLess(smaller.empty_mass_kg, baseline.empty_mass_kg)
        self.assertGreater(smaller.empty_mass_kg, 0.0)


if __name__ == "__main__":
    unittest.main()
