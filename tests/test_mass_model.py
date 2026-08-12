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

    def test_calibrated_model_adds_chamber_wall_mass_on_top_of_configured_baseline(self):
        # docs/design_convergence.md "Pulsejet chamber-wall mass": this term
        # is deliberately NOT netted against the existing
        # shared_engine_body_combustor_nozzle budget line (that line has no
        # internal breakdown to split from, and at this baseline's 0.025 m^3
        # chamber volume the thin-shell estimate alone exceeds the entire
        # $4.20 kg line item). So the model's total empty mass at baseline
        # geometry is now honestly higher than the previously configured
        # value, by exactly this new term -- not an approximation.
        breakdown = evaluate_parametric_mass(self.case, self.calibration)
        configured_empty_mass_kg = (
            self.case.flight.initial_mass_kg - self.case.mission.loaded_fuel_mass_kg
        )
        self.assertGreater(breakdown.pulsejet_chamber_wall_mass_kg, 0.0)
        self.assertAlmostEqual(
            breakdown.empty_mass_kg,
            configured_empty_mass_kg + breakdown.pulsejet_chamber_wall_mass_kg,
            places=6,
        )

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

    def test_smaller_body_reduces_body_skin_mass_and_stays_positive(self):
        # Total empty_mass_kg is NOT asserted to decrease here: shrinking
        # body_diameter_m for a *fixed* chamber_volume_m3 forces a longer,
        # thinner chamber tube (wall_area = 4*V/D, inversely proportional to
        # diameter -- see test_smaller_body_grows_chamber_wall_mass_for_fixed_chamber_volume
        # below), which can outweigh the body-skin reduction tested here.
        # That is the correct, intended behavior this whole term exists for
        # (docs/design_convergence.md): the search should not be able to
        # shrink the body toward the selector's own floor for free anymore.
        baseline = evaluate_parametric_mass(self.case, self.calibration)
        smaller = evaluate_parametric_mass(
            self.case,
            self.calibration,
            body_diameter_m=self.case.vehicle.body_diameter_m * 0.8,
            body_length_m=self.case.vehicle.body_length_m * 0.8,
        )
        self.assertLess(smaller.body_skin_mass_kg, baseline.body_skin_mass_kg)
        self.assertGreater(smaller.empty_mass_kg, 0.0)

    def test_smaller_body_grows_chamber_wall_mass_for_fixed_chamber_volume(self):
        # A real, physically-correct interaction, not an artifact: a smaller
        # body diameter forces a longer/thinner chamber tube for the same
        # chamber_volume_m3, which has *more* wall area, not less -- this is
        # the mechanism that now discourages the optimizer from shrinking
        # body_diameter_m toward the fixed-intake floor for free (every prior
        # design-optimize run in docs/design_convergence.md did exactly
        # that, with zero mass consequence, before this term existed).
        baseline = evaluate_parametric_mass(self.case, self.calibration)
        smaller = evaluate_parametric_mass(
            self.case,
            self.calibration,
            body_diameter_m=self.case.vehicle.body_diameter_m * 0.8,
        )
        self.assertGreater(
            smaller.pulsejet_chamber_wall_mass_kg, baseline.pulsejet_chamber_wall_mass_kg
        )

    def test_chamber_volume_override_scales_chamber_wall_mass_only(self):
        baseline = evaluate_parametric_mass(self.case, self.calibration)
        larger_chamber = evaluate_parametric_mass(
            self.case,
            self.calibration,
            chamber_volume_m3=self.case.pulsejet.chamber_volume_m3 * 1.5,
        )
        self.assertGreater(
            larger_chamber.pulsejet_chamber_wall_mass_kg, baseline.pulsejet_chamber_wall_mass_kg
        )
        self.assertAlmostEqual(
            larger_chamber.body_skin_mass_kg, baseline.body_skin_mass_kg, places=9
        )
        self.assertAlmostEqual(
            larger_chamber.propulsion_hardware_mass_kg,
            baseline.propulsion_hardware_mass_kg,
            places=9,
        )


if __name__ == "__main__":
    unittest.main()
