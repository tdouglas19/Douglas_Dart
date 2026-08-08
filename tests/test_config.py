import unittest
from dataclasses import replace
from pathlib import Path

from douglas_dart.config import SelectorConfig, load_reference_case


ROOT = Path(__file__).resolve().parents[1]


class ConfigTests(unittest.TestCase):
    def test_user_half_area_requirement(self):
        selector = SelectorConfig(0.3, 0.5, 0.8, 0.99, 0.9)
        self.assertAlmostEqual(selector.available_area_m2, 0.5 * selector.circular_area_m2)

    def test_more_than_half_open_is_rejected(self):
        with self.assertRaises(ValueError):
            SelectorConfig(0.3, 0.6, 0.8, 0.99, 0.9)

    def test_reference_case_loads(self):
        case = load_reference_case(ROOT / "configs" / "reference_case.yaml")
        self.assertEqual(case.fuel.key, "jet_a_reference")
        self.assertGreater(case.nozzle.exit_area_m2, case.nozzle.throat_area_m2)
        self.assertEqual(case.mission.peak_mach, 1.10)
        self.assertEqual(case.mission.ramjet_speed_run_fuel_budget_kg, 1.40)
        self.assertEqual(case.vehicle.body_diameter_m, 0.195)
        self.assertEqual(case.selector.circular_intake_diameter_m, 0.195)
        self.assertEqual(case.requirements.maximum_takeoff_mass_kg, 25.0)
        self.assertEqual(case.requirements.minimum_time_above_mach_one_s, 5.0)
        self.assertEqual(case.geometry.api_version, "3.51.2")
        self.assertEqual(case.geometry.fin_count, 4)
        self.assertGreater(
            case.geometry.lifting_surface.exposed_area_per_surface_m2,
            0.0,
        )

    def test_shared_nozzle_candidate_keeps_requirements_separate(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        self.assertEqual(case.name, "shared_nozzle_candidate_a_unvalidated")
        self.assertEqual(case.nozzle.throat_diameter_m, 0.130)
        self.assertEqual(case.nozzle.exit_to_throat_area_ratio, 1.05)
        self.assertLess(case.flight.initial_mass_kg, case.requirements.maximum_takeoff_mass_kg)
        self.assertEqual(
            case.requirements.pulsejet_rule_status,
            "confirmed_eligible_boomsupersonic_com_prize",
        )
        self.assertTrue(case.requirements.transonic_no_altitude_loss_required)
        self.assertAlmostEqual(case.requirements.transonic_regime_start_mach, 0.80)

    def test_candidate_b_keeps_closure_inputs_distinct_from_sled_target(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
        )
        self.assertEqual(case.vehicle.body_diameter_m, 0.210)
        self.assertEqual(case.nozzle.throat_diameter_m, 0.170)
        self.assertEqual(case.flight.initial_mass_kg, 24.6)
        self.assertEqual(case.mission.loaded_fuel_mass_kg, 7.40)
        self.assertEqual(case.mission.ramjet_speed_run_fuel_budget_kg, 1.20)
        self.assertEqual(
            case.mission_simulation.reference_release_speed_m_per_s,
            100.0,
        )
        self.assertGreater(
            case.mission_simulation.reference_release_speed_m_per_s,
            case.mission.sled_release_speed_max_m_per_s,
        )
        self.assertEqual(case.mission_simulation.pulsejet_map_mach_values[0], 0.0)
        self.assertTrue(
            case.mission_simulation.allow_forced_ramjet_below_self_sustaining
        )

    def test_flight_and_vspaero_reference_area_cannot_silently_diverge(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        with self.assertRaisesRegex(ValueError, "flight reference area"):
            replace(
                case,
                flight=replace(case.flight, reference_area_m2=0.18),
            )


if __name__ == "__main__":
    unittest.main()
