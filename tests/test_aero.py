import unittest
from pathlib import Path

from douglas_dart.aero import (
    BudgetAeroModel,
    launch_lift_screen,
    ramjet_spillage_momentum_scale_n,
)
from douglas_dart.atmosphere import standard_atmosphere
from douglas_dart.config import load_reference_case
from douglas_dart.flight import PointMassState
from douglas_dart.ramjet import evaluate_ramjet


ROOT = Path(__file__).resolve().parents[1]


class AeroTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        self.model = BudgetAeroModel(self.case)

    def test_peak_mach_zero_lift_drag_matches_scaled_budget(self):
        altitude_m = self.case.mission.speed_run_altitude_msl_m
        atmosphere = standard_atmosphere(altitude_m)
        state = PointMassState(
            downrange_m=0.0,
            altitude_m=altitude_m,
            speed_m_per_s=(
                self.case.mission.peak_mach * atmosphere.speed_of_sound_m_per_s
            ),
            flight_path_angle_rad=0.0,
            mass_kg=self.case.flight.initial_mass_kg,
        )
        forces = self.model.forces(state, angle_of_attack_rad=0.0)
        expected_drag_area_m2 = (
            self.case.vehicle.peak_mach_drag_area_ceiling_m2
            * (
                self.case.vehicle.body_diameter_m
                / self.case.vehicle.drag_area_reference_body_diameter_m
            )
            ** 2
        )
        self.assertAlmostEqual(forces.parasitic_drag_area_m2, expected_drag_area_m2)
        self.assertEqual(forces.induced_drag_area_m2, 0.0)

    def test_launch_screen_exposes_current_release_mismatch(self):
        midpoint_speed_m_per_s = 0.5 * (
            self.case.mission.sled_release_speed_min_m_per_s
            + self.case.mission.sled_release_speed_max_m_per_s
        )
        screen = launch_lift_screen(self.case, midpoint_speed_m_per_s)
        self.assertFalse(screen.lift_closes_at_release)
        self.assertGreater(screen.required_reference_area_m2, 0.4)
        self.assertGreater(screen.minimum_level_flight_speed_m_per_s, 88.0)
        self.assertTrue(launch_lift_screen(self.case, 90.0).lift_closes_at_release)

    def test_candidate_b_reference_release_closes_but_sled_target_does_not(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
        )
        reference = launch_lift_screen(
            case,
            case.mission_simulation.reference_release_speed_m_per_s,
        )
        sled_limit = launch_lift_screen(
            case,
            case.mission.sled_release_speed_max_m_per_s,
        )
        self.assertTrue(reference.lift_closes_at_release)
        self.assertFalse(sled_limit.lift_closes_at_release)
        self.assertAlmostEqual(
            reference.minimum_level_flight_speed_m_per_s,
            95.7143,
            places=3,
        )
        self.assertLess(sled_limit.configured_to_required_area_ratio, 0.20)

    def test_ramjet_spillage_momentum_scale_uses_only_uncaptured_air(self):
        result = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.mission.speed_run_altitude_msl_m,
            self.case.mission.peak_mach,
        )
        atmosphere = standard_atmosphere(self.case.mission.speed_run_altitude_msl_m)
        expected = (
            result.potential_captured_air_mass_flow_kg_per_s
            - result.air_mass_flow_kg_per_s
        ) * self.case.mission.peak_mach * atmosphere.speed_of_sound_m_per_s
        self.assertAlmostEqual(
            ramjet_spillage_momentum_scale_n(
                result,
                self.case.mission.speed_run_altitude_msl_m,
            ),
            expected,
        )


if __name__ == "__main__":
    unittest.main()
