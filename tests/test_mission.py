import unittest
from pathlib import Path

from douglas_dart.atmosphere import standard_atmosphere
from douglas_dart.config import load_reference_case
from douglas_dart.flight import PointMassDerivative, PointMassState
from douglas_dart.mission import MissionPhase, MissionPolicy, MissionSimulator
from douglas_dart.performance_maps import RectilinearEngineMap


ROOT = Path(__file__).resolve().parents[1]


def constant_pulsejet_map(thrust_n: float = 200.0, fuel_flow: float = 0.02):
    return RectilinearEngineMap(
        altitudes_m=(0.0, 8000.0),
        mach_values=(0.0, 1.2),
        net_thrust_rows_n=((thrust_n, thrust_n), (thrust_n, thrust_n)),
        fuel_flow_rows_kg_per_s=((fuel_flow, fuel_flow), (fuel_flow, fuel_flow)),
    )


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )

    def test_current_release_condition_fails_lift_screen_and_contacts_ground(self):
        result = MissionSimulator(
            self.case,
            constant_pulsejet_map(),
        ).run(maximum_duration_s=1.0)
        self.assertFalse(result.launch_screen.lift_closes_at_release)
        self.assertEqual(result.summary.termination_reason, "ground_contact")
        self.assertFalse(result.summary.full_mission_numerically_closes)
        self.assertAlmostEqual(
            self.case.flight.initial_mass_kg - result.summary.total_fuel_used_kg,
            result.samples[-1].mass_kg,
        )

    def test_subsonic_handoff_requires_explicit_forced_operation(self):
        with self.assertRaisesRegex(ValueError, "forced-operation"):
            MissionPolicy.from_case(
                self.case,
                ramjet_handoff_mach=0.8,
            )
        policy = MissionPolicy.from_case(
            self.case,
            ramjet_handoff_mach=0.8,
            allow_forced_ramjet_below_self_sustaining=True,
        )
        self.assertTrue(policy.allow_forced_ramjet_below_self_sustaining)

    def test_ramjet_acceleration_uses_full_throttle(self):
        policy = MissionPolicy.from_case(
            self.case,
            ramjet_handoff_mach=0.8,
            allow_forced_ramjet_below_self_sustaining=True,
        )
        simulator = MissionSimulator(
            self.case,
            constant_pulsejet_map(),
            policy=policy,
        )
        atmosphere = standard_atmosphere(
            self.case.mission.speed_run_altitude_msl_m
        )
        state = PointMassState(
            downrange_m=0.0,
            altitude_m=self.case.mission.speed_run_altitude_msl_m,
            speed_m_per_s=0.8 * atmosphere.speed_of_sound_m_per_s,
            flight_path_angle_rad=-0.1,
            mass_kg=20.0,
        )
        acceleration = simulator._control(
            state,
            MissionPhase.RAMJET_ACCELERATION,
        )
        run = simulator._control(state, MissionPhase.RAMJET_RUN)
        self.assertEqual(acceleration.throttle_fraction, 1.0)
        self.assertLessEqual(run.throttle_fraction, 1.0)

    def test_midpoint_advance_is_exact_for_constant_derivative(self):
        state = PointMassState(0.0, 1000.0, 100.0, 0.1, 20.0)
        derivative = PointMassDerivative(99.0, 10.0, 2.0, -0.01, -0.1, 0.0, 0.0)
        advanced = MissionSimulator._advance(state, derivative, 0.5)
        self.assertAlmostEqual(advanced.downrange_m, 49.5)
        self.assertAlmostEqual(advanced.altitude_m, 1005.0)
        self.assertAlmostEqual(advanced.speed_m_per_s, 101.0)
        self.assertAlmostEqual(advanced.flight_path_angle_rad, 0.095)
        self.assertAlmostEqual(advanced.mass_kg, 19.95)


if __name__ == "__main__":
    unittest.main()
