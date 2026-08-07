import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import (
    NOMINAL,
    PULSEJET_MODE,
    RAMJET_MODE,
    PropulsionScenario,
    build_propulsion_map,
    evaluate_propulsion_map_point,
)

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
        point = evaluate_propulsion_map_point(self.case, 0.5, 0.0, PULSEJET_MODE)
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
        nominal_pj = evaluate_propulsion_map_point(self.case, 0.2, 0.0, PULSEJET_MODE, scenario=NOMINAL)
        derated_pj = evaluate_propulsion_map_point(self.case, 0.2, 0.0, PULSEJET_MODE, scenario=derated)
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
            self.case, mach_values=(0.2, 0.5), altitude_values=(0.0,), modes=(PULSEJET_MODE,)
        )
        self.assertEqual(len(points), 2)

    def test_ramjet_lightoff_status_reflects_configured_gates(self):
        below = evaluate_propulsion_map_point(self.case, 0.5, 4500.0, RAMJET_MODE)
        self.assertEqual(below.lightoff_status, "below_lightoff_test_mach")
        above = evaluate_propulsion_map_point(self.case, 1.1, 4500.0, RAMJET_MODE)
        self.assertEqual(above.lightoff_status, "at_or_above_self_sustaining_mach")


if __name__ == "__main__":
    unittest.main()
