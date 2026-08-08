import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.mission_trade import mission_trade_sweep
from douglas_dart.performance_maps import RectilinearEngineMap


ROOT = Path(__file__).resolve().parents[1]


class MissionTradeTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        self.engine_map = RectilinearEngineMap(
            altitudes_m=(0.0, 8000.0),
            mach_values=(0.0, 1.2),
            net_thrust_rows_n=((200.0, 200.0), (200.0, 200.0)),
            fuel_flow_rows_kg_per_s=((0.02, 0.02), (0.02, 0.02)),
        )

    def test_sweep_preserves_explicit_fuel_split_and_gate_labels(self):
        points = mission_trade_sweep(
            self.case,
            self.engine_map,
            release_speeds_m_per_s=(40.5,),
            top_of_climb_altitudes_m=(6000.0,),
            climb_flight_path_angles_deg=(45.0,),
            dive_flight_path_angles_deg=(-20.0,),
            ramjet_fuel_allocations_kg=(0.75,),
            ramjet_handoff_mach=0.8,
            allow_forced_ramjet_below_self_sustaining=True,
        )
        self.assertEqual(len(points), 1)
        point = points[0]
        self.assertAlmostEqual(point.pulsejet_fuel_allocation_kg, 3.05)
        self.assertEqual(point.ramjet_fuel_allocation_kg, 0.75)
        self.assertTrue(point.forced_operability_remains_unvalidated)
        self.assertTrue(point.spillage_drag_remains_unvalidated)
        self.assertFalse(point.launch_lift_closes)

    def test_ramjet_allocation_cannot_exceed_loaded_fuel(self):
        with self.assertRaises(ValueError):
            mission_trade_sweep(
                self.case,
                self.engine_map,
                release_speeds_m_per_s=(90.0,),
                top_of_climb_altitudes_m=(6000.0,),
                climb_flight_path_angles_deg=(45.0,),
                dive_flight_path_angles_deg=(-20.0,),
                ramjet_fuel_allocations_kg=(4.0,),
                ramjet_handoff_mach=1.1,
                allow_forced_ramjet_below_self_sustaining=False,
            )

    def test_loaded_fuel_trade_holds_dry_mass_fixed(self):
        point = mission_trade_sweep(
            self.case,
            self.engine_map,
            release_speeds_m_per_s=(90.0,),
            top_of_climb_altitudes_m=(6000.0,),
            climb_flight_path_angles_deg=(45.0,),
            dive_flight_path_angles_deg=(-20.0,),
            loaded_fuel_masses_kg=(5.0,),
            ramjet_fuel_allocations_kg=(0.75,),
            ramjet_handoff_mach=0.8,
            allow_forced_ramjet_below_self_sustaining=True,
        )[0]
        configured_dry_mass_kg = (
            self.case.flight.initial_mass_kg
            - self.case.mission.loaded_fuel_mass_kg
        )
        self.assertAlmostEqual(point.initial_mass_kg, configured_dry_mass_kg + 5.0)
        self.assertAlmostEqual(point.loaded_fuel_mass_kg, 5.0)
        self.assertAlmostEqual(point.pulsejet_fuel_allocation_kg, 4.25)

    def test_loaded_fuel_trade_rejects_takeoff_mass_violation(self):
        with self.assertRaisesRegex(ValueError, "maximum takeoff mass"):
            mission_trade_sweep(
                self.case,
                self.engine_map,
                release_speeds_m_per_s=(90.0,),
                top_of_climb_altitudes_m=(6000.0,),
                climb_flight_path_angles_deg=(45.0,),
                dive_flight_path_angles_deg=(-20.0,),
                loaded_fuel_masses_kg=(8.0,),
                ramjet_fuel_allocations_kg=(0.75,),
                ramjet_handoff_mach=0.8,
                allow_forced_ramjet_below_self_sustaining=True,
            )


if __name__ == "__main__":
    unittest.main()
