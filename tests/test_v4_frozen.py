"""Freeze the V4 design: the ramjet must keep lighting IN THE DIVE, the pitch
arcs must keep holding the 400 ft floor, and the body loads must not drift.

Same subprocess device as tests/test_v2_frozen.py and tests/test_v3_frozen.py:
CD0_FRONTAL is baked into simple_model.constants at IMPORT time, so once any
other test module has imported simple_model an in-process override silently
does nothing. scripts/fly_frozen_v4.py runs the flight in its own process with
the design's own CD0 and prints one JSON line.
"""
from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DESIGN_PATH = ROOT / "docs" / "v4_frozen" / "design.json"


def _fly() -> dict:
    proc = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "fly_frozen_v4.py")],
        capture_output=True, text=True, cwd=str(ROOT), check=True,
    )
    for line in proc.stdout.splitlines():
        if line.startswith("FLIGHT_JSON:"):
            return json.loads(line[len("FLIGHT_JSON:"):])
    raise AssertionError(f"no FLIGHT_JSON in output:\n{proc.stdout}\n{proc.stderr}")


class FrozenV4DesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.design = json.loads(DESIGN_PATH.read_text())
        cls.expected = cls.design["verified_mission"]
        cls.flight = _fly()

    def test_ramjet_still_lights_in_the_dive(self) -> None:
        """THE V4 requirement (user, 2026-08-13). A gate that fires in the
        climb or after the pull-out passes the Mach test and fails the
        mission -- see docs/v3_learnings_for_v4.md section 3.3."""
        self.assertTrue(self.flight["ramjet_lit_in_dive"])
        self.assertEqual(self.flight["ramjet_lightoff_mode"], "v3_dive")
        gate = self.design["trajectory"]["ramjet_gate_mach"]
        self.assertGreaterEqual(self.flight["ramjet_lightoff_mach"], gate)
        self.assertAlmostEqual(self.flight["ramjet_lightoff_mach"], gate, places=2)

    def test_mission_still_closes(self) -> None:
        self.assertTrue(self.flight["motor_cutoff_reached"])
        self.assertFalse(self.flight["stalled"])
        self.assertFalse(self.flight["hit_mass_floor"])
        self.assertFalse(self.flight["rule_violated"])
        self.assertGreaterEqual(self.flight["peak_mach"], 1.0)
        self.assertLessEqual(
            self.flight["fuel_burned_kg"],
            self.flight["tank_capacity_kg"] / 1.25 + 1e-9,
            "burned more than the tank minus the 25% reserve",
        )

    def test_pitch_arcs_hold_the_floor(self) -> None:
        """The pull-out arc must bottom out at or above the 400 ft floor.
        V3 had no arc at all -- it switched flight path angle in one
        timestep and never charged the altitude (learnings section 3.6)."""
        floor = self.design["trajectory"]["floor_altitude_m"]
        self.assertFalse(self.flight["floor_violated"])
        self.assertGreaterEqual(self.flight["min_powered_altitude_m"], floor - 1.0)
        for mode in ("v4_pushover", "v4_pullout"):
            self.assertIn(mode, self.flight["modes_flown"])
        self.assertGreater(self.flight["pushover_radius_m"], 0.0)
        self.assertGreater(self.flight["pullout_radius_m"], 0.0)

    def test_body_loads_have_not_drifted(self) -> None:
        for key, places in (("peak_load_n_total", 2), ("peak_load_n_yaw", 3),
                            ("peak_load_n_roll", 2)):
            self.assertAlmostEqual(self.flight[key], self.expected[key],
                                   places=places, msg=key)
        self.assertEqual(self.flight["peak_load_mode"], self.expected["peak_load_mode"])
        # The commanded nominal is 3 g; the limiter may pull harder to hold
        # the floor, but never past the design limit it was frozen under.
        self.assertLessEqual(self.flight["peak_load_n_total"], 4.0)

    def test_headline_numbers_reproduce(self) -> None:
        for key, places in (
            ("peak_mach", 3), ("dive_exit_mach", 3),
            ("peak_thrust_to_weight", 2), ("min_traverse_accel_g", 3),
            ("min_powered_accel_g", 3), ("min_powered_thrust_margin", 3),
            ("payload_margin_kg", 2), ("dry_mass_kg", 2),
            ("fuel_burned_kg", 3), ("stall_speed_m_per_s", 1),
            ("pushover_radius_m", 0), ("pullout_radius_m", 0),
            ("spiral_radius_m", 0),
        ):
            self.assertAlmostEqual(self.flight[key], self.expected[key],
                                   places=places, msg=key)

    def test_spiral_climb_is_what_makes_the_landing_work(self) -> None:
        """The climb is a spiral specifically so the vehicle can get home: a
        straight climb to the same 1100 m top cuts off ~17.5 km downrange and
        lands 4-6 km short. Guard both halves of that."""
        self.assertTrue(self.design["trajectory"]["spiral_climb"])
        self.assertTrue(self.flight["safe_landing"])
        self.assertGreater(self.flight["spiral_radius_m"], 0.0)
        self.assertLess(self.flight["lands_from_launch_m"], 50.0)

    def test_stall_speed_within_cap(self) -> None:
        self.assertLessEqual(self.flight["stall_speed_m_per_s"], 45.0)

    def test_throat_area_fraction_within_operability_cap(self) -> None:
        """The 214 mm body forced the throat down; if it ever drifts back up
        past 0.30 the pulsejet operability gate zeroes the engine outright."""
        v = self.design["vehicle"]
        fraction = (v["throat_diameter_m"] / v["diameter_m"]) ** 2
        self.assertLessEqual(
            fraction, self.design["constants_at_freeze"]["PULSEJET_MAX_THROAT_AREA_FRACTION"])

    def test_constants_have_not_drifted(self) -> None:
        import simple_model.constants as c
        import simple_model.flight_sim as fs
        frozen = self.design["constants_at_freeze"]
        self.assertEqual(c.PULSEJET_MAX_THROAT_AREA_FRACTION,
                         frozen["PULSEJET_MAX_THROAT_AREA_FRACTION"])
        self.assertEqual(c.MIN_POWERED_ACCELERATION_G,
                         frozen["MIN_POWERED_ACCELERATION_G"])
        self.assertEqual(c.MIN_POWERED_THRUST_MARGIN_FRACTION,
                         frozen["MIN_POWERED_THRUST_MARGIN_FRACTION"])
        self.assertAlmostEqual(fs.V3_FLOOR_ALTITUDE_M, frozen["V3_FLOOR_ALTITUDE_M"], places=2)
        self.assertEqual(fs.V4_PUSHOVER_LOAD_FACTOR, frozen["V4_PUSHOVER_LOAD_FACTOR"])
        self.assertEqual(fs.V4_PULLOUT_LOAD_FACTOR, frozen["V4_PULLOUT_LOAD_FACTOR"])
        self.assertEqual(fs.V4_PULLOUT_MAX_LOAD_FACTOR, frozen["V4_PULLOUT_MAX_LOAD_FACTOR"])
        self.assertEqual(fs.V4_SPIRAL_BANK_DEG, frozen["V4_SPIRAL_BANK_DEG"])
        self.assertEqual(fs.V4_FLOOR_TOLERANCE_M, frozen["V4_FLOOR_TOLERANCE_M"])

    def test_known_gate_failures_are_still_recorded_honestly(self) -> None:
        """V4 does NOT close the traverse-acceleration, powered-acceleration
        or thrust-margin gates, and the design file must keep saying so. This
        test exists to stop a future edit quietly deleting the admission."""
        gates = self.design["gates"]
        self.assertIn("FAIL", gates["min_traverse_accel_0.25g"])
        self.assertIn("FAIL", gates["min_powered_accel_positive"])
        self.assertIn("FAIL", gates["min_powered_thrust_margin_1.15"])
        self.assertLess(self.flight["min_traverse_accel_g"], 0.25)
        self.assertLess(self.flight["min_powered_thrust_margin"], 1.15)


if __name__ == "__main__":
    unittest.main()
