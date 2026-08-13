"""The frozen V3 climb-dive design must stay re-flyable and keep its
headline numbers as the models keep changing.

Same contract and same mechanics as tests/test_v2_frozen.py: the flight runs
in a SUBPROCESS (scripts/fly_frozen_v3.py) because CD0 is baked into
simple_model.constants at import time and re-exported by drag/flight_sim, so
an in-process override silently does nothing once any other test module has
imported simple_model.
"""
from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FROZEN = ROOT / "docs" / "v3_frozen" / "design.json"
DESIGN = json.loads(FROZEN.read_text())


def _fly_in_subprocess() -> dict:
    proc = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "fly_frozen_v3.py")],
        cwd=ROOT, capture_output=True, text=True, timeout=600,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"frozen V3 re-fly failed (exit {proc.returncode}):\n{proc.stdout}\n{proc.stderr}")
    for line in proc.stdout.splitlines():
        if line.startswith("FLIGHT_JSON:"):
            return json.loads(line[len("FLIGHT_JSON:"):])
    raise AssertionError(f"no FLIGHT_JSON in output:\n{proc.stdout}\n{proc.stderr}")


class FrozenV3DesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.flight = _fly_in_subprocess()

    def test_constants_have_not_drifted(self):
        from simple_model import constants as C
        from simple_model.flight_sim import V3_FLOOR_ALTITUDE_M
        frozen = DESIGN["constants_at_freeze"]
        for name in ("MIN_POWERED_ACCELERATION_G",
                     "MIN_POWERED_THRUST_MARGIN_FRACTION",
                     "PULSEJET_MAX_THROAT_AREA_FRACTION",
                     "RAMJET_MIN_LIGHTOFF_MACH"):
            self.assertAlmostEqual(getattr(C, name), frozen[name], places=9,
                                   msg=f"{name} drifted since the V3 freeze")
        self.assertAlmostEqual(V3_FLOOR_ALTITUDE_M,
                               frozen["V3_FLOOR_ALTITUDE_M"], places=9,
                               msg="the 400 ft hard floor moved")

    def test_mission_still_closes(self):
        f = self.flight
        self.assertTrue(f["motor_cutoff_reached"])
        self.assertTrue(f["safe_landing"])
        self.assertFalse(f["stalled"])
        self.assertFalse(f["hit_mass_floor"])

    def test_headline_numbers_reproduce(self):
        f, v = self.flight, DESIGN["verified_mission"]
        for key, places in (("peak_thrust_to_weight", 2),
                            ("min_traverse_accel_g", 3),
                            ("min_traverse_accel_mach", 2),
                            ("min_powered_accel_g", 3),
                            ("min_powered_thrust_margin", 3)):
            self.assertAlmostEqual(f[key], v[key], places=places, msg=key)
        self.assertAlmostEqual(f["climb_dive_top_altitude_m"],
                               v["climb_dive_top_altitude_m"], places=2)

    def test_climb_dive_phases_and_floor_hold(self):
        f = self.flight
        modes = f["modes_ordered"]
        for phase in ("v3_climb", "v3_dive", "drag_strip"):
            self.assertIn(phase, modes, phase)
        self.assertLess(modes.index("v3_climb"), modes.index("v3_dive"))
        self.assertLess(modes.index("v3_dive"), modes.index("drag_strip"))
        # the 400 ft hard floor is the user's constraint, not a soft target
        self.assertGreaterEqual(
            f["dive_min_altitude_m"],
            DESIGN["constants_at_freeze"]["V3_FLOOR_ALTITUDE_M"] - 25.0)

    def test_rule_is_satisfied(self):
        """gamma >= 0 from M 0.80 through cutoff."""
        self.assertFalse(self.flight["rule_violated"])

    def test_still_returns_to_launch(self):
        f = self.flight
        self.assertLess(abs(f["final_distance_m"]), 50.0)
        self.assertGreater(f["max_distance_m"], 5_000.0)

    def test_gates_it_was_selected_under_still_pass(self):
        from simple_model.constants import (MIN_POWERED_ACCELERATION_G,
                                            MIN_POWERED_THRUST_MARGIN_FRACTION)
        f = self.flight
        self.assertGreaterEqual(f["min_traverse_accel_g"],
                                MIN_POWERED_ACCELERATION_G)
        self.assertGreater(f["min_powered_accel_g"], 0.0)
        self.assertGreaterEqual(f["min_powered_thrust_margin"],
                                1.0 + MIN_POWERED_THRUST_MARGIN_FRACTION)


if __name__ == "__main__":
    unittest.main()
