"""The frozen V2 optimal design must stay re-flyable and keep its headline
numbers as flight_sim.py grows new trajectory modes (V3 and beyond).

This is the test that actually protects docs/v2_frozen/ -- the copied report
files there are a record, not a guard. If a V3 change alters V2's numbers,
that is a regression in the shared physics, not a V3 feature.

The flight runs in a SUBPROCESS (scripts/fly_frozen_v2.py). CD0 is baked
into simple_model.constants at import time and re-exported by drag and
flight_sim, so once any other test module has imported simple_model an
in-process override silently does nothing -- which made an earlier version
of this file pass or fail purely on test module import order.
"""
from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FROZEN = ROOT / "docs" / "v2_frozen" / "design.json"
DESIGN = json.loads(FROZEN.read_text())


def _fly_in_subprocess() -> dict:
    proc = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "fly_frozen_v2.py")],
        cwd=ROOT, capture_output=True, text=True, timeout=600,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"frozen V2 re-fly failed (exit {proc.returncode}):\n{proc.stdout}\n{proc.stderr}")
    for line in proc.stdout.splitlines():
        if line.startswith("FLIGHT_JSON:"):
            return json.loads(line[len("FLIGHT_JSON:"):])
    raise AssertionError(f"no FLIGHT_JSON in output:\n{proc.stdout}\n{proc.stderr}")


class FrozenV2DesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.flight = _fly_in_subprocess()

    def test_constants_have_not_drifted(self):
        from simple_model import constants as C
        frozen = DESIGN["constants_at_freeze"]
        # CD0 is overridden per-run, so only the baked-in physics/gates are
        # asserted here -- a change to any of these invalidates the V2 numbers
        for name in ("MIN_POWERED_ACCELERATION_G",
                     "MIN_POWERED_THRUST_MARGIN_FRACTION",
                     "PULSEJET_MAX_THROAT_AREA_FRACTION",
                     "RAMJET_MIN_LIGHTOFF_MACH"):
            self.assertAlmostEqual(getattr(C, name), frozen[name], places=9,
                                   msg=f"{name} drifted since the V2 freeze")

    def test_mission_still_closes(self):
        f = self.flight
        self.assertTrue(f["motor_cutoff_reached"])
        self.assertTrue(f["safe_landing"])
        self.assertFalse(f["stalled"])
        self.assertFalse(f["hit_mass_floor"])

    def test_headline_numbers_reproduce(self):
        f, v = self.flight, DESIGN["verified_mission"]
        self.assertAlmostEqual(f["min_powered_accel_g"],
                               v["min_powered_accel_g"], places=2)
        self.assertAlmostEqual(f["min_accel_mach"], v["min_accel_mach"], places=2)
        self.assertAlmostEqual(f["min_powered_thrust_margin"],
                               v["min_powered_thrust_margin"], places=2)
        self.assertAlmostEqual(f["peak_thrust_to_weight"],
                               v["peak_thrust_to_weight"], places=2)

    def test_traverse_and_powered_accel_coincide_without_v3(self):
        """V2 flies no commanded climb, so the V3 traverse metric must be
        identical to the all-powered one -- i.e. V3 added a measurement, not
        a change to what V2 experiences."""
        self.assertAlmostEqual(self.flight["min_traverse_accel_g"],
                               self.flight["min_powered_accel_g"], places=9)

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
        self.assertGreaterEqual(f["min_powered_thrust_margin"],
                                1.0 + MIN_POWERED_THRUST_MARGIN_FRACTION)


if __name__ == "__main__":
    unittest.main()
