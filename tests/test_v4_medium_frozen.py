"""Lock docs/v4_medium_frozen/design.json -- the medium_model V4 result.

WHAT THIS TEST CAN AND CANNOT DO, stated plainly because the gap is real.

The frozen flight uses the first-principles engines at CONFIRM resolution, so
re-flying it costs ~35 minutes. No test suite can carry that, and there is no
cheaper flight of this design left in the freeze to stand in for it.

So by default this test does NOT re-fly. It checks the freeze against the
per-step state trace that flight wrote (out_medium_model/v4_rungc_lap_states
.json.gz), recomputing the headline numbers from the raw states. That catches a
hand-edited freeze, a stale record, or a summary that disagrees with its own
flight -- it does NOT catch a code change that would make the vehicle fly
differently. Only the full re-fly does that:

    DOUGLAS_DART_REFLY_FP_RUNGS=1 python -m unittest tests.test_v4_medium_frozen

CI-equivalent protection for the trajectory machinery comes from
tests/test_v2_frozen.py, test_v3_frozen.py, test_v4_frozen.py and
test_medium_model_v2_parity.py, which are cheap and do re-fly.
"""
from __future__ import annotations

import gzip
import json
import math
import os
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FREEZE = ROOT / "docs" / "v4_medium_frozen" / "design.json"
FLYER = ROOT / "scripts" / "fly_frozen_v4_medium.py"
TRACE = ROOT / "out_medium_model" / "v4_rungc_lap_states.json.gz"

G0 = 9.80665
# 4 dp recorded; 1e-3 absolute survives a last-digit float change and still
# catches any real behavioural drift.
TOL = 1e-3

_FLOAT_FIELDS = [
    "peak_mach", "dive_exit_mach", "peak_tw", "traverse_g", "powered_g",
    "margin", "peak_load_n_total", "peak_load_n_yaw", "peak_load_n_roll",
    "pushover_radius_m", "pullout_radius_m", "spiral_radius_m",
    "min_powered_altitude_m", "fuel_kg", "burn_cap_kg", "flight_s",
    "lands_from_launch_m", "ramjet_lightoff_mach",
    "ramjet_lightoff_altitude_m", "ramjet_lightoff_time_s",
]
_EXACT_FIELDS = [
    "cutoff", "ramjet_lightoff_mode", "ramjet_lit_in_dive",
    "ramjet_light_refused", "peak_load_mode", "floor_violated",
    "rule_violated", "stalled", "safe_landing",
]
_POWERED_MODES = {"v3_climb", "v4_pushover", "v3_dive", "v4_pullout",
                  "drag_strip"}


class V4MediumFrozenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.freeze = json.loads(FREEZE.read_text())
        cls.verified = cls.freeze["verified_mission"]
        cls.states = None
        if TRACE.exists():
            with gzip.open(TRACE, "rt", encoding="utf-8") as fh:
                cls.states = json.load(fh)["states"]

    # -- structure of the record ------------------------------------------
    def test_the_freeze_states_its_claims(self):
        v = self.verified
        gate = self.freeze["trajectory"]["ramjet_gate_mach"]
        self.assertTrue(self.freeze["trajectory"]["ramjet_light_at_pullout"])
        # The whole result: it lights BELOW the gate, in the pull-out rather
        # than the dive, and the mission closes anyway.
        self.assertIsNotNone(v["ramjet_lightoff_mach"])
        self.assertLess(v["ramjet_lightoff_mach"], gate)
        self.assertEqual(v["ramjet_lightoff_mode"], "v4_pullout")
        self.assertFalse(v["ramjet_lit_in_dive"])
        self.assertFalse(v["ramjet_light_refused"])
        self.assertTrue(v["cutoff"])
        self.assertGreaterEqual(v["peak_mach"], 1.1)
        self.assertLessEqual(v["fuel_kg"], v["burn_cap_kg"])

    def test_the_fidelity_is_the_one_claimed(self):
        f = self.freeze["fidelity"]
        self.assertEqual(f["drag_model"], "buildup")
        self.assertEqual(f["propulsion"], "FP")
        self.assertEqual(f["n_cells"], 324)
        self.assertEqual(f["chamber_diameter_fraction"], 1.0)

    def test_the_structural_warning_is_backed_by_the_phase_table(self):
        """The freeze claims the return loop out-loads every powered phase.
        That claim is what anyone sizing structure will act on, so it is
        checked against the recorded phases rather than trusted."""
        phases = {q["mode"]: q["peak_load_n"] for q in self.freeze["phases"]}
        self.assertIn("loop", phases)
        worst_powered = max(v for m, v in phases.items()
                            if m in _POWERED_MODES)
        self.assertGreater(phases["loop"], worst_powered)
        self.assertAlmostEqual(worst_powered,
                               self.verified["peak_load_n_total"], delta=0.02)

    # -- the record vs the flight it came from -----------------------------
    @unittest.skipUnless(TRACE.exists(), f"{TRACE.name} not present")
    def test_recorded_numbers_match_the_stored_state_trace(self):
        """Recompute the headline numbers from the raw per-step states.

        This is the guard that runs by default. It cannot detect a physics
        change (nothing is re-flown) but it does detect a freeze that no
        longer describes its own flight."""
        s = self.states
        v = self.verified
        n = len(s["time_s"])
        powered = [i for i in range(n) if s["thrust_n"][i] > 0.0]

        self.assertAlmostEqual(max(s["mach"]), v["peak_mach"], delta=TOL)
        self.assertAlmostEqual(max(s["fuel_burned_kg"]), v["fuel_kg"],
                               delta=TOL)
        self.assertAlmostEqual(max(s["thrust_to_weight"]), v["peak_tw"],
                               delta=TOL)
        self.assertAlmostEqual(s["time_s"][-1], v["flight_s"], delta=TOL)
        self.assertAlmostEqual(abs(s["distance_m"][-1]),
                               v["lands_from_launch_m"], delta=TOL)

        peak_powered = max(s["load_n_total"][i] for i in powered
                           if s["mode"][i] in _POWERED_MODES)
        self.assertAlmostEqual(peak_powered, v["peak_load_n_total"],
                               delta=TOL)
        peak_mode = max((i for i in powered if s["mode"][i] in _POWERED_MODES),
                        key=lambda i: s["load_n_total"][i])
        self.assertEqual(s["mode"][peak_mode], v["peak_load_mode"])

        # The ramjet lightoff, located in the trace by the phase it happened
        # in rather than taken on trust from the summary.
        t_lit = v["ramjet_lightoff_time_s"]
        i_lit = min(range(n), key=lambda i: abs(s["time_s"][i] - t_lit))
        self.assertEqual(s["mode"][i_lit], v["ramjet_lightoff_mode"])
        self.assertAlmostEqual(s["mach"][i_lit], v["ramjet_lightoff_mach"],
                               delta=2e-3)
        self.assertAlmostEqual(s["altitude_m"][i_lit],
                               v["ramjet_lightoff_altitude_m"], delta=1.0)

        # The floor is the constraint the whole pull-out law exists to serve.
        floor = self.freeze["trajectory"]["floor_altitude_m"]
        tol_m = self.freeze["constants_at_freeze"]["V4_FLOOR_TOLERANCE_M"]
        climb_done = [i for i in powered if s["mode"][i] != "v3_climb"]
        flown_min = min(s["altitude_m"][i] for i in climb_done)
        self.assertAlmostEqual(flown_min, v["min_powered_altitude_m"],
                               delta=0.05)
        self.assertGreater(flown_min, floor - tol_m)
        self.assertEqual(v["floor_violated"], False)

    @unittest.skipUnless(TRACE.exists(), f"{TRACE.name} not present")
    def test_loop_load_in_the_trace_matches_the_phase_table(self):
        s = self.states
        loop = [s["load_n_total"][i] for i in range(len(s["time_s"]))
                if s["mode"][i] == "loop"]
        recorded = next(q["peak_load_n"] for q in self.freeze["phases"]
                        if q["mode"] == "loop")
        self.assertAlmostEqual(max(loop), recorded, delta=TOL)
        self.assertGreater(max(loop), 6.0)   # the fixed 6 g plus axial

    # -- the real thing, opt-in --------------------------------------------
    @unittest.skipUnless(os.environ.get("DOUGLAS_DART_REFLY_FP_RUNGS") == "1",
                         "the FP re-fly costs ~35 min; set "
                         "DOUGLAS_DART_REFLY_FP_RUNGS=1 to run it")
    def test_refly_reproduces_the_freeze(self):
        proc = subprocess.run(
            [sys.executable, str(FLYER)], cwd=ROOT, capture_output=True,
            text=True, env={**os.environ, "PYTHONPATH": str(ROOT)})
        if proc.returncode != 0:
            raise AssertionError(
                f"fly_frozen_v4_medium.py failed:\n{proc.stderr}")
        line = next(ln for ln in proc.stdout.splitlines()
                    if ln.startswith("FLIGHT_JSON:"))
        got = json.loads(line[len("FLIGHT_JSON:"):])
        for f in _FLOAT_FIELDS:
            want = self.verified.get(f)
            if want is None:
                self.assertIsNone(got[f], msg=f)
                continue
            self.assertAlmostEqual(
                got[f], want, delta=TOL,
                msg=f"{f} drifted {want} -> {got[f]}")
        for f in _EXACT_FIELDS:
            self.assertEqual(got[f], self.verified[f], msg=f)


if __name__ == "__main__":
    unittest.main()
