"""Guards for the translating-inlet feasibility study.

Geometry and kinematics only -- no flight model, no FP dependency.
"""
from __future__ import annotations

import math
import subprocess
import sys
import unittest
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from scripts.propulsion_2d.geometry import MM, build_vehicle, load_inputs  # noqa: E402
from scripts.propulsion_2d.translating_inlet import (  # noqa: E402
    SPAR_RADIUS_M, area_curve, build_chains, from_vehicle, solve, sweep)


class TranslatingInletTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.dims, cls.assumptions = load_inputs()
        cls.v = build_vehicle(cls.dims, cls.assumptions)
        cls.t = from_vehicle(cls.v, 0.300, 0.86 * 0.5 * cls.v.d_body, 45.0)

    # ---- the solve ------------------------------------------------------
    def test_open_area_equals_the_engine_requirement(self):
        want = self.dims["inlet"]["implied_capture_area_m2"]
        self.assertAlmostEqual(self.t.open_area_m2(0.0), want, places=12)
        self.assertAlmostEqual(self.t.capture_area_m2, want, places=12)

    def test_sealed_at_full_stroke(self):
        self.assertAlmostEqual(self.t.open_area_m2(self.t.stroke_m), 0.0,
                               places=14)
        self.assertAlmostEqual(self.t.radius_at_slot(self.t.stroke_m),
                               self.t.r_lip_m, places=12)

    def test_stroke_is_gap_over_tan_seal(self):
        self.assertAlmostEqual(
            self.t.stroke_m,
            self.t.gap_m / math.tan(math.radians(self.t.seal_half_angle_deg)),
            places=12)

    def test_area_falls_monotonically_with_stroke(self):
        areas = [a for _, a, _ in area_curve(self.t, 200)]
        self.assertEqual(areas, sorted(areas, reverse=True))
        self.assertAlmostEqual(areas[0], self.t.capture_area_m2, places=12)
        self.assertAlmostEqual(areas[-1], 0.0, places=14)

    def test_area_fraction_inverse_round_trips(self):
        for frac in (0.9, 0.5, 0.25, 0.10, 0.01):
            s = self.t.stroke_for_area_fraction(frac)
            self.assertAlmostEqual(self.t.area_fraction(s), frac, places=9)
            self.assertGreaterEqual(s, 0.0)
            self.assertLessEqual(s, self.t.stroke_m + 1e-12)

    def test_outboard_moves_shorten_the_stroke(self):
        """The whole premise: for a fixed capture area, a bigger radius means
        a thinner annulus and therefore less travel to close it."""
        r_body = 0.5 * self.v.d_body
        strokes = [from_vehicle(self.v, 0.300, f * r_body, 45.0).stroke_m
                   for f in (0.62, 0.70, 0.78, 0.86)]
        self.assertEqual(strokes, sorted(strokes, reverse=True))

    def test_steeper_seat_shortens_the_stroke(self):
        strokes = [from_vehicle(self.v, 0.300, 0.86 * 0.5 * self.v.d_body,
                                a).stroke_m for a in (20.0, 30.0, 45.0, 60.0)]
        self.assertEqual(strokes, sorted(strokes, reverse=True))

    def test_beats_the_baseline_spike_by_a_wide_margin(self):
        """Baseline: 20 deg spike on the frozen 10.28 mm annulus = 28.2 mm."""
        cur = self.assumptions["inlet"]
        r_lip = 0.5 * self.v.scalars["cowl lip diameter"][0] / MM
        r_cb = 0.5 * self.v.scalars["centrebody max diameter"][0] / MM
        baseline = (r_lip - r_cb) / math.tan(
            math.radians(cur["centrebody_cone_half_angle_deg"]))
        self.assertGreater(baseline / self.t.stroke_m, 3.5)

    def test_stroke_is_independent_of_slot_station(self):
        """Useful decoupling: with the lip radius fixed, axial position buys
        diffuser length and nose volume without costing any travel."""
        strokes = {round(r["stroke_m"], 12) for r in sweep(self.v)
                   if r["seal_deg"] == 45.0}
        self.assertEqual(len(strokes), 1)

    def test_impossible_lip_radius_is_rejected(self):
        with self.assertRaises(ValueError):
            solve(capture_area_m2=0.10, x_slot_m=0.30, r_lip_m=0.05,
                  x_chamber_head_m=0.428)

    # ---- drawn geometry must match the analytic kinematics --------------
    def test_drawn_sleeve_reproduces_radius_at_slot(self):
        for i in range(11):
            s = self.t.stroke_m * i / 10
            drawn = build_chains(self.t, self.v, s)["sleeve"].r_at(
                self.t.x_slot_m)
            self.assertAlmostEqual(drawn, self.t.radius_at_slot(s), places=12,
                                   msg=f"stroke {s * MM:.3f} mm")

    def test_sleeve_is_a_rigid_translation(self):
        """Extending must SHIFT the body, not re-profile it -- the seal cone
        keeps its length and its endpoint radii."""
        a = build_chains(self.t, self.v, 0.0)["sleeve"].segments[0]
        b = build_chains(self.t, self.v, self.t.stroke_m)["sleeve"].segments[0]
        self.assertAlmostEqual(a.r0, b.r0, places=12)
        self.assertAlmostEqual(a.r1, b.r1, places=12)
        self.assertAlmostEqual(a.length, b.length, places=12)
        self.assertAlmostEqual(a.x0 - b.x0, self.t.stroke_m, places=12)

    def test_seal_cone_rises_aft_so_forward_travel_closes(self):
        seal = build_chains(self.t, self.v, 0.0)["sleeve"].segments[0]
        self.assertGreater(seal.r1, seal.r0)
        self.assertAlmostEqual(seal.half_angle_deg,
                               self.t.seal_half_angle_deg, places=9)

    def test_seat_and_seal_are_parallel(self):
        seat = build_chains(self.t, self.v, 0.0)["cowl_inner"].segments[0]
        seal = build_chains(self.t, self.v, 0.0)["sleeve"].segments[0]
        self.assertAlmostEqual(seat.half_angle_deg, seal.half_angle_deg,
                               places=9)

    def test_sleeve_never_escapes_the_cowl(self):
        for i in range(11):
            s = self.t.stroke_m * i / 10
            ch = build_chains(self.t, self.v, s)
            sleeve, cowl_in = ch["sleeve"], ch["cowl_inner"]
            n = 200
            for k in range(n + 1):
                x = cowl_in.x0 + (cowl_in.x1 - cowl_in.x0) * k / n
                rs = sleeve.r_at(x)
                if rs is None:
                    continue
                self.assertLessEqual(rs, cowl_in.r_at(x) + 1e-9,
                                     f"sleeve outside the cowl at "
                                     f"x={x * MM:.2f}, stroke {s * MM:.2f}")

    def test_chains_are_continuous(self):
        for s in (0.0, self.t.stroke_m):
            for name, ch in build_chains(self.t, self.v, s).items():
                for a, b in zip(ch.segments, ch.segments[1:]):
                    self.assertAlmostEqual(a.x1, b.x0, places=12, msg=name)
                    self.assertAlmostEqual(a.r1, b.r0, places=12, msg=name)

    def test_sleeve_closes_onto_the_spar(self):
        ch = build_chains(self.t, self.v, 0.0)["sleeve"]
        self.assertAlmostEqual(ch.segments[-1].r1, SPAR_RADIUS_M, places=12)

    # ---- the OML must not move (user directive) -------------------------
    def test_outer_mould_line_is_identical_in_both_states(self):
        open_ = build_chains(self.t, self.v, 0.0)
        shut = build_chains(self.t, self.v, self.t.stroke_m)
        for name in ("fairing", "cowl"):
            a, b = open_[name], shut[name]
            self.assertEqual(len(a.segments), len(b.segments), name)
            for sa, sb in zip(a.segments, b.segments):
                self.assertAlmostEqual(sa.x0, sb.x0, places=12, msg=name)
                self.assertAlmostEqual(sa.x1, sb.x1, places=12, msg=name)
                self.assertAlmostEqual(sa.r0, sb.r0, places=12, msg=name)
                self.assertAlmostEqual(sa.r1, sb.r1, places=12, msg=name)

    # ---- it must not disturb the frozen tool ----------------------------
    def test_frozen_section_is_untouched(self):
        """The study imports the kernel; it must not mutate the vehicle it
        was built from."""
        before = dict(self.v.scalars)
        from_vehicle(self.v, 0.25, 0.8 * 0.5 * self.v.d_body, 30.0)
        self.assertEqual(before, dict(self.v.scalars))
        self.assertNotIn("sleeve", self.v.chains)
        self.assertNotIn("fairing", self.v.chains)

    def test_full_drawing_duct_never_leaves_the_outer_mould_line(self):
        """Regression: an earlier revision let the diffuser wall poke ~2.5 mm
        outside the cowl over x 300-315 mm, because the sleeve rises 7.08 mm
        there while the cowl only grew 1.3 mm."""
        import scripts.propulsion_2d_full_drawing as F
        c = F.compose(self.v, self.t)
        rep = F.containment_report(self.v, self.t, c)
        self.assertLessEqual(rep["max_overshoot_m"] * MM, 1e-6,
                             f"duct outside the OML at "
                             f"x={rep['at_x_m'] * MM:.2f} mm")

    def test_full_drawing_does_not_neck_below_capture(self):
        """If the duct necked, the real throat would be the neck and the
        quoted capture area would be a fiction."""
        import scripts.propulsion_2d_full_drawing as F
        c = F.compose(self.v, self.t)
        rep = F.containment_report(self.v, self.t, c)
        self.assertGreaterEqual(rep["min_area_m2"],
                                self.t.capture_area_m2 - 1e-9)

    def test_full_drawing_internals_are_sane(self):
        import scripts.propulsion_2d_full_drawing as F
        c = F.compose(self.v, self.t)
        r_ch = self.v.chains["flow"].segments[0].r0
        # flameholder sits inside the chamber, at a sane radius and blockage
        self.assertGreater(c["fh"]["x"], self.v.station("chamber_start"))
        self.assertLess(c["fh"]["x"], self.v.station("chamber_end"))
        self.assertLess(c["fh"]["r"], r_ch)
        self.assertTrue(0.15 < c["fh"]["blockage"] < 0.55,
                        f"flameholder blockage {c['fh']['blockage']:.2f} is "
                        "outside the usual 30-50% band")
        # reed valves are at the chamber HEAD end, where a pulsejet needs them
        self.assertGreater(c["rv"]["x"], self.v.station("chamber_start"))
        self.assertLess(c["rv"]["x"],
                        self.v.station("chamber_start")
                        + 0.25 * (self.v.station("chamber_end")
                                  - self.v.station("chamber_start")))
        self.assertTrue(0.05 < c["rv"]["port_frac"] < 0.30)

    def test_full_drawing_script_runs(self):
        r = subprocess.run(
            [sys.executable, "scripts/propulsion_2d_full_drawing.py"],
            cwd=_REPO, capture_output=True, text=True, timeout=300)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("CONTAINMENT CHECK", r.stdout)
        self.assertIn("OK", r.stdout)
        self.assertNotIn("STILL VIOLATING", r.stdout)

    def test_study_script_runs(self):
        r = subprocess.run(
            [sys.executable, "scripts/propulsion_2d_translating_study.py"],
            cwd=_REPO, capture_output=True, text=True, timeout=300)
        self.assertEqual(r.returncode, 0, r.stderr)
        for heading in ("BASELINE", "TRADE", "CHOSEN DESIGN",
                        "open area vs stroke"):
            self.assertIn(heading, r.stdout)


if __name__ == "__main__":
    unittest.main()
