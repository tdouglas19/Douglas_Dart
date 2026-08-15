"""Guards for the 2D propulsion geometry model.

These are geometry invariants, not flight results: nothing here imports
medium_model or runs an engine, so the suite is fast and has no FP
kill-switch dependency.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from scripts.propulsion_2d.geometry import (  # noqa: E402
    ASSUMED, DEFAULT_ASSUMPTIONS, DEFAULT_DIMS, DERIVED, JSON, MM,
    build_vehicle, diffuser_diagnostics, export_area_csv, export_contour_csv,
    export_geometry_json, export_stations_csv, load_inputs, reconciliation)


class PropulsionGeometryTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.dims, cls.assumptions = load_inputs()
        cls.v = build_vehicle(cls.dims, cls.assumptions)

    # ---- the JSON is reproduced exactly --------------------------------
    def test_stations_match_the_json_bit_for_bit(self):
        """Any station tagged JSON must equal its source value exactly --
        no rounding, no re-derivation."""
        src = self.dims["stations_from_nose_tip_m"]
        expect = {
            "nose_tip": src["nose_tip"],
            "nose_fairing_end": src["nose_fairing_end__barrel_start"],
            "chamber_start": src["combustion_chamber_start"],
            "chamber_end": src["combustion_chamber_end__tailpipe_start"],
            "tailpipe_end": src["tailpipe_end"],
            "nozzle_exit": src["nozzle_exit__body_end"],
        }
        for st in self.v.stations:
            if st.name in expect:
                self.assertEqual(st.x, expect[st.name], st.name)
                self.assertEqual(st.source, JSON, st.name)

    def test_body_dimensions_match_the_json(self):
        body = self.dims["body"]
        self.assertEqual(self.v.d_body, body["diameter_m"])
        self.assertEqual(self.v.x_max, body["overall_length_m"])
        self.assertAlmostEqual(self.v.a_body, body["frontal_area_m2"], places=12)

    def test_stations_are_monotonic(self):
        xs = [st.x for st in self.v.stations]
        self.assertEqual(xs, sorted(xs))

    # ---- contour integrity ---------------------------------------------
    def test_every_chain_is_continuous(self):
        """No gaps or steps between consecutive segments of a chain."""
        for name, ch in self.v.chains.items():
            for a, b in zip(ch.segments, ch.segments[1:]):
                self.assertAlmostEqual(a.x1, b.x0, places=12,
                                       msg=f"{name}: x gap at {a.name}->{b.name}")
                self.assertAlmostEqual(a.r1, b.r0, places=12,
                                       msg=f"{name}: r step at {a.name}->{b.name}")

    def test_segments_have_positive_length_and_radius(self):
        for name, ch in self.v.chains.items():
            for s in ch.segments:
                self.assertGreater(s.length, 0.0, f"{name}/{s.name}")
                self.assertGreaterEqual(min(s.r0, s.r1), 0.0, f"{name}/{s.name}")

    def test_r_at_reproduces_endpoints_exactly(self):
        for name, ch in self.v.chains.items():
            for s in ch.segments:
                self.assertAlmostEqual(s.r_at(s.x0), s.r0, places=12)
                self.assertAlmostEqual(s.r_at(s.x1), s.r1, places=12)

    def test_flow_stays_inside_the_outer_mould_line(self):
        """The duct can never poke through the skin."""
        oml, duct = self.v.chains["oml"], self.v.chains["duct_od"]
        n = 2000
        for i in range(n + 1):
            x = duct.x0 + (duct.x1 - duct.x0) * i / n
            r_o, r_d = oml.r_at(x), duct.r_at(x)
            if r_o is None or r_d is None:
                continue
            self.assertLessEqual(r_d, r_o + 1e-9,
                                 f"duct outside the OML at x={x * MM:.2f} mm")

    def test_centrebody_stays_inside_the_cowl(self):
        cb, cowl = self.v.chains["centrebody"], self.v.chains["cowl_inner"]
        n = 1000
        for i in range(n + 1):
            x = cowl.x0 + (cowl.x1 - cowl.x0) * i / n
            self.assertLess(cb.r_at(x), cowl.r_at(x) + 1e-9,
                            f"centrebody outside the cowl at x={x * MM:.2f} mm")

    def test_flow_area_is_continuous_except_at_the_dump(self):
        """No area discontinuity anywhere in the duct EXCEPT the dump plane.

        The dump is a real feature -- a ramjet dump combustor expands
        suddenly at the chamber head -- so it is asserted to be there and to
        have the right size, rather than smoothed over.
        """
        x_dump = self.v.station("chamber_start")
        x0, x1 = self.v.station("cowl_lip"), self.v.station("nozzle_exit")
        n = 4000
        prev_x = prev = None
        jumps = []
        for i in range(n + 1):
            x = x0 + (x1 - x0) * i / n
            a = self.v.flow_area(x)
            if a is None:
                continue
            if prev is not None:
                # the 15 deg contraction cone is the steepest smooth feature
                if abs(a - prev) >= 0.05 * math.pi * 0.05782 ** 2:
                    jumps.append((prev_x, x))
            prev, prev_x = a, x

        self.assertEqual(len(jumps), 1,
                         f"expected exactly one jump (the dump); got {jumps}")
        lo, hi = jumps[0]
        self.assertLessEqual(lo, x_dump)
        self.assertGreaterEqual(hi, x_dump)

        # and the step is the dump ratio the diffuser policy asked for
        r_dump = self.v.chains["cowl_inner"].segments[-1].r1
        r_cham = self.v.chains["flow"].segments[0].r0
        self.assertAlmostEqual(
            self.v.scalars["dump sudden-expansion area ratio"][0],
            (r_cham / r_dump) ** 2, places=9)

    def test_full_diffuser_policy_has_no_dump(self):
        """The alternative policy diffuses the whole way, so the dump ratio
        collapses to 1 and the area distribution becomes continuous."""
        alt = json.loads(json.dumps(self.assumptions))
        alt["inlet"]["diffuser_policy"] = "full"
        v2 = build_vehicle(self.dims, alt)
        self.assertAlmostEqual(
            v2.scalars["dump sudden-expansion area ratio"][0], 1.0, places=9)
        # ...at the cost of a wall angle that certainly separates
        from scripts.propulsion_2d.geometry import diffuser_diagnostics
        self.assertGreater(
            diffuser_diagnostics(v2)["max_half_angle_deg"], 10.0)

    def test_unknown_diffuser_policy_is_rejected(self):
        bad = json.loads(json.dumps(self.assumptions))
        bad["inlet"]["diffuser_policy"] = "nope"
        with self.assertRaises(ValueError):
            build_vehicle(self.dims, bad)

    # ---- the design rules actually hold --------------------------------
    def test_capture_area_matches_the_engine_requirement(self):
        """The lip is SOLVED so the annular capture equals what the flight
        model says the engine actually consumes."""
        want = self.dims["inlet"]["implied_capture_area_m2"]
        self.assertAlmostEqual(self.v.flow_area(self.v.station("cowl_lip")),
                               want, places=12)
        self.assertAlmostEqual(
            self.v.scalars["annular capture area"][0], want * 1e4, places=9)

    def test_capture_is_solved_from_the_centrebody_not_the_throat(self):
        """pi(r_lip^2 - r_cb^2) = A_capture, with the centrebody free."""
        inl = self.assumptions["inlet"]
        r_cb = 0.5 * inl["centrebody_max_diameter_frac_of_body"] * self.v.d_body
        a_cap = self.dims["inlet"]["implied_capture_area_m2"]
        r_lip = 0.5 * self.v.scalars["cowl lip diameter"][0] / MM
        self.assertAlmostEqual(math.pi * (r_lip ** 2 - r_cb ** 2), a_cap,
                               places=12)

    def test_the_old_throat_rule_is_still_reproducible(self):
        """Kept so the earlier drawing can be rebuilt -- and so the 2.67x
        oversizing the flight model found stays visible."""
        alt = json.loads(json.dumps(self.assumptions))
        alt["inlet"]["capture_area_rule"] = "annulus_equals_throat_area"
        v2 = build_vehicle(self.dims, alt)
        self.assertAlmostEqual(v2.scalars["capture / throat area"][0], 1.0,
                               places=12)
        self.assertGreater(v2.scalars["annular capture area"][0],
                           2.5 * self.v.scalars["annular capture area"][0])

    def test_inlet_is_no_longer_oversized(self):
        """The headline correction: the throat rule oversized the inlet
        2.67x, and the drawing must not still be doing that."""
        frac = self.v.scalars["capture / throat area"][0]
        self.assertAlmostEqual(
            frac, self.dims["inlet"]["capture_area_as_fraction_of_throat_area"],
            places=6)
        self.assertLess(frac, 0.5)

    def test_cone_half_angle_is_what_was_asked_for(self):
        want = self.assumptions["chamber_to_tailpipe_cone"]["half_angle_deg"]
        cone = self.v.chains["flow"].segments[1]
        self.assertAlmostEqual(-cone.half_angle_deg, want, places=9)

    def test_spike_half_angle_is_what_was_asked_for(self):
        want = self.assumptions["inlet"]["centrebody_cone_half_angle_deg"]
        spike = self.v.chains["centrebody"].segments[0]
        self.assertAlmostEqual(spike.half_angle_deg, want, places=9)

    def test_constant_area_angle_diffuser_is_actually_constant(self):
        """The law's whole job: max equivalent angle == mean, not worse."""
        d = diffuser_diagnostics(self.v)
        self.assertAlmostEqual(d["max_half_angle_deg"],
                               d["mean_half_angle_deg"], places=3)

    def test_boattail_angle_is_derived_not_assumed(self):
        """It is a consequence of the JSON's own numbers, so it must not be
        listed as an assumption."""
        bt = self.v.chains["oml"].segments[-1]
        self.assertEqual(bt.source, DERIVED)
        body = self.dims["body"]
        want = math.degrees(math.atan2(
            0.5 * body["diameter_m"] - bt.r1, body["boattail_length_m"]))
        self.assertAlmostEqual(-bt.half_angle_deg, want, places=9)
        # the corrected export closes at exactly the 8 deg drag_buildup uses
        self.assertAlmostEqual(-bt.half_angle_deg,
                               self.dims["body"]["boattail_half_angle_deg"],
                               places=6)

    def test_boattail_does_not_close_onto_the_nozzle(self):
        """The corrected export (2026-08-14) closes the aft body to 153.849
        mm, NOT to the nozzle.  An annular base remains, and base drag on it
        is a real term in the flown build-up."""
        body = self.dims["body"]
        t_duct = self.dims["materials_and_gauges"]["duct_wall_thickness_m"]
        r_exit = self.v.chains["flow"].r_at(self.v.station("nozzle_exit"))

        bt = self.v.chains["oml"].segments[-1]
        self.assertAlmostEqual(2 * bt.r1, body["boattail_exit_diameter_m"],
                               places=12)
        # comfortable clearance -- the old "unbuildable" finding dissolved
        self.assertGreater(bt.r1 - (r_exit + t_duct), 0.010)
        self.assertGreater(self.v.scalars["annular base area (as drawn)"][0],
                           0.0)

    def test_base_annulus_matches_the_export_when_undiverged(self):
        """With no nozzle expansion the drawn base annulus must reproduce
        the export's own engine-on figure."""
        alt = json.loads(json.dumps(self.assumptions))
        alt["divergent_nozzle"]["expansion_area_ratio"] = 1.0
        v2 = build_vehicle(self.dims, alt)
        t_duct = self.dims["materials_and_gauges"]["duct_wall_thickness_m"]
        # the export's annulus is measured to the nozzle FLOW diameter; ours
        # is measured to the duct OUTER wall, so ours is smaller by the wall
        r_b = 0.5 * self.dims["body"]["boattail_exit_diameter_m"]
        r_n = 0.5 * self.dims["body"]["nozzle_flow_diameter_m"]
        self.assertAlmostEqual(
            self.dims["body"]["annular_base_area_engine_on_m2"],
            math.pi * (r_b ** 2 - r_n ** 2), places=9)
        self.assertAlmostEqual(
            v2.scalars["annular base area (as drawn)"][0] / 1e4,
            math.pi * (r_b ** 2 - (r_n + t_duct) ** 2), places=9)

    def test_divergence_moves_the_exit_and_never_the_throat(self):
        """Adding expansion must not disturb the throat: throat area sets
        the mass flow, the area fraction, AND the inlet capture rule."""
        flow = self.dims["internal_flowpath"]
        ar = self.assumptions["divergent_nozzle"]["expansion_area_ratio"]

        r_th = self.v.chains["flow"].r_at(self.v.station("nozzle_throat"))
        self.assertAlmostEqual(2 * r_th, flow["tailpipe_diameter_m"],
                               places=12)
        # the model calls this the exit; it is really the throat
        self.assertAlmostEqual(2 * r_th, flow["nozzle_exit_diameter_m"],
                               places=12)
        # the area fraction still comes off the throat, unchanged
        self.assertAlmostEqual(
            (2 * r_th / self.v.d_body) ** 2,
            flow["throat_to_body_area_fraction"], places=12)

        r_exit = self.v.chains["flow"].r_at(self.v.station("nozzle_exit"))
        self.assertAlmostEqual((r_exit / r_th) ** 2, ar, places=12)
        self.assertGreater(r_exit, r_th)

    def test_divergent_length_follows_the_half_angle(self):
        noz = self.assumptions["divergent_nozzle"]
        div = self.v.chains["flow"].segments[-1]
        self.assertAlmostEqual(div.half_angle_deg, noz["half_angle_deg"],
                               places=9)
        self.assertEqual(div.source, ASSUMED)
        self.assertAlmostEqual(div.x1, self.v.station("nozzle_exit"),
                               places=12)

    def test_bad_expansion_ratios_are_rejected(self):
        for ar, why in ((0.9, "a contraction"), (12.0, "wider than the body")):
            bad = json.loads(json.dumps(self.assumptions))
            bad["divergent_nozzle"]["expansion_area_ratio"] = ar
            with self.assertRaises(ValueError, msg=why):
                build_vehicle(self.dims, bad)

    def test_nozzle_diagnostics_are_self_consistent(self):
        from scripts.propulsion_2d.geometry import (_area_ratio,
                                                    nozzle_diagnostics)
        n = nozzle_diagnostics(self.v)
        g = n["gamma"]
        # the reported exit Mach must actually reproduce the built area ratio
        self.assertAlmostEqual(_area_ratio(n["exit_mach"], g),
                               n["area_ratio"], places=4)
        # choking Mach must sit right at the choking pressure ratio
        self.assertTrue(n["choked"])
        self.assertLess(n["choking_mach"], n["design_mach"])
        # over-expanded: built ratio exceeds the optimum, so pe < pa
        self.assertGreater(n["area_ratio"], n["optimum_area_ratio"])
        self.assertLess(n["pe_over_pa"], 1.0)

    def test_expansion_trade_base_area_falls_with_area_ratio(self):
        """Expansion cannot move the boattail (it is fixed at 8 deg by the
        export and does not touch the nozzle).  What it does is eat the
        annular base."""
        from scripts.propulsion_2d.geometry import expansion_trade
        rows = expansion_trade(self.v)
        areas = [r["base_area_m2"] for r in rows]
        self.assertEqual(areas, sorted(areas, reverse=True))
        # the trade must agree with the built geometry at the built ratio
        ar = self.assumptions["divergent_nozzle"]["expansion_area_ratio"]
        match = [r for r in rows if abs(r["area_ratio"] - ar) < 1e-9]
        self.assertTrue(match, f"trade table should include AR {ar}")
        self.assertAlmostEqual(
            match[0]["base_area_m2"] * 1e4,
            self.v.scalars["annular base area (as drawn)"][0], places=9)
        self.assertTrue(all(r["fits"] for r in rows), "a listed ratio does "
                        "not fit inside the base")

    def test_tailpipe_flow_diameter_is_the_json_value(self):
        pipe = self.v.chains["flow"].segments[2]
        self.assertAlmostEqual(
            2 * pipe.r0,
            self.dims["internal_flowpath"]["tailpipe_diameter_m"], places=12)
        self.assertAlmostEqual(2 * pipe.r1, 2 * pipe.r0, places=12)

    # ---- volumes --------------------------------------------------------
    def test_frustum_volume_matches_numeric_integration(self):
        """The closed-form straight-segment volume must agree with Simpson."""
        cone = self.v.chains["flow"].segments[1]
        exact = cone.volume()
        n, h, total = 4000, cone.length / 4000, 0.0
        for i in range(n):
            x = cone.x0 + (i + 0.5) * h
            total += math.pi * cone.r_at(x) ** 2 * h
        self.assertAlmostEqual(exact, total, delta=1e-7 * exact)

    def test_reconciliation_reports_the_known_deltas(self):
        rows = {r[0]: r for r in reconciliation(self.v)}
        # chamber shrinks by exactly the wall stack
        mat = self.dims["materials_and_gauges"]
        stack = mat["skin_gauge_m"] + mat["duct_wall_thickness_m"]
        _, model_d, drawn_d, _, _ = rows["chamber flow diameter"]
        self.assertAlmostEqual(model_d - drawn_d, 2 * stack * MM, places=9)
        # the fuel result must stay safe: capacity above what is loaded
        _, _, cap, _, _ = rows["fuel capacity"]
        self.assertGreater(cap, self.dims["fuel_system"]["fuel_loaded_kg"])

    # ---- provenance discipline -----------------------------------------
    def test_nothing_invented_is_tagged_json(self):
        """A segment may only claim JSON provenance if it is not built from
        an assumptions.json value."""
        assumed_chains = {"centrebody", "cowl_inner"}
        for name in assumed_chains:
            for s in self.v.chains[name].segments:
                self.assertEqual(s.source, ASSUMED, f"{name}/{s.name}")

    def test_every_assumption_knob_is_actually_read(self):
        """A knob that silently does nothing is worse than no knob: someone
        edits it, nothing changes, and they trust a drawing that ignored
        them.  Keys prefixed '_' are comments and are exempt."""
        pkg = _REPO / "scripts" / "propulsion_2d"
        src = "\n".join(f.read_text("utf-8") for f in pkg.glob("*.py"))

        # 'not_modelled' is a documentation block enumerated wholesale by
        # print_assumptions, so its individual entries are not knobs
        WHOLESALE = {"not_modelled"}
        for block in WHOLESALE:
            self.assertIn(f'"{block}"', src,
                          f"'{block}' is exempt but nothing consumes it")

        dead: list[str] = []

        def walk(node, path=""):
            for key, val in node.items():
                if key.startswith("_") or key in WHOLESALE:
                    continue
                if isinstance(val, dict):
                    walk(val, f"{path}{key}.")
                elif f'"{key}"' not in src:
                    dead.append(f"{path}{key}")

        walk(self.assumptions)
        self.assertEqual(dead, [],
                         f"assumptions.json keys never read by the code: {dead}")

    def test_unknown_capture_area_rule_is_rejected(self):
        bad = json.loads(json.dumps(self.assumptions))
        bad["inlet"]["capture_area_rule"] = "something_else"
        with self.assertRaises(ValueError):
            build_vehicle(self.dims, bad)

    def test_wall_readings_are_checked_against_the_json(self):
        """Both readings are proven by the export, so a build must fail loudly
        if either stops holding rather than draw the wall on the wrong side."""
        from scripts.propulsion_2d.geometry import _check_wall_readings
        _check_wall_readings(self.dims, self.assumptions["walls"])

        bad = json.loads(json.dumps(self.dims))
        bad["body"]["frontal_area_m2"] *= 1.05
        with self.assertRaises(ValueError):
            _check_wall_readings(bad, self.assumptions["walls"])

        bad2 = json.loads(json.dumps(self.dims))
        bad2["internal_flowpath"]["throat_to_body_area_fraction"] *= 1.05
        with self.assertRaises(ValueError):
            _check_wall_readings(bad2, self.assumptions["walls"])

    def test_every_segment_has_a_known_provenance_tag(self):
        for name, ch in self.v.chains.items():
            for s in ch.segments:
                self.assertIn(s.source, (JSON, DERIVED, ASSUMED),
                              f"{name}/{s.name}")

    # ---- sampling / exports ---------------------------------------------
    def test_uniform_sample_closes_on_the_true_endpoint(self):
        """The 1 mm grid does not land on 2273.607, so the export must add
        the real end rather than stop short."""
        ch = self.v.chains["oml"]
        xs, _ = ch.sample_uniform(0.001)
        self.assertAlmostEqual(xs[-1], ch.x1, places=12)
        self.assertAlmostEqual(xs[0], ch.x0, places=12)

    def test_adaptive_polyline_uses_two_points_on_a_straight(self):
        pipe = self.v.chains["flow"].segments[2]
        xs, _ = pipe.sample()
        self.assertEqual(len(xs), 2)

    def test_polyline_is_dense_on_a_curve(self):
        diff = self.v.chains["cowl_inner"].segments[0]
        xs, _ = diff.sample()
        self.assertGreater(len(xs), 100)

    def test_exports_are_written_and_reloadable(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            export_stations_csv(self.v, out / "stations.csv")
            export_contour_csv(self.v, out / "contour.csv")
            export_area_csv(self.v, out / "flow_area.csv")
            export_geometry_json(self.v, out / "geometry.json")
            for f in ("stations.csv", "contour.csv", "flow_area.csv",
                      "geometry.json"):
                self.assertTrue((out / f).exists(), f)
                self.assertGreater((out / f).stat().st_size, 0, f)

            payload = json.loads((out / "geometry.json").read_text("utf-8"))
            self.assertEqual(len(payload["stations"]), len(self.v.stations))
            self.assertEqual(set(payload["chains"]), set(self.v.chains))
            # round-trip a segment endpoint at full precision
            seg = payload["chains"]["flow"]["segments"][0]
            self.assertEqual(seg["x0_m"], self.v.chains["flow"].segments[0].x0)

    def test_exports_contain_every_station_as_an_exact_row(self):
        """The 1 mm grid straddles the real breakpoints, so they must be
        merged in -- otherwise the exact area at 817.307 mm is something you
        have to interpolate for."""
        import csv
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            export_area_csv(self.v, out / "flow_area.csv")
            export_contour_csv(self.v, out / "contour.csv")

            with open(out / "flow_area.csv", encoding="utf-8") as fh:
                rows = [r for r in csv.DictReader(fh) if r["station"]]
            # two stations share x=428.0, so names are joined with '|' --
            # neither may be dropped
            named = {n for r in rows for n in r["station"].split("|")}
            for st in self.v.stations:
                if self.v.flow_area(st.x) is not None:
                    self.assertIn(st.name, named, st.name)
            self.assertIn("nose_fairing_end|chamber_start",
                          {r["station"] for r in rows})

            by_name = {n: r for r in rows for n in r["station"].split("|")}
            # capture is the engine requirement, not the throat area
            self.assertAlmostEqual(
                float(by_name["cowl_lip"]["flow_area_cm2"]),
                self.dims["inlet"]["implied_capture_area_m2"] * 1e4, places=3)
            for s in ("cone_end", "tailpipe_end", "nozzle_throat"):
                self.assertAlmostEqual(
                    float(by_name[s]["area_over_throat"]), 1.0, places=5)
            # and the exit carries exactly the expansion ratio
            self.assertAlmostEqual(
                float(by_name["nozzle_exit"]["area_over_throat"]),
                self.assumptions["divergent_nozzle"]["expansion_area_ratio"],
                places=5)

            with open(out / "contour.csv", encoding="utf-8") as fh:
                cont = [r for r in csv.DictReader(fh) if r["station"]]
            self.assertTrue(cont)
            # x values must be the exact station values, not rounded grid ones
            xs = {float(r["x_mm"]) for r in cont}
            self.assertIn(round(self.v.station("chamber_end") * MM, 4), xs)

    def test_geometry_json_carries_the_source_git_sha(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "geometry.json"
            export_geometry_json(self.v, p)
            payload = json.loads(p.read_text("utf-8"))
            self.assertEqual(payload["provenance"]["git_sha"],
                             self.dims["provenance"]["git_sha"])

    # ---- the tool runs --------------------------------------------------
    def test_cli_runs_and_prints_the_tables(self):
        r = subprocess.run(
            [sys.executable, "-m", "scripts.propulsion_2d"],
            cwd=_REPO, capture_output=True, text=True, timeout=180)
        self.assertEqual(r.returncode, 0, r.stderr)
        for heading in ("STATIONS", "KEY DIMENSIONS", "INTAKE DIFFUSER",
                        "MODEL vs AS-DRAWN", "ASSUMED GEOMETRY"):
            self.assertIn(heading, r.stdout)

    def test_renders_without_a_display(self):
        with tempfile.TemporaryDirectory() as td:
            r = subprocess.run(
                [sys.executable, "-m", "scripts.propulsion_2d", "--quiet",
                 "--save-vector", "--out-dir", td],
                cwd=_REPO, capture_output=True, text=True, timeout=300)
            self.assertEqual(r.returncode, 0, r.stderr)
            for ext in ("svg", "pdf", "png"):
                p = Path(td) / f"propulsion_2d_section.{ext}"
                self.assertTrue(p.exists(), p)
                self.assertGreater(p.stat().st_size, 1024, p)

    def test_input_files_exist_where_the_defaults_say(self):
        self.assertTrue(DEFAULT_DIMS.exists(), DEFAULT_DIMS)
        self.assertTrue(DEFAULT_ASSUMPTIONS.exists(), DEFAULT_ASSUMPTIONS)

    def test_no_medium_model_import(self):
        """This tool must not drift with the flight code."""
        for f in (_REPO / "scripts" / "propulsion_2d").glob("*.py"):
            text = f.read_text("utf-8")
            self.assertNotIn("import medium_model", text, f.name)
            self.assertNotIn("from medium_model", text, f.name)


class ViewerInteractionTest(unittest.TestCase):
    """The interactive paths ARE the deliverable, so exercise them headless
    instead of trusting that --show works."""

    @classmethod
    def setUpClass(cls):
        import matplotlib
        matplotlib.use("Agg")
        from scripts.propulsion_2d.viewer import SectionViewer
        cls.dims, cls.assumptions = load_inputs()
        cls.v = build_vehicle(cls.dims, cls.assumptions)
        cls.viewer = SectionViewer(cls.v, Path(tempfile.gettempdir()))
        cls.viewer.fig.canvas.draw()

    @classmethod
    def tearDownClass(cls):
        import matplotlib.pyplot as plt
        plt.close(cls.viewer.fig)

    def test_cursor_readout_reports_the_right_region_and_area(self):
        mid_pipe = 0.5 * (self.v.station("cone_end")
                          + self.v.station("tailpipe_end")) * MM
        txt = self.viewer._describe(mid_pipe, 30.0)
        self.assertIn("TAILPIPE", txt)
        self.assertIn("A/A_th", txt)
        # in the constant-diameter pipe the area IS the throat area
        self.assertIn("1.000", txt)

    def test_cursor_readout_outside_the_body(self):
        self.assertIn("outside the body", self.viewer._describe(-500.0, 0.0))
        self.assertIn("outside the body",
                      self.viewer._describe(self.v.x_max * MM + 500, 0.0))

    def test_readout_covers_every_region_without_raising(self):
        seen = set()
        for i in range(400):
            x = self.v.x_max * MM * i / 399
            seen.add(self.v.region_at(x / MM))
            self.viewer._describe(x, 20.0)
        self.assertGreaterEqual(len(seen), 6, seen)

    def test_pin_lands_on_a_contour_and_carries_exact_values(self):
        before = len(self.viewer.pins)
        x = 0.5 * (self.v.station("chamber_start")
                   + self.v.station("chamber_end")) * MM
        r = self.v.chains["flow"].segments[0].r0 * MM
        self.viewer._pin(x, r)
        self.assertEqual(len(self.viewer.pins), before + 1)
        text = self.viewer.pins[-1][0].get_text()
        self.assertIn("chamber", text)
        self.assertIn("mm", text)
        self.assertIn("A_flow", text)

    def test_pin_snaps_to_a_named_station(self):
        x = self.v.station("chamber_end") * MM
        r = self.v.chains["flow"].r_at(self.v.station("chamber_end")) * MM
        self.viewer._pin(x + 2.0, r)          # 2 mm off, inside the 6 mm tol
        text = self.viewer.pins[-1][0].get_text()
        self.assertIn("STATION chamber_end", text)

    def test_pin_far_from_any_contour_is_ignored(self):
        before = len(self.viewer.pins)
        self.viewer._pin(1200.0, 420.0)       # well clear of the body
        self.viewer._pin(600.0, 0.0)          # centreline inside the chamber
        self.assertEqual(len(self.viewer.pins), before)

    def test_pin_works_along_a_long_straight(self):
        """A straight segment has only two drawing vertices; picking must
        project onto the segment, not snap to its ends."""
        x0 = self.v.station("cone_end") * MM
        x1 = self.v.station("tailpipe_end") * MM
        r = self.v.chains["flow"].r_at(self.v.station("tailpipe_end")) * MM
        for frac in (0.2, 0.5, 0.8):
            x = x0 + (x1 - x0) * frac
            n = len(self.viewer.pins)
            self.viewer._pin(x, r)
            self.assertEqual(len(self.viewer.pins), n + 1,
                             f"no pin at {frac:.0%} along the tailpipe")
            self.assertAlmostEqual(self.viewer.pins[-1][1], x, delta=1.0)
        while self.viewer.pins:
            self.viewer.pins.pop()[0].remove()

    def test_unpin_and_clear(self):
        self.viewer._pin(600.0, 104.5)
        n = len(self.viewer.pins)
        self.assertGreater(n, 0)
        self.viewer._unpin(600.0, 104.5)
        self.assertEqual(len(self.viewer.pins), n - 1)

        self.viewer._pin(600.0, 104.5)

        class _Ev:
            key = "c"
        self.viewer._on_key(_Ev())
        self.assertEqual(len(self.viewer.pins), 0)

    def test_layer_toggles_hide_and_show(self):
        class _Ev:
            def __init__(self, key):
                self.key = key

        for i, layer in enumerate(self.viewer.LAYERS, start=1):
            self.assertTrue(self.viewer.artists[layer],
                            f"layer '{layer}' registered no artists")
            self.viewer._on_key(_Ev(str(i)))
            self.assertFalse(self.viewer.visible[layer])
            self.assertFalse(self.viewer.artists[layer][0].get_visible())
            self.viewer._on_key(_Ev(str(i)))          # back on
            self.assertTrue(self.viewer.visible[layer])
            self.assertTrue(self.viewer.artists[layer][0].get_visible())

    def test_area_panel_readout(self):
        txt = self.viewer.axa.format_coord(1500.0, 105.0)
        self.assertIn("cm2", txt)
        self.assertIn("TAILPIPE", txt)

    def test_section_axes_is_true_to_scale(self):
        self.assertEqual(self.viewer.ax.get_aspect(), 1.0)

    def test_no_dead_band__figure_height_matches_the_drawing(self):
        """The section axes must be exactly as tall as its data demands, or
        the figure grows a blank band above and below the vehicle."""
        bbox = self.viewer.ax.get_window_extent()
        x0, x1 = self.viewer.xlim
        y0, y1 = self.viewer.ylim
        self.assertAlmostEqual(bbox.height / bbox.width,
                               (y1 - y0) / (x1 - x0), places=2)
        """This tool must not drift with the flight code."""
        for f in (_REPO / "scripts" / "propulsion_2d").glob("*.py"):
            text = f.read_text("utf-8")
            self.assertNotIn("import medium_model", text, f.name)
            self.assertNotIn("from medium_model", text, f.name)


if __name__ == "__main__":
    unittest.main()
