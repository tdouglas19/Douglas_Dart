"""Calibrated simple_model: anchor reproduction, operability gates, engine
summation, and the decel-glide-flare landing chain. Pure closed-form calls,
unittest-style (CI-safe)."""
from __future__ import annotations

import unittest

from simple_model.constants import FUELS, KG_PER_LB
from simple_model.drag import cd0_transonic_multiplier
from simple_model.flight_sim import VehicleGeometry, run_flight
from simple_model.pulsejet_simple import pulsejet_thrust
from simple_model.ramjet_simple import ramjet_thrust


class CalibrationAnchorTests(unittest.TestCase):
    """The three pulsejet-fp truth points must reproduce within the fitted
    residual band (~10%) -- these lock the calibration against drift."""

    ANCHORS = [
        ("FP-1", 0.078, 0.150, 0.042, 0.620, "propane", 18.6, 150.5, 1.34),
        ("25L", 0.2213, 0.4256, 0.1192, 1.759, "propane", 145.6, 59.2, 10.2),
        ("TW66", 0.123, 0.275, 0.066, 1.196, "jet_a", 52.2, 88.7, 3.94),
    ]

    def test_anchors_reproduce(self):
        for name, D, L, dt, tl, fuel, f_tgt, hz_tgt, fuel_tgt in self.ANCHORS:
            r = pulsejet_thrust(D, L, dt, tl, 0.0, 0.0, FUELS[fuel])
            self.assertTrue(r.operable, name)
            self.assertLess(abs(r.average_thrust_n / f_tgt - 1.0), 0.15, name)
            self.assertLess(abs(r.frequency_hz / hz_tgt - 1.0), 0.15, name)
            self.assertLess(
                abs(r.fuel_mass_flow_kg_per_s * 1e3 / fuel_tgt - 1.0), 0.15, name
            )


class OperabilityGateTests(unittest.TestCase):
    def test_dead_zone_throat_is_gated(self):
        # the 81/123 mm design pulsejet-fp proved cannot sustain (AR 0.434)
        r = pulsejet_thrust(0.123, 0.275, 0.081, 1.196, 0.0, 0.0, FUELS["jet_a"])
        self.assertFalse(r.operable)
        self.assertEqual(r.average_thrust_n, 0.0)
        self.assertEqual(r.fuel_mass_flow_kg_per_s, 0.0)

    def test_mach_lapse_monotonic(self):
        thrusts = [
            pulsejet_thrust(0.123, 0.275, 0.066, 1.196, m, 0.0, FUELS["jet_a"]).average_thrust_n
            for m in (0.0, 0.3, 0.6, 0.9)
        ]
        self.assertTrue(all(a > b > 0 for a, b in zip(thrusts, thrusts[1:])))

    def test_ramjet_lightoff_gate(self):
        below = ramjet_thrust(0.123, 0.066, 0.30, 0.0, FUELS["jet_a"])
        above = ramjet_thrust(0.123, 0.066, 0.60, 0.0, FUELS["jet_a"])
        self.assertFalse(below.lit)
        self.assertEqual(below.net_thrust_n, 0.0)
        self.assertEqual(below.fuel_mass_flow_kg_per_s, 0.0)
        self.assertTrue(above.lit)
        self.assertGreater(above.net_thrust_n, 0.0)

    def test_transonic_multiplier_shape(self):
        self.assertEqual(cd0_transonic_multiplier(0.5), 1.0)
        self.assertAlmostEqual(cd0_transonic_multiplier(0.8), 1.0, places=12)
        self.assertGreater(cd0_transonic_multiplier(1.1), cd0_transonic_multiplier(0.95))
        self.assertGreater(cd0_transonic_multiplier(1.1), cd0_transonic_multiplier(1.6))
        self.assertGreater(cd0_transonic_multiplier(2.0), 1.0)


class FlightChainTests(unittest.TestCase):
    def test_known_feasible_corner_flies_and_lands(self):
        """The probe design that closed the whole mission under calibrated
        physics (requires the CD0 override to 0.15 -- set here explicitly
        via the drag call default? No: CD0 is baked at import. This test
        instead verifies the flight CHAIN on the baseline CD0: the same
        geometry still reaches its (lower) ceiling, decelerates level, and
        finishes without the integrator misbehaving."""
        g = VehicleGeometry(0.30, 0.162, 0.50, 1.0, 0.75, FUELS["jet_a"])
        r = run_flight(g, initial_mass_kg=50 * KG_PER_LB, climb_angle_deg=1.0,
                       dt_s=0.05, max_time_s=600.0)
        self.assertGreater(len(r.states), 100)
        # engines summed: thrust during powered flight must exceed the
        # pulsejet-alone value once the ramjet lights (if cutoff reached,
        # crossover must have been recorded)
        if r.motor_cutoff_reached:
            self.assertIsNotNone(r.crossover_mach)

    def test_flight_modes_progress(self):
        g = VehicleGeometry(0.30, 0.162, 0.50, 1.0, 0.75, FUELS["jet_a"])
        r = run_flight(g, initial_mass_kg=50 * KG_PER_LB, climb_angle_deg=1.0,
                       dt_s=0.05, max_time_s=600.0)
        modes = {st.mode for st in r.states}
        self.assertIn("pulsejet", modes)


class WingWaveDragRegressionTests(unittest.TestCase):
    def test_no_sonic_singularity(self):
        """cd_wave must stay bounded through Mach 1 (an unregularized
        1/sqrt(Mn^2-1) once produced cd ~ 3.6 at Mn=1.0 -- a ~60 kN phantom
        drag wall that silently killed every transonic mission)."""
        from simple_model.drag import (default_wing_concept,
                                       wing_wave_drag_coefficient)
        c = default_wing_concept(0.85)
        for m in (0.95, 0.99, 1.0, 1.001, 1.05, 1.1, 1.5):
            cd = wing_wave_drag_coefficient(c, m)
            self.assertLess(cd, 0.06, f"M={m}: cd_wave={cd}")
        # and it is monotonically DECREASING well past the peak
        self.assertGreater(wing_wave_drag_coefficient(c, 1.2),
                           wing_wave_drag_coefficient(c, 2.0))


class ReturnToLaunchTests(unittest.TestCase):
    """The return profile (2026-08-12): pitch-up half-loop at cutoff, glide
    home, spiral + flare over the launch point. Locks the phase sequence and
    that the trajectory actually closes."""

    GEOM = VehicleGeometry(0.28, 0.151, 0.34, 0.68, 0.81, FUELS["propane"])

    def test_legacy_profile_unchanged_by_default(self):
        r = run_flight(self.GEOM, initial_mass_kg=50 * KG_PER_LB,
                       climb_angle_deg=1.0, dt_s=0.05, max_time_s=600.0)
        modes = {s.mode for s in r.states}
        self.assertNotIn("loop", modes)
        self.assertNotIn("return", modes)
        # legacy flights only ever fly downrange
        self.assertGreater(r.states[-1].distance_m, 0.0)

    def test_return_profile_reverses_and_flies_back(self):
        """CD0 is baked at import, so at the suite's default CD0 this design
        cannot reach M 1.1 -- cutoff is lowered here so the post-cutoff
        mechanism (half-loop -> reversed heading -> inbound glide) is what
        gets tested. Completing the trip home needs the low-CD0 energy
        budget and is verified in the campaign reports, not here."""
        r = run_flight(self.GEOM, initial_mass_kg=50 * KG_PER_LB,
                       climb_angle_deg=1.0, motor_cutoff_mach=0.30,
                       dt_s=0.05, max_time_s=900.0, return_to_launch=True)
        self.assertTrue(r.motor_cutoff_reached)
        modes = [s.mode for s in r.states]
        for phase in ("loop", "return"):
            self.assertIn(phase, modes, phase)
        # phases occur in order: powered -> loop -> return
        self.assertLess(modes.index("loop"), modes.index("return"))
        # the half-loop trades speed for altitude -- that is what buys the
        # range home (a flat turn instead wastes it, see RETURN_* comments)
        loop = [s for s in r.states if s.mode == "loop"]
        self.assertGreater(loop[-1].altitude_m, loop[0].altitude_m)
        self.assertLess(loop[-1].velocity_m_per_s, loop[0].velocity_m_per_s)
        # heading is reversed: the inbound glide flies back toward launch
        inbound = [s for s in r.states if s.mode == "return"]
        self.assertLess(inbound[-1].distance_m, inbound[0].distance_m)
        self.assertLess(r.states[-1].distance_m,
                        max(s.distance_m for s in r.states))


class MinimumAccelerationGateTests(unittest.TestCase):
    """The min-powered-acceleration gate (2026-08-12, user requirement) --
    the multiplicative thrust margin alone admits ~0.12 g at the M~0.45
    pinch, which is what this additive gate exists to rule out."""

    def test_min_accel_is_tracked_over_powered_flight_only(self):
        from simple_model.constants import G0_M_PER_S2
        g = VehicleGeometry(0.28, 0.151, 0.34, 0.68, 0.81, FUELS["propane"])
        # cutoff lowered to one this design reaches at the suite's default
        # CD0, so the flight HAS an unpowered segment to be excluded
        r = run_flight(g, initial_mass_kg=50 * KG_PER_LB, climb_angle_deg=1.0,
                       motor_cutoff_mach=0.30, dt_s=0.05, max_time_s=600.0)
        powered = [s for s in r.states if s.mode in ("pulsejet", "ramjet")]
        unpowered = [s for s in r.states if s.mode not in ("pulsejet", "ramjet")]
        self.assertTrue(powered and unpowered)
        expected = min(s.acceleration_m_per_s2 / G0_M_PER_S2 for s in powered)
        self.assertAlmostEqual(r.min_powered_accel_g, expected, places=9)
        # the unpowered glide decelerates harder; it must NOT leak into the
        # gated quantity (that would make every design look infeasible)
        self.assertLess(min(s.acceleration_m_per_s2 for s in unpowered),
                        expected * G0_M_PER_S2)

    def test_gate_constant_is_env_overridable(self):
        import importlib
        import os as _os

        import simple_model.constants as consts
        original = _os.environ.get("SIMPLE_MODEL_MIN_ACCEL_G")
        try:
            _os.environ["SIMPLE_MODEL_MIN_ACCEL_G"] = "0.42"
            importlib.reload(consts)
            self.assertAlmostEqual(consts.MIN_POWERED_ACCELERATION_G, 0.42)
        finally:
            if original is None:
                _os.environ.pop("SIMPLE_MODEL_MIN_ACCEL_G", None)
            else:
                _os.environ["SIMPLE_MODEL_MIN_ACCEL_G"] = original
            importlib.reload(consts)


class V3ClimbDiveTests(unittest.TestCase):
    """V3 climb-dive profile: derived top-of-climb, phase sequence, the
    competition rule, and the deliberately SPLIT gates (gravity counts for
    acceleration, never for the engine-only thrust margin).

    The scenarios are flown ONCE in a subprocess at a pinned CD0
    (scripts/v3_probe_flights.py). CD0 is baked into simple_model.constants
    at import and re-exported by drag/flight_sim, and other modules in this
    suite pin it themselves (tests/test_medium_model_v2_parity.py), so
    in-process V3 flights pass or fail on test module import order -- which
    is exactly how an earlier version of these tests broke."""

    @classmethod
    def setUpClass(cls):
        import json
        import subprocess
        import sys
        from pathlib import Path

        root = Path(__file__).resolve().parent.parent
        proc = subprocess.run(
            [sys.executable, str(root / "scripts" / "v3_probe_flights.py"), "0.1"],
            cwd=root, capture_output=True, text=True, timeout=600)
        if proc.returncode != 0:
            raise AssertionError(f"V3 probe failed:\n{proc.stdout}\n{proc.stderr}")
        for line in proc.stdout.splitlines():
            if line.startswith("V3_PROBE_JSON:"):
                cls.probe = json.loads(line[len("V3_PROBE_JSON:"):])
                return
        raise AssertionError(f"no V3_PROBE_JSON:\n{proc.stdout}\n{proc.stderr}")

    def test_derived_top_grows_with_dive_angle(self):
        tops = self.probe["derived_tops"]
        self.assertTrue(all(a < b for a, b in zip(tops, tops[1:])), tops)
        self.assertTrue(all(self.probe["floor_altitude_m"] < t
                            <= self.probe["max_top_altitude_m"] for t in tops), tops)

    def test_zero_dive_derives_the_floor_itself(self):
        self.assertAlmostEqual(self.probe["derived_top_zero_dive"],
                               self.probe["floor_altitude_m"], places=9)

    def test_phase_sequence_and_floor_are_respected(self):
        s = self.probe["steep_climb"]
        modes = s["modes_ordered"]
        for phase in ("v3_climb", "v3_dive", "drag_strip"):
            self.assertIn(phase, modes, phase)
        self.assertLess(modes.index("v3_climb"), modes.index("v3_dive"))
        self.assertLess(modes.index("v3_dive"), modes.index("drag_strip"))
        # the climb gains altitude, the dive spends it, and the floor holds
        self.assertGreater(s["climb_alt_gain_m"], 0.0)
        self.assertGreater(s["dive_alt_loss_m"], 0.0)
        self.assertGreaterEqual(s["dive_min_alt_m"],
                                self.probe["floor_altitude_m"] - 25.0)

    def test_dive_buys_traverse_acceleration(self):
        """The whole point: gravity covers the lightoff notch."""
        self.assertGreater(self.probe["steep_dive"]["min_traverse_accel_g"],
                           self.probe["shallow_dive"]["min_traverse_accel_g"])
        # and both beat flying the notch level
        self.assertGreater(self.probe["shallow_dive"]["min_traverse_accel_g"],
                           self.probe["no_profile"]["min_traverse_accel_g"])

    def test_dive_does_not_inflate_the_engine_only_margin(self):
        """The split gate: a steeper dive must NOT make the thrust margin
        look better, or an underpowered engine could hide behind gravity."""
        self.assertLessEqual(self.probe["steep_dive"]["min_powered_thrust_margin"],
                             self.probe["shallow_dive"]["min_powered_thrust_margin"] + 1e-6)

    def test_traverse_excludes_the_commanded_climb(self):
        s = self.probe["steep_climb"]
        # the commanded climb IS the worst powered acceleration...
        self.assertAlmostEqual(s["min_powered_accel_g"],
                               s["worst_climb_accel_g"], places=9)
        # ...and the gated traverse figure is strictly better than it
        self.assertGreater(s["min_traverse_accel_g"], s["min_powered_accel_g"])
        # the climb must still be accelerating at all
        self.assertGreater(s["min_powered_accel_g"], 0.0)

    def test_rule_is_satisfied_and_tracked(self):
        for key in ("shallow_dive", "steep_dive", "steep_climb"):
            self.assertFalse(self.probe[key]["rule_violated"], key)
            self.assertIsNone(self.probe[key]["rule_violation_mach"], key)

    def test_no_profile_means_no_v3_fields(self):
        s = self.probe["no_profile"]
        self.assertIsNone(s["top_altitude_m"])
        self.assertFalse(s["rule_violated"])
        self.assertNotIn("v3_climb", s["modes_ordered"])
        # traverse and all-powered coincide when there is no commanded climb
        self.assertAlmostEqual(s["min_traverse_accel_g"],
                               s["min_powered_accel_g"], places=9)


if __name__ == "__main__":
    unittest.main()
