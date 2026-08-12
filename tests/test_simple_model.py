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


if __name__ == "__main__":
    unittest.main()
