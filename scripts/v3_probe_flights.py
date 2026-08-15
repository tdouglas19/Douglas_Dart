"""Fly a fixed set of V3 climb-dive scenarios at a PINNED CD0 and print the
results as JSON, for tests/test_simple_model.py's V3 assertions.

Why a script: CD0 is baked into simple_model.constants at import time and
re-exported by drag/flight_sim, so an in-process override does nothing once
any other test module has imported simple_model -- and the suite contains
modules that pin CD0 themselves (tests/test_medium_model_v2_parity.py).
Running the scenarios in their own process makes the V3 behaviour tests
independent of both CD0 and test module import order.

Usage: python scripts/v3_probe_flights.py [cd0]
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
CD0 = sys.argv[1] if len(sys.argv) > 1 else "0.1"
os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = CD0

from simple_model.constants import FUELS, G0_M_PER_S2, KG_PER_LB  # noqa: E402
from simple_model.flight_sim import (  # noqa: E402
    ClimbDiveProfile, V3_FLOOR_ALTITUDE_M, V3_MAX_TOP_ALTITUDE_M,
    VehicleGeometry, derive_top_altitude, run_flight)

GEOM = VehicleGeometry(0.28, 0.151, 0.34, 0.68, 0.81, FUELS["propane"])
MASS = 50 * KG_PER_LB


def fly(climb_deg=None, dive_deg=None):
    profile = (None if climb_deg is None else
               ClimbDiveProfile(initial_climb_angle_deg=climb_deg,
                                dive_angle_deg=dive_deg))
    r = run_flight(GEOM, MASS, climb_angle_deg=1.0, motor_cutoff_mach=1.1,
                   dt_s=0.02, max_time_s=900.0, climb_dive=profile)
    modes = [s.mode for s in r.states]
    climb = [s for s in r.states if s.mode == "v3_climb"]
    dive = [s for s in r.states if s.mode == "v3_dive"]
    return {
        "modes_ordered": sorted(set(modes), key=modes.index),
        "min_traverse_accel_g": r.min_traverse_accel_g,
        "min_powered_accel_g": r.min_powered_accel_g,
        "min_powered_thrust_margin": r.min_powered_thrust_margin,
        "rule_violated": r.rule_violated,
        "rule_violation_mach": r.rule_violation_mach,
        "top_altitude_m": r.climb_dive_top_altitude_m,
        "motor_cutoff_reached": r.motor_cutoff_reached,
        "safe_landing": r.safe_landing,
        "climb_alt_gain_m": (climb[-1].altitude_m - climb[0].altitude_m) if climb else None,
        "dive_alt_loss_m": (dive[0].altitude_m - dive[-1].altitude_m) if dive else None,
        "dive_min_alt_m": min((s.altitude_m for s in dive), default=None),
        "worst_climb_accel_g": (min(s.acceleration_m_per_s2 for s in climb) / G0_M_PER_S2
                                if climb else None),
    }


out = {
    "cd0": float(CD0),
    "floor_altitude_m": V3_FLOOR_ALTITUDE_M,
    "max_top_altitude_m": V3_MAX_TOP_ALTITUDE_M,
    "no_profile": fly(),
    "shallow_dive": fly(15.0, 5.0),
    "steep_dive": fly(15.0, 20.0),
    "steep_climb": fly(25.0, 15.0),
    "derived_tops": [
        derive_top_altitude(GEOM, None, MASS,
                            ClimbDiveProfile(initial_climb_angle_deg=15.0,
                                             dive_angle_deg=d))
        for d in (5.0, 10.0, 15.0)
    ],
    "derived_top_zero_dive": derive_top_altitude(
        GEOM, None, MASS,
        ClimbDiveProfile(initial_climb_angle_deg=15.0, dive_angle_deg=0.0)),
}
print("V3_PROBE_JSON:" + json.dumps(out))
