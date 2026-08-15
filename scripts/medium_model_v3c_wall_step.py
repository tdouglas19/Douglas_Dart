"""The DRAG STEP at the lightoff Mach -- how much of the wall is it?

flight_sim decides the spillage term from whether the ramjet is "lit"
(`_duct_swallowed_kg_per_s`: net_thrust > 0 -> the duct is throat-limited,
else it is a cold straight-through pipe).  But ramjet_simple ramps thrust
linearly over RAMJET_LIGHTOFF_RAMP_MACH = 0.10 above the gate, so the
instant M crosses the gate the duct becomes fully restrictive while the
engine is still making ~zero thrust.  That is a step increase in drag
landing exactly at the gate Mach -- which is exactly where the minimum
traverse acceleration lives.

This prints the fine trace across the crossing and sizes the step in g's.
"""
from __future__ import annotations
import os
from math import pi
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                       # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere       # noqa: E402
from medium_model.design import fly, load_frozen_design       # noqa: E402
from medium_model.drag_buildup import spillage_drag_n         # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile          # noqa: E402
from medium_model.ramjet_simple import ramjet_thrust          # noqa: E402

G0 = 9.80665


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    lip = pi * d.geometry.throat_diameter_m ** 2 / 4.0
    try:
        for gate in (0.40, 0.45, 0.50):
            rs.RAMJET_MIN_LIGHTOFF_MACH = gate
            r = fly(d, drag_model="buildup",
                    climb_dive=ClimbDiveProfile(initial_climb_angle_deg=12.0,
                                                dive_angle_deg=20.0,
                                                floor_altitude_m=122.0))
            near = [s for s in r.states
                    if abs(s.mach - gate) < 0.004 and s.mode != "v3_climb"]
            print(f"\n=== gate {gate:.2f}, climb 12 / dive 20 -- "
                  f"crossing trace (dt 0.02 s) ===")
            print(f"{'M':>7} {'alt':>6} {'T':>7} {'D':>7} {'a/g':>7} "
                  f"{'dD':>6}")
            prevD = None
            for s in near[::3]:
                dd = "" if prevD is None else f"{s.drag_n - prevD:+6.2f}"
                print(f"{s.mach:7.4f} {s.altitude_m:6.0f} {s.thrust_n:7.2f} "
                      f"{s.drag_n:7.2f} {s.acceleration_m_per_s2/G0:7.4f} "
                      f"{dd:>6}")
                prevD = s.drag_n
            print(f"min traverse {r.min_traverse_accel_g:.4f} g "
                  f"at M {r.min_traverse_accel_mach:.4f}")

            # size the step analytically at the gate condition
            smin = min((s for s in r.states if s.mode in
                        ("v3_dive", "drag_strip")),
                       key=lambda s: s.acceleration_m_per_s2)
            atm = standard_atmosphere(smin.altitude_m)
            v = smin.mach * atm.speed_of_sound_m_per_s
            rj = ramjet_thrust(d.geometry.diameter_m,
                               d.geometry.throat_diameter_m, smin.mach,
                               smin.altitude_m, d.geometry.fuel,
                               atmosphere=atm)
            cold = 0.90 * atm.density_kg_per_m3 * v * lip
            hot = rj.air_mass_flow_kg_per_s
            sp_cold = spillage_drag_n(cold, lip, atm.density_kg_per_m3, v, 0.92)
            sp_hot = spillage_drag_n(hot, lip, atm.density_kg_per_m3, v, 0.92)
            step_g = (sp_hot - sp_cold) / (smin.mass_kg * G0)
            print(f"at the min step (M {smin.mach:.4f}, {smin.altitude_m:.0f} m): "
                  f"duct mdot cold {cold:.3f} -> hot {hot:.3f} kg/s; "
                  f"spillage {sp_cold:.2f} -> {sp_hot:.2f} N "
                  f"= {step_g:+.4f} g")
            print(f"  ramjet thrust actually delivered there: "
                  f"{rj.net_thrust_n:.2f} N "
                  f"(ramp factor {(smin.mach-gate)/0.10:.3f})")
            print(f"  traverse without the step would be "
                  f"{r.min_traverse_accel_g + step_g:.4f} g")
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig


if __name__ == "__main__":
    main()
