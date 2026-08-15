"""Thrust / drag / gravity balance for the V3a duct, pulsejet ALONE.

Defines the one number the wall is made of:

    e(M, h) = (T_pulsejet - D_vehicle) / W        [g's, "specific excess"]

Dive acceleration is then simply  a = e + sin(gamma_dive), and the Gate-3
traverse test is  a >= 0.26 g.  So for each dive angle there is a ceiling
Mach where e falls to (0.26 - sin gamma) and the gate stops being passable
by thrust at all; above that, the only thing still accelerating the vehicle
is the altitude in the bank, and the 400 ft floor says how much of that
there is.

Also prints:
  * the derived top-of-climb vs ramjet gate -- the altitude BUDGET, which
    is the second (and, it turns out, binding) ceiling;
  * the ramjet's own lightoff capability vs Mach, to settle whether the
    ramjet is the limit (it is not).

Closed-form propulsion + the real drag build-up. Seconds, not minutes.
"""
from __future__ import annotations
import json, os
from math import pi, radians, sin
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                       # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere       # noqa: E402
from medium_model.constants import (NOSE_TAIL_LENGTH_DIAMETERS,  # noqa: E402
                                    TAIL_LENGTH_DIAMETERS)
from medium_model.design import fly, load_frozen_design        # noqa: E402
from medium_model.drag_buildup import total_drag_buildup       # noqa: E402
from medium_model.flight_sim import (ClimbDiveProfile,         # noqa: E402
                                     derive_top_altitude)
from medium_model.pulsejet_simple import pulsejet_thrust       # noqa: E402
from medium_model.ramjet_simple import ramjet_thrust           # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
G0 = 9.80665
FLOOR_M = 122.0
TARGET_G = 0.26
MACHS = (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60)
ALTS = (130.0, 300.0, 600.0, 900.0, 1200.0)
DIVES = (9.890058542648028, 10.0, 14.0, 20.0)


def geom_cache(d):
    g = d.geometry
    w = d.wing
    return dict(
        diameter_m=g.diameter_m, duct_exit_diameter_m=g.throat_diameter_m,
        tail_length_m=TAIL_LENGTH_DIAMETERS * g.diameter_m,
        body_length_m=(g.chamber_length_m + g.throat_length_m
                       + NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m),
        wing_area_m2=w.span_m ** 2 / w.aspect_ratio,
        aspect_ratio=w.aspect_ratio, wing_sweep_deg=w.sweep_deg,
        wing_thickness_ratio=getattr(w.airfoil, "thickness_ratio", 0.03),
        oswald_e=w.oswald_e,
        lip_area_m2=pi * g.throat_diameter_m ** 2 / 4.0)


def balance(d, gc, mach, alt, mass_kg, gamma_rad):
    from math import cos
    atm = standard_atmosphere(alt)
    v = mach * atm.speed_of_sound_m_per_s
    pj = pulsejet_thrust(d.geometry.diameter_m, d.geometry.chamber_length_m,
                         d.geometry.throat_diameter_m,
                         d.geometry.throat_length_m, mach, alt,
                         d.geometry.fuel, atmosphere=atm)
    b = total_drag_buildup(
        diameter_m=gc["diameter_m"], body_length_m=gc["body_length_m"],
        duct_exit_diameter_m=gc["duct_exit_diameter_m"],
        tail_length_m=gc["tail_length_m"],
        wing_reference_area_m2=gc["wing_area_m2"],
        wing_thickness_ratio=gc["wing_thickness_ratio"],
        wing_sweep_deg=gc["wing_sweep_deg"], velocity_m_per_s=v,
        density_kg_per_m3=atm.density_kg_per_m3,
        temperature_k=atm.temperature_k, mach=mach,
        required_lift_n=mass_kg * G0 * cos(gamma_rad),
        oswald_efficiency=gc["oswald_e"], wing_aspect_ratio=gc["aspect_ratio"],
        engine_on=True,
        captured_mass_flow_kg_per_s=0.90 * atm.density_kg_per_m3 * v
        * gc["lip_area_m2"],
        lip_area_m2=gc["lip_area_m2"], cowl_suction_recovery=None)
    return pj.average_thrust_n, b.total_n, v, atm


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    gc = geom_cache(d)
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH

    # representative in-dive mass, measured (not assumed) from the flight
    # that actually passes: climb 14 / dive 20, gate 0.45
    rs.RAMJET_MIN_LIGHTOFF_MACH = 0.45
    ref = fly(d, drag_model="buildup",
              climb_dive=ClimbDiveProfile(initial_climb_angle_deg=14.0,
                                          dive_angle_deg=20.0,
                                          floor_altitude_m=FLOOR_M))
    dive_states = [s for s in ref.states if s.mode == "v3_dive"]
    m_mid = sum(s.mass_kg for s in dive_states) / len(dive_states)
    w_mid = m_mid * G0
    print(f"mid-dive mass {m_mid:.2f} kg -> W = {w_mid:.1f} N "
          f"(wet 22.68 kg = 222.4 N)")
    print(f"dive M->alt actually flown (gate 0.45, dive 20 deg):")
    for target in MACHS:
        near = min(dive_states, key=lambda s: abs(s.mach - target))
        if abs(near.mach - target) < 0.02:
            print(f"   M {target:.2f} -> {near.altitude_m:5.0f} m")

    # ---------------- Table 1: e(M, h), pulsejet alone ------------------
    print(f"\n=== e(M,h) = (T_pulsejet - D)/W, g's -- CLOSED FORM ===")
    print(f"(dive accel = e + sin(gamma); gate needs >= {TARGET_G} g)")
    hdr = "  M   " + "".join(f"{a:>10.0f}m" for a in ALTS)
    print(hdr)
    grid = {}
    for mach in MACHS:
        cells = []
        for alt in ALTS:
            T, D, v, atm = balance(d, gc, mach, alt, m_mid,
                                   -radians(20.0))
            e = (T - D) / w_mid
            grid[(mach, alt)] = dict(T=T, D=D, e=e)
            cells.append(f"{e:+11.3f}")
        print(f"{mach:5.2f} " + "".join(cells))

    print(f"\n=== the same, as raw N: T_pulsejet / D_vehicle ===")
    print(hdr)
    for mach in MACHS:
        print(f"{mach:5.2f} " + "".join(
            f"{grid[(mach,a)]['T']:5.0f}/{grid[(mach,a)]['D']:<5.0f}"
            for a in ALTS))

    # ------------- ceilings: where e + sin(dive) crosses --------------
    def cross(alt, want):
        """finest Mach where e(M,alt) drops through `want` (linear scan)."""
        prev_m, prev_e = None, None
        m = 0.28
        while m <= 0.80001:
            T, D, _, _ = balance(d, gc, m, alt, m_mid, -radians(20.0))
            e = (T - D) / w_mid
            if prev_e is not None and prev_e >= want > e:
                f = (prev_e - want) / (prev_e - e)
                return prev_m + f * (m - prev_m)
            prev_m, prev_e = m, e
            m += 0.005
        return float("nan")

    print(f"\n=== THRUST-BALANCE ceilings (pulsejet alone) ===")
    print(f"{'alt_m':>6} {'M(T=D)':>8} " + "".join(
        f"{'M_gate@' + f'{dv:.0f}deg':>14}" for dv in (10.0, 14.0, 20.0)))
    ceil = {}
    for alt in ALTS:
        row = [f"{alt:6.0f} {cross(alt, 0.0):8.3f} "]
        for dv in (10.0, 14.0, 20.0):
            want = TARGET_G - sin(radians(dv))
            mc = cross(alt, want)
            ceil[(alt, dv)] = mc
            row.append(f"{mc:14.3f}")
        print("".join(row))

    # -------------- Table 3: the ALTITUDE BUDGET runaway ---------------
    print(f"\n=== derived top-of-climb vs ramjet gate (the altitude bill) ===")
    print(f"{'gate':>5} " + "".join(f"{'dive ' + f'{dv:.1f}':>12}"
                                    for dv in DIVES))
    tops = {}
    for gate in (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60):
        rs.RAMJET_MIN_LIGHTOFF_MACH = gate
        cells = []
        for dv in DIVES:
            p = ClimbDiveProfile(initial_climb_angle_deg=14.0,
                                 dive_angle_deg=dv, floor_altitude_m=FLOOR_M)
            top = derive_top_altitude(d.geometry, d.wing, 22.68, p)
            tops[(gate, dv)] = top
            cells.append(f"{top:12.0f}")
        print(f"{gate:5.2f} " + "".join(cells))

    # ---------- Table 4: is the RAMJET the limit? ----------------------
    rs.RAMJET_MIN_LIGHTOFF_MACH = 0.05        # ungated: raw capability
    print(f"\n=== ramjet raw capability, gate removed (closed form) ===")
    print(f"{'M':>5} {'alt':>6} {'net_N':>8} {'mdot_air':>9} {'Tc_K':>7} "
          f"{'choked':>7}")
    for mach in (0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
        for alt in (300.0, 900.0):
            r = ramjet_thrust(d.geometry.diameter_m,
                              d.geometry.throat_diameter_m, mach, alt,
                              d.geometry.fuel)
            print(f"{mach:5.2f} {alt:6.0f} {r.net_thrust_n:8.1f} "
                  f"{r.air_mass_flow_kg_per_s:9.3f} "
                  f"{r.chamber_total_temperature_k:7.0f} "
                  f"{str(r.choked):>7}")
    rs.RAMJET_MIN_LIGHTOFF_MACH = orig

    (OUT / "v3c_wall_balance.json").write_text(json.dumps(dict(
        mid_dive_mass_kg=m_mid, weight_n=w_mid,
        e_grid={f"{m}|{a}": grid[(m, a)] for m in MACHS for a in ALTS},
        ceilings={f"{a}|{dv}": ceil[(a, dv)] for a in ALTS
                  for dv in (10.0, 14.0, 20.0)},
        tops={f"{g}|{dv}": tops[(g, dv)] for g in
              (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60) for dv in DIVES},
    ), indent=2))
    print(f"\nwrote {OUT / 'v3c_wall_balance.json'}")


if __name__ == "__main__":
    main()
