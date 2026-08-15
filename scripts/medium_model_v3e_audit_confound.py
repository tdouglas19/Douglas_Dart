"""ADVERSARIAL AUDIT part 3 -- what is really doing the work?

Four separations, each one re-flown rather than argued:

1. LIGHTOFF DRAG STEP.  Drag at M = gate with the duct COLD vs LIT, and
   the ramjet thrust actually delivered at that instant.  Sizes the
   modelling artifact the mechanism report claims.

2. DERIVED-TOP CONFOUND.  Dive angle changes two things at once: the
   gravity term sin(dive) AND the derived top-of-climb.  Fly each dive
   angle at (a) its own derived top and (b) a COMMON pinned top, so the
   two contributions separate.

3. FLOOR-CLAMP CONFOUND.  Vary the floor at fixed dive/gate and watch
   where the min traverse lives.

4. DIVE-BAND MISMATCH.  derive_top_altitude sizes the drop over
   M 0.35 -> 0.60.  Measure the Mach the climb actually tops out at and
   the Mach the dive actually exits at, versus climb angle.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_confound.py
"""
from __future__ import annotations

import json
import os
from math import pi, radians
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                          # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere          # noqa: E402
from medium_model.constants import (G0_M_PER_S2,                 # noqa: E402
                                    NOSE_TAIL_LENGTH_DIAMETERS,
                                    RAMJET_LIGHTOFF_RAMP_MACH,
                                    TAIL_LENGTH_DIAMETERS)
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.flight_sim import (ClimbDiveProfile,           # noqa: E402
                                     _buildup_drag,
                                     derive_top_altitude)
from medium_model.pulsejet_simple import pulsejet_thrust         # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0
DIVE_MASS_KG = 22.04


def make_cache(d):
    g, w = d.geometry, d.wing
    return {
        "diameter_m": g.diameter_m,
        "duct_exit_diameter_m": g.throat_diameter_m,
        "tail_length_m": TAIL_LENGTH_DIAMETERS * g.diameter_m,
        "body_length_m": (g.chamber_length_m + g.throat_length_m
                          + NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m),
        "wing_area_m2": w.span_m ** 2 / w.aspect_ratio,
        "aspect_ratio": w.aspect_ratio,
        "wing_sweep_deg": w.sweep_deg,
        "wing_thickness_ratio": w.airfoil.thickness_ratio,
        "oswald_e": w.oswald_e,
        "lip_area_m2": pi * g.throat_diameter_m ** 2 / 4.0,
        "cowl_suction_recovery": None,
    }


def flight_row(d, climb, dive, gate, floor=FLOOR_M, top=None):
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    try:
        cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                              dive_angle_deg=dive, floor_altitude_m=floor,
                              top_altitude_m=top)
        r = fly(d, drag_model="buildup", climb_dive=cd)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    st = r.states
    powered = [s for s in st if s.mode in ("pulsejet", "ramjet", "v3_climb",
                                           "v3_dive", "drag_strip")]
    trav = [s for s in powered if s.mode != "v3_climb"]
    mt = min(trav, key=lambda s: s.acceleration_m_per_s2) if trav else None
    climb_st = [s for s in st if s.mode == "v3_climb"]
    dive_st = [s for s in st if s.mode == "v3_dive"]
    peak = max(s.mach for s in st)
    fuel = max(s.fuel_burned_kg for s in st)
    return dict(
        climb=climb, dive=dive, gate=gate, floor=floor, top_cmd=top,
        top_used=r.climb_dive_top_altitude_m,
        climb_top_mach=(climb_st[-1].mach if climb_st else None),
        dive_in_mach=(dive_st[0].mach if dive_st else None),
        dive_out_mach=(dive_st[-1].mach if dive_st else None),
        dive_out_alt=(dive_st[-1].altitude_m if dive_st else None),
        traverse=r.min_traverse_accel_g,
        trav_mach=(mt.mach if mt else None),
        trav_mode=(mt.mode if mt else None),
        trav_alt=(mt.altitude_m if mt else None),
        margin=r.min_powered_thrust_margin, peak=peak, fuel=fuel,
        cutoff=bool(r.motor_cutoff_reached),
        passes=bool(r.motor_cutoff_reached and peak >= 1.0
                    and r.min_traverse_accel_g >= TARGET_G
                    and fuel < d.burn_limit_kg - 1e-6))


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    cache = make_cache(d)
    out = {}

    # ---- 1. lightoff drag step -------------------------------------------
    print("=== 1. duct COLD vs LIT at the lightoff instant (300 m, "
          "dive 20, 22.04 kg) ===")
    print(f"ramp width {RAMJET_LIGHTOFF_RAMP_MACH} Mach")
    print(f"{'gate':>5} {'T_rj@+eps':>10} {'D_cold':>8} {'D_lit':>8} "
          f"{'step_N':>7} {'step_g':>7} {'net_g':>7}")
    step_rows = []
    for gate in (0.40, 0.45, 0.50, 0.55):
        m = gate + 1e-3
        alt = 300.0
        atm = standard_atmosphere(alt)
        v = m * atm.speed_of_sound_m_per_s
        g = d.geometry
        orig = rs.RAMJET_MIN_LIGHTOFF_MACH
        rs.RAMJET_MIN_LIGHTOFF_MACH = gate
        try:
            rj = rs.ramjet_thrust(g.diameter_m, g.throat_diameter_m, m, alt,
                                  g.fuel, atmosphere=atm)
        finally:
            rs.RAMJET_MIN_LIGHTOFF_MACH = orig
        cold = 0.90 * atm.density_kg_per_m3 * v * cache["lip_area_m2"]
        lit = rj.air_mass_flow_kg_per_s
        d_cold = _buildup_drag(cache, d.wing, v, atm.density_kg_per_m3,
                               atm.temperature_k, DIVE_MASS_KG,
                               -radians(20.0), m, engine_on=True,
                               captured_mdot_kg_per_s=cold)[0].total_n
        d_lit = _buildup_drag(cache, d.wing, v, atm.density_kg_per_m3,
                              atm.temperature_k, DIVE_MASS_KG,
                              -radians(20.0), m, engine_on=True,
                              captured_mdot_kg_per_s=lit)[0].total_n
        w_n = DIVE_MASS_KG * G0_M_PER_S2
        row = dict(gate=gate, rj_thrust_n=rj.net_thrust_n, d_cold_n=d_cold,
                   d_lit_n=d_lit, step_n=d_lit - d_cold,
                   step_g=(d_lit - d_cold) / w_n,
                   net_g=(rj.net_thrust_n - (d_lit - d_cold)) / w_n,
                   mdot_cold=cold, mdot_lit=lit)
        step_rows.append(row)
        print(f"{gate:5.2f} {rj.net_thrust_n:10.3f} {d_cold:8.2f} "
              f"{d_lit:8.2f} {d_lit - d_cold:7.2f} "
              f"{(d_lit - d_cold) / w_n:7.4f} {row['net_g']:7.4f}")
    out["lightoff_step"] = step_rows

    # ---- 2. derived-top confound ----------------------------------------
    print("\n=== 2. dive angle at its OWN derived top vs a PINNED common "
          "top ===")
    print("gate 0.50, climb 12; pinned top = the dive-20 derived top")
    pin = derive_top_altitude(
        d.geometry, d.wing, 22.6796185,
        ClimbDiveProfile(initial_climb_angle_deg=12.0, dive_angle_deg=20.0,
                         floor_altitude_m=FLOOR_M))
    print(f"pinned top = {pin:.1f} m\n")
    print(f"{'dive':>5} {'topDER':>7} {'trav_der':>9} {'trav_pin':>9} "
          f"{'delta':>7} {'divOut_der':>10} {'divOut_pin':>10}")
    conf = []
    for dv in (14.0, 20.0, 25.0, 30.0):
        a = flight_row(d, 12.0, dv, 0.50)
        b = flight_row(d, 12.0, dv, 0.50, top=pin)
        conf.append(dict(dive=dv, derived=a, pinned=b))
        print(f"{dv:5.1f} {a['top_used']:7.1f} {a['traverse']:9.3f} "
              f"{b['traverse']:9.3f} {b['traverse'] - a['traverse']:+7.3f} "
              f"{a['dive_out_mach']:10.3f} {b['dive_out_mach']:10.3f}",
              flush=True)
    out["derived_top_confound"] = conf

    # ---- 3. floor confound ----------------------------------------------
    print("\n=== 3. floor clamp: gate 0.50, dive 20 ===")
    print(f"{'climb':>6} {'floor':>6} {'trav':>6} {'@M':>6} {'@mode':>11} "
          f"{'divOutM':>8} {'peakM':>6} {'pass':>5}")
    floors = []
    for climb in (12.0, 14.0, 16.0):
        for fl in (122.0, 200.0, 300.0):
            r = flight_row(d, climb, 20.0, 0.50, floor=fl)
            floors.append(r)
            print(f"{climb:6.1f} {fl:6.0f} {r['traverse']:6.3f} "
                  f"{r['trav_mach']:6.3f} {str(r['trav_mode']):>11} "
                  f"{r['dive_out_mach']:8.3f} {r['peak']:6.3f} "
                  f"{str(r['passes']):>5}", flush=True)
    out["floor_confound"] = floors

    # ---- 4. dive-band mismatch ------------------------------------------
    print("\n=== 4. sized dive band (M 0.35 -> 0.60) vs the FLOWN band ===")
    print(f"{'climb':>6} {'dive':>5} {'gate':>5} {'topM':>6} {'inM':>6} "
          f"{'outM':>6} {'flown_band':>11} {'sized_band':>11}")
    bands = []
    for climb in (8.0, 10.0, 12.0, 14.0, 16.0):
        r = flight_row(d, climb, 20.0, 0.50)
        bands.append(r)
        print(f"{climb:6.1f} {20.0:5.1f} {0.50:5.2f} "
              f"{r['climb_top_mach']:6.3f} {r['dive_in_mach']:6.3f} "
              f"{r['dive_out_mach']:6.3f} "
              f"{r['dive_out_mach'] - r['dive_in_mach']:11.3f} "
              f"{0.25:11.3f}", flush=True)
    out["dive_band"] = bands

    (OUT / "v3e_audit_confound.json").write_text(json.dumps(out, indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_confound.json'}")


if __name__ == "__main__":
    main()
