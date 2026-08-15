"""ADVERSARIAL AUDIT part 5 -- split the dive-angle credit, and push the
dive past 30 deg to find where the real bound is.

1. CREDIT SPLIT.  At gate 0.50 fly each dive angle three ways:
     (a) its own derived top          -- what the campaign does
     (b) a top pinned to the SHALLOWEST dive's derived top
     (c) a top pinned to the STEEPEST dive's derived top
   (b) removes the altitude-budget channel; (c) gives every dive angle the
   same generous budget so only sin(dive) differs.  The gap between (a)
   and (c) is the altitude-budget credit; the spread inside (c) is the
   pure gravity credit.

2. HOW FAR DOES DIVE ANGLE GO?  Fine gate bisection at dive 30/35/40/45,
   climbs 6-14, to test "dive 20 is not the bound" and to locate the
   actual bound.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_split.py
"""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                          # noqa: E402
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.flight_sim import (ClimbDiveProfile,           # noqa: E402
                                     derive_top_altitude)

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0
WET = 22.6796185


def row(d, climb, dive, gate, floor=FLOOR_M, top=None):
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
    dv = [s for s in st if s.mode == "v3_dive"]
    trav = [s for s in st if s.mode in ("pulsejet", "ramjet", "v3_dive",
                                        "drag_strip")]
    mt = min(trav, key=lambda s: s.acceleration_m_per_s2) if trav else None
    peak = max(s.mach for s in st)
    fuel = max(s.fuel_burned_kg for s in st)
    return dict(climb=climb, dive=dive, gate=gate, top_cmd=top,
                top_used=r.climb_dive_top_altitude_m,
                dive_in=(dv[0].mach if dv else None),
                dive_out=(dv[-1].mach if dv else None),
                traverse=r.min_traverse_accel_g,
                trav_mach=(mt.mach if mt else None),
                trav_mode=(mt.mode if mt else None),
                margin=r.min_powered_thrust_margin, peak=peak, fuel=fuel,
                cutoff=bool(r.motor_cutoff_reached),
                passes=bool(r.motor_cutoff_reached and peak >= 1.0
                            and r.min_traverse_accel_g >= TARGET_G
                            and fuel < d.burn_limit_kg - 1e-6))


def ceiling(d, dive, climbs, lo=0.30, hi=0.62, step=0.01):
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    try:
        g = hi
        while g >= lo - 1e-9:
            for c in climbs:
                r = row(d, c, dive, round(g, 3))
                if r["passes"]:
                    return round(g, 3), r
            g -= step
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    return None, None


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    out = {}
    DIVES = (14.0, 20.0, 25.0, 30.0)
    tops = {dv: derive_top_altitude(
        d.geometry, d.wing, WET,
        ClimbDiveProfile(initial_climb_angle_deg=12.0, dive_angle_deg=dv,
                         floor_altitude_m=FLOOR_M)) for dv in DIVES}
    pin_lo, pin_hi = tops[14.0], tops[30.0]

    print("=== 1. credit split, gate 0.50, climb 12 ===")
    print(f"derived tops: " + ", ".join(f"d{int(k)}={v:.0f}m"
                                        for k, v in tops.items()))
    print(f"pin_lo = {pin_lo:.1f} m (dive-14 top), "
          f"pin_hi = {pin_hi:.1f} m (dive-30 top)\n")
    print(f"{'dive':>5} | {'own top':>26} | {'pinned LOW':>26} | "
          f"{'pinned HIGH':>26}")
    print(f"{'':>5} | {'trav':>7}{'divOut':>8}{'mode':>11} | "
          f"{'trav':>7}{'divOut':>8}{'mode':>11} | "
          f"{'trav':>7}{'divOut':>8}{'mode':>11}")
    split = []
    for dv in DIVES:
        a = row(d, 12.0, dv, 0.50)
        b = row(d, 12.0, dv, 0.50, top=pin_lo)
        c = row(d, 12.0, dv, 0.50, top=pin_hi)
        split.append(dict(dive=dv, own=a, pin_lo=b, pin_hi=c))
        print(f"{dv:5.1f} | {a['traverse']:7.3f}{a['dive_out']:8.3f}"
              f"{str(a['trav_mode']):>11} | "
              f"{b['traverse']:7.3f}{b['dive_out']:8.3f}"
              f"{str(b['trav_mode']):>11} | "
              f"{c['traverse']:7.3f}{c['dive_out']:8.3f}"
              f"{str(c['trav_mode']):>11}", flush=True)
    out["split"] = split
    lo14 = split[0]["pin_hi"]["traverse"]
    hi30 = split[-1]["pin_hi"]["traverse"]
    own14 = split[0]["own"]["traverse"]
    own30 = split[-1]["own"]["traverse"]
    print(f"\n  at the COMMON high top, dive 14 -> 30 moves traverse "
          f"{lo14:.3f} -> {hi30:.3f}  (pure gravity credit "
          f"{hi30 - lo14:+.3f} g)")
    print(f"  with each at its OWN derived top,          "
          f"{own14:.3f} -> {own30:.3f}  (total credit "
          f"{own30 - own14:+.3f} g)")
    print(f"  => altitude-budget share of the dive credit: "
          f"{100 * (1 - (hi30 - lo14) / max(own30 - own14, 1e-9)):.0f}%")

    print("\n=== 2. how far does dive angle go? fine ceiling, climbs 6-16 ===")
    CLIMBS = (6.0, 8.0, 10.0, 12.0, 14.0, 16.0)
    print(f"{'dive':>5} {'ceiling':>8} {'climb':>6} {'trav':>6} "
          f"{'margin':>7} {'fuel':>6} {'top_m':>7} {'divOut':>7}")
    ceils = []
    for dv in (20.0, 25.0, 30.0, 35.0, 40.0, 45.0):
        g, r = ceiling(d, dv, CLIMBS)
        ceils.append(dict(dive=dv, ceiling=g, row=r))
        if r:
            print(f"{dv:5.1f} {g:8.2f} {r['climb']:6.1f} "
                  f"{r['traverse']:6.3f} {r['margin']:7.3f} "
                  f"{r['fuel']:6.3f} {r['top_used']:7.1f} "
                  f"{r['dive_out']:7.3f}", flush=True)
        else:
            print(f"{dv:5.1f} {'NONE':>8}", flush=True)
    out["ceilings"] = ceils

    (OUT / "v3e_audit_split.json").write_text(json.dumps(out, indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_split.json'}")


if __name__ == "__main__":
    main()
