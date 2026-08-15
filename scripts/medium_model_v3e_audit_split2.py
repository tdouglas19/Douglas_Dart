"""ADVERSARIAL AUDIT part 5b -- the credit split, done fairly.

Part 5's pin was computed with the module-default lightoff gate (0.45),
so every pinned top sat BELOW every gate-0.50 derived top and all four
dive angles were truncated.  Redone here with pins at and above the
LARGEST derived top at the gate under test, so nothing is truncated and
only sin(dive) differs.

Also: the one-variable sweep that settles it -- fix the dive angle and
raise the top alone.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_split2.py
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
GATE = 0.50
CLIMB = 12.0
DIVES = (14.0, 20.0, 25.0, 30.0)


def row(d, climb, dive, gate, top=None, floor=FLOOR_M):
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


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    rs.RAMJET_MIN_LIGHTOFF_MACH = GATE
    try:
        tops = {dv: derive_top_altitude(
            d.geometry, d.wing, WET,
            ClimbDiveProfile(initial_climb_angle_deg=CLIMB, dive_angle_deg=dv,
                             floor_altitude_m=FLOOR_M)) for dv in DIVES}
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    print(f"=== derived tops AT gate {GATE} (climb {CLIMB}) ===")
    for k, v in tops.items():
        print(f"  dive {k:4.1f} -> {v:7.1f} m")
    pins = (round(max(tops.values()), 1), 1300.0, 1500.0)
    print(f"\n=== dive angle at COMMON tops {pins} (nothing truncated) ===")
    print(f"{'dive':>5} {'own_top':>8} {'own_tv':>7} {'own_out':>8}  |  "
          + "  ".join(f"{'pin%d' % p:>7}{'':>1}{'out':>6}" for p in pins))
    rows = []
    for dv in DIVES:
        a = row(d, CLIMB, dv, GATE)
        cells = []
        pinned = []
        for p in pins:
            b = row(d, CLIMB, dv, GATE, top=p)
            pinned.append(b)
            cells.append(f"{b['traverse']:7.3f} {b['dive_out']:6.3f}")
        rows.append(dict(dive=dv, own=a, pinned=pinned))
        print(f"{dv:5.1f} {a['top_used']:8.1f} {a['traverse']:7.3f} "
              f"{a['dive_out']:8.3f}  |  " + "  ".join(cells), flush=True)

    print("\n=== one variable at a time: dive 20 FIXED, top raised alone ===")
    print(f"{'top_m':>7} {'trav':>7} {'divOut':>7} {'mode':>11} {'peakM':>6} "
          f"{'fuel':>6} {'pass':>5}")
    ladder = []
    for top in (772.5, 900.0, 1053.1, 1200.0, 1350.0, 1500.0):
        r = row(d, CLIMB, 20.0, GATE, top=top)
        ladder.append(r)
        print(f"{top:7.1f} {r['traverse']:7.3f} {r['dive_out']:7.3f} "
              f"{str(r['trav_mode']):>11} {r['peak']:6.3f} {r['fuel']:6.3f} "
              f"{str(r['passes']):>5}", flush=True)

    print("\n=== and the mirror: top FIXED at 1053.1 m, dive angle swept ===")
    print(f"{'dive':>5} {'trav':>7} {'divOut':>7} {'mode':>11} {'pass':>5}")
    mirror = []
    for dv in (10.0, 14.0, 20.0, 25.0, 30.0, 40.0):
        r = row(d, CLIMB, dv, GATE, top=1053.1)
        mirror.append(r)
        print(f"{dv:5.1f} {r['traverse']:7.3f} {r['dive_out']:7.3f} "
              f"{str(r['trav_mode']):>11} {str(r['passes']):>5}", flush=True)

    (OUT / "v3e_audit_split2.json").write_text(json.dumps(
        dict(tops={str(k): v for k, v in tops.items()}, pins=list(pins),
             rows=rows, ladder=ladder, mirror=mirror), indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_split2.json'}")


if __name__ == "__main__":
    main()
