"""ADVERSARIAL AUDIT part 1 -- what the trace actually says.

Independent of both prior agents.  For each (climb, dive, gate) it
re-flies and reports, from the STATE LIST rather than from any summary
field:

  * where min_traverse actually sits: mode, Mach, altitude
  * whether the vehicle ever reached the derived top (v3_climb latched)
  * dive entry/exit Mach + altitude, and why the dive ended
    (Mach 0.60 vs floor clamp)
  * the mode and altitude at the instant Mach crosses the lightoff gate
  * a traced flight-path angle from the integrated altitude history

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_trace.py
"""
from __future__ import annotations

import json
import os
from math import asin, degrees
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                        # noqa: E402
from medium_model.design import fly, load_frozen_design        # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile           # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
FLOOR_M = 122.0
TARGET_G = 0.26

CASES = [
    # (climb, dive, gate)   -- the corners both reports lean on
    (12.0, 9.890058542648028, 0.40),
    (12.0, 9.890058542648028, 0.45),
    (14.0, 14.0, 0.45),
    (14.0, 14.0, 0.50),
    (12.0, 20.0, 0.45),
    (12.0, 20.0, 0.50),
    (14.0, 20.0, 0.50),
    (16.0, 20.0, 0.50),
    (12.0, 25.0, 0.50),
    (12.0, 30.0, 0.50),
    (12.0, 30.0, 0.55),
]


def analyse(r, gate, cd):
    st = r.states
    powered = [s for s in st if s.mode in ("pulsejet", "ramjet", "v3_climb",
                                           "v3_dive", "drag_strip")]
    trav = [s for s in powered if s.mode != "v3_climb"]
    # min traverse recomputed from the trace, not read off the summary
    mt = min(trav, key=lambda s: s.acceleration_m_per_s2) if trav else None
    climb = [s for s in st if s.mode == "v3_climb"]
    dive = [s for s in st if s.mode == "v3_dive"]
    strip = [s for s in st if s.mode == "drag_strip"]
    top_reached = max((s.altitude_m for s in st), default=0.0)

    # instant the lightoff gate is crossed, whatever mode that is
    cross = None
    for s in powered:
        if s.mach >= gate:
            cross = s
            break

    # traced flight-path angle over M 0.80..1.10 powered steps
    worst_gamma = None
    for a, b in zip(st, st[1:]):
        if b.mode not in ("pulsejet", "ramjet", "v3_climb", "v3_dive",
                          "drag_strip"):
            continue
        if not (0.80 <= a.mach <= 1.10):
            continue
        dh = b.altitude_m - a.altitude_m
        ds = max(a.velocity_m_per_s * (b.time_s - a.time_s), 1e-9)
        g = degrees(asin(max(min(dh / ds, 1.0), -1.0)))
        if worst_gamma is None or g < worst_gamma:
            worst_gamma = g

    return dict(
        gate=gate, climb=cd.initial_climb_angle_deg, dive=cd.dive_angle_deg,
        derived_top_m=r.climb_dive_top_altitude_m,
        max_alt_m=top_reached,
        climb_latched=bool(dive or strip),
        climb_top_alt_m=(max(s.altitude_m for s in climb) if climb else None),
        climb_end_mach=(climb[-1].mach if climb else None),
        climb_reached_derived_top=(
            bool(climb) and r.climb_dive_top_altitude_m is not None
            and max(s.altitude_m for s in climb)
            >= r.climb_dive_top_altitude_m - 1.0),
        dive_n=len(dive), strip_n=len(strip),
        dive_entry_mach=(dive[0].mach if dive else None),
        dive_entry_alt=(dive[0].altitude_m if dive else None),
        dive_exit_mach=(dive[-1].mach if dive else None),
        dive_exit_alt=(dive[-1].altitude_m if dive else None),
        dive_end_reason=(None if not dive else
                         ("mach0.60" if dive[-1].mach >= cd.dive_end_mach
                          else "floor")),
        min_trav_g=(mt.acceleration_m_per_s2 / 9.80665 if mt else None),
        min_trav_mach=(mt.mach if mt else None),
        min_trav_mode=(mt.mode if mt else None),
        min_trav_alt=(mt.altitude_m if mt else None),
        summary_trav_g=r.min_traverse_accel_g,
        summary_trav_mach=r.min_traverse_accel_mach,
        cross_mode=(cross.mode if cross else None),
        cross_alt=(cross.altitude_m if cross else None),
        cross_mach=(cross.mach if cross else None),
        peak_mach=max(s.mach for s in st),
        cutoff=bool(r.motor_cutoff_reached),
        fuel=max(s.fuel_burned_kg for s in st),
        margin=r.min_powered_thrust_margin,
        rule_violated=bool(r.rule_violated),
        traced_min_gamma_deg=worst_gamma,
    )


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    rows = []
    hdr = (f"{'gate':>5} {'clm':>5} {'div':>5} {'topDER':>7} {'topCLB':>7} "
           f"{'reach':>5} {'divM_in':>7} {'divH_in':>7} {'divM_out':>8} "
           f"{'end':>6} {'minTv':>6} {'@M':>5} {'@mode':>10} {'@h':>6} "
           f"{'crossMode':>10} {'crossH':>7} {'peakM':>6} {'fuel':>6} "
           f"{'gam80':>6}")
    print(hdr)
    try:
        for climb, dive, gate in CASES:
            rs.RAMJET_MIN_LIGHTOFF_MACH = gate
            cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                  dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
            r = fly(d, drag_model="buildup", climb_dive=cd)
            a = analyse(r, gate, cd)
            rows.append(a)
            print(f"{gate:5.2f} {climb:5.1f} {dive:5.1f} "
                  f"{a['derived_top_m']:7.1f} "
                  f"{(a['climb_top_alt_m'] or 0):7.1f} "
                  f"{str(a['climb_reached_derived_top'])[:5]:>5} "
                  f"{(a['dive_entry_mach'] or 0):7.3f} "
                  f"{(a['dive_entry_alt'] or 0):7.1f} "
                  f"{(a['dive_exit_mach'] or 0):8.3f} "
                  f"{str(a['dive_end_reason']):>6} "
                  f"{(a['min_trav_g'] or 0):6.3f} "
                  f"{(a['min_trav_mach'] or 0):5.3f} "
                  f"{str(a['min_trav_mode']):>10} "
                  f"{(a['min_trav_alt'] or 0):6.1f} "
                  f"{str(a['cross_mode']):>10} {(a['cross_alt'] or 0):7.1f} "
                  f"{a['peak_mach']:6.3f} {a['fuel']:6.3f} "
                  f"{(a['traced_min_gamma_deg'] if a['traced_min_gamma_deg'] is not None else float('nan')):6.2f}",
                  flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    (OUT / "v3e_audit_trace.json").write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_trace.json'}")


if __name__ == "__main__":
    main()
