"""Re-pick the CONFIRM configuration under the CONFIRM-tier pulsejet
service ceiling.

New measurement (scripts/medium_model_v3f_fp_ratio.py + the n_cells scans
that followed): the V3a duct FLAMES OUT with altitude, and the ceiling
MOVES DOWN when the grid is refined to the CONFIRM tier --

    n_cells 162 : alive 1275 m, dead 1500 m   (M 0.35)
    n_cells 324 : alive 1250 m, dead 1275 m   (M 0.35)

Every candidate the closed-form screen liked at gate >= 0.52 has a
derived top-of-climb of 1265-1345 m, i.e. AT or OVER the CONFIRM-tier
ceiling.  So the pick must be re-made with a hard constraint on the
maximum altitude the powered flight reaches.

Guard band: max powered altitude <= 1150 m (100 m below the last
sustaining point measured at n_cells 324).

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3f_ceiling_constrained.py
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                      # noqa: E402
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import (V3_FLOOR_ALTITUDE_M,    # noqa: E402
                                     ClimbDiveProfile)

OUT = Path("out_medium_model")
TRAV_HAIRCUT, FUEL_PENALTY, CAP = 0.781, 1.069, 2.423931
ALT_CAP = 1150.0
POWERED = ("v3_climb", "v3_dive", "drag_strip", "pulsejet", "ramjet", "climb")

d = load_frozen_design("docs/v3a_medium_model/design.json")


def probe(climb, dive, gate, floor=121.92, dt=0.02):
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    prof = ClimbDiveProfile(initial_climb_angle_deg=climb,
                            dive_angle_deg=dive,
                            floor_altitude_m=max(floor, V3_FLOOR_ALTITUDE_M))
    r = fly(d, drag_model="buildup", climb_dive=prof, dt_s=dt)
    fuel = max(s.fuel_burned_kg for s in r.states)
    peak = max(s.mach for s in r.states)
    hmax = 0.0
    for s in r.states:
        if s.mode not in POWERED:
            break
        hmax = max(hmax, s.altitude_m)
    trav = r.min_traverse_accel_g
    return dict(climb=climb, dive=dive, gate=round(gate, 3), floor=floor,
                dt=dt, traverse=trav, fp_traverse=trav * TRAV_HAIRCUT,
                fuel=fuel, fp_fuel=fuel * FUEL_PENALTY, peak_mach=peak,
                cutoff=bool(r.motor_cutoff_reached), max_powered_alt_m=hmax,
                top_m=r.climb_dive_top_altitude_m,
                margin=r.min_powered_thrust_margin,
                trav_mach=r.min_traverse_accel_mach,
                rule_violated=bool(r.rule_violated),
                alt_ok=bool(hmax <= ALT_CAP),
                fp_pass=bool(trav * TRAV_HAIRCUT >= 0.26 and peak >= 1.0
                             and r.motor_cutoff_reached
                             and fuel * FUEL_PENALTY < CAP
                             and hmax <= ALT_CAP))


t0 = time.time()
rows = []
for dive in (14.0, 16.0, 18.0, 20.0, 22.0, 25.0, 28.0, 30.0, 33.0):
    for climb in (8.0, 10.0, 12.0, 14.0):
        for g in [round(0.40 + 0.01 * i, 2) for i in range(13)]:
            rows.append(probe(climb, dive, g))
    ok = [r for r in rows if r["dive"] == dive and r["fp_pass"]]
    b = max(ok, key=lambda r: (r["gate"], r["fp_traverse"])) if ok else None
    print(f"dive {dive:4.1f}  best FP-defensible gate "
          f"{b['gate'] if b else None}"
          + (f" (climb {b['climb']:.0f}, cf_trav {b['traverse']:.3f} -> fp "
             f"{b['fp_traverse']:.3f}, hmax {b['max_powered_alt_m']:.0f} m, "
             f"fuel {b['fuel']:.3f})" if b else "")
          + f"   [{time.time()-t0:.0f} s]", flush=True)

json.dump(dict(alt_cap_m=ALT_CAP, rows=rows),
          open(OUT / "v3f_ceiling_constrained.json", "w"), indent=1)

ok = [r for r in rows if r["fp_pass"]]
best_gate = max((r["gate"] for r in ok), default=None)
print(f"\nLATEST FP-DEFENSIBLE GATE UNDER THE {ALT_CAP:.0f} m CEILING: "
      f"{best_gate}")
print(f"{'climb':>6}{'dive':>5}{'gate':>6}{'cfTrv':>7}{'fpTrv':>7}{'cfFuel':>7}"
      f"{'hmax':>7}{'marg':>7}{'trvM':>6}")
for r in sorted([x for x in ok if x["gate"] >= (best_gate or 0) - 0.01],
                key=lambda r: (-r["gate"], -r["fp_traverse"])):
    print(f"{r['climb']:6.0f}{r['dive']:5.0f}{r['gate']:6.2f}"
          f"{r['traverse']:7.3f}{r['fp_traverse']:7.3f}{r['fuel']:7.3f}"
          f"{r['max_powered_alt_m']:7.0f}{r['margin']:7.3f}"
          f"{r['trav_mach']:6.3f}")
print("wrote", OUT / "v3f_ceiling_constrained.json", f"{time.time()-t0:.0f} s")
