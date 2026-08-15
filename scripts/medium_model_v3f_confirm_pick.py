"""Pick the ONE configuration that gets the expensive FP CONFIRM run.

Two jobs:

  1. CALIBRATE the closed-form -> FP traverse haircut on paired flights.
     out_medium_model/v3b_joint.json holds SIX real FP flights of the V3a
     airframe (frozen wing 0.532526 m / AR 1.85958, dive 9.89 deg, floor
     133.14 m) at known climb x lightoff-gate.  Re-fly the identical six
     closed-form and take the ratio.  Everything downstream is scored
     against the WORST observed ratio, not the mean.

  2. SWEEP the steep-dive corner both prior studies stopped short of, on
     both candidate airframes, at 0.01 gate resolution, and report the
     latest gate whose CLOSED-FORM traverse still clears 0.26 g after the
     measured haircut.

Closed-form propulsion only -- this ranks, it does not certify.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3f_confirm_pick.py
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
OUT.mkdir(exist_ok=True)

DESIGNS = {
    "V3a": "docs/v3a_medium_model/design.json",
    "235mm": "docs/v3c_235mm/design.json",
}

TARGET_G = 0.26


def run(d, climb, dive, floor, gate):
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    prof = ClimbDiveProfile(initial_climb_angle_deg=climb,
                            dive_angle_deg=dive,
                            floor_altitude_m=max(floor, V3_FLOOR_ALTITUDE_M))
    r = fly(d, drag_model="buildup", climb_dive=prof)
    trav = r.min_traverse_accel_g
    fuel = max(s.fuel_burned_kg for s in r.states)
    peak = max(s.mach for s in r.states)
    ok = (trav >= TARGET_G and peak >= 1.0
          and r.motor_cutoff_reached and fuel < d.burn_limit_kg - 1e-6)
    return dict(climb=climb, dive=dive, floor=floor, gate=round(gate, 4),
                traverse=trav, peak_mach=peak,
                cutoff=bool(r.motor_cutoff_reached), fuel=fuel,
                margin=r.min_powered_thrust_margin,
                powered_g=r.min_powered_accel_g,
                top_m=r.climb_dive_top_altitude_m,
                rule_violated=bool(getattr(r, "rule_violated", False)),
                passes=bool(ok))


# --------------------------------------------------------------------------
# 1. FP haircut calibration -- six paired points
# --------------------------------------------------------------------------
FP_PAIRS = [           # climb, gate, FP min_traverse_g (v3b_joint.json)
    (12.0, 0.35, 0.32720598571050824),
    (10.0, 0.35, 0.31980120840058124),
    (12.0, 0.40, 0.251827467283456),
    (10.0, 0.30, 0.3758474235556132),
    (14.0, 0.30, 0.39188479567686973),
    (12.0, 0.45, 0.010715165075998831),
]
V3B_DIVE = 9.890058542648028
V3B_FLOOR = 133.1432273621529

t0 = time.time()
v3a = load_frozen_design(DESIGNS["V3a"])
calib = []
for climb, gate, fp_trav in FP_PAIRS:
    cf = run(v3a, climb, V3B_DIVE, V3B_FLOOR, gate)
    calib.append(dict(climb=climb, gate=gate, cf=cf["traverse"], fp=fp_trav,
                      ratio=fp_trav / cf["traverse"] if cf["traverse"] else None,
                      cf_fuel=cf["fuel"], cf_margin=cf["margin"]))
    print(f"calib climb {climb:4.1f} gate {gate:.2f}  cf {cf['traverse']:.4f}"
          f"  fp {fp_trav:.4f}  ratio {calib[-1]['ratio']:.4f}")

# --------------------------------------------------------------------------
# 2. steep-dive fine sweep, both airframes
# --------------------------------------------------------------------------
CLIMBS = (8.0, 10.0, 12.0)
DIVES = (25.0, 28.0, 30.0, 35.0)
GATES = [round(0.44 + 0.01 * i, 2) for i in range(13)]     # 0.44 .. 0.56
FLOOR = 121.92

sweep = []
for name, path in DESIGNS.items():
    d = load_frozen_design(path)
    for dive in DIVES:
        for climb in CLIMBS:
            for gate in GATES:
                row = run(d, climb, dive, FLOOR, gate)
                row["design"] = name
                sweep.append(row)
            best = [r for r in sweep
                    if r["design"] == name and r["dive"] == dive
                    and r["climb"] == climb and r["passes"]]
            top = max((r["gate"] for r in best), default=None)
            print(f"{name:6s} dive {dive:4.1f} climb {climb:4.1f}  "
                  f"ceiling {top}  ({time.time()-t0:.0f} s)")

# --------------------------------------------------------------------------
# 3. floor sensitivity on the leading cell (does a higher floor help?)
# --------------------------------------------------------------------------
floor_rows = []
for name, path in DESIGNS.items():
    d = load_frozen_design(path)
    for floor in (121.92, 150.0, 200.0, 250.0):
        for gate in (0.50, 0.51, 0.52, 0.53, 0.54):
            row = run(d, 10.0, 30.0, floor, gate)
            row["design"] = name
            floor_rows.append(row)
    print(f"floor scan {name} done ({time.time()-t0:.0f} s)")

json.dump(dict(calibration=calib, sweep=sweep, floor_scan=floor_rows),
          open(OUT / "v3f_confirm_pick.json", "w"), indent=1)
print("wrote", OUT / "v3f_confirm_pick.json", f"{time.time()-t0:.0f} s")
