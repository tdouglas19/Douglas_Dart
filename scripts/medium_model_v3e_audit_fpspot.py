"""ADVERSARIAL AUDIT part 6 -- two FP pulsejet spot-checks.

The mechanism report's whole "FP may place the wall at 0.50" argument
rests on a claimed FP/closed-form pulsejet thrust ratio of 0.91 at M 0.30
rising to 0.95 at M 0.50, at 300 m.  Two engine-level FP queries (no
flights -- the CPU is busy) test that.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_fpspot.py
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import load_frozen_design               # noqa: E402
from medium_model.fp_propulsion import FpPropulsion              # noqa: E402
from medium_model.fp_spec import spec_from_geometry              # noqa: E402
from medium_model.pulsejet_simple import pulsejet_thrust         # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
ALT = 300.0
POINTS = (0.30, 0.50)
CLAIMED = {0.30: (127.9, 116.5, 0.911), 0.50: (115.3, 109.8, 0.952)}


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    g = d.geometry
    prop = FpPropulsion(spec_from_geometry(g), fuel="propane",
                        lightoff_mach=0.95, n_cells=162)
    rows = []
    print(f"{'M':>5} {'alt':>5} {'T_cf':>8} {'T_fp':>8} {'FP/CF':>7} "
          f"{'claim_cf':>9} {'claim_fp':>9} {'claim_r':>8} {'secs':>6}")
    for m in POINTS:
        cf = pulsejet_thrust(g.diameter_m, g.chamber_length_m,
                             g.throat_diameter_m, g.throat_length_m,
                             m, ALT, g.fuel).average_thrust_n
        t0 = time.time()
        prop.thrust_and_fuel(t0 * 0.0 + len(rows) * 100.0, m, ALT)
        fp = prop.pulsejet_thrust_n
        dt = time.time() - t0
        c = CLAIMED[m]
        rows.append(dict(mach=m, alt=ALT, t_cf=cf, t_fp=fp,
                         ratio=fp / cf if cf else None,
                         claimed_cf=c[0], claimed_fp=c[1], claimed_ratio=c[2],
                         seconds=dt))
        print(f"{m:5.2f} {ALT:5.0f} {cf:8.2f} {fp:8.2f} {fp/cf:7.4f} "
              f"{c[0]:9.1f} {c[1]:9.1f} {c[2]:8.3f} {dt:6.1f}", flush=True)
    (OUT / "v3e_audit_fpspot.json").write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_fpspot.json'}")


if __name__ == "__main__":
    main()
