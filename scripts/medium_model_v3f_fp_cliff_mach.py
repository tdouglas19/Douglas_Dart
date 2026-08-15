"""The 235 mm duct is DEAD at flight Mach -- localise why.

medium_model_v3f_fp_ratio.py measured the docs/v3c_235mm duct (t/D 4.00)
at ~1 N over M 0.35-0.60 while V3a (t/D 4.80) makes 106-115 N there.  The
only FP evidence behind the 235 mm design was ONE point, M 0.15 at 60 m
(scripts/medium_model_v3b_cliff.py line 81), where it makes 149.2 N.

Three questions:
  A. does the M 0.15 / 60 m point reproduce?  (is the cliff data right)
  B. at what Mach does t/D 4.00 stop sustaining?
  C. is it the RATIO or the DIAMETER?  -- same 235 mm chamber, tail
     stretched to V3a's t/D 4.80, at flight Mach.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3f_fp_cliff_mach.py
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

OUT = Path("out_medium_model")
DEST = OUT / "v3f_fp_cliff_mach.json"


def main():
    from pulsejet_fp import (Numerics, pulsejet_thrust, reference_gas,
                             reference_valve)
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    d235 = load_frozen_design("docs/v3c_235mm/design.json")
    dV3a = load_frozen_design("docs/v3a_medium_model/design.json")
    s235, sV3a = spec_from_geometry(d235.geometry), spec_from_geometry(dV3a.geometry)
    g235, gV3a = s235.pulsejet_geometry(), sV3a.pulsejet_geometry()
    # C: same 235 mm chamber + throat, tail stretched from t/D 4.00 to 4.80
    g235_tD48 = replace(g235, tailpipe_length=4.80 * g235.chamber_diameter)

    CASES = [
        ("235mm t/D4.00", g235, s235, 0.15, 60.0),
        ("V3a   t/D4.80", gV3a, sV3a, 0.15, 60.0),
        ("235mm t/D4.00", g235, s235, 0.25, 300.0),
        ("235mm t/D4.00", g235, s235, 0.30, 300.0),
        ("235mm t/D4.80", g235_tD48, s235, 0.15, 60.0),
        ("235mm t/D4.80", g235_tD48, s235, 0.35, 300.0),
        ("235mm t/D4.80", g235_tD48, s235, 0.45, 300.0),
        ("235mm t/D4.80", g235_tD48, s235, 0.54, 300.0),
        ("235mm t/D4.80", g235_tD48, s235, 0.54, 900.0),
    ]
    rows = []
    print(f"{'case':16s}{'M':>6}{'alt':>7}{'thrust':>9}{'Hz':>7}"
          f"{'p_min':>7}{'p_max':>7}{'sust':>6}{'s':>5}")
    for label, geom, spec, mach, alt in CASES:
        t0 = time.perf_counter()
        r = pulsejet_thrust(mach=mach, altitude_m=alt,
                            gas=reference_gas(phi=1.0), geom=geom,
                            valve=spec.pulsejet_valve(reference_valve()),
                            intake=spec.pulsejet_intake(),
                            numerics=Numerics(n_cells=162), t_end=1.2,
                            stop_when_converged=True)
        sust = r.thrust_n > 10.0 and (r.p_max_ratio - r.p_min_ratio) > 0.2
        rows.append(dict(case=label, mach=mach, alt_m=alt,
                         tail_mm=geom.tailpipe_length * 1e3,
                         t_over_d=geom.tailpipe_length / geom.chamber_diameter,
                         thrust_n=r.thrust_n, hz=r.frequency_hz,
                         p_min=r.p_min_ratio, p_max=r.p_max_ratio,
                         sustains=bool(sust), status=str(r.status)))
        print(f"{label:16s}{mach:6.2f}{alt:7.0f}{r.thrust_n:9.2f}"
              f"{r.frequency_hz:7.1f}{r.p_min_ratio:7.3f}"
              f"{r.p_max_ratio:7.3f}{'YES' if sust else 'DEAD':>6}"
              f"{time.perf_counter()-t0:5.0f}", flush=True)
        json.dump(rows, open(DEST, "w"), indent=1)
    print("wrote", DEST)


if __name__ == "__main__":
    main()
