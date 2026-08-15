"""De-risk the 235 mm pick: is its +20.6% pulsejet advantage REAL on FP,
across Mach, or only in the closed form?

The 235 mm duct has exactly ONE first-principles measurement in the repo
(the M 0.15 sustain-cliff point, 149.19 N).  The entire closed-form case
for it rests on a thrust ratio of 1.2064 that is FLAT in Mach.  If the
bigger duct lapses faster with Mach on FP, the advantage evaporates
exactly where it is spent -- in the late dive at M 0.45-0.60.

Engine-level FP queries only (no flights).  V3a's column already exists
in out_medium_model/v3c_wall_fp.json; this measures the 235 mm on the
same points and prints the FP ratio against the closed-form ratio.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3f_fp_ratio.py
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

OUT = Path("out_medium_model")
DEST = OUT / "v3f_fp_ratio.json"

POINTS = [(0.35, 300.0), (0.45, 300.0), (0.54, 300.0), (0.60, 300.0),
          (0.40, 900.0), (0.54, 900.0)]


def main():
    from pulsejet_fp import (Numerics, pulsejet_thrust, reference_gas,
                             reference_valve)
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry
    from medium_model.pulsejet_simple import pulsejet_thrust as cf_thrust
    from douglas_dart.atmosphere import standard_atmosphere

    designs = {"V3a": "docs/v3a_medium_model/design.json",
               "235mm": "docs/v3c_235mm/design.json"}
    rows = []
    for name, path in designs.items():
        d = load_frozen_design(path)
        spec = spec_from_geometry(d.geometry)
        geom = spec.pulsejet_geometry()
        valve = spec.pulsejet_valve(reference_valve())
        intake = spec.pulsejet_intake()
        print(f"\n{name}: chamber {geom.chamber_diameter*1e3:.1f} x "
              f"{geom.chamber_length*1e3:.0f} mm, tail "
              f"{geom.tailpipe_diameter*1e3:.1f} x "
              f"{geom.tailpipe_length*1e3:.0f} mm "
              f"(t/D {geom.tailpipe_length/geom.chamber_diameter:.2f})")
        for mach, alt in POINTS:
            t0 = time.perf_counter()
            try:
                r = pulsejet_thrust(mach=mach, altitude_m=alt,
                                    gas=reference_gas(phi=1.0), geom=geom,
                                    valve=valve, intake=intake,
                                    numerics=Numerics(n_cells=162),
                                    t_end=1.2, stop_when_converged=True)
                rec = dict(design=name, mach=mach, alt_m=alt,
                           fp_thrust_n=r.thrust_n, hz=r.frequency_hz,
                           mdot_fuel=r.mdot_fuel_kg_s, status=str(r.status))
            except Exception as exc:
                rec = dict(design=name, mach=mach, alt_m=alt,
                           error=repr(exc))
            atm = standard_atmosphere(alt)
            g = d.geometry
            try:
                cf = cf_thrust(g.diameter_m, g.chamber_length_m,
                               g.throat_diameter_m, g.throat_length_m,
                               mach, alt, g.fuel, atmosphere=atm)
                rec["cf_thrust_n"] = cf.thrust_n
            except Exception as exc:
                rec["cf_error"] = repr(exc)
            rows.append(rec)
            print(f"  M {mach:.2f} @ {alt:4.0f} m  FP "
                  f"{rec.get('fp_thrust_n', float('nan')):7.2f} N  CF "
                  f"{rec.get('cf_thrust_n', float('nan')):7.2f} N  "
                  f"({time.perf_counter()-t0:.0f} s)", flush=True)
            json.dump(rows, open(DEST, "w"), indent=1)

    print(f"\n{'M':>5}{'alt':>6}{'FP V3a':>9}{'FP 235':>9}{'FPratio':>9}"
          f"{'CF V3a':>9}{'CF 235':>9}{'CFratio':>9}")
    for mach, alt in POINTS:
        a = next((r for r in rows if r["design"] == "V3a"
                  and r["mach"] == mach and r["alt_m"] == alt), None)
        b = next((r for r in rows if r["design"] == "235mm"
                  and r["mach"] == mach and r["alt_m"] == alt), None)
        if not (a and b and "fp_thrust_n" in a and "fp_thrust_n" in b):
            continue
        print(f"{mach:5.2f}{alt:6.0f}{a['fp_thrust_n']:9.2f}"
              f"{b['fp_thrust_n']:9.2f}"
              f"{b['fp_thrust_n']/a['fp_thrust_n']:9.4f}"
              f"{a.get('cf_thrust_n', float('nan')):9.2f}"
              f"{b.get('cf_thrust_n', float('nan')):9.2f}"
              f"{b.get('cf_thrust_n', 1)/a.get('cf_thrust_n', 1):9.4f}")
    print("wrote", DEST)


if __name__ == "__main__":
    main()
