"""FP pulsejet thrust vs Mach at the altitudes the V3a dive passes through.

Engine-level queries only -- no full FP flights (another process owns the
CPU for those). Two altitude rows so the ALTITUDE lapse is separable from
the MACH lapse: that separation is the whole question, because a later
ramjet gate forces derive_top_altitude to buy more climb, which puts the
pulsejet in thinner air exactly when it is being asked to do more.

Writes incrementally so a partial run is still usable.
"""
from __future__ import annotations
import json, os, time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
DEST = OUT / "v3c_wall_fp.json"

# (mach, altitude_m) -- LOW row is the late-dive / drag-strip air the
# gate-0.45 flight actually flies; HIGH row is where gate 0.50/0.55 push it.
POINTS = [
    ("low", 0.30, 300.0), ("low", 0.35, 300.0), ("low", 0.40, 300.0),
    ("low", 0.45, 300.0), ("low", 0.50, 300.0), ("low", 0.55, 300.0),
    ("low", 0.60, 300.0),
    ("high", 0.30, 900.0), ("high", 0.40, 900.0), ("high", 0.50, 900.0),
    ("high", 0.60, 900.0),
]


def main():
    from pulsejet_fp import (Numerics, pulsejet_thrust, reference_gas,
                             reference_valve)
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    geom = spec.pulsejet_geometry()
    valve = spec.pulsejet_valve(reference_valve())
    intake = spec.pulsejet_intake()
    print(f"V3a duct: chamber {geom.chamber_diameter*1e3:.1f} x "
          f"{geom.chamber_length*1e3:.0f}, cone {geom.cone_length*1e3:.0f}, "
          f"tail {geom.tailpipe_diameter*1e3:.1f} x "
          f"{geom.tailpipe_length*1e3:.0f} mm "
          f"(t/D {geom.tailpipe_length/geom.chamber_diameter:.2f})\n")
    print(f"{'row':>5} {'M':>5} {'alt':>6} {'thrust':>8} {'Hz':>6} "
          f"{'mdot_f':>8} {'p/p0':>12} {'status':>10} {'s':>5}")

    rows = []
    for row, mach, alt in POINTS:
        t0 = time.perf_counter()
        try:
            r = pulsejet_thrust(mach=mach, altitude_m=alt,
                                gas=reference_gas(phi=1.0), geom=geom,
                                valve=valve, intake=intake,
                                numerics=Numerics(n_cells=162), t_end=1.2,
                                stop_when_converged=True)
            rec = dict(row=row, mach=mach, alt_m=alt, thrust_n=r.thrust_n,
                       hz=r.frequency_hz, mdot_fuel=r.mdot_fuel_kg_s,
                       p_min=r.p_min_ratio, p_max=r.p_max_ratio,
                       status=str(r.status))
            print(f"{row:>5} {mach:5.2f} {alt:6.0f} {r.thrust_n:8.1f} "
                  f"{r.frequency_hz:6.1f} {r.mdot_fuel_kg_s:8.5f} "
                  f"{r.p_min_ratio:5.2f}-{r.p_max_ratio:<5.2f} "
                  f"{str(r.status):>10} {time.perf_counter()-t0:5.0f}",
                  flush=True)
        except Exception as exc:
            rec = dict(row=row, mach=mach, alt_m=alt, error=repr(exc))
            print(f"{row:>5} {mach:5.2f} {alt:6.0f}   ERROR {exc!r}",
                  flush=True)
        rows.append(rec)
        DEST.write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {DEST}")


if __name__ == "__main__":
    main()
