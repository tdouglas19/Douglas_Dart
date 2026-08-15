"""Is the RAMJET the thing that caps the gate? FP cold-light check.

Two engine-level ramjet_operating_point queries (cold light, no seed) at
the low end of the gate ladder. If the ramjet cold-lights at M 0.25-0.30
then it is emphatically not the constraint and the wall lives entirely in
the pulsejet / trajectory side.
"""
from __future__ import annotations
import json, os, time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")
OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)


def main():
    from ramjet_fp import Numerics, ramjet_operating_point
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    rows = []
    print(f"{'M':>5} {'alt':>6} {'viable':>7} {'net_N':>8} {'phi':>6} "
          f"{'mdot_f':>8} {'mdot_air':>9} {'s':>5}")
    for mach, alt in ((0.25, 300.0), (0.30, 300.0), (0.45, 300.0),
                      (0.50, 900.0)):
        t0 = time.perf_counter()
        op = ramjet_operating_point(
            mach, alt, geom=spec.ramjet_geometry(),
            fh=spec.ramjet_flameholder(), numerics=Numerics(n_cells=162),
            fuel="propane", phi_seed=None, t_end=0.35, seed_state=None,
            ramp_from=None)
        rows.append(dict(mach=mach, alt_m=alt, viable=bool(op.viable),
                         net_thrust_n=op.net_thrust_n,
                         phi=op.required_phi,
                         mdot_fuel=op.mdot_fuel_kg_s,
                         mdot_air=op.mdot_air_kg_s))
        print(f"{mach:5.2f} {alt:6.0f} {str(op.viable):>7} "
              f"{op.net_thrust_n:8.1f} "
              f"{(op.required_phi if op.required_phi else float('nan')):6.3f} "
              f"{op.mdot_fuel_kg_s:8.5f} {op.mdot_air_kg_s:9.4f} "
              f"{time.perf_counter()-t0:5.0f}", flush=True)
        (OUT / "v3c_wall_rjcheck.json").write_text(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
