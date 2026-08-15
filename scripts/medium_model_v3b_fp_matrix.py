"""V3b: FP-verified matrix over climb angle x wing, seeking traverse >= 0.26 g.

Two competing constraints, discovered by disagreement between the models:
  * launch/climb  -- FP dies here (induced drag is 67 of 71 N at 40 m/s
                     release), wants a BIGGER wing and a shallower climb
  * transonic strip -- closed-form's minimum lives here, wants a SMALLER
                     wing and more thrust
Only a first-principles flight sees both, so this matrix is FP throughout.
"""
from __future__ import annotations
import json, os, sys
from dataclasses import replace
from pathlib import Path
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")
from medium_model.constants import AIRFOILS                  # noqa: E402
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.drag import WingConcept                    # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    combos = [(8.0, 0.5325, 1.86), (8.0, 0.65, 2.5), (8.0, 0.80, 3.0),
              (12.0, 0.65, 2.5), (5.0, 0.65, 2.5), (8.0, 0.72, 2.8)]
    rows = []
    print(f"{'climb':>6} {'span':>6} {'AR':>5} {'peakM':>6} {'cut':>5} "
          f"{'trav_g':>7} {'pow_g':>7} {'margin':>7} {'fuel':>6} {'land_m':>8} "
          f"{'runs':>5}")
    for climb, span, ar in combos:
        wc = WingConcept(span, ar, 0.512, 13.4, AIRFOILS["thin_cambered"])
        cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                              dive_angle_deg=d.climb_dive.dive_angle_deg,
                              floor_altitude_m=d.climb_dive.floor_altitude_m)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=0.45,
                          n_cells=162)
        r = fly(replace(d, wing=wc), drag_model="buildup", climb_dive=cd,
                propulsion=fp)
        pm = max(s.mach for s in r.states)
        row = dict(climb_deg=climb, span_m=span, aspect_ratio=ar,
                   peak_mach=pm, cutoff=r.motor_cutoff_reached,
                   above_m1=pm >= 1.0,
                   min_traverse_g=r.min_traverse_accel_g,
                   min_powered_g=r.min_powered_accel_g,
                   margin=r.min_powered_thrust_margin,
                   fuel_kg=max(s.fuel_burned_kg for s in r.states),
                   lands_from_launch_m=abs(r.states[-1].distance_m),
                   safe_landing=r.safe_landing,
                   flight_s=r.states[-1].time_s,
                   fp_runs=fp.n_transients,
                   events=[f"{t:.1f}s {w}" for t, w in fp.trace.events])
        rows.append(row)
        print(f"{climb:6.1f} {span:6.3f} {ar:5.2f} {pm:6.3f} "
              f"{str(r.motor_cutoff_reached):>5} {r.min_traverse_accel_g:7.3f} "
              f"{r.min_powered_accel_g:7.3f} {r.min_powered_thrust_margin:7.2f} "
              f"{row['fuel_kg']:6.3f} {row['lands_from_launch_m']:8.0f} "
              f"{fp.n_transients:5d}", flush=True)
        for e in row["events"][:4]:
            print(f"        {e}")
        (OUT / "v3b_fp_matrix.json").write_text(json.dumps(rows, indent=2,
                                                           default=str))
    ok = [r for r in rows if r["above_m1"] and r["min_traverse_g"] >= 0.26]
    print(f"\n{len(ok)} of {len(rows)} meet BOTH Mach>=1.0 and traverse>=0.26 g")


if __name__ == "__main__":
    main()
