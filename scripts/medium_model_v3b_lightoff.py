"""The binding constraint was an ASSUMPTION, not physics.

V3's RAMJET_MIN_LIGHTOFF_MACH = 0.45 is inherited from the closed-form
model. ramjet-fp says V3a's duct lights on propane all the way down to
M 0.25 (66.8 N) -- so the vehicle was coasting on the pulsejet alone
through the one band where it had no margin (M 0.45 dive pinch: thrust
115.8 N vs drag 111.1 N).

Lighting earlier costs nothing in length or mass, but it DOES cost fuel
(0.022-0.041 kg/s from a 2.424 kg budget), so there is an optimum.
Sweep it with full FP flights.
"""
from __future__ import annotations
import json, os
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
    # (lightoff, climb_deg, span, AR) -- start from the UNCHANGED V3a wing
    combos = [(0.45, 8.0, 0.5325, 1.86),
              (0.35, 8.0, 0.5325, 1.86),
              (0.30, 8.0, 0.5325, 1.86),
              (0.25, 8.0, 0.5325, 1.86),
              (0.30, 12.0, 0.5325, 1.86),
              (0.30, 16.66, 0.5325, 1.86),
              (0.30, 8.0, 0.65, 2.50)]
    rows = []
    print(f"{'light':>6} {'climb':>6} {'span':>6} {'peakM':>6} {'cut':>5} "
          f"{'trav_g':>7} {'pow_g':>7} {'margin':>7} {'fuel':>6} {'dry?':>5} "
          f"{'land_m':>7} {'runs':>5}")
    for lo, climb, span, ar in combos:
        wc = WingConcept(span, ar, 0.512, 13.4, AIRFOILS["thin_cambered"])
        cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                              dive_angle_deg=d.climb_dive.dive_angle_deg,
                              floor_altitude_m=d.climb_dive.floor_altitude_m)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=lo, n_cells=162)
        r = fly(replace(d, wing=wc), drag_model="buildup", climb_dive=cd,
                propulsion=fp)
        pm = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        row = dict(lightoff_mach=lo, climb_deg=climb, span_m=span,
                   aspect_ratio=ar, peak_mach=pm, above_m1=pm >= 1.0,
                   cutoff=r.motor_cutoff_reached,
                   min_traverse_g=r.min_traverse_accel_g,
                   min_powered_g=r.min_powered_accel_g,
                   margin=r.min_powered_thrust_margin, fuel_kg=fuel,
                   tank_dry=fuel >= d.burn_limit_kg - 1e-3,
                   lands_from_launch_m=abs(r.states[-1].distance_m),
                   safe_landing=r.safe_landing,
                   flight_s=r.states[-1].time_s, fp_runs=fp.n_transients,
                   events=[f"{t:.1f}s {w}" for t, w in fp.trace.events])
        rows.append(row)
        print(f"{lo:6.2f} {climb:6.1f} {span:6.3f} {pm:6.3f} "
              f"{str(r.motor_cutoff_reached):>5} {r.min_traverse_accel_g:7.3f} "
              f"{r.min_powered_accel_g:7.3f} {r.min_powered_thrust_margin:7.2f} "
              f"{fuel:6.3f} {str(row['tank_dry']):>5} "
              f"{row['lands_from_launch_m']:7.0f} {fp.n_transients:5d}",
              flush=True)
        for e in row["events"][:3]:
            print(f"        {e}")
        (OUT / "v3b_lightoff.json").write_text(
            json.dumps(rows, indent=2, default=str))
    ok = [r for r in rows if r["above_m1"] and r["min_traverse_g"] >= 0.26]
    print(f"\n{len(ok)} of {len(rows)} meet Mach>=1.0 AND traverse>=0.26 g")
    for r in ok:
        print(f"   PASS: lightoff {r['lightoff_mach']}, climb "
              f"{r['climb_deg']}, span {r['span_m']} -> traverse "
              f"{r['min_traverse_g']:.3f} g, fuel {r['fuel_kg']:.3f} kg")


if __name__ == "__main__":
    main()
