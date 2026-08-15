"""Bracket the lightoff optimum on its upper side.

The coarse sweep gives traverse 0.183 / 0.331 / 0.052 / 0.073 g at
lightoff M 0.45 / 0.35 / 0.30 / 0.25 -- a sharp peak at 0.35 with a
cliff immediately below it.  Two samples do not establish a peak, so
this fills M 0.38-0.42 to check that 0.35 is a genuine optimum rather
than a lucky grid point, and to find which side the cliff is on.

Matters because the recommendation is to make this constant a design
variable: if the good region is 0.34-0.36 it is a knife-edge that needs
a real throttle schedule, and if it is 0.33-0.44 it is a safe default.
"""
from __future__ import annotations
import json, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
CLIMB_DEG = 8.0
LIGHTOFFS = (0.42, 0.40, 0.38, 0.33)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    print(f"V3a body, climb {CLIMB_DEG:g} deg, frozen dive/floor, "
          f"frozen wing, FP propulsion")
    print(f"{'lit':>5} {'trav_g':>7} {'pow_g':>7} {'margin':>7} "
          f"{'fuel':>6} {'peakM':>6} {'cut':>5} {'runs':>5}  event")
    rows = []
    for lit in LIGHTOFFS:
        cd = ClimbDiveProfile(
            initial_climb_angle_deg=CLIMB_DEG,
            dive_angle_deg=d.climb_dive.dive_angle_deg,
            floor_altitude_m=d.climb_dive.floor_altitude_m)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=lit,
                          n_cells=162)
        r = fly(d, drag_model="buildup", climb_dive=cd, propulsion=fp)
        peak = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        ev = [f"{t:.1f}s {w}" for t, w in fp.trace.events]
        lit_ev = next((e for e in ev if "ramjet_lit" in e), "-")
        rows.append(dict(lightoff_mach=lit, min_traverse_g=r.min_traverse_accel_g,
                         min_powered_g=r.min_powered_accel_g,
                         margin=r.min_powered_thrust_margin, fuel_kg=fuel,
                         peak_mach=peak, cutoff=bool(r.motor_cutoff_reached),
                         fp_runs=fp.n_transients, events=ev))
        print(f"{lit:5.2f} {r.min_traverse_accel_g:7.3f} "
              f"{r.min_powered_accel_g:7.3f} "
              f"{r.min_powered_thrust_margin:7.3f} {fuel:6.3f} "
              f"{peak:6.3f} {str(r.motor_cutoff_reached):>5} "
              f"{fp.n_transients:5d}  {lit_ev}", flush=True)
        (OUT / "v3b_lightoff_fine.json").write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {OUT / 'v3b_lightoff_fine.json'}")


if __name__ == "__main__":
    main()
