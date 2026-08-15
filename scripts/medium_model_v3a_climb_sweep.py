"""V3's initial climb angle was chosen against a closed-form engine.
With the first-principles engine (only 6-16% weaker) the 16.7 deg climb
decelerates into a stall in 16 s -- because the climb's own margin is
0.072 g, i.e. ~16 N of net force, so a 23 N thrust change flips its sign.

Sweep the initial climb angle with FP propulsion to find what the real
engine can actually hold.
"""
from __future__ import annotations
import json, os, sys
from dataclasses import replace
from pathlib import Path
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    use_fp = "--fp" in sys.argv
    print(f"V3a, {'FP' if use_fp else 'closed-form'} propulsion, "
          f"drag build-up\n")
    print(f"{'climb':>6} {'peakM':>6} {'>M1':>5} {'cutoff':>7} "
          f"{'min_pow_g':>10} {'min_trav_g':>11} {'margin':>7} "
          f"{'fuel':>6} {'time':>6} {'runs':>5}")
    for climb_deg in (16.66, 12.0, 8.0, 5.0, 3.0):
        cd = ClimbDiveProfile(
            initial_climb_angle_deg=climb_deg,
            dive_angle_deg=d.climb_dive.dive_angle_deg,
            floor_altitude_m=d.climb_dive.floor_altitude_m)
        fp = (FpPropulsion(spec, fuel="propane", lightoff_mach=0.45,
                           n_cells=162) if use_fp else None)
        r = fly(d, drag_model="buildup", climb_dive=cd, propulsion=fp)
        pm = max(s.mach for s in r.states)
        print(f"{climb_deg:6.2f} {pm:6.3f} {str(pm>=1.0):>5} "
              f"{str(r.motor_cutoff_reached):>7} "
              f"{r.min_powered_accel_g:10.3f} {r.min_traverse_accel_g:11.3f} "
              f"{r.min_powered_thrust_margin:7.2f} "
              f"{max(s.fuel_burned_kg for s in r.states):6.3f} "
              f"{r.states[-1].time_s:6.0f} "
              f"{fp.n_transients if fp else 0:5d}", flush=True)


if __name__ == "__main__":
    main()
