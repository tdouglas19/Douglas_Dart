"""Lightoff Mach and climb angle INTERACT -- map the pair, not each alone.

The 1-D sweep at climb 8 deg said lightoff M 0.35 was a sharp optimum and
that lighting earlier was a trap (0.30 -> 0.052 g, 0.25 -> 0.073 g).  Then
the same lightoff 0.30 at climb 12 deg returned 0.363 g -- better than the
supposed optimum.

The mechanism is that the gate is a Mach threshold but its consequence is
positional: it decides WHERE in the trajectory the ramjet lights.  A
shallower climb reaches any given Mach earlier and lower, so at climb 8
deg the M 0.30 gate fires at 34 s / 343 m, deep in the climb where the
ramjet makes 95 N for 0.027 kg/s.  A steeper climb reaches the same Mach
later and higher, closer to the dive, where the same gate is worth having.

So neither variable has an optimum on its own.  This maps the pair.
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
TARGET_G = 0.26
# Fill the corners the 1-D sweeps left open, cheapest-informative first.
COMBOS = [(12.0, 0.35), (10.0, 0.35), (12.0, 0.40), (10.0, 0.30),
          (14.0, 0.30), (12.0, 0.45)]


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    print("V3a body + frozen wing/dive/floor, FP propulsion, drag build-up")
    print("known already: (8, 0.45) 0.183 | (8, 0.35) 0.331 | "
          "(8, 0.30) 0.052 | (8, 0.25) 0.073 | (12, 0.30) 0.363\n")
    print(f"{'climb':>6} {'lit':>5} {'trav_g':>7} {'pow_g':>7} "
          f"{'margin':>7} {'fuel':>6} {'peakM':>6} {'cut':>5} "
          f"{'runs':>5}  gate  event")
    rows = []
    for climb, lit in COMBOS:
        cd = ClimbDiveProfile(
            initial_climb_angle_deg=climb,
            dive_angle_deg=d.climb_dive.dive_angle_deg,
            floor_altitude_m=d.climb_dive.floor_altitude_m)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=lit,
                          n_cells=162)
        r = fly(d, drag_model="buildup", climb_dive=cd, propulsion=fp)
        peak = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        ev = [f"{t:.1f}s {w}" for t, w in fp.trace.events]
        lit_ev = next((e for e in ev if "ramjet_lit" in e), "-")
        ok = (r.motor_cutoff_reached and peak >= 1.0
              and r.min_traverse_accel_g >= TARGET_G
              and fuel < d.burn_limit_kg - 1e-6)
        rows.append(dict(climb_deg=climb, lightoff_mach=lit,
                         min_traverse_g=r.min_traverse_accel_g,
                         min_powered_g=r.min_powered_accel_g,
                         margin=r.min_powered_thrust_margin,
                         fuel_kg=fuel, peak_mach=peak,
                         cutoff=bool(r.motor_cutoff_reached),
                         tank_dry=fuel >= d.burn_limit_kg - 1e-6,
                         fp_runs=fp.n_transients, passes=bool(ok),
                         events=ev))
        print(f"{climb:6.1f} {lit:5.2f} {r.min_traverse_accel_g:7.3f} "
              f"{r.min_powered_accel_g:7.3f} "
              f"{r.min_powered_thrust_margin:7.3f} {fuel:6.3f} "
              f"{peak:6.3f} {str(r.motor_cutoff_reached):>5} "
              f"{fp.n_transients:5d}  {'PASS' if ok else 'fail'}  {lit_ev}",
              flush=True)
        (OUT / "v3b_joint.json").write_text(json.dumps(rows, indent=2))

    ok = [q for q in rows if q["passes"]]
    print(f"\n{len(ok)}/{len(rows)} pass. Best by traverse:")
    for q in sorted(ok, key=lambda z: -z["min_traverse_g"])[:5]:
        print(f"  climb {q['climb_deg']:.0f} / lightoff {q['lightoff_mach']:.2f}"
              f": {q['min_traverse_g']:.3f} g, {q['fuel_kg']:.3f} kg")
    print(f"\nwrote {OUT / 'v3b_joint.json'}")


if __name__ == "__main__":
    main()
