"""V3b candidate verification with FIRST-PRINCIPLES propulsion.

Closed-form screening (medium_model_v3b_window.py) leaves a box of 17
configs that clear the 0.26 g traverse gate.  Closed-form is not the
authority here -- it is 6-16% optimistic on thrust and, more importantly,
it hard-codes `RAMJET_MIN_LIGHTOFF_MACH = 0.45` when the FP engine
actually lights at M 0.25.  Both errors land on the same place: the
`v3_dive` trough at M 0.450, the last step before lightoff, which is
where the traverse minimum sits in every one of those flights.

So this re-flies the survivors against the FP engine, and sweeps the
lightoff Mach as a real design variable rather than a constant.  Lighting
earlier costs no length and no mass, but it does cost fuel (0.022 kg/s at
M 0.25 rising to 0.041 kg/s at M 0.45, against a 2.424 kg burn cap), so
there is an optimum rather than a monotone win.

Body geometry is V3a, untouched: this asks whether the vehicle closes
with NO length change at all before spending any length on it.
"""
from __future__ import annotations
import json, os, sys, traceback
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.drag import WingConcept                    # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402

TARGET_TRAVERSE_G = 0.26
FLOOR_M = 122.0

# (climb_deg, dive_deg, lightoff_mach, span_m, aspect_ratio, note)
CANDIDATES = [
    (14.0, 14.0, 0.45, 0.78, 4.5, "control: configured lightoff"),
    (14.0, 14.0, 0.30, 0.78, 4.5, "early lightoff"),
    (14.0, 16.0, 0.30, 0.78, 4.5, "more dive"),
    (14.0, 12.0, 0.30, 0.78, 4.5, "less dive, best climb margin"),
    (12.0, 16.0, 0.30, 0.78, 4.5, "shallower climb"),
    (14.0, 14.0, 0.25, 0.78, 4.5, "earliest lightoff"),
]


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    w = d.wing

    print(f"V3b verification -- V3a body ({d.geometry.diameter_m*1000:.0f} mm "
          f"dia, unchanged), FP propulsion, drag build-up")
    print(f"loaded fuel {d.loaded_fuel_kg:.3f} kg -> burn limit "
          f"{d.burn_limit_kg:.3f} kg\n")
    print(f"{'climb':>6} {'dive':>5} {'lit':>5} {'span':>5} {'AR':>4} "
          f"{'peakM':>6} {'cut':>5} {'trav_g':>7} {'pow_g':>7} "
          f"{'margin':>7} {'fuel':>6} {'dry':>5} {'runs':>5}  note")
    print("-" * 108)

    rows = []
    out = Path("out_medium_model"); out.mkdir(exist_ok=True)
    for climb, dive, lit, span, ar, note in CANDIDATES:
        wing = WingConcept(span_m=span, aspect_ratio=ar,
                           taper_ratio=w.taper_ratio,
                           sweep_deg=w.sweep_deg, airfoil=w.airfoil)
        cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                              dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=lit,
                          n_cells=162)
        try:
            r = fly(d, drag_model="buildup", climb_dive=cd,
                    wing_concept=wing, propulsion=fp)
        except Exception:
            traceback.print_exc()
            continue
        peak = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        row = dict(climb=climb, dive=dive, lightoff=lit, span=span,
                   aspect_ratio=ar, note=note, peak_mach=peak,
                   cutoff=bool(r.motor_cutoff_reached),
                   traverse_g=r.min_traverse_accel_g,
                   powered_g=r.min_powered_accel_g,
                   margin=r.min_powered_thrust_margin,
                   fuel_kg=fuel, tank_dry=fuel >= d.burn_limit_kg - 1e-6,
                   fp_runs=fp.n_transients,
                   passes=bool(r.motor_cutoff_reached and peak >= 1.0
                               and r.min_traverse_accel_g
                               >= TARGET_TRAVERSE_G))
        rows.append(row)
        print(f"{climb:6.1f} {dive:5.1f} {lit:5.2f} {span:5.2f} {ar:4.1f} "
              f"{peak:6.3f} {str(row['cutoff']):>5} "
              f"{row['traverse_g']:7.3f} {row['powered_g']:7.3f} "
              f"{row['margin']:7.3f} {fuel:6.3f} "
              f"{str(row['tank_dry']):>5} {fp.n_transients:5d}  {note}",
              flush=True)
        for ev in getattr(fp, "events", []) or []:
            print(f"        {ev}", flush=True)
        (out / "v3b_verify.json").write_text(json.dumps(rows, indent=2))

    ok = [q for q in rows if q["passes"]]
    print(f"\n{len(ok)}/{len(rows)} candidates close Gate 3 under FP "
          f"(M>=1.0, cutoff, traverse >= {TARGET_TRAVERSE_G} g)")
    for q in sorted(ok, key=lambda z: -z["traverse_g"]):
        print(f"  climb {q['climb']:.0f} dive {q['dive']:.0f} "
              f"lightoff {q['lightoff']:.2f}: traverse {q['traverse_g']:.3f} g, "
              f"powered {q['powered_g']:.3f} g, fuel {q['fuel_kg']:.3f} kg")
    print(f"\nwrote {out / 'v3b_verify.json'}")


if __name__ == "__main__":
    main()
