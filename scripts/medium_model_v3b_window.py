"""Find the climb/dive box that survives FIRST-PRINCIPLES propulsion.

The composition screen says the trajectory lever subsumes the wing lever
(traverse 0.385 alone, 0.394 with the wing), so traverse is not the
binding worry any more.  The worry is `min_powered_accel_g`, which lives
in `v3_climb` in every flight and which the trajectory lever *spends*:
climb 15.5 deg / dive 20 deg buys traverse 0.385 g at the cost of a
0.030 g climb margin.

FP propulsion runs 6-16% below closed-form.  At climb 15 deg the climb
exits at T 112 N against D 49 N + W*sin 56 N -- a 7 N surplus that a 16%
thrust cut erases outright.  Prior FP flights confirm it: climb 16.657
deg stalls at M 0.118, climb 12 deg holds +0.011 g, climb 8 deg +0.040 g.

So the FP-legal window is climb ~8-12 deg, and the question is how much
dive angle is needed there to still clear 0.26 g traverse.  Screen it
closed-form (seconds), then hand the survivors to FP.

Wing held at the knee the wing agent recommended (0.78 m / AR 4.5): it
costs nothing in traverse, saves 0.11 kg of fuel, raises thrust margin,
and unlike the frozen 0.5325 m wing it does not stall.
"""
from __future__ import annotations
import json, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.drag import WingConcept                    # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402

TARGET_TRAVERSE_G = 0.26
FLOOR_M = 122.0
CLIMBS = (8.0, 10.0, 12.0, 14.0)
DIVES = (10.0, 12.0, 14.0, 16.0, 18.0, 20.0)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    w = d.wing
    wing = WingConcept(span_m=0.78, aspect_ratio=4.5,
                       taper_ratio=w.taper_ratio, sweep_deg=w.sweep_deg,
                       airfoil=w.airfoil)

    print("V3a body, wing 0.780 m / AR 4.5, floor 122 m, "
          "closed-form propulsion, drag build-up")
    print(f"target traverse {TARGET_TRAVERSE_G} g; "
          "powered-g is the FP-survivability proxy\n")

    rows = []
    for climb in CLIMBS:
        for dive in DIVES:
            cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                  dive_angle_deg=dive,
                                  floor_altitude_m=FLOOR_M)
            r = fly(d, drag_model="buildup", climb_dive=cd,
                    wing_concept=wing)
            peak = max(s.mach for s in r.states)
            rows.append(dict(
                climb=climb, dive=dive,
                trav=r.min_traverse_accel_g, pow=r.min_powered_accel_g,
                peak=peak, cutoff=r.motor_cutoff_reached,
                margin=r.min_powered_thrust_margin,
                fuel=max(s.fuel_burned_kg for s in r.states)))

    print(f"{'':>6}", end="")
    for dive in DIVES:
        print(f"{'dive'+str(int(dive)):>9}", end="")
    print("      <- min_traverse_accel_g")
    for climb in CLIMBS:
        print(f"{climb:5.0f} ", end="")
        for dive in DIVES:
            q = next(x for x in rows if x["climb"] == climb
                     and x["dive"] == dive)
            mark = "*" if q["trav"] >= TARGET_TRAVERSE_G else " "
            print(f"{q['trav']:8.3f}{mark}", end="")
        print()

    print(f"\n{'':>6}", end="")
    for dive in DIVES:
        print(f"{'dive'+str(int(dive)):>9}", end="")
    print("      <- min_powered_accel_g (climb phase)")
    for climb in CLIMBS:
        print(f"{climb:5.0f} ", end="")
        for dive in DIVES:
            q = next(x for x in rows if x["climb"] == climb
                     and x["dive"] == dive)
            print(f"{q['pow']:9.3f}", end="")
        print()

    ok = [q for q in rows if q["trav"] >= TARGET_TRAVERSE_G
          and q["cutoff"] and q["peak"] >= 1.0]
    ok.sort(key=lambda q: -q["pow"])
    print(f"\n{len(ok)} configs clear traverse {TARGET_TRAVERSE_G} g "
          f"AND reach cutoff, ranked by climb margin (FP headroom):")
    print(f"{'climb':>6} {'dive':>6} {'trav_g':>7} {'pow_g':>7} "
          f"{'margin':>7} {'fuel':>6} {'peakM':>6}")
    for q in ok[:10]:
        print(f"{q['climb']:6.0f} {q['dive']:6.0f} {q['trav']:7.3f} "
              f"{q['pow']:7.3f} {q['margin']:7.3f} {q['fuel']:6.3f} "
              f"{q['peak']:6.3f}")

    out = Path("out_medium_model"); out.mkdir(exist_ok=True)
    (out / "v3b_window.json").write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {out / 'v3b_window.json'}")


if __name__ == "__main__":
    main()
