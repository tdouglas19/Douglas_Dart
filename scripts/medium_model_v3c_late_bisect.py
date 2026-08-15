"""Find the LATEST FP-verified ramjet ignition by bisection, not by grid.

The closed-form screen said gate 0.45 was reachable and that climb 14 /
dive 20 would give traverse 0.380 g.  FP says that flight scores 0.006 g.
The screen was not merely optimistic in magnitude -- it was optimistic in
MECHANISM.  Reaching a late gate is an ACCELERATION question, and
acceleration is exactly where the closed-form and first-principles engines
differ most: the FP vehicle takes until 97 s to reach M 0.45, by which
point it is at 179 m and the 122 m floor is about to end the dive.  The
thrust arrives after the gravity assist it was supposed to pair with.

Separately, gate 0.50 is not merely bad, it is unreachable: that flight
peaked at M 0.453, never hit cutoff, and emptied the tank.  The pulsejet
cannot push this vehicle past ~M 0.45 no matter what the trajectory does.

So the answer is a bisection on the gate, not a grid: walk down from the
last known failure until a gate passes, at the dive angle that gives the
vehicle the most help.  Each step is one ~10 min FP flight, so an ordered
walk finds the boundary in far fewer flights than a product grid.

Known FP anchors (V3a body, frozen wing):
  climb 10, dive  9.9, gate 0.30 -> 0.389 g  PASS   (the V3b answer)
  climb  8, dive  9.9, gate 0.38 -> 0.277 g  PASS
  climb  8, dive  9.9, gate 0.40 -> 0.244 g  fail
  climb 14, dive 20.0, gate 0.45 -> 0.006 g  fail
  climb 14, dive 20.0, gate 0.50 -> unreachable (peak M 0.453)
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
FLOOR_M = 122.0

# Ordered latest-first. Dive 20 is kept for the early entries because a
# steeper dive is the only thing that gets the vehicle to a high Mach while
# it still has altitude to spend; once the gate drops to the range where
# lightoff lands near top of climb, the frozen 9.9 deg dive comes back in
# because it no longer needs the help and it costs less fuel.
# ROUND 2. Round 1 established: gate 0.42 at climb 14 / dive 20 PASSES
# with traverse 0.412 g -- better than V3b's 0.389 at gate 0.30, on less
# fuel (1.802 vs 1.845 kg). But its climb margin is 0.006 g, roughly 1.3 N
# of net force on a 222 N vehicle, which is below what this model can
# resolve. So the question is no longer "how late" alone; it is "how late
# with a climb margin the model can actually stand behind".
#
# Two directions, interleaved so the most valuable answers land first:
#   * push the boundary UP (0.43, 0.44) -- 0.45 already failed at 0.006 g
#     traverse and 0.50 is unreachable (peak M 0.453)
#   * hold gate 0.42 and BUY BACK climb margin with a shallower climb
# ROUND 3. Rounds 1-2 established gate 0.44 at climb 12 / dive 20 as the
# latest FP-verified ignition (traverse 0.368). A parallel investigation
# then showed dive 20 was never the bound -- the closed-form ceiling keeps
# rising with dive angle (0.43 at dive 9.9, 0.46 at dive 14, 0.47 at dive
# 16, and gate 0.50 opens at dive >= 22) because a steeper dive earns a
# higher derived top, which gives the vehicle more altitude AND more
# gravity to cross the gate before the floor terminates the dive.
#
# But the dive cannot be extended freely: pricing the pull-out that the
# flight model omits entirely (it switches gamma in ONE timestep) gives,
# against 15.4 g available at the dive exit,
#     dive 20 -> 9.5 g,  dive 25 -> 14.2 g,  dive 30 -> 19.9 g
# for a 30 m arc. Dive 30 is NOT flyable and dive 25 has 8% margin, so
# this round stops at 25 and leans on 22.
# ROUND 3. A parallel investigation recommends climb 12 / dive 30 / gate
# 0.50, estimating FP traverse ~0.30 g by applying a 0.781 "haircut" to
# the closed-form 0.415. That estimate is an EXTRAPOLATION and its own
# author flags the weakness: no FP flight has ever been run at dive > 20
# or at any gate > 0.45, and the one gate-0.45 pair on record collapsed
# (closed-form 0.212 -> FP 0.011, ratio 0.05). The haircut is a cliff near
# the boundary, not a scaling. Round 2 saw exactly that collapse at
# (14, 20, 0.45): 0.380 closed-form -> 0.006 FP.
#
# So this round FLIES the recommendation instead of trusting the estimate,
# and brackets it with the fallbacks, latest gate first.
CANDIDATES = [
    (12.0, 30.0, 0.50),     # the recommendation, untested territory
    (12.0, 28.0, 0.50),
    (12.0, 30.0, 0.49),
    (12.0, 25.0, 0.48),
    (12.0, 22.0, 0.47),
    (12.0, 25.0, 0.46),
    (12.0, 22.0, 0.46),
    (12.0, 22.0, 0.45),
]


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    print(f"V3a body + frozen wing, FP propulsion, floor {FLOOR_M} m")
    print(f"walking the gate DOWN from 0.42 until it passes "
          f"(traverse >= {TARGET_G} g, cutoff, fuel < "
          f"{d.burn_limit_kg:.3f} kg)\n")
    print(f"{'climb':>6} {'dive':>6} {'gate':>5} {'trav_g':>7} {'pow_g':>7} "
          f"{'margin':>7} {'fuel':>6} {'peakM':>6} {'cut':>5} {'runs':>5} "
          f" gate  lights at")
    rows, best = [], None
    for climb, dive, gate in CANDIDATES:
        cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                              dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=gate,
                          n_cells=162)
        r = fly(d, drag_model="buildup", climb_dive=cd, propulsion=fp)
        peak = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        ev = [f"{t:.1f}s {w}" for t, w in fp.trace.events]
        lit = next((e for e in ev if "ramjet_lit" in e), "-")
        ok = (r.motor_cutoff_reached and peak >= 1.0
              and r.min_traverse_accel_g >= TARGET_G
              and fuel < d.burn_limit_kg - 1e-6)
        rows.append(dict(climb=climb, dive=dive, gate=gate,
                         traverse=r.min_traverse_accel_g,
                         powered=r.min_powered_accel_g,
                         margin=r.min_powered_thrust_margin,
                         fuel_kg=fuel, peak_mach=peak,
                         cutoff=bool(r.motor_cutoff_reached),
                         fp_runs=fp.n_transients, passes=bool(ok),
                         events=ev))
        print(f"{climb:6.1f} {dive:6.1f} {gate:5.2f} "
              f"{r.min_traverse_accel_g:7.3f} {r.min_powered_accel_g:7.3f} "
              f"{r.min_powered_thrust_margin:7.3f} {fuel:6.3f} {peak:6.3f} "
              f"{str(r.motor_cutoff_reached):>5} {fp.n_transients:5d} "
              f" {'PASS' if ok else 'fail'}  {lit}", flush=True)
        (OUT / "v3c_late_bisect.json").write_text(json.dumps(rows, indent=2))
        if ok and (best is None or gate > best["gate"]):
            best = rows[-1]
            print(f"        ^^ PASSES at gate {gate:.2f} -- this is the "
                  f"latest ignition verified so far", flush=True)

    print()
    if best:
        print(f"LATEST FP-VERIFIED IGNITION: M {best['gate']:.2f} "
              f"(climb {best['climb']:.0f} / dive {best['dive']:.0f})")
        print(f"  traverse {best['traverse']:.3f} g, powered "
              f"{best['powered']:.3f} g, margin {best['margin']:.3f}, "
              f"fuel {best['fuel_kg']:.3f} kg")
        print(f"  vs V3b's gate 0.30: ignition {100*(best['gate']/0.30-1):+.0f}% "
              f"later in Mach")
    else:
        print("nothing in 0.34-0.42 passed; the V3b answer (gate 0.30) "
              "stands as the recommendation")
    print(f"\nwrote {OUT / 'v3c_late_bisect.json'}")


if __name__ == "__main__":
    main()
