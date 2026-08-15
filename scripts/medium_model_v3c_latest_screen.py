"""How LATE can the ramjet light and still close Gate 3?

V3b closes at traverse 0.389 g against a 0.26 g target -- 50% of headroom
that is currently being spent on lighting the ramjet early.  Buying
ignition margin with it is a good trade in the real world: at M 0.30 the
ramjet lights on 66-95 N with a pilot at low ram pressure, while at M 0.45
it lights on 202 N.  Later ignition also burns less fuel (1.764 kg at gate
0.45 vs 1.845 kg at 0.30).

The V3b rule was "set the gate to the Mach at top of climb", which pins it
low.  But that rule was derived at the FROZEN dive angle of 9.89 deg.  The
hypothesis here is that DIVE ANGLE breaks the trade: a steeper dive
accelerates the vehicle to a high Mach while it is still high, so a late
gate can fire EARLY IN THE DIVE rather than deep in it -- late in Mach but
early in position, which is what actually mattered.

Closed-form screen (monkeypatching ramjet_simple's module-global gate --
it is read at call time, verified).  Closed-form is optimistic vs FP by
roughly 25% on traverse, so this ranks candidates; it does not certify
them.  FP verification follows separately.

Objective: MAXIMISE lightoff Mach subject to traverse >= 0.26 g, peak
M >= 1.0 with cutoff, and fuel under the 2.424 kg burn limit.
"""
from __future__ import annotations
import json, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                     # noqa: E402
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0          # V3_FLOOR_ALTITUDE_M clamp; lowest legal

CLIMBS = (6.0, 8.0, 10.0, 12.0, 14.0, 16.0)
DIVES = (9.890058542648028, 12.0, 14.0, 16.0, 18.0, 20.0)
GATES = (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    original_gate = rs.RAMJET_MIN_LIGHTOFF_MACH
    print(f"V3a body + frozen wing, closed-form propulsion, drag build-up")
    print(f"floor {FLOOR_M} m (the V3 clamp); baseline gate "
          f"{original_gate}; target traverse >= {TARGET_G} g")
    print(f"{len(CLIMBS)*len(DIVES)*len(GATES)} flights\n")

    rows = []
    try:
        for climb in CLIMBS:
            for dive in DIVES:
                cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                      dive_angle_deg=dive,
                                      floor_altitude_m=FLOOR_M)
                for gate in GATES:
                    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                    r = fly(d, drag_model="buildup", climb_dive=cd)
                    peak = max(s.mach for s in r.states)
                    fuel = max(s.fuel_burned_kg for s in r.states)
                    ok = (r.motor_cutoff_reached and peak >= 1.0
                          and r.min_traverse_accel_g >= TARGET_G
                          and fuel < d.burn_limit_kg - 1e-6)
                    rows.append(dict(
                        climb=climb, dive=dive, gate=gate,
                        traverse=r.min_traverse_accel_g,
                        powered=r.min_powered_accel_g,
                        margin=r.min_powered_thrust_margin,
                        peak=peak, cutoff=bool(r.motor_cutoff_reached),
                        fuel=fuel, passes=bool(ok)))
            best = max((q for q in rows if q["climb"] == climb
                        and q["passes"]), key=lambda q: q["gate"],
                       default=None)
            print(f"climb {climb:4.1f}: latest passing gate "
                  f"{best['gate'] if best else 'NONE':>6}"
                  + (f"  (dive {best['dive']:.1f}, traverse "
                     f"{best['traverse']:.3f}, fuel {best['fuel']:.3f})"
                     if best else ""), flush=True)
            (OUT / "v3c_latest_screen.json").write_text(
                json.dumps(rows, indent=2))
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original_gate

    ok = [q for q in rows if q["passes"]]
    print(f"\n{len(ok)} of {len(rows)} pass all gates")
    if not ok:
        print("nothing passes -- widen the box")
        return

    top = max(q["gate"] for q in ok)
    print(f"\nLATEST ignition that closes Gate 3: M {top:.2f}")
    print(f"{'climb':>6} {'dive':>6} {'gate':>5} {'trav':>6} {'pow':>6} "
          f"{'margin':>7} {'fuel':>6} {'peakM':>6}")
    for q in sorted((z for z in ok if z["gate"] >= top - 0.05),
                    key=lambda z: (-z["gate"], -z["traverse"])):
        print(f"{q['climb']:6.1f} {q['dive']:6.1f} {q['gate']:5.2f} "
              f"{q['traverse']:6.3f} {q['powered']:6.3f} "
              f"{q['margin']:7.3f} {q['fuel']:6.3f} {q['peak']:6.3f}")

    # Does dive angle buy late ignition? That is the hypothesis.
    print(f"\nlatest passing gate vs dive angle (any climb):")
    for dive in DIVES:
        g = [q["gate"] for q in ok if q["dive"] == dive]
        print(f"  dive {dive:5.1f} deg -> "
              f"{max(g) if g else 'NONE'}")
    print(f"\nwrote {OUT / 'v3c_latest_screen.json'}")


if __name__ == "__main__":
    main()
