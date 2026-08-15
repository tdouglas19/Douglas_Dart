"""V4 reachability screen: what ramjet gate Mach is even ATTAINABLE in the dive?

Method: fly with the gate set unreachably high (0.99) so the ramjet never
lights, and read `dive_exit_mach` -- the Mach the PULSEJET ALONE gets the
vehicle to before the pull-out arc has to start. Any gate above that number
can never fire in the dive, no matter how the rest of the design is tuned.
Once a gate is below it, lighting the ramjet mid-dive only adds thrust, so
the real dive exit is faster still -- this screen is the conservative bound.

This is deliberately the one question docs/v3_learnings_for_v4.md section 4.1
says must NOT be trusted from a closed-form screen alone ("can it REACH X" is
an acceleration question, and the closed-form/FP haircut collapses from 0.78
to 0.05 near exactly this kind of boundary). It is run here to SIZE the
search box, and every number it produces is re-checked under a thrust
haircut in simple_model_v4_campaign.py.

Usage: .venv/Scripts/python scripts/simple_model_v4_reach.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.simple_model_v4_baseline import (  # noqa: E402
    FP_PULSEJET_CEILING_M, fly, v4_geometry)
from simple_model.flight_sim import V3_FLOOR_ALTITUDE_M  # noqa: E402

UNREACHABLE_GATE = 0.99
DIVES = [10.0, 14.0, 18.0, 22.0, 26.0, 30.0]
TOPS = [600.0, 800.0, 1000.0, 1200.0]
CLIMB = 10.0
PULLOUT_NS = [2.0, 3.0]

FINE_DIVES = [6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
FINE_TOPS = [1000.0, 1100.0, 1200.0]
FINE_NS = [3.0, 4.0, 5.0]


def main() -> None:
    v4_geometry()
    print("Pulsejet-only dive-exit Mach (gate 0.99 = ramjet never lights).")
    print("A ramjet gate is reachable in the dive only if it is BELOW this.")
    print(f"Top of climb COMMANDED; FP pulsejet flame-out ceiling "
          f"~{FP_PULSEJET_CEILING_M:.0f} m.\n")
    for n in PULLOUT_NS:
        print(f"=== pull-out {n:.1f} g ===")
        print(f"{'top m':>7} | " + " ".join(f"{d:>6.0f}deg" for d in DIVES))
        print("-" * (10 + 10 * len(DIVES)))
        for top in TOPS:
            cells = []
            for dive in DIVES:
                r = fly(climb_deg=CLIMB, dive_deg=dive,
                        floor_m=V3_FLOOR_ALTITUDE_M, gate_mach=UNREACHABLE_GATE,
                        pullout_n=n, top_altitude_m=top)
                cells.append(f"{r.dive_exit_mach:9.3f}")
            print(f"{top:7.0f} | " + " ".join(cells))
        print()

    # Refinement around the optimum: shallow dives, tops at/below the FP
    # ceiling, firmer pull-outs (a tighter arc gives the dive back the
    # altitude the arc would otherwise eat).
    print("=== refinement: max reachable gate, tops at/below the FP ceiling ===")
    best = (0.0, None)
    for n in FINE_NS:
        print(f"\n--- pull-out {n:.1f} g ---")
        print(f"{'top m':>7} | " + " ".join(f"{d:>6.0f}deg" for d in FINE_DIVES))
        print("-" * (10 + 10 * len(FINE_DIVES)))
        for top in FINE_TOPS:
            cells = []
            for dive in FINE_DIVES:
                r = fly(climb_deg=CLIMB, dive_deg=dive,
                        floor_m=V3_FLOOR_ALTITUDE_M, gate_mach=UNREACHABLE_GATE,
                        pullout_n=n, top_altitude_m=top)
                cells.append(f"{r.dive_exit_mach:9.3f}")
                if r.dive_exit_mach > best[0]:
                    best = (r.dive_exit_mach, (top, dive, n, r.pullout_radius_m,
                                               r.pullout_entry_altitude_m))
            print(f"{top:7.0f} | " + " ".join(cells))
    top, dive, n, radius, entry = best[1]
    print(f"\nBEST pulsejet-only dive exit: M {best[0]:.3f} at top {top:.0f} m, "
          f"dive {dive:.0f} deg, pull-out {n:.1f} g "
          f"(R {radius:.0f} m, arc entry {entry:.0f} m)")
    print("Any ramjet gate at or above that Mach CANNOT light in the dive.")


if __name__ == "__main__":
    main()
