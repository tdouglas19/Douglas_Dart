"""Nail down the 235 mm vehicle's lightoff wall, and audit WHY it moved.

Two questions the coarse screen leaves open.

1. The coarse grid steps the gate by 0.05, so "0.50 passes, 0.55 fails"
   locates the wall only to +/-0.05.  Refine on 0.01 steps, and push the
   dive past the screen's 20 deg ceiling to see whether the "steeper dive
   buys lateness" trend has more in it for this vehicle.

2. HONESTY CHECK on the screen itself.  The +21% thrust claim comes from
   the FIRST-PRINCIPLES pulsejet (out_medium_model/v3b_cliff.json:
   149.2 N vs V3a's 123.6 N at M 0.15).  The screen flies CLOSED-FORM
   propulsion.  If the closed-form engine does not scale the same way,
   the screen's answer is being driven by a different number than the one
   that motivated the design.  Print both engines' thrust side by side so
   that is visible rather than assumed.  (Only the closed-form engine is
   evaluated here -- FP engine calls are ~20 s each and another process is
   using the CPU.  The FP figures are quoted from the stored cliff run.)
"""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                        # noqa: E402
from medium_model.design import fly, load_frozen_design        # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile           # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0

V3A = "docs/v3a_medium_model/design.json"
NEW = "docs/v3c_235mm/design.json"


def engine_audit() -> None:
    from medium_model.pulsejet_simple import pulsejet_thrust
    a = load_frozen_design(V3A).geometry
    b = load_frozen_design(NEW).geometry

    def t(g, mach):
        return pulsejet_thrust(g.diameter_m, g.chamber_length_m,
                               g.throat_diameter_m, g.throat_length_m,
                               mach, 122.0, g.fuel)

    print("closed-form pulsejet thrust (the engine the screen actually flies)")
    print(f"  {'Mach':>5} {'V3a 225mm':>10} {'235mm':>10} {'ratio':>7} "
          f"{'V3a Hz':>7} {'235 Hz':>7} {'op':>8}")
    for mach in (0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.70, 0.90):
        ra, rb = t(a, mach), t(b, mach)
        print(f"  {mach:5.2f} {ra.average_thrust_n:10.2f} "
              f"{rb.average_thrust_n:10.2f} "
              f"{rb.average_thrust_n/ra.average_thrust_n:7.4f} "
              f"{ra.frequency_hz:7.1f} {rb.frequency_hz:7.1f} "
              f"{str(ra.operable)[0]+'/'+str(rb.operable)[0]:>8}")
    print("  FP (out_medium_model/v3b_cliff.json + V3a reference), M 0.15 / 60 m:"
          "  123.6 -> 149.2 N, ratio 1.207")


def refine() -> list[dict]:
    d = load_frozen_design(NEW)
    gates = [round(0.48 + 0.01 * i, 2) for i in range(11)]      # 0.48 .. 0.58
    climbs = (10.0, 12.0, 14.0, 16.0, 18.0)
    dives = (18.0, 20.0, 22.0, 24.0, 26.0)
    print(f"\n235 mm refinement: {len(climbs)*len(dives)*len(gates)} flights, "
          f"gates {gates[0]}..{gates[-1]} step 0.01")
    rows = []
    for climb in climbs:
        for dive in dives:
            cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                  dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
            for gate in gates:
                rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                r = fly(d, drag_model="buildup", climb_dive=cd)
                peak = max(s.mach for s in r.states)
                fuel = max(s.fuel_burned_kg for s in r.states)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                rows.append(dict(climb=climb, dive=dive, gate=gate,
                                 traverse=r.min_traverse_accel_g,
                                 powered=r.min_powered_accel_g,
                                 margin=r.min_powered_thrust_margin,
                                 peak=peak, fuel=fuel,
                                 cutoff=bool(r.motor_cutoff_reached),
                                 passes=bool(ok)))
    print("  latest passing gate by climb (rows) x dive (cols)")
    print("  climb |" + "".join(f"{x:>7.0f}" for x in dives))
    for climb in climbs:
        cells = []
        for dive in dives:
            g = [q["gate"] for q in rows
                 if q["climb"] == climb and q["dive"] == dive and q["passes"]]
            cells.append(f"{max(g):>7.2f}" if g else f"{'--':>7}")
        print(f"  {climb:5.1f} |" + "".join(cells))
    return rows


def main() -> None:
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    try:
        engine_audit()
        rows = refine()
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original
    (OUT / "v3c_235mm_refine.json").write_text(json.dumps(rows, indent=2))

    ok = [q for q in rows if q["passes"]]
    if ok:
        top = max(q["gate"] for q in ok)
        print(f"\n235 mm latest passing gate (0.01 resolution): M {top:.2f}")
        print(f"  {'climb':>6} {'dive':>5} {'gate':>5} {'trav':>6} {'pow':>7} "
              f"{'margin':>7} {'peakM':>6} {'fuel':>6}")
        for q in sorted((z for z in ok if z["gate"] >= top - 0.01),
                        key=lambda z: (-z["gate"], -z["traverse"])):
            print(f"  {q['climb']:6.1f} {q['dive']:5.1f} {q['gate']:5.2f} "
                  f"{q['traverse']:6.3f} {q['powered']:7.3f} "
                  f"{q['margin']:7.3f} {q['peak']:6.3f} {q['fuel']:6.3f}")
    print(f"\nwrote {OUT/'v3c_235mm_refine.json'}")


if __name__ == "__main__":
    main()
