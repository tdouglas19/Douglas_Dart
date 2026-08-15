"""CONTROL: separate the pulsejet effect from the dive-angle effect.

The 235 mm refinement reached gate M 0.54 -- but only at dive 26 deg,
outside the 20 deg ceiling of both the original V3c screen and the coarse
screen here.  The established "V3a walls at 0.45" fact was measured with
dive <= 20 deg too.  So the 0.45 -> 0.54 improvement is currently
CONFOUNDED: part engine, part trajectory box.

This flies V3a on the IDENTICAL extended grid (dive 18-26 deg, gates
0.44-0.58 at 0.01) so the two effects can be read apart:
  * if V3a also climbs past 0.45 at dive 26, the dive box was the binding
    constraint and the pulsejet bought less than it looks;
  * if V3a still walls at 0.45 with dive 26 available, the extra thrust is
    doing the work.
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

CLIMBS = (10.0, 12.0, 14.0, 16.0, 18.0)
DIVES = (18.0, 20.0, 22.0, 24.0, 26.0)
GATES = tuple(round(0.44 + 0.01 * i, 2) for i in range(15))     # 0.44..0.58

VEHICLES = {"v3a_225mm": "docs/v3a_medium_model/design.json",
            "v3c_235mm": "docs/v3c_235mm/design.json"}


def main() -> None:
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    rows: list[dict] = []
    try:
        for tag, path in VEHICLES.items():
            d = load_frozen_design(path)
            print(f"\n=== {tag} (burn cap {d.burn_limit_kg:.4f} kg) ===",
                  flush=True)
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
                            vehicle=tag, climb=climb, dive=dive, gate=gate,
                            traverse=r.min_traverse_accel_g,
                            powered=r.min_powered_accel_g,
                            margin=r.min_powered_thrust_margin, peak=peak,
                            fuel=fuel, cutoff=bool(r.motor_cutoff_reached),
                            passes=bool(ok)))
                print(f"  climb {climb:4.1f} done", flush=True)
            (OUT / "v3c_235mm_control.json").write_text(json.dumps(rows, indent=2))
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original

    for tag in VEHICLES:
        sub = [q for q in rows if q["vehicle"] == tag]
        print(f"\n{tag}: latest passing gate, climb (rows) x dive (cols)")
        print("  climb |" + "".join(f"{x:>7.0f}" for x in DIVES))
        for climb in CLIMBS:
            cells = []
            for dive in DIVES:
                g = [q["gate"] for q in sub
                     if q["climb"] == climb and q["dive"] == dive and q["passes"]]
                cells.append(f"{max(g):>7.2f}" if g else f"{'--':>7}")
            print(f"  {climb:5.1f} |" + "".join(cells))

    print("\nCEILING vs DIVE ANGLE (best over all climbs) -- the control")
    print(f"  {'dive':>5} | {'v3a ceiling':>12} {'235 ceiling':>12} {'delta':>7}")
    for dive in DIVES:
        out = {}
        for tag in VEHICLES:
            g = [q["gate"] for q in rows
                 if q["vehicle"] == tag and q["dive"] == dive and q["passes"]]
            out[tag] = max(g) if g else None
        a, b = out["v3a_225mm"], out["v3c_235mm"]
        print(f"  {dive:5.0f} | {a if a else 'NONE':>12} {b if b else 'NONE':>12} "
              f"{(b-a) if (a and b) else float('nan'):>+7.2f}")

    print("\noverall")
    for tag in VEHICLES:
        ok = [q for q in rows if q["vehicle"] == tag and q["passes"]]
        if not ok:
            print(f"  {tag}: nothing passes"); continue
        top = max(q["gate"] for q in ok)
        best = max((q for q in ok if q["gate"] == top), key=lambda q: q["traverse"])
        print(f"  {tag}: M {top:.2f} at climb {best['climb']:.0f}/dive "
              f"{best['dive']:.0f} -> traverse {best['traverse']:.3f}, "
              f"powered {best['powered']:.3f}, margin {best['margin']:.3f}, "
              f"peakM {best['peak']:.3f}, fuel {best['fuel']:.3f} kg")
    print(f"\nwrote {OUT/'v3c_235mm_control.json'}")


if __name__ == "__main__":
    main()
