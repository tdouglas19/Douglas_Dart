"""Does a BIGGER PULSEJET raise the M 0.45 lightoff wall?

The V3c screen (scripts/medium_model_v3c_latest_screen.py) found a hard
wall at gate M 0.45 on the V3a body: M 0.50 failed at all 36 climb x dive
combinations.  A wall that ignores the trajectory points at the engine --
the vehicle must reach the gate Mach on pulsejet thrust alone before the
ramjet takes over.  out_medium_model/v3b_cliff.json says a 235 mm chamber
at t/D 4.00 makes 149.2 N at M 0.15 vs V3a's 123.6 N (+21%).

The catch, and the reason this is a real experiment rather than a
formality: CD0 is FRONTAL-AREA referenced and the bigger chamber needs a
247.4 mm body instead of 225.2 mm, which is +20.6% frontal area.  The
thrust is +21% and the drag reference is +21%.  Whether that nets out
positive is what this measures.

Same method as the V3a screen: closed-form propulsion, drag build-up,
lightoff gate swept by monkeypatching ramjet_simple's module global (read
at call time).  BOTH vehicles are flown here on the identical grid in the
same process, so the comparison is not against a stale JSON.

Closed-form runs ~22% high on traverse vs FP; this ranks, it does not
certify.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                        # noqa: E402
from medium_model.design import fly, load_frozen_design        # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile           # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0                       # V3_FLOOR_ALTITUDE_M clamp

CLIMBS = (8.0, 10.0, 12.0, 14.0, 16.0)
DIVES = (14.0, 16.0, 18.0, 20.0)
GATES = (0.40, 0.45, 0.50, 0.55, 0.60, 0.65)

VEHICLES = {
    "v3a_225mm": "docs/v3a_medium_model/design.json",
    "v3c_235mm": "docs/v3c_235mm/design.json",
}


def screen(tag: str, path: str) -> list[dict]:
    d = load_frozen_design(path)
    g = d.geometry
    print(f"\n=== {tag}: body {g.diameter_m*1e3:.1f} mm dia, duct "
          f"{(g.chamber_length_m+g.throat_length_m)*1e3:.0f} mm, "
          f"burn cap {d.burn_limit_kg:.4f} kg ===", flush=True)
    rows = []
    for climb in CLIMBS:
        for dive in DIVES:
            cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                  dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
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
                    cutoff=bool(r.motor_cutoff_reached), fuel=fuel,
                    passes=bool(ok)))
        best = max((q for q in rows if q["climb"] == climb and q["passes"]),
                   key=lambda q: q["gate"], default=None)
        print(f"  climb {climb:4.1f}: latest passing gate "
              f"{best['gate'] if best else 'NONE':>6}"
              + (f"  (dive {best['dive']:.0f}, traverse {best['traverse']:.3f},"
                 f" peakM {best['peak']:.3f}, fuel {best['fuel']:.3f})"
                 if best else ""), flush=True)
    return rows


def gate_matrix(rows: list[dict], tag: str) -> None:
    """latest passing gate for each climb x dive cell."""
    print(f"\n{tag}: latest passing gate by climb (rows) x dive (cols)")
    print("  climb |" + "".join(f"{d:>7.0f}" for d in DIVES))
    for climb in CLIMBS:
        cells = []
        for dive in DIVES:
            g = [q["gate"] for q in rows
                 if q["climb"] == climb and q["dive"] == dive and q["passes"]]
            cells.append(f"{max(g):>7.2f}" if g else f"{'--':>7}")
        print(f"  {climb:5.1f} |" + "".join(cells))


def main() -> None:
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    t0 = time.perf_counter()
    n = len(CLIMBS) * len(DIVES) * len(GATES)
    print(f"{n} flights per vehicle x {len(VEHICLES)} vehicles; "
          f"target traverse >= {TARGET_G} g, peak M >= 1.0 with cutoff, "
          f"fuel under each design's own cap")
    all_rows: list[dict] = []
    try:
        for tag, path in VEHICLES.items():
            rows = screen(tag, path)
            all_rows += rows
            (OUT / "v3c_235mm_screen.json").write_text(
                json.dumps(all_rows, indent=2))
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original

    for tag in VEHICLES:
        gate_matrix([q for q in all_rows if q["vehicle"] == tag], tag)

    print("\nhead-to-head at each gate: best traverse over all climb x dive")
    print(f"  {'gate':>5} | {'v3a trav':>9} {'v3a peakM':>10} {'v3a fuel':>9} "
          f"| {'235 trav':>9} {'235 peakM':>10} {'235 fuel':>9} | {'d trav':>7}")
    for gate in GATES:
        cells = {}
        for tag in VEHICLES:
            sub = [q for q in all_rows if q["vehicle"] == tag and q["gate"] == gate]
            cells[tag] = max(sub, key=lambda q: q["traverse"]) if sub else None
        a, b = cells["v3a_225mm"], cells["v3c_235mm"]
        print(f"  {gate:5.2f} | {a['traverse']:9.3f} {a['peak']:10.3f} "
              f"{a['fuel']:9.3f} | {b['traverse']:9.3f} {b['peak']:10.3f} "
              f"{b['fuel']:9.3f} | {b['traverse']-a['traverse']:+7.3f}")

    print("\nverdict inputs")
    for tag in VEHICLES:
        ok = [q for q in all_rows if q["vehicle"] == tag and q["passes"]]
        if not ok:
            print(f"  {tag}: NOTHING PASSES on this grid")
            continue
        top = max(q["gate"] for q in ok)
        best = max((q for q in ok if q["gate"] == top),
                   key=lambda q: q["traverse"])
        print(f"  {tag}: latest passing gate M {top:.2f} "
              f"({len(ok)}/{len(CLIMBS)*len(DIVES)*len(GATES)} cells pass); "
              f"best there climb {best['climb']:.0f}/dive {best['dive']:.0f} -> "
              f"traverse {best['traverse']:.3f}, powered {best['powered']:.3f}, "
              f"margin {best['margin']:.3f}, peakM {best['peak']:.3f}, "
              f"fuel {best['fuel']:.3f}")
    print(f"\nwrote {OUT/'v3c_235mm_screen.json'}  ({time.perf_counter()-t0:.0f}s)")


if __name__ == "__main__":
    main()
