"""ATTRIBUTION: the report credits the whole ceiling gain to "a bigger
PULSEJET".  But in the closed-form model BOTH engines are area-scaled off
the SAME body/throat diameters:

    pulsejet thrust ~ throat area      (choked neck)
    ramjet   thrust ~ inlet area = frontal area, and throat capacity

so growing the body by 1.2064x in area grows the RAMJET by 1.2064x too.
The report's isolation experiment scaled ONLY the pulsejet, so it cannot
see this.  This script separates the three effects on the IDENTICAL V3a
airframe (same drag, same length, same mass, same fuel budget):

    A  V3a, stock                              (baseline)
    B  V3a, pulsejet x1.2064 free              (report's experiment)
    C  V3a, ramjet   x1.2064 free              (never run)
    D  V3a, BOTH     x1.2064 free              (the real engine delta)
    E  235 mm as designed                      (engines bought with diameter)
    F  235 mm with the ramjet DE-scaled to V3a capture

Isp is preserved in every scaled case (thrust and fuel flow scaled
together), so no case wins by inventing free fuel.  It also prints WHERE
in Mach the binding traverse minimum sits, which is what decides which
engine is doing the work.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.flight_sim as fs                             # noqa: E402
import medium_model.ramjet_simple as rs                          # noqa: E402
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile             # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
TARGET_G, FLOOR_M = 0.26, 122.0
K = (0.13357894736842105 / 0.12161734042955713) ** 2      # 1.20638...

V3A = "docs/v3a_medium_model/design.json"
NEW = "docs/v3c_235mm/design.json"

_PJ0, _RJ0 = fs.pulsejet_thrust, fs.ramjet_thrust


def scaled(pj=1.0, rj=1.0):
    """Context: scale engine thrust AND fuel flow by the same factor."""
    def wrap_pj(*a, **k):
        r = _PJ0(*a, **k)
        if pj == 1.0:
            return r
        return r._replace(average_thrust_n=r.average_thrust_n * pj,
                          fuel_mass_flow_kg_per_s=r.fuel_mass_flow_kg_per_s * pj)

    def wrap_rj(*a, **k):
        r = _RJ0(*a, **k)
        if rj == 1.0:
            return r
        return r._replace(net_thrust_n=r.net_thrust_n * rj,
                          gross_thrust_n=r.gross_thrust_n * rj,
                          fuel_mass_flow_kg_per_s=r.fuel_mass_flow_kg_per_s * rj)
    return wrap_pj, wrap_rj


CASES = [
    ("A V3a stock",              V3A, 1.0, 1.0),
    ("B V3a pulsejet x1.2064",   V3A, K,   1.0),
    ("C V3a ramjet   x1.2064",   V3A, 1.0, K),
    ("D V3a BOTH     x1.2064",   V3A, K,   K),
    ("E 235mm as designed",      NEW, 1.0, 1.0),
    ("F 235mm ramjet de-scaled", NEW, 1.0, 1.0 / K),
]
DIVES = (20.0, 26.0)
CLIMBS = (10.0, 12.0, 14.0, 16.0)
GATES = tuple(round(0.44 + 0.01 * i, 2) for i in range(17))       # 0.44..0.60


def main() -> None:
    orig_gate = rs.RAMJET_MIN_LIGHTOFF_MACH
    rows = []
    try:
        for name, path, pjf, rjf in CASES:
            d = load_frozen_design(path)
            fs.pulsejet_thrust, fs.ramjet_thrust = scaled(pjf, rjf)
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
                        rows.append(dict(case=name, climb=climb, dive=dive,
                                         gate=gate,
                                         traverse=r.min_traverse_accel_g,
                                         notch_mach=r.min_traverse_accel_mach,
                                         peak=peak, fuel=fuel,
                                         passes=bool(ok)))
            print(f"  {name} done", flush=True)
    finally:
        fs.pulsejet_thrust, fs.ramjet_thrust = _PJ0, _RJ0
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig_gate

    print("\nCEILING (latest passing gate, best over climb) vs DIVE")
    print(f"  {'case':>26} |" + "".join(f"{x:>7.0f}" for x in DIVES) + "   best")
    for name, *_ in CASES:
        cells, best = [], None
        for dive in DIVES:
            g = [q["gate"] for q in rows
                 if q["case"] == name and q["dive"] == dive and q["passes"]]
            cells.append(f"{max(g):>7.2f}" if g else f"{'--':>7}")
            if g:
                best = max(g) if best is None else max(best, max(g))
        print(f"  {name:>26} |" + "".join(cells)
              + f"   {best if best else 'NONE'}")

    print("\nWHERE THE BINDING TRAVERSE MINIMUM SITS (climb 12 / dive 20)")
    print(f"  {'case':>26} {'gate':>5} {'traverse':>9} {'notch M':>8}")
    for name, *_ in CASES:
        for gate in (0.45, 0.50, 0.54):
            q = [z for z in rows if z["case"] == name and z["climb"] == 12.0
                 and z["dive"] == 20.0 and abs(z["gate"] - gate) < 1e-9]
            if q:
                z = q[0]
                print(f"  {name:>26} {gate:5.2f} {z['traverse']:9.3f} "
                      f"{z['notch_mach']:8.3f}")
    (OUT / "v3c_235mm_attribution.json").write_text(json.dumps(rows, indent=2))
    print(f"\nK (area ratio) = {K:.6f}")
    print(f"wrote {OUT/'v3c_235mm_attribution.json'}")


if __name__ == "__main__":
    main()
