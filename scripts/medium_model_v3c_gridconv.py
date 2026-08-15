"""How much pulsejet thrust does grid refinement remove, across the flight?

The V3c high-fidelity flight failed outright: at n_cells 324 the pulsejet
made ~105 N where the n_cells 162 march had ~119 N, the 12 deg climb
DECELERATED (M 0.118 -> 0.111 over 81 s), and the vehicle never reached
the lightoff gate.  Every result in the V3b and V3c campaigns was flown at
162.

So the question is no longer "which trajectory is best" but "how much of
the answer is grid".  This measures the pulsejet alone -- no flight, no
trajectory -- at the conditions the mission actually passes through, on
the three fidelity tiers.

If the thrust deficit is roughly constant with Mach, the campaign's
CONCLUSIONS survive and only the absolute margins move.  If it grows with
Mach, then the late-ignition result specifically is an artifact, because
late ignition depends on thrust at the TOP of the pulsejet's Mach range.
"""
from __future__ import annotations
import json, os, time
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
# (mach, altitude_m) along the actual V3b/V3c mission profile
CONDITIONS = [
    (0.118, 30.0),      # release
    (0.20, 250.0),      # early climb
    (0.30, 580.0),      # top of climb (V3b lightoff)
    (0.35, 500.0),      # early dive
    (0.44, 220.0),      # V3c lightoff
    (0.50, 150.0),      # dive exit
    (0.60, 122.0),      # drag strip entry
]
TIERS = [(162, "MARCH"), (324, "CONFIRM")]


def main():
    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    d = load_frozen_design("docs/v3a_medium_model/design.json")
    geom = spec_from_geometry(d.geometry).pulsejet_geometry()
    try:
        gas = reference_gas(phi=1.0, fuel="propane")
    except TypeError:
        gas = reference_gas(phi=1.0)
    rv = reference_valve()
    s = geom.chamber_diameter / 0.078
    valve = replace(rv, petal_length=rv.petal_length*s,
                    petal_width=rv.petal_width*s,
                    petal_thickness=rv.petal_thickness*s,
                    port_area=rv.port_area*s*s, max_lift=rv.max_lift*s,
                    seat_preload=rv.seat_preload*s)
    intake = IntakeDesign(duct_length=0.060*s, duct_diameter=0.050*s,
                          plenum_volume=5e-5*s**3, orientation="side",
                          bl_momentum_fraction=0.6)

    print("V3a duct, propane, phi 1.0 -- pulsejet thrust vs grid\n")
    print(f"{'M':>6} {'alt_m':>7} " +
          " ".join(f"{lbl+'('+str(n)+')':>14}" for n, lbl in TIERS) +
          f" {'delta':>9} {'%':>7}")
    rows = []
    for mach, alt in CONDITIONS:
        vals = {}
        for n, lbl in TIERS:
            t0 = time.perf_counter()
            try:
                r = pulsejet_thrust(mach=mach, altitude_m=alt, gas=gas,
                                    geom=geom, valve=valve, intake=intake,
                                    numerics=Numerics(n_cells=n),
                                    t_end=1.5, stop_when_converged=True)
                alive = (r.p_max_ratio - r.p_min_ratio) > 0.25
                vals[n] = dict(thrust_n=r.thrust_n, alive=bool(alive),
                               freq_hz=r.frequency_hz,
                               p_span=r.p_max_ratio - r.p_min_ratio,
                               wall_s=time.perf_counter() - t0)
            except Exception as exc:
                vals[n] = dict(thrust_n=float("nan"), alive=False,
                               error=f"{type(exc).__name__}: {exc}")
        a, b = vals[TIERS[0][0]]["thrust_n"], vals[TIERS[1][0]]["thrust_n"]
        delta = b - a
        pct = 100.0 * delta / a if a else float("nan")
        rows.append(dict(mach=mach, altitude_m=alt, tiers=vals,
                         delta_n=delta, delta_pct=pct))
        cells = []
        for n, lbl in TIERS:
            v = vals[n]
            cells.append(f"{v['thrust_n']:10.1f}"
                         + ("  " if v["alive"] else " D"))
        print(f"{mach:6.3f} {alt:7.0f} " + " ".join(f"{c:>14}" for c in cells)
              + f" {delta:9.1f} {pct:7.1f}", flush=True)
        (OUT / "v3c_gridconv.json").write_text(json.dumps(rows, indent=2))

    print("\n('D' = duct did not sustain a cycle at that grid)")
    good = [r for r in rows if r["delta_pct"] == r["delta_pct"]]
    if good:
        lo = min(good, key=lambda r: r["mach"])
        hi = max(good, key=lambda r: r["mach"])
        print(f"\nthrust deficit at M {lo['mach']:.3f}: {lo['delta_pct']:+.1f}%")
        print(f"thrust deficit at M {hi['mach']:.3f}: {hi['delta_pct']:+.1f}%")
        if abs(hi["delta_pct"] - lo["delta_pct"]) < 3.0:
            print("-> roughly CONSTANT with Mach: the campaign's orderings "
                  "survive, absolute margins shift")
        else:
            print("-> GROWS with Mach: the late-ignition result specifically "
                  "is grid-dependent, because it leans on thrust at the top "
                  "of the pulsejet's Mach range")
    print(f"\nwrote {OUT / 'v3c_gridconv.json'}")


if __name__ == "__main__":
    main()
