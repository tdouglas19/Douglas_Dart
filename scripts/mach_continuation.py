"""Operating-branch thrust vs Mach by CONTINUATION: each flight point is
seeded from the previous point's limit-cycle state, exactly as a real
vehicle accelerates with the engine running. The cold-start sweep
(thrust_vs_mach.py) maps the other branch; together they show the
start/quench hysteresis of the nonlinear oscillator."""
import argparse
import csv
import os
import sys
import time
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from pulsejet_fp import (IntakeDesign, Numerics, PulsejetEngine, ThrustResult,
                         reference_gas, reference_geometry, reference_valve)
from pulsejet_fp.query import analyze_cycles

OUT = os.path.join(os.path.dirname(__file__), "..", "out")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--m-max", type=float, default=0.9)
    ap.add_argument("--dm", type=float, default=0.05)
    ap.add_argument("--t-first", type=float, default=0.25)
    ap.add_argument("--t-point", type=float, default=0.12)
    ap.add_argument("--n-cells", type=int, default=200)
    ap.add_argument("--preload", type=float, default=0.5e-3)
    ap.add_argument("--tag", type=str, default="fp1_continuation")
    ap.add_argument("--side-inlet", action="store_true")
    args = ap.parse_args()

    gas = reference_gas()
    geom = reference_geometry()
    valve = replace(reference_valve(), seat_preload=args.preload)
    intake = IntakeDesign(orientation="side") if args.side_inlet else None

    machs = []
    m = 0.0
    while m <= args.m_max + 1e-9:
        machs.append(round(m, 3))
        m += args.dm

    snap = None
    rows = []
    results = []
    t_start = time.time()
    for i, mach in enumerate(machs):
        eng = PulsejetEngine(gas, geom, valve, mach=mach, intake=intake,
                             numerics=Numerics(n_cells=args.n_cells))
        if snap is not None:
            eng.restore(snap)
        t_end = args.t_first if snap is None else args.t_point
        t0 = time.time()
        hist = eng.run(t_end)
        wall = time.time() - t0

        an = analyze_cycles(hist, eng.p_a, gas,
                            settle_frac=0.35 if snap is None else 0.25)
        tail = hist["t"] > hist["t"][-1] - 0.25 * (hist["t"][-1] - hist["t"][0])
        quenched = bool(np.max(hist["T_max"][tail]) < 900.0)
        if "thrust" in an and not quenched:
            status = "converged" if an["converged"] else "unconverged"
            res = ThrustResult(
                thrust_n=an["thrust"], status=status,
                frequency_hz=an["frequency"],
                thrust_surface_n=an["thrust_surf"],
                mdot_air_kg_s=an["mdot_air_mix"] - an["mdot_fuel"],
                mdot_fuel_kg_s=an["mdot_fuel"],
                tsfc_kg_per_n_hr=(an["mdot_fuel"] * 3600.0 / an["thrust"]
                                  if an["thrust"] > 1e-6 else float("nan")),
                p_min_ratio=an["p_min_ratio"], p_max_ratio=an["p_max_ratio"],
                rayleigh_index=an["rayleigh"], n_cycles=an["n_cycles"],
                mach=mach)
            snap = eng.snapshot()  # seed the next point from a live cycle
        else:
            res = ThrustResult(thrust_n=float("nan"),
                               status="quenched" if quenched else "unconverged",
                               mach=mach)
            # do NOT reseed from a dead state; keep the last live snapshot
        results.append(res)
        rows.append((mach, res, wall))
        print(f"M={mach:4.2f}  F={res.thrust_n:7.2f} N  "
              f"f={res.frequency_hz:6.1f} Hz  "
              f"pmin/pmax={res.p_min_ratio:5.3f}/{res.p_max_ratio:5.3f}  "
              f"[{res.status}]  ({wall:.0f}s)", flush=True)
        if res.status == "quenched" and i > 0:
            # one retry from the last live snapshot already happened by
            # construction; a quench on the operating branch is a result
            pass

    os.makedirs(OUT, exist_ok=True)
    csv_path = os.path.join(OUT, f"thrust_vs_mach_{args.tag}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mach", "thrust_n", "thrust_surface_n", "frequency_hz",
                    "mdot_air_g_s", "mdot_fuel_g_s", "tsfc_kg_n_hr",
                    "p_min_ratio", "p_max_ratio", "rayleigh", "status",
                    "n_cycles", "wall_s"])
        for m_, r, wall in rows:
            w.writerow([m_, f"{r.thrust_n:.3f}", f"{r.thrust_surface_n:.3f}",
                        f"{r.frequency_hz:.2f}",
                        f"{r.mdot_air_kg_s*1e3:.2f}",
                        f"{r.mdot_fuel_kg_s*1e3:.3f}",
                        f"{r.tsfc_kg_per_n_hr:.3f}",
                        f"{r.p_min_ratio:.3f}", f"{r.p_max_ratio:.3f}",
                        f"{r.rayleigh_index:.3e}", r.status, r.n_cycles,
                        f"{wall:.0f}"])
    print(f"CSV -> {csv_path}")

    from pulsejet_fp.diagnostics import thrust_vs_mach_plot
    plot_path = os.path.join(OUT, f"thrust_vs_mach_{args.tag}.png")
    thrust_vs_mach_plot(machs, results, plot_path,
                        title="FP-1 valved pulsejet: thrust vs Mach, "
                              "operating branch (continuation, sea level)")
    print(f"plot -> {plot_path}")
    print(f"total wall: {time.time()-t_start:.0f} s")


if __name__ == "__main__":
    main()
