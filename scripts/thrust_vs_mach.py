"""Thrust vs flight Mach sweep for the FP-1 reference design (derivation.md
#13, final step). Runs Mach points in parallel worker processes, writes a CSV
and the summary plot."""
import argparse
import csv
import os
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

OUT = os.path.join(os.path.dirname(__file__), "..", "out")


def run_point(args):
    mach, t_end, n_cells, preload = args
    from dataclasses import replace
    from pulsejet_fp import Numerics, pulsejet_thrust, reference_valve
    valve = reference_valve()
    if preload is not None:
        valve = replace(valve, seat_preload=preload)
    t0 = time.time()
    res = pulsejet_thrust(mach=mach, t_end=t_end, valve=valve,
                          numerics=Numerics(n_cells=n_cells))
    res.traces = None
    return mach, res, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--m-max", type=float, default=0.9)
    ap.add_argument("--dm", type=float, default=0.05)
    ap.add_argument("--t-end", type=float, default=0.30)
    ap.add_argument("--n-cells", type=int, default=300)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--tag", type=str, default="fp1")
    ap.add_argument("--preload", type=float, default=None)
    args = ap.parse_args()

    machs = []
    m = 0.0
    while m <= args.m_max + 1e-9:
        machs.append(round(m, 3))
        m += args.dm

    jobs = [(m, args.t_end, args.n_cells, args.preload) for m in machs]
    t0 = time.time()
    with Pool(processes=args.workers) as pool:
        out = pool.map(run_point, jobs)
    out.sort(key=lambda r: r[0])

    os.makedirs(OUT, exist_ok=True)
    csv_path = os.path.join(OUT, f"thrust_vs_mach_{args.tag}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mach", "thrust_n", "thrust_surface_n", "frequency_hz",
                    "mdot_air_g_s", "mdot_fuel_g_s", "tsfc_kg_n_hr",
                    "p_min_ratio", "p_max_ratio", "rayleigh", "status",
                    "n_cycles", "wall_s"])
        for m, r, wall in out:
            w.writerow([m, f"{r.thrust_n:.3f}", f"{r.thrust_surface_n:.3f}",
                        f"{r.frequency_hz:.2f}",
                        f"{r.mdot_air_kg_s*1e3:.2f}",
                        f"{r.mdot_fuel_kg_s*1e3:.3f}",
                        f"{r.tsfc_kg_per_n_hr:.3f}",
                        f"{r.p_min_ratio:.3f}", f"{r.p_max_ratio:.3f}",
                        f"{r.rayleigh_index:.3e}", r.status, r.n_cycles,
                        f"{wall:.0f}"])
    print(f"CSV -> {csv_path}")

    from pulsejet_fp.diagnostics import thrust_vs_mach_plot
    results = [r for _, r, _ in out]
    plot_path = os.path.join(OUT, f"thrust_vs_mach_{args.tag}.png")
    thrust_vs_mach_plot([m for m, _, _ in out], results, plot_path,
                        title="FP-1 valved pulsejet: thrust vs flight Mach "
                              "(sea level, first-principles transient model)")
    print(f"plot -> {plot_path}")
    for m, r, wall in out:
        print(f"M={m:4.2f}  F={r.thrust_n:7.2f} N  f={r.frequency_hz:6.1f} Hz "
              f" pmin/pmax={r.p_min_ratio:5.3f}/{r.p_max_ratio:5.3f} "
              f" [{r.status}]  ({wall:.0f}s)")
    print(f"total wall: {time.time()-t0:.0f} s")


if __name__ == "__main__":
    main()
