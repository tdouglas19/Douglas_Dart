"""Run the FP-1 reference engine at one flight condition, print the cycle
summary, and save the standard diagnostic plots to out/."""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from pulsejet_fp import Numerics, pulsejet_thrust
from pulsejet_fp.atmosphere import ambient
from pulsejet_fp.diagnostics import overview_plot, standard_cycle_plot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mach", type=float, default=0.0)
    ap.add_argument("--alt", type=float, default=0.0)
    ap.add_argument("--t-end", type=float, default=0.30)
    ap.add_argument("--n-cells", type=int, default=300)
    ap.add_argument("--tag", type=str, default="")
    args = ap.parse_args()

    t0 = time.time()
    res = pulsejet_thrust(
        mach=args.mach, altitude_m=args.alt, t_end=args.t_end,
        numerics=Numerics(n_cells=args.n_cells), keep_traces=True)
    wall = time.time() - t0

    print(f"status          : {res.status}")
    print(f"thrust (eq.24)  : {res.thrust_n:8.2f} N")
    print(f"thrust (eq.25)  : {res.thrust_surface_n:8.2f} N (surface check)")
    print(f"frequency       : {res.frequency_hz:8.1f} Hz")
    print(f"air flow        : {res.mdot_air_kg_s*1e3:8.2f} g/s")
    print(f"fuel flow       : {res.mdot_fuel_kg_s*1e3:8.3f} g/s")
    print(f"TSFC            : {res.tsfc_kg_per_n_hr:8.3f} kg/(N hr)")
    print(f"p_min/p_max     : {res.p_min_ratio:6.3f} / {res.p_max_ratio:6.3f}")
    print(f"Rayleigh index  : {res.rayleigh_index:10.3e}")
    print(f"cycles detected : {res.n_cycles}")
    print(f"wall time       : {wall:6.1f} s")

    p_a, T_a = ambient(args.alt)
    tag = args.tag or f"m{args.mach:.2f}"
    outdir = os.path.join(os.path.dirname(__file__), "..", "out")
    os.makedirs(outdir, exist_ok=True)
    h = res.traces
    t_ms = h["t"][-1] * 1e3
    standard_cycle_plot(h, p_a, T_a,
                        os.path.join(outdir, f"cycle_{tag}.png"),
                        title=f"FP-1  M={args.mach}  [{res.status}]")
    standard_cycle_plot(h, p_a, T_a,
                        os.path.join(outdir, f"cycle_{tag}_zoom.png"),
                        t_window_ms=(max(0.0, t_ms - 40.0), t_ms),
                        title=f"FP-1  M={args.mach}  last 40 ms  [{res.status}]")
    overview_plot(h, p_a, os.path.join(outdir, f"overview_{tag}.png"),
                  title=f"FP-1  M={args.mach}")
    print(f"plots -> out/cycle_{tag}.png, cycle_{tag}_zoom.png, overview_{tag}.png")


if __name__ == "__main__":
    main()
