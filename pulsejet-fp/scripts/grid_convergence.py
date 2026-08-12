"""Grid-convergence evidence: run the M=0 reference case at three
resolutions and compare cycle-averaged thrust and frequency. The scheme is
2nd-order; the limit-cycle observables should approach a limit as dx -> 0."""
import os
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


def run_n(n):
    from pulsejet_fp import Numerics, pulsejet_thrust
    t0 = time.time()
    r = pulsejet_thrust(mach=0.0, t_end=0.20, numerics=Numerics(n_cells=n))
    return n, r.thrust_n, r.frequency_hz, r.p_min_ratio, r.p_max_ratio, \
        r.status, time.time() - t0


def main():
    ns = [150, 200, 300, 450]
    with Pool(processes=len(ns)) as pool:
        rows = pool.map(run_n, ns)
    print(f"{'N':>5} {'thrust N':>10} {'freq Hz':>9} {'p_min':>7} "
          f"{'p_max':>7} {'status':>12} {'wall s':>7}")
    for n, F, f, pmin, pmax, st, w in rows:
        print(f"{n:>5} {F:>10.3f} {f:>9.1f} {pmin:>7.3f} {pmax:>7.3f} "
              f"{st:>12} {w:>7.0f}")


if __name__ == "__main__":
    main()
