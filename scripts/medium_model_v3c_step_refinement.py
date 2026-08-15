"""Does refining the propulsion time step CHANGE the answer, or only its
appearance?

The user's observation is that the FP thrust trace is "a little staggery".
It is: thrust is held constant between engine re-convergences, which fire
on drift triggers of dM 0.05 / d_alt 250 ft, giving ~38 solves across a
70 s powered phase. Visually it is a staircase.

Two very different things could be true, and they call for opposite
responses:

  (a) The staircase is COSMETIC. The held value is a good average over the
      interval, errors alternate sign and cancel, and the trajectory is
      converged. Then refining is a presentation fix and the V3b/V3c
      numbers stand as they are.

  (b) The staircase is BIASED. Holding thrust constant while the vehicle
      accelerates systematically under- or over-states impulse -- during
      the transonic push, thrust is rising steeply and monotonically, so a
      held value is stale-low for the whole interval. Then every number in
      this campaign is off in a knowable direction and the coarse results
      need correcting, not smoothing.

The difference is visible only by refining and watching the answer move,
so: same configuration, four step sizes, and look at whether the metrics
drift monotonically (bias) or jitter (discretisation noise).

The drift triggers of dM 0.05 / d_alt 250 ft are the VALIDATED CEILING of
the ramjet-fp continuation campaign, not a default chosen for speed, so
refinement is the safe direction -- it only costs solves.

Usage:
  python scripts/medium_model_v3c_step_refinement.py --climb C --dive D
         --lightoff G [--floor F] [--n-cells N]
"""
from __future__ import annotations
import argparse, json, os, time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import matplotlib                                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
METRES_PER_FOOT = 0.3048

# (mach_step, altitude_step_m, label). 0.05/250 ft is the current default
# and the validated ceiling; the rest refine from there.
LADDER = [
    (0.05, 250.0 * METRES_PER_FOOT, "default (validated ceiling)"),
    (0.03, 150.0 * METRES_PER_FOOT, "refined"),
    (0.02, 100.0 * METRES_PER_FOOT, "fine"),
    (0.01, 50.0 * METRES_PER_FOOT, "very fine"),
]


def parse():
    p = argparse.ArgumentParser()
    p.add_argument("--climb", type=float, required=True)
    p.add_argument("--dive", type=float, required=True)
    p.add_argument("--lightoff", type=float, required=True)
    p.add_argument("--floor", type=float, default=122.0)
    p.add_argument("--n-cells", type=int, default=162)
    p.add_argument("--dt", type=float, default=0.02)
    p.add_argument("--design", default="docs/v3a_medium_model/design.json")
    p.add_argument("--tag", default="v3c")
    return p.parse_args()


def main():
    a = parse()
    d = load_frozen_design(a.design)
    spec = spec_from_geometry(d.geometry)

    print(f"Propulsion step refinement -- {a.design}")
    print(f"climb {a.climb:g} / dive {a.dive:g} / floor {a.floor:g}, "
          f"lightoff M {a.lightoff:g}, n_cells {a.n_cells}, dt {a.dt:g} s\n")
    print(f"{'dM':>5} {'d_alt_m':>8} {'solves':>7} {'trav_g':>8} "
          f"{'pow_g':>7} {'peakM':>7} {'fuel':>7} {'cut':>5} {'wall_s':>7}"
          f"  note")

    rows = []
    for mstep, astep, note in LADDER:
        cd = ClimbDiveProfile(initial_climb_angle_deg=a.climb,
                              dive_angle_deg=a.dive,
                              floor_altitude_m=a.floor)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=a.lightoff,
                          n_cells=a.n_cells, mach_step=mstep,
                          altitude_step_m=astep)
        t0 = time.perf_counter()
        r = fly(d, drag_model="buildup", climb_dive=cd, propulsion=fp,
                dt_s=a.dt)
        wall = time.perf_counter() - t0
        peak = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        rows.append(dict(
            mach_step=mstep, altitude_step_m=astep, note=note,
            reconvergences=len(fp.trace.time_s), fp_runs=fp.n_transients,
            traverse_g=r.min_traverse_accel_g,
            powered_g=r.min_powered_accel_g,
            margin=r.min_powered_thrust_margin, peak_mach=peak,
            fuel_kg=fuel, cutoff=bool(r.motor_cutoff_reached),
            wall_s=wall,
            trace=dict(time_s=fp.trace.time_s,
                       pulsejet_thrust_n=fp.trace.pulsejet_thrust_n,
                       ramjet_thrust_n=fp.trace.ramjet_thrust_n,
                       mach=fp.trace.mach)))
        print(f"{mstep:5.2f} {astep:8.1f} {len(fp.trace.time_s):7d} "
              f"{r.min_traverse_accel_g:8.4f} {r.min_powered_accel_g:7.4f} "
              f"{peak:7.4f} {fuel:7.4f} "
              f"{str(r.motor_cutoff_reached):>5} {wall:7.0f}  {note}",
              flush=True)
        (OUT / f"{a.tag}_step_refinement.json").write_text(
            json.dumps(rows, indent=2))

    # --- is it bias or noise? -----------------------------------------
    print()
    base = rows[0]
    fin = rows[-1]
    for key, label in (("traverse_g", "traverse"), ("peak_mach", "peak M"),
                       ("fuel_kg", "fuel")):
        seq = [q[key] for q in rows]
        drift = fin[key] - base[key]
        monotone = (all(b >= a_ for a_, b in zip(seq, seq[1:]))
                    or all(b <= a_ for a_, b in zip(seq, seq[1:])))
        pct = 100.0 * drift / base[key] if base[key] else float("nan")
        print(f"{label:>9}: {' -> '.join(f'{v:.4f}' for v in seq)}   "
              f"net {drift:+.4f} ({pct:+.1f}%)  "
              f"{'MONOTONE -> bias' if monotone else 'non-monotone -> noise'}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for q in rows:
        tot = [p + r for p, r in zip(q["trace"]["pulsejet_thrust_n"],
                                     q["trace"]["ramjet_thrust_n"])]
        ax1.plot(q["trace"]["time_s"], tot, marker=".", ms=3, lw=1.2,
                 label=f"dM {q['mach_step']:.2f} "
                       f"({q['reconvergences']} solves)")
        ax2.plot(q["trace"]["time_s"], q["trace"]["mach"], lw=1.2)
    ax1.set_ylabel("total thrust (N)")
    ax1.legend(fontsize=8, loc="upper left")
    ax1.set_title(f"{a.tag}: propulsion step refinement -- climb {a.climb:g}"
                  f" / dive {a.dive:g} / lightoff M {a.lightoff:g}, "
                  f"n_cells {a.n_cells}\ndoes the staircase change the "
                  f"answer, or only its appearance?", fontsize=10)
    ax2.set_ylabel("Mach at each solve")
    ax2.set_xlabel("time (s)")
    fig.tight_layout()
    p = OUT / f"{a.tag}_step_refinement.png"
    fig.savefig(p, dpi=130); plt.close(fig)
    print(f"\nwrote {p}\n      {OUT / (a.tag + '_step_refinement.json')}")


if __name__ == "__main__":
    main()
