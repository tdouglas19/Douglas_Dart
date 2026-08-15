"""Fly the chosen V3b config once under FP, dump the trace, plot, and
write the per-phase fuel budget.

Separate from the candidate sweep because an FP flight costs minutes: the
sweep picks the config, this records it properly.  Everything the report
needs comes out of this one flight -- trajectory, force balance, fuel
budget by phase -- so the numbers in the report cannot drift from each
other the way they would if each table re-flew the vehicle.

Usage:
  python scripts/medium_model_v3b_final.py [--climb D] [--dive D]
         [--lightoff M] [--span M] [--ar A] [--tag NAME] [--closed-form]
"""
from __future__ import annotations
import argparse, json, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import matplotlib                                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.drag import WingConcept                    # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402

OUT = Path("out_medium_model")
G = 9.80665
# Phases in the order they occur, with the label used on the plots.
PHASE_ORDER = ["v3_climb", "v3_dive", "drag_strip", "coast", "loop",
               "glide", "spiral", "flare"]


def parse():
    p = argparse.ArgumentParser()
    p.add_argument("--climb", type=float, default=8.0)
    p.add_argument("--dive", type=float, default=14.0)
    p.add_argument("--floor", type=float, default=122.0)
    p.add_argument("--lightoff", type=float, default=0.35)
    # Default to the design's OWN wing. Passing rounded values (0.5325 /
    # 1.86 instead of 0.532526287202532 / 1.8595845502152375) moved
    # min_traverse_accel_g from 0.376 to 0.389 -- not because the wing
    # matters that much, but because the minimum sits on a discrete
    # timestep and a nudge flips which step wins. Never re-type the wing.
    p.add_argument("--span", type=float, default=None)
    p.add_argument("--ar", type=float, default=None)
    p.add_argument("--tag", default="v3b")
    p.add_argument("--design", default="docs/v3a_medium_model/design.json")
    p.add_argument("--closed-form", action="store_true")
    p.add_argument("--n-cells", type=int, default=162)
    # Propulsion re-convergence rate. The defaults (dM 0.05, d_alt 250 ft)
    # are the VALIDATED CEILING from the ramjet-fp continuation campaign,
    # not a preference -- so refining them is the safe direction, it just
    # costs FP solves. Refining is what removes the thrust staircase.
    p.add_argument("--mach-step", type=float, default=None)
    p.add_argument("--alt-step-m", type=float, default=None)
    p.add_argument("--dt", type=float, default=0.02)
    return p.parse_args()


def main():
    a = parse()
    OUT.mkdir(exist_ok=True)
    d = load_frozen_design(a.design)
    w = d.wing
    if a.span is None:
        a.span = w.span_m
    if a.ar is None:
        a.ar = w.aspect_ratio
    wing = WingConcept(span_m=a.span, aspect_ratio=a.ar,
                       taper_ratio=w.taper_ratio, sweep_deg=w.sweep_deg,
                       airfoil=w.airfoil)
    cd = ClimbDiveProfile(initial_climb_angle_deg=a.climb,
                          dive_angle_deg=a.dive, floor_altitude_m=a.floor)
    fp_kw = {}
    if a.mach_step is not None:
        fp_kw["mach_step"] = a.mach_step
    if a.alt_step_m is not None:
        fp_kw["altitude_step_m"] = a.alt_step_m
    fp = (None if a.closed_form else
          FpPropulsion(spec_from_geometry(d.geometry), fuel="propane",
                       lightoff_mach=a.lightoff, n_cells=a.n_cells,
                       **fp_kw))

    fid = ""
    if not a.closed_form:
        fid = (f", n_cells {a.n_cells}, dM {fp.mach_step:g}, "
               f"d_alt {fp.altitude_step_m:.0f} m, dt {a.dt:g} s")
    label = (f"{a.tag}: climb {a.climb:g} deg / dive {a.dive:g} deg / "
             f"floor {a.floor:g} m, lightoff M {a.lightoff:g}, "
             f"wing {a.span:g} m AR {a.ar:g}, "
             f"{'closed-form' if a.closed_form else 'FP'} propulsion{fid}")
    print(label, flush=True)

    r = fly(d, drag_model="buildup", climb_dive=cd, wing_concept=wing,
            propulsion=fp, dt_s=a.dt)
    st = r.states
    peak = max(s.mach for s in st)
    fuel = max(s.fuel_burned_kg for s in st)
    tr = fp.trace if fp else None
    events = [f"{t:.1f}s {w}" for t, w in tr.events] if tr else []

    # ---- per-phase budget -------------------------------------------
    phases, seen = [], {}
    for i, s in enumerate(st):
        if s.mode not in seen:
            seen[s.mode] = dict(mode=s.mode, t0=s.time_s, i0=i)
        seen[s.mode]["t1"] = s.time_s
        seen[s.mode]["i1"] = i
    for mode, q in seen.items():
        lo, hi = q["i0"], q["i1"]
        span = st[lo:hi + 1]
        burn = st[hi].fuel_burned_kg - st[lo].fuel_burned_kg
        phases.append(dict(
            mode=mode, t0=q["t0"], t1=q["t1"], dt=q["t1"] - q["t0"],
            fuel_kg=burn,
            fuel_pct=100.0 * burn / fuel if fuel else 0.0,
            mach0=st[lo].mach, mach1=st[hi].mach,
            alt0=st[lo].altitude_m, alt1=st[hi].altitude_m,
            mean_thrust_n=sum(x.thrust_n for x in span) / len(span),
            mean_drag_n=sum(x.drag_n for x in span) / len(span),
            min_accel_g=min(x.acceleration_m_per_s2 for x in span) / G))
    phases.sort(key=lambda q: q["t0"])

    summary = dict(
        label=label, tag=a.tag, climb=a.climb, dive=a.dive, floor=a.floor,
        lightoff=a.lightoff, span=a.span, aspect_ratio=a.ar,
        propulsion="closed-form" if a.closed_form else "FP",
        n_cells=None if a.closed_form else a.n_cells,
        mach_step=None if a.closed_form else fp.mach_step,
        altitude_step_m=None if a.closed_form else fp.altitude_step_m,
        dt_s=a.dt,
        peak_mach=peak, cutoff=bool(r.motor_cutoff_reached),
        traverse_g=r.min_traverse_accel_g, powered_g=r.min_powered_accel_g,
        margin=r.min_powered_thrust_margin, fuel_kg=fuel,
        burn_limit_kg=d.burn_limit_kg, loaded_fuel_kg=d.loaded_fuel_kg,
        tank_dry=fuel >= d.burn_limit_kg - 1e-6,
        peak_tw=max(s.thrust_to_weight for s in st),
        flight_s=st[-1].time_s, stalled=bool(r.stalled),
        safe_landing=bool(r.safe_landing),
        fp_runs=fp.n_transients if fp else 0, events=events,
        phases=phases,
        engine_trace=(dict(
            time_s=tr.time_s, mach=tr.mach, altitude_m=tr.altitude_m,
            pulsejet_thrust_n=tr.pulsejet_thrust_n,
            ramjet_thrust_n=tr.ramjet_thrust_n,
            pulsejet_fuel_kg_s=tr.pulsejet_fuel_kg_s,
            ramjet_fuel_kg_s=tr.ramjet_fuel_kg_s,
            ramjet_phi=tr.ramjet_phi, ramjet_lit=tr.ramjet_lit)
            if tr else None))
    (OUT / f"{a.tag}_final.json").write_text(json.dumps(summary, indent=2))

    print(f"\npeak M {peak:.3f}  cutoff {r.motor_cutoff_reached}  "
          f"traverse {r.min_traverse_accel_g:.3f} g  "
          f"powered {r.min_powered_accel_g:.3f} g  "
          f"margin {r.min_powered_thrust_margin:.3f}")
    print(f"fuel {fuel:.3f} / {d.burn_limit_kg:.3f} kg  "
          f"({100*fuel/d.burn_limit_kg:.0f}% of burn limit)   "
          f"FP runs {summary['fp_runs']}")
    for ev in events:
        print(f"  event {ev}")
    print(f"\n{'phase':<12} {'t0':>6} {'t1':>6} {'dt':>6} {'M0':>6} "
          f"{'M1':>6} {'alt0':>6} {'alt1':>6} {'fuel':>6} {'%':>5} "
          f"{'T':>7} {'D':>7} {'min_g':>7}")
    for q in phases:
        print(f"{q['mode']:<12} {q['t0']:6.1f} {q['t1']:6.1f} {q['dt']:6.1f} "
              f"{q['mach0']:6.3f} {q['mach1']:6.3f} {q['alt0']:6.0f} "
              f"{q['alt1']:6.0f} {q['fuel_kg']:6.3f} {q['fuel_pct']:5.1f} "
              f"{q['mean_thrust_n']:7.1f} {q['mean_drag_n']:7.1f} "
              f"{q['min_accel_g']:7.3f}")

    # ---- plots -------------------------------------------------------
    t = [s.time_s for s in st]
    powered = [s for s in st if s.thrust_n > 1.0]
    t_cut = max((s.time_s for s in powered), default=None)
    t_lit = None
    for ev in events:
        if "ramjet_lit" in ev:
            try:
                t_lit = float(ev.split("s ")[0])
            except ValueError:
                pass

    def shade(ax):
        colors = {"v3_climb": "#eef4ff", "v3_dive": "#fff3e0",
                  "drag_strip": "#ffe9e9", "coast": "#f2f2f2",
                  "loop": "#f0e9ff", "glide": "#eaf7ea",
                  "spiral": "#e6f6fb", "flare": "#fdf0f6"}
        for q in phases:
            ax.axvspan(q["t0"], q["t1"], color=colors.get(q["mode"],
                       "#fafafa"), zorder=0)
        if t_lit is not None:
            ax.axvline(t_lit, color="#c0392b", ls="--", lw=1.2, zorder=5)
        if t_cut is not None:
            ax.axvline(t_cut, color="#2c3e50", ls=":", lw=1.2, zorder=5)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    shade(ax1)
    ax1.plot(t, [s.mach for s in st], color="#1f4e79", lw=1.8, label="Mach")
    ax1.axhline(1.0, color="#888", lw=0.9, ls="-.")
    ax1.set_ylabel("Mach", color="#1f4e79")
    ax1.tick_params(axis="y", labelcolor="#1f4e79")
    axa = ax1.twinx()
    axa.plot(t, [s.altitude_m for s in st], color="#7b3f00", lw=1.4,
             label="altitude")
    axa.set_ylabel("altitude (m)", color="#7b3f00")
    axa.tick_params(axis="y", labelcolor="#7b3f00")
    ax1.set_title(f"{label}\npeak M {peak:.3f}, traverse "
                  f"{r.min_traverse_accel_g:.3f} g, fuel {fuel:.3f} kg "
                  f"of {d.burn_limit_kg:.3f} kg limit", fontsize=10)
    if t_lit is not None:
        ax1.annotate(f"ramjet lights\nM {a.lightoff:g}",
                     xy=(t_lit, 1.0), xytext=(t_lit, 0.55), fontsize=8,
                     color="#c0392b", ha="center",
                     arrowprops=dict(arrowstyle="->", color="#c0392b"))

    shade(ax2)
    ax2.plot(t, [s.thrust_n for s in st], color="#c0392b", lw=1.6,
             label="thrust")
    ax2.plot(t, [s.drag_n for s in st], color="#2c3e50", lw=1.4,
             label="drag")
    ax2.set_ylabel("force (N)")
    ax2.set_xlabel("time (s)")
    ax2.legend(loc="upper right", fontsize=8)
    axb = ax2.twinx()
    axb.plot(t, [s.acceleration_m_per_s2 / G for s in st], color="#27632a",
             lw=1.1, alpha=0.8)
    axb.axhline(0.26, color="#27632a", ls="--", lw=0.9)
    axb.set_ylabel("along-track accel (g)", color="#27632a")
    axb.tick_params(axis="y", labelcolor="#27632a")
    axb.set_ylim(-1.0, 3.0)
    ax2.text(0.01, 0.96, "shading = flight phase;  dashed red = ramjet "
             "light;  dotted = motor cutoff;  dashed green = 0.26 g gate",
             transform=ax2.transAxes, fontsize=7.5, va="top", color="#444")
    fig.tight_layout()
    p1 = OUT / f"{a.tag}_trajectory.png"
    fig.savefig(p1, dpi=130); plt.close(fig)

    fig, (bx1, bx2) = plt.subplots(1, 2, figsize=(12, 4.6))
    burn = [q for q in phases if q["fuel_kg"] > 1e-6]
    bx1.bar([q["mode"] for q in burn], [q["fuel_kg"] for q in burn],
            color="#7b3f00")
    bx1.set_ylabel("fuel burned (kg)")
    bx1.set_title("fuel by phase")
    bx1.tick_params(axis="x", rotation=30, labelsize=8)
    for i, q in enumerate(burn):
        bx1.text(i, q["fuel_kg"], f" {q['fuel_kg']:.3f}\n {q['fuel_pct']:.0f}%",
                 ha="center", va="bottom", fontsize=7.5)
    bx2.plot(t, [s.fuel_burned_kg for s in st], color="#7b3f00", lw=1.8)
    bx2.axhline(d.burn_limit_kg, color="#c0392b", ls="--", lw=1.2)
    bx2.text(t[-1], d.burn_limit_kg, f" burn limit {d.burn_limit_kg:.3f} kg",
             color="#c0392b", fontsize=8, ha="right", va="bottom")
    bx2.axhline(d.loaded_fuel_kg, color="#888", ls=":", lw=1.0)
    bx2.text(t[-1], d.loaded_fuel_kg, f" loaded {d.loaded_fuel_kg:.3f} kg",
             color="#666", fontsize=8, ha="right", va="bottom")
    if t_lit is not None:
        bx2.axvline(t_lit, color="#c0392b", ls="--", lw=1.0)
    bx2.set_xlabel("time (s)"); bx2.set_ylabel("cumulative fuel (kg)")
    bx2.set_title("cumulative burn vs the 90%-of-loaded cap")
    fig.tight_layout()
    p2 = OUT / f"{a.tag}_fuel.png"
    fig.savefig(p2, dpi=130); plt.close(fig)

    p3 = None
    if tr and tr.time_s:
        # The engine trace is sampled at each FP re-convergence, not each
        # timestep -- that IS the propulsion model's own resolution, so
        # plotting it raw shows honestly how coarse the march is.
        fig, (cx1, cx2, cx3) = plt.subplots(3, 1, figsize=(12, 10),
                                            sharex=True)
        tt = tr.time_s
        cx1.plot(tt, tr.pulsejet_thrust_n, color="#1f4e79", lw=1.5,
                 marker=".", ms=3, label="pulsejet")
        cx1.plot(tt, tr.ramjet_thrust_n, color="#c0392b", lw=1.5,
                 marker=".", ms=3, label="ramjet")
        cx1.plot(tt, [p + r for p, r in zip(tr.pulsejet_thrust_n,
                                            tr.ramjet_thrust_n)],
                 color="#444", lw=1.0, ls="--", label="total")
        cx1.set_ylabel("thrust (N)")
        cx1.legend(fontsize=8, loc="upper left")
        cx1.set_title(f"{label}\nengine split at each of the "
                      f"{len(tt)} first-principles re-convergences",
                      fontsize=10)
        cx2.plot(tt, tr.pulsejet_fuel_kg_s, color="#1f4e79", lw=1.5,
                 marker=".", ms=3, label="pulsejet")
        cx2.plot(tt, tr.ramjet_fuel_kg_s, color="#c0392b", lw=1.5,
                 marker=".", ms=3, label="ramjet")
        cx2.set_ylabel("fuel rate (kg/s)")
        cx2.legend(fontsize=8, loc="upper left")
        phi = [p if p else float("nan") for p in tr.ramjet_phi]
        cx3.plot(tt, phi, color="#7b3f00", lw=1.5, marker=".", ms=3)
        cx3.set_ylabel("ramjet phi (OUTPUT)")
        cx3.set_xlabel("time (s)")
        cx3.text(0.01, 0.05, "phi is self-selected per (M, alt) as "
                 "leanest-stable-plus-margin -- it is not commanded",
                 transform=cx3.transAxes, fontsize=8, color="#555")
        for cx in (cx1, cx2, cx3):
            if t_lit is not None:
                cx.axvline(t_lit, color="#c0392b", ls="--", lw=1.1)
            if t_cut is not None:
                cx.axvline(t_cut, color="#2c3e50", ls=":", lw=1.1)
        fig.tight_layout()
        p3 = OUT / f"{a.tag}_engines.png"
        fig.savefig(p3, dpi=130); plt.close(fig)

    print(f"\nwrote {p1}\n      {p2}" + (f"\n      {p3}" if p3 else "")
          + f"\n      {OUT / (a.tag + '_final.json')}")


if __name__ == "__main__":
    main()
