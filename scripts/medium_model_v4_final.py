"""Fly the FROZEN V4 design through medium_model once and record it properly.

This is a re-fly, not a search: docs/v4_frozen/design.json is the input and
nothing in here optimizes anything (user directive, 2026-08-13). The point is
the fidelity ladder, one rung per invocation:

  --rung a   legacy drag + closed-form engines   PORT CHECK. Must reproduce
             simple_model's frozen V4 numbers. A disagreement here means the
             V4 port into medium_model is wrong, not that anything was learnt.
  --rung b   build-up drag + closed-form engines  cost of real drag, alone.
  --rung c   build-up drag + FP engines           the actual Gate 3 answer.

Rung A deliberately flies simple_model's OWN fuel cap (tank volume / 1.25) so
the comparison isolates the port. B and C fly medium_model's stricter rule
(90% of the design's loaded allocation, mission.py) -- that divergence is
medium_model's whole point, and whether it binds is reported either way.

Usage:
  python scripts/medium_model_v4_final.py --rung a
  python scripts/medium_model_v4_final.py --rung c --n-cells 324
"""
from __future__ import annotations
import argparse, gzip, json, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import matplotlib                                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402

from dataclasses import replace                              # noqa: E402

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402
from medium_model.mission import (FUEL_RESERVE_MARGIN,       # noqa: E402
                                  FUEL_VOLUME_FRACTION_OF_ANNULUS,
                                  annular_volume_m3)

OUT = Path("out_medium_model")
G = 9.80665
DESIGN_PATH = "docs/v4_frozen/design.json"

# V4 adds two phases to the V3 order.
PHASE_ORDER = ["v3_climb", "v4_pushover", "v3_dive", "v4_pullout",
               "drag_strip", "coast", "loop", "return", "glide", "decel",
               "spiral", "flare"]
PHASE_COLORS = {"v3_climb": "#eef4ff", "v4_pushover": "#f3e8ff",
                "v3_dive": "#fff3e0", "v4_pullout": "#ffe0cc",
                "drag_strip": "#ffe9e9", "coast": "#f2f2f2",
                "loop": "#f0e9ff", "return": "#eef9ee", "glide": "#eaf7ea",
                "decel": "#f7f7e8", "spiral": "#e6f6fb", "flare": "#fdf0f6"}

RUNGS = {
    "a": dict(drag="legacy", closed_form=True,
              what="legacy drag + closed-form engines (port check)"),
    "b": dict(drag="buildup", closed_form=True,
              what="build-up drag + closed-form engines"),
    "c": dict(drag="buildup", closed_form=False,
              what="build-up drag + first-principles engines"),
}


def parse():
    p = argparse.ArgumentParser()
    p.add_argument("--rung", choices=sorted(RUNGS), default="a")
    p.add_argument("--design", default=DESIGN_PATH)
    p.add_argument("--tag", default=None)
    p.add_argument("--n-cells", type=int, default=324)
    # V4 premise: the chamber IS the body (docs/v4_frozen/design.json). The
    # historical medium_model default is 0.95, kept so V3b/V3c/V3d FP runs
    # stay reproducible; 1.0 is the V4 airframe (user decision, 2026-08-13).
    p.add_argument("--chamber-fraction", type=float, default=1.0)
    # Last-chance lightoff (user, 2026-08-13): if the ramjet has not made the
    # gate by the time the pull-out starts, light it there anyway. Off by
    # default so the plain frozen V4 stays the plain frozen V4.
    p.add_argument("--light-at-pullout", action="store_true")
    p.add_argument("--mach-step", type=float, default=None)
    p.add_argument("--alt-step-m", type=float, default=None)
    p.add_argument("--dt", type=float, default=0.02)
    return p.parse_args()


def main():
    a = parse()
    cfg = RUNGS[a.rung]
    tag = a.tag or (f"v4_rung{a.rung}"
                    + ("_lap" if a.light_at_pullout else ""))
    OUT.mkdir(exist_ok=True)
    d = load_frozen_design(a.design)
    if a.light_at_pullout:
        # An OVERRIDE on top of the freeze, not an edit of it: design.json is
        # untouched and the flag is recorded in the output JSON.
        d = replace(d, ramjet_start=replace(d.ramjet_start,
                                            light_at_pullout=True))
    cd = d.climb_dive
    rs = d.ramjet_start

    # Rung A re-flies simple_model's fuel rule so the port check is a port
    # check; B and C fly medium_model's own (see module docstring).
    tank_capacity_kg = (
        FUEL_VOLUME_FRACTION_OF_ANNULUS
        * annular_volume_m3(d.geometry.diameter_m, d.geometry.throat_diameter_m,
                            d.geometry.throat_length_m)
        * d.geometry.fuel.density_kg_per_m3)
    simple_cap_kg = tank_capacity_kg / (1.0 + FUEL_RESERVE_MARGIN)
    burn_cap_kg = simple_cap_kg if a.rung == "a" else d.burn_limit_kg

    fp = None
    if not cfg["closed_form"]:
        fp_kw = {}
        if a.mach_step is not None:
            fp_kw["mach_step"] = a.mach_step
        if a.alt_step_m is not None:
            fp_kw["altitude_step_m"] = a.alt_step_m
        fp = FpPropulsion(
            spec_from_geometry(d.geometry,
                               chamber_diameter_fraction=a.chamber_fraction),
            fuel="propane", lightoff_mach=rs.gate_mach,
            n_cells=a.n_cells, **fp_kw)

    fid = ""
    if fp is not None:
        fid = (f", n_cells {a.n_cells}, dM {fp.mach_step:g}, "
               f"d_alt {fp.altitude_step_m:.0f} m, "
               f"chamber/body {a.chamber_fraction:g}")
    label = (f"{tag}: V4 frozen -- top {cd.top_altitude_m:g} m, climb "
             f"{cd.initial_climb_angle_deg:g} deg"
             f"{' spiral' if cd.spiral_climb else ''}, dive "
             f"{cd.dive_angle_deg:g} deg, pull-out {cd.pullout_load_factor:g} g "
             f"(max {cd.pullout_max_load_factor:g}), floor "
             f"{cd.floor_altitude_m:.1f} m, ramjet gate M {rs.gate_mach:g}"
             f"{' descending-only' if rs.require_descending else ''}"
             f"{' + LIGHT AT PULL-OUT regardless of Mach' if rs.light_at_pullout else ''}"
             f"; {cfg['what']}{fid}, dt {a.dt:g} s")
    print(label, flush=True)

    r = fly(d, drag_model=cfg["drag"], propulsion=fp, dt_s=a.dt,
            max_fuel_burn_kg=burn_cap_kg, max_time_s=600.0)
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
            gamma0_deg=st[lo].flight_path_angle_rad * 180.0 / 3.141592653589793,
            gamma1_deg=st[hi].flight_path_angle_rad * 180.0 / 3.141592653589793,
            peak_load_n=max(x.load_n_total for x in span),
            mean_thrust_n=sum(x.thrust_n for x in span) / len(span),
            mean_drag_n=sum(x.drag_n for x in span) / len(span),
            min_accel_g=min(x.acceleration_m_per_s2 for x in span) / G))
    phases.sort(key=lambda q: q["t0"])

    summary = dict(
        label=label, tag=tag, rung=a.rung, design=a.design,
        drag_model=cfg["drag"],
        propulsion="closed-form" if fp is None else "FP",
        n_cells=None if fp is None else a.n_cells,
        chamber_diameter_fraction=None if fp is None else a.chamber_fraction,
        mach_step=None if fp is None else fp.mach_step,
        altitude_step_m=None if fp is None else fp.altitude_step_m,
        dt_s=a.dt,
        # --- trajectory as commanded ---
        top_altitude_m=cd.top_altitude_m,
        climb_angle_deg=cd.initial_climb_angle_deg,
        spiral_climb=cd.spiral_climb, spiral_bank_deg=cd.spiral_bank_deg,
        dive_angle_deg=cd.dive_angle_deg, floor_altitude_m=cd.floor_altitude_m,
        pullout_load_factor=cd.pullout_load_factor,
        pullout_max_load_factor=cd.pullout_max_load_factor,
        gate_mach=rs.gate_mach, require_descending=rs.require_descending,
        light_at_pullout=bool(rs.light_at_pullout),
        # --- what it flew ---
        peak_mach=peak, cutoff=bool(r.motor_cutoff_reached),
        ramjet_lightoff_mach=r.ramjet_lightoff_mach,
        ramjet_lightoff_altitude_m=r.ramjet_lightoff_altitude_m,
        ramjet_lightoff_time_s=r.ramjet_lightoff_time_s,
        ramjet_lightoff_mode=r.ramjet_lightoff_mode,
        ramjet_lit_in_dive=bool(r.ramjet_lit_in_dive),
        ramjet_light_refused=bool(r.ramjet_light_refused),
        dive_exit_mach=r.dive_exit_mach,
        traverse_g=r.min_traverse_accel_g, powered_g=r.min_powered_accel_g,
        pushover_g=(None if r.min_pushover_accel_g == float("inf")
                    else r.min_pushover_accel_g),
        margin=r.min_powered_thrust_margin,
        peak_load_n_total=r.peak_load_n_total,
        peak_load_n_yaw=r.peak_load_n_yaw,
        peak_load_n_roll=r.peak_load_n_roll,
        peak_load_mode=r.peak_load_mode,
        pushover_radius_m=r.pushover_radius_m,
        pullout_radius_m=r.pullout_radius_m,
        spiral_radius_m=r.spiral_radius_m,
        pushover_duration_s=r.pushover_duration_s,
        pullout_duration_s=r.pullout_duration_s,
        min_powered_altitude_m=(None if r.min_powered_altitude_m == float("inf")
                                else r.min_powered_altitude_m),
        floor_violated=bool(r.floor_violated),
        rule_violated=bool(r.rule_violated),
        fuel_kg=fuel, burn_cap_kg=burn_cap_kg,
        medium_burn_limit_kg=d.burn_limit_kg,
        simple_burn_cap_kg=simple_cap_kg,
        loaded_fuel_kg=d.loaded_fuel_kg,
        tank_dry=fuel >= burn_cap_kg - 1e-6,
        peak_tw=max(s.thrust_to_weight for s in st),
        flight_s=st[-1].time_s, stalled=bool(r.stalled),
        safe_landing=bool(r.safe_landing),
        lands_from_launch_m=abs(st[-1].distance_m),
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
    (OUT / f"{tag}_final.json").write_text(json.dumps(summary, indent=2))

    # Full per-step trace, gzipped as parallel arrays. An FP flight costs
    # ~35 min of wall clock, so anything that would otherwise force a re-fly
    # to answer a later question (plots, say) gets recorded now. Dict of
    # arrays rather than array of dicts: ~6x smaller and it loads straight
    # into the plotters.
    fields = ["time_s", "altitude_m", "distance_m", "velocity_m_per_s",
              "mach", "mass_kg", "fuel_burned_kg", "mode", "thrust_n",
              "drag_n", "acceleration_m_per_s2", "thrust_to_weight",
              "specific_impulse_s", "stall_speed_m_per_s",
              "flight_path_angle_rad", "load_n_roll", "load_n_yaw",
              "load_n_total", "turn_radius_m"]
    states_path = OUT / f"{tag}_states.json.gz"
    with gzip.open(states_path, "wt", encoding="utf-8") as fh:
        json.dump({"tag": tag, "fields": fields,
                   "result": {k: summary[k] for k in (
                       "peak_load_n_total", "peak_load_n_yaw",
                       "peak_load_n_roll", "peak_load_mode",
                       "pushover_radius_m", "pullout_radius_m",
                       "spiral_radius_m", "ramjet_lightoff_mach",
                       "ramjet_lightoff_altitude_m", "ramjet_lightoff_time_s",
                       "ramjet_lightoff_mode", "safe_landing", "stalled",
                       "loaded_fuel_kg", "gate_mach", "top_altitude_m",
                       "climb_angle_deg", "dive_angle_deg",
                       "pullout_load_factor", "spiral_climb",
                       "light_at_pullout")},
                   "states": {f: [getattr(s, f) for s in st] for f in fields}},
                  fh)

    # ---- console ------------------------------------------------------
    def fmt(x, spec=".3f"):
        return "n/a" if x is None else format(x, spec)

    print(f"\npeak M {peak:.3f}  cutoff {r.motor_cutoff_reached}  "
          f"dive exit M {r.dive_exit_mach:.3f}")
    print(f"ramjet: lit {'yes' if r.ramjet_lightoff_mach else 'NO'}"
          + (f" at M {r.ramjet_lightoff_mach:.3f}, "
             f"{r.ramjet_lightoff_altitude_m:.0f} m, "
             f"mode {r.ramjet_lightoff_mode}  IN DIVE: "
             f"{r.ramjet_lit_in_dive}" if r.ramjet_lightoff_mach else "")
          + ("   [FP REFUSED the light inside the policy window]"
             if r.ramjet_light_refused else ""))
    print(f"traverse {r.min_traverse_accel_g:.3f} g  "
          f"powered {r.min_powered_accel_g:.3f} g  "
          f"pushover {fmt(summary['pushover_g'])} g  "
          f"margin {r.min_powered_thrust_margin:.3f}")
    print(f"loads: total {r.peak_load_n_total:.3f} g in {r.peak_load_mode}  "
          f"(yaw {r.peak_load_n_yaw:.3f}, roll {r.peak_load_n_roll:.3f})")
    print(f"floor: flown min {fmt(summary['min_powered_altitude_m'], '.2f')} m "
          f"vs {cd.floor_altitude_m:.2f} m  violated {r.floor_violated}")
    print(f"radii: spiral {r.spiral_radius_m:.0f} m  pushover "
          f"{r.pushover_radius_m:.0f} m ({r.pushover_duration_s:.1f} s)  "
          f"pull-out {r.pullout_radius_m:.0f} m ({r.pullout_duration_s:.1f} s)")
    print(f"fuel {fuel:.3f} / {burn_cap_kg:.3f} kg cap "
          f"({100*fuel/burn_cap_kg:.0f}%)  tank dry {summary['tank_dry']}  "
          f"FP runs {summary['fp_runs']}")
    print(f"landing: safe {r.safe_landing}  stalled {r.stalled}  "
          f"{abs(st[-1].distance_m):.1f} m from launch  "
          f"flight {st[-1].time_s:.0f} s")
    for ev in events:
        print(f"  event {ev}")
    print(f"\n{'phase':<12} {'t0':>6} {'t1':>6} {'dt':>6} {'M0':>6} "
          f"{'M1':>6} {'alt0':>6} {'alt1':>6} {'g0':>6} {'g1':>6} "
          f"{'nmax':>5} {'fuel':>6} {'%':>5} {'T':>7} {'D':>7} {'min_g':>7}")
    for q in phases:
        print(f"{q['mode']:<12} {q['t0']:6.1f} {q['t1']:6.1f} {q['dt']:6.1f} "
              f"{q['mach0']:6.3f} {q['mach1']:6.3f} {q['alt0']:6.0f} "
              f"{q['alt1']:6.0f} {q['gamma0_deg']:6.1f} {q['gamma1_deg']:6.1f} "
              f"{q['peak_load_n']:5.2f} "
              f"{q['fuel_kg']:6.3f} {q['fuel_pct']:5.1f} "
              f"{q['mean_thrust_n']:7.1f} {q['mean_drag_n']:7.1f} "
              f"{q['min_accel_g']:7.3f}")

    # Rung A is a port check, so print the head-to-head straight away.
    if a.rung == "a":
        vm = d.verified
        checks = [
            ("peak_mach", peak, vm.get("peak_mach")),
            ("dive_exit_mach", r.dive_exit_mach, vm.get("dive_exit_mach")),
            ("ramjet_lightoff_mach", r.ramjet_lightoff_mach,
             vm.get("ramjet_lightoff_mach")),
            ("ramjet_lightoff_altitude_m", r.ramjet_lightoff_altitude_m,
             vm.get("ramjet_lightoff_altitude_m")),
            ("peak_thrust_to_weight", summary["peak_tw"],
             vm.get("peak_thrust_to_weight")),
            ("min_traverse_accel_g", r.min_traverse_accel_g,
             vm.get("min_traverse_accel_g")),
            ("min_powered_accel_g", r.min_powered_accel_g,
             vm.get("min_powered_accel_g")),
            ("min_powered_thrust_margin", r.min_powered_thrust_margin,
             vm.get("min_powered_thrust_margin")),
            ("peak_load_n_total", r.peak_load_n_total,
             vm.get("peak_load_n_total")),
            ("peak_load_n_roll", r.peak_load_n_roll, vm.get("peak_load_n_roll")),
            ("pushover_radius_m", r.pushover_radius_m,
             vm.get("pushover_radius_m")),
            ("pullout_radius_m", r.pullout_radius_m, vm.get("pullout_radius_m")),
            ("spiral_radius_m", r.spiral_radius_m, vm.get("spiral_radius_m")),
            ("min_powered_altitude_m", summary["min_powered_altitude_m"],
             vm.get("min_powered_altitude_m")),
            ("fuel_burned_kg", fuel, vm.get("fuel_burned_kg")),
            ("stall_speed_m_per_s", st[-1].stall_speed_m_per_s,
             vm.get("stall_speed_m_per_s")),
            ("lands_from_launch_m", abs(st[-1].distance_m),
             vm.get("lands_from_launch_m")),
        ]
        print(f"\nPORT CHECK vs simple_model's frozen V4 "
              f"({'%s' % a.design})")
        print(f"{'quantity':<28} {'medium':>12} {'simple':>12} {'delta':>12} "
              f"{'rel':>9}")
        worst = 0.0
        for name, got, want in checks:
            if want is None or got is None:
                print(f"{name:<28} {fmt(got, '12.4f') if got is not None else 'n/a':>12} "
                      f"{'n/a':>12} {'-':>12} {'-':>9}")
                continue
            delta = got - want
            rel = abs(delta) / abs(want) if want else abs(delta)
            worst = max(worst, rel)
            print(f"{name:<28} {got:12.4f} {want:12.4f} {delta:12.4f} "
                  f"{100*rel:8.3f}%")
        print(f"\nworst relative disagreement: {100*worst:.4f}%")
        print("A port check should be ~0%. Anything above ~0.1% is a porting "
              "difference, not a modelling finding.")

    # ---- plots -------------------------------------------------------
    t = [s.time_s for s in st]
    powered = [s for s in st if s.thrust_n > 1.0]
    t_cut = max((s.time_s for s in powered), default=None)
    t_lit = r.ramjet_lightoff_time_s

    def shade(ax):
        for q in phases:
            ax.axvspan(q["t0"], q["t1"],
                       color=PHASE_COLORS.get(q["mode"], "#fafafa"), zorder=0)
        if t_lit is not None:
            ax.axvline(t_lit, color="#c0392b", ls="--", lw=1.2, zorder=5)
        if t_cut is not None:
            ax.axvline(t_cut, color="#2c3e50", ls=":", lw=1.2, zorder=5)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    shade(ax1)
    ax1.plot(t, [s.mach for s in st], color="#1f4e79", lw=1.8, label="Mach")
    ax1.axhline(1.0, color="#888", lw=0.9, ls="-.")
    ax1.axhline(rs.gate_mach, color="#c0392b", lw=0.9, ls="--")
    ax1.set_ylabel("Mach", color="#1f4e79")
    ax1.tick_params(axis="y", labelcolor="#1f4e79")
    axa = ax1.twinx()
    axa.plot(t, [s.altitude_m for s in st], color="#7b3f00", lw=1.4)
    axa.axhline(cd.floor_altitude_m, color="#7b3f00", ls="--", lw=0.9)
    axa.set_ylabel("altitude (m)", color="#7b3f00")
    axa.tick_params(axis="y", labelcolor="#7b3f00")
    ax1.set_title(f"{label}\npeak M {peak:.3f}, dive exit M "
                  f"{r.dive_exit_mach:.3f}, traverse "
                  f"{r.min_traverse_accel_g:.3f} g, fuel {fuel:.3f} kg of "
                  f"{burn_cap_kg:.3f} kg cap", fontsize=9)
    if t_lit is not None:
        ax1.annotate(f"ramjet lights\nM {r.ramjet_lightoff_mach:.3f}\n"
                     f"{r.ramjet_lightoff_mode}",
                     xy=(t_lit, r.ramjet_lightoff_mach),
                     xytext=(t_lit, 0.90), fontsize=8, color="#c0392b",
                     ha="center",
                     arrowprops=dict(arrowstyle="->", color="#c0392b"))

    shade(ax2)
    ax2.plot(t, [s.thrust_n for s in st], color="#c0392b", lw=1.6,
             label="thrust")
    ax2.plot(t, [s.drag_n for s in st], color="#2c3e50", lw=1.4, label="drag")
    ax2.set_ylabel("force (N)")
    ax2.legend(loc="upper right", fontsize=8)
    axb = ax2.twinx()
    axb.plot(t, [s.acceleration_m_per_s2 / G for s in st], color="#27632a",
             lw=1.1, alpha=0.8)
    axb.axhline(0.25, color="#27632a", ls="--", lw=0.9)
    axb.set_ylabel("along-track accel (g)", color="#27632a")
    axb.tick_params(axis="y", labelcolor="#27632a")
    axb.set_ylim(-1.0, 3.0)

    # V4's own plot: what the airframe actually carries.
    shade(ax3)
    ax3.plot(t, [s.load_n_total for s in st], color="#6a1b9a", lw=1.7,
             label="total")
    ax3.plot(t, [s.load_n_yaw for s in st], color="#1565c0", lw=1.2,
             label="yaw (normal)")
    ax3.plot(t, [s.load_n_roll for s in st], color="#ef6c00", lw=1.2,
             label="roll (axial)")
    ax3.axhline(cd.pullout_load_factor, color="#6a1b9a", ls="--", lw=0.9)
    ax3.axhline(cd.pullout_max_load_factor, color="#c0392b", ls=":", lw=0.9)
    ax3.set_ylabel("body load (g, gravity excluded)")
    ax3.set_xlabel("time (s)")
    ax3.legend(loc="upper left", fontsize=8)
    ax3.set_ylim(-1.0, max(7.0, r.peak_load_n_total + 1.0))
    ax3.text(0.01, 0.02, "accelerometer convention; dashed = commanded "
             "pull-out g, dotted = limiter", transform=ax3.transAxes,
             fontsize=7.5, color="#555")
    fig.tight_layout()
    p1 = OUT / f"{tag}_trajectory.png"
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
    bx2.axhline(burn_cap_kg, color="#c0392b", ls="--", lw=1.2)
    bx2.text(t[-1], burn_cap_kg, f" cap {burn_cap_kg:.3f} kg", color="#c0392b",
             fontsize=8, ha="right", va="bottom")
    bx2.axhline(d.loaded_fuel_kg, color="#888", ls=":", lw=1.0)
    bx2.text(t[-1], d.loaded_fuel_kg, f" loaded {d.loaded_fuel_kg:.3f} kg",
             color="#666", fontsize=8, ha="right", va="bottom")
    if t_lit is not None:
        bx2.axvline(t_lit, color="#c0392b", ls="--", lw=1.0)
    bx2.set_xlabel("time (s)"); bx2.set_ylabel("cumulative fuel (kg)")
    bx2.set_title("cumulative burn vs the cap")
    fig.tight_layout()
    p2 = OUT / f"{tag}_fuel.png"
    fig.savefig(p2, dpi=130); plt.close(fig)

    p3 = None
    if tr and tr.time_s:
        # The engine trace is sampled at each FP re-convergence, not each
        # timestep -- that IS the propulsion model's own resolution, so
        # plotting it raw shows honestly how coarse the march is.
        fig, (cx1, cx2, cx3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        tt = tr.time_s
        cx1.plot(tt, tr.pulsejet_thrust_n, color="#1f4e79", lw=1.5,
                 marker=".", ms=3, label="pulsejet")
        cx1.plot(tt, tr.ramjet_thrust_n, color="#c0392b", lw=1.5,
                 marker=".", ms=3, label="ramjet")
        cx1.plot(tt, [p + q for p, q in zip(tr.pulsejet_thrust_n,
                                            tr.ramjet_thrust_n)],
                 color="#444", lw=1.0, ls="--", label="total")
        cx1.set_ylabel("thrust (N)")
        cx1.legend(fontsize=8, loc="upper left")
        cx1.set_title(f"{label}\nengine split at each of the {len(tt)} "
                      f"first-principles re-convergences", fontsize=9)
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
        p3 = OUT / f"{tag}_engines.png"
        fig.savefig(p3, dpi=130); plt.close(fig)

    print(f"\nwrote {p1}\n      {p2}" + (f"\n      {p3}" if p3 else "")
          + f"\n      {OUT / (tag + '_final.json')}")


if __name__ == "__main__":
    main()
