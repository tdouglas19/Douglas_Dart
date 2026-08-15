"""medium_model V4 plots in the SAME house style as the simple_model ones.

Same three figures `scripts/simple_model_v4_plots.py` produces, driven by a
medium_model flight instead of a simple_model one:

  flight_profile_<tag>.png   7-panel profile   -- run_demo.plot_flight_profile
  body_loading_<tag>.png     3-panel companion -- same layout as V4's
  propulsion_<tag>.png       3-panel thrust / Isp / SFC vs Mach

The first two reuse `simple_model.run_demo`'s own plotters rather than a
second, divergent plotting stack -- medium_model's `FlightState` is a
field-for-field copy of simple_model's, so the plotters read it directly.
`run_demo` builds from module-level GEOMETRY/CLIMB_ANGLE_DEG/OUT_DIR, so
those are pointed at this design for the duration and restored afterwards.

The propulsion figure is the one that CANNOT simply be reused. `run_demo`'s
version sweeps the closed-form engines over Mach at a fixed altitude; a
first-principles rung has no such curve -- it has a march, sampled at the
re-convergences, along the real flight path. So the layout is reproduced
(same three panels, same colours, same units) and the FP march is drawn on
it as points, with the closed-form sweep behind it as the reference it is
being measured against. Both are labelled for what they are.

Input is the gzipped state trace written by scripts/medium_model_v4_final.py
-- an FP flight costs ~35 min, so these plots never re-fly one.

Usage:
  python scripts/medium_model_v4_plots.py [--tag v4_rungc_lap]
"""
from __future__ import annotations

import argparse
import gzip
import json
import os
import sys
from math import degrees, nan
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

# Both packages bake CD0 at import; the V4 rungs are all flown at 0.1.
os.environ.setdefault("SIMPLE_MODEL_CD0_FRONTAL", "0.1")
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from simple_model import run_demo  # noqa: E402

OUT = ROOT / "out_medium_model"
KG_PER_LB = 0.45359237
_C_MAIN = run_demo._C_MAIN
_C_COMPANION = run_demo._C_COMPANION
_C_REF = run_demo._C_REF


class _State:
    """Duck-typed stand-in for a FlightState -- the plotters only ever read
    attributes off it, and rebuilding the real NamedTuple would drag
    medium_model's import (and its CD0 binding) into a plotting script."""
    __slots__ = ("time_s", "altitude_m", "distance_m", "velocity_m_per_s",
                 "mach", "mass_kg", "fuel_burned_kg", "mode", "thrust_n",
                 "drag_n", "acceleration_m_per_s2", "thrust_to_weight",
                 "specific_impulse_s", "stall_speed_m_per_s",
                 "flight_path_angle_rad", "load_n_roll", "load_n_yaw",
                 "load_n_total", "turn_radius_m")

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _Result:
    """Ditto for FlightResult: the recorded summary fields, dot-accessible."""

    def __init__(self, d):
        self.__dict__.update(d)
        self.landed = bool(d.get("safe_landing"))


def load_trace(tag: str):
    path = OUT / f"{tag}_states.json.gz"
    if not path.exists():
        raise SystemExit(
            f"{path} not found -- re-run scripts/medium_model_v4_final.py "
            f"(it writes the state trace alongside the summary).")
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        blob = json.load(fh)
    cols = blob["states"]
    n = len(cols["time_s"])
    states = [_State(**{f: cols[f][i] for f in blob["fields"]})
              for i in range(n)]
    result = _Result(blob["result"])
    result.states = states
    summary = json.loads((OUT / f"{tag}_final.json").read_text())
    return result, summary


def plot_body_loading(result, summary, tag: str) -> Path:
    """Body loading + pitch arcs, styled to match the flight profile figure:
    same mode background shading, same shared clock, same colour constants."""
    states = result.states
    time_s = [s.time_s for s in states]
    # Frame on the POWERED mission. The unpowered return half-loop pulls a
    # fixed RETURN_LOOP_LOAD_FACTOR (6 g) by construction -- it is not a
    # searched quantity, it is ~2x the pull-out this figure exists to show,
    # and leaving it in scale squashes the arcs into invisibility.
    powered = [s for s in states if s.thrust_n > 0.0]
    t_powered_end = powered[-1].time_s if powered else time_s[-1]

    fig, (ax_load, ax_radius, ax_gamma) = plt.subplots(
        3, 1, figsize=(12.5, 10.0), sharex=True,
        # top is tighter than simple_model's 0.92: the suptitle here is three
        # lines (a V4 trajectory has more commanded parameters to name than
        # fit on one), and the top panel carries its own title underneath it.
        gridspec_kw=dict(hspace=0.18, left=0.08, right=0.93, top=0.875,
                         bottom=0.07))
    for ax in (ax_load, ax_radius, ax_gamma):
        run_demo._shade_modes(ax, states)
        ax.grid(alpha=0.25)
        ax.set_xlim(0.0, t_powered_end)

    ax_load.plot(time_s, [s.load_n_total for s in states], color="black",
                 linewidth=2.0, label="total |n|")
    ax_load.plot(time_s, [s.load_n_yaw for s in states], color=_C_COMPANION,
                 linewidth=1.5, label="yaw axis (normal, in-plane)")
    ax_load.plot(time_s, [s.load_n_roll for s in states], color=_C_MAIN,
                 linewidth=1.5, label="roll axis (axial)")
    ax_load.axhline(0.0, color="black", linewidth=0.8)
    ax_load.axhline(result.peak_load_n_total, color=_C_REF, linestyle=":",
                    linewidth=1.0)
    ax_load.text(time_s[-1], result.peak_load_n_total,
                 f" peak {result.peak_load_n_total:.2f} g "
                 f"({result.peak_load_mode}) ",
                 color=_C_REF, fontsize=8, va="bottom", ha="right")
    ax_load.set_ylabel("Body load factor (g)")
    ax_load.legend(loc="upper left", fontsize=8)
    ax_load.set_title(
        "Body loading -- accelerometer convention (gravity EXCLUDED), so the "
        "roll trace is (T-D)/W, not the along-path acceleration.  "
        "Powered mission only: the unpowered return loop pulls a fixed 6 g.",
        fontsize=9)

    radius_ft = [s.turn_radius_m / 0.3048 if s.turn_radius_m > 0.0 else nan
                 for s in states]
    ax_radius.plot(time_s, radius_ft, color=_C_MAIN, linewidth=1.8)
    ax_radius.set_ylabel("Pitch-arc radius (ft)")
    for label, radius_m, colour in (
            ("pushover (pitch-down)", result.pushover_radius_m, "tab:orange"),
            ("pull-out (pitch-up)", result.pullout_radius_m, "tab:green")):
        if radius_m > 0.0:
            ax_radius.axhline(radius_m / 0.3048, color=colour, linestyle="--",
                              linewidth=1.0)
            ax_radius.text(time_s[0], radius_m / 0.3048,
                           f" {label}: R = {radius_m:,.0f} m "
                           f"({radius_m / 0.3048:,.0f} ft)",
                           color=colour, fontsize=8, va="bottom", ha="left")
    ax_radius.set_title(
        "Turn radius (blank on straight legs; the spiral-climb value is the "
        "GROUND-track circle, the arcs are in the vertical plane)",
        fontsize=10)

    ax_gamma.plot(time_s, [degrees(s.flight_path_angle_rad) for s in states],
                  color=_C_MAIN, linewidth=1.8)
    ax_gamma.axhline(0.0, color="black", linewidth=0.8)
    ax_gamma.set_ylabel("Flight path angle (deg)")
    ax_gamma.set_xlabel("Time (s)")
    if result.ramjet_lightoff_time_s is not None:
        for ax in (ax_load, ax_radius, ax_gamma):
            ax.axvline(result.ramjet_lightoff_time_s, color="crimson",
                       linewidth=1.2, alpha=0.7)
        # Anchor the label on whichever side keeps it inside the axes.
        late = result.ramjet_lightoff_time_s > 0.6 * t_powered_end
        ax_gamma.text(result.ramjet_lightoff_time_s, 0.0,
                      f"{'  ' if not late else ''}ramjet light: "
                      f"M {result.ramjet_lightoff_mach:.3f}, "
                      f"{result.ramjet_lightoff_altitude_m:.0f} m, "
                      f"in {result.ramjet_lightoff_mode}{'  ' if late else ''}",
                      color="crimson", fontsize=8, va="bottom",
                      ha="right" if late else "left")

    # Autoscale sees the whole flight, so the y-ranges have to be pinned to
    # the powered data too -- otherwise the return loop's 6 g and its 0->180
    # deg sweep set the scale for panels the loop has been cropped out of.
    def _span(values, pad_frac=0.10, floor_pad=0.5):
        lo, hi = min(values), max(values)
        pad = max((hi - lo) * pad_frac, floor_pad)
        return lo - pad, hi + pad

    ax_load.set_ylim(*_span([v for s in powered for v in (
        s.load_n_total, s.load_n_yaw, s.load_n_roll)]))
    arc_radii_ft = [s.turn_radius_m / 0.3048 for s in powered
                    if s.turn_radius_m > 0.0]
    if arc_radii_ft:
        ax_radius.set_ylim(*_span(arc_radii_ft, floor_pad=200.0))
    ax_gamma.set_ylim(*_span(
        [degrees(s.flight_path_angle_rad) for s in powered], floor_pad=2.0))

    # Three lines, not one: run_demo's single-line suptitle is already at the
    # width limit for a V2 design, and a V4 trajectory has twice as many
    # commanded parameters to name.
    fig.suptitle(
        f"medium_model V4 body loading and pitch arcs\n"
        f"{_headline(summary)}\n"
        f"(background shading = flight mode, as in the flight profile)",
        fontsize=10)
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"body_loading_{tag}.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_propulsion(result, summary, tag: str) -> Path:
    """run_demo's 3-panel propulsion layout, but sourced from the FP march.

    run_demo sweeps the closed-form engines over Mach at a fixed altitude.
    A first-principles rung has no such curve: it has a sequence of converged
    operating points along the real flight path, at the real altitude, each
    one a separate transient. Drawing those as a smooth line would claim a
    continuity the model does not have, so they are drawn as points on the
    same axes, with the closed-form sweep behind them as the reference.
    """
    from medium_model.design import load_frozen_design
    from medium_model.pulsejet_simple import pulsejet_thrust
    from medium_model.ramjet_simple import ramjet_thrust

    d = load_frozen_design(ROOT / "docs/v4_frozen/design.json")
    g = d.geometry
    tr = summary.get("engine_trace")
    ref_alt_m = run_demo.REFERENCE_ALTITUDE_M

    mach_values = [i * 0.02 for i in range(0, 76)]  # 0.00 .. 1.50
    pj = [pulsejet_thrust(g.diameter_m, g.chamber_length_m,
                          g.throat_diameter_m, g.throat_length_m, m,
                          ref_alt_m, g.fuel) for m in mach_values]
    rj = [ramjet_thrust(g.diameter_m, g.throat_diameter_m, m, ref_alt_m,
                        g.fuel, lightoff_mach=summary.get("gate_mach"))
          for m in mach_values]

    fig, (ax_thrust, ax_isp, ax_sfc) = plt.subplots(3, 1, figsize=(9, 12),
                                                    sharex=True)
    ax_thrust.plot(mach_values, [r.average_thrust_n for r in pj],
                   label="pulsejet (avg, closed-form, sea level)",
                   color="tab:red", alpha=0.45, linewidth=1.4)
    ax_thrust.plot(mach_values, [r.net_thrust_n for r in rj],
                   label="ramjet (net, closed-form, sea level)",
                   color="tab:green", alpha=0.45, linewidth=1.4)
    ax_isp.plot(mach_values,
                [r.specific_impulse_s if r.average_thrust_n > 0.0 else nan
                 for r in pj], color="tab:red", alpha=0.45, linewidth=1.4,
                label="pulsejet Isp (closed-form)")
    ax_isp.plot(mach_values,
                [r.specific_impulse_s if r.net_thrust_n > 0.0 else nan
                 for r in rj], color="tab:green", alpha=0.45, linewidth=1.4,
                label="ramjet Isp (closed-form)")
    ax_sfc.plot(mach_values,
                [run_demo._sfc_lb_per_lbf_hr(r.fuel_mass_flow_kg_per_s,
                                             r.average_thrust_n) for r in pj],
                color="tab:red", alpha=0.45, linewidth=1.4,
                label="pulsejet SFC (closed-form)")
    ax_sfc.plot(mach_values,
                [run_demo._sfc_lb_per_lbf_hr(r.fuel_mass_flow_kg_per_s,
                                             r.net_thrust_n) for r in rj],
                color="tab:green", alpha=0.45, linewidth=1.4,
                label="ramjet SFC (closed-form)")

    if tr and tr.get("time_s"):
        m_fp = tr["mach"]
        pj_n, rj_n = tr["pulsejet_thrust_n"], tr["ramjet_thrust_n"]
        pj_f, rj_f = tr["pulsejet_fuel_kg_s"], tr["ramjet_fuel_kg_s"]
        quench = [(m, 0.0) for m, n in zip(m_fp, pj_n) if n <= 0.0]
        ax_thrust.plot(m_fp, pj_n, "o", color="tab:red", markersize=4.5,
                       label="pulsejet (FP, along the flown path)")
        lit_m = [m for m, n in zip(m_fp, rj_n) if n > 0.0]
        lit_n = [n for n in rj_n if n > 0.0]
        if lit_m:
            ax_thrust.plot(lit_m, lit_n, "o", color="tab:green",
                           markersize=4.5,
                           label="ramjet (FP, along the flown path)")
        if quench:
            ax_thrust.plot([m for m, _ in quench], [0.0] * len(quench), "x",
                           color="black", markersize=9, markeredgewidth=2,
                           label="pulsejet QUENCHED (FP)")

        def _isp(n, f):
            return n / (f * 9.80665) if (n > 0.0 and f > 0.0) else nan
        ax_isp.plot(m_fp, [_isp(n, f) for n, f in zip(pj_n, pj_f)], "o",
                    color="tab:red", markersize=4.5, label="pulsejet Isp (FP)")
        if lit_m:
            ax_isp.plot([m for m, n in zip(m_fp, rj_n) if n > 0.0],
                        [_isp(n, f) for n, f in zip(rj_n, rj_f) if n > 0.0],
                        "o", color="tab:green", markersize=4.5,
                        label="ramjet Isp (FP)")
        ax_sfc.plot(m_fp, [run_demo._sfc_lb_per_lbf_hr(f, n)
                           for n, f in zip(pj_n, pj_f)], "o", color="tab:red",
                    markersize=4.5, label="pulsejet SFC (FP)")
        if lit_m:
            ax_sfc.plot([m for m, n in zip(m_fp, rj_n) if n > 0.0],
                        [run_demo._sfc_lb_per_lbf_hr(f, n)
                         for n, f in zip(rj_n, rj_f) if n > 0.0], "o",
                        color="tab:green", markersize=4.5,
                        label="ramjet SFC (FP)")

    ax_thrust.axhline(0.0, color="black", linewidth=0.8)
    ax_thrust.set_ylabel("Thrust (N)")
    ax_isp.set_ylabel("Specific impulse (s)")
    ax_sfc.set_ylabel("SFC (lb fuel / lbf·hr)")
    ax_sfc.set_xlabel("Mach")
    for ax in (ax_thrust, ax_isp, ax_sfc):
        ax.legend(loc="best", fontsize=8)
        ax.grid(alpha=0.3)
    has_fp = bool(tr and tr.get("time_s"))
    caption = ("Lines = closed-form at sea level (the reference).  Points = "
               "first-principles, one per re-convergence, at the REAL\n"
               "altitude flown -- so the FP points sit below the sea-level "
               "lines partly because of altitude and partly because the\n"
               "engines disagree." if has_fp else
               "Closed-form engines at sea level -- this rung has no "
               "first-principles march to overlay.")
    ax_thrust.set_title(
        f"Propulsion: thrust, Isp, and SFC vs Mach\n"
        f"D={g.diameter_m * 1000:.0f} mm, "
        f"throat={g.throat_diameter_m * 1000:.0f} mm, "
        f"chamber L={g.chamber_length_m * 1000:.0f} mm, "
        f"throat L={g.throat_length_m * 1000:.0f} mm, "
        f"fuel={g.fuel.display_name}\n{caption}", fontsize=9)

    fig.tight_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"propulsion_{tag}.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _headline(summary) -> str:
    return (f"gate M {summary['gate_mach']:.2f}"
            + (" + light at pull-out" if summary.get("light_at_pullout")
               else " in the dive")
            + f", top {summary['top_altitude_m']:.0f} m, "
              f"dive {summary['dive_angle_deg']:.0f}°, "
              f"climb {summary['climb_angle_deg']:.0f}°"
            + (" spiral" if summary.get("spiral_climb") else "")
            + f", {summary['pullout_load_factor']:.1f} g pull-out"
            + f"  [{summary['drag_model']} drag, "
              f"{summary['propulsion']} propulsion]")


def _short_note(summary) -> str:
    """Compact profile note. run_demo puts this on ONE suptitle line together
    with the mass/diameter/fuel block, so it has to stay short or the line
    runs off both edges of the figure."""
    return (f"{summary['climb_angle_deg']:.0f}°"
            + ("spiral" if summary.get("spiral_climb") else "")
            + f" → {summary['top_altitude_m']:.0f} m → "
              f"{summary['dive_angle_deg']:.0f}° dive, gate M "
              f"{summary['gate_mach']:.2f}"
            + ("+pull-out light" if summary.get("light_at_pullout") else "")
            + f", {summary['pullout_load_factor']:.1f} g pull-out"
            # The fidelity of the rung is the whole point of a medium_model
            # figure, so it is named here even though space is tight.
            + f"  [{summary['drag_model']}/{summary['propulsion']}]")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="v4_rungc_lap")
    args = ap.parse_args()
    tag = args.tag

    result, summary = load_trace(tag)

    # run_demo's plotters read module-level GEOMETRY / CLIMB_ANGLE_DEG /
    # OUT_DIR. Point them at this design for the duration, then put them back
    # so the module is left exactly as found for any later caller.
    from simple_model.constants import FUELS
    from simple_model.flight_sim import VehicleGeometry

    frozen = json.loads(
        (ROOT / "docs/v4_frozen/design.json").read_text())["vehicle"]
    geometry = VehicleGeometry(
        frozen["diameter_m"], frozen["throat_diameter_m"],
        frozen["chamber_length_m"], frozen["throat_length_m"],
        frozen["legacy_wingspan_m"], FUELS[frozen["fuel_key"]])

    saved = (run_demo.GEOMETRY, run_demo.CLIMB_ANGLE_DEG, run_demo.OUT_DIR)
    run_demo.GEOMETRY = geometry
    run_demo.CLIMB_ANGLE_DEG = summary["climb_angle_deg"]
    run_demo.OUT_DIR = OUT
    try:
        profile = run_demo.plot_flight_profile(
            result, summary["loaded_fuel_kg"],
            profile_note=_short_note(summary))
    finally:
        run_demo.GEOMETRY, run_demo.CLIMB_ANGLE_DEG, run_demo.OUT_DIR = saved

    # Path.replace, not .rename: .rename raises if the target already exists
    # on Windows.
    profile = Path(profile).replace(OUT / f"flight_profile_{tag}.png")
    loading = plot_body_loading(result, summary, tag)
    propulsion = plot_propulsion(result, summary, tag)

    for p in (profile, loading, propulsion):
        print(f"wrote {p}")


if __name__ == "__main__":
    main()
