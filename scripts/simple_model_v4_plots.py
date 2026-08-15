"""V4 plots in the established house style.

Reuses `simple_model.run_demo`'s own plotters -- the same 7-panel flight
profile (mode shading on every time panel, ft/lb units, mode-coloured
trajectory) and the same 3-panel propulsion figure the V2/V3 reports use --
rather than a second, divergent plotting stack. `run_demo` builds those from
module-level GEOMETRY/CLIMB_ANGLE_DEG, so those are pointed at the V4 design
for the duration of the call and restored afterwards.

Adds one companion figure in the same visual language for the V4-specific
quantities that have no home in the legacy layout: body loading on the two
axes a 2-D trajectory loads, the pitch-arc radii, and the flight path angle.

Usage: .venv/Scripts/python scripts/simple_model_v4_plots.py
Writes out_simple_model/{flight_profile_v4,propulsion_v4,body_loading_v4}.png
"""
from __future__ import annotations

import json
import sys
from math import degrees
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from scripts.simple_model_v4_baseline import (  # noqa: E402
    V4_DRAG_STRIP_ANGLE_DEG, V4_WING, fly, mass_block, v4_geometry)
from simple_model import run_demo  # noqa: E402
from simple_model.flight_sim import V3_FLOOR_ALTITUDE_M  # noqa: E402

OUT = ROOT / "out_simple_model"
_C_MAIN = run_demo._C_MAIN
_C_COMPANION = run_demo._C_COMPANION
_C_REF = run_demo._C_REF


def plot_body_loading(result, pick: dict) -> Path:
    """Body loading + pitch arcs, styled to match the flight profile figure:
    same mode background shading, same shared clock, same colour constants."""
    states = result.states
    time_s = [s.time_s for s in states]
    # Frame on the POWERED mission. The unpowered return half-loop pulls a
    # fixed RETURN_LOOP_LOAD_FACTOR (6 g) by construction -- it is not a
    # searched quantity, it is 2x the pull-out this figure exists to show, and
    # leaving it in scale squashes the arcs into invisibility.
    powered = [s for s in states if s.thrust_n > 0.0]
    t_powered_end = powered[-1].time_s if powered else time_s[-1]

    fig, (ax_load, ax_radius, ax_gamma) = plt.subplots(
        3, 1, figsize=(12.5, 10.0), sharex=True,
        gridspec_kw=dict(hspace=0.18, left=0.08, right=0.93, top=0.92, bottom=0.07))
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
                 f" peak {result.peak_load_n_total:.2f} g ({result.peak_load_mode}) ",
                 color=_C_REF, fontsize=8, va="bottom", ha="right")
    ax_load.set_ylabel("Body load factor (g)")
    ax_load.legend(loc="upper left", fontsize=8)
    ax_load.set_title(
        "Body loading -- accelerometer convention (gravity EXCLUDED), so the "
        "roll trace is (T-D)/W, not the along-path acceleration.  "
        "Powered mission only: the unpowered return loop pulls a fixed 6 g.",
        fontsize=9)

    radius_ft = [s.turn_radius_m / 0.3048 if s.turn_radius_m > 0.0 else float("nan")
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
        "GROUND-track circle, the arcs are in the vertical plane)", fontsize=10)

    ax_gamma.plot(time_s, [degrees(s.flight_path_angle_rad) for s in states],
                  color=_C_MAIN, linewidth=1.8)
    ax_gamma.axhline(0.0, color="black", linewidth=0.8)
    ax_gamma.set_ylabel("Flight path angle (deg)")
    ax_gamma.set_xlabel("Time (s)")
    if result.ramjet_lightoff_time_s is not None:
        for ax in (ax_load, ax_radius, ax_gamma):
            ax.axvline(result.ramjet_lightoff_time_s, color="crimson",
                       linewidth=1.2, alpha=0.7)
        # Anchor the label on whichever side keeps it inside the axes -- the
        # gate fires late in the powered phase, so a left-anchored label runs
        # off the right edge.
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

    ax_load.set_ylim(*_span([v for s in powered
                             for v in (s.load_n_total, s.load_n_yaw, s.load_n_roll)]))
    arc_radii_ft = [s.turn_radius_m / 0.3048 for s in powered if s.turn_radius_m > 0.0]
    if arc_radii_ft:
        ax_radius.set_ylim(*_span(arc_radii_ft, floor_pad=200.0))
    ax_gamma.set_ylim(*_span([degrees(s.flight_path_angle_rad) for s in powered],
                             floor_pad=2.0))

    fig.suptitle(
        f"V4 body loading and pitch arcs -- gate M {pick['gate']:.2f} in the dive, "
        f"top {pick['top_m']:.0f} m, dive {pick['dive_deg']:.0f}°, "
        f"climb {pick['climb_deg']:.0f}° spiral, {pick['pullout_n']:.1f} g nominal pull-out\n"
        f"(background shading = flight mode, as in the flight profile)",
        fontsize=11)
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "body_loading_v4.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main() -> None:
    pick = json.loads((OUT / "v4_pick.json").read_text())
    geometry = v4_geometry()
    result = fly(climb_deg=pick["climb_deg"], dive_deg=pick["dive_deg"],
                 floor_m=V3_FLOOR_ALTITUDE_M, gate_mach=pick["gate"],
                 pullout_n=pick["pullout_n"], top_altitude_m=pick["top_m"])
    mass = mass_block(result, geometry, V4_WING)

    # run_demo's plotters read module-level GEOMETRY / CLIMB_ANGLE_DEG /
    # OUT_DIR. Point them at V4 for the duration, then put them back so the
    # module is left exactly as found for any later caller in this process.
    saved = (run_demo.GEOMETRY, run_demo.CLIMB_ANGLE_DEG, run_demo.OUT_DIR)
    run_demo.GEOMETRY = geometry
    run_demo.CLIMB_ANGLE_DEG = V4_DRAG_STRIP_ANGLE_DEG
    run_demo.OUT_DIR = OUT
    try:
        propulsion = run_demo.plot_propulsion(lightoff_mach=pick["gate"])
        profile = run_demo.plot_flight_profile(
            result, mass["fuel_loaded_kg"],
            profile_note=(
                f"{pick['climb_deg']:.0f}° SPIRAL climb to {pick['top_m']:.0f} m"
                f" → {pick['dive_deg']:.0f}° dive, ramjet gate M {pick['gate']:.2f}"
                f" in the dive, {pick['pullout_n']:.1f} g pull-out"),
        )
    finally:
        run_demo.GEOMETRY, run_demo.CLIMB_ANGLE_DEG, run_demo.OUT_DIR = saved

    # keep the V4 plots under their own names (same convention as the V3
    # report). Path.replace, not .rename: .rename raises if the target
    # already exists on Windows.
    propulsion = Path(propulsion).replace(OUT / "propulsion_v4.png")
    profile = Path(profile).replace(OUT / "flight_profile_v4.png")
    loading = plot_body_loading(result, pick)

    for p in (profile, propulsion, loading):
        print(f"wrote {p}")


if __name__ == "__main__":
    main()
