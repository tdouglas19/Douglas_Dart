"""Demo runner: thrust/Isp/SFC-vs-Mach verification plot + one point-mass
flight (climb, glide, flare-to-stall-speed landing).

Run with:  python -m simple_model.run_demo

Writes two PNGs (propulsion.png, flight_profile.png) to
results/generated/simple_model/ and prints a design table + flight summary. GEOMETRY below is normally the output of
simple_model/optimize.py's search (see that module), not hand-picked --
change it freely, that's the point of this module being a handful of plain
function calls instead of a config file.
"""

from __future__ import annotations

from math import nan
from pathlib import Path

import matplotlib.pyplot as plt

from douglas_dart.atmosphere import SEA_LEVEL_PRESSURE_PA

from .constants import FUELS, G0_M_PER_S2, KG_PER_LB, M_PER_IN
from .flight_sim import VehicleGeometry, run_flight
from .pulsejet_simple import pulsejet_thrust
from .ramjet_simple import ramjet_thrust

OUT_DIR = Path("results/generated/simple_model")

# --- Demo design point -----------------------------------------------------
# Output of simple_model/optimize.py's search (parallel random search +
# batched parallel local refinement, both validated at the final dt -- see
# that module), not hand-picked: minimizes *peak thrust-to-weight ratio*
# (the "easiest to build" propulsion metric -- lower peak T/W means a
# smaller throat, lower chamber pressure, less structural load) subject to
# reaching motor-cutoff Mach, completing a *safe* landing (flare down to a
# stall speed capped at 45 m/s -- see optimize.py's
# MAX_ACCEPTABLE_STALL_SPEED_M_PER_S for why that cap is 45 m/s and not a
# true light-aircraft stall speed), and the fuel actually fitting: its
# volume must be <= 50% of the annular volume between the vehicle OD and
# the throat OD over the throat/nozzle section's length (see optimize.py's
# FUEL_VOLUME_FRACTION_OF_ANNULUS).
#
# This is the result of the re-run after adding wing parasitic drag
# (drag.py's wing_parasitic_drag_n) and promoting climb angle from a fixed
# 15 degrees to its own search variable (optimize.py's
# CLIMB_ANGLE_BOUNDS_DEG) -- both change the physics/search space enough
# that the old fuel-volume-constrained winner (max T/W ~4.2, jet_a) no
# longer even reaches motor cutoff before hitting the altitude ceiling
# (confirmed by direct re-run, not assumed). max T/W ~6.9 here is higher
# than that old number, but it's the honest cost of a strictly more
# complete drag model (wing parasitic drag was previously entirely
# missing) plus an independently re-optimized climb angle, not a
# regression in the search itself. Uses propane this time, ~24% of the
# available annular fuel-tank volume.
GEOMETRY = VehicleGeometry(
    diameter_m=0.17816917417736486,
    throat_diameter_m=0.13287763705797825,
    chamber_length_m=0.38736667101493366,
    throat_length_m=0.9809150845225322,
    wingspan_m=0.7394075790852311,
    fuel=FUELS["propane"],
)
REFERENCE_ALTITUDE_M = 0.0  # sea level, for the thrust-vs-Mach verification plot
MAX_WET_MASS_KG = 50.0 * KG_PER_LB
CLIMB_ANGLE_DEG = 12.243346555293243
# Fuel/thrust cut off here; the sim then glides unpowered to the ground (see
# flight_sim.py's module docstring for how the glide angle is found).
MOTOR_CUTOFF_MACH = 1.1
# Standard reserve margin: load 25% more fuel than the flight actually
# burns, rather than sizing the tank to exactly zero remaining at the end.
# Since the sim's mass at any instant is always MAX_WET_MASS_KG minus fuel
# burned so far, regardless of how that starting mass conceptually splits
# between structure and fuel, the whole trajectory (Mach, altitude, drag,
# everything) is unaffected by this margin -- it only changes how the fixed
# 50 lb wet mass gets divided into dry-mass and fuel-load line items after
# the fact.
FUEL_RESERVE_MARGIN = 0.25

# One palette for the whole flight-profile figure: steel blue is always
# "the quantity this panel is named after", burnt orange is always "its
# companion series", green/red are reserved for thrust/drag, gray for
# reference lines. Mode colors are used only for phase shading + the
# trajectory panel.
_C_MAIN = "#2E5E73"
_C_COMPANION = "#C05A2E"
_C_THRUST = "#2E7D46"
_C_DRAG = "#B3402F"
_C_REF = "#707070"
_MODE_COLORS = {
    "pulsejet": "tab:red",
    "ramjet": "tab:green",
    # V3 powered phases
    "v3_climb": "tab:olive",
    "v3_dive": "tab:brown",
    # V4 pitch arcs. Without these the arcs fall through to the "gray"
    # default and read as unlabelled gaps in the trajectory panel -- they are
    # short (3-5 s) but they are where all the body load lives.
    "v4_pushover": "tab:orange",
    "v4_pullout": "tab:green",
    "drag_strip": "tab:red",
    "loop": "goldenrod",
    "return": "tab:cyan",
    "spiral": "tab:pink",
    "decel": "tab:purple",
    "glide": "tab:blue",
    "flare": "tab:orange",
}


def _sfc_lb_per_lbf_hr(fuel_mass_flow_kg_per_s: float, thrust_n: float) -> float:
    """Thrust-specific fuel consumption in the conventional aviation unit
    (lb fuel per lbf thrust per hour) -- algebraically just 3600/Isp[s], but
    engineers read TSFC directly rather than via Isp, so it gets its own
    panel per the user's request rather than leaving it implicit."""

    if thrust_n <= 0.0:
        return nan
    fuel_lb_per_hr = fuel_mass_flow_kg_per_s * 3600.0 / KG_PER_LB
    thrust_lbf = thrust_n / 4.44822
    return fuel_lb_per_hr / thrust_lbf


def plot_propulsion(lightoff_mach: float | None = None) -> Path:
    # lightoff_mach (V4): draw the ramjet curve against the gate the design
    # actually flies rather than the module default, so the plot cannot show
    # the engine coming alive somewhere the mission does not. None keeps the
    # pre-V4 behaviour (RAMJET_MIN_LIGHTOFF_MACH).
    mach_values = [i * 0.02 for i in range(0, 76)]  # 0.00 .. 1.50
    pulsejet_results = [
        pulsejet_thrust(
            GEOMETRY.diameter_m,
            GEOMETRY.chamber_length_m,
            GEOMETRY.throat_diameter_m,
            GEOMETRY.throat_length_m,
            m,
            REFERENCE_ALTITUDE_M,
            GEOMETRY.fuel,
        )
        for m in mach_values
    ]
    ramjet_results = [
        ramjet_thrust(GEOMETRY.diameter_m, GEOMETRY.throat_diameter_m, m,
                      REFERENCE_ALTITUDE_M, GEOMETRY.fuel,
                      lightoff_mach=lightoff_mach)
        for m in mach_values
    ]
    pulsejet_thrust_n = [r.average_thrust_n for r in pulsejet_results]
    ramjet_thrust_n = [r.net_thrust_n for r in ramjet_results]
    # Isp/SFC are undefined (no fuel flow) wherever an engine produces no net
    # thrust at all -- leave those Mach points out of the line (NaN) rather
    # than a bogus zero. Gating on net thrust > 0, not on `choked`: the
    # ramjet now produces real (if small) thrust from just above Mach 0 in
    # the *unchoked* regime (see ramjet_simple.py's module docstring) --
    # gating on `choked` would wrongly blank out Isp for that whole unchoked
    # range even though thrust, fuel flow, and Isp are all well-defined
    # there, matching what build_design_table() already publishes.
    pulsejet_isp_s = [r.specific_impulse_s if r.average_thrust_n > 0.0 else nan for r in pulsejet_results]
    ramjet_isp_s = [r.specific_impulse_s if r.net_thrust_n > 0.0 else nan for r in ramjet_results]
    pulsejet_sfc = [_sfc_lb_per_lbf_hr(r.fuel_mass_flow_kg_per_s, r.average_thrust_n) for r in pulsejet_results]
    ramjet_sfc = [_sfc_lb_per_lbf_hr(r.fuel_mass_flow_kg_per_s, r.net_thrust_n) for r in ramjet_results]

    fig, (ax_thrust, ax_isp, ax_sfc) = plt.subplots(3, 1, figsize=(9, 12), sharex=True)

    ax_thrust.plot(mach_values, pulsejet_thrust_n, label="pulsejet (avg, closed-form)", color="tab:red")
    ax_thrust.plot(mach_values, ramjet_thrust_n, label="ramjet (net, closed-form)", color="tab:green")
    ax_thrust.axhline(0.0, color="black", linewidth=0.8)
    ax_thrust.set_ylabel("Thrust (N)")
    ax_thrust.legend(loc="best")
    ax_thrust.grid(alpha=0.3)
    ax_thrust.set_title(
        f"Propulsion: thrust, Isp, and SFC vs Mach, sea level\n"
        f"D={GEOMETRY.diameter_m * 1000:.0f} mm, throat={GEOMETRY.throat_diameter_m * 1000:.0f} mm, "
        f"chamber L={GEOMETRY.chamber_length_m * 1000:.0f} mm, throat L={GEOMETRY.throat_length_m * 1000:.0f} mm, "
        f"fuel={GEOMETRY.fuel.display_name}"
    )

    ax_isp.plot(mach_values, pulsejet_isp_s, label="pulsejet Isp", color="tab:red")
    ax_isp.plot(mach_values, ramjet_isp_s, label="ramjet Isp", color="tab:green")
    ax_isp.set_ylabel("Specific impulse (s)")
    ax_isp.legend(loc="best")
    ax_isp.grid(alpha=0.3)

    ax_sfc.plot(mach_values, pulsejet_sfc, label="pulsejet SFC", color="tab:red")
    ax_sfc.plot(mach_values, ramjet_sfc, label="ramjet SFC", color="tab:green")
    ax_sfc.set_ylabel("SFC (lb fuel / lbf·hr)")
    ax_sfc.set_xlabel("Mach")
    ax_sfc.legend(loc="best")
    ax_sfc.grid(alpha=0.3)

    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "propulsion.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def _mode_segments(states):
    """Contiguous (t_start, t_end, mode) runs of the flight mode."""
    segments = []
    start_time = states[0].time_s
    mode = states[0].mode
    for state in states[1:]:
        if state.mode != mode:
            segments.append((start_time, state.time_s, mode))
            start_time, mode = state.time_s, state.mode
    segments.append((start_time, states[-1].time_s, mode))
    return segments


def _shade_modes(ax, states):
    """Tint the background of a time panel by flight mode -- the one shared
    phase indicator for every panel, replacing per-panel marker clutter."""
    for t_start, t_end, mode in _mode_segments(states):
        ax.axvspan(t_start, t_end, color=_MODE_COLORS.get(mode, "gray"),
                   alpha=0.07, linewidth=0)


def _align_twin_ticks(ax_left, ax_right):
    """Pin a twin axis's ticks to the SAME vertical positions as the
    primary's, so both scales share one set of grid lines (mismatched
    grids read as noise)."""
    l_lo, l_hi = ax_left.get_ylim()
    r_lo, r_hi = ax_right.get_ylim()
    ticks = [t for t in ax_left.get_yticks() if l_lo <= t <= l_hi]
    ax_left.set_ylim(l_lo, l_hi)
    ax_left.set_yticks(ticks)
    mapped = [r_lo + (t - l_lo) * (r_hi - r_lo) / (l_hi - l_lo) for t in ticks]
    ax_right.set_ylim(r_lo, r_hi)
    ax_right.set_yticks(mapped)
    ax_right.set_yticklabels([f"{t:.1f}" for t in mapped])


def plot_flight_profile(result, fuel_loaded_kg: float,
                        profile_note: str | None = None) -> Path:
    """The whole flight on one figure: six time-history panels on a shared
    clock (Mach, altitude, velocity vs stall, thrust vs drag, mass & fuel
    remaining, T/W & acceleration), plus altitude-vs-downrange as a
    full-width bottom panel. Flight mode is shown once -- background
    shading on every time panel and point color in the trajectory panel."""

    states = result.states
    time_s = [s.time_s for s in states]

    fig = plt.figure(figsize=(12.5, 12.5))
    grid = fig.add_gridspec(4, 2, height_ratios=(1.0, 1.0, 1.0, 1.35),
                            hspace=0.30, wspace=0.32,
                            left=0.07, right=0.97, top=0.92, bottom=0.05)
    ax_mach = fig.add_subplot(grid[0, 0])
    ax_alt = fig.add_subplot(grid[0, 1], sharex=ax_mach)
    ax_vel = fig.add_subplot(grid[1, 0], sharex=ax_mach)
    ax_force = fig.add_subplot(grid[1, 1], sharex=ax_mach)
    ax_mass = fig.add_subplot(grid[2, 0], sharex=ax_mach)
    ax_tw = fig.add_subplot(grid[2, 1], sharex=ax_mach)
    time_axes = (ax_mach, ax_alt, ax_vel, ax_force, ax_mass, ax_tw)
    ax_traj = fig.add_subplot(grid[3, :])

    for ax in time_axes:
        _shade_modes(ax, states)
        ax.grid(alpha=0.25)
    for ax in time_axes[:4]:
        ax.tick_params(labelbottom=False)
    for ax in (ax_mass, ax_tw):
        ax.set_xlabel("Time (s)")

    ax_mach.plot(time_s, [s.mach for s in states], color=_C_MAIN)
    ax_mach.axhline(MOTOR_CUTOFF_MACH, color=_C_REF, linestyle=":", linewidth=1.0)
    ax_mach.text(time_s[-1], MOTOR_CUTOFF_MACH, "cutoff ", color=_C_REF,
                 fontsize=8, va="bottom", ha="right")
    ax_mach.set_ylabel("Mach")

    ax_alt.plot(time_s, [s.altitude_m / 0.3048 for s in states], color=_C_MAIN)
    ax_alt.set_ylabel("Altitude (ft)")

    ax_vel.plot(time_s, [s.velocity_m_per_s for s in states], color=_C_MAIN,
                label="velocity")
    ax_vel.plot(time_s, [s.stall_speed_m_per_s for s in states], color=_C_REF,
                linestyle="--", linewidth=1.0, label="stall speed")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.legend(loc="best", fontsize=8)

    ax_force.plot(time_s, [s.thrust_n for s in states], color=_C_THRUST, label="thrust")
    ax_force.plot(time_s, [s.drag_n for s in states], color=_C_DRAG, label="drag")
    ax_force.set_ylabel("Force (N)")
    ax_force.legend(loc="best", fontsize=8)

    mass_lb = [s.mass_kg / KG_PER_LB for s in states]
    fuel_remaining_lb = [(fuel_loaded_kg - s.fuel_burned_kg) / KG_PER_LB for s in states]
    reserve_lb = fuel_loaded_kg * FUEL_RESERVE_MARGIN / (1.0 + FUEL_RESERVE_MARGIN) / KG_PER_LB
    # mass and fuel remaining differ by a CONSTANT (mass = dry mass + fuel),
    # so ONE line serves both axes exactly: the left scale reads vehicle
    # mass, the right scale reads fuel remaining (left minus the dry
    # offset). Right ticks are pinned to the left ticks' positions.
    offset_lb = mass_lb[0] - fuel_remaining_lb[0]
    ax_mass.plot(time_s, mass_lb, color=_C_MAIN,
                 label="vehicle mass (left) / fuel remaining (right)")
    ax_mass.set_ylabel("Vehicle mass (lb)")
    ax_fuel = ax_mass.twinx()
    l_lo, l_hi = ax_mass.get_ylim()
    ax_fuel.set_ylim(l_lo - offset_lb, l_hi - offset_lb)
    ax_fuel.axhline(reserve_lb, color=_C_REF, linestyle=":", linewidth=1.0,
                    label=f"{FUEL_RESERVE_MARGIN:.0%} reserve (fuel)")
    ax_fuel.set_ylabel("Fuel remaining (lb)")
    _align_twin_ticks(ax_mass, ax_fuel)
    handles = (ax_mass.get_legend_handles_labels()[0]
               + ax_fuel.get_legend_handles_labels()[0])
    labels = (ax_mass.get_legend_handles_labels()[1]
              + ax_fuel.get_legend_handles_labels()[1])
    ax_fuel.legend(handles, labels, loc="center right", fontsize=8)

    ax_tw.plot(time_s, [s.thrust_to_weight for s in states], color=_C_MAIN,
               label="thrust / weight")
    ax_tw.axhline(1.0, color=_C_REF, linestyle=":", linewidth=1.0)
    ax_tw.set_ylabel("Thrust / weight", color=_C_MAIN)
    ax_tw.tick_params(axis="y", labelcolor=_C_MAIN)
    ax_acc = ax_tw.twinx()
    ax_acc.plot(time_s, [s.acceleration_m_per_s2 / G0_M_PER_S2 for s in states],
                color=_C_COMPANION, label="acceleration (g)")
    ax_acc.axhline(0.0, color="black", linewidth=0.8)
    ax_acc.set_ylabel("Acceleration (g)", color=_C_COMPANION)
    ax_acc.tick_params(axis="y", labelcolor=_C_COMPANION)
    _align_twin_ticks(ax_tw, ax_acc)
    handles = (ax_tw.get_legend_handles_labels()[0]
               + ax_acc.get_legend_handles_labels()[0])
    labels = (ax_tw.get_legend_handles_labels()[1]
              + ax_acc.get_legend_handles_labels()[1])
    ax_acc.legend(handles, labels, loc="best", fontsize=8)

    for mode, color in _MODE_COLORS.items():
        segment_x = [s.distance_m / 0.3048 for s in states if s.mode == mode]
        segment_y = [s.altitude_m / 0.3048 for s in states if s.mode == mode]
        if segment_x:
            ax_traj.plot(segment_x, segment_y, ".", color=color, markersize=3,
                         label=mode)
    final = states[-1]
    if result.safe_landing:
        touchdown_note = (
            f"landed at stall speed: {final.velocity_m_per_s:.0f} m/s "
            f"({final.velocity_m_per_s * 2.237:.0f} mph)"
        )
    elif result.landed:
        touchdown_note = (
            f"UNSAFE landing -- hit the ground still gliding at "
            f"{final.velocity_m_per_s:.0f} m/s, never slowed enough to flare"
        )
    else:
        touchdown_note = (
            f"did not land -- {'stalled' if result.stalled else 'time/mass limit'} "
            f"at {final.altitude_m / 0.3048:.0f} ft"
        )
    ax_traj.axhline(0.0, color="black", linewidth=0.8)
    ax_traj.grid(alpha=0.25)
    ax_traj.set_xlabel("Downrange distance (ft)")
    ax_traj.set_ylabel("Altitude (ft)")
    ax_traj.set_title(
        f"Trajectory ({touchdown_note}; ends "
        f"{abs(final.distance_m) / 0.3048:.0f} ft from launch)", fontsize=10)
    ax_traj.legend(loc="best", fontsize=8, markerscale=2.5)

    # profile_note (V4): CLIMB_ANGLE_DEG is the single powered climb angle for
    # V2, but for a V3/V4 climb-dive profile it is only the post-pullout drag
    # strip -- printing it as "the climb" actively misdescribes the flight
    # (V4 climbs at 6 deg and would be titled "1 deg climb"). Callers with a
    # multi-phase profile pass their own description.
    fig.suptitle(
        f"Flight profile: "
        f"{profile_note if profile_note is not None else f'{CLIMB_ANGLE_DEG:.0f}° climb'}"
        f" -- wet mass {MAX_WET_MASS_KG / KG_PER_LB:.0f} lb, "
        f"D={GEOMETRY.diameter_m * 1000:.0f} mm, fuel={GEOMETRY.fuel.display_name}\n"
        f"(background shading / trajectory point color = flight mode)",
        fontsize=11,
    )
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "flight_profile.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def build_design_table() -> str:
    """Markdown table of key dimensions + chamber pressure/temperature at the
    demo design point. Pulsejet peak P/T are Mach-independent in this model
    (see pulsejet_simple.py's docstring), so they get one row; ramjet's
    depend on Mach, so it gets one row per representative Mach.
    """

    lines = ["### Key dimensions", "", "| Quantity | Value |", "|---|---|"]
    dims = [
        ("Vehicle (body) diameter", GEOMETRY.diameter_m),
        ("Throat diameter", GEOMETRY.throat_diameter_m),
        ("Chamber length (pulsejet)", GEOMETRY.chamber_length_m),
        ("Throat length (resonance tube)", GEOMETRY.throat_length_m),
        ("Wingspan", GEOMETRY.wingspan_m),
    ]
    for label, value_m in dims:
        lines.append(f"| {label} | {value_m * 1000:.0f} mm ({value_m / M_PER_IN:.2f} in) |")
    lines.append(f"| Fuel | {GEOMETRY.fuel.display_name} |")
    lines.append(f"| Wet mass | {MAX_WET_MASS_KG:.2f} kg ({MAX_WET_MASS_KG / KG_PER_LB:.1f} lb) |")
    # Annular volume between the vehicle OD and throat OD, over the
    # throat/nozzle section's length -- the fuel tank's available space
    # (see optimize.py's FUEL_VOLUME_FRACTION_OF_ANNULUS). A pure geometry
    # quantity, so it's reported here; actual fuel volume needed depends on
    # the flight (fuel burned), reported in the flight summary instead.
    annular_volume_l = (
        (3.141592653589793 / 4.0) * (GEOMETRY.diameter_m**2 - GEOMETRY.throat_diameter_m**2)
        * GEOMETRY.throat_length_m * 1000.0
    )
    lines.append(f"| Annular volume available for fuel tank (throat section) | {annular_volume_l:.2f} L |")

    pulsejet_result = pulsejet_thrust(
        GEOMETRY.diameter_m,
        GEOMETRY.chamber_length_m,
        GEOMETRY.throat_diameter_m,
        GEOMETRY.throat_length_m,
        0.0,
        REFERENCE_ALTITUDE_M,
        GEOMETRY.fuel,
    )
    lines += [
        "",
        "### Pulsejet chamber conditions (sea level; Mach-independent in this model)",
        "",
        "| Quantity | Value |",
        "|---|---|",
        f"| Chamber volume | {pulsejet_result.chamber_volume_m3 * 1000:.2f} L |",
        f"| Peak pressure | {pulsejet_result.peak_pressure_pa / SEA_LEVEL_PRESSURE_PA:.2f} atm "
        f"({pulsejet_result.peak_pressure_pa / 1000:.0f} kPa) |",
        f"| Peak temperature | {pulsejet_result.peak_temperature_k:.0f} K |",
        f"| Cycle frequency | {pulsejet_result.frequency_hz:.1f} Hz |",
        f"| Average thrust | {pulsejet_result.average_thrust_n:.0f} N ({pulsejet_result.average_thrust_n / 4.448:.0f} lbf) |",
        f"| Peak thrust | {pulsejet_result.peak_thrust_n:.0f} N ({pulsejet_result.peak_thrust_n / 4.448:.0f} lbf) |",
        f"| Specific impulse | {pulsejet_result.specific_impulse_s:.0f} s |",
    ]

    # Full Mach range the ramjet actually produces thrust over -- it now
    # ramps up smoothly from just above Mach 0 (see ramjet_simple.py's
    # module docstring) rather than switching on at the old hard choking
    # cutoff, so "produces thrust" is a real, wide range worth publishing
    # in full rather than a handful of representative points.
    fine_mach_values = [i * 0.01 for i in range(0, 251)]  # 0.00 .. 2.50
    thrust_onset_mach = next(
        (
            m for m in fine_mach_values
            if ramjet_thrust(GEOMETRY.diameter_m, GEOMETRY.throat_diameter_m, m, REFERENCE_ALTITUDE_M, GEOMETRY.fuel).net_thrust_n > 0.1
        ),
        None,
    )
    lines += [
        "",
        "### Ramjet chamber conditions across its full thrust-producing Mach range (sea level)",
        "",
    ]
    if thrust_onset_mach is None:
        lines.append("_Ramjet never produces net thrust for this geometry at sea level._")
    else:
        lines.append(f"Produces net thrust from Mach {thrust_onset_mach:.2f} up through at least Mach 2.5 (this sweep's ceiling).")
        lines += [
            "",
            "| Mach | Chamber pressure | Chamber temperature | Net thrust | Isp | Choked? | Spilled? |",
            "|---|---|---|---|---|---|---|",
        ]
        table_mach_values = sorted({round(thrust_onset_mach, 2)} | {round(m, 2) for m in fine_mach_values if m >= thrust_onset_mach and round(m * 100) % 10 == 0})
        for mach in table_mach_values:
            r = ramjet_thrust(GEOMETRY.diameter_m, GEOMETRY.throat_diameter_m, mach, REFERENCE_ALTITUDE_M, GEOMETRY.fuel)
            if r.net_thrust_n <= 0.0:
                continue
            lines.append(
                f"| {mach:.2f} | {r.chamber_total_pressure_pa / SEA_LEVEL_PRESSURE_PA:.2f} atm "
                f"| {r.chamber_total_temperature_k:.0f} K | {r.net_thrust_n:.0f} N "
                f"| {r.specific_impulse_s:.0f} s | {'yes' if r.choked else 'no'} | {'yes' if r.spilled else 'no'} |"
            )

    return "\n".join(lines)


def main() -> None:
    print(build_design_table())
    print()

    propulsion_plot_path = plot_propulsion()
    print(f"Wrote {propulsion_plot_path.resolve()}")

    result = run_flight(
        GEOMETRY,
        MAX_WET_MASS_KG,
        climb_angle_deg=CLIMB_ANGLE_DEG,
        motor_cutoff_mach=MOTOR_CUTOFF_MACH,
        # flight_sim.run_flight's own default (240s) is fine for a
        # high-T/W design but cuts off a deliberately low-T/W, slow-climb
        # design before it finishes landing -- match optimize.py's own
        # FINAL_MAX_TIME_S so the published demo reflects a completed flight.
        max_time_s=600.0,
        return_to_launch=True,
    )
    cutoff_state = next((s for s in result.states if s.mode in ("glide", "flare")), None)
    flare_state = next((s for s in result.states if s.mode == "flare"), None)
    # Stall speed falls throughout the climb (mass drops as fuel burns) while
    # velocity rises (thrust), so this is the first point where the vehicle
    # is going fast enough to fly on its own -- not necessarily t=0, since
    # release velocity (DEFAULT_RELEASE_VELOCITY_M_PER_S) is fixed while the
    # release-instant stall speed (full wet mass, the heaviest the vehicle
    # ever is) can be well above it.
    takeoff_stall_state = next((s for s in result.states if s.velocity_m_per_s >= s.stall_speed_m_per_s), None)
    final = result.states[-1]

    total_fuel_consumed_kg = final.fuel_burned_kg
    fuel_loaded_kg = (1.0 + FUEL_RESERVE_MARGIN) * total_fuel_consumed_kg
    dry_mass_kg = MAX_WET_MASS_KG - fuel_loaded_kg
    flight_plot_path = plot_flight_profile(result, fuel_loaded_kg)
    print(f"Wrote {flight_plot_path.resolve()}")

    print()
    print("--- Flight summary ---")
    print(f"Motor cutoff (Mach {MOTOR_CUTOFF_MACH}) reached: {result.motor_cutoff_reached}")
    if result.crossover_mach is not None:
        print(f"Pulsejet -> ramjet crossover at Mach {result.crossover_mach:.3f}")
    else:
        print("Never crossed over to ramjet (pulsejet thrust exceeded ramjet the whole powered flight)")
    if cutoff_state is not None:
        print(
            f"Cutoff at t={cutoff_state.time_s:.2f}s, altitude={cutoff_state.altitude_m:.0f} m "
            f"({cutoff_state.altitude_m / 0.3048:.0f} ft), fuel burned by cutoff={cutoff_state.fuel_burned_kg:.3f} kg "
            f"({cutoff_state.fuel_burned_kg / KG_PER_LB:.2f} lb)"
        )
    if flare_state is not None:
        print(
            f"Flare begins at t={flare_state.time_s:.2f}s, altitude={flare_state.altitude_m / 0.3048:.0f} ft, "
            f"V={flare_state.velocity_m_per_s:.0f} m/s (stall speed there: {flare_state.stall_speed_m_per_s:.0f} m/s)"
        )
    if takeoff_stall_state is not None:
        print(
            f"Reaches its own stall speed ({takeoff_stall_state.stall_speed_m_per_s:.0f} m/s) at "
            f"t={takeoff_stall_state.time_s:.2f}s, {takeoff_stall_state.distance_m:.0f} m "
            f"({takeoff_stall_state.distance_m / 0.3048:.0f} ft) downrange from release"
        )
    else:
        print("Never reaches its own stall speed during the tracked flight")
    print(
        f"Landed: {result.landed}  |  Safe landing (at stall speed): {result.safe_landing}  |  "
        f"Stalled: {result.stalled}  |  Hit mass floor: {result.hit_mass_floor}"
    )
    if result.safe_landing:
        print(
            f"Touchdown after {final.time_s:.1f}s, {final.distance_m / 0.3048:.0f} ft downrange, "
            f"at stall speed {final.velocity_m_per_s:.0f} m/s ({final.velocity_m_per_s * 2.237:.0f} mph)"
        )
    elif result.landed:
        print(
            f"UNSAFE: flew into the ground still gliding after {final.time_s:.1f}s, "
            f"{final.distance_m / 0.3048:.0f} ft downrange, at {final.velocity_m_per_s:.0f} m/s "
            f"({final.velocity_m_per_s * 2.237:.0f} mph) -- never slowed enough to flare"
        )
    else:
        print(
            f"Did NOT land within the time cap -- last state at t={final.time_s:.1f}s, "
            f"altitude={final.altitude_m / 0.3048:.0f} ft, velocity={final.velocity_m_per_s:.0f} m/s"
        )
    print()
    print(f"Total fuel consumed (whole flight): {total_fuel_consumed_kg / KG_PER_LB:.2f} lb")
    print(f"Fuel loaded ({FUEL_RESERVE_MARGIN:.0%} reserve margin): {fuel_loaded_kg / KG_PER_LB:.2f} lb")
    print(f"Implied dry mass (50 lb wet - fuel loaded): {dry_mass_kg / KG_PER_LB:.2f} lb")

    fuel_volume_l = fuel_loaded_kg / GEOMETRY.fuel.density_kg_per_m3 * 1000.0
    annular_volume_l = (
        (3.141592653589793 / 4.0) * (GEOMETRY.diameter_m**2 - GEOMETRY.throat_diameter_m**2)
        * GEOMETRY.throat_length_m * 1000.0
    )
    print(
        f"Fuel volume needed: {fuel_volume_l:.2f} L, using "
        f"{100.0 * fuel_volume_l / annular_volume_l:.0f}% of the {annular_volume_l:.2f} L annulus "
        f"(50% limit)"
    )


if __name__ == "__main__":
    main()
