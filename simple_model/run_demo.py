"""Demo runner: thrust/Isp/SFC-vs-Mach verification plot + one point-mass
flight (climb, glide, flare-to-stall-speed landing).

Run with:  python -m simple_model.run_demo

Writes four PNGs to results/generated/simple_model/ and prints a design
table + flight summary. GEOMETRY below is normally the output of
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
# FUEL_VOLUME_FRACTION_OF_ANNULUS) -- this design uses ~34% of that space.
# max T/W ~4.2 here (higher than an earlier, fuel-tank-unconstrained
# version of this same search found -- fitting the tank costs some of the
# T/W headroom that version was spending on a smaller vehicle).
GEOMETRY = VehicleGeometry(
    diameter_m=0.17084211946682157,
    throat_diameter_m=0.10568979495538257,
    chamber_length_m=0.5491118290154582,
    throat_length_m=0.929042838305439,
    wingspan_m=0.8983266008711583,
    fuel=FUELS["jet_a"],
)
REFERENCE_ALTITUDE_M = 0.0  # sea level, for the thrust-vs-Mach verification plot
MAX_WET_MASS_KG = 50.0 * KG_PER_LB
CLIMB_ANGLE_DEG = 15.0
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

_MODE_COLORS = {"pulsejet": "tab:red", "ramjet": "tab:green", "glide": "tab:blue", "flare": "tab:orange"}


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


def plot_thrust_vs_mach() -> Path:
    mach_values = [i * 0.02 for i in range(0, 111)]  # 0.00 .. 2.20
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
        ramjet_thrust(GEOMETRY.diameter_m, GEOMETRY.throat_diameter_m, m, REFERENCE_ALTITUDE_M, GEOMETRY.fuel)
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
        f"simple_model: thrust, Isp, and SFC vs Mach, sea level\n"
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
    out_path = OUT_DIR / "thrust_vs_mach.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def _phase_markers(states):
    """(time, label, color) for the pulsejet->ramjet crossover (if any),
    motor cutoff (if reached), and flare initiation (if reached), used to
    annotate plots."""

    markers = []
    if any(s.mode == "ramjet" for s in states):
        crossover_state = next(s for s in states if s.mode == "ramjet")
        markers.append((crossover_state.time_s, f"pulsejet→ramjet\nM={crossover_state.mach:.2f}", "gray"))
    if any(s.mode in ("glide", "flare") for s in states):
        cutoff_state = next(s for s in states if s.mode in ("glide", "flare"))
        markers.append((cutoff_state.time_s, f"motor cutoff\nM={cutoff_state.mach:.2f}", "black"))
    if any(s.mode == "flare" for s in states):
        flare_state = next(s for s in states if s.mode == "flare")
        markers.append(
            (flare_state.time_s, f"flare begins\nV={flare_state.velocity_m_per_s:.0f} m/s", "tab:orange")
        )
    return markers


def plot_flight_profile(result) -> Path:
    states = result.states
    time_s = [s.time_s for s in states]
    mach = [s.mach for s in states]
    velocity = [s.velocity_m_per_s for s in states]
    stall_speed = [s.stall_speed_m_per_s for s in states]
    altitude_ft = [s.altitude_m / 0.3048 for s in states]
    mass_lb = [s.mass_kg / KG_PER_LB for s in states]
    thrust_n = [s.thrust_n for s in states]
    drag_n = [s.drag_n for s in states]
    thrust_to_weight = [s.thrust_to_weight for s in states]
    acceleration_g = [s.acceleration_m_per_s2 / G0_M_PER_S2 for s in states]

    fig, axes = plt.subplots(7, 1, figsize=(9, 17), sharex=True)

    axes[0].plot(time_s, mach, color="tab:blue")
    axes[0].axhline(MOTOR_CUTOFF_MACH, color="gray", linestyle=":", label="motor cutoff Mach")
    axes[0].set_ylabel("Mach")
    axes[0].legend(loc="lower right", fontsize=8)

    axes[1].plot(time_s, velocity, color="tab:blue", label="velocity")
    axes[1].plot(time_s, stall_speed, color="gray", linestyle=":", label="stall speed")
    axes[1].set_ylabel("Velocity (m/s)")
    axes[1].legend(loc="best", fontsize=8)

    axes[2].plot(time_s, altitude_ft, color="tab:orange")
    axes[2].set_ylabel("Altitude (ft)")

    axes[3].plot(time_s, mass_lb, color="tab:purple")
    axes[3].set_ylabel("Mass (lb)")

    axes[4].plot(time_s, thrust_to_weight, color="tab:brown")
    axes[4].axhline(1.0, color="gray", linestyle=":", linewidth=0.8)
    axes[4].set_ylabel("Thrust / Weight")

    axes[5].plot(time_s, acceleration_g, color="tab:cyan")
    axes[5].axhline(0.0, color="black", linewidth=0.8)
    axes[5].set_ylabel("Acceleration (g)")

    axes[6].plot(time_s, thrust_n, color="tab:green", label="thrust")
    axes[6].plot(time_s, drag_n, color="tab:red", label="drag")
    axes[6].set_ylabel("Force (N)")
    axes[6].set_xlabel("Time (s)")
    axes[6].legend(loc="best", fontsize=8)

    for marker_index, (marker_time, label, color) in enumerate(_phase_markers(states)):
        for ax in axes:
            ax.axvline(marker_time, color=color, linestyle="--", linewidth=0.8)
        y_fraction = 0.05 + 0.12 * marker_index
        axes[0].text(marker_time, axes[0].get_ylim()[1] * y_fraction, f" {label}", fontsize=8, color=color)

    axes[0].set_title(
        f"simple_model: point-mass flight, {CLIMB_ANGLE_DEG:.0f}° climb, unpowered glide, flare to stall speed\n"
        f"wet mass={MAX_WET_MASS_KG / KG_PER_LB:.0f} lb, "
        f"D={GEOMETRY.diameter_m * 1000:.0f} mm, span={GEOMETRY.wingspan_m * 1000:.0f} mm, "
        f"fuel={GEOMETRY.fuel.display_name}"
    )
    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "flight_profile.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_altitude_vs_distance(result) -> Path:
    states = result.states
    distance_ft = [s.distance_m / 0.3048 for s in states]
    altitude_ft = [s.altitude_m / 0.3048 for s in states]

    fig, ax = plt.subplots(figsize=(10, 6))
    # Color each point by propulsion mode so the climb/cruise/glide arc is
    # visible in the trajectory shape itself, not just in a separate plot.
    for mode, color in _MODE_COLORS.items():
        segment_x = [d for d, s in zip(distance_ft, states) if s.mode == mode]
        segment_y = [a for a, s in zip(altitude_ft, states) if s.mode == mode]
        if segment_x:
            ax.plot(segment_x, segment_y, ".", color=color, markersize=3, label=mode)

    for marker_index, (marker_time, label, color) in enumerate(_phase_markers(states)):
        marker_state = next(s for s in states if s.time_s == marker_time)
        ax.axvline(marker_state.distance_m / 0.3048, color=color, linestyle="--", linewidth=0.8)
        ax.annotate(
            label, (marker_state.distance_m / 0.3048, marker_state.altitude_m / 0.3048),
            textcoords="offset points", xytext=(6, 6 + 30 * marker_index), fontsize=8, color=color,
        )

    final = states[-1]
    if result.safe_landing:
        touchdown_note = (
            f"landed at stall speed: {final.velocity_m_per_s:.0f} m/s ({final.velocity_m_per_s * 2.237:.0f} mph)"
        )
    elif result.landed:
        touchdown_note = (
            f"UNSAFE landing -- flew into the ground still gliding at {final.velocity_m_per_s:.0f} m/s "
            f"({final.velocity_m_per_s * 2.237:.0f} mph), never slowed enough to flare"
        )
    else:
        touchdown_note = (
            f"did not land -- {'stalled' if result.stalled else 'time/mass limit'} at "
            f"{final.altitude_m / 0.3048:.0f} ft, {final.velocity_m_per_s:.0f} m/s"
        )
    ax.set_xlabel("Downrange distance (ft)")
    ax.set_ylabel("Altitude (ft)")
    ax.set_title(
        f"simple_model: altitude vs downrange distance\n{touchdown_note}"
    )
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    ax.axhline(0.0, color="black", linewidth=0.8)
    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "altitude_vs_distance.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_fuel_mass(result, fuel_loaded_kg: float) -> Path:
    """Fuel remaining vs time, for a tank sized at fuel_loaded_kg = (1 +
    FUEL_RESERVE_MARGIN) * (fuel actually consumed this flight)."""

    states = result.states
    time_s = [s.time_s for s in states]
    fuel_remaining_lb = [(fuel_loaded_kg - s.fuel_burned_kg) / KG_PER_LB for s in states]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time_s, fuel_remaining_lb, color="tab:purple")
    ax.axhline(0.0, color="black", linewidth=0.8)
    reserve_lb = fuel_loaded_kg * FUEL_RESERVE_MARGIN / (1.0 + FUEL_RESERVE_MARGIN) / KG_PER_LB
    ax.axhline(reserve_lb, color="gray", linestyle=":", label=f"{FUEL_RESERVE_MARGIN:.0%} reserve ({reserve_lb:.2f} lb)")

    for marker_index, (marker_time, label, color) in enumerate(_phase_markers(states)):
        ax.axvline(marker_time, color=color, linestyle="--", linewidth=0.8)
        y_fraction = 0.85 - 0.12 * marker_index
        ax.text(marker_time, ax.get_ylim()[1] * y_fraction, f" {label}", fontsize=8, color=color)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Fuel remaining (lb)")
    ax.set_title(
        f"simple_model: fuel remaining vs time\n"
        f"loaded={fuel_loaded_kg / KG_PER_LB:.2f} lb "
        f"({FUEL_RESERVE_MARGIN:.0%} margin over {(fuel_loaded_kg / (1.0 + FUEL_RESERVE_MARGIN)) / KG_PER_LB:.2f} lb consumed)"
    )
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "fuel_mass.png"
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

    thrust_plot_path = plot_thrust_vs_mach()
    print(f"Wrote {thrust_plot_path.resolve()}")

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
    )
    flight_plot_path = plot_flight_profile(result)
    print(f"Wrote {flight_plot_path.resolve()}")
    trajectory_plot_path = plot_altitude_vs_distance(result)
    print(f"Wrote {trajectory_plot_path.resolve()}")

    cutoff_state = next((s for s in result.states if s.mode in ("glide", "flare")), None)
    flare_state = next((s for s in result.states if s.mode == "flare"), None)
    final = result.states[-1]

    total_fuel_consumed_kg = final.fuel_burned_kg
    fuel_loaded_kg = (1.0 + FUEL_RESERVE_MARGIN) * total_fuel_consumed_kg
    dry_mass_kg = MAX_WET_MASS_KG - fuel_loaded_kg
    fuel_plot_path = plot_fuel_mass(result, fuel_loaded_kg)
    print(f"Wrote {fuel_plot_path.resolve()}")

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
