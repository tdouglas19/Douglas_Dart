"""Phase-based longitudinal mission trajectory integrator.

This replaces the previously missing "coupled mission model" named in
``docs/roadmap.md``: sled release -> pulsejet climb -> pulsejet acceleration ->
optional shallow dive -> ramjet ignition -> transonic acceleration -> fuel-limited
Mach-1.10 hold -> zoom climb -> unpowered glide -> landing.

Design choices, stated explicitly so they are not mistaken for validated dynamics:

- Flight path angle is *prescribed* per phase rather than solved from a lift/trim
  balance. This is an energy-state longitudinal model, not a full 6-DOF or even a
  trimmed 3-DOF simulation. Stability, control authority, and AoA limits are not
  represented here (see ``docs/roadmap.md`` "Coupled mission model").
- Drag uses the Mach-indexed drag-area model in :mod:`douglas_dart.drag`, which is
  itself an unvalidated proxy anchored at the single peak-Mach design point.
- Pulsejet thrust below Mach ~0.5 is read from a short static/low-speed table built
  once per run with :class:`douglas_dart.pulsejet.PulsejetSimulator` at sea level and
  scaled by the local-to-sea-level density ratio. This is a first-order installed
  effect, not a re-run of the unsteady chamber model at every trajectory time step.
- Ramjet thrust is the steady station model in :mod:`douglas_dart.ramjet`, evaluated
  at each step's altitude and Mach.
- The Mach-1.10 hold uses the same linear throttle/fuel scaling already used in
  ``sizing.py`` and is fuel-limited, not duration-prescribed, per the mission
  requirement that speed-run duration is an output, not an input.
- Every scenario multiplier (thrust, drag, ramjet recovery, mass growth) matches the
  named scenarios in ``configs/robustness_candidate_b.yaml`` so this integrator can be
  run at the same nominal/conservative/adverse points as the static screens.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan, cos, radians, sin, sqrt

from .atmosphere import standard_atmosphere
from .config import ReferenceCase
from .drag import evaluate_total_drag
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet

G0_M_PER_S2 = 9.80665
_SEA_LEVEL_DENSITY_KG_PER_M3 = standard_atmosphere(0.0).density_kg_per_m3


@dataclass(frozen=True)
class MissionScenario:
    """One named off-nominal point, matching ``robustness_candidate_b.yaml``."""

    name: str
    thrust_multiplier: float = 1.0
    drag_multiplier: float = 1.0
    ramjet_total_pressure_recovery: float | None = None
    mass_growth_kg: float = 0.0

    def __post_init__(self) -> None:
        if self.thrust_multiplier <= 0.0:
            raise ValueError("thrust multiplier must be positive")
        if self.drag_multiplier <= 0.0:
            raise ValueError("drag multiplier must be positive")
        if self.mass_growth_kg < 0.0:
            raise ValueError("mass growth cannot be negative")
        if self.ramjet_total_pressure_recovery is not None and not (
            0.0 < self.ramjet_total_pressure_recovery <= 1.0
        ):
            raise ValueError("ramjet total-pressure recovery must be in (0, 1]")


NOMINAL_SCENARIO = MissionScenario("nominal")
CONSERVATIVE_SCENARIO = MissionScenario(
    "conservative",
    thrust_multiplier=0.85,
    drag_multiplier=1.10,
    ramjet_total_pressure_recovery=0.87,
    mass_growth_kg=1.50,
)
ADVERSE_SCENARIO = MissionScenario(
    "adverse",
    thrust_multiplier=0.75,
    drag_multiplier=1.20,
    ramjet_total_pressure_recovery=0.82,
    mass_growth_kg=2.90,
)


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    phase: str
    altitude_m: float
    downrange_m: float
    speed_m_per_s: float
    mach: float
    flight_path_angle_deg: float
    mass_kg: float
    pulsejet_fuel_remaining_kg: float
    ramjet_fuel_remaining_kg: float
    thrust_n: float
    drag_n: float
    net_axial_force_n: float


@dataclass(frozen=True)
class PhaseOutcome:
    name: str
    start_time_s: float
    end_time_s: float
    start_altitude_m: float
    end_altitude_m: float
    start_mach: float
    end_mach: float
    fuel_used_kg: float
    termination_reason: str


@dataclass(frozen=True)
class TrajectoryResult:
    scenario: str
    thrust_multiplier: float
    drag_multiplier: float
    ramjet_total_pressure_recovery: float
    loaded_mass_kg: float
    points: tuple[TrajectoryPoint, ...]
    phases: tuple[PhaseOutcome, ...]
    peak_mach_reached: float
    reached_peak_mach_target: bool
    time_above_mach_one_s: float
    minimum_time_above_mach_one_requirement_s: float
    meets_minimum_time_above_mach_one: bool
    landed_at_or_below_field_elevation: bool
    transonic_no_altitude_loss_rule_satisfied: bool
    final_status: tuple[str, ...]
    numerical_reference_only: bool = True


def _pulsejet_static_thrust_table(
    case: ReferenceCase,
    scenario: MissionScenario,
    mach_values: tuple[float, ...],
    *,
    warmup_s: float = 0.25,
    measurement_s: float = 0.25,
    time_step_s: float = 0.00005,
) -> list[tuple[float, float, float]]:
    """Return ``[(mach, mean_net_thrust_n, mean_fuel_flow_kg_per_s), ...]`` at sea level."""

    table: list[tuple[float, float, float]] = []
    for mach in mach_values:
        simulator = PulsejetSimulator(
            case.pulsejet, case.selector, case.nozzle, case.fuel, 0.0, mach
        )
        summary = summarize_pulsejet(
            simulator.run(warmup_s + measurement_s, time_step_s),
            minimum_time_s=warmup_s,
        )
        table.append(
            (
                mach,
                summary.mean_net_thrust_n * scenario.thrust_multiplier,
                summary.mean_fuel_mass_flow_kg_per_s,
            )
        )
    return table


def _interp_table(table: list[tuple[float, float, float]], mach: float) -> tuple[float, float]:
    if mach <= table[0][0]:
        return table[0][1], table[0][2]
    if mach >= table[-1][0]:
        return table[-1][1], table[-1][2]
    for index in range(1, len(table)):
        lower, upper = table[index - 1], table[index]
        if lower[0] <= mach <= upper[0]:
            fraction = (mach - lower[0]) / (upper[0] - lower[0])
            thrust_n = lower[1] + fraction * (upper[1] - lower[1])
            fuel_flow = lower[2] + fraction * (upper[2] - lower[2])
            return thrust_n, fuel_flow
    return table[-1][1], table[-1][2]


def _clamped_altitude(altitude_m: float) -> float:
    return min(max(altitude_m, -999.0), 19_999.0)


def _installed_pulsejet_thrust_n(
    pulsejet_table: list[tuple[float, float, float]],
    mach: float,
    altitude_m: float,
) -> tuple[float, float]:
    """Scale the sea-level static table by local/sea-level density.

    Net thrust from a lumped filling/blowdown cycle scales approximately with
    captured mass flow, which is proportional to ambient density at fixed Mach and
    selector geometry. This is a first-order installed-altitude correction, not a
    re-run of the unsteady chamber model at altitude.
    """

    thrust_n, fuel_flow_kg_per_s = _interp_table(pulsejet_table, mach)
    density_ratio = (
        standard_atmosphere(_clamped_altitude(altitude_m)).density_kg_per_m3
        / _SEA_LEVEL_DENSITY_KG_PER_M3
    )
    return thrust_n * density_ratio, fuel_flow_kg_per_s * density_ratio


def _stall_speed_m_per_s(case: ReferenceCase, mass_kg: float, altitude_m: float) -> float:
    """Return the 1g stall speed at the current mass and altitude.

    ``V_stall = sqrt(2 W / (rho S CL_max))`` from steady level-flight lift balance
    (L = W at CL_max). Replaces a fixed zoom-exit speed with one that scales with
    the vehicle's actual instantaneous weight and local air density, matching
    standard practice (approach/maneuver speeds are always referenced to stall
    speed, not a fixed airspeed).
    """

    atmosphere = standard_atmosphere(_clamped_altitude(altitude_m))
    weight_n = mass_kg * G0_M_PER_S2
    denominator = (
        atmosphere.density_kg_per_m3
        * case.flight.reference_area_m2
        * case.flight.maximum_lift_coefficient
    )
    return sqrt(2.0 * weight_n / denominator)


def _best_glide_lift_coefficient(case: ReferenceCase, zero_lift_drag_area_m2: float) -> float:
    """Return the CL that maximizes L/D for the current parabolic-drag model."""

    zero_lift_drag_coefficient = zero_lift_drag_area_m2 / case.flight.reference_area_m2
    return sqrt(
        max(zero_lift_drag_coefficient, 1e-9) / case.flight.induced_drag_factor
    )


def simulate_mission(
    case: ReferenceCase,
    scenario: MissionScenario = NOMINAL_SCENARIO,
    *,
    time_step_s: float = 0.05,
    climb_angle_deg: float = 8.0,
    dive_angle_deg: float = -10.0,
    dive_entry_mach: float = 0.45,
    zoom_angle_deg: float = 25.0,
    zoom_exit_stall_margin_factor: float = 1.3,
    pull_out_load_factor_g: float = 4.0,
    max_time_s: float = 900.0,
    record_every_n_steps: int = 4,
    pulsejet_table_warmup_s: float = 0.25,
    pulsejet_table_measurement_s: float = 0.25,
    pulsejet_table_time_step_s: float = 0.00005,
) -> TrajectoryResult:
    """Integrate one phase-based mission from sled release to landing.

    Returns a full trajectory time history plus per-phase outcomes. Termination
    within a phase is event-based (fuel depletion, Mach target, altitude limit) per
    the mission requirement that speed-run duration is an output of the model.

    ``pull_out_load_factor_g`` is the assumed maximum structural/aerodynamic load
    factor available to recover from the dive; it sets the speed-dependent pull-out
    altitude margin via constant-load-factor circular-arc flight mechanics rather
    than a fixed altitude buffer. 4.0 g is a conservative placeholder pending a real
    structural limit from the mass/structure model.
    """

    if time_step_s <= 0.0:
        raise ValueError("time step must be positive")

    ramjet_recovery = (
        case.selector.ramjet_total_pressure_recovery
        if scenario.ramjet_total_pressure_recovery is None
        else scenario.ramjet_total_pressure_recovery
    )
    selector = replace(case.selector, ramjet_total_pressure_recovery=ramjet_recovery)

    loaded_mass_kg = case.flight.initial_mass_kg + scenario.mass_growth_kg
    pulsejet_fuel_kg = case.mission.loaded_fuel_mass_kg - case.mission.ramjet_speed_run_fuel_budget_kg
    ramjet_fuel_kg = case.mission.ramjet_speed_run_fuel_budget_kg
    if pulsejet_fuel_kg <= 0.0:
        raise ValueError("configured fuel allocation leaves no pulsejet-phase fuel")

    pulsejet_table = _pulsejet_static_thrust_table(
        case,
        scenario,
        (0.0, 0.10, 0.20, 0.30, 0.40, 0.50),
        warmup_s=pulsejet_table_warmup_s,
        measurement_s=pulsejet_table_measurement_s,
        time_step_s=pulsejet_table_time_step_s,
    )

    field_elevation_m = case.mission.field_elevation_msl_m
    top_of_climb_m = case.mission.top_of_climb_altitude_min_msl_m
    speed_run_altitude_m = case.mission.speed_run_altitude_msl_m
    peak_mach = case.mission.peak_mach
    lightoff_mach = case.ramjet.minimum_lightoff_test_mach

    t = 0.0
    altitude_m = field_elevation_m
    speed_m_per_s = case.mission.sled_release_speed_max_m_per_s
    downrange_m = 0.0
    mass_kg = loaded_mass_kg

    points: list[TrajectoryPoint] = []
    phases: list[PhaseOutcome] = []
    status: list[str] = []
    time_above_mach_one_s = 0.0
    peak_mach_reached = 0.0
    reached_peak_mach_target = False

    phase_name = "pulsejet_climb"
    phase_start_t = 0.0
    phase_start_altitude_m = altitude_m
    phase_start_mach = 0.0
    phase_fuel_start_kg = pulsejet_fuel_kg + ramjet_fuel_kg
    step_index = 0

    def close_phase(reason: str) -> None:
        atmosphere_now = standard_atmosphere(_clamped_altitude(altitude_m))
        phases.append(
            PhaseOutcome(
                name=phase_name,
                start_time_s=phase_start_t,
                end_time_s=t,
                start_altitude_m=phase_start_altitude_m,
                end_altitude_m=altitude_m,
                start_mach=phase_start_mach,
                end_mach=speed_m_per_s / atmosphere_now.speed_of_sound_m_per_s,
                fuel_used_kg=phase_fuel_start_kg - (pulsejet_fuel_kg + ramjet_fuel_kg),
                termination_reason=reason,
            )
        )

    aborted = False
    while t < max_time_s:
        atmosphere = standard_atmosphere(_clamped_altitude(altitude_m))
        mach = speed_m_per_s / atmosphere.speed_of_sound_m_per_s
        peak_mach_reached = max(peak_mach_reached, mach)
        if mach >= 1.0:
            time_above_mach_one_s += time_step_s

        gamma_deg = 0.0
        thrust_n = 0.0
        fuel_flow_kg_per_s = 0.0
        fuel_ledger = "none"

        if phase_name == "pulsejet_climb":
            gamma_deg = climb_angle_deg
            thrust_n, fuel_flow_kg_per_s = _installed_pulsejet_thrust_n(
                pulsejet_table, mach, altitude_m
            )
            fuel_ledger = "pulsejet"
            if mach >= dive_entry_mach:
                close_phase("reached_dive_entry_mach_during_climb")
                phase_name, phase_start_t = "dive", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach
            elif altitude_m >= top_of_climb_m:
                close_phase("reached_top_of_climb_altitude")
                phase_name, phase_start_t = "pulsejet_accel", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach
            elif pulsejet_fuel_kg <= 0.0:
                close_phase("pulsejet_fuel_exhausted_during_climb")
                status.append("pulsejet_fuel_exhausted_before_top_of_climb")
                aborted = True

        elif phase_name == "pulsejet_accel":
            gamma_deg = 0.0
            thrust_n, fuel_flow_kg_per_s = _installed_pulsejet_thrust_n(
                pulsejet_table, mach, altitude_m
            )
            fuel_ledger = "pulsejet"
            if mach >= dive_entry_mach or pulsejet_fuel_kg <= 0.0:
                reason = (
                    "reached_dive_entry_mach"
                    if mach >= dive_entry_mach
                    else "pulsejet_fuel_exhausted_during_acceleration"
                )
                close_phase(reason)
                phase_name, phase_start_t = "dive", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach

        elif phase_name == "dive":
            # Boom Supersonic Prize rule: the entire acceleration through Mach 0.8
            # to past Mach 1 must be flown level or climbing, with no altitude loss
            # (boomsupersonic.com/prize, requirements.transonic_regime_start_mach).
            # The dive must therefore level off strictly before that Mach, not at it.
            transonic_start_mach = case.requirements.transonic_regime_start_mach
            gamma_deg = dive_angle_deg if mach < transonic_start_mach else 0.0
            if pulsejet_fuel_kg > 0.0:
                thrust_n, fuel_flow_kg_per_s = _installed_pulsejet_thrust_n(
                    pulsejet_table, mach, altitude_m
                )
                fuel_ledger = "pulsejet"
            # Pull-out altitude margin from constant-load-factor circular-arc flight
            # mechanics (r = V^2/(g(n-1)), altitude lost = r(1-cos(gamma))), evaluated
            # at the current speed rather than a fixed distance -- a faster dive
            # needs proportionally more room to recover to level flight.
            load_factor_margin_g = max(pull_out_load_factor_g - 1.0, 0.1)
            pull_out_radius_m = speed_m_per_s**2 / (G0_M_PER_S2 * load_factor_margin_g)
            pull_out_altitude_loss_m = pull_out_radius_m * (
                1.0 - cos(radians(abs(dive_angle_deg)))
            )
            floor_m = field_elevation_m + pull_out_altitude_loss_m
            if mach >= lightoff_mach:
                close_phase("reached_ramjet_lightoff_mach")
                phase_name, phase_start_t = "ramjet_accel", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach
            elif altitude_m <= floor_m:
                close_phase("reached_dive_altitude_floor_below_lightoff_mach")
                status.append("dive_floor_reached_without_ramjet_lightoff_mach")
                aborted = True

        elif phase_name == "ramjet_accel":
            # Same no-altitude-loss rule: never allow a negative (descending)
            # flight-path angle once inside the regulated Mach 0.8+ regime.
            target_delta_m = speed_run_altitude_m - altitude_m
            gamma_deg = max(0.0, min(6.0, target_delta_m * 0.01))
            ramjet_nozzle = case.nozzle
            ramjet_result = evaluate_ramjet(
                case.ramjet, selector, ramjet_nozzle, case.fuel, altitude_m, mach
            )
            thrust_n = ramjet_result.net_thrust_n * scenario.thrust_multiplier
            fuel_flow_kg_per_s = ramjet_result.fuel_mass_flow_kg_per_s
            fuel_ledger = "ramjet"
            if mach >= peak_mach:
                close_phase("reached_peak_mach_target")
                reached_peak_mach_target = True
                phase_name, phase_start_t = "mach_hold", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach
            elif ramjet_fuel_kg <= 0.0:
                close_phase("ramjet_fuel_exhausted_before_peak_mach")
                status.append("ramjet_fuel_exhausted_before_reaching_peak_mach")
                aborted = True
            elif thrust_n <= 0.0 and mach < peak_mach:
                # Full-throttle net thrust cannot even hold speed at this scenario;
                # continuing would silently coast, so the run is explicitly stopped.
                close_phase("nonpositive_ramjet_net_thrust_cannot_accelerate")
                status.append("ramjet_net_thrust_nonpositive_during_acceleration")
                aborted = True

        elif phase_name == "mach_hold":
            gamma_deg = 0.0
            ramjet_result = evaluate_ramjet(
                case.ramjet, selector, case.nozzle, case.fuel, altitude_m, mach
            )
            full_throttle_thrust_n = ramjet_result.net_thrust_n * scenario.thrust_multiplier
            drag = evaluate_total_drag(
                case,
                case.flight,
                altitude_m,
                mach,
                lift_coefficient=0.0,
                drag_multiplier=scenario.drag_multiplier,
            )
            if full_throttle_thrust_n <= drag.total_drag_n or full_throttle_thrust_n <= 0.0:
                close_phase("insufficient_thrust_margin_to_hold_peak_mach")
                status.append("cannot_hold_peak_mach_under_scenario_multipliers")
                thrust_n = full_throttle_thrust_n
                fuel_flow_kg_per_s = ramjet_result.fuel_mass_flow_kg_per_s
                fuel_ledger = "ramjet"
                phase_name, phase_start_t = "zoom_climb", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach
            else:
                throttle_fraction = drag.total_drag_n / full_throttle_thrust_n
                thrust_n = drag.total_drag_n
                fuel_flow_kg_per_s = throttle_fraction * ramjet_result.fuel_mass_flow_kg_per_s
                fuel_ledger = "ramjet"
                if ramjet_fuel_kg <= 0.0:
                    close_phase("ramjet_fuel_exhausted_ending_mach_hold")
                    phase_name, phase_start_t = "zoom_climb", t
                    phase_start_altitude_m, phase_start_mach = altitude_m, mach

        elif phase_name == "zoom_climb":
            gamma_deg = zoom_angle_deg
            thrust_n = 0.0
            # Exit the zoom at a speed referenced to the actual (mass- and
            # altitude-dependent) stall speed, not one fixed airspeed for the whole
            # flight -- standard practice for maneuver/approach speed margins.
            zoom_exit_speed_m_per_s = zoom_exit_stall_margin_factor * _stall_speed_m_per_s(
                case, mass_kg, altitude_m
            )
            if speed_m_per_s <= zoom_exit_speed_m_per_s or altitude_m >= case.mission.top_of_climb_altitude_max_msl_m:
                close_phase("zoom_apex_or_altitude_cap_reached")
                phase_name, phase_start_t = "glide", t
                phase_start_altitude_m, phase_start_mach = altitude_m, mach

        elif phase_name == "glide":
            drag_probe = evaluate_total_drag(
                case, case.flight, altitude_m, max(mach, 1e-6), lift_coefficient=0.0
            )
            best_glide_cl = _best_glide_lift_coefficient(
                case, drag_probe.zero_lift_drag_area_m2
            )
            best_glide_ld = best_glide_cl / (
                drag_probe.zero_lift_drag_area_m2 / case.flight.reference_area_m2
                + case.flight.induced_drag_factor * best_glide_cl**2
            )
            gamma_deg = -atan(1.0 / max(best_glide_ld, 1e-6)) * 180.0 / 3.141592653589793
            thrust_n = 0.0
            if altitude_m <= field_elevation_m:
                close_phase("landed_at_field_elevation")
                break

        gamma_rad = radians(gamma_deg)
        drag = evaluate_total_drag(
            case,
            case.flight,
            altitude_m,
            mach,
            lift_coefficient=0.0,
            drag_multiplier=scenario.drag_multiplier,
        )
        net_axial_force_n = thrust_n - drag.total_drag_n - mass_kg * G0_M_PER_S2 * sin(gamma_rad)
        acceleration_m_per_s2 = net_axial_force_n / mass_kg

        if step_index % record_every_n_steps == 0:
            points.append(
                TrajectoryPoint(
                    time_s=t,
                    phase=phase_name,
                    altitude_m=altitude_m,
                    downrange_m=downrange_m,
                    speed_m_per_s=speed_m_per_s,
                    mach=mach,
                    flight_path_angle_deg=gamma_deg,
                    mass_kg=mass_kg,
                    pulsejet_fuel_remaining_kg=pulsejet_fuel_kg,
                    ramjet_fuel_remaining_kg=ramjet_fuel_kg,
                    thrust_n=thrust_n,
                    drag_n=drag.total_drag_n,
                    net_axial_force_n=net_axial_force_n,
                )
            )

        if aborted:
            break

        speed_m_per_s = max(speed_m_per_s + acceleration_m_per_s2 * time_step_s, 0.0)
        altitude_m += speed_m_per_s * sin(gamma_rad) * time_step_s
        downrange_m += speed_m_per_s * cos(gamma_rad) * time_step_s
        fuel_used_kg = fuel_flow_kg_per_s * time_step_s
        mass_kg = max(mass_kg - fuel_used_kg, 1e-3)
        if fuel_ledger == "pulsejet":
            pulsejet_fuel_kg = max(pulsejet_fuel_kg - fuel_used_kg, 0.0)
        elif fuel_ledger == "ramjet":
            ramjet_fuel_kg = max(ramjet_fuel_kg - fuel_used_kg, 0.0)

        t += time_step_s
        step_index += 1

        if altitude_m < -500.0:
            status.append("altitude_went_negative_mission_aborted")
            close_phase("altitude_excursion_below_ground")
            break
    else:
        status.append("mission_time_cap_reached_before_landing")
        close_phase("max_time_s_reached")

    meets_duration = time_above_mach_one_s >= case.requirements.minimum_time_above_mach_one_s
    if not reached_peak_mach_target:
        status.append("peak_mach_target_not_reached")
    if not meets_duration:
        status.append("time_above_mach_one_below_competition_minimum")
    landed = altitude_m <= field_elevation_m + 1.0
    if not landed:
        status.append("mission_did_not_reach_a_landing_altitude_state")

    # Boom Supersonic Prize rule audit: no altitude loss while accelerating through
    # the regulated Mach 0.8-to-past-Mach-1 regime (boomsupersonic.com/prize).
    transonic_start_mach = case.requirements.transonic_regime_start_mach
    regulated_phases = {"dive", "ramjet_accel", "mach_hold"}
    regulated_points = [
        point
        for point in points
        if point.mach >= transonic_start_mach and point.phase in regulated_phases
    ]
    rule_satisfied = all(
        later.altitude_m >= earlier.altitude_m - 1e-6
        for earlier, later in zip(regulated_points, regulated_points[1:])
    )
    if not rule_satisfied:
        status.append("transonic_no_altitude_loss_rule_violated")
    status.append("energy_state_model_no_lift_trim_stability_or_control_solved")

    return TrajectoryResult(
        scenario=scenario.name,
        thrust_multiplier=scenario.thrust_multiplier,
        drag_multiplier=scenario.drag_multiplier,
        ramjet_total_pressure_recovery=ramjet_recovery,
        loaded_mass_kg=loaded_mass_kg,
        points=tuple(points),
        phases=tuple(phases),
        peak_mach_reached=peak_mach_reached,
        reached_peak_mach_target=reached_peak_mach_target,
        time_above_mach_one_s=time_above_mach_one_s,
        minimum_time_above_mach_one_requirement_s=case.requirements.minimum_time_above_mach_one_s,
        meets_minimum_time_above_mach_one=meets_duration,
        landed_at_or_below_field_elevation=landed,
        transonic_no_altitude_loss_rule_satisfied=rule_satisfied,
        final_status=tuple(status),
    )
