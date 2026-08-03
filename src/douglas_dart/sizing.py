"""Transparent propulsion-geometry sizing trades for the handoff envelope."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import floor, sqrt

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .config import ReferenceCase
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet


@dataclass(frozen=True)
class RamjetHandoffSizingPoint:
    altitude_m: float
    mach: float
    intake_diameter_m: float
    nominal_body_diameter_m: float
    current_throat_diameter_m: float
    required_matched_throat_diameter_m: float
    required_throat_to_body_diameter_ratio: float
    required_throat_within_nominal_body: bool
    potential_captured_air_mass_flow_kg_per_s: float
    current_air_mass_flow_kg_per_s: float
    current_inlet_spillage_fraction: float
    current_net_thrust_n: float
    matched_net_thrust_n: float
    matched_fuel_mass_flow_kg_per_s: float
    matched_nozzle_mass_flow_residual_fraction: float
    matched_status: tuple[str, ...]
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class PeakMachDiameterTradePoint:
    """One outer-body packaging, drag-budget, and fuel-endurance trade point."""

    altitude_m: float
    peak_mach: float
    body_diameter_m: float
    geometrically_scaled_body_length_m: float
    fixed_intake_diameter_m: float
    flow_matched_throat_diameter_m: float
    available_radial_clearance_m: float
    throat_packageable_without_radial_allowance: bool
    geometrically_scaled_drag_area_target_m2: float
    target_drag_n: float
    full_throttle_ramjet_net_thrust_n: float
    full_throttle_ramjet_fuel_mass_flow_kg_per_s: float
    full_throttle_fuel_endurance_s: float | None
    thrust_margin_against_scaled_drag_target_n: float
    propulsion_supported_drag_area_m2: float
    drag_area_reduction_required_fraction: float
    can_hold_peak_mach_against_scaled_drag_target: bool
    peak_mach_hold_throttle_fraction: float | None
    fuel_limited_peak_mach_hold_duration_s: float | None
    fuel_limited_peak_mach_hold_distance_m: float | None
    ramjet_speed_run_fuel_budget_kg: float
    status: tuple[str, ...]
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class SharedNozzleTradePoint:
    """One fixed-nozzle point evaluated in both pulsejet and ramjet modes."""

    altitude_m: float
    peak_mach: float
    body_diameter_m: float
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    exit_diameter_m: float
    minimum_packaging_body_diameter_m: float
    available_minimum_radial_packaging_margin_m: float
    packageable_with_configured_allowances: bool
    drag_area_budget_m2: float
    drag_at_peak_mach_n: float
    ramjet_gross_thrust_n: float
    ramjet_net_thrust_n: float
    ramjet_derated_net_thrust_n: float
    ramjet_nominal_thrust_margin_n: float
    ramjet_derated_thrust_margin_n: float
    ramjet_inlet_spillage_fraction: float
    ramjet_air_mass_flow_kg_per_s: float
    ramjet_fuel_mass_flow_kg_per_s: float
    ramjet_full_throttle_fuel_endurance_s: float | None
    ramjet_hold_throttle_fraction_with_derate: float | None
    ramjet_fuel_limited_hold_duration_s: float | None
    ramjet_fuel_limited_hold_distance_m: float | None
    pulsejet_mean_gross_thrust_n: float
    pulsejet_mean_net_thrust_n: float
    pulsejet_peak_net_thrust_n: float
    pulsejet_mean_fuel_mass_flow_kg_per_s: float
    pulsejet_peak_chamber_pressure_pa: float
    pulsejet_peak_chamber_temperature_k: float
    pulsejet_completed_cycles: int
    pulsejet_warmup_duration_s: float
    pulsejet_measurement_duration_s: float
    pulsejet_mean_net_thrust_to_weight: float
    loaded_mass_margin_to_requirement_kg: float
    propulsion_derate_fraction: float
    can_hold_peak_mach_nominal: bool
    can_hold_peak_mach_with_derate: bool
    static_fuel_hold_exceeds_minimum_supersonic_duration: bool
    configured_loaded_mass_within_requirement: bool
    status: tuple[str, ...]
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class SharedNozzleFeasibilityBounds:
    """Local fixed-architecture bounds around a configured shared nozzle."""

    altitude_m: float
    peak_mach: float
    fixed_exit_to_throat_area_ratio: float
    propulsion_derate_fraction: float
    minimum_packageable_body_diameter_m: float
    maximum_body_diameter_for_derated_drag_budget_m: float
    configured_body_diameter_m: float
    configured_body_margin_above_packaging_minimum_m: float
    configured_body_margin_below_drag_maximum_m: float
    minimum_throat_diameter_for_derated_drag_budget_m: float | None
    configured_throat_diameter_m: float
    configured_throat_margin_above_thrust_minimum_m: float | None
    fixed_architecture_has_body_feasibility_interval: bool
    status: tuple[str, ...]
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class PeakMachAltitudeTradePoint:
    """One static peak-Mach propulsion/drag/fuel point at a candidate altitude."""

    altitude_m: float
    peak_mach: float
    true_airspeed_m_per_s: float
    dynamic_pressure_pa: float
    drag_area_budget_m2: float
    drag_n: float
    ramjet_net_thrust_n: float
    ramjet_derated_net_thrust_n: float
    ramjet_derated_thrust_margin_n: float
    ramjet_inlet_spillage_fraction: float
    ramjet_fuel_mass_flow_kg_per_s: float
    ramjet_hold_throttle_fraction_with_derate: float | None
    ramjet_fuel_limited_hold_duration_s: float | None
    ramjet_fuel_limited_hold_distance_m: float | None
    can_hold_peak_mach_with_derate: bool
    status: tuple[str, ...]
    numerical_reference_only: bool = True


def evaluate_ramjet_handoff_sizing(
    case: ReferenceCase,
    altitude_m: float,
    mach: float,
) -> RamjetHandoffSizingPoint:
    """Size the throat that would pass all potential captured flow at one point.

    For fixed total conditions and exit/throat area ratio, nozzle capacity is
    proportional to throat area. The resulting diameter is therefore an exact
    low-order flow-match for this model, not a claim that the geometry is physically
    packageable or that the inlet/combustor will remain stable.
    """

    current = evaluate_ramjet(
        case.ramjet,
        case.selector,
        case.nozzle,
        case.fuel,
        altitude_m,
        mach,
    )
    demanded_nozzle_mass_flow_kg_per_s = (
        current.potential_captured_air_mass_flow_kg_per_s * (1.0 + current.fuel_air_ratio)
    )
    if current.nozzle_capacity_kg_per_s <= 0.0:
        required_throat_diameter_m = float("inf")
        matched = current
    else:
        required_throat_diameter_m = case.nozzle.throat_diameter_m * sqrt(
            demanded_nozzle_mass_flow_kg_per_s / current.nozzle_capacity_kg_per_s
        )
        matched_nozzle = replace(
            case.nozzle,
            throat_diameter_m=required_throat_diameter_m,
        )
        matched = evaluate_ramjet(
            case.ramjet,
            case.selector,
            matched_nozzle,
            case.fuel,
            altitude_m,
            mach,
        )

    body_diameter_m = case.vehicle.body_diameter_m
    throat_to_body_ratio = required_throat_diameter_m / body_diameter_m
    return RamjetHandoffSizingPoint(
        altitude_m=altitude_m,
        mach=mach,
        intake_diameter_m=case.selector.circular_intake_diameter_m,
        nominal_body_diameter_m=body_diameter_m,
        current_throat_diameter_m=case.nozzle.throat_diameter_m,
        required_matched_throat_diameter_m=required_throat_diameter_m,
        required_throat_to_body_diameter_ratio=throat_to_body_ratio,
        required_throat_within_nominal_body=required_throat_diameter_m <= body_diameter_m,
        potential_captured_air_mass_flow_kg_per_s=(
            current.potential_captured_air_mass_flow_kg_per_s
        ),
        current_air_mass_flow_kg_per_s=current.air_mass_flow_kg_per_s,
        current_inlet_spillage_fraction=current.inlet_spillage_fraction,
        current_net_thrust_n=current.net_thrust_n,
        matched_net_thrust_n=matched.net_thrust_n,
        matched_fuel_mass_flow_kg_per_s=matched.fuel_mass_flow_kg_per_s,
        matched_nozzle_mass_flow_residual_fraction=(
            matched.nozzle_mass_flow_residual_fraction
        ),
        matched_status=matched.status,
    )


def ramjet_handoff_sweep(
    case: ReferenceCase,
    altitude_m: float,
    minimum_mach: float,
    maximum_mach: float,
    mach_step: float,
) -> list[RamjetHandoffSizingPoint]:
    if altitude_m < 0.0:
        raise ValueError("altitude cannot be negative")
    if minimum_mach < 0.0 or maximum_mach < minimum_mach:
        raise ValueError("Mach bounds are invalid")
    if mach_step <= 0.0:
        raise ValueError("Mach step must be positive")

    point_count = floor((maximum_mach - minimum_mach) / mach_step + 1e-10) + 1
    points: list[RamjetHandoffSizingPoint] = []
    for index in range(point_count):
        mach = minimum_mach + index * mach_step
        points.append(evaluate_ramjet_handoff_sizing(case, altitude_m, mach))
    return points


def geometrically_scaled_drag_area_target_m2(
    case: ReferenceCase,
    body_diameter_m: float,
) -> float:
    """Scale the prior drag-area ceiling for a geometrically similar outer mold line.

    Drag area has units of area. Holding fineness ratio, surface proportions, and
    aerodynamic coefficients fixed therefore produces a diameter-squared scaling.
    This is a transparent design-budget proxy until VSPAERO supplies actual tables.
    """

    if body_diameter_m <= 0.0:
        raise ValueError("body diameter must be positive")
    diameter_ratio = (
        body_diameter_m / case.vehicle.drag_area_reference_body_diameter_m
    )
    return case.vehicle.peak_mach_drag_area_ceiling_m2 * diameter_ratio**2


def evaluate_peak_mach_diameter_trade(
    case: ReferenceCase,
    body_diameter_m: float,
) -> PeakMachDiameterTradePoint:
    """Trade outer-body packaging against peak-Mach drag and fuel endurance.

    The 195 mm selector intake remains fixed while only the outer mold line changes.
    The nozzle is resized to the model's full-capture flow-match point. Packaging is
    a zero-clearance lower bound; wall thickness, insulation, structure, and selector
    hardware will require a larger real body.
    """

    intake_diameter_m = case.selector.circular_intake_diameter_m
    if body_diameter_m < intake_diameter_m:
        raise ValueError("outer body diameter cannot be smaller than the intake diameter")

    altitude_m = case.mission.speed_run_altitude_msl_m
    peak_mach = case.mission.peak_mach
    handoff = evaluate_ramjet_handoff_sizing(case, altitude_m, peak_mach)
    matched_nozzle = replace(
        case.nozzle,
        throat_diameter_m=handoff.required_matched_throat_diameter_m,
    )
    ramjet = evaluate_ramjet(
        case.ramjet,
        case.selector,
        matched_nozzle,
        case.fuel,
        altitude_m,
        peak_mach,
    )

    atmosphere = standard_atmosphere(altitude_m)
    speed_m_per_s = peak_mach * atmosphere.speed_of_sound_m_per_s
    dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * speed_m_per_s**2
    drag_area_m2 = geometrically_scaled_drag_area_target_m2(case, body_diameter_m)
    target_drag_n = dynamic_pressure_pa * drag_area_m2
    thrust_n = ramjet.net_thrust_n
    thrust_margin_n = thrust_n - target_drag_n
    supported_drag_area_m2 = max(thrust_n, 0.0) / dynamic_pressure_pa
    drag_reduction_fraction = max(
        0.0,
        1.0 - supported_drag_area_m2 / drag_area_m2,
    )
    packageable = body_diameter_m >= handoff.required_matched_throat_diameter_m
    radial_clearance_m = 0.5 * (
        body_diameter_m - handoff.required_matched_throat_diameter_m
    )
    full_throttle_fuel_endurance_s = (
        case.mission.ramjet_speed_run_fuel_budget_kg
        / ramjet.fuel_mass_flow_kg_per_s
        if ramjet.fuel_mass_flow_kg_per_s > 0.0
        else None
    )
    can_hold = (
        packageable
        and ramjet.self_sustaining_candidate
        and thrust_margin_n >= 0.0
        and ramjet.fuel_mass_flow_kg_per_s > 0.0
    )

    throttle_fraction: float | None = None
    hold_duration_s: float | None = None
    hold_distance_m: float | None = None
    if can_hold:
        # Linear thrust/fuel scaling is a low-order hold estimate, not a combustor
        # turndown model. The duration is an output of the allocated fuel mass.
        throttle_fraction = target_drag_n / thrust_n
        hold_fuel_flow_kg_per_s = (
            throttle_fraction * ramjet.fuel_mass_flow_kg_per_s
        )
        hold_duration_s = (
            case.mission.ramjet_speed_run_fuel_budget_kg
            / hold_fuel_flow_kg_per_s
        )
        hold_distance_m = speed_m_per_s * hold_duration_s

    status = list(ramjet.status)
    if not packageable:
        status.append("flow_matched_throat_exceeds_outer_body_zero_clearance")
    if thrust_margin_n < 0.0:
        status.append("scaled_drag_target_exceeds_full_throttle_ramjet_thrust")
    if can_hold:
        status.append("fuel_limited_hold_uses_linear_throttle_scaling")

    body_scale = body_diameter_m / case.vehicle.body_diameter_m
    return PeakMachDiameterTradePoint(
        altitude_m=altitude_m,
        peak_mach=peak_mach,
        body_diameter_m=body_diameter_m,
        geometrically_scaled_body_length_m=case.vehicle.body_length_m * body_scale,
        fixed_intake_diameter_m=intake_diameter_m,
        flow_matched_throat_diameter_m=handoff.required_matched_throat_diameter_m,
        available_radial_clearance_m=radial_clearance_m,
        throat_packageable_without_radial_allowance=packageable,
        geometrically_scaled_drag_area_target_m2=drag_area_m2,
        target_drag_n=target_drag_n,
        full_throttle_ramjet_net_thrust_n=thrust_n,
        full_throttle_ramjet_fuel_mass_flow_kg_per_s=ramjet.fuel_mass_flow_kg_per_s,
        full_throttle_fuel_endurance_s=full_throttle_fuel_endurance_s,
        thrust_margin_against_scaled_drag_target_n=thrust_margin_n,
        propulsion_supported_drag_area_m2=supported_drag_area_m2,
        drag_area_reduction_required_fraction=drag_reduction_fraction,
        can_hold_peak_mach_against_scaled_drag_target=can_hold,
        peak_mach_hold_throttle_fraction=throttle_fraction,
        fuel_limited_peak_mach_hold_duration_s=hold_duration_s,
        fuel_limited_peak_mach_hold_distance_m=hold_distance_m,
        ramjet_speed_run_fuel_budget_kg=(
            case.mission.ramjet_speed_run_fuel_budget_kg
        ),
        status=tuple(status),
    )


def peak_mach_diameter_trade_sweep(
    case: ReferenceCase,
    minimum_body_diameter_m: float,
    maximum_body_diameter_m: float,
    body_diameter_step_m: float,
) -> list[PeakMachDiameterTradePoint]:
    if minimum_body_diameter_m < case.selector.circular_intake_diameter_m:
        raise ValueError("minimum outer body diameter cannot be smaller than the intake")
    if maximum_body_diameter_m < minimum_body_diameter_m:
        raise ValueError("body diameter bounds are invalid")
    if body_diameter_step_m <= 0.0:
        raise ValueError("body diameter step must be positive")

    point_count = (
        floor(
            (maximum_body_diameter_m - minimum_body_diameter_m)
            / body_diameter_step_m
            + 1e-10
        )
        + 1
    )
    return [
        evaluate_peak_mach_diameter_trade(
            case,
            minimum_body_diameter_m + index * body_diameter_step_m,
        )
        for index in range(point_count)
    ]


def evaluate_shared_nozzle_trade(
    case: ReferenceCase,
    body_diameter_m: float,
    throat_diameter_m: float,
    exit_to_throat_area_ratio: float,
    *,
    propulsion_derate_fraction: float = 0.15,
    pulsejet_warmup_s: float | None = None,
    pulsejet_measurement_s: float | None = None,
    pulsejet_time_step_s: float | None = None,
) -> SharedNozzleTradePoint:
    """Evaluate one fixed throat/exit geometry in both propulsion modes.

    The ramjet is allowed to spill potential capture when its fixed nozzle is the
    limiting area. The configured drag-area ceiling remains a budget rather than an
    aerodynamic prediction. ``propulsion_derate_fraction`` is an explicit reserve
    applied to ramjet net thrust; it is not hidden inside a component efficiency.
    """

    if body_diameter_m < case.selector.circular_intake_diameter_m:
        raise ValueError("body diameter cannot be smaller than the circular intake")
    if throat_diameter_m <= 0.0:
        raise ValueError("throat diameter must be positive")
    if exit_to_throat_area_ratio < 1.0:
        raise ValueError("exit-to-throat area ratio must be at least one")
    if not 0.0 <= propulsion_derate_fraction < 1.0:
        raise ValueError("propulsion derate must be in [0, 1)")

    nozzle = replace(
        case.nozzle,
        throat_diameter_m=throat_diameter_m,
        exit_to_throat_area_ratio=exit_to_throat_area_ratio,
    )
    altitude_m = case.mission.speed_run_altitude_msl_m
    peak_mach = case.mission.peak_mach
    ramjet = evaluate_ramjet(
        case.ramjet,
        case.selector,
        nozzle,
        case.fuel,
        altitude_m,
        peak_mach,
    )

    exit_diameter_m = throat_diameter_m * sqrt(exit_to_throat_area_ratio)
    selector_package_diameter_m = (
        case.selector.circular_intake_diameter_m
        + 2.0 * case.geometry.selector_radial_allowance_m
    )
    nozzle_package_diameter_m = (
        exit_diameter_m + 2.0 * case.geometry.nozzle_radial_allowance_m
    )
    minimum_body_diameter_m = max(
        selector_package_diameter_m,
        nozzle_package_diameter_m,
    )
    packageable = body_diameter_m + 1e-12 >= minimum_body_diameter_m
    packaging_margin_m = 0.5 * (body_diameter_m - minimum_body_diameter_m)
    if packageable and packaging_margin_m < 0.0:
        packaging_margin_m = 0.0

    atmosphere = standard_atmosphere(altitude_m)
    speed_m_per_s = peak_mach * atmosphere.speed_of_sound_m_per_s
    dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * speed_m_per_s**2
    drag_area_budget_m2 = geometrically_scaled_drag_area_target_m2(
        case,
        body_diameter_m,
    )
    drag_n = dynamic_pressure_pa * drag_area_budget_m2
    derated_thrust_n = (1.0 - propulsion_derate_fraction) * ramjet.net_thrust_n
    nominal_margin_n = ramjet.net_thrust_n - drag_n
    derated_margin_n = derated_thrust_n - drag_n

    full_endurance_s = (
        case.mission.ramjet_speed_run_fuel_budget_kg / ramjet.fuel_mass_flow_kg_per_s
        if ramjet.fuel_mass_flow_kg_per_s > 0.0
        else None
    )
    can_hold_nominal = (
        packageable
        and ramjet.self_sustaining_candidate
        and nominal_margin_n >= 0.0
        and ramjet.fuel_mass_flow_kg_per_s > 0.0
    )
    can_hold_derated = can_hold_nominal and derated_margin_n >= 0.0
    hold_throttle_fraction: float | None = None
    hold_duration_s: float | None = None
    hold_distance_m: float | None = None
    if can_hold_derated:
        hold_throttle_fraction = drag_n / derated_thrust_n
        hold_fuel_flow_kg_per_s = (
            hold_throttle_fraction * ramjet.fuel_mass_flow_kg_per_s
        )
        hold_duration_s = (
            case.mission.ramjet_speed_run_fuel_budget_kg
            / hold_fuel_flow_kg_per_s
        )
        hold_distance_m = speed_m_per_s * hold_duration_s

    warmup_s = (
        case.simulation.pulsejet_steady_warmup_s
        if pulsejet_warmup_s is None
        else pulsejet_warmup_s
    )
    measurement_s = (
        case.simulation.pulsejet_steady_measurement_s
        if pulsejet_measurement_s is None
        else pulsejet_measurement_s
    )
    if warmup_s <= 0.0 or measurement_s <= 0.0:
        raise ValueError("pulsejet warmup and measurement durations must be positive")
    time_step_s = (
        case.simulation.time_step_s
        if pulsejet_time_step_s is None
        else pulsejet_time_step_s
    )
    pulsejet = PulsejetSimulator(
        case.pulsejet,
        case.selector,
        nozzle,
        case.fuel,
        case.altitude_m,
        case.mach,
    )
    pulsejet_summary = summarize_pulsejet(
        pulsejet.run(warmup_s + measurement_s, time_step_s),
        minimum_time_s=warmup_s,
    )
    pulsejet_thrust_to_weight = pulsejet_summary.mean_net_thrust_n / (
        case.flight.initial_mass_kg * G0_M_PER_S2
    )

    mass_margin_kg = (
        case.requirements.maximum_takeoff_mass_kg - case.flight.initial_mass_kg
    )
    mass_pass = mass_margin_kg >= 0.0
    duration_pass = (
        hold_duration_s is not None
        and hold_duration_s >= case.requirements.minimum_time_above_mach_one_s
    )
    status = list(ramjet.status)
    if not packageable:
        status.append("configured_radial_hardware_allowances_do_not_package")
    if nominal_margin_n < 0.0:
        status.append("drag_area_budget_exceeds_nominal_ramjet_thrust")
    elif derated_margin_n < 0.0:
        status.append("propulsion_reserve_not_closed")
    if hold_duration_s is not None:
        status.append("fuel_hold_uses_linear_thrust_fuel_scaling")
    if not duration_pass:
        status.append(
            "static_fuel_hold_does_not_exceed_minimum_supersonic_duration"
        )
    if not mass_pass:
        status.append("configured_loaded_mass_exceeds_requirement")
    if pulsejet_summary.mean_net_thrust_n <= 0.0:
        status.append("pulsejet_mean_net_thrust_nonpositive")
    status.append("pulsejet_statistics_exclude_configured_startup_transient")

    return SharedNozzleTradePoint(
        altitude_m=altitude_m,
        peak_mach=peak_mach,
        body_diameter_m=body_diameter_m,
        throat_diameter_m=throat_diameter_m,
        exit_to_throat_area_ratio=exit_to_throat_area_ratio,
        exit_diameter_m=exit_diameter_m,
        minimum_packaging_body_diameter_m=minimum_body_diameter_m,
        available_minimum_radial_packaging_margin_m=packaging_margin_m,
        packageable_with_configured_allowances=packageable,
        drag_area_budget_m2=drag_area_budget_m2,
        drag_at_peak_mach_n=drag_n,
        ramjet_gross_thrust_n=ramjet.gross_thrust_n,
        ramjet_net_thrust_n=ramjet.net_thrust_n,
        ramjet_derated_net_thrust_n=derated_thrust_n,
        ramjet_nominal_thrust_margin_n=nominal_margin_n,
        ramjet_derated_thrust_margin_n=derated_margin_n,
        ramjet_inlet_spillage_fraction=ramjet.inlet_spillage_fraction,
        ramjet_air_mass_flow_kg_per_s=ramjet.air_mass_flow_kg_per_s,
        ramjet_fuel_mass_flow_kg_per_s=ramjet.fuel_mass_flow_kg_per_s,
        ramjet_full_throttle_fuel_endurance_s=full_endurance_s,
        ramjet_hold_throttle_fraction_with_derate=hold_throttle_fraction,
        ramjet_fuel_limited_hold_duration_s=hold_duration_s,
        ramjet_fuel_limited_hold_distance_m=hold_distance_m,
        pulsejet_mean_gross_thrust_n=pulsejet_summary.mean_gross_thrust_n,
        pulsejet_mean_net_thrust_n=pulsejet_summary.mean_net_thrust_n,
        pulsejet_peak_net_thrust_n=pulsejet_summary.peak_net_thrust_n,
        pulsejet_mean_fuel_mass_flow_kg_per_s=(
            pulsejet_summary.mean_fuel_mass_flow_kg_per_s
        ),
        pulsejet_peak_chamber_pressure_pa=(
            pulsejet_summary.peak_chamber_pressure_pa
        ),
        pulsejet_peak_chamber_temperature_k=(
            pulsejet_summary.peak_chamber_temperature_k
        ),
        pulsejet_completed_cycles=pulsejet_summary.completed_cycles,
        pulsejet_warmup_duration_s=warmup_s,
        pulsejet_measurement_duration_s=measurement_s,
        pulsejet_mean_net_thrust_to_weight=pulsejet_thrust_to_weight,
        loaded_mass_margin_to_requirement_kg=mass_margin_kg,
        propulsion_derate_fraction=propulsion_derate_fraction,
        can_hold_peak_mach_nominal=can_hold_nominal,
        can_hold_peak_mach_with_derate=can_hold_derated,
        static_fuel_hold_exceeds_minimum_supersonic_duration=duration_pass,
        configured_loaded_mass_within_requirement=mass_pass,
        status=tuple(status),
    )


def shared_nozzle_trade_sweep(
    case: ReferenceCase,
    body_diameters_m: list[float] | tuple[float, ...],
    throat_diameters_m: list[float] | tuple[float, ...],
    exit_to_throat_area_ratios: list[float] | tuple[float, ...],
    *,
    propulsion_derate_fraction: float = 0.15,
    pulsejet_warmup_s: float | None = None,
    pulsejet_measurement_s: float | None = None,
    pulsejet_time_step_s: float | None = None,
) -> list[SharedNozzleTradePoint]:
    if not body_diameters_m or not throat_diameters_m or not exit_to_throat_area_ratios:
        raise ValueError("shared-nozzle sweep arrays cannot be empty")
    return [
        evaluate_shared_nozzle_trade(
            case,
            body_diameter_m,
            throat_diameter_m,
            area_ratio,
            propulsion_derate_fraction=propulsion_derate_fraction,
            pulsejet_warmup_s=pulsejet_warmup_s,
            pulsejet_measurement_s=pulsejet_measurement_s,
            pulsejet_time_step_s=pulsejet_time_step_s,
        )
        for body_diameter_m in body_diameters_m
        for throat_diameter_m in throat_diameters_m
        for area_ratio in exit_to_throat_area_ratios
    ]


def select_minimum_feasible_shared_nozzle(
    points: list[SharedNozzleTradePoint],
) -> SharedNozzleTradePoint | None:
    """Select the smallest feasible body, then throat, then expansion ratio.

    This explicit lexicographic rule avoids a hidden weighted score. It should only
    be applied to a sweep whose lower expansion-ratio bound already reflects the
    desired fixed C-D architecture.
    """

    feasible = [
        point
        for point in points
        if point.packageable_with_configured_allowances
        and point.can_hold_peak_mach_with_derate
        and point.static_fuel_hold_exceeds_minimum_supersonic_duration
        and point.configured_loaded_mass_within_requirement
        and point.pulsejet_mean_net_thrust_n > 0.0
    ]
    if not feasible:
        return None
    return min(
        feasible,
        key=lambda point: (
            point.body_diameter_m,
            point.throat_diameter_m,
            point.exit_to_throat_area_ratio,
        ),
    )


def shared_nozzle_feasibility_bounds(
    case: ReferenceCase,
    *,
    propulsion_derate_fraction: float = 0.15,
    throat_root_tolerance_m: float = 1e-7,
) -> SharedNozzleFeasibilityBounds:
    """Find the local body and throat interval for the configured architecture.

    Body drag follows the explicitly configured diameter-squared budget proxy. The
    minimum throat is solved by repeatedly evaluating the ramjet rather than assuming
    thrust scales with throat area. These are local model bounds at one altitude,
    Mach, expansion ratio, and derate—not manufacturing tolerances or validated limits.
    """

    if not 0.0 <= propulsion_derate_fraction < 1.0:
        raise ValueError("propulsion derate must be in [0, 1)")
    if throat_root_tolerance_m <= 0.0:
        raise ValueError("throat root tolerance must be positive")

    altitude_m = case.mission.speed_run_altitude_msl_m
    peak_mach = case.mission.peak_mach
    atmosphere = standard_atmosphere(altitude_m)
    speed_m_per_s = peak_mach * atmosphere.speed_of_sound_m_per_s
    dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * speed_m_per_s**2

    configured_ramjet = evaluate_ramjet(
        case.ramjet,
        case.selector,
        case.nozzle,
        case.fuel,
        altitude_m,
        peak_mach,
    )
    derated_configured_thrust_n = (
        1.0 - propulsion_derate_fraction
    ) * configured_ramjet.net_thrust_n
    maximum_body_diameter_m = (
        case.vehicle.drag_area_reference_body_diameter_m
        * sqrt(
            max(derated_configured_thrust_n, 0.0)
            / (
                dynamic_pressure_pa
                * case.vehicle.peak_mach_drag_area_ceiling_m2
            )
        )
    )

    configured_exit_diameter_m = case.nozzle.throat_diameter_m * sqrt(
        case.nozzle.exit_to_throat_area_ratio
    )
    minimum_body_diameter_m = max(
        case.selector.circular_intake_diameter_m
        + 2.0 * case.geometry.selector_radial_allowance_m,
        configured_exit_diameter_m
        + 2.0 * case.geometry.nozzle_radial_allowance_m,
    )

    configured_drag_n = dynamic_pressure_pa * geometrically_scaled_drag_area_target_m2(
        case,
        case.vehicle.body_diameter_m,
    )

    def derated_margin_n(throat_diameter_m: float) -> float:
        nozzle = replace(case.nozzle, throat_diameter_m=throat_diameter_m)
        ramjet = evaluate_ramjet(
            case.ramjet,
            case.selector,
            nozzle,
            case.fuel,
            altitude_m,
            peak_mach,
        )
        return (
            (1.0 - propulsion_derate_fraction) * ramjet.net_thrust_n
            - configured_drag_n
        )

    upper_throat_m = case.nozzle.throat_diameter_m
    minimum_throat_m: float | None = None
    if derated_margin_n(upper_throat_m) >= 0.0:
        lower_throat_m = min(0.001, 0.01 * upper_throat_m)
        if derated_margin_n(lower_throat_m) <= 0.0:
            while upper_throat_m - lower_throat_m > throat_root_tolerance_m:
                midpoint_m = 0.5 * (lower_throat_m + upper_throat_m)
                if derated_margin_n(midpoint_m) >= 0.0:
                    upper_throat_m = midpoint_m
                else:
                    lower_throat_m = midpoint_m
            minimum_throat_m = upper_throat_m

    status = list(configured_ramjet.status)
    body_interval_exists = minimum_body_diameter_m <= maximum_body_diameter_m
    body_packaging_margin_m = (
        case.vehicle.body_diameter_m - minimum_body_diameter_m
    )
    # Remove binary floating-point noise at an exactly coincident boundary.
    if abs(body_packaging_margin_m) < 1e-12:
        body_packaging_margin_m = 0.0
    if not body_interval_exists:
        status.append("fixed_nozzle_has_no_body_interval_between_packaging_and_drag")
    if minimum_throat_m is None:
        status.append("configured_throat_does_not_bound_a_derated_thrust_root")
    status.append("bounds_hold_altitude_mach_area_ratio_and_drag_proxy_fixed")

    return SharedNozzleFeasibilityBounds(
        altitude_m=altitude_m,
        peak_mach=peak_mach,
        fixed_exit_to_throat_area_ratio=case.nozzle.exit_to_throat_area_ratio,
        propulsion_derate_fraction=propulsion_derate_fraction,
        minimum_packageable_body_diameter_m=minimum_body_diameter_m,
        maximum_body_diameter_for_derated_drag_budget_m=maximum_body_diameter_m,
        configured_body_diameter_m=case.vehicle.body_diameter_m,
        configured_body_margin_above_packaging_minimum_m=body_packaging_margin_m,
        configured_body_margin_below_drag_maximum_m=(
            maximum_body_diameter_m - case.vehicle.body_diameter_m
        ),
        minimum_throat_diameter_for_derated_drag_budget_m=minimum_throat_m,
        configured_throat_diameter_m=case.nozzle.throat_diameter_m,
        configured_throat_margin_above_thrust_minimum_m=(
            case.nozzle.throat_diameter_m - minimum_throat_m
            if minimum_throat_m is not None
            else None
        ),
        fixed_architecture_has_body_feasibility_interval=body_interval_exists,
        status=tuple(status),
    )


def evaluate_peak_mach_altitude_trade(
    case: ReferenceCase,
    altitude_m: float,
    *,
    propulsion_derate_fraction: float = 0.15,
) -> PeakMachAltitudeTradePoint:
    """Evaluate the configured fixed nozzle at peak Mach and one altitude."""

    if altitude_m < 0.0:
        raise ValueError("altitude cannot be negative")
    if not 0.0 <= propulsion_derate_fraction < 1.0:
        raise ValueError("propulsion derate must be in [0, 1)")

    peak_mach = case.mission.peak_mach
    atmosphere = standard_atmosphere(altitude_m)
    true_airspeed_m_per_s = peak_mach * atmosphere.speed_of_sound_m_per_s
    dynamic_pressure_pa = (
        0.5 * atmosphere.density_kg_per_m3 * true_airspeed_m_per_s**2
    )
    drag_area_m2 = geometrically_scaled_drag_area_target_m2(
        case,
        case.vehicle.body_diameter_m,
    )
    drag_n = dynamic_pressure_pa * drag_area_m2
    ramjet = evaluate_ramjet(
        case.ramjet,
        case.selector,
        case.nozzle,
        case.fuel,
        altitude_m,
        peak_mach,
    )
    derated_thrust_n = (1.0 - propulsion_derate_fraction) * ramjet.net_thrust_n
    derated_margin_n = derated_thrust_n - drag_n
    can_hold = (
        ramjet.self_sustaining_candidate
        and ramjet.fuel_mass_flow_kg_per_s > 0.0
        and derated_margin_n >= 0.0
    )
    throttle_fraction: float | None = None
    hold_duration_s: float | None = None
    hold_distance_m: float | None = None
    if can_hold:
        throttle_fraction = drag_n / derated_thrust_n
        hold_fuel_flow_kg_per_s = (
            throttle_fraction * ramjet.fuel_mass_flow_kg_per_s
        )
        hold_duration_s = (
            case.mission.ramjet_speed_run_fuel_budget_kg
            / hold_fuel_flow_kg_per_s
        )
        hold_distance_m = true_airspeed_m_per_s * hold_duration_s

    status = list(ramjet.status)
    status.append("static_altitude_trade_excludes_climb_and_acceleration_energy")
    if hold_duration_s is not None:
        status.append("fuel_hold_uses_linear_thrust_fuel_scaling")
    else:
        status.append("derated_thrust_does_not_close_drag_budget_at_altitude")

    return PeakMachAltitudeTradePoint(
        altitude_m=altitude_m,
        peak_mach=peak_mach,
        true_airspeed_m_per_s=true_airspeed_m_per_s,
        dynamic_pressure_pa=dynamic_pressure_pa,
        drag_area_budget_m2=drag_area_m2,
        drag_n=drag_n,
        ramjet_net_thrust_n=ramjet.net_thrust_n,
        ramjet_derated_net_thrust_n=derated_thrust_n,
        ramjet_derated_thrust_margin_n=derated_margin_n,
        ramjet_inlet_spillage_fraction=ramjet.inlet_spillage_fraction,
        ramjet_fuel_mass_flow_kg_per_s=ramjet.fuel_mass_flow_kg_per_s,
        ramjet_hold_throttle_fraction_with_derate=throttle_fraction,
        ramjet_fuel_limited_hold_duration_s=hold_duration_s,
        ramjet_fuel_limited_hold_distance_m=hold_distance_m,
        can_hold_peak_mach_with_derate=can_hold,
        status=tuple(status),
    )


def peak_mach_altitude_trade_sweep(
    case: ReferenceCase,
    minimum_altitude_m: float,
    maximum_altitude_m: float,
    altitude_step_m: float,
    *,
    propulsion_derate_fraction: float = 0.15,
) -> list[PeakMachAltitudeTradePoint]:
    if minimum_altitude_m < 0.0 or maximum_altitude_m < minimum_altitude_m:
        raise ValueError("altitude bounds are invalid")
    if altitude_step_m <= 0.0:
        raise ValueError("altitude step must be positive")
    point_count = (
        floor((maximum_altitude_m - minimum_altitude_m) / altitude_step_m + 1e-10)
        + 1
    )
    return [
        evaluate_peak_mach_altitude_trade(
            case,
            minimum_altitude_m + index * altitude_step_m,
            propulsion_derate_fraction=propulsion_derate_fraction,
        )
        for index in range(point_count)
    ]
