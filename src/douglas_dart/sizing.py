"""Transparent propulsion-geometry sizing trades for the handoff envelope."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import floor, sqrt

from .atmosphere import standard_atmosphere
from .config import ReferenceCase
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
