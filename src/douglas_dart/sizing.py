"""Transparent propulsion-geometry sizing trades for the handoff envelope."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import floor, sqrt

from .config import ReferenceCase
from .ramjet import evaluate_ramjet


@dataclass(frozen=True)
class RamjetHandoffSizingPoint:
    altitude_m: float
    mach: float
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

    body_diameter_m = case.selector.circular_intake_diameter_m
    throat_to_body_ratio = required_throat_diameter_m / body_diameter_m
    return RamjetHandoffSizingPoint(
        altitude_m=altitude_m,
        mach=mach,
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
