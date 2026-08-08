"""Longitudinal point-mass flight-equation kernel."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .config import FlightConfig


@dataclass(frozen=True)
class PointMassState:
    downrange_m: float
    altitude_m: float
    speed_m_per_s: float
    flight_path_angle_rad: float
    mass_kg: float


@dataclass(frozen=True)
class PointMassDerivative:
    downrange_rate_m_per_s: float
    climb_rate_m_per_s: float
    acceleration_m_per_s2: float
    flight_path_rate_rad_per_s: float
    mass_rate_kg_per_s: float
    lift_n: float
    drag_n: float


def aerodynamic_coefficients(
    flight: FlightConfig, angle_of_attack_rad: float
) -> tuple[float, float]:
    lift_coefficient = flight.lift_curve_slope_per_rad * angle_of_attack_rad
    drag_coefficient = (
        flight.zero_lift_drag_coefficient
        + flight.induced_drag_factor * lift_coefficient**2
    )
    return lift_coefficient, drag_coefficient


def longitudinal_derivative(
    state: PointMassState,
    flight: FlightConfig,
    thrust_n: float,
    fuel_mass_flow_kg_per_s: float,
    angle_of_attack_rad: float,
) -> PointMassDerivative:
    if state.mass_kg <= 0.0 or state.speed_m_per_s <= 0.0:
        raise ValueError("mass and speed must be positive")
    atmosphere = standard_atmosphere(state.altitude_m)
    dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * state.speed_m_per_s**2
    lift_coefficient, drag_coefficient = aerodynamic_coefficients(
        flight, angle_of_attack_rad
    )
    lift_n = dynamic_pressure_pa * flight.reference_area_m2 * lift_coefficient
    drag_n = dynamic_pressure_pa * flight.reference_area_m2 * drag_coefficient
    return longitudinal_derivative_from_forces(
        state,
        thrust_n=thrust_n,
        fuel_mass_flow_kg_per_s=fuel_mass_flow_kg_per_s,
        angle_of_attack_rad=angle_of_attack_rad,
        lift_n=lift_n,
        drag_n=drag_n,
    )


def longitudinal_derivative_from_forces(
    state: PointMassState,
    *,
    thrust_n: float,
    fuel_mass_flow_kg_per_s: float,
    angle_of_attack_rad: float,
    lift_n: float,
    drag_n: float,
) -> PointMassDerivative:
    """Evaluate the point-mass equations from independently audited forces."""

    if state.mass_kg <= 0.0 or state.speed_m_per_s <= 0.0:
        raise ValueError("mass and speed must be positive")
    if fuel_mass_flow_kg_per_s < 0.0:
        raise ValueError("fuel mass flow cannot be negative")
    if drag_n < 0.0:
        raise ValueError("drag input cannot be negative")
    gamma = state.flight_path_angle_rad
    acceleration_m_per_s2 = (
        thrust_n * cos(angle_of_attack_rad)
        - drag_n
        - state.mass_kg * G0_M_PER_S2 * sin(gamma)
    ) / state.mass_kg
    flight_path_rate_rad_per_s = (
        lift_n
        + thrust_n * sin(angle_of_attack_rad)
        - state.mass_kg * G0_M_PER_S2 * cos(gamma)
    ) / (state.mass_kg * state.speed_m_per_s)
    return PointMassDerivative(
        downrange_rate_m_per_s=state.speed_m_per_s * cos(gamma),
        climb_rate_m_per_s=state.speed_m_per_s * sin(gamma),
        acceleration_m_per_s2=acceleration_m_per_s2,
        flight_path_rate_rad_per_s=flight_path_rate_rad_per_s,
        mass_rate_kg_per_s=-fuel_mass_flow_kg_per_s,
        lift_n=lift_n,
        drag_n=drag_n,
    )
