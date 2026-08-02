"""Compressible-flow utilities for reservoirs, restrictions, and fixed C-D nozzles."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt


def stagnation_temperature(static_temperature_k: float, mach: float, gamma: float = 1.4) -> float:
    if static_temperature_k <= 0.0 or mach < 0.0 or gamma <= 1.0:
        raise ValueError("temperature, Mach, and gamma are outside their physical domain")
    return static_temperature_k * (1.0 + 0.5 * (gamma - 1.0) * mach**2)


def stagnation_pressure(static_pressure_pa: float, mach: float, gamma: float = 1.4) -> float:
    if static_pressure_pa <= 0.0 or mach < 0.0 or gamma <= 1.0:
        raise ValueError("pressure, Mach, and gamma are outside their physical domain")
    factor = 1.0 + 0.5 * (gamma - 1.0) * mach**2
    return static_pressure_pa * factor ** (gamma / (gamma - 1.0))


def critical_pressure_ratio(gamma: float) -> float:
    if gamma <= 1.0:
        raise ValueError("gamma must exceed one")
    return (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))


def compressible_orifice_mass_flow(
    upstream_pressure_pa: float,
    upstream_temperature_k: float,
    downstream_pressure_pa: float,
    area_m2: float,
    discharge_coefficient: float,
    gamma: float,
    gas_constant_j_per_kg_k: float,
) -> tuple[float, bool]:
    """Return forward reservoir-to-reservoir flow and whether the restriction chokes."""

    if min(upstream_pressure_pa, upstream_temperature_k, area_m2, gas_constant_j_per_kg_k) <= 0:
        raise ValueError("pressure, temperature, area, and gas constant must be positive")
    if downstream_pressure_pa < 0.0 or not 0.0 < discharge_coefficient <= 1.0:
        raise ValueError("invalid downstream pressure or discharge coefficient")
    if gamma <= 1.0:
        raise ValueError("gamma must exceed one")
    if downstream_pressure_pa >= upstream_pressure_pa:
        return 0.0, False

    pressure_ratio = max(downstream_pressure_pa / upstream_pressure_pa, 0.0)
    common = (
        discharge_coefficient
        * area_m2
        * upstream_pressure_pa
        / sqrt(gas_constant_j_per_kg_k * upstream_temperature_k)
    )
    if pressure_ratio <= critical_pressure_ratio(gamma):
        mass_flow_kg_per_s = common * sqrt(gamma) * (
            2.0 / (gamma + 1.0)
        ) ** ((gamma + 1.0) / (2.0 * (gamma - 1.0)))
        return mass_flow_kg_per_s, True

    bracket = pressure_ratio ** (2.0 / gamma) - pressure_ratio ** ((gamma + 1.0) / gamma)
    mass_flow_kg_per_s = common * sqrt(2.0 * gamma / (gamma - 1.0) * max(bracket, 0.0))
    return mass_flow_kg_per_s, False


def area_ratio_from_mach(mach: float, gamma: float) -> float:
    if mach <= 0.0 or gamma <= 1.0:
        raise ValueError("Mach must be positive and gamma must exceed one")
    base = (2.0 / (gamma + 1.0)) * (1.0 + 0.5 * (gamma - 1.0) * mach**2)
    exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0))
    return base**exponent / mach


def supersonic_mach_from_area_ratio(area_ratio: float, gamma: float) -> float:
    """Solve the supersonic isentropic branch using monotonic bisection."""

    if area_ratio < 1.0:
        raise ValueError("exit-to-throat area ratio must be at least one")
    if area_ratio == 1.0:
        return 1.0
    low, high = 1.0, 20.0
    if area_ratio_from_mach(high, gamma) < area_ratio:
        raise ValueError("area ratio is outside the solver bracket")
    for _ in range(120):
        midpoint = 0.5 * (low + high)
        if area_ratio_from_mach(midpoint, gamma) < area_ratio:
            low = midpoint
        else:
            high = midpoint
    return 0.5 * (low + high)


@dataclass(frozen=True)
class NozzleResult:
    mass_flow_kg_per_s: float
    exit_mach: float
    exit_pressure_pa: float
    exit_temperature_k: float
    exit_velocity_m_per_s: float
    gross_thrust_n: float
    raw_gross_thrust_n: float
    choked: bool
    regime: str
    warning: str | None = None


def fixed_cd_nozzle(
    chamber_total_pressure_pa: float,
    chamber_total_temperature_k: float,
    ambient_pressure_pa: float,
    throat_area_m2: float,
    exit_area_m2: float,
    discharge_coefficient: float,
    gamma: float,
    gas_constant_j_per_kg_k: float,
) -> NozzleResult:
    """Evaluate a fixed-area C-D nozzle from a uniform stagnation reservoir.

    The choked branch assumes attached, isentropic expansion to the geometric exit.
    A warning marks cases where shocks/separation are likely but not modeled.
    """

    if exit_area_m2 < throat_area_m2:
        raise ValueError("a C-D nozzle exit area cannot be below its throat area")
    if chamber_total_pressure_pa <= ambient_pressure_pa:
        return NozzleResult(
            mass_flow_kg_per_s=0.0,
            exit_mach=0.0,
            exit_pressure_pa=ambient_pressure_pa,
            exit_temperature_k=chamber_total_temperature_k,
            exit_velocity_m_per_s=0.0,
            gross_thrust_n=0.0,
            raw_gross_thrust_n=0.0,
            choked=False,
            regime="no_forward_flow",
        )

    mass_flow_kg_per_s, choked = compressible_orifice_mass_flow(
        chamber_total_pressure_pa,
        chamber_total_temperature_k,
        ambient_pressure_pa,
        throat_area_m2,
        discharge_coefficient,
        gamma,
        gas_constant_j_per_kg_k,
    )
    cp_j_per_kg_k = gamma * gas_constant_j_per_kg_k / (gamma - 1.0)

    if not choked:
        pressure_ratio = ambient_pressure_pa / chamber_total_pressure_pa
        exit_temperature_k = chamber_total_temperature_k * pressure_ratio ** (
            (gamma - 1.0) / gamma
        )
        exit_velocity_m_per_s = sqrt(
            max(2.0 * cp_j_per_kg_k * (chamber_total_temperature_k - exit_temperature_k), 0.0)
        )
        raw_thrust_n = mass_flow_kg_per_s * exit_velocity_m_per_s
        return NozzleResult(
            mass_flow_kg_per_s=mass_flow_kg_per_s,
            exit_mach=exit_velocity_m_per_s / sqrt(
                gamma * gas_constant_j_per_kg_k * exit_temperature_k
            ),
            exit_pressure_pa=ambient_pressure_pa,
            exit_temperature_k=exit_temperature_k,
            exit_velocity_m_per_s=exit_velocity_m_per_s,
            gross_thrust_n=raw_thrust_n,
            raw_gross_thrust_n=raw_thrust_n,
            choked=False,
            regime="unchoked",
        )

    exit_mach = supersonic_mach_from_area_ratio(exit_area_m2 / throat_area_m2, gamma)
    temperature_ratio = 1.0 + 0.5 * (gamma - 1.0) * exit_mach**2
    exit_temperature_k = chamber_total_temperature_k / temperature_ratio
    exit_pressure_pa = chamber_total_pressure_pa / temperature_ratio ** (gamma / (gamma - 1.0))
    exit_velocity_m_per_s = exit_mach * sqrt(
        gamma * gas_constant_j_per_kg_k * exit_temperature_k
    )
    raw_thrust_n = (
        mass_flow_kg_per_s * exit_velocity_m_per_s
        + (exit_pressure_pa - ambient_pressure_pa) * exit_area_m2
    )
    warning = None
    if ambient_pressure_pa > 1.5 * exit_pressure_pa:
        warning = "strongly_overexpanded_shock_or_separation_not_modeled"
    return NozzleResult(
        mass_flow_kg_per_s=mass_flow_kg_per_s,
        exit_mach=exit_mach,
        exit_pressure_pa=exit_pressure_pa,
        exit_temperature_k=exit_temperature_k,
        exit_velocity_m_per_s=exit_velocity_m_per_s,
        gross_thrust_n=max(raw_thrust_n, 0.0),
        raw_gross_thrust_n=raw_thrust_n,
        choked=True,
        regime="choked_supersonic_exit",
        warning=warning,
    )
