"""Steady low-order ramjet cycle with explicit operability and flow-balance flags."""

from __future__ import annotations

from dataclasses import dataclass

from .atmosphere import standard_atmosphere
from .compressible import fixed_cd_nozzle, stagnation_pressure, stagnation_temperature
from .config import Fuel, NozzleConfig, RamjetConfig, SelectorConfig


@dataclass(frozen=True)
class RamjetResult:
    mach: float
    potential_captured_air_mass_flow_kg_per_s: float
    air_mass_flow_kg_per_s: float
    fuel_mass_flow_kg_per_s: float
    fuel_air_ratio: float
    combustor_inlet_total_pressure_pa: float
    combustor_exit_total_pressure_pa: float
    combustor_inlet_total_temperature_k: float
    combustor_exit_total_temperature_k: float
    nozzle_capacity_kg_per_s: float
    nozzle_mass_flow_residual_fraction: float
    inlet_spillage_fraction: float
    net_thrust_n: float
    specific_thrust_n_s_per_kg_air: float
    self_sustaining_candidate: bool
    status: tuple[str, ...]


def evaluate_ramjet(
    config: RamjetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
) -> RamjetResult:
    if mach < 0.0:
        raise ValueError("Mach cannot be negative")
    atmosphere = standard_atmosphere(altitude_m)
    velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
    potential_air_mass_flow_kg_per_s = (
        atmosphere.density_kg_per_m3
        * velocity_m_per_s
        * selector.available_area_m2
        * config.mass_capture_coefficient
    )
    inlet_total_temperature_k = stagnation_temperature(atmosphere.temperature_k, mach)
    ideal_total_pressure_pa = stagnation_pressure(atmosphere.pressure_pa, mach)
    inlet_total_pressure_pa = atmosphere.pressure_pa + (
        ideal_total_pressure_pa - atmosphere.pressure_pa
    ) * selector.total_pressure_recovery
    combustor_exit_pressure_pa = inlet_total_pressure_pa * (
        1.0 - config.combustor_total_pressure_loss_fraction
    )
    cp_j_per_kg_k = config.gamma * config.gas_constant_j_per_kg_k / (config.gamma - 1.0)
    target_temperature_k = config.target_combustor_exit_temperature_k
    numerator = cp_j_per_kg_k * max(target_temperature_k - inlet_total_temperature_k, 0.0)
    denominator = (
        config.combustor_efficiency * fuel.lower_heating_value_j_per_kg
        - cp_j_per_kg_k * target_temperature_k
    )
    fuel_air_ratio = numerator / denominator if denominator > 0.0 else 0.0
    potential_fuel_mass_flow_kg_per_s = potential_air_mass_flow_kg_per_s * fuel_air_ratio
    demanded_nozzle_mass_flow_kg_per_s = (
        potential_air_mass_flow_kg_per_s + potential_fuel_mass_flow_kg_per_s
    )

    nozzle_result = fixed_cd_nozzle(
        combustor_exit_pressure_pa,
        target_temperature_k,
        atmosphere.pressure_pa,
        nozzle.throat_area_m2,
        nozzle.exit_area_m2,
        nozzle.discharge_coefficient,
        config.gamma,
        config.gas_constant_j_per_kg_k,
    )
    if demanded_nozzle_mass_flow_kg_per_s > 1e-12:
        residual_fraction = (
            nozzle_result.mass_flow_kg_per_s - demanded_nozzle_mass_flow_kg_per_s
        ) / demanded_nozzle_mass_flow_kg_per_s
    else:
        residual_fraction = 0.0

    actual_nozzle_mass_flow_kg_per_s = min(
        demanded_nozzle_mass_flow_kg_per_s, nozzle_result.mass_flow_kg_per_s
    )
    air_mass_flow_kg_per_s = actual_nozzle_mass_flow_kg_per_s / (1.0 + fuel_air_ratio)
    fuel_mass_flow_kg_per_s = air_mass_flow_kg_per_s * fuel_air_ratio
    inlet_spillage_fraction = (
        1.0 - air_mass_flow_kg_per_s / potential_air_mass_flow_kg_per_s
        if potential_air_mass_flow_kg_per_s > 1e-12
        else 0.0
    )
    nozzle_flow_scale = (
        actual_nozzle_mass_flow_kg_per_s / nozzle_result.mass_flow_kg_per_s
        if nozzle_result.mass_flow_kg_per_s > 1e-12
        else 0.0
    )
    # Scaling the complete fixed-nozzle gross thrust is an explicit low-order
    # treatment for under-fed cases. A future matching solver will instead adjust
    # combustor back pressure until captured and discharged flow agree.
    gross_thrust_n = nozzle_result.gross_thrust_n * nozzle_flow_scale
    net_thrust_n = gross_thrust_n - air_mass_flow_kg_per_s * velocity_m_per_s
    specific_thrust = (
        net_thrust_n / air_mass_flow_kg_per_s if air_mass_flow_kg_per_s > 1e-12 else 0.0
    )

    status: list[str] = []
    if mach < config.minimum_self_sustaining_mach:
        status.append("below_configured_self_sustaining_mach")
    if combustor_exit_pressure_pa <= atmosphere.pressure_pa:
        status.append("insufficient_nozzle_pressure_ratio")
    if abs(residual_fraction) > 0.20:
        status.append("fixed_nozzle_mass_flow_mismatch")
    if nozzle_result.warning:
        status.append(nozzle_result.warning)
    self_sustaining_candidate = not any(
        flag
        in {
            "below_configured_self_sustaining_mach",
            "insufficient_nozzle_pressure_ratio",
            "fixed_nozzle_mass_flow_mismatch",
        }
        for flag in status
    )
    return RamjetResult(
        mach=mach,
        potential_captured_air_mass_flow_kg_per_s=potential_air_mass_flow_kg_per_s,
        air_mass_flow_kg_per_s=air_mass_flow_kg_per_s,
        fuel_mass_flow_kg_per_s=fuel_mass_flow_kg_per_s,
        fuel_air_ratio=fuel_air_ratio,
        combustor_inlet_total_pressure_pa=inlet_total_pressure_pa,
        combustor_exit_total_pressure_pa=combustor_exit_pressure_pa,
        combustor_inlet_total_temperature_k=inlet_total_temperature_k,
        combustor_exit_total_temperature_k=target_temperature_k,
        nozzle_capacity_kg_per_s=nozzle_result.mass_flow_kg_per_s,
        nozzle_mass_flow_residual_fraction=residual_fraction,
        inlet_spillage_fraction=inlet_spillage_fraction,
        net_thrust_n=net_thrust_n,
        specific_thrust_n_s_per_kg_air=specific_thrust,
        self_sustaining_candidate=self_sustaining_candidate,
        status=tuple(status),
    )
