"""Performance-only fuel comparison around one configured vehicle point."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping

from .config import Fuel, ReferenceCase
from .sizing import evaluate_shared_nozzle_trade


@dataclass(frozen=True)
class FuelPerformanceTradePoint:
    fuel_key: str
    display_name: str
    property_source_status: str
    lower_heating_value_mj_per_kg: float
    density_kg_per_m3: float
    loaded_fuel_volume_l: float
    ramjet_budget_fuel_volume_l: float
    loaded_chemical_energy_mj: float
    pulsejet_steady_mean_net_thrust_n: float
    pulsejet_steady_mean_fuel_flow_kg_per_s: float
    ramjet_net_thrust_n: float
    ramjet_fuel_flow_kg_per_s: float
    ramjet_static_fuel_limited_hold_s: float | None
    storage_and_feed_system_mass_included: bool
    ignition_atomization_materials_and_safety_scored: bool
    status: tuple[str, ...]
    numerical_reference_only: bool = True


def fuel_performance_trade(
    case: ReferenceCase,
    fuels: Mapping[str, Fuel],
    *,
    propulsion_derate_fraction: float = 0.15,
) -> list[FuelPerformanceTradePoint]:
    """Compare configured fuel properties without creating an unsupported score."""

    if not fuels:
        raise ValueError("at least one fuel is required")
    points: list[FuelPerformanceTradePoint] = []
    for fuel_key, fuel in fuels.items():
        fuel_case = replace(case, fuel=fuel)
        trade = evaluate_shared_nozzle_trade(
            fuel_case,
            case.vehicle.body_diameter_m,
            case.nozzle.throat_diameter_m,
            case.nozzle.exit_to_throat_area_ratio,
            propulsion_derate_fraction=propulsion_derate_fraction,
        )
        status = [
            "performance_only_no_automatic_fuel_ranking",
            "tank_and_feed_system_mass_not_modeled",
            "ignition_atomization_materials_and_safety_not_scored",
        ]
        if fuel.source_status != "validated":
            status.append("fuel_properties_require_source_validation")
        if "propane" in fuel_key:
            status.append("pressurized_liquid_storage_hardware_not_modeled")

        points.append(
            FuelPerformanceTradePoint(
                fuel_key=fuel_key,
                display_name=fuel.display_name,
                property_source_status=fuel.source_status,
                lower_heating_value_mj_per_kg=(
                    fuel.lower_heating_value_j_per_kg / 1.0e6
                ),
                density_kg_per_m3=fuel.density_kg_per_m3,
                loaded_fuel_volume_l=(
                    1_000.0 * case.mission.loaded_fuel_mass_kg / fuel.density_kg_per_m3
                ),
                ramjet_budget_fuel_volume_l=(
                    1_000.0
                    * case.mission.ramjet_speed_run_fuel_budget_kg
                    / fuel.density_kg_per_m3
                ),
                loaded_chemical_energy_mj=(
                    case.mission.loaded_fuel_mass_kg
                    * fuel.lower_heating_value_j_per_kg
                    / 1.0e6
                ),
                pulsejet_steady_mean_net_thrust_n=(
                    trade.pulsejet_mean_net_thrust_n
                ),
                pulsejet_steady_mean_fuel_flow_kg_per_s=(
                    trade.pulsejet_mean_fuel_mass_flow_kg_per_s
                ),
                ramjet_net_thrust_n=trade.ramjet_net_thrust_n,
                ramjet_fuel_flow_kg_per_s=trade.ramjet_fuel_mass_flow_kg_per_s,
                ramjet_static_fuel_limited_hold_s=(
                    trade.ramjet_fuel_limited_hold_duration_s
                ),
                storage_and_feed_system_mass_included=False,
                ignition_atomization_materials_and_safety_scored=False,
                status=tuple(status),
            )
        )
    return points
