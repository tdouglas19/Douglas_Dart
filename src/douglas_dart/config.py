"""Validated configuration objects and YAML loading."""

from __future__ import annotations

from dataclasses import dataclass
from math import pi
from pathlib import Path
from typing import Any, Mapping

import yaml


def _positive(name: str, value: float) -> float:
    value = float(value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _fraction(name: str, value: float, *, inclusive_one: bool = True) -> float:
    value = float(value)
    upper_ok = value <= 1.0 if inclusive_one else value < 1.0
    if value <= 0.0 or not upper_ok:
        interval = "(0, 1]" if inclusive_one else "(0, 1)"
        raise ValueError(f"{name} must be in {interval}")
    return value


@dataclass(frozen=True)
class Fuel:
    key: str
    display_name: str
    lower_heating_value_j_per_kg: float
    stoichiometric_air_fuel_ratio: float
    density_kg_per_m3: float
    source_status: str

    def __post_init__(self) -> None:
        _positive("lower_heating_value_j_per_kg", self.lower_heating_value_j_per_kg)
        _positive("stoichiometric_air_fuel_ratio", self.stoichiometric_air_fuel_ratio)
        _positive("density_kg_per_m3", self.density_kg_per_m3)


@dataclass(frozen=True)
class SelectorConfig:
    circular_intake_diameter_m: float
    open_fraction: float
    discharge_coefficient: float
    total_pressure_recovery: float

    def __post_init__(self) -> None:
        _positive("circular_intake_diameter_m", self.circular_intake_diameter_m)
        if not 0.0 < self.open_fraction <= 0.5:
            raise ValueError(
                "open_fraction must be in (0, 0.5] for mutually exclusive half-intakes"
            )
        _fraction("discharge_coefficient", self.discharge_coefficient)
        _fraction("total_pressure_recovery", self.total_pressure_recovery)

    @property
    def circular_area_m2(self) -> float:
        return pi * self.circular_intake_diameter_m**2 / 4.0

    @property
    def available_area_m2(self) -> float:
        return self.open_fraction * self.circular_area_m2


@dataclass(frozen=True)
class NozzleConfig:
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    discharge_coefficient: float

    def __post_init__(self) -> None:
        _positive("throat_diameter_m", self.throat_diameter_m)
        if self.exit_to_throat_area_ratio < 1.0:
            raise ValueError("exit_to_throat_area_ratio must be at least one")
        _fraction("discharge_coefficient", self.discharge_coefficient)

    @property
    def throat_area_m2(self) -> float:
        return pi * self.throat_diameter_m**2 / 4.0

    @property
    def exit_area_m2(self) -> float:
        return self.throat_area_m2 * self.exit_to_throat_area_ratio


@dataclass(frozen=True)
class PulsejetConfig:
    chamber_volume_m3: float
    gamma: float
    gas_constant_j_per_kg_k: float
    combustion_efficiency: float
    target_equivalence_ratio: float
    initial_equivalence_ratio: float
    burn_duration_s: float
    minimum_cycle_period_s: float
    ignition_pressure_ratio_max: float
    minimum_fresh_air_fraction: float
    wall_temperature_k: float
    wall_heat_transfer_w_per_k: float
    maximum_gas_temperature_k: float

    def __post_init__(self) -> None:
        for name in (
            "chamber_volume_m3",
            "gas_constant_j_per_kg_k",
            "target_equivalence_ratio",
            "initial_equivalence_ratio",
            "burn_duration_s",
            "minimum_cycle_period_s",
            "ignition_pressure_ratio_max",
            "minimum_fresh_air_fraction",
            "wall_temperature_k",
            "maximum_gas_temperature_k",
        ):
            _positive(name, getattr(self, name))
        if self.gamma <= 1.0:
            raise ValueError("gamma must exceed one")
        _fraction("combustion_efficiency", self.combustion_efficiency)
        if self.wall_heat_transfer_w_per_k < 0.0:
            raise ValueError("wall_heat_transfer_w_per_k cannot be negative")
        if self.maximum_gas_temperature_k <= self.wall_temperature_k:
            raise ValueError("maximum gas temperature must exceed wall temperature")


@dataclass(frozen=True)
class RamjetConfig:
    gamma: float
    gas_constant_j_per_kg_k: float
    mass_capture_coefficient: float
    combustor_total_pressure_loss_fraction: float
    combustor_efficiency: float
    target_combustor_exit_temperature_k: float
    minimum_lightoff_test_mach: float
    minimum_self_sustaining_mach: float

    def __post_init__(self) -> None:
        if self.gamma <= 1.0:
            raise ValueError("gamma must exceed one")
        _positive("gas_constant_j_per_kg_k", self.gas_constant_j_per_kg_k)
        _fraction("mass_capture_coefficient", self.mass_capture_coefficient)
        _fraction("combustor_efficiency", self.combustor_efficiency)
        if not 0.0 <= self.combustor_total_pressure_loss_fraction < 1.0:
            raise ValueError("combustor pressure loss must be in [0, 1)")
        _positive(
            "target_combustor_exit_temperature_k", self.target_combustor_exit_temperature_k
        )
        _positive("minimum_lightoff_test_mach", self.minimum_lightoff_test_mach)
        _positive("minimum_self_sustaining_mach", self.minimum_self_sustaining_mach)
        if self.minimum_self_sustaining_mach < self.minimum_lightoff_test_mach:
            raise ValueError("self-sustaining Mach cannot be below the light-off test Mach")


@dataclass(frozen=True)
class FlightConfig:
    reference_area_m2: float
    initial_mass_kg: float
    zero_lift_drag_coefficient: float
    induced_drag_factor: float
    lift_curve_slope_per_rad: float

    def __post_init__(self) -> None:
        for name in (
            "reference_area_m2",
            "initial_mass_kg",
            "zero_lift_drag_coefficient",
            "induced_drag_factor",
            "lift_curve_slope_per_rad",
        ):
            _positive(name, getattr(self, name))


@dataclass(frozen=True)
class SimulationConfig:
    pulsejet_duration_s: float
    time_step_s: float

    def __post_init__(self) -> None:
        _positive("pulsejet_duration_s", self.pulsejet_duration_s)
        _positive("time_step_s", self.time_step_s)


@dataclass(frozen=True)
class ReferenceCase:
    name: str
    altitude_m: float
    mach: float
    fuel: Fuel
    selector: SelectorConfig
    nozzle: NozzleConfig
    pulsejet: PulsejetConfig
    ramjet: RamjetConfig
    flight: FlightConfig
    simulation: SimulationConfig


def _mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"missing or invalid '{key}' mapping")
    return value


def _read_yaml(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def load_reference_case(
    case_path: str | Path,
    fuels_path: str | Path | None = None,
) -> ReferenceCase:
    case_path = Path(case_path)
    fuels_path = Path(fuels_path) if fuels_path else case_path.with_name("fuels.yaml")
    data = _read_yaml(case_path)
    fuel_data = _mapping(_read_yaml(fuels_path), "fuels")
    case_header = _mapping(data, "case")
    fuel_key = str(case_header["fuel_key"])
    selected_fuel = _mapping(fuel_data, fuel_key)

    environment = _mapping(data, "environment")
    selector = _mapping(data, "selector")
    nozzle = _mapping(data, "nozzle")
    pulsejet = _mapping(data, "pulsejet")
    ramjet = _mapping(data, "ramjet")
    flight = _mapping(data, "flight")
    simulation = _mapping(data, "simulation")

    return ReferenceCase(
        name=str(case_header["name"]),
        altitude_m=float(environment["altitude_m"]),
        mach=float(environment["mach"]),
        fuel=Fuel(key=fuel_key, **selected_fuel),
        selector=SelectorConfig(**selector),
        nozzle=NozzleConfig(**nozzle),
        pulsejet=PulsejetConfig(**pulsejet),
        ramjet=RamjetConfig(**ramjet),
        flight=FlightConfig(**flight),
        simulation=SimulationConfig(**simulation),
    )
