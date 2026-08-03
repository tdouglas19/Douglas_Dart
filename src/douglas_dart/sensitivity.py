"""Local key-variable sensitivities for the low-order propulsion models."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

from .config import ReferenceCase
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet


@dataclass(frozen=True)
class EngineSensitivityPoint:
    engine: str
    variable: str
    low_input: float
    baseline_input: float
    high_input: float
    low_net_thrust_n: float
    baseline_net_thrust_n: float
    high_net_thrust_n: float
    net_thrust_normalized_slope: float
    low_fuel_mass_flow_kg_per_s: float
    baseline_fuel_mass_flow_kg_per_s: float
    high_fuel_mass_flow_kg_per_s: float
    fuel_flow_normalized_slope: float
    low_peak_chamber_pressure_pa: float | None
    baseline_peak_chamber_pressure_pa: float | None
    high_peak_chamber_pressure_pa: float | None
    low_inlet_spillage_fraction: float | None
    baseline_inlet_spillage_fraction: float | None
    high_inlet_spillage_fraction: float | None
    low_status: tuple[str, ...]
    baseline_status: tuple[str, ...]
    high_status: tuple[str, ...]
    perturbation_fraction: float
    numerical_reference_only: bool = True


def _normalized_slope(
    low_input: float,
    baseline_input: float,
    high_input: float,
    low_output: float,
    baseline_output: float,
    high_output: float,
) -> float:
    input_span_fraction = (high_input - low_input) / baseline_input
    if abs(input_span_fraction) <= 1e-15 or abs(baseline_output) <= 1e-15:
        return 0.0
    return ((high_output - low_output) / baseline_output) / input_span_fraction


def _bounded_values(
    baseline: float,
    fraction: float,
    *,
    lower: float | None = None,
    upper: float | None = None,
) -> tuple[float, float, float]:
    low = baseline * (1.0 - fraction)
    high = baseline * (1.0 + fraction)
    if lower is not None:
        low = max(low, lower)
    if upper is not None:
        high = min(high, upper)
    return low, baseline, high


def _pulsejet_case_modifier(
    case: ReferenceCase,
    variable: str,
    value: float,
) -> ReferenceCase:
    if variable == "throat_diameter_m":
        return replace(case, nozzle=replace(case.nozzle, throat_diameter_m=value))
    if variable == "exit_to_throat_area_ratio":
        return replace(
            case,
            nozzle=replace(case.nozzle, exit_to_throat_area_ratio=value),
        )
    if variable == "selector_discharge_coefficient":
        return replace(
            case,
            selector=replace(case.selector, discharge_coefficient=value),
        )
    if variable == "selector_total_pressure_recovery":
        return replace(
            case,
            selector=replace(case.selector, total_pressure_recovery=value),
        )
    if variable in {
        "chamber_volume_m3",
        "combustion_efficiency",
        "target_equivalence_ratio",
        "burn_duration_s",
    }:
        return replace(case, pulsejet=replace(case.pulsejet, **{variable: value}))
    if variable == "fuel_lower_heating_value_j_per_kg":
        return replace(
            case,
            fuel=replace(case.fuel, lower_heating_value_j_per_kg=value),
        )
    raise ValueError(f"unsupported pulsejet sensitivity variable: {variable}")


def _ramjet_case_modifier(
    case: ReferenceCase,
    variable: str,
    value: float,
) -> ReferenceCase:
    if variable == "throat_diameter_m":
        return replace(case, nozzle=replace(case.nozzle, throat_diameter_m=value))
    if variable == "exit_to_throat_area_ratio":
        return replace(
            case,
            nozzle=replace(case.nozzle, exit_to_throat_area_ratio=value),
        )
    if variable == "selector_total_pressure_recovery":
        return replace(
            case,
            selector=replace(case.selector, total_pressure_recovery=value),
        )
    if variable in {
        "mass_capture_coefficient",
        "combustor_total_pressure_loss_fraction",
        "combustor_efficiency",
        "target_combustor_exit_temperature_k",
    }:
        return replace(case, ramjet=replace(case.ramjet, **{variable: value}))
    if variable == "fuel_lower_heating_value_j_per_kg":
        return replace(
            case,
            fuel=replace(case.fuel, lower_heating_value_j_per_kg=value),
        )
    raise ValueError(f"unsupported ramjet sensitivity variable: {variable}")


def _input_values(
    case: ReferenceCase,
    variable: str,
    fraction: float,
) -> tuple[float, float, float]:
    sources = {
        "throat_diameter_m": case.nozzle.throat_diameter_m,
        "exit_to_throat_area_ratio": case.nozzle.exit_to_throat_area_ratio,
        "selector_discharge_coefficient": case.selector.discharge_coefficient,
        "selector_total_pressure_recovery": case.selector.total_pressure_recovery,
        "chamber_volume_m3": case.pulsejet.chamber_volume_m3,
        "combustion_efficiency": case.pulsejet.combustion_efficiency,
        "target_equivalence_ratio": case.pulsejet.target_equivalence_ratio,
        "burn_duration_s": case.pulsejet.burn_duration_s,
        "mass_capture_coefficient": case.ramjet.mass_capture_coefficient,
        "combustor_total_pressure_loss_fraction": (
            case.ramjet.combustor_total_pressure_loss_fraction
        ),
        "combustor_efficiency": case.ramjet.combustor_efficiency,
        "target_combustor_exit_temperature_k": (
            case.ramjet.target_combustor_exit_temperature_k
        ),
        "fuel_lower_heating_value_j_per_kg": case.fuel.lower_heating_value_j_per_kg,
    }
    if variable not in sources:
        raise ValueError(f"unsupported sensitivity variable: {variable}")
    lower = 1.0 if variable == "exit_to_throat_area_ratio" else None
    upper = 1.0 if variable in {
        "selector_discharge_coefficient",
        "selector_total_pressure_recovery",
        "combustion_efficiency",
        "mass_capture_coefficient",
        "combustor_efficiency",
    } else None
    return _bounded_values(sources[variable], fraction, lower=lower, upper=upper)


def _pulsejet_outputs(
    case: ReferenceCase,
    warmup_s: float,
    measurement_s: float,
    time_step_s: float,
) -> tuple[float, float, float]:
    simulator = PulsejetSimulator(
        case.pulsejet,
        case.selector,
        case.nozzle,
        case.fuel,
        case.altitude_m,
        case.mach,
    )
    summary = summarize_pulsejet(
        simulator.run(warmup_s + measurement_s, time_step_s),
        minimum_time_s=warmup_s,
    )
    return (
        summary.mean_net_thrust_n,
        summary.mean_fuel_mass_flow_kg_per_s,
        summary.peak_chamber_pressure_pa,
    )


def pulsejet_local_sensitivities(
    case: ReferenceCase,
    variables: Iterable[str] = (
        "throat_diameter_m",
        "exit_to_throat_area_ratio",
        "chamber_volume_m3",
        "combustion_efficiency",
        "target_equivalence_ratio",
        "burn_duration_s",
        "selector_discharge_coefficient",
        "selector_total_pressure_recovery",
        "fuel_lower_heating_value_j_per_kg",
    ),
    *,
    perturbation_fraction: float = 0.10,
    warmup_s: float = 0.25,
    measurement_s: float = 0.25,
    time_step_s: float = 0.00004,
) -> list[EngineSensitivityPoint]:
    if not 0.0 < perturbation_fraction < 1.0:
        raise ValueError("perturbation fraction must be in (0, 1)")
    if warmup_s <= 0.0 or measurement_s <= 0.0:
        raise ValueError("pulsejet warmup and measurement durations must be positive")
    baseline_outputs = _pulsejet_outputs(
        case,
        warmup_s,
        measurement_s,
        time_step_s,
    )
    points: list[EngineSensitivityPoint] = []
    for variable in variables:
        low, baseline, high = _input_values(case, variable, perturbation_fraction)
        low_outputs = _pulsejet_outputs(
            _pulsejet_case_modifier(case, variable, low),
            warmup_s,
            measurement_s,
            time_step_s,
        )
        high_outputs = _pulsejet_outputs(
            _pulsejet_case_modifier(case, variable, high),
            warmup_s,
            measurement_s,
            time_step_s,
        )
        points.append(
            EngineSensitivityPoint(
                engine="pulsejet",
                variable=variable,
                low_input=low,
                baseline_input=baseline,
                high_input=high,
                low_net_thrust_n=low_outputs[0],
                baseline_net_thrust_n=baseline_outputs[0],
                high_net_thrust_n=high_outputs[0],
                net_thrust_normalized_slope=_normalized_slope(
                    low, baseline, high, low_outputs[0], baseline_outputs[0], high_outputs[0]
                ),
                low_fuel_mass_flow_kg_per_s=low_outputs[1],
                baseline_fuel_mass_flow_kg_per_s=baseline_outputs[1],
                high_fuel_mass_flow_kg_per_s=high_outputs[1],
                fuel_flow_normalized_slope=_normalized_slope(
                    low, baseline, high, low_outputs[1], baseline_outputs[1], high_outputs[1]
                ),
                low_peak_chamber_pressure_pa=low_outputs[2],
                baseline_peak_chamber_pressure_pa=baseline_outputs[2],
                high_peak_chamber_pressure_pa=high_outputs[2],
                low_inlet_spillage_fraction=None,
                baseline_inlet_spillage_fraction=None,
                high_inlet_spillage_fraction=None,
                low_status=(),
                baseline_status=(),
                high_status=(),
                perturbation_fraction=perturbation_fraction,
            )
        )
    return points


def _ramjet_outputs(
    case: ReferenceCase,
) -> tuple[float, float, float, tuple[str, ...]]:
    result = evaluate_ramjet(
        case.ramjet,
        case.selector,
        case.nozzle,
        case.fuel,
        case.mission.speed_run_altitude_msl_m,
        case.mission.peak_mach,
    )
    return (
        result.net_thrust_n,
        result.fuel_mass_flow_kg_per_s,
        result.inlet_spillage_fraction,
        result.status,
    )


def ramjet_local_sensitivities(
    case: ReferenceCase,
    variables: Iterable[str] = (
        "throat_diameter_m",
        "exit_to_throat_area_ratio",
        "selector_total_pressure_recovery",
        "mass_capture_coefficient",
        "combustor_total_pressure_loss_fraction",
        "combustor_efficiency",
        "target_combustor_exit_temperature_k",
        "fuel_lower_heating_value_j_per_kg",
    ),
    *,
    perturbation_fraction: float = 0.10,
) -> list[EngineSensitivityPoint]:
    if not 0.0 < perturbation_fraction < 1.0:
        raise ValueError("perturbation fraction must be in (0, 1)")
    baseline_outputs = _ramjet_outputs(case)
    points: list[EngineSensitivityPoint] = []
    for variable in variables:
        low, baseline, high = _input_values(case, variable, perturbation_fraction)
        low_outputs = _ramjet_outputs(_ramjet_case_modifier(case, variable, low))
        high_outputs = _ramjet_outputs(_ramjet_case_modifier(case, variable, high))
        points.append(
            EngineSensitivityPoint(
                engine="ramjet",
                variable=variable,
                low_input=low,
                baseline_input=baseline,
                high_input=high,
                low_net_thrust_n=low_outputs[0],
                baseline_net_thrust_n=baseline_outputs[0],
                high_net_thrust_n=high_outputs[0],
                net_thrust_normalized_slope=_normalized_slope(
                    low, baseline, high, low_outputs[0], baseline_outputs[0], high_outputs[0]
                ),
                low_fuel_mass_flow_kg_per_s=low_outputs[1],
                baseline_fuel_mass_flow_kg_per_s=baseline_outputs[1],
                high_fuel_mass_flow_kg_per_s=high_outputs[1],
                fuel_flow_normalized_slope=_normalized_slope(
                    low, baseline, high, low_outputs[1], baseline_outputs[1], high_outputs[1]
                ),
                low_peak_chamber_pressure_pa=None,
                baseline_peak_chamber_pressure_pa=None,
                high_peak_chamber_pressure_pa=None,
                low_inlet_spillage_fraction=low_outputs[2],
                baseline_inlet_spillage_fraction=baseline_outputs[2],
                high_inlet_spillage_fraction=high_outputs[2],
                low_status=low_outputs[3],
                baseline_status=baseline_outputs[3],
                high_status=high_outputs[3],
                perturbation_fraction=perturbation_fraction,
            )
        )
    return points


def rank_by_net_thrust_sensitivity(
    points: Iterable[EngineSensitivityPoint],
) -> list[EngineSensitivityPoint]:
    return sorted(
        points,
        key=lambda point: abs(point.net_thrust_normalized_slope),
        reverse=True,
    )
