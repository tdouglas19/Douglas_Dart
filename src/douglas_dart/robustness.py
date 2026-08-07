"""Transparent mass and off-nominal peak-Mach robustness trades."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import sqrt
from pathlib import Path
from typing import Any, Mapping

import yaml

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .config import ReferenceCase
from .propulsion_map import RAMJET_MODE, PropulsionScenario, evaluate_propulsion_map_point
from .sizing import evaluate_shared_nozzle_trade, geometrically_scaled_drag_area_target_m2


@dataclass(frozen=True)
class MassComponent:
    name: str
    current_mass_kg: float
    uncertainty_plus_kg: float
    maturity: str


@dataclass(frozen=True)
class MassBudgetSummary:
    components: tuple[MassComponent, ...]
    current_total_mass_kg: float
    uncertainty_plus_total_kg: float
    high_total_mass_kg: float
    current_mass_matches_case: bool
    high_mass_margin_to_requirement_kg: float
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class RobustnessScenario:
    name: str
    ramjet_total_pressure_recovery: float
    propulsion_thrust_multiplier: float
    drag_multiplier: float
    mass_growth_kg: float
    required_excess_thrust_n: float
    required_for_candidate_selection: bool


@dataclass(frozen=True)
class RobustnessScenarioResult:
    scenario: str
    ramjet_total_pressure_recovery: float
    propulsion_thrust_multiplier: float
    drag_multiplier: float
    evaluated_mass_kg: float
    mass_margin_to_requirement_kg: float
    ramjet_net_thrust_n: float
    available_thrust_n: float
    drag_n: float
    excess_thrust_n: float
    required_excess_thrust_n: float
    inlet_spillage_fraction: float
    fuel_mass_flow_kg_per_s: float
    full_throttle_fuel_endurance_s: float | None
    passes_mass: bool
    passes_excess_thrust: bool
    passes_operability_flags: bool
    required_for_candidate_selection: bool
    status: tuple[str, ...]
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class CandidateRobustnessPoint:
    body_diameter_m: float
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    exit_diameter_m: float
    radial_packaging_margin_m: float
    packageable_with_configured_allowances: bool
    pulsejet_mean_net_thrust_n: float
    pulsejet_mean_net_thrust_to_weight: float
    scenarios: tuple[RobustnessScenarioResult, ...]
    required_scenarios_pass: bool
    objective_score: float
    status: tuple[str, ...]
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class RobustnessTradeResult:
    mass_budget: MassBudgetSummary
    selection_rule: str
    selected_candidate: CandidateRobustnessPoint | None
    points: tuple[CandidateRobustnessPoint, ...]
    numerical_reference_only: bool = True


def _mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"missing or invalid '{key}' mapping")
    return value


def _load_yaml(path: str | Path) -> Mapping[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, Mapping):
        raise ValueError("robustness configuration must be a YAML mapping")
    return value


def summarize_mass_budget(
    case: ReferenceCase,
    robustness_path: str | Path,
) -> MassBudgetSummary:
    data = _load_yaml(robustness_path)
    mass_data = _mapping(data, "mass_budget")
    component_data = _mapping(mass_data, "components")
    components = tuple(
        MassComponent(name=str(name), **_mapping(component_data, str(name)))
        for name in component_data
    )
    for component in components:
        if component.current_mass_kg < 0.0 or component.uncertainty_plus_kg < 0.0:
            raise ValueError("mass components and uncertainty additions cannot be negative")
        if not component.maturity.strip():
            raise ValueError("mass component maturity cannot be empty")
    current_total_kg = sum(item.current_mass_kg for item in components)
    expected_total_kg = float(mass_data["expected_current_total_mass_kg"])
    if abs(current_total_kg - expected_total_kg) > 1e-9:
        raise ValueError("mass components do not sum to expected_current_total_mass_kg")
    uncertainty_kg = sum(item.uncertainty_plus_kg for item in components)
    high_total_kg = current_total_kg + uncertainty_kg
    return MassBudgetSummary(
        components=components,
        current_total_mass_kg=current_total_kg,
        uncertainty_plus_total_kg=uncertainty_kg,
        high_total_mass_kg=high_total_kg,
        current_mass_matches_case=abs(current_total_kg - case.flight.initial_mass_kg) <= 1e-9,
        high_mass_margin_to_requirement_kg=(
            case.requirements.maximum_takeoff_mass_kg - high_total_kg
        ),
    )


def _load_scenarios(data: Mapping[str, Any]) -> tuple[RobustnessScenario, ...]:
    scenario_data = _mapping(_mapping(data, "robustness"), "scenarios")
    scenarios = tuple(
        RobustnessScenario(name=str(name), **_mapping(scenario_data, str(name)))
        for name in scenario_data
    )
    if not scenarios or not any(item.required_for_candidate_selection for item in scenarios):
        raise ValueError("at least one robustness scenario must be required for selection")
    for scenario in scenarios:
        if not 0.0 < scenario.ramjet_total_pressure_recovery <= 1.0:
            raise ValueError("ramjet total-pressure recovery must be in (0, 1]")
        if not 0.0 < scenario.propulsion_thrust_multiplier <= 1.0:
            raise ValueError("propulsion thrust multiplier must be in (0, 1]")
        if scenario.drag_multiplier <= 0.0 or scenario.mass_growth_kg < 0.0:
            raise ValueError("drag multiplier must be positive and mass growth nonnegative")
    return scenarios


def _evaluate_scenario(
    case: ReferenceCase,
    body_diameter_m: float,
    throat_diameter_m: float,
    area_ratio: float,
    scenario: RobustnessScenario,
    current_mass_kg: float,
) -> RobustnessScenarioResult:
    nozzle = replace(
        case.nozzle,
        throat_diameter_m=throat_diameter_m,
        exit_to_throat_area_ratio=area_ratio,
    )
    candidate_case = replace(case, nozzle=nozzle)
    # thrust_multiplier stays 1.0 here (not scenario.propulsion_thrust_multiplier)
    # so `point.net_thrust_n` below is the raw, unscaled ramjet result -- reported
    # separately from `available_thrust_n`, which applies the scenario multiplier
    # explicitly, matching this function's prior two-value convention.
    propulsion_scenario = PropulsionScenario(
        scenario.name,
        ramjet_total_pressure_recovery_override=scenario.ramjet_total_pressure_recovery,
    )
    point = evaluate_propulsion_map_point(
        candidate_case,
        case.mission.peak_mach,
        case.mission.speed_run_altitude_msl_m,
        RAMJET_MODE,
        scenario=propulsion_scenario,
    )
    atmosphere = standard_atmosphere(case.mission.speed_run_altitude_msl_m)
    speed_m_per_s = case.mission.peak_mach * atmosphere.speed_of_sound_m_per_s
    dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * speed_m_per_s**2
    drag_n = (
        dynamic_pressure_pa
        * geometrically_scaled_drag_area_target_m2(case, body_diameter_m)
        * scenario.drag_multiplier
    )
    available_thrust_n = point.net_thrust_n * scenario.propulsion_thrust_multiplier
    excess_thrust_n = available_thrust_n - drag_n
    evaluated_mass_kg = current_mass_kg + scenario.mass_growth_kg
    mass_margin_kg = case.requirements.maximum_takeoff_mass_kg - evaluated_mass_kg
    passes_operability = point.self_sustaining_status
    status = list(point.validity_flags)
    if mass_margin_kg < 0.0:
        status.append("scenario_mass_exceeds_requirement")
    if excess_thrust_n < scenario.required_excess_thrust_n:
        status.append("scenario_excess_thrust_below_required_budget")
    if not passes_operability:
        status.append("scenario_ramjet_not_self_sustaining_candidate")
    endurance_s = (
        case.mission.ramjet_speed_run_fuel_budget_kg / point.fuel_mass_flow_kg_per_s
        if point.fuel_mass_flow_kg_per_s > 0.0
        else None
    )
    return RobustnessScenarioResult(
        scenario=scenario.name,
        ramjet_total_pressure_recovery=scenario.ramjet_total_pressure_recovery,
        propulsion_thrust_multiplier=scenario.propulsion_thrust_multiplier,
        drag_multiplier=scenario.drag_multiplier,
        evaluated_mass_kg=evaluated_mass_kg,
        mass_margin_to_requirement_kg=mass_margin_kg,
        ramjet_net_thrust_n=point.net_thrust_n,
        available_thrust_n=available_thrust_n,
        drag_n=drag_n,
        excess_thrust_n=excess_thrust_n,
        required_excess_thrust_n=scenario.required_excess_thrust_n,
        inlet_spillage_fraction=point.spilled_mass_flow_fraction,
        fuel_mass_flow_kg_per_s=point.fuel_mass_flow_kg_per_s,
        full_throttle_fuel_endurance_s=endurance_s,
        passes_mass=mass_margin_kg >= 0.0,
        passes_excess_thrust=excess_thrust_n >= scenario.required_excess_thrust_n,
        passes_operability_flags=passes_operability,
        required_for_candidate_selection=scenario.required_for_candidate_selection,
        status=tuple(status),
    )


def run_robustness_trade(
    case: ReferenceCase,
    robustness_path: str | Path,
) -> RobustnessTradeResult:
    data = _load_yaml(robustness_path)
    mass_budget = summarize_mass_budget(case, robustness_path)
    if not mass_budget.current_mass_matches_case:
        raise ValueError("mass budget current total does not match case initial mass")
    scenarios = _load_scenarios(data)
    objective = _mapping(data, "objective")
    sweep = _mapping(data, "sweep")
    body_values = tuple(float(value) for value in sweep["body_diameters_m"])
    throat_values = tuple(float(value) for value in sweep["throat_diameters_m"])
    area_ratios = tuple(float(value) for value in sweep["exit_to_throat_area_ratios"])
    if not body_values or not throat_values or not area_ratios:
        raise ValueError("robustness sweep arrays cannot be empty")

    points: list[CandidateRobustnessPoint] = []
    for body_diameter_m in body_values:
        for throat_diameter_m in throat_values:
            for area_ratio in area_ratios:
                shared = evaluate_shared_nozzle_trade(
                    case,
                    body_diameter_m,
                    throat_diameter_m,
                    area_ratio,
                    propulsion_derate_fraction=0.0,
                    pulsejet_warmup_s=float(sweep["pulsejet_warmup_s"]),
                    pulsejet_measurement_s=float(sweep["pulsejet_measurement_s"]),
                    pulsejet_time_step_s=float(sweep["pulsejet_time_step_s"]),
                )
                scenario_results = tuple(
                    _evaluate_scenario(
                        case,
                        body_diameter_m,
                        throat_diameter_m,
                        area_ratio,
                        scenario,
                        mass_budget.current_total_mass_kg,
                    )
                    for scenario in scenarios
                )
                required_results = tuple(
                    result
                    for result in scenario_results
                    if result.required_for_candidate_selection
                )
                required_pass = (
                    shared.packageable_with_configured_allowances
                    and shared.pulsejet_mean_net_thrust_n > 0.0
                    and all(
                        result.passes_mass
                        and result.passes_excess_thrust
                        and result.passes_operability_flags
                        for result in required_results
                    )
                )
                minimum_required_margin_n = min(
                    result.excess_thrust_n - result.required_excess_thrust_n
                    for result in required_results
                )
                adverse_margin_n = min(
                    result.excess_thrust_n
                    for result in scenario_results
                    if not result.required_for_candidate_selection
                )
                score = 0.0
                if not required_pass:
                    score -= float(objective["required_scenario_failure_penalty"])
                for result in scenario_results:
                    if result.mass_margin_to_requirement_kg < 0.0:
                        score += (
                            result.mass_margin_to_requirement_kg
                            * float(objective["mass_over_limit_penalty_per_kg"])
                        )
                    if result.required_for_candidate_selection:
                        shortfall_n = max(
                            result.required_excess_thrust_n - result.excess_thrust_n,
                            0.0,
                        )
                        score -= shortfall_n * float(
                            objective["excess_thrust_shortfall_penalty_per_n"]
                        )
                score += max(minimum_required_margin_n, 0.0) * float(
                    objective["minimum_required_excess_thrust_reward_per_n"]
                )
                score += adverse_margin_n * float(
                    objective["adverse_excess_thrust_reward_per_n"]
                )
                score += min(
                    result.mass_margin_to_requirement_kg for result in scenario_results
                ) * float(objective["mass_margin_reward_per_kg"])
                score += shared.available_minimum_radial_packaging_margin_m * float(
                    objective["radial_packaging_margin_reward_per_m"]
                )
                score -= max(
                    result.inlet_spillage_fraction for result in required_results
                ) * float(objective["inlet_spillage_penalty_per_fraction"])
                score -= throat_diameter_m * float(
                    objective["throat_diameter_penalty_per_m"]
                )
                score -= body_diameter_m * float(
                    objective["body_diameter_penalty_per_m"]
                )
                score += shared.pulsejet_mean_net_thrust_to_weight * float(
                    objective["pulsejet_thrust_to_weight_reward"]
                )
                status: list[str] = []
                if not shared.packageable_with_configured_allowances:
                    status.append("candidate_not_packageable_with_configured_allowances")
                if not required_pass:
                    status.append("candidate_fails_required_robustness_scenario")
                if any(
                    not result.passes_excess_thrust
                    for result in scenario_results
                    if not result.required_for_candidate_selection
                ):
                    status.append("candidate_fails_informational_adverse_scenario")
                status.append("static_peak_mach_screen_not_mission_closure")
                points.append(
                    CandidateRobustnessPoint(
                        body_diameter_m=body_diameter_m,
                        throat_diameter_m=throat_diameter_m,
                        exit_to_throat_area_ratio=area_ratio,
                        exit_diameter_m=throat_diameter_m * sqrt(area_ratio),
                        radial_packaging_margin_m=(
                            shared.available_minimum_radial_packaging_margin_m
                        ),
                        packageable_with_configured_allowances=(
                            shared.packageable_with_configured_allowances
                        ),
                        pulsejet_mean_net_thrust_n=shared.pulsejet_mean_net_thrust_n,
                        pulsejet_mean_net_thrust_to_weight=(
                            shared.pulsejet_mean_net_thrust_n
                            / (mass_budget.current_total_mass_kg * G0_M_PER_S2)
                        ),
                        scenarios=scenario_results,
                        required_scenarios_pass=required_pass,
                        objective_score=score,
                        status=tuple(status),
                    )
                )

    feasible = [point for point in points if point.required_scenarios_pass]
    selected = max(feasible, key=lambda point: point.objective_score) if feasible else None
    return RobustnessTradeResult(
        mass_budget=mass_budget,
        selection_rule=(
            "highest visible weighted score among points passing packageability, "
            "positive pulsejet thrust, and every scenario marked required for selection"
        ),
        selected_candidate=selected,
        points=tuple(points),
    )
