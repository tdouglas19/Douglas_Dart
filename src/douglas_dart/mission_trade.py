"""Repeatable mission-policy trades without a hidden aggregate score."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import product

from .config import ReferenceCase
from .mission import MissionPolicy, MissionSimulator
from .performance_maps import RectilinearEngineMap


@dataclass(frozen=True)
class MissionTradePoint:
    body_diameter_m: float
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    release_speed_m_per_s: float
    top_of_climb_altitude_m: float
    climb_flight_path_angle_deg: float
    dive_flight_path_angle_deg: float
    ramjet_handoff_mach: float
    forced_ramjet_below_self_sustaining: bool
    ramjet_spillage_drag_momentum_fraction: float
    initial_mass_kg: float
    loaded_fuel_mass_kg: float
    pulsejet_fuel_allocation_kg: float
    ramjet_fuel_allocation_kg: float
    launch_lift_closes: bool
    termination_reason: str
    maximum_altitude_m: float
    maximum_mach: float
    minimum_mach: float
    minimum_speed_m_per_s: float
    time_above_mach_one_s: float
    time_at_angle_of_attack_limit_s: float
    pulsejet_fuel_used_kg: float
    ramjet_fuel_used_kg: float
    unused_fuel_kg: float
    minimum_ramjet_full_throttle_margin_n: float | None
    minimum_ramjet_acceleration_full_throttle_margin_n: float | None
    minimum_ramjet_run_full_throttle_margin_n: float | None
    top_of_climb_time_s: float | None
    ramjet_handoff_time_s: float | None
    ramjet_run_end_time_s: float | None
    reached_target_peak_mach: bool
    exceeded_minimum_supersonic_duration: bool
    full_mission_numerically_closes: bool
    forced_operability_remains_unvalidated: bool
    spillage_drag_remains_unvalidated: bool
    status: tuple[str, ...]
    numerical_reference_only: bool = True


def mission_trade_sweep(
    case: ReferenceCase,
    pulsejet_map: RectilinearEngineMap,
    *,
    release_speeds_m_per_s: tuple[float, ...],
    top_of_climb_altitudes_m: tuple[float, ...],
    climb_flight_path_angles_deg: tuple[float, ...],
    dive_flight_path_angles_deg: tuple[float, ...],
    ramjet_fuel_allocations_kg: tuple[float, ...],
    loaded_fuel_masses_kg: tuple[float, ...] | None = None,
    ramjet_handoff_mach: float,
    allow_forced_ramjet_below_self_sustaining: bool,
    ramjet_spillage_drag_momentum_fractions: tuple[float, ...] = (0.0,),
    integration_time_step_s: float | None = None,
) -> list[MissionTradePoint]:
    """Evaluate every explicit policy combination and return unranked points."""

    selected_loaded_fuel_masses_kg = (
        (case.mission.loaded_fuel_mass_kg,)
        if loaded_fuel_masses_kg is None
        else loaded_fuel_masses_kg
    )
    arrays = (
        release_speeds_m_per_s,
        top_of_climb_altitudes_m,
        climb_flight_path_angles_deg,
        dive_flight_path_angles_deg,
        ramjet_fuel_allocations_kg,
        ramjet_spillage_drag_momentum_fractions,
        selected_loaded_fuel_masses_kg,
    )
    if any(not values for values in arrays):
        raise ValueError("mission trade arrays cannot be empty")
    dry_mass_kg = case.flight.initial_mass_kg - case.mission.loaded_fuel_mass_kg
    if dry_mass_kg <= 0.0:
        raise ValueError("mission trade inferred dry mass must be positive")
    points: list[MissionTradePoint] = []
    combinations = product(
        selected_loaded_fuel_masses_kg,
        ramjet_fuel_allocations_kg,
        release_speeds_m_per_s,
        top_of_climb_altitudes_m,
        climb_flight_path_angles_deg,
        dive_flight_path_angles_deg,
        ramjet_spillage_drag_momentum_fractions,
    )
    for (
        loaded_fuel_mass_kg,
        ramjet_fuel_allocation_kg,
        release_speed_m_per_s,
        top_of_climb_altitude_m,
        climb_angle_deg,
        dive_angle_deg,
        spillage_fraction,
    ) in combinations:
        if loaded_fuel_mass_kg <= 0.0:
            raise ValueError("loaded fuel mass must be positive")
        initial_mass_kg = dry_mass_kg + loaded_fuel_mass_kg
        if initial_mass_kg > case.requirements.maximum_takeoff_mass_kg:
            raise ValueError("loaded fuel trade point exceeds maximum takeoff mass")
        if not 0.0 < ramjet_fuel_allocation_kg <= loaded_fuel_mass_kg:
            raise ValueError(
                "ramjet fuel allocation lies outside the loaded fuel mass"
            )
        point_case = replace(
            case,
            flight=replace(case.flight, initial_mass_kg=initial_mass_kg),
            mission=replace(
                case.mission,
                loaded_fuel_mass_kg=loaded_fuel_mass_kg,
                ramjet_speed_run_fuel_budget_kg=ramjet_fuel_allocation_kg,
            ),
        )
        pulsejet_fuel_allocation_kg = (
            loaded_fuel_mass_kg - ramjet_fuel_allocation_kg
        )
        policy = MissionPolicy.from_case(
            point_case,
            top_of_climb_altitude_m=top_of_climb_altitude_m,
            climb_flight_path_angle_deg=climb_angle_deg,
            dive_flight_path_angle_deg=dive_angle_deg,
            ramjet_handoff_mach=ramjet_handoff_mach,
            allow_forced_ramjet_below_self_sustaining=(
                allow_forced_ramjet_below_self_sustaining
            ),
            ramjet_spillage_drag_momentum_fraction=spillage_fraction,
        )
        result = MissionSimulator(
            point_case,
            pulsejet_map,
            policy=policy,
        ).run(
            release_speed_m_per_s=release_speed_m_per_s,
            integration_time_step_s=integration_time_step_s,
        )
        summary = result.summary
        unused_fuel_kg = loaded_fuel_mass_kg - summary.total_fuel_used_kg
        points.append(
            MissionTradePoint(
                body_diameter_m=point_case.vehicle.body_diameter_m,
                throat_diameter_m=point_case.nozzle.throat_diameter_m,
                exit_to_throat_area_ratio=(
                    point_case.nozzle.exit_to_throat_area_ratio
                ),
                release_speed_m_per_s=release_speed_m_per_s,
                top_of_climb_altitude_m=top_of_climb_altitude_m,
                climb_flight_path_angle_deg=climb_angle_deg,
                dive_flight_path_angle_deg=dive_angle_deg,
                ramjet_handoff_mach=ramjet_handoff_mach,
                forced_ramjet_below_self_sustaining=(
                    allow_forced_ramjet_below_self_sustaining
                ),
                ramjet_spillage_drag_momentum_fraction=spillage_fraction,
                initial_mass_kg=point_case.flight.initial_mass_kg,
                loaded_fuel_mass_kg=loaded_fuel_mass_kg,
                pulsejet_fuel_allocation_kg=pulsejet_fuel_allocation_kg,
                ramjet_fuel_allocation_kg=ramjet_fuel_allocation_kg,
                launch_lift_closes=result.launch_screen.lift_closes_at_release,
                termination_reason=summary.termination_reason,
                maximum_altitude_m=summary.maximum_altitude_m,
                maximum_mach=summary.maximum_mach,
                minimum_mach=summary.minimum_mach,
                minimum_speed_m_per_s=summary.minimum_speed_m_per_s,
                time_above_mach_one_s=summary.time_above_mach_one_s,
                time_at_angle_of_attack_limit_s=(
                    summary.time_at_angle_of_attack_limit_s
                ),
                pulsejet_fuel_used_kg=summary.pulsejet_fuel_used_kg,
                ramjet_fuel_used_kg=summary.ramjet_fuel_used_kg,
                unused_fuel_kg=unused_fuel_kg,
                minimum_ramjet_full_throttle_margin_n=(
                    summary.minimum_ramjet_full_throttle_margin_n
                ),
                minimum_ramjet_acceleration_full_throttle_margin_n=(
                    summary.minimum_ramjet_acceleration_full_throttle_margin_n
                ),
                minimum_ramjet_run_full_throttle_margin_n=(
                    summary.minimum_ramjet_run_full_throttle_margin_n
                ),
                top_of_climb_time_s=summary.top_of_climb_time_s,
                ramjet_handoff_time_s=summary.ramjet_handoff_time_s,
                ramjet_run_end_time_s=summary.ramjet_run_end_time_s,
                reached_target_peak_mach=summary.reached_target_peak_mach,
                exceeded_minimum_supersonic_duration=(
                    summary.exceeded_minimum_supersonic_duration
                ),
                full_mission_numerically_closes=(
                    summary.full_mission_numerically_closes
                ),
                forced_operability_remains_unvalidated=(
                    allow_forced_ramjet_below_self_sustaining
                ),
                spillage_drag_remains_unvalidated=True,
                status=summary.status,
            )
        )
    return points
