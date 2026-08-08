"""Transparent aerodynamic budget model pending solver-backed tables."""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees, isclose, radians, sqrt
from typing import Callable, Protocol

from .aero_tables import VSPAeroCoefficientTable
from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .config import ReferenceCase
from .flight import PointMassState
from .ramjet import RamjetResult


@dataclass(frozen=True)
class AerodynamicForces:
    mach: float
    dynamic_pressure_pa: float
    lift_coefficient: float
    parasitic_drag_area_m2: float
    induced_drag_area_m2: float
    total_drag_area_m2: float
    lift_n: float
    drag_n: float
    provisional_budget_model: bool = True
    inviscid_external_drag_area_m2: float = 0.0
    supplementary_drag_area_m2: float = 0.0
    solver_table_clamped: bool = False


class LongitudinalAeroModel(Protocol):
    status_tags: tuple[str, ...]

    def forces(
        self,
        state: PointMassState,
        angle_of_attack_rad: float,
    ) -> AerodynamicForces: ...

    def angle_of_attack_for_lift_coefficient(
        self,
        mach: float,
        lift_coefficient: float,
    ) -> float: ...

    def best_glide_angle_of_attack_rad(self, mach: float) -> float: ...


@dataclass(frozen=True)
class LaunchLiftScreen:
    release_speed_m_per_s: float
    altitude_m: float
    maximum_angle_of_attack_deg: float
    maximum_lift_coefficient: float
    configured_reference_area_m2: float
    required_reference_area_m2: float
    configured_to_required_area_ratio: float
    minimum_level_flight_speed_m_per_s: float
    lift_closes_at_release: bool
    provisional_linear_lift_model: bool = True


class BudgetAeroModel:
    """Blend an explicit subsonic polar into the prior Mach-peak CdS budget.

    The zero-lift subsonic drag area comes directly from ``flight``. The transonic
    endpoint comes directly from the body-diameter-scaled drag-area budget. A cubic
    smoothstep only connects those two visible endpoints; it is not a VSPAERO result.
    """

    def __init__(self, case: ReferenceCase) -> None:
        self.case = case
        self.status_tags = ("budget_aerodynamics_not_solver_backed",)
        self.subsonic_parasitic_drag_area_m2 = (
            case.flight.zero_lift_drag_coefficient * case.flight.reference_area_m2
        )
        self.peak_mach_parasitic_drag_area_m2 = (
            case.vehicle.peak_mach_drag_area_ceiling_m2
            * (
                case.vehicle.body_diameter_m
                / case.vehicle.drag_area_reference_body_diameter_m
            )
            ** 2
        )
        if (
            self.peak_mach_parasitic_drag_area_m2
            < self.subsonic_parasitic_drag_area_m2
        ):
            raise ValueError(
                "peak-Mach drag-area budget cannot be below the configured "
                "subsonic zero-lift drag area"
            )

    def parasitic_drag_area_m2(self, mach: float) -> float:
        if mach < 0.0:
            raise ValueError("Mach cannot be negative")
        start = self.case.mission_simulation.drag_rise_start_mach
        end = self.case.mission_simulation.drag_rise_end_mach
        if mach <= start:
            blend = 0.0
        elif mach >= end:
            blend = 1.0
        else:
            coordinate = (mach - start) / (end - start)
            blend = coordinate**2 * (3.0 - 2.0 * coordinate)
        return self.subsonic_parasitic_drag_area_m2 + blend * (
            self.peak_mach_parasitic_drag_area_m2
            - self.subsonic_parasitic_drag_area_m2
        )

    def forces(
        self,
        state: PointMassState,
        angle_of_attack_rad: float,
    ) -> AerodynamicForces:
        atmosphere = standard_atmosphere(state.altitude_m)
        mach = state.speed_m_per_s / atmosphere.speed_of_sound_m_per_s
        dynamic_pressure_pa = (
            0.5 * atmosphere.density_kg_per_m3 * state.speed_m_per_s**2
        )
        lift_coefficient = (
            self.case.flight.lift_curve_slope_per_rad * angle_of_attack_rad
        )
        induced_drag_area_m2 = (
            self.case.flight.induced_drag_factor
            * lift_coefficient**2
            * self.case.flight.reference_area_m2
        )
        parasitic_drag_area_m2 = self.parasitic_drag_area_m2(mach)
        total_drag_area_m2 = parasitic_drag_area_m2 + induced_drag_area_m2
        return AerodynamicForces(
            mach=mach,
            dynamic_pressure_pa=dynamic_pressure_pa,
            lift_coefficient=lift_coefficient,
            parasitic_drag_area_m2=parasitic_drag_area_m2,
            induced_drag_area_m2=induced_drag_area_m2,
            total_drag_area_m2=total_drag_area_m2,
            lift_n=(
                dynamic_pressure_pa
                * self.case.flight.reference_area_m2
                * lift_coefficient
            ),
            drag_n=dynamic_pressure_pa * total_drag_area_m2,
        )

    def best_glide_angle_of_attack_rad(self, mach: float) -> float:
        parasite_cd = (
            self.parasitic_drag_area_m2(mach)
            / self.case.flight.reference_area_m2
        )
        optimum_lift_coefficient = sqrt(
            parasite_cd / self.case.flight.induced_drag_factor
        )
        return optimum_lift_coefficient / self.case.flight.lift_curve_slope_per_rad

    def angle_of_attack_for_lift_coefficient(
        self,
        mach: float,
        lift_coefficient: float,
    ) -> float:
        del mach
        return lift_coefficient / self.case.flight.lift_curve_slope_per_rad


class SupplementedVSPAeroModel:
    """Use VSPAERO lift/inviscid drag plus a separate drag-area model.

    The supplementary function must return the viscous, wave, base, and other
    external drag area that the caller wants added. Inlet spillage remains a
    propulsion-installation term in the mission model and is not added here.
    """

    def __init__(
        self,
        case: ReferenceCase,
        coefficient_table: VSPAeroCoefficientTable,
        supplementary_drag_area_model: Callable[[float], float],
        *,
        supplementary_drag_status: str,
        allow_nonlive_table: bool = False,
    ) -> None:
        if coefficient_table.case_name != case.name:
            raise ValueError(
                "VSPAERO table case name does not match the active configuration"
            )
        if not isclose(
            coefficient_table.reference_area_m2,
            case.flight.reference_area_m2,
            rel_tol=1.0e-9,
            abs_tol=1.0e-12,
        ):
            raise ValueError(
                "VSPAERO table reference area does not match the flight model"
            )
        if not supplementary_drag_status.strip():
            raise ValueError("supplementary drag provenance cannot be empty")
        if not coefficient_table.live_solver_run and not allow_nonlive_table:
            raise ValueError(
                "non-live VSPAERO tables require an explicit test-only override"
            )
        self.case = case
        self.coefficient_table = coefficient_table
        self.supplementary_drag_area_model = supplementary_drag_area_model
        self.status_tags = (
            "vspaero_inviscid_external_aerodynamics_loaded",
            supplementary_drag_status,
            *(
                ("nonlive_vspaero_table_explicitly_allowed_for_testing",)
                if not coefficient_table.live_solver_run
                else ()
            ),
        )

    def forces(
        self,
        state: PointMassState,
        angle_of_attack_rad: float,
    ) -> AerodynamicForces:
        atmosphere = standard_atmosphere(state.altitude_m)
        mach = state.speed_m_per_s / atmosphere.speed_of_sound_m_per_s
        dynamic_pressure_pa = (
            0.5 * atmosphere.density_kg_per_m3 * state.speed_m_per_s**2
        )
        coefficients = self.coefficient_table.evaluate(
            mach,
            degrees(angle_of_attack_rad),
        )
        if coefficients.drag_coefficient_inviscid < -1.0e-9:
            raise ValueError("VSPAERO inviscid drag coefficient cannot be negative")
        inviscid_drag_area_m2 = (
            max(coefficients.drag_coefficient_inviscid, 0.0)
            * self.case.flight.reference_area_m2
        )
        supplementary_drag_area_m2 = self.supplementary_drag_area_model(mach)
        if supplementary_drag_area_m2 < 0.0:
            raise ValueError("supplementary drag area cannot be negative")
        total_drag_area_m2 = (
            inviscid_drag_area_m2 + supplementary_drag_area_m2
        )
        return AerodynamicForces(
            mach=mach,
            dynamic_pressure_pa=dynamic_pressure_pa,
            lift_coefficient=coefficients.lift_coefficient,
            parasitic_drag_area_m2=supplementary_drag_area_m2,
            induced_drag_area_m2=0.0,
            total_drag_area_m2=total_drag_area_m2,
            lift_n=(
                dynamic_pressure_pa
                * self.case.flight.reference_area_m2
                * coefficients.lift_coefficient
            ),
            drag_n=dynamic_pressure_pa * total_drag_area_m2,
            provisional_budget_model=False,
            inviscid_external_drag_area_m2=inviscid_drag_area_m2,
            supplementary_drag_area_m2=supplementary_drag_area_m2,
            solver_table_clamped=coefficients.clamped_to_table_boundary,
        )

    def angle_of_attack_for_lift_coefficient(
        self,
        mach: float,
        lift_coefficient: float,
    ) -> float:
        return radians(
            self.coefficient_table.alpha_deg_for_lift_coefficient(
                mach,
                lift_coefficient,
            )
        )

    def best_glide_angle_of_attack_rad(self, mach: float) -> float:
        alpha_min = self.coefficient_table.alpha_deg_values[0]
        alpha_max = self.coefficient_table.alpha_deg_values[-1]
        best_alpha_deg = alpha_min
        best_ratio = float("-inf")
        for index in range(81):
            alpha_deg = alpha_min + index * (alpha_max - alpha_min) / 80.0
            coefficients = self.coefficient_table.evaluate(mach, alpha_deg)
            supplementary_area_m2 = self.supplementary_drag_area_model(mach)
            total_drag_coefficient = (
                coefficients.drag_coefficient_inviscid
                + supplementary_area_m2 / self.case.flight.reference_area_m2
            )
            if total_drag_coefficient <= 0.0:
                continue
            ratio = coefficients.lift_coefficient / total_drag_coefficient
            if ratio > best_ratio:
                best_ratio = ratio
                best_alpha_deg = alpha_deg
        return radians(best_alpha_deg)


def launch_lift_screen(case: ReferenceCase, release_speed_m_per_s: float) -> LaunchLiftScreen:
    if release_speed_m_per_s <= 0.0:
        raise ValueError("release speed must be positive")
    atmosphere = standard_atmosphere(case.mission.field_elevation_msl_m)
    maximum_angle_of_attack_rad = radians(
        case.mission_simulation.maximum_angle_of_attack_deg
    )
    maximum_lift_coefficient = (
        case.flight.lift_curve_slope_per_rad * maximum_angle_of_attack_rad
    )
    if maximum_lift_coefficient <= 0.0:
        raise ValueError("maximum configured lift coefficient must be positive")
    required_area_m2 = (
        2.0
        * case.flight.initial_mass_kg
        * G0_M_PER_S2
        / (
            atmosphere.density_kg_per_m3
            * release_speed_m_per_s**2
            * maximum_lift_coefficient
        )
    )
    minimum_level_speed_m_per_s = sqrt(
        2.0
        * case.flight.initial_mass_kg
        * G0_M_PER_S2
        / (
            atmosphere.density_kg_per_m3
            * case.flight.reference_area_m2
            * maximum_lift_coefficient
        )
    )
    area_ratio = case.flight.reference_area_m2 / required_area_m2
    return LaunchLiftScreen(
        release_speed_m_per_s=release_speed_m_per_s,
        altitude_m=case.mission.field_elevation_msl_m,
        maximum_angle_of_attack_deg=(
            case.mission_simulation.maximum_angle_of_attack_deg
        ),
        maximum_lift_coefficient=maximum_lift_coefficient,
        configured_reference_area_m2=case.flight.reference_area_m2,
        required_reference_area_m2=required_area_m2,
        configured_to_required_area_ratio=area_ratio,
        minimum_level_flight_speed_m_per_s=minimum_level_speed_m_per_s,
        lift_closes_at_release=area_ratio >= 1.0,
    )


def ramjet_spillage_momentum_scale_n(
    result: RamjetResult,
    altitude_m: float,
) -> float:
    """Return the uncaptured stream momentum scale before an empirical factor."""

    atmosphere = standard_atmosphere(altitude_m)
    speed_m_per_s = result.mach * atmosphere.speed_of_sound_m_per_s
    spilled_air_mass_flow_kg_per_s = max(
        result.potential_captured_air_mass_flow_kg_per_s
        - result.air_mass_flow_kg_per_s,
        0.0,
    )
    return spilled_air_mass_flow_kg_per_s * speed_m_per_s
