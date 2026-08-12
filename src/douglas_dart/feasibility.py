"""Level 0 hand-calculation feasibility bounds (docs/design_workflow.md Gate 1).

These are deliberately simple, closed-form or single-point calculations, not a
trajectory simulation. Per docs/design_workflow.md: "These calculations are
not intended to select a final design. They should reject impossible
concepts, identify required margins, and define reasonable optimizer
bounds." Every function here is a hand-calc analogue of something a real
propulsion/aero engineer would sanity-check on paper before trusting a
solver's output.

This module calls `evaluate_ramjet`/`PulsejetSimulator` directly rather than
through a single authoritative propulsion-map interface, because that
interface (docs/design_workflow.md Gate 2) does not exist yet. When
`propulsion_map.py` is built, this module should migrate to it like every
other consumer.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .config import ReferenceCase
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet


@dataclass(frozen=True)
class SledLaunchFeasibility:
    """Can the sled reach the target release speed within its rail length?

    This is the one hard, user-stated requirement behind
    ``sled_release_speed_*``: reach release speed within a reasonable
    distance. ``required_acceleration_g`` is always reported so a human can
    judge "reasonable" -- it is compared against
    ``MissionConfig.sled_launch_acceleration_g`` only because that is the
    only bound configured right now, and that bound is itself an explicitly
    UNSOURCED placeholder (see ``MissionConfig``'s docstring), not a
    validated structural or physiological ceiling.
    """

    release_speed_m_per_s: float
    rail_length_m: float
    required_acceleration_m_per_s2: float
    required_acceleration_g: float
    configured_acceleration_ceiling_g: float
    within_configured_ceiling: bool
    required_rail_length_at_ceiling_m: float


def sled_launch_feasibility(
    case: ReferenceCase, release_speed_m_per_s: float | None = None
) -> SledLaunchFeasibility:
    """required a = v^2 / (2 L); constant-acceleration rail launch, no drag/friction loss.

    This is a lower bound on required acceleration (a coasting, non-ideal
    sled would need more), so it should not be read as "this is exactly what
    the sled needs to do," only "the sled needs at least this much."
    """

    release_speed_m_per_s = (
        case.mission.sled_release_speed_max_m_per_s
        if release_speed_m_per_s is None
        else release_speed_m_per_s
    )
    rail_length_m = case.mission.sled_rail_length_m
    required_acceleration_m_per_s2 = release_speed_m_per_s**2 / (2.0 * rail_length_m)
    required_acceleration_g = required_acceleration_m_per_s2 / G0_M_PER_S2
    ceiling_g = case.mission.sled_launch_acceleration_g
    required_rail_length_at_ceiling_m = release_speed_m_per_s**2 / (2.0 * ceiling_g * G0_M_PER_S2)
    return SledLaunchFeasibility(
        release_speed_m_per_s=release_speed_m_per_s,
        rail_length_m=rail_length_m,
        required_acceleration_m_per_s2=required_acceleration_m_per_s2,
        required_acceleration_g=required_acceleration_g,
        configured_acceleration_ceiling_g=ceiling_g,
        within_configured_ceiling=required_acceleration_g <= ceiling_g,
        required_rail_length_at_ceiling_m=required_rail_length_at_ceiling_m,
    )


@dataclass(frozen=True)
class StallSpeedBounds:
    """Sled-release stall-speed bound (docs/design_workflow.md Level 0)."""

    stall_speed_m_per_s: float
    sled_release_speed_min_m_per_s: float
    sled_release_speed_max_m_per_s: float
    release_margin_min_fraction: float
    passes: bool
    note: str


def stall_speed_bounds(case: ReferenceCase) -> StallSpeedBounds:
    """V_stall = sqrt(2W / (rho S CL_max)) at max takeoff mass, field elevation."""

    atmosphere = standard_atmosphere(case.mission.field_elevation_msl_m)
    weight_n = case.requirements.maximum_takeoff_mass_kg * G0_M_PER_S2
    denominator = (
        atmosphere.density_kg_per_m3
        * case.flight.reference_area_m2
        * case.flight.maximum_lift_coefficient
    )
    stall_speed_m_per_s = sqrt(2.0 * weight_n / denominator)
    margin_fraction = (
        case.mission.sled_release_speed_min_m_per_s / stall_speed_m_per_s - 1.0
    )
    passes = case.mission.sled_release_speed_min_m_per_s > stall_speed_m_per_s
    return StallSpeedBounds(
        stall_speed_m_per_s=stall_speed_m_per_s,
        sled_release_speed_min_m_per_s=case.mission.sled_release_speed_min_m_per_s,
        sled_release_speed_max_m_per_s=case.mission.sled_release_speed_max_m_per_s,
        release_margin_min_fraction=margin_fraction,
        passes=passes,
        note=(
            "at maximum_takeoff_mass_kg, field elevation, CL_max; a real release "
            "will be lighter and see ground effect, so this is a conservative bound"
        ),
    )


@dataclass(frozen=True)
class RequiredLiftArea:
    """Required reference area for a target CL_max at the slowest flight speed."""

    required_area_m2: float
    configured_area_m2: float
    passes: bool


def required_lift_area(case: ReferenceCase, cl_max: float | None = None) -> RequiredLiftArea:
    """S_required = 2W / (rho V^2 CL_max) at the minimum sled-release speed."""

    cl_max = case.flight.maximum_lift_coefficient if cl_max is None else cl_max
    atmosphere = standard_atmosphere(case.mission.field_elevation_msl_m)
    weight_n = case.requirements.maximum_takeoff_mass_kg * G0_M_PER_S2
    required_area_m2 = (2.0 * weight_n) / (
        atmosphere.density_kg_per_m3
        * case.mission.sled_release_speed_min_m_per_s**2
        * cl_max
    )
    return RequiredLiftArea(
        required_area_m2=required_area_m2,
        configured_area_m2=case.flight.reference_area_m2,
        passes=case.flight.reference_area_m2 >= required_area_m2,
    )


@dataclass(frozen=True)
class ThrustToWeightPoint:
    mach: float
    altitude_m: float
    mode: str
    net_thrust_n: float
    weight_n: float
    thrust_to_weight: float
    dynamic_pressure_pa: float
    excess_power_w: float | None


def thrust_to_weight_sweep(
    case: ReferenceCase,
    mach_values: tuple[float, ...] = (0.2, 0.5, 0.8, 1.0, 1.1),
    *,
    mass_kg: float | None = None,
) -> list[ThrustToWeightPoint]:
    """T/W and excess power (T-D)*V at a handful of Mach points.

    Below the configured ramjet self-sustaining Mach, uses the pulsejet static
    thrust table at sea level; at/above it, uses the ramjet steady-state model
    at the mission speed-run altitude. Drag uses the same Mach-indexed drag
    area the trajectory model uses (docs/drag.py), at zero lift (excess-power
    bound, not a trimmed climb estimate).
    """

    from .drag import mach_indexed_zero_lift_drag_area_m2

    mass_kg = case.flight.initial_mass_kg if mass_kg is None else mass_kg
    weight_n = mass_kg * G0_M_PER_S2
    points: list[ThrustToWeightPoint] = []
    for mach in mach_values:
        if mach < case.ramjet.minimum_self_sustaining_mach:
            altitude_m = 0.0
            atmosphere = standard_atmosphere(altitude_m)
            simulator = PulsejetSimulator(
                case.pulsejet, case.selector, case.nozzle, case.fuel, altitude_m, mach
            )
            summary = summarize_pulsejet(
                simulator.run(0.5, 0.00002), minimum_time_s=0.25
            )
            net_thrust_n = summary.mean_net_thrust_n
            mode = "pulsejet"
        else:
            altitude_m = case.mission.speed_run_altitude_msl_m
            atmosphere = standard_atmosphere(altitude_m)
            result = evaluate_ramjet(
                case.ramjet, case.selector, case.nozzle, case.fuel, altitude_m, mach
            )
            net_thrust_n = result.net_thrust_n
            mode = "ramjet"

        velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
        dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * velocity_m_per_s**2
        drag_area_m2 = mach_indexed_zero_lift_drag_area_m2(case, mach)
        drag_n = dynamic_pressure_pa * drag_area_m2
        excess_power_w = (net_thrust_n - drag_n) * velocity_m_per_s if velocity_m_per_s > 0 else None
        points.append(
            ThrustToWeightPoint(
                mach=mach,
                altitude_m=altitude_m,
                mode=mode,
                net_thrust_n=net_thrust_n,
                weight_n=weight_n,
                thrust_to_weight=net_thrust_n / weight_n,
                dynamic_pressure_pa=dynamic_pressure_pa,
                excess_power_w=excess_power_w,
            )
        )
    return points


@dataclass(frozen=True)
class ClimbRateEstimate:
    mach: float
    climb_rate_m_per_s: float | None
    note: str


def climb_rate_estimates(points: list[ThrustToWeightPoint]) -> list[ClimbRateEstimate]:
    """Climb rate = excess power / weight (energy-height rate, not a trimmed climb)."""

    estimates = []
    for point in points:
        if point.excess_power_w is None:
            estimates.append(
                ClimbRateEstimate(mach=point.mach, climb_rate_m_per_s=None, note="zero speed")
            )
            continue
        estimates.append(
            ClimbRateEstimate(
                mach=point.mach,
                climb_rate_m_per_s=point.excess_power_w / point.weight_n,
                note="energy-height rate at zero lift/bank; a real climb trades some excess power for induced drag",
            )
        )
    return estimates


@dataclass(frozen=True)
class DiveEnergyBenefit:
    dive_height_m: float
    entry_speed_m_per_s: float
    exit_speed_m_per_s: float
    exit_mach_estimate: float


def dive_energy_benefit(
    case: ReferenceCase, dive_height_m: float, entry_speed_m_per_s: float
) -> DiveEnergyBenefit:
    """V_exit = sqrt(V_entry^2 + 2 g h), pure energy conversion (no drag loss).

    An upper bound on what a pre-transonic dive can buy in speed -- the real
    trajectory model (trajectory.py) integrates drag and thrust through the
    dive and will show a smaller gain; this bound is for sizing the dive
    height a search should even consider.
    """

    if dive_height_m < 0.0:
        raise ValueError("dive height cannot be negative")
    exit_speed_m_per_s = sqrt(entry_speed_m_per_s**2 + 2.0 * G0_M_PER_S2 * dive_height_m)
    exit_altitude_m = max(case.mission.field_elevation_msl_m, 0.0)
    atmosphere = standard_atmosphere(exit_altitude_m)
    return DiveEnergyBenefit(
        dive_height_m=dive_height_m,
        entry_speed_m_per_s=entry_speed_m_per_s,
        exit_speed_m_per_s=exit_speed_m_per_s,
        exit_mach_estimate=exit_speed_m_per_s / atmosphere.speed_of_sound_m_per_s,
    )


@dataclass(frozen=True)
class FuelEnergyBounds:
    loaded_fuel_mass_kg: float
    ramjet_fuel_budget_kg: float
    loaded_chemical_energy_j: float
    ramjet_budget_chemical_energy_j: float
    pulsejet_fuel_mass_kg: float
    pulsejet_endurance_s_bound: float | None
    ramjet_endurance_s_bound: float | None


def fuel_energy_and_endurance_bounds(case: ReferenceCase) -> FuelEnergyBounds:
    """Total chemical energy and a steady-state fuel-flow endurance bound per mode.

    Endurance bound = fuel mass / fuel flow at one representative operating
    point per mode -- an upper bound, since neither mode runs at exactly one
    throttle setting for the whole flight.
    """

    pulsejet_fuel_mass_kg = case.mission.loaded_fuel_mass_kg - case.mission.ramjet_speed_run_fuel_budget_kg
    loaded_energy_j = case.mission.loaded_fuel_mass_kg * case.fuel.lower_heating_value_j_per_kg
    ramjet_budget_energy_j = (
        case.mission.ramjet_speed_run_fuel_budget_kg * case.fuel.lower_heating_value_j_per_kg
    )

    pulsejet_endurance_s = None
    if pulsejet_fuel_mass_kg > 0.0:
        simulator = PulsejetSimulator(
            case.pulsejet, case.selector, case.nozzle, case.fuel, 0.0, 0.2
        )
        summary = summarize_pulsejet(simulator.run(0.5, 0.00002), minimum_time_s=0.25)
        if summary.mean_fuel_mass_flow_kg_per_s > 1e-9:
            pulsejet_endurance_s = pulsejet_fuel_mass_kg / summary.mean_fuel_mass_flow_kg_per_s

    ramjet_endurance_s = None
    if case.mission.ramjet_speed_run_fuel_budget_kg > 0.0:
        result = evaluate_ramjet(
            case.ramjet,
            case.selector,
            case.nozzle,
            case.fuel,
            case.mission.speed_run_altitude_msl_m,
            case.mission.peak_mach,
        )
        if result.fuel_mass_flow_kg_per_s > 1e-9:
            ramjet_endurance_s = (
                case.mission.ramjet_speed_run_fuel_budget_kg / result.fuel_mass_flow_kg_per_s
            )

    return FuelEnergyBounds(
        loaded_fuel_mass_kg=case.mission.loaded_fuel_mass_kg,
        ramjet_fuel_budget_kg=case.mission.ramjet_speed_run_fuel_budget_kg,
        loaded_chemical_energy_j=loaded_energy_j,
        ramjet_budget_chemical_energy_j=ramjet_budget_energy_j,
        pulsejet_fuel_mass_kg=pulsejet_fuel_mass_kg,
        pulsejet_endurance_s_bound=pulsejet_endurance_s,
        ramjet_endurance_s_bound=ramjet_endurance_s,
    )


@dataclass(frozen=True)
class PackagingBoundCheck:
    name: str
    passes: bool
    detail: str


def packaging_bounds(case: ReferenceCase) -> list[PackagingBoundCheck]:
    """Coarse "does it physically fit" checks -- not a real internal layout.

    See scripts/vehicle_cross_section.py's own disclaimer: no internal-layout
    solver exists in this repo. These are single-inequality sanity checks
    only (things that would make a design impossible, not things that make it
    good).
    """

    checks: list[PackagingBoundCheck] = []

    checks.append(
        PackagingBoundCheck(
            name="intake fits within body diameter",
            passes=case.selector.circular_intake_diameter_m <= case.vehicle.body_diameter_m,
            detail=(
                f"intake {case.selector.circular_intake_diameter_m*1000:.0f} mm vs "
                f"body {case.vehicle.body_diameter_m*1000:.0f} mm"
            ),
        )
    )

    throat_with_allowance_m = (
        case.nozzle.throat_diameter_m + 2.0 * case.geometry.nozzle_radial_allowance_m
    )
    checks.append(
        PackagingBoundCheck(
            name="nozzle throat + radial allowance fits within body diameter",
            passes=throat_with_allowance_m <= case.vehicle.body_diameter_m,
            detail=(
                f"throat+allowance {throat_with_allowance_m*1000:.0f} mm vs "
                f"body {case.vehicle.body_diameter_m*1000:.0f} mm"
            ),
        )
    )

    selector_with_allowance_m = (
        case.selector.circular_intake_diameter_m + 2.0 * case.geometry.selector_radial_allowance_m
    )
    checks.append(
        PackagingBoundCheck(
            name="selector + radial allowance fits within body diameter",
            passes=selector_with_allowance_m <= case.vehicle.body_diameter_m,
            detail=(
                f"selector+allowance {selector_with_allowance_m*1000:.0f} mm vs "
                f"body {case.vehicle.body_diameter_m*1000:.0f} mm"
            ),
        )
    )

    chamber_length_upper_bound_m = case.geometry.forebody_transition_length_m
    body_cross_section_area_m2 = 3.141592653589793 * (case.vehicle.body_diameter_m / 2.0) ** 2
    chamber_length_required_m = case.pulsejet.chamber_volume_m3 / max(
        body_cross_section_area_m2, 1e-9
    )
    checks.append(
        PackagingBoundCheck(
            name="pulsejet chamber volume fits within forebody length at full body cross-section",
            passes=chamber_length_required_m <= chamber_length_upper_bound_m,
            detail=(
                f"chamber needs >= {chamber_length_required_m*1000:.0f} mm length at full body "
                f"cross-section vs {chamber_length_upper_bound_m*1000:.0f} mm forebody -- "
                "loosest possible bound, ignores that other hardware shares this volume"
            ),
        )
    )

    fuel_volume_m3 = case.mission.loaded_fuel_mass_kg / case.fuel.density_kg_per_m3
    shell = case.geometry.shell
    shell_outer_diameter_m = case.vehicle.body_diameter_m + 2.0 * shell.radial_offset_m
    annulus_area_m2 = 3.141592653589793 * (
        (shell_outer_diameter_m / 2.0) ** 2 - (case.vehicle.body_diameter_m / 2.0) ** 2
    )
    shell_constant_length_m = (
        shell.end_x_m - shell.start_x_m - shell.forward_taper_length_m - shell.aft_taper_length_m
    )
    annulus_volume_m3 = annulus_area_m2 * max(shell_constant_length_m, 0.0)
    checks.append(
        PackagingBoundCheck(
            name="fuel volume fits within the fin-can annulus (loosest bound, ignores avionics/structure sharing it)",
            passes=fuel_volume_m3 <= annulus_volume_m3,
            detail=(
                f"fuel {fuel_volume_m3*1e6:.0f} cm3 vs annulus {annulus_volume_m3*1e6:.0f} cm3"
            ),
        )
    )

    return checks


@dataclass(frozen=True)
class Level0FeasibilityReport:
    case_name: str
    sled_launch: SledLaunchFeasibility
    stall: StallSpeedBounds
    lift_area: RequiredLiftArea
    thrust_to_weight: list[ThrustToWeightPoint]
    climb_rate: list[ClimbRateEstimate]
    dive_energy: DiveEnergyBenefit
    fuel_energy: FuelEnergyBounds
    packaging: list[PackagingBoundCheck]
    all_pass: bool
    failing_checks: tuple[str, ...]


def evaluate_level0_feasibility(case: ReferenceCase) -> Level0FeasibilityReport:
    """Run every Level 0 hand-calc bound and roll up pass/fail (Gate 1 evidence).

    This does not select a design -- per docs/design_workflow.md, it exists to
    reject impossible concepts and bound the optimizer's search space.
    """

    sled_launch = sled_launch_feasibility(case)
    stall = stall_speed_bounds(case)
    lift_area = required_lift_area(case)
    ttw = thrust_to_weight_sweep(case)
    climb = climb_rate_estimates(ttw)
    dive = dive_energy_benefit(
        case,
        dive_height_m=case.mission.top_of_climb_altitude_min_msl_m
        - case.mission.speed_run_altitude_msl_m,
        entry_speed_m_per_s=case.mission.sled_release_speed_max_m_per_s,
    )
    fuel = fuel_energy_and_endurance_bounds(case)
    packaging = packaging_bounds(case)

    failing: list[str] = []
    if not sled_launch.within_configured_ceiling:
        failing.append("sled launch acceleration exceeds configured ceiling")
    if not stall.passes:
        failing.append("stall speed bound")
    if not lift_area.passes:
        failing.append("required lift area")
    for check in packaging:
        if not check.passes:
            failing.append(f"packaging: {check.name}")

    return Level0FeasibilityReport(
        case_name=case.name,
        sled_launch=sled_launch,
        stall=stall,
        lift_area=lift_area,
        thrust_to_weight=ttw,
        climb_rate=climb,
        dive_energy=dive,
        fuel_energy=fuel,
        packaging=packaging,
        all_pass=not failing,
        failing_checks=tuple(failing),
    )
