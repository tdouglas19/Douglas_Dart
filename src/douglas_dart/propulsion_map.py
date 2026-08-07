"""The authoritative combined propulsion-map interface (docs/design_workflow.md Gate 2).

Before this module, `PulsejetSimulator`/`evaluate_ramjet` were called
independently at 9+ sites (`trajectory.py`, `sizing.py`, `jsbsim_model.py`,
`robustness.py`, `sensitivity.py`, `pipeline.py`, `cli.py`, plus the orphaned
`propulsion.py`), each building its own table/interpolation/fidelity
settings. That is the duplicated-interface problem
`docs/design_workflow.md`'s Gate 2 exists to close.

This module does not replace `ramjet.py`/`pulsejet.py` -- they remain the
physics layer. It is the one place that turns
(Mach, altitude, mode, robustness scenario) into one common, fully-populated
result schema, so every consumer reads the same numbers computed the same
way. New consumers should call `evaluate_propulsion_map_point` (or
`build_propulsion_map` for a Mach sweep) instead of constructing
`PulsejetSimulator`/`evaluate_ramjet` directly.

Migration status (docs/design_workflow.md priority list step 3 -- "migrate
trajectory, plotting, sizing, optimization, and JSBSim-table generation
toward that interface", one file per commit): complete. `trajectory.py`,
`robustness.py`, `sensitivity.py`, `sizing.py` (5 of 7 call sites),
`jsbsim_model.py`, and `pipeline.py`'s ramjet-peak-Mach stage all read
through this module now. The remaining direct calls (`sizing.py`'s
`evaluate_ramjet_handoff_sizing` and one nozzle-matched pulsejet report
field, `pipeline.py`'s pulsejet diagnostic stage, `cli.py`'s standalone
`pulsejet`/`ramjet` commands) are documented in-place as intentional
exceptions: each needs raw physics-layer internals (fuel_air_ratio,
nozzle_capacity_kg_per_s, per-step samples, conservation audits) that
`PropulsionMapPoint` deliberately excludes from its common schema, and none
feeds another design calculation that could silently drift from this map.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass, replace as dataclasses_replace
from pathlib import Path
from statistics import fmean

from .atmosphere import standard_atmosphere
from .compressible import stagnation_pressure
from .config import ReferenceCase
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet

PULSEJET_MODE = "pulsejet"
RAMJET_MODE = "ramjet"


@dataclass(frozen=True)
class PropulsionScenario:
    """The subset of trajectory.py's MissionScenario relevant to propulsion.

    Kept as a separate, minimal type (rather than importing MissionScenario
    directly) so this module has no dependency on trajectory.py -- consumers
    of the propulsion map should not need the mission-phase machinery, and
    trajectory.py itself becomes a consumer of this module, not the other way
    around. `docs/design_workflow.md`: "propulsion-map lookup" is listed as
    an independent fast-inner-loop component, not part of the trajectory
    integrator.
    """

    name: str
    thrust_multiplier: float = 1.0
    ramjet_total_pressure_recovery_override: float | None = None

    def __post_init__(self) -> None:
        if self.thrust_multiplier <= 0.0:
            raise ValueError("thrust multiplier must be positive")
        if self.ramjet_total_pressure_recovery_override is not None and not (
            0.0 < self.ramjet_total_pressure_recovery_override <= 1.0
        ):
            raise ValueError("ramjet total-pressure recovery override must be in (0, 1]")


NOMINAL = PropulsionScenario("nominal")


@dataclass(frozen=True)
class PropulsionMapPoint:
    """One fully-populated propulsion operating point, either mode.

    Field coverage matches Core objective.md's Level 1 requirement list.
    Fields that do not apply to a mode (e.g. inlet spillage for a pulsejet,
    which has no freestream-capture-vs-demand concept the way a ramjet does)
    are `None`, not a fabricated zero -- see each field's inline note.
    """

    mode: str
    mach: float
    altitude_m: float
    scenario_name: str

    gross_thrust_n: float
    inlet_momentum_drag_n: float
    net_thrust_n: float
    fuel_mass_flow_kg_per_s: float
    tsfc_per_hour: float | None
    """Thrust-specific fuel consumption, fuel weight flow / thrust (hr^-1)."""
    specific_impulse_s: float | None

    captured_air_mass_flow_kg_per_s: float
    potential_air_mass_flow_kg_per_s: float | None
    """Ramjet only -- freestream capture before spillage. None for pulsejet."""
    spilled_mass_flow_fraction: float | None
    """Ramjet only. None for pulsejet -- see module docstring."""

    installed_total_pressure_recovery: float
    nozzle_flow_regime: str
    """Ramjet: the fixed_cd_nozzle regime string. Pulsejet: "unsteady" --
    the model has no single steady regime; it cycles through
    combustion/blowdown/refill/dwell phases within one period."""
    combustor_temperature_k: float | None
    """Ramjet: combustor exit total temperature. Pulsejet: peak chamber
    temperature over the sampled window (the closest unsteady analogue)."""
    peak_chamber_pressure_pa: float | None
    """Pulsejet only. None for ramjet, which does not track a chamber
    pressure history the way the unsteady pulsejet model does."""

    lightoff_status: str
    self_sustaining_status: bool | None
    """Ramjet only -- see RamjetResult.self_sustaining_candidate. None for
    pulsejet, which has no analogous self-sustaining-Mach gate."""

    validity_flags: tuple[str, ...]
    numerical_reference_only: bool = True


def _pulsejet_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    warmup_s: float,
    measurement_s: float,
    time_step_s: float,
) -> PropulsionMapPoint:
    simulator = PulsejetSimulator(
        case.pulsejet, case.selector, case.nozzle, case.fuel, altitude_m, mach
    )
    samples = simulator.run(warmup_s + measurement_s, time_step_s)
    window = [s for s in samples if s.time_s >= warmup_s]
    summary = summarize_pulsejet(samples, minimum_time_s=warmup_s)

    mean_net_thrust_n = summary.mean_net_thrust_n * scenario.thrust_multiplier
    mean_gross_thrust_n = summary.mean_gross_thrust_n * scenario.thrust_multiplier
    mean_inlet_air_mass_flow_kg_per_s = (
        fmean(s.inlet_air_mass_flow_kg_per_s for s in window) if window else 0.0
    )

    ambient = standard_atmosphere(altitude_m)
    ideal_total_pressure_pa = stagnation_pressure(ambient.pressure_pa, mach)
    installed_recovery = simulator.inlet_total_pressure_pa / ideal_total_pressure_pa

    tsfc_per_hour = None
    if summary.specific_impulse_s > 1e-9:
        tsfc_per_hour = 3600.0 / summary.specific_impulse_s

    validity_flags: list[str] = []
    if summary.completed_cycles == 0:
        validity_flags.append("no_completed_cycles_in_sample_window")
    if mach >= case.ramjet.minimum_lightoff_test_mach:
        validity_flags.append("above_configured_ramjet_lightoff_mach_pulsejet_mode_unusual")

    return PropulsionMapPoint(
        mode=PULSEJET_MODE,
        mach=mach,
        altitude_m=altitude_m,
        scenario_name=scenario.name,
        gross_thrust_n=mean_gross_thrust_n,
        inlet_momentum_drag_n=mean_gross_thrust_n - mean_net_thrust_n,
        net_thrust_n=mean_net_thrust_n,
        fuel_mass_flow_kg_per_s=summary.mean_fuel_mass_flow_kg_per_s,
        tsfc_per_hour=tsfc_per_hour,
        specific_impulse_s=summary.specific_impulse_s if summary.specific_impulse_s > 0.0 else None,
        captured_air_mass_flow_kg_per_s=mean_inlet_air_mass_flow_kg_per_s,
        potential_air_mass_flow_kg_per_s=None,
        spilled_mass_flow_fraction=None,
        installed_total_pressure_recovery=installed_recovery,
        nozzle_flow_regime="unsteady",
        combustor_temperature_k=summary.peak_chamber_temperature_k,
        peak_chamber_pressure_pa=summary.peak_chamber_pressure_pa,
        lightoff_status=(
            "not_applicable_pulsejet_has_no_lightoff_gate"
        ),
        self_sustaining_status=None,
        validity_flags=tuple(validity_flags),
    )


def _ramjet_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
) -> PropulsionMapPoint:
    recovery = (
        case.selector.ramjet_total_pressure_recovery
        if scenario.ramjet_total_pressure_recovery_override is None
        else scenario.ramjet_total_pressure_recovery_override
    )
    selector = dataclasses_replace(case.selector, ramjet_total_pressure_recovery=recovery)
    result = evaluate_ramjet(case.ramjet, selector, case.nozzle, case.fuel, altitude_m, mach)

    net_thrust_n = result.net_thrust_n * scenario.thrust_multiplier
    gross_thrust_n = result.gross_thrust_n * scenario.thrust_multiplier

    tsfc_per_hour = None
    if result.specific_impulse_s > 1e-9:
        tsfc_per_hour = 3600.0 / result.specific_impulse_s

    validity_flags = list(result.status)

    lightoff_status = "below_lightoff_test_mach"
    if mach >= case.ramjet.minimum_self_sustaining_mach:
        lightoff_status = "at_or_above_self_sustaining_mach"
    elif mach >= case.ramjet.minimum_lightoff_test_mach:
        lightoff_status = "lightoff_test_regime_below_self_sustaining_mach"

    return PropulsionMapPoint(
        mode=RAMJET_MODE,
        mach=mach,
        altitude_m=altitude_m,
        scenario_name=scenario.name,
        gross_thrust_n=gross_thrust_n,
        inlet_momentum_drag_n=result.inlet_momentum_drag_n,
        net_thrust_n=net_thrust_n,
        fuel_mass_flow_kg_per_s=result.fuel_mass_flow_kg_per_s,
        tsfc_per_hour=tsfc_per_hour,
        specific_impulse_s=result.specific_impulse_s if result.specific_impulse_s > 0.0 else None,
        captured_air_mass_flow_kg_per_s=result.air_mass_flow_kg_per_s,
        potential_air_mass_flow_kg_per_s=result.potential_captured_air_mass_flow_kg_per_s,
        spilled_mass_flow_fraction=result.inlet_spillage_fraction,
        installed_total_pressure_recovery=result.installed_total_pressure_recovery,
        nozzle_flow_regime=result.nozzle_flow_regime,
        combustor_temperature_k=result.combustor_exit_total_temperature_k,
        peak_chamber_pressure_pa=None,
        lightoff_status=lightoff_status,
        self_sustaining_status=result.self_sustaining_candidate,
        validity_flags=tuple(validity_flags),
    )


def evaluate_propulsion_map_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    mode: str,
    *,
    scenario: PropulsionScenario = NOMINAL,
    pulsejet_warmup_s: float = 0.25,
    pulsejet_measurement_s: float = 0.25,
    pulsejet_time_step_s: float = 0.00005,
) -> PropulsionMapPoint:
    """Evaluate one authoritative propulsion-map point for either mode.

    ``mode`` selects the physics model directly (this module does not decide
    which mode is "active" at a given Mach -- that mode-selection logic
    belongs to the mission solver, per docs/design_workflow.md's Level 1/2
    split). Pulsejet fidelity knobs default to trajectory.py's prior values
    for continuity with existing results.
    """

    if mode == PULSEJET_MODE:
        return _pulsejet_point(
            case,
            mach,
            altitude_m,
            scenario,
            warmup_s=pulsejet_warmup_s,
            measurement_s=pulsejet_measurement_s,
            time_step_s=pulsejet_time_step_s,
        )
    if mode == RAMJET_MODE:
        return _ramjet_point(case, mach, altitude_m, scenario)
    raise ValueError(f"unknown propulsion mode: {mode!r}")


def build_propulsion_map(
    case: ReferenceCase,
    mach_values: tuple[float, ...],
    altitude_values: tuple[float, ...],
    modes: tuple[str, ...] = (PULSEJET_MODE, RAMJET_MODE),
    *,
    scenario: PropulsionScenario = NOMINAL,
    **pulsejet_fidelity_kwargs,
) -> list[PropulsionMapPoint]:
    """Sweep (mode x altitude x Mach) into one flat list of map points."""

    points: list[PropulsionMapPoint] = []
    for mode in modes:
        for altitude_m in altitude_values:
            for mach in mach_values:
                points.append(
                    evaluate_propulsion_map_point(
                        case, mach, altitude_m, mode, scenario=scenario, **pulsejet_fidelity_kwargs
                    )
                )
    return points


def write_propulsion_map_csv(path: str | Path, points: list[PropulsionMapPoint]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(asdict(points[0]).keys()) if points else []
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for point in points:
            writer.writerow(asdict(point))
    return path
