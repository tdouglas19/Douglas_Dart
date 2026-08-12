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
import math
from dataclasses import asdict, dataclass, replace as dataclasses_replace
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    # pulsejet-km is an OPTIONAL dependency -- see config.py's identical
    # TYPE_CHECKING import for the full rationale. Real, runtime imports are
    # lazy, inside _run_pulsejet_km_query below.
    from pulsejet_km.config import EngineConfig as PulsejetKmEngineConfig
    from pulsejet_km.query import QueryResult as PulsejetKmQueryResult
    # pulsejet-fp: same optional-dependency pattern (pulsejet_fp_bridge.py's
    # runtime imports are lazy inside run_pulsejet_fp_query).
    from pulsejet_fp import ThrustResult as PulsejetFpThrustResult

from .atmosphere import standard_atmosphere
from .compressible import stagnation_pressure
from .config import Fuel, NozzleConfig, PulsejetConfig, ReferenceCase, SelectorConfig
from .pulsejet import PulsejetCycleAverage, run_pulsejet_to_converged_cycle_average
from .pulsejet_fp_bridge import (
    PULSEJET_FP_ADIABATIC_OPTIMISM_FLAG,
    PULSEJET_FP_PROPANE_SURROGATE_FLAG,
    PULSEJET_FP_SCALED_GEOMETRY_FLAG,
    PulsejetFpSpec,
    derive_pulsejet_fp_spec,
    pulsejet_fp_momentum_drag_n,
    pulsejet_fp_primary_enabled,
    pulsejet_fp_result_is_trustworthy,
    run_pulsejet_fp_query,
)
from .ramjet import evaluate_ramjet

PULSEJET_MODE = "pulsejet"
RAMJET_MODE = "ramjet"
PULSEJET_KM_MODE = "pulsejet_km"
PULSEJET_FP_MODE = "pulsejet_fp"
"""Direct-query mode for the first-principles pulsejet-fp model (sibling
repo, see pulsejet_fp_bridge.py). PULSEJET_MODE itself now dispatches to
pulsejet-fp as its guarded PRIMARY -- this constant exists for explicit
comparison queries, mirroring PULSEJET_KM_MODE."""
"""The sibling pulsejet-km (Khrulev & Muntyan) model, queried directly and
unconditionally -- always pulsejet-km's raw answer, never falling back to
the native `pulsejet.py` simulator, for direct model-to-model comparison.
Requires `case.pulsejet_km_engine_config` to be set (see config.py's
ReferenceCase docstring) -- raises ValueError otherwise, rather than
silently falling back to a different engine's data.

**2026-08-11: PULSEJET_MODE itself now tries pulsejet-km first too.**
pulsejet-km is the primary pulsejet query source architecturally: whenever
a candidate provides `pulsejet_km_engine_config`, PULSEJET_MODE's dispatch
(`_pulsejet_mode_point` below) queries pulsejet-km first and only falls
back to this native simulator when pulsejet-km itself signals it cannot
answer -- see that function's docstring for the exact guards. This
PULSEJET_KM_MODE constant/branch is unaffected by that change and stays
useful precisely because it skips the guards and the native fallback: use
it to see pulsejet-km's answer even where PULSEJET_MODE's guarded dispatch
would have rejected it.

pulsejet-km's own thrust output is, as of this integration, independently
confirmed by that project's own architecture.md to still be substantially
short of its validation target (~8x low even after its most recent fix,
and unboundedly negative above its own validated Mach 0.7 envelope) --
clearing pulsejet-km's own convergence/envelope/classification checks is
not the same as the thrust number being numerically accurate. Every point
sourced from pulsejet-km, through either mode, carries validity flags
saying so (`_PULSEJET_KM_THRUST_LOW_BIAS_FLAG` for PULSEJET_MODE;
`pulsejet_km_classification_*`/`pulsejet_km_no_steady_state_thrust_available`
for both) so no consumer mistakes a clean-looking result for a validated
one."""

# dt_convergence_solver_spec.md (2026-08-08): closes the second half of the
# original Gate 2 fidelity bug. time_step_s (dt) is retired as a caller-
# adjustable float the same way warmup_s/measurement_s already were
# (cycle_based_averaging_fix.md) -- but unlike that window, dt has one
# legitimate reason for two different fixed values to coexist: a cheap
# setting for the search's inner loop and a fine setting for verification,
# not an arbitrary knob two callers could each set to something different by
# accident. `find_converged_time_step` (pulsejet.py,
# scripts/dt_convergence_study.py) measured the finest dt genuinely needed
# across a representative Mach/geometry set: only 2 of 6 test points formally
# converged (M=0.10 -> 2.5e-5s; shared_nozzle_candidate_a M=0.50 -> 6.25e-6s)
# within the study's halving budget, the finest of which is adopted below as
# PULSEJET_FIDELITY_FULL. Three more points (M=0.20, M=0.50 on
# reference_case, shared_nozzle_candidate_b M=0.50) were still visibly
# settling with small residual diffs when the halving budget ran out --
# consistent with, not contradicting, this value. One point (M=0.95) showed
# genuine numerical fragility (its inner cycle-average hit an extended cap at
# dt=2.5e-5s before settling at finer dt) -- a real open question about that
# regime specifically, not resolved by this constant. See
# docs/design_convergence.md, "dt-convergence solver" for the full report.
# PULSEJET_FIDELITY_FAST keeps the search's existing 1e-4s -- explicitly NOT
# re-derived from the same convergence study (that would multiply the
# search's already-substantial cost by roughly 16x, per direct measurement,
# making it impractical) -- it exists for search speed, not verification
# accuracy, and this is a deliberate, named, fixed choice, not a value one
# caller could silently pick differently from another.
PULSEJET_FIDELITY_FAST = "fast"
PULSEJET_FIDELITY_FULL = "full"
_PULSEJET_TIME_STEP_S_BY_FIDELITY = {
    PULSEJET_FIDELITY_FAST: 1e-4,
    PULSEJET_FIDELITY_FULL: 6.25e-6,
}

# The cycle-average convergence cap (pulsejet.py's
# _CYCLE_AVERAGE_MAXIMUM_CYCLES/_CYCLE_AVERAGE_MAXIMUM_SIMULATED_TIME_S) is
# also tier-dependent, not one global constant -- found the hard way
# (2026-08-08): a single `evaluate_design` call builds an 11-point pulsejet
# table for each of 2 scenarios, and a pathological candidate (e.g. an
# oversized chamber_volume_m3 near this search's own upper bound) can hit the
# cap at *every* point. At the original global cap (100 cycles, matching
# PULSEJET_FIDELITY_FULL's verification-grade needs), one such candidate
# could cost up to ~2.4 hours (11 points x 2 scenarios x the per-point worst
# case) -- fine for a one-off plot, fatal for anything resembling a search
# loop, where per-candidate cost must stay bounded and predictable across
# thousands of candidates, not just accurate for the well-behaved ones.
# PULSEJET_FIDELITY_FAST therefore gets a much tighter cap than
# PULSEJET_FIDELITY_FULL keeps -- this is not a search-speed vs. accuracy
# trade unique to dt; the cycle cap needs the identical two-tier treatment
# for the identical reason.
_PULSEJET_CYCLE_CAP_BY_FIDELITY = {
    PULSEJET_FIDELITY_FAST: 30,
    PULSEJET_FIDELITY_FULL: 50,
}
_PULSEJET_SIMULATED_TIME_CAP_S_BY_FIDELITY = {
    PULSEJET_FIDELITY_FAST: 30.0,
    PULSEJET_FIDELITY_FULL: 150.0,
}

# pulsejet-km's own fidelity knobs (time_step, maximum_cycles,
# maximum_dimensionless_time -- all DIMENSIONLESS, unlike PULSEJET_MODE's
# real-second dt above; not the same unit convention, deliberately not
# reusing _PULSEJET_TIME_STEP_S_BY_FIDELITY's values) don't map 1:1 onto
# PULSEJET_MODE's two-tier system (docs/pulsejet_external_model_audit.md's
# own finding). FULL below matches this integration session's own manually-
# validated Argus parameters (STABLE_LIMIT_CYCLE reached reliably). FAST is
# a first-pass, untested-but-reasonable faster tier for search-loop use --
# not separately profiled/convergence-checked the way PULSEJET_MODE's own
# tiers were (dt_convergence_solver_spec.md) -- flagged honestly as a gap,
# not presented as equally validated.
_PULSEJET_KM_TIME_STEP_BY_FIDELITY = {
    PULSEJET_FIDELITY_FAST: 0.0005,
    PULSEJET_FIDELITY_FULL: 0.0001,
}
_PULSEJET_KM_CYCLE_CAP_BY_FIDELITY = {
    PULSEJET_FIDELITY_FAST: 40,
    PULSEJET_FIDELITY_FULL: 80,
}
_PULSEJET_KM_DIMENSIONLESS_TIME_CAP_BY_FIDELITY = {
    PULSEJET_FIDELITY_FAST: 40.0,
    PULSEJET_FIDELITY_FULL: 60.0,
}


def pulsejet_time_step_s_for_fidelity(pulsejet_fidelity: str) -> float:
    """Resolve a named fidelity tier to its fixed dt -- the one place this
    mapping lives. For `sizing.py`'s intentional-exception direct-
    construction call sites (need raw physics-layer internals
    `PropulsionMapPoint` excludes), not for general use -- prefer
    `evaluate_propulsion_map_point`/`build_propulsion_map`."""

    if pulsejet_fidelity not in _PULSEJET_TIME_STEP_S_BY_FIDELITY:
        raise ValueError(f"unknown pulsejet fidelity: {pulsejet_fidelity!r}")
    return _PULSEJET_TIME_STEP_S_BY_FIDELITY[pulsejet_fidelity]


def pulsejet_cycle_bounds_for_fidelity(pulsejet_fidelity: str) -> tuple[int, float]:
    """Resolve a named fidelity tier to its (cycle cap, simulated-time cap).

    Same rationale/audience as `pulsejet_time_step_s_for_fidelity` -- these
    bound `run_pulsejet_to_converged_cycle_average`'s worst-case cost per
    point, tier-dependent for the same reason dt is (see module comment
    above)."""

    if pulsejet_fidelity not in _PULSEJET_CYCLE_CAP_BY_FIDELITY:
        raise ValueError(f"unknown pulsejet fidelity: {pulsejet_fidelity!r}")
    return (
        _PULSEJET_CYCLE_CAP_BY_FIDELITY[pulsejet_fidelity],
        _PULSEJET_SIMULATED_TIME_CAP_S_BY_FIDELITY[pulsejet_fidelity],
    )


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

    installed_total_pressure_recovery: float | None
    """None for PULSEJET_KM_MODE -- pulsejet-km's QueryResult does not expose
    inlet total pressure the way this repo's own PulsejetCycleAverage/
    RamjetResult do (see module docstring's PULSEJET_KM_MODE note)."""
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


# cycle_based_averaging_fix.md (2026-08-08): the pulsejet path no longer uses
# a fixed warmup_s+measurement_s wall-clock window. See pulsejet.py's
# run_pulsejet_to_converged_cycle_average and its module-level comment block
# for the full rationale (the old fixed window caught a different, non-
# integer number of real ignition cycles at each Mach step, producing broad
# sawtooth jaggedness in any Mach sweep -- structurally the same class of bug
# as the original Gate 2 fast/full-fidelity mismatch). dt_convergence_solver_
# spec.md (2026-08-08) closes the other half: time_step_s is no longer a
# caller-chosen float either -- see PULSEJET_FIDELITY_FAST/FULL above.
@lru_cache(maxsize=512)
def _run_pulsejet_simulation(
    config: PulsejetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
    pulsejet_fidelity: str,
) -> PulsejetCycleAverage:
    """Run to a converged cycle average and return it.

    Pure and deterministic given these arguments (no RNG, no shared mutable
    state -- see `PulsejetSimulator`'s docstring), so safe to memoize.
    `_pulsejet_point` below applies `scenario.thrust_multiplier` only *after*
    this call returns -- the simulation itself never reads `scenario`.
    Callers (`trajectory.py`'s nominal/adverse/conservative scenarios,
    `robustness.py`'s screens) therefore re-request this exact
    (case, altitude, mach, fidelity) combination multiple times per candidate;
    memoizing avoids re-running the unsteady sim for a result that would
    come out byte-identical.
    """
    time_step_s = _PULSEJET_TIME_STEP_S_BY_FIDELITY[pulsejet_fidelity]
    cycle_cap, simulated_time_cap_s = pulsejet_cycle_bounds_for_fidelity(pulsejet_fidelity)
    return run_pulsejet_to_converged_cycle_average(
        config,
        selector,
        nozzle,
        fuel,
        altitude_m,
        mach,
        time_step_s,
        _maximum_cycles_override=cycle_cap,
        _maximum_simulated_time_s_override=simulated_time_cap_s,
    )


def _pulsejet_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    pulsejet_fidelity: str,
) -> PropulsionMapPoint:
    cycle_average = _run_pulsejet_simulation(
        case.pulsejet,
        case.selector,
        case.nozzle,
        case.fuel,
        altitude_m,
        mach,
        pulsejet_fidelity,
    )

    mean_net_thrust_n = cycle_average.mean_net_thrust_n * scenario.thrust_multiplier
    mean_gross_thrust_n = cycle_average.mean_gross_thrust_n * scenario.thrust_multiplier

    ambient = standard_atmosphere(altitude_m)
    ideal_total_pressure_pa = stagnation_pressure(ambient.pressure_pa, mach)
    installed_recovery = cycle_average.inlet_total_pressure_pa / ideal_total_pressure_pa

    tsfc_per_hour = None
    if cycle_average.specific_impulse_s > 1e-9:
        tsfc_per_hour = 3600.0 / cycle_average.specific_impulse_s

    validity_flags: list[str] = []
    if cycle_average.averaged_cycles == 0:
        validity_flags.append("no_completed_cycles_within_simulation_cap")
    elif not cycle_average.converged:
        validity_flags.append("cycle_average_did_not_converge")
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
        fuel_mass_flow_kg_per_s=cycle_average.mean_fuel_mass_flow_kg_per_s,
        tsfc_per_hour=tsfc_per_hour,
        specific_impulse_s=(
            cycle_average.specific_impulse_s if cycle_average.specific_impulse_s > 0.0 else None
        ),
        captured_air_mass_flow_kg_per_s=cycle_average.mean_inlet_air_mass_flow_kg_per_s,
        potential_air_mass_flow_kg_per_s=None,
        spilled_mass_flow_fraction=None,
        installed_total_pressure_recovery=installed_recovery,
        nozzle_flow_regime="unsteady",
        combustor_temperature_k=cycle_average.peak_chamber_temperature_k,
        peak_chamber_pressure_pa=cycle_average.peak_chamber_pressure_pa,
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


@lru_cache(maxsize=512)
def _run_pulsejet_km_query(
    engine_config: "PulsejetKmEngineConfig",
    mach: float,
    pulsejet_fidelity: str,
) -> "PulsejetKmQueryResult":
    """Run pulsejet-km's query_pulsejet to a converged cycle average and
    return it. Pure and deterministic given these arguments (EngineConfig is
    frozen/hashable), same memoization rationale as `_run_pulsejet_simulation`
    above. Imports pulsejet-km lazily -- only reached when PULSEJET_KM_MODE
    is actually requested, so it stays a genuinely optional dependency
    (config.py's `_load_pulsejet_km_engine_config` has the same pattern)."""

    from pulsejet_km.query import query_pulsejet as query_pulsejet_km

    if pulsejet_fidelity not in _PULSEJET_KM_TIME_STEP_BY_FIDELITY:
        raise ValueError(f"unknown pulsejet fidelity: {pulsejet_fidelity!r}")
    return query_pulsejet_km(
        engine_config,
        flight_mach=mach,
        time_step=_PULSEJET_KM_TIME_STEP_BY_FIDELITY[pulsejet_fidelity],
        maximum_cycles=_PULSEJET_KM_CYCLE_CAP_BY_FIDELITY[pulsejet_fidelity],
        maximum_dimensionless_time=_PULSEJET_KM_DIMENSIONLESS_TIME_CAP_BY_FIDELITY[pulsejet_fidelity],
    )


_PULSEJET_KM_THRUST_LOW_BIAS_FLAG = "pulsejet_km_thrust_known_low_bias_see_pulsejet_km_architecture_md"
"""pulsejet-km's own architecture.md (Section 54, 2026-08-11) independently
confirms its thrust output reads ~8x low against its own validation target
even for a query that clears every structural guard in
`_pulsejet_km_result_is_trustworthy_enough_to_prefer` (converged,
STABLE_LIMIT_CYCLE, within its validated Mach envelope, positive net
thrust). Clearing those guards means the query executed and looks
numerically sane -- it does not mean the thrust figure is accurate. Every
PropulsionMapPoint sourced from pulsejet-km via PULSEJET_MODE's guarded
dispatch carries this flag unconditionally, so no downstream consumer
(trajectory.py, robustness.py, sensitivity.py, the optimizer) can mistake
"no validity flags raised" for "this thrust figure is trustworthy"."""


def _km_result_to_propulsion_map_point(
    result: "PulsejetKmQueryResult",
    mode: str,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    extra_validity_flags: tuple[str, ...] = (),
) -> PropulsionMapPoint:
    """Map a pulsejet-km ``QueryResult`` into this module's common schema.

    Shared by ``_pulsejet_km_point`` (PULSEJET_KM_MODE's unconditional,
    comparison-only query) and ``_pulsejet_mode_point`` (PULSEJET_MODE's
    guarded-primary dispatch) -- both need the identical field mapping;
    only ``mode`` and the extra flags differ.
    """

    net_thrust_n = result.mean_net_thrust_n * scenario.thrust_multiplier
    gross_thrust_n = result.mean_gross_thrust_n * scenario.thrust_multiplier

    tsfc_per_hour = None
    if result.specific_impulse_s > 1e-9:
        tsfc_per_hour = 3600.0 / result.specific_impulse_s

    # pulsejet-km's own classification/convergence/envelope concepts don't
    # map onto PULSEJET_MODE's validity_flags vocabulary one-for-one -- kept
    # as distinct, clearly-pulsejet-km-scoped flag strings rather than
    # forced into the existing "no_completed_cycles_within_simulation_cap"/
    # "cycle_average_did_not_converge" strings, which describe this repo's
    # own pulsejet.py convergence machinery specifically.
    validity_flags: list[str] = [f"pulsejet_km_classification_{result.classification.lower()}"]
    if not result.converged:
        validity_flags.append("pulsejet_km_cycle_average_did_not_converge")
    if not result.within_validated_mach_envelope:
        validity_flags.append("pulsejet_km_above_validated_mach_envelope")
    if not result.steady_state_thrust_available:
        validity_flags.append("pulsejet_km_no_steady_state_thrust_available")
    validity_flags.extend(extra_validity_flags)

    return PropulsionMapPoint(
        mode=mode,
        mach=mach,
        altitude_m=altitude_m,
        scenario_name=scenario.name,
        gross_thrust_n=gross_thrust_n,
        inlet_momentum_drag_n=gross_thrust_n - net_thrust_n,
        net_thrust_n=net_thrust_n,
        fuel_mass_flow_kg_per_s=result.mean_fuel_mass_flow_kg_per_s,
        tsfc_per_hour=tsfc_per_hour,
        specific_impulse_s=result.specific_impulse_s if result.specific_impulse_s > 0.0 else None,
        captured_air_mass_flow_kg_per_s=result.mean_air_mass_flow_kg_per_s,
        potential_air_mass_flow_kg_per_s=None,
        spilled_mass_flow_fraction=None,
        installed_total_pressure_recovery=None,
        nozzle_flow_regime="unsteady",
        combustor_temperature_k=None,
        peak_chamber_pressure_pa=None,
        lightoff_status="not_applicable_pulsejet_has_no_lightoff_gate",
        self_sustaining_status=None,
        validity_flags=tuple(validity_flags),
    )


def _fp_result_to_propulsion_map_point(
    result: "PulsejetFpThrustResult",
    spec: PulsejetFpSpec,
    mode: str,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    extra_validity_flags: tuple[str, ...] = (),
) -> PropulsionMapPoint:
    """Map a pulsejet-fp ``ThrustResult`` into the common schema.

    pulsejet-fp's reported thrust is already NET of side-inlet momentum
    drag (its eq. 24 charges the swallowed boundary layer at
    k_bl * u_inf); gross is reconstructed with the identical bookkeeping so
    ``gross - drag == net`` holds exactly. TSFC/Isp follow the same
    weight-flow convention as the native path.
    """

    net_thrust_n = result.thrust_n * scenario.thrust_multiplier
    gross_drag_n = pulsejet_fp_momentum_drag_n(
        result, mach, altitude_m, spec.bl_momentum_fraction
    )
    gross_thrust_n = net_thrust_n + gross_drag_n

    fuel_flow = result.mdot_fuel_kg_s if math.isfinite(result.mdot_fuel_kg_s) else 0.0
    specific_impulse_s = None
    tsfc_per_hour = None
    if fuel_flow > 1e-12 and net_thrust_n > 0.0:
        specific_impulse_s = net_thrust_n / (fuel_flow * 9.80665)
        tsfc_per_hour = 3600.0 / specific_impulse_s

    ambient = standard_atmosphere(altitude_m)
    peak_chamber_pressure_pa = None
    if math.isfinite(result.p_max_ratio):
        peak_chamber_pressure_pa = result.p_max_ratio * ambient.pressure_pa
    # side inlet feeds at ambient STATIC pressure -- its true installed
    # recovery relative to the ideal ram stagnation state is p_a / p0(M)
    installed_recovery = ambient.pressure_pa / stagnation_pressure(ambient.pressure_pa, mach)

    validity_flags: list[str] = [
        f"pulsejet_fp_status_{result.status}",
        PULSEJET_FP_ADIABATIC_OPTIMISM_FLAG,
        PULSEJET_FP_PROPANE_SURROGATE_FLAG,
    ]
    if spec.geometry_derived_from_chamber_volume:
        validity_flags.append(PULSEJET_FP_SCALED_GEOMETRY_FLAG)
    validity_flags.extend(extra_validity_flags)

    captured_air = result.mdot_air_kg_s if math.isfinite(result.mdot_air_kg_s) else 0.0
    return PropulsionMapPoint(
        mode=mode,
        mach=mach,
        altitude_m=altitude_m,
        scenario_name=scenario.name,
        gross_thrust_n=gross_thrust_n,
        inlet_momentum_drag_n=gross_drag_n,
        net_thrust_n=net_thrust_n,
        fuel_mass_flow_kg_per_s=fuel_flow,
        tsfc_per_hour=tsfc_per_hour,
        specific_impulse_s=specific_impulse_s,
        captured_air_mass_flow_kg_per_s=captured_air,
        potential_air_mass_flow_kg_per_s=None,
        spilled_mass_flow_fraction=None,
        installed_total_pressure_recovery=installed_recovery,
        nozzle_flow_regime="unsteady",
        combustor_temperature_k=None,
        peak_chamber_pressure_pa=peak_chamber_pressure_pa,
        lightoff_status="not_applicable_pulsejet_has_no_lightoff_gate",
        self_sustaining_status=None,
        validity_flags=tuple(validity_flags),
    )


def _pulsejet_fp_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    pulsejet_fidelity: str,
) -> PropulsionMapPoint:
    """PULSEJET_FP_MODE: unconditional direct query of the first-principles
    model (comparison/analysis use -- no fallback, failures surface)."""

    spec = derive_pulsejet_fp_spec(case)
    result = run_pulsejet_fp_query(spec, mach, altitude_m, pulsejet_fidelity)
    return _fp_result_to_propulsion_map_point(
        result, spec, PULSEJET_FP_MODE, mach, altitude_m, scenario
    )


def _pulsejet_km_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    pulsejet_fidelity: str,
) -> PropulsionMapPoint:
    if case.pulsejet_km_engine_config is None:
        raise ValueError(
            "PULSEJET_KM_MODE requires case.pulsejet_km_engine_config to be set -- "
            "no 'pulsejet_km:' section was present when this ReferenceCase was loaded"
        )

    result = _run_pulsejet_km_query(case.pulsejet_km_engine_config, mach, pulsejet_fidelity)
    return _km_result_to_propulsion_map_point(result, PULSEJET_KM_MODE, mach, altitude_m, scenario)


def _pulsejet_km_result_is_trustworthy_enough_to_prefer(result: "PulsejetKmQueryResult") -> bool:
    """The structural (not magnitude) guards a pulsejet-km result must clear
    before PULSEJET_MODE's guarded-primary dispatch prefers it over the
    native simulator: real convergence, inside pulsejet-km's own validated
    Mach envelope, a genuine steady-state limit cycle (not a decayed,
    failed, or growing one), and positive net thrust -- pulsejet-km's own
    architecture.md documents this model going unboundedly negative above
    Mach 0.7, and a result that obviously unphysical must never reach a
    design calculation. This does NOT check thrust magnitude against any
    external truth -- see `_PULSEJET_KM_THRUST_LOW_BIAS_FLAG` for why a
    result can clear every check here and still be quantitatively wrong."""

    return (
        result.converged
        and result.within_validated_mach_envelope
        and result.steady_state_thrust_available
        and result.mean_net_thrust_n > 0.0
    )


def _pulsejet_mode_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    scenario: PropulsionScenario,
    *,
    pulsejet_fidelity: str,
) -> PropulsionMapPoint:
    """PULSEJET_MODE's actual dispatch target. Priority order (2026-08-12):

    1. **pulsejet-fp** (first-principles transient model, sibling repo with
       real git history and a pinned editable install) -- the PRIMARY.
       Always derivable from the case's own chamber_volume_m3, guarded by
       `pulsejet_fp_result_is_trustworthy`.
    2. pulsejet-km (guarded, only when the case provides
       `pulsejet_km_engine_config`; still carries its known ~8x thrust
       low-bias flag) -- retained for comparison continuity.
    3. The native 0D `pulsejet.py` simulator -- the final fallback.

    Each demotion is visible via fallback validity flags, never silent.
    Exceptions in either sibling model fall through -- a crash inside a
    sibling dependency must not take down an overnight design-optimize run.

    A point sourced from pulsejet-km still carries
    `_PULSEJET_KM_THRUST_LOW_BIAS_FLAG` -- clearing the structural guards
    above is not the same as the thrust number being numerically accurate.
    A point that fell back to the native simulator carries
    "pulsejet_km_primary_rejected_fell_back_to_native" so the fallback is
    visible in the output, not silent.

    Every current vehicle candidate config (shared_nozzle_candidate_a/b,
    robustness_candidate_b) has no `pulsejet_km:` YAML section yet -- only
    reference_case.yaml does, and that section describes pulsejet-km's own
    Argus As-014/V-1 reference geometry, not this vehicle -- so today this
    still resolves to the native path for every real design candidate.
    Populating real per-vehicle pulsejet-km geometry is separate,
    substantial engineering work (mapping this repo's valve/chamber/
    tailpipe design variables onto pulsejet-km's EngineGeometry/
    ValvePetalGeometry) and is tracked, not done here."""

    # --- pulsejet-fp: the PRIMARY pulsejet query source (2026-08-12) ---
    # First-principles transient model (see pulsejet_fp_bridge.py's module
    # docstring for why it supersedes both pulsejet-km and the native 0D
    # simulator). Always derivable -- the spec scales from the case's own
    # chamber_volume_m3 -- so unlike the km path this primary applies to
    # every real design candidate, not only configs with a hand-written
    # sibling section. Guarded exactly like the km path was: structural
    # trustworthiness checks, and any exception falls through (a sibling-
    # repo crash must not take down an overnight design-optimize run).
    fp_result: "PulsejetFpThrustResult | None" = None
    fp_spec = None
    if pulsejet_fp_primary_enabled():
        try:
            fp_spec = derive_pulsejet_fp_spec(case)
            fp_result = run_pulsejet_fp_query(fp_spec, mach, altitude_m, pulsejet_fidelity)
        except Exception:
            fp_result = None
    if fp_result is not None and fp_spec is not None and pulsejet_fp_result_is_trustworthy(fp_result):
        return _fp_result_to_propulsion_map_point(
            fp_result,
            fp_spec,
            PULSEJET_MODE,
            mach,
            altitude_m,
            scenario,
        )
    fp_fallback_flag = "pulsejet_fp_primary_rejected_fell_back"

    if case.pulsejet_km_engine_config is not None:
        km_result: "PulsejetKmQueryResult | None"
        try:
            km_result = _run_pulsejet_km_query(case.pulsejet_km_engine_config, mach, pulsejet_fidelity)
        except Exception:
            km_result = None
        if km_result is not None and _pulsejet_km_result_is_trustworthy_enough_to_prefer(km_result):
            return _km_result_to_propulsion_map_point(
                km_result,
                PULSEJET_MODE,
                mach,
                altitude_m,
                scenario,
                extra_validity_flags=(_PULSEJET_KM_THRUST_LOW_BIAS_FLAG, fp_fallback_flag),
            )

    point = _pulsejet_point(case, mach, altitude_m, scenario, pulsejet_fidelity=pulsejet_fidelity)
    fallback_flags = [fp_fallback_flag]
    if case.pulsejet_km_engine_config is not None:
        fallback_flags.append("pulsejet_km_primary_rejected_fell_back_to_native")
    point = dataclasses_replace(
        point,
        validity_flags=point.validity_flags + tuple(fallback_flags),
    )
    return point


def evaluate_propulsion_map_point(
    case: ReferenceCase,
    mach: float,
    altitude_m: float,
    mode: str,
    *,
    scenario: PropulsionScenario = NOMINAL,
    pulsejet_fidelity: str = PULSEJET_FIDELITY_FULL,
) -> PropulsionMapPoint:
    """Evaluate one authoritative propulsion-map point for either mode.

    ``mode`` selects the physics model directly (this module does not decide
    which mode is "active" at a given Mach -- that mode-selection logic
    belongs to the mission solver, per docs/design_workflow.md's Level 1/2
    split). No pulsejet warmup/measurement window exists anymore
    (cycle_based_averaging_fix.md, 2026-08-08), and dt is no longer a raw
    caller-chosen float either (dt_convergence_solver_spec.md, 2026-08-08) --
    ``pulsejet_fidelity`` selects between the two fixed, named dt tiers
    (``PULSEJET_FIDELITY_FAST``/``PULSEJET_FIDELITY_FULL`` above) instead.
    """

    if mode == PULSEJET_MODE:
        if pulsejet_fidelity not in _PULSEJET_TIME_STEP_S_BY_FIDELITY:
            raise ValueError(f"unknown pulsejet fidelity: {pulsejet_fidelity!r}")
        return _pulsejet_mode_point(
            case,
            mach,
            altitude_m,
            scenario,
            pulsejet_fidelity=pulsejet_fidelity,
        )
    if mode == RAMJET_MODE:
        return _ramjet_point(case, mach, altitude_m, scenario)
    if mode == PULSEJET_KM_MODE:
        if pulsejet_fidelity not in _PULSEJET_KM_TIME_STEP_BY_FIDELITY:
            raise ValueError(f"unknown pulsejet fidelity: {pulsejet_fidelity!r}")
        return _pulsejet_km_point(
            case,
            mach,
            altitude_m,
            scenario,
            pulsejet_fidelity=pulsejet_fidelity,
        )
    if mode == PULSEJET_FP_MODE:
        return _pulsejet_fp_point(
            case,
            mach,
            altitude_m,
            scenario,
            pulsejet_fidelity=pulsejet_fidelity,
        )
    raise ValueError(f"unknown propulsion mode: {mode!r}")


def build_propulsion_map(
    case: ReferenceCase,
    mach_values: tuple[float, ...],
    altitude_values: tuple[float, ...],
    modes: tuple[str, ...] = (PULSEJET_MODE, RAMJET_MODE),
    *,
    scenario: PropulsionScenario = NOMINAL,
    pulsejet_fidelity: str = PULSEJET_FIDELITY_FULL,
) -> list[PropulsionMapPoint]:
    """Sweep (mode x altitude x Mach) into one flat list of map points."""

    points: list[PropulsionMapPoint] = []
    for mode in modes:
        for altitude_m in altitude_values:
            for mach in mach_values:
                points.append(
                    evaluate_propulsion_map_point(
                        case,
                        mach,
                        altitude_m,
                        mode,
                        scenario=scenario,
                        pulsejet_fidelity=pulsejet_fidelity,
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
