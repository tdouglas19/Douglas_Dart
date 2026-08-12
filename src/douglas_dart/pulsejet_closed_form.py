"""Closed-form, cycle-averaged pulsejet correlation -- NOT currently wired
into Gate 2/3.

**Status (confirmed by docs/pulsejet_external_model_audit.md, 2026-08-10):**
this module was originally built to replace `PulsejetSimulator` in
`propulsion_map.py`'s default per-call evaluation path via a
`force_unsteady=True/False` switch, but that switch was never implemented --
`propulsion_map.py` has no `force_unsteady` parameter anywhere and never
calls `evaluate_pulsejet_closed_form`. This is confirmed, structurally dead
code from Gate 2/3's perspective: `_pulsejet_point`/`_pulsejet_mode_point`
call `run_pulsejet_to_converged_cycle_average` (the full unsteady
`PulsejetSimulator`) directly, unconditionally. Do not assume this module's
output reaches any live design calculation until it is actually wired in
and re-verified -- the equations and derivations below remain useful
reference/candidate logic, just not connected to anything today.

This module's structure and every simplification in it were agreed with the
user in an earlier chat session, cited throughout below as decisions from
`gate2_pulsejet_fork_decisions.md`/`gate2_refill_phase_decision.md` --
**neither file was ever persisted to this repo** (confirmed by a repo-wide
search; the audit above found the same thing). Treat every such citation
below as "reasoning agreed upon in a prior conversation, not independently
re-checkable from a file in this tree," not as a pointer to a real document.
Nothing here was resolved silently -- the reasoning existed and was applied
consistently -- but the cited source itself is unrecoverable.

## Cycle decomposition

One pulsejet cycle is idealized as three sequential phases -- ignition
(instantaneous), blowdown (outflow only), refill (inflow only) -- instead of
`PulsejetSimulator`'s concurrent in/out flow every timestep (decision #2,
`gate2_pulsejet_fork_decisions.md`: accepted as a structural simplification).

### Phase A -- ignition (`_solve_ignition`)

Exact, not approximated, given a pre-ignition state. `pulsejet.py`'s own
`_ignite_if_ready` docstring already states the governing relation for
constant-volume heat addition (process 2->3,
docs/pulsejet_ramjet_governing_equations.md sec. 2.4):

    Q = m_fuel_burned * LHV * combustion_efficiency        [[pulsejet.py:349-353]]
    T3 = T2 + Q / (m2 * cv)                                 [1st law, fixed V, fixed m]
    p3 = p2 * (T3 / T2)                                     [ideal gas, V & m constant]

The pre-ignition state itself (`p2`, `T2`, composition) is *not* a free
assumption -- see "Periodic steady state" below.

### Phase B -- choked blowdown (`_solve_choked_blowdown`)

Derived from scratch by integrating the exact choked mass-flow formula
already used by `compressible_orifice_mass_flow`'s choked branch and
`fixed_cd_nozzle`'s `choked_supersonic_exit` regime, under the standard
isentropic-uniform-reservoir blowdown assumption. Setting
`dm/dt = -C*(m/m0)^n` with `n = (gamma+1)/2` and `C` the choked mass flow at
the initial (post-ignition) state gives a separable ODE with the closed-form
solution (independently checked against the mass-flow and stream-thrust
formulas twice during derivation):

    m(t)/m0 = [1 + (gamma-1)/(2*tau) * t] ** (-2/(gamma-1)),   tau = m0/C
    p(t)/p0 = (m(t)/m0) ** gamma                                [isentropic, V const]
    F(t) = (p(t)/p0) * (F0 + P_amb*A_e*C_d) - P_amb*A_e*C_d     [stream thrust, exit
                                                                  Mach fixed by area
                                                                  ratio alone in this
                                                                  regime]

Only valid while the nozzle stays in `fixed_cd_nozzle`'s
`choked_supersonic_exit` regime, i.e. `p(t) >= p_ambient /
shock_at_exit_pressure_ratio` (the same threshold `fixed_cd_nozzle` itself
uses via `_internal_shock_exit_state`, reused here, not re-derived). Beyond
that point exit Mach is no longer a pure geometry constant and the ODE has no
known elementary solution -- decision #1: the formula above is extrapolated
past its strict validity as the least-bad option, and
`PulsejetCorrectionCoefficients.tail_impulse_fraction` /
`tail_time_fraction` are FIT corrections on that extrapolation, never to be
read as symbolic.

### Phase C -- refill (`_solve_refill_rate`)

**Correction to the original plan, not a continuation of it** (per
`gate2_refill_phase_decision.md` condition #4): the original fix proposal
predicted the closed form would be weakest "near the ignition-limited regime
at high altitude/low density." What was actually found, by inspecting
`PulsejetSimulator`'s own chamber-pressure trace directly (not assumed):
refill is driven by a **Mach-dependent near-equilibrium effect**, not an
altitude/density one. At `reference_case.yaml`, Mach 0.05-0.3, chamber
pressure self-regulates within ~0.01-0.02% of `Pt_inlet` (61-75% of simulated
time spent fractionally below it); by Mach 0.6-0.9 that deficit grows to
2.8-7.6% and the mechanism becomes an ordinary driving-pressure-differential
one. A single instantaneous-differential-pressure orifice evaluation is
ill-posed in the low-Mach band (the driving pressure difference is smaller
than the quasi-static model's own resolution) -- verified before relying on
it: `PulsejetSimulator`'s *time-averaged* inflow rate over a dwell/refill
window is timestep-independent to 4 significant figures across a 5x
timestep sweep (real, converged physics), while the naive "rate averaged
over only the nonzero-flow samples" is not (a discretization artifact).
Therefore refill rate here is modeled as a **fit scale on a well-defined
reference flow** (the inlet's own choked capacity at `Pt_inlet`/`Tt_inlet`,
never singular), not derived from an ill-posed instantaneous differential:

    mdot_refill = fresh_air_inflow_discount * C_reference

`fresh_air_inflow_discount` is therefore not a small correction on an
already-physical estimate -- it is the primary determinant of refill rate,
honestly fit from calibration data, not derived.

### Periodic steady state (`_solve_periodic_state`)

The pre-ignition state Phase A needs cannot be assumed (neither "fully
refilled to ambient" nor any other fixed reference -- both were tried and
falsified against `PulsejetSimulator` during derivation). Per
`gate2_refill_phase_decision.md` (option 3, accepted over conceding the
low-Mach regime to `force_unsteady` or a linearized small-deficit model):
solve for the pre-ignition fresh-air mass and total mass that are
**self-consistent** -- whatever state one full cycle (ignition -> blowdown
-> refill) produces at its end must equal the state it started from. This is
a fixed-point iteration over algebraic (not time-stepped) evaluations, the
same category already used by `ramjet.py`'s supercritical-recovery
correction (cited there as precedent for why this doesn't compromise Task
1's "closed-form, not time-integrated" intent -- it solves a well-posed
steady-state condition, not a truncated simulation).

Per `gate2_refill_phase_decision.md`'s four conditions:
1. Iteration count and tolerance are fixed module constants
   (`_MAX_PERIODIC_STATE_ITERATIONS`, `_PERIODIC_STATE_RELATIVE_TOLERANCE`),
   never caller-adjustable -- the one way this stage could quietly
   reintroduce the original per-caller-fidelity-disagreement bug.
2. Non-convergence within the fixed cap raises
   `periodic_state_did_not_converge` in `validity_flags`, not a silent
   last-iterate return.
3. The Mach-dependence claim above was checked at M=0.05, 0.1, 0.3, 0.6, 0.9
   (`reference_case.yaml`), not just the single M=0.1 case that first
   surfaced it.
4. Documented here as a correction to the original "altitude/density-limited"
   hypothesis, with both the prediction and the actual finding stated, per
   condition #4.

### Cycle time -- four-condition ignition trigger (`_solve_cycle`)

`pulsejet.py`'s `_ignite_if_ready` requires all four of: burn complete
(`t >= burn_duration_s`), minimum cycle period elapsed, chamber pressure
decayed to `ignition_pressure_ratio_max * Pt_inlet`, and enough fresh air
recaptured. The next ignition cannot happen before *all four* clear, so cycle
time is their max -- the actual, validated timing mechanism, unlike the
quarter-wave acoustic resonance formula the original fix proposal suggested
(decision #3, `gate2_pulsejet_fork_decisions.md`: rejected as the runtime
default -- `pulsejet_cycle_mode`'s own docstring already says quarter-wave is
"diagnostic/reporting only," never consulted by `PulsejetSimulator`, and the
governing-equations doc flags quarter-wave-vs-Helmholtz as an unresolved
literature debate). `quarter_wave_resonance_frequency_hz` is still called
here, but only to populate a reported, non-driving diagnostic field.

    f_cycle = 1 / max(burn_duration_s, minimum_cycle_period_s,
                       t_pressure_decay, t_fresh_air_recapture)

## Decision #4 -- no separate `cycle_model` config field

The original fix proposal asked for a `cycle_model: "lenoir"|"humphrey"`
option. Rejected as redundant (`gate2_pulsejet_fork_decisions.md`): in this
codebase's own terms (`pulsejet.py`'s `pulsejet_cycle_mode` docstring), the
Lenoir/Humphrey distinction is entirely about how much ram-pressure rise
counts as real pre-compression before ignition -- already exactly what
`SelectorConfig.inlet_type` (`"straight"`/`"side"`) controls via the
existing side-inlet ram-pressure derate. Phase A's heat-addition math is
identical either way; only the pre-ignition pressure/temperature state
feeding it differs, and that's already parameterized. Recorded here so this
isn't mistaken for an oversight later.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .compressible import (
    critical_pressure_ratio,
    fixed_cd_nozzle,
    stagnation_pressure,
    stagnation_temperature,
)
from .compressible import _internal_shock_exit_state  # reused, not re-derived -- see module docstring
from .config import Fuel, NozzleConfig, PulsejetConfig, SelectorConfig
from .pulsejet import quarter_wave_resonance_frequency_hz, side_inlet_ram_recovery_ratio
from .pulsejet import _INLET_AIR_GAMMA, _INLET_AIR_GAS_CONSTANT_J_PER_KG_K

# gate2_refill_phase_decision.md condition #1: fixed constants, never a
# caller-adjustable parameter (that would reintroduce the original bug one
# layer deeper).
_MAX_PERIODIC_STATE_ITERATIONS = 25
_PERIODIC_STATE_RELATIVE_TOLERANCE = 1e-4


@dataclass(frozen=True)
class PulsejetCorrectionCoefficients:
    """Fit (not derived) coefficients calibrated against `PulsejetSimulator`.

    All default to neutral (no correction) until
    `scripts/pulsejet_calibration_fit.py` has run and produced
    `configs/pulsejet_closed_form_calibration.yaml`. `calibrated=False` at
    these defaults -- always check that flag (surfaced as the
    `closed_form_uncalibrated` validity flag) before trusting output computed
    with it.
    """

    tail_impulse_fraction: float = 0.0
    """Extra impulse from the internal-shock/unchoked blowdown tail, as a
    fraction of the closed-form choked-phase impulse. FIT, NOT DERIVED
    (decision #1, gate2_pulsejet_fork_decisions.md) -- module docstring,
    Phase B."""

    tail_time_fraction: float = 0.0
    """Correction on how long the post-t1 tail actually lasts. FIT, NOT
    DERIVED, same basis as tail_impulse_fraction."""

    fresh_air_inflow_discount: float = 0.02
    """Scale on the reference choked inlet capacity that sets the refill
    mass flow rate (module docstring, Phase C / gate2_refill_phase_decision.md).
    This is the primary determinant of refill rate, not a small correction --
    FIT, NOT DERIVED, because the true low-Mach mechanism is a near-equilibrium
    effect below this quasi-static model's resolution. Default 0.02 is a
    rough order-of-magnitude placeholder from the validation spot-checks in
    the conversation that authorized this module (NOT a calibration fit --
    `calibrated=False` at this default), sized so f_cycle doesn't diverge to
    absurd values before real calibration lands."""

    net_thrust_efficiency: float = 1.0
    """Final multiplicative correction on mean net thrust, absorbing whatever
    the physically-named corrections above don't. Fit last; a large deviation
    from 1.0 here is itself a signal the structural model needs revisiting,
    not just another coefficient."""

    calibrated: bool = False
    source: str = "uncalibrated defaults"


UNCALIBRATED_COEFFICIENTS = PulsejetCorrectionCoefficients()


@dataclass(frozen=True)
class ClosedFormPulsejetResult:
    mach: float
    altitude_m: float
    mean_net_thrust_n: float
    mean_gross_thrust_n: float
    mean_fuel_mass_flow_kg_per_s: float
    specific_impulse_s: float
    cycle_frequency_hz: float
    cycle_time_s: float
    binding_ignition_condition: str
    """Which of the four `_ignite_if_ready` conditions set cycle time:
    'burn_duration', 'minimum_cycle_period', 'pressure_decay', or
    'fresh_air_recapture'."""
    choked_blowdown_transition_time_s: float
    periodic_state_converged: bool
    periodic_state_iterations: int
    outside_calibration_envelope: bool
    diagnostic_quarter_wave_frequency_hz: float | None
    """Reported only -- never drives the calculation above. See module
    docstring, decision #3."""
    calibrated: bool
    validity_flags: tuple[str, ...]


def _choked_mass_flow_kg_per_s(
    pressure_pa: float,
    temperature_k: float,
    area_m2: float,
    discharge_coefficient: float,
    gamma: float,
    gas_constant_j_per_kg_k: float,
) -> float:
    """Choked mass flow at one reservoir state -- same formula as
    `compressible.compressible_orifice_mass_flow`'s choked branch, reproduced
    symbolically here because the blowdown ODE needs it evaluated at an
    arbitrary future chamber state, not just numerically at one instant."""

    return (
        discharge_coefficient
        * area_m2
        * pressure_pa
        / sqrt(gas_constant_j_per_kg_k * temperature_k)
        * sqrt(gamma)
        * (2.0 / (gamma + 1.0)) ** ((gamma + 1.0) / (2.0 * (gamma - 1.0)))
    )


@dataclass(frozen=True)
class _InletState:
    freestream_velocity_m_per_s: float
    inlet_total_pressure_pa: float
    inlet_total_temperature_k: float


def _solve_inlet_state(selector: SelectorConfig, altitude_m: float, mach: float) -> _InletState:
    """Approximates `PulsejetSimulator.__init__`'s inlet stagnation-state block.

    NOT an exact reproduction as of 2026-08-08: `PulsejetSimulator` now
    recomputes the side-inlet ram-recovery ratio every step from the
    inertance model's own instantaneous captured mass flow
    (`side_inlet_ram_recovery_ratio`, pulsejet.py), since Hall & Frank's
    correlation is a function of mass-flow coefficient, not Mach. This
    closed-form module has no per-cycle mass-flow state to evaluate that
    against, so it falls back to the zero-flow anchor (0.50) as a static
    approximation -- flagged stale, same as this whole module's
    `calibrated=False` default, pending the closed-form re-derivation task
    (docs/design_convergence.md) that will need a cycle-representative
    mass-flow estimate here too, not just re-fit coefficients."""

    atmosphere = standard_atmosphere(altitude_m)
    freestream_velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
    inlet_total_temperature_k = stagnation_temperature(atmosphere.temperature_k, mach)
    ideal_total_pressure_pa = stagnation_pressure(atmosphere.pressure_pa, mach)
    if selector.inlet_type == "side":
        ram_pressure_rise_pa = ideal_total_pressure_pa - atmosphere.pressure_pa
        ideal_total_pressure_pa = (
            atmosphere.pressure_pa
            + side_inlet_ram_recovery_ratio(0.0) * ram_pressure_rise_pa
        )
    inlet_total_pressure_pa = ideal_total_pressure_pa * selector.pulsejet_total_pressure_recovery
    return _InletState(
        freestream_velocity_m_per_s=freestream_velocity_m_per_s,
        inlet_total_pressure_pa=inlet_total_pressure_pa,
        inlet_total_temperature_k=inlet_total_temperature_k,
    )


@dataclass(frozen=True)
class _IgnitionState:
    pre_ignition_mass_kg: float
    pre_ignition_fresh_air_kg: float
    pre_ignition_pressure_pa: float
    pre_ignition_temperature_k: float
    burnable_fuel_kg: float
    post_ignition_mass_kg: float
    post_ignition_pressure_pa: float
    post_ignition_temperature_k: float
    post_ignition_fresh_air_kg: float


def _solve_ignition(
    pulsejet: PulsejetConfig,
    fuel: Fuel,
    pre_ignition_mass_kg: float,
    pre_ignition_fresh_air_kg: float,
    pre_ignition_pressure_pa: float,
    pre_ignition_temperature_k: float,
) -> _IgnitionState:
    """Phase A -- see module docstring. Takes the pre-ignition state as given
    (solved for by `_solve_periodic_state`, not assumed here)."""

    cv_j_per_kg_k = pulsejet.gas_constant_j_per_kg_k / (pulsejet.gamma - 1.0)
    pre_ignition_fuel_kg = (
        pre_ignition_mass_kg * pulsejet.target_equivalence_ratio / fuel.stoichiometric_air_fuel_ratio
    )
    burnable_fuel_kg = min(
        pre_ignition_fuel_kg, pre_ignition_fresh_air_kg / fuel.stoichiometric_air_fuel_ratio
    )
    heat_release_j = burnable_fuel_kg * fuel.lower_heating_value_j_per_kg * pulsejet.combustion_efficiency
    post_ignition_fresh_air_kg = pre_ignition_fresh_air_kg - burnable_fuel_kg * fuel.stoichiometric_air_fuel_ratio
    # Phase A -- exact, see module docstring: p3/p2 = T3/T2 at fixed V, m.
    delta_temperature_k = heat_release_j / (max(pre_ignition_mass_kg, 1e-12) * cv_j_per_kg_k)
    post_ignition_temperature_k = pre_ignition_temperature_k + delta_temperature_k
    post_ignition_pressure_pa = pre_ignition_pressure_pa * (
        post_ignition_temperature_k / pre_ignition_temperature_k
    )
    return _IgnitionState(
        pre_ignition_mass_kg=pre_ignition_mass_kg,
        pre_ignition_fresh_air_kg=pre_ignition_fresh_air_kg,
        pre_ignition_pressure_pa=pre_ignition_pressure_pa,
        pre_ignition_temperature_k=pre_ignition_temperature_k,
        burnable_fuel_kg=burnable_fuel_kg,
        post_ignition_mass_kg=pre_ignition_mass_kg,  # heat addition: m, V fixed
        post_ignition_pressure_pa=post_ignition_pressure_pa,
        post_ignition_temperature_k=post_ignition_temperature_k,
        post_ignition_fresh_air_kg=post_ignition_fresh_air_kg,
    )


@dataclass(frozen=True)
class _ChokedBlowdownSolution:
    tau_s: float
    initial_gross_thrust_n: float
    ambient_pressure_pa: float
    exit_area_m2: float
    discharge_coefficient: float
    transition_pressure_pa: float
    transition_time_s: float
    gamma: float
    post_ignition_pressure_pa: float

    def time_to_pressure_s(self, target_pressure_pa: float) -> float:
        """Invert p(t) = target -- same closed form used for the choked/tail
        transition time, the ignition-pressure-trigger time, and the
        refill-start time. Extrapolates past t1 if `target_pressure_pa` is
        below `transition_pressure_pa` -- see module docstring, Phase B."""

        ratio = target_pressure_pa / self.post_ignition_pressure_pa
        if ratio >= 1.0:
            return 0.0
        a = (self.gamma - 1.0) / (2.0 * self.tau_s)
        return (ratio ** (-(self.gamma - 1.0) / (2.0 * self.gamma)) - 1.0) / a

    def mass_fraction_at_pressure(self, target_pressure_pa: float) -> float:
        ratio = min(target_pressure_pa / self.post_ignition_pressure_pa, 1.0)
        return ratio ** (1.0 / self.gamma)


def _solve_choked_blowdown(
    ignition: _IgnitionState, nozzle: NozzleConfig, pulsejet: PulsejetConfig, ambient_pressure_pa: float
) -> _ChokedBlowdownSolution:
    """Phase B -- see module docstring for the full derivation."""

    gamma = pulsejet.gamma
    gas_constant = pulsejet.gas_constant_j_per_kg_k
    nozzle_result_0 = fixed_cd_nozzle(
        ignition.post_ignition_pressure_pa,
        ignition.post_ignition_temperature_k,
        ambient_pressure_pa,
        nozzle.throat_area_m2,
        nozzle.exit_area_m2,
        nozzle.discharge_coefficient,
        gamma,
        gas_constant,
    )
    initial_mass_flow_kg_per_s = _choked_mass_flow_kg_per_s(
        ignition.post_ignition_pressure_pa,
        ignition.post_ignition_temperature_k,
        nozzle.throat_area_m2,
        nozzle.discharge_coefficient,
        gamma,
        gas_constant,
    )
    tau_s = ignition.post_ignition_mass_kg / max(initial_mass_flow_kg_per_s, 1e-12)

    if nozzle.exit_to_throat_area_ratio > 1.0:
        shock_at_exit_pressure_ratio, _, _ = _internal_shock_exit_state(
            nozzle.exit_to_throat_area_ratio, nozzle.exit_to_throat_area_ratio, gamma
        )
    else:
        shock_at_exit_pressure_ratio = critical_pressure_ratio(gamma)
    transition_pressure_pa = ambient_pressure_pa / shock_at_exit_pressure_ratio

    partial = _ChokedBlowdownSolution(
        tau_s=tau_s,
        initial_gross_thrust_n=nozzle_result_0.gross_thrust_n,
        ambient_pressure_pa=ambient_pressure_pa,
        exit_area_m2=nozzle.exit_area_m2,
        discharge_coefficient=nozzle.discharge_coefficient,
        transition_pressure_pa=transition_pressure_pa,
        transition_time_s=0.0,
        gamma=gamma,
        post_ignition_pressure_pa=ignition.post_ignition_pressure_pa,
    )
    transition_time_s = partial.time_to_pressure_s(min(transition_pressure_pa, ignition.post_ignition_pressure_pa))
    return _ChokedBlowdownSolution(
        tau_s=tau_s,
        initial_gross_thrust_n=nozzle_result_0.gross_thrust_n,
        ambient_pressure_pa=ambient_pressure_pa,
        exit_area_m2=nozzle.exit_area_m2,
        discharge_coefficient=nozzle.discharge_coefficient,
        transition_pressure_pa=transition_pressure_pa,
        transition_time_s=transition_time_s,
        gamma=gamma,
        post_ignition_pressure_pa=ignition.post_ignition_pressure_pa,
    )


def _choked_impulse_n_s(solution: _ChokedBlowdownSolution, t_s: float) -> float:
    """J(t) = integral of F from 0 to t -- closed form, see module docstring.
    Valid as-derived only for t <= transition_time_s; extrapolated beyond
    that (decision #1's tail correction applies to the difference)."""

    if t_s <= 0.0:
        return 0.0
    gamma = solution.gamma
    ambient_term_n = solution.ambient_pressure_pa * solution.exit_area_m2 * solution.discharge_coefficient
    f0_plus_ambient_term_n = solution.initial_gross_thrust_n + ambient_term_n
    a = (gamma - 1.0) / (2.0 * solution.tau_s)
    pressure_ratio_integral = (2.0 * solution.tau_s / (gamma + 1.0)) * (
        1.0 - (1.0 + a * t_s) ** (-(gamma + 1.0) / (gamma - 1.0))
    )
    return f0_plus_ambient_term_n * pressure_ratio_integral - ambient_term_n * t_s


@dataclass(frozen=True)
class _CycleSolution:
    ignition: _IgnitionState
    blowdown: _ChokedBlowdownSolution
    blowdown_end_time_s: float
    blowdown_end_mass_kg: float
    blowdown_end_fresh_air_kg: float
    cycle_time_s: float
    binding_condition: str
    refill_duration_s: float
    effective_inflow_kg_per_s: float
    net_impulse_n_s: float
    gross_impulse_n_s: float
    fuel_injected_kg: float
    end_of_cycle_mass_kg: float
    end_of_cycle_fresh_air_kg: float


def _solve_one_cycle(
    pulsejet: PulsejetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    inlet: _InletState,
    ambient_pressure_pa: float,
    pre_ignition_mass_kg: float,
    pre_ignition_fresh_air_kg: float,
    coefficients: PulsejetCorrectionCoefficients,
) -> _CycleSolution:
    # Pre-ignition pressure is pinned at Pt_inlet, not ambient -- the
    # empirically-confirmed equilibrium point (module docstring, Phase C).
    # Temperature is derived from the (fixed-point) mass via ideal gas at
    # that fixed pressure, not assumed independently.
    pre_ignition_pressure_pa = inlet.inlet_total_pressure_pa
    pre_ignition_temperature_k = (
        pre_ignition_pressure_pa
        * pulsejet.chamber_volume_m3
        / (max(pre_ignition_mass_kg, 1e-12) * pulsejet.gas_constant_j_per_kg_k)
    )
    ignition = _solve_ignition(
        pulsejet,
        fuel,
        pre_ignition_mass_kg,
        pre_ignition_fresh_air_kg,
        pre_ignition_pressure_pa,
        pre_ignition_temperature_k,
    )
    blowdown = _solve_choked_blowdown(ignition, nozzle, pulsejet, ambient_pressure_pa)

    t1_s = blowdown.transition_time_s
    raw_refill_start_time_s = blowdown.time_to_pressure_s(inlet.inlet_total_pressure_pa)
    blowdown_end_time_s = t1_s + max(raw_refill_start_time_s - t1_s, 0.0) * (
        1.0 + coefficients.tail_time_fraction
    )
    blowdown_end_mass_kg = ignition.post_ignition_mass_kg * blowdown.mass_fraction_at_pressure(
        inlet.inlet_total_pressure_pa
    )
    blowdown_end_fresh_air_kg = ignition.post_ignition_fresh_air_kg * (
        blowdown_end_mass_kg / max(ignition.post_ignition_mass_kg, 1e-12)
    )

    j_choked_raw_n_s = _choked_impulse_n_s(blowdown, min(t1_s, blowdown_end_time_s))
    j_tail_raw_n_s = _choked_impulse_n_s(blowdown, blowdown_end_time_s) - j_choked_raw_n_s
    gross_impulse_blowdown_n_s = (j_choked_raw_n_s + j_tail_raw_n_s) * (1.0 + coefficients.tail_impulse_fraction)

    # Phase C -- refill: fit-scale rate against a well-defined reference
    # (module docstring). C_reference is the inlet orifice's own choked
    # capacity at Pt_inlet/Tt_inlet -- never singular, unlike an
    # instantaneous downstream-pressure evaluation in the near-equilibrium band.
    c_reference_kg_per_s = _choked_mass_flow_kg_per_s(
        inlet.inlet_total_pressure_pa,
        inlet.inlet_total_temperature_k,
        selector.available_area_m2,
        selector.discharge_coefficient,
        _INLET_AIR_GAMMA,
        _INLET_AIR_GAS_CONSTANT_J_PER_KG_K,
    )
    effective_inflow_kg_per_s = max(c_reference_kg_per_s * coefficients.fresh_air_inflow_discount, 1e-12)

    fresh_air_threshold_kg = pulsejet.minimum_fresh_air_fraction * pre_ignition_mass_kg
    if blowdown_end_fresh_air_kg >= fresh_air_threshold_kg:
        t_fresh_air_s = blowdown_end_time_s
    else:
        t_fresh_air_s = blowdown_end_time_s + (
            fresh_air_threshold_kg - blowdown_end_fresh_air_kg
        ) / effective_inflow_kg_per_s

    # Ignition-pressure-trigger time: same closed form, extrapolated the
    # same way as the refill-start time above when the trigger threshold
    # falls below the strictly-valid choked regime.
    pressure_trigger_pa = pulsejet.ignition_pressure_ratio_max * inlet.inlet_total_pressure_pa
    if pressure_trigger_pa >= blowdown.transition_pressure_pa:
        t_pressure_s = blowdown.time_to_pressure_s(pressure_trigger_pa)
    else:
        raw_t_pressure_s = blowdown.time_to_pressure_s(pressure_trigger_pa)
        t_pressure_s = t1_s + max(raw_t_pressure_s - t1_s, 0.0) * (1.0 + coefficients.tail_time_fraction)

    candidates = {
        "burn_duration": pulsejet.burn_duration_s,
        "minimum_cycle_period": pulsejet.minimum_cycle_period_s,
        "pressure_decay": t_pressure_s,
        "fresh_air_recapture": t_fresh_air_s,
    }
    binding_condition = max(candidates, key=lambda name: candidates[name])
    cycle_time_s = candidates[binding_condition]

    refill_duration_s = max(cycle_time_s - blowdown_end_time_s, 0.0)
    inflow_mass_kg = effective_inflow_kg_per_s * refill_duration_s
    end_of_cycle_mass_kg = blowdown_end_mass_kg + inflow_mass_kg
    end_of_cycle_fresh_air_kg = min(blowdown_end_fresh_air_kg + inflow_mass_kg, end_of_cycle_mass_kg)

    inlet_momentum_drag_impulse_n_s = (
        effective_inflow_kg_per_s * inlet.freestream_velocity_m_per_s * refill_duration_s
    )
    fuel_injected_kg = (
        effective_inflow_kg_per_s * pulsejet.target_equivalence_ratio / fuel.stoichiometric_air_fuel_ratio
    ) * refill_duration_s

    net_impulse_n_s = (
        gross_impulse_blowdown_n_s - inlet_momentum_drag_impulse_n_s
    ) * coefficients.net_thrust_efficiency
    gross_impulse_n_s = gross_impulse_blowdown_n_s * coefficients.net_thrust_efficiency

    return _CycleSolution(
        ignition=ignition,
        blowdown=blowdown,
        blowdown_end_time_s=blowdown_end_time_s,
        blowdown_end_mass_kg=blowdown_end_mass_kg,
        blowdown_end_fresh_air_kg=blowdown_end_fresh_air_kg,
        cycle_time_s=cycle_time_s,
        binding_condition=binding_condition,
        refill_duration_s=refill_duration_s,
        effective_inflow_kg_per_s=effective_inflow_kg_per_s,
        net_impulse_n_s=net_impulse_n_s,
        gross_impulse_n_s=gross_impulse_n_s,
        fuel_injected_kg=fuel_injected_kg,
        end_of_cycle_mass_kg=end_of_cycle_mass_kg,
        end_of_cycle_fresh_air_kg=end_of_cycle_fresh_air_kg,
    )


@dataclass(frozen=True)
class _PeriodicState:
    cycle: _CycleSolution
    converged: bool
    iterations: int


def _solve_periodic_state(
    pulsejet: PulsejetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    inlet: _InletState,
    ambient_pressure_pa: float,
    ambient_temperature_k: float,
    coefficients: PulsejetCorrectionCoefficients,
) -> _PeriodicState:
    """Fixed-point iteration to a self-consistent pre-ignition state -- see
    module docstring, "Periodic steady state", and
    gate2_refill_phase_decision.md. Iteration count/tolerance are the fixed
    module constants (`_MAX_PERIODIC_STATE_ITERATIONS`,
    `_PERIODIC_STATE_RELATIVE_TOLERANCE`) -- condition #1, never caller-set."""

    # Initial guess: chamber at Pt_inlet, ambient temperature, fully fresh air.
    mass_guess_kg = (
        inlet.inlet_total_pressure_pa
        * pulsejet.chamber_volume_m3
        / (pulsejet.gas_constant_j_per_kg_k * ambient_temperature_k)
    )
    fresh_air_guess_kg = mass_guess_kg

    cycle: _CycleSolution | None = None
    converged = False
    iterations = 0
    for iterations in range(1, _MAX_PERIODIC_STATE_ITERATIONS + 1):
        cycle = _solve_one_cycle(
            pulsejet,
            selector,
            nozzle,
            fuel,
            inlet,
            ambient_pressure_pa,
            mass_guess_kg,
            fresh_air_guess_kg,
            coefficients,
        )
        new_mass_kg = cycle.end_of_cycle_mass_kg
        new_fresh_air_kg = cycle.end_of_cycle_fresh_air_kg
        mass_change = abs(new_mass_kg - mass_guess_kg) / max(mass_guess_kg, 1e-12)
        fresh_air_change = abs(new_fresh_air_kg - fresh_air_guess_kg) / max(fresh_air_guess_kg, 1e-12)
        mass_guess_kg, fresh_air_guess_kg = new_mass_kg, new_fresh_air_kg
        if (
            mass_change < _PERIODIC_STATE_RELATIVE_TOLERANCE
            and fresh_air_change < _PERIODIC_STATE_RELATIVE_TOLERANCE
        ):
            converged = True
            break

    assert cycle is not None
    return _PeriodicState(cycle=cycle, converged=converged, iterations=iterations)


def _solve_cycle(
    pulsejet: PulsejetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
    coefficients: PulsejetCorrectionCoefficients,
) -> ClosedFormPulsejetResult:
    atmosphere = standard_atmosphere(altitude_m)
    inlet = _solve_inlet_state(selector, altitude_m, mach)
    periodic = _solve_periodic_state(
        pulsejet, selector, nozzle, fuel, inlet, atmosphere.pressure_pa, atmosphere.temperature_k, coefficients
    )
    cycle = periodic.cycle

    mean_net_thrust_n = cycle.net_impulse_n_s / cycle.cycle_time_s
    mean_gross_thrust_n = cycle.gross_impulse_n_s / cycle.cycle_time_s
    mean_fuel_flow_kg_per_s = cycle.fuel_injected_kg / cycle.cycle_time_s
    specific_impulse_s = (
        mean_net_thrust_n / (mean_fuel_flow_kg_per_s * G0_M_PER_S2)
        if mean_fuel_flow_kg_per_s > 1e-12
        else 0.0
    )

    diagnostic_frequency_hz = None
    try:
        hot_gas_speed_of_sound_m_per_s = sqrt(
            pulsejet.gamma * pulsejet.gas_constant_j_per_kg_k * cycle.ignition.post_ignition_temperature_k
        )
        effective_tube_length_m = nozzle.exit_area_m2**0.5  # coarse geometric proxy, diagnostic only
        diagnostic_frequency_hz = quarter_wave_resonance_frequency_hz(
            hot_gas_speed_of_sound_m_per_s, effective_tube_length_m
        )
    except ValueError:
        pass

    validity_flags: list[str] = []
    if not coefficients.calibrated:
        validity_flags.append("closed_form_uncalibrated")
    if not periodic.converged:
        # gate2_refill_phase_decision.md condition #2: explicit flag, not a
        # silent return of the last (unconverged) iterate.
        validity_flags.append("periodic_state_did_not_converge")

    return ClosedFormPulsejetResult(
        mach=mach,
        altitude_m=altitude_m,
        mean_net_thrust_n=mean_net_thrust_n,
        mean_gross_thrust_n=mean_gross_thrust_n,
        mean_fuel_mass_flow_kg_per_s=mean_fuel_flow_kg_per_s,
        specific_impulse_s=specific_impulse_s,
        cycle_frequency_hz=1.0 / cycle.cycle_time_s,
        cycle_time_s=cycle.cycle_time_s,
        binding_ignition_condition=cycle.binding_condition,
        choked_blowdown_transition_time_s=cycle.blowdown.transition_time_s,
        periodic_state_converged=periodic.converged,
        periodic_state_iterations=periodic.iterations,
        outside_calibration_envelope=not coefficients.calibrated or not periodic.converged,
        diagnostic_quarter_wave_frequency_hz=diagnostic_frequency_hz,
        calibrated=coefficients.calibrated,
        validity_flags=tuple(validity_flags),
    )


def evaluate_pulsejet_closed_form(
    pulsejet: PulsejetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
    coefficients: PulsejetCorrectionCoefficients = UNCALIBRATED_COEFFICIENTS,
) -> ClosedFormPulsejetResult:
    """Gate 2's default pulsejet evaluation path (`propulsion_map.py`'s
    `_pulsejet_point`, `force_unsteady=False`). See module docstring."""

    if mach < 0.0:
        raise ValueError("Mach cannot be negative")
    return _solve_cycle(pulsejet, selector, nozzle, fuel, altitude_m, mach, coefficients)
