"""Hybrid-event, zero-dimensional pulsejet chamber model."""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from statistics import fmean

from .atmosphere import G0_M_PER_S2, Atmosphere, standard_atmosphere
from .compressible import (
    fixed_cd_nozzle,
    stagnation_pressure,
    stagnation_temperature,
)
from .config import Fuel, NozzleConfig, PulsejetConfig, SelectorConfig


# The inlet stream is treated as calorically perfect ambient air. Fuel sensible
# enthalpy is a small constant-cp placeholder relative to its chemical heat release;
# both become explicit calibration inputs when variable properties are introduced.
_INLET_AIR_GAMMA = 1.4
_INLET_AIR_GAS_CONSTANT_J_PER_KG_K = 287.05287
_FUEL_SENSIBLE_SPECIFIC_HEAT_J_PER_KG_K = 2_000.0

# docs/pulsejet_ramjet_governing_equations.md sec. 2.2: a side-mounted inlet shows
# "little pre-compression regardless of flight speed," unlike a straight inlet
# whose pre-compression genuinely grows with Mach. The reference gives no
# quantitative side-inlet correlation, only that qualitative statement. This was
# previously an UNSOURCED flat 0.15 engineering placeholder
# (SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION, removed 2026-08-08). Replaced with a
# real external correlation now that the vehicle's flush/side-mounted reed-valve
# architecture is confirmed (not a forward-facing inlet): Hall, Charles F., and
# Frank, Joseph L., "Ram-Recovery Characteristics of NACA Submerged Inlets at
# High Subsonic Speeds," NACA RM A8I29, 1948 -- flush fuselage inlets at Mach
# 0.30-0.875. Reports ram-recovery ratio (fraction of the ideal ram-pressure
# rise actually recovered) driven almost entirely by mass-flow coefficient
# (captured mass flow / freestream mass flow through the capture area), not
# Mach number or angle of attack (both under 0.03 effect): representative
# values 0.50 at zero flow, 0.90 at a 0.6 mass-flow coefficient, 0.95 near 1.0.
# Coupled to Part 1's inertance model, not independent: evaluated each step
# (see `step()`) from the same captured `inlet_mass_flow_kg_per_s` state the
# inertance ODE integrates, not a separate/assumed flow rate.
_SIDE_INLET_RAM_RECOVERY_ANCHORS: tuple[tuple[float, float], ...] = (
    (0.0, 0.50),
    (0.6, 0.90),
    (1.0, 0.95),
)


def side_inlet_ram_recovery_ratio(mass_flow_coefficient: float) -> float:
    """Hall & Frank (NACA RM A8I29) submerged-inlet ram-recovery correlation.

    Piecewise-linear through the three reported anchor points (see module-level
    comment above). Held flat at the endpoints outside [0, 1] -- 0.50 below
    zero flow (not physically reachable, but keeps the function total), 0.95
    above a mass-flow coefficient of 1.0 since the source gives no data there
    (an extrapolation, not a cited value).
    """

    if mass_flow_coefficient <= _SIDE_INLET_RAM_RECOVERY_ANCHORS[0][0]:
        return _SIDE_INLET_RAM_RECOVERY_ANCHORS[0][1]
    if mass_flow_coefficient >= _SIDE_INLET_RAM_RECOVERY_ANCHORS[-1][0]:
        return _SIDE_INLET_RAM_RECOVERY_ANCHORS[-1][1]
    for (x0, y0), (x1, y1) in zip(
        _SIDE_INLET_RAM_RECOVERY_ANCHORS, _SIDE_INLET_RAM_RECOVERY_ANCHORS[1:]
    ):
        if x0 <= mass_flow_coefficient <= x1:
            fraction = (mass_flow_coefficient - x0) / (x1 - x0)
            return y0 + fraction * (y1 - y0)
    raise AssertionError("unreachable")  # pragma: no cover

# Fluid-inertance inlet model (replaces the instantaneous
# compressible_orifice_mass_flow inlet calculation as of 2026-08-08).
#
# Root cause this fixes: an instantaneous orifice model has the inlet flow
# snap to whatever the current pressure differential implies, with no memory
# of prior flow state -- confirmed (this session's Task 1 audit) as the
# structure of both PulsejetSimulator's old inlet calc and the closed form's
# Phase C. NASA/TM-2008-215432 (Geng, Paxson, et al., "Comparison Between
# Numerically Simulated and Experimentally Measured Flowfield Quantities
# Behind a Pulsejet") found this same instantaneous structure underpredicted
# static thrust by 14% (16.44N vs. 19.13N measured) for a valved pulsejet
# architecturally like this one; adding inlet flow inertia dropped the error
# to 1.7%. Their eq. (2) (transcribed from the actual typeset PDF page, not
# OCR text -- pdftotext mangled the stacked fractions):
#
#   dmdot/dt = 12 * (A_in/L_in) * gc * (P_in - P_up)
#
# The "12" and "gc" (their nomenclature: "Newton constant") are English-unit-
# system artifacts (a probable inches-to-feet conversion paired with the
# lbm/lbf force-mass conversion constant) -- both drop out in this codebase's
# SI convention, confirmed not needed, not silently carried over:
#
#   dmdot/dt = (A_in/L_in) * (P_in - P_up)                    [this module]
#
# Their eq. (3)-(4) (reed valve velocity/position, driven by spring constant
# k_v and valve mass m_v) are NOT ported: that's structural mechanics for
# hardware that was never characterized numerically in the literature (their
# own cited experimental source, Paxson/Wilson/Dougherty AIAA-2002-3915,
# states the valve was "obtained directly from a Dynajet pulsejet" -- a
# commercial part with no published mass/stiffness) and doesn't exist yet for
# this vehicle's own valve. Per user decision (revised_inertance_plan_no_beam_
# mechanics.md, 2026-08-08): the reed valve itself is instead treated as an
# idealized instantaneous check valve (fully open when the pressure
# differential favors inflow, fully closed otherwise, no valve inertia of its
# own) -- a literature-supported simplification, not an improvised shortcut:
# Ghulam, Muralidharan, Anand, Prisell, and Gutmark, "Operational Mechanism of
# Valved-Pulsejet Engines," Aerospace Science and Technology, vol. 148, 2024,
# art. 109060, model reed-valved pulsejets with exactly this two-state
# (open/closed) idealization and report good agreement with experiment,
# particularly for the simplest configuration.
#
# P_up (pressure just upstream of the reed valve, NASA nomenclature) is taken
# here as the current chamber pressure directly -- with the reed valve's own
# dynamics idealized away (no separate valve-head pressure node), the duct
# connects the inlet reservoir (Pt_inlet) to the chamber, gated by the check
# valve, which is the simplest model consistent with dropping eq. (3)-(4).
#
# A_in, L_in: geometric estimates, NOT hardware measurements -- this is the
# whole point of the simplified approach (Eq. 2 only needs inlet duct
# geometry, unlike eq. 3-4's reed hardware properties). Both are explicitly
# first-pass, adjustable placeholders, not literature or measured values:
_INLET_DUCT_OPEN_FRACTION_ESTIMATE = 0.75
"""Fraction of the intake circumference assumed open to flow through a side-
mounted reed-valve array. First-pass geometric estimate (revised_inertance_
plan_no_beam_mechanics.md), not a measured or sourced value."""
_INLET_DUCT_PORT_WIDTH_ESTIMATE_M = 0.0508  # 2.0 in
"""Assumed axial width of the reed-valve band. First-pass geometric estimate,
same basis as _INLET_DUCT_OPEN_FRACTION_ESTIMATE -- "a few inches" per the
authorizing decision, not a measured or sourced value."""
_INLET_DUCT_LENGTH_TO_DIAMETER_ESTIMATE = 1.0
"""L_in assumed equal to one intake diameter (a short inlet duct/valve-head
section). First-pass geometric estimate, adjustable once the actual valve
layout is designed in more detail -- not a measured or sourced value."""


def _inertance_inlet_duct_area_m2(selector: SelectorConfig) -> float:
    """A_in: first-pass geometric estimate, see module-level inertance-model
    comment block above. NOT selector.available_area_m2/circular_area_m2 --
    those describe the pulsejet's *captured* flow area convention used
    elsewhere in this module; A_in here is NASA/TM-2008-215432 eq. (2)'s own
    inlet-duct cross-section, a distinct geometric quantity by construction
    of the inertance model (the duct that has fluid mass to accelerate)."""

    circumference_m = 3.141592653589793 * selector.circular_intake_diameter_m
    return (
        _INLET_DUCT_OPEN_FRACTION_ESTIMATE
        * circumference_m
        * _INLET_DUCT_PORT_WIDTH_ESTIMATE_M
    )


def _inertance_inlet_duct_length_m(selector: SelectorConfig) -> float:
    """L_in: first-pass geometric estimate, see module-level inertance-model
    comment block above."""

    return _INLET_DUCT_LENGTH_TO_DIAMETER_ESTIMATE * selector.circular_intake_diameter_m


def pulsejet_cycle_mode(mach: float, inlet_type: str) -> str:
    """Return which idealized pulsejet cycle regime applies (sec. 2.2, 2.4).

    A straight inlet's charge genuinely pre-compresses as flight speed increases:
    Lenoir (no pre-compression) at zero/near-zero Mach, shifting toward Humphrey
    (real pre-compression before constant-volume heat addition) as Mach rises. A
    side inlet stays close to Lenoir at any speed. This is diagnostic/reporting
    only -- ``PulsejetSimulator`` itself is a numerical mass/energy-conservation
    model, not a closed-form cycle calculation, so it does not branch on this
    label; instead it reproduces the same physical trend through the Mach-
    dependent inlet stagnation pressure computed in ``__init__``/``step()`` (damped
    for side inlets via ``side_inlet_ram_recovery_ratio``).
    """

    if inlet_type == "side":
        return "lenoir"
    if mach < 0.05:
        return "lenoir"
    if mach < 0.30:
        return "lenoir_to_humphrey_transitional"
    return "humphrey"


def humphrey_cycle_thermal_efficiency(pressure_ratio: float, gamma: float) -> float:
    """Idealized Humphrey (constant-volume heat addition) thermal efficiency.

    docs/pulsejet_ramjet_governing_equations.md sec. 2.4:
    ``eta = 1 - gamma * (tau**(1/gamma) - 1) / (tau - 1)``, tau = p3/p2 the
    constant-volume pressure ratio. This is the idealized closed-form reference
    value, provided for validating/sanity-checking the numerical simulator's own
    effective thermal efficiency -- ``PulsejetSimulator`` does not use this
    formula directly, since it integrates real unsteady mass/energy flows rather
    than an idealized closed cycle.
    """

    if pressure_ratio <= 1.0 or gamma <= 1.0:
        raise ValueError("pressure ratio must exceed one and gamma must exceed one")
    return 1.0 - gamma * (pressure_ratio ** (1.0 / gamma) - 1.0) / (pressure_ratio - 1.0)


# docs/pulsejet_ramjet_governing_equations.md sec. 2.4 explicitly flags an UNRESOLVED debate over
# whether valved-pulsejet combustion behaves like a quarter-wave organ-pipe tube
# or a Helmholtz resonator. Only the quarter-wave model is implemented below;
# treat its output as one candidate estimate to validate against test data, not
# a settled closed-form truth. A Helmholtz-resonator model is NOT implemented.
RESONANCE_FREQUENCY_MODEL = "quarter_wave"
RESONANCE_FREQUENCY_MODEL_ALTERNATIVES = ("helmholtz",)  # not implemented; sec. 2.4


def quarter_wave_resonance_frequency_hz(
    hot_gas_speed_of_sound_m_per_s: float, effective_tube_length_m: float
) -> float:
    """Quarter-wave (one-open/one-closed-end organ-pipe) resonance frequency.

    docs/pulsejet_ramjet_governing_equations.md sec. 2.4: ``f = a / (4*L_eff)``, where ``a`` is the
    local speed of sound in the hot combustion gas (not ambient air) and
    ``L_eff`` is the acoustic length from the combustion-chamber center to the
    tailpipe exit, including end corrections. See ``RESONANCE_FREQUENCY_MODEL``
    for the competing-model caveat.
    """

    if hot_gas_speed_of_sound_m_per_s <= 0.0 or effective_tube_length_m <= 0.0:
        raise ValueError("speed of sound and effective tube length must be positive")
    return hot_gas_speed_of_sound_m_per_s / (4.0 * effective_tube_length_m)


# docs/pulsejet_ramjet_governing_equations.md sec. 2.4: the semi-empirical valveless-pulsejet
# frequency/mean-thrust correlation is reported to have errors under 10%
# (frequency) and up to +-17% (mean thrust) against experimental data, and is
# stated to also apply to valved pulsejets. The reference describes this
# correlation's existence and reported accuracy but does NOT give its closed-
# form equation, so it is not implemented here (see the session changelog's
# "still open" list). These constants exist so downstream callers can apply the
# reported accuracy ceiling to this codebase's own thrust numbers, not as a
# claim that this simulator has itself been validated to that accuracy.
RESONANCE_FREQUENCY_RELATIVE_UNCERTAINTY = 0.10
RESONANCE_MEAN_THRUST_RELATIVE_UNCERTAINTY = 0.17


def resonance_mean_thrust_uncertainty_band_n(mean_thrust_n: float) -> tuple[float, float]:
    """Return a (low, high) bound applying the sec. 2.4 +-17% thrust uncertainty.

    A useful reality check per the reference: "a sizing tool claiming much
    tighter agreement without equivalent experimental validation should be
    treated with suspicion." Apply this to any mean thrust value from this
    codebase's own model (e.g. ``PulsejetSummary.mean_net_thrust_n``).
    """

    if mean_thrust_n < 0.0:
        raise ValueError("mean thrust cannot be negative")
    return (
        mean_thrust_n * (1.0 - RESONANCE_MEAN_THRUST_RELATIVE_UNCERTAINTY),
        mean_thrust_n * (1.0 + RESONANCE_MEAN_THRUST_RELATIVE_UNCERTAINTY),
    )


@dataclass
class PulsejetState:
    time_s: float
    total_mass_kg: float
    fresh_air_mass_kg: float
    unburned_fuel_mass_kg: float
    internal_energy_j: float
    pending_heat_release_j: float
    burn_time_remaining_s: float
    last_ignition_time_s: float
    cycle_count: int
    inlet_mass_flow_kg_per_s: float
    """State, not an instantaneous quantity -- integrated by the fluid-
    inertance inlet model (see INERTANCE_* constants and `step()`), not
    recomputed fresh each step from the current pressure differential."""
    cumulative_fuel_injected_kg: float
    cumulative_fuel_burned_kg: float
    cumulative_air_ingested_kg: float
    cumulative_exhaust_discharged_kg: float
    cumulative_inlet_enthalpy_j: float
    cumulative_exhaust_enthalpy_j: float
    cumulative_combustion_heat_added_j: float
    cumulative_heat_rejected_j: float
    cumulative_wall_heat_rejected_j: float
    cumulative_temperature_limit_heat_rejected_j: float
    cumulative_numerical_energy_added_j: float


@dataclass(frozen=True)
class PulsejetSample:
    time_s: float
    chamber_pressure_pa: float
    chamber_temperature_k: float
    inlet_air_mass_flow_kg_per_s: float
    fuel_mass_flow_kg_per_s: float
    exhaust_mass_flow_kg_per_s: float
    gross_thrust_n: float
    inlet_momentum_drag_n: float
    net_thrust_n: float
    phase: str
    event: str | None
    cycle_count: int
    nozzle_warning: str | None


@dataclass(frozen=True)
class PulsejetSummary:
    window_start_s: float
    window_end_s: float
    duration_s: float
    completed_cycles: int
    mean_gross_thrust_n: float
    peak_gross_thrust_n: float
    mean_net_thrust_n: float
    peak_net_thrust_n: float
    mean_fuel_mass_flow_kg_per_s: float
    peak_chamber_pressure_pa: float
    peak_chamber_temperature_k: float
    final_chamber_pressure_pa: float
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.5: I_sp = F_net / (mdot_fuel * g0).
    specific_impulse_s: float
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class PulsejetConservationAudit:
    initial_total_mass_kg: float
    final_total_mass_kg: float
    cumulative_air_ingested_kg: float
    cumulative_fuel_injected_after_start_kg: float
    cumulative_exhaust_discharged_kg: float
    mass_balance_residual_kg: float
    relative_mass_balance_residual: float
    initial_internal_energy_j: float
    final_internal_energy_j: float
    cumulative_inlet_enthalpy_j: float
    cumulative_exhaust_enthalpy_j: float
    cumulative_combustion_heat_added_j: float
    cumulative_heat_rejected_j: float
    cumulative_wall_heat_rejected_j: float
    cumulative_temperature_limit_heat_rejected_j: float
    cumulative_numerical_energy_added_j: float
    energy_balance_residual_j: float
    relative_energy_balance_residual: float
    numerical_reference_only: bool = True


class PulsejetSimulator:
    """Advance a constant-volume chamber with discrete ignition events.

    The model conserves tracked mass and energy at the lumped-control-volume level.
    Combustion products are not species-resolved, and gamma/R remain constant.
    """

    def __init__(
        self,
        config: PulsejetConfig,
        selector: SelectorConfig,
        nozzle: NozzleConfig,
        fuel: Fuel,
        altitude_m: float,
        mach: float,
    ) -> None:
        if mach < 0.0:
            raise ValueError("Mach cannot be negative")
        self.config = config
        self.selector = selector
        self.nozzle = nozzle
        self.fuel = fuel
        self.atmosphere: Atmosphere = standard_atmosphere(altitude_m)
        self.mach = mach
        self.freestream_velocity_m_per_s = (
            mach * self.atmosphere.speed_of_sound_m_per_s
        )
        self.inlet_total_temperature_k = stagnation_temperature(
            self.atmosphere.temperature_k, mach
        )
        # docs/pulsejet_ramjet_governing_equations.md sec. 2.2: a straight inlet's
        # pre-compression genuinely grows with Mach (credit the full ram-pressure
        # rise); a side inlet shows little pre-compression at any speed, so only
        # the mass-flow-coefficient-dependent fraction reported by Hall & Frank
        # (NACA RM A8I29) is credited -- see side_inlet_ram_recovery_ratio above.
        ideal_stagnation_pressure_pa = stagnation_pressure(self.atmosphere.pressure_pa, mach)
        self.ram_pressure_rise_pa = ideal_stagnation_pressure_pa - self.atmosphere.pressure_pa
        self.cycle_mode = pulsejet_cycle_mode(mach, selector.inlet_type)
        self.side_inlet_ram_recovery_active = selector.inlet_type == "side"
        if self.side_inlet_ram_recovery_active:
            # No flow history yet at t=0 -- start from the zero-flow anchor;
            # step() recomputes this every step from the inertance model's own
            # captured mass flow once the simulation is running.
            ideal_total_pressure_pa = (
                self.atmosphere.pressure_pa
                + side_inlet_ram_recovery_ratio(0.0) * self.ram_pressure_rise_pa
            )
        else:
            ideal_total_pressure_pa = ideal_stagnation_pressure_pa
        self.inlet_total_pressure_pa = (
            ideal_total_pressure_pa * selector.pulsejet_total_pressure_recovery
        )
        # Fluid-inertance inlet model geometry -- see module-level comment
        # block above (_INLET_DUCT_*, _inertance_inlet_duct_area_m2/length_m).
        self.inertance_duct_area_m2 = _inertance_inlet_duct_area_m2(selector)
        self.inertance_duct_length_m = _inertance_inlet_duct_length_m(selector)
        self.cv_j_per_kg_k = config.gas_constant_j_per_kg_k / (config.gamma - 1.0)
        self.cp_j_per_kg_k = config.gamma * self.cv_j_per_kg_k
        self.initial_air_reference_mass_kg = (
            self.atmosphere.pressure_pa
            * config.chamber_volume_m3
            / (config.gas_constant_j_per_kg_k * self.atmosphere.temperature_k)
        )
        initial_fuel_mass_kg = (
            self.initial_air_reference_mass_kg
            * config.initial_equivalence_ratio
            / fuel.stoichiometric_air_fuel_ratio
        )
        total_mass_kg = self.initial_air_reference_mass_kg + initial_fuel_mass_kg
        self.initial_fuel_mass_kg = initial_fuel_mass_kg
        self.initial_total_mass_kg = total_mass_kg
        self.initial_internal_energy_j = (
            total_mass_kg * self.cv_j_per_kg_k * self.atmosphere.temperature_k
        )
        self.state = PulsejetState(
            time_s=0.0,
            total_mass_kg=total_mass_kg,
            fresh_air_mass_kg=self.initial_air_reference_mass_kg,
            unburned_fuel_mass_kg=initial_fuel_mass_kg,
            internal_energy_j=self.initial_internal_energy_j,
            pending_heat_release_j=0.0,
            burn_time_remaining_s=0.0,
            last_ignition_time_s=-config.minimum_cycle_period_s,
            cycle_count=0,
            inlet_mass_flow_kg_per_s=0.0,
            cumulative_fuel_injected_kg=initial_fuel_mass_kg,
            cumulative_fuel_burned_kg=0.0,
            cumulative_air_ingested_kg=0.0,
            cumulative_exhaust_discharged_kg=0.0,
            cumulative_inlet_enthalpy_j=0.0,
            cumulative_exhaust_enthalpy_j=0.0,
            cumulative_combustion_heat_added_j=0.0,
            cumulative_heat_rejected_j=0.0,
            cumulative_wall_heat_rejected_j=0.0,
            cumulative_temperature_limit_heat_rejected_j=0.0,
            cumulative_numerical_energy_added_j=0.0,
        )

    @property
    def temperature_k(self) -> float:
        return self.state.internal_energy_j / (
            max(self.state.total_mass_kg, 1e-12) * self.cv_j_per_kg_k
        )

    @property
    def pressure_pa(self) -> float:
        return (
            self.state.total_mass_kg
            * self.config.gas_constant_j_per_kg_k
            * self.temperature_k
            / self.config.chamber_volume_m3
        )

    def _ignite_if_ready(self, chamber_pressure_pa: float) -> str | None:
        # Humphrey/Lenoir cycle mapping (docs/pulsejet_ramjet_governing_equations.md sec. 2.4): the
        # mass-flow-in phase between ignitions is this simulation's analogue of
        # process 1->2 (isentropic pre-compression, credited per inlet_type and
        # Mach in __init__'s inlet_total_pressure_pa); the constant-chamber-volume
        # heat release below is process 2->3 (p3/p2 = T3/T2 at fixed volume); the
        # nozzle expansion in step() is process 3->4. This is an unsteady
        # conservation simulation, not the closed-form idealized cycle, so it does
        # not evaluate humphrey_cycle_thermal_efficiency() directly -- that
        # function exists to sanity-check this simulation's own effective
        # efficiency against the idealized reference value.
        #
        # chamber_pressure_pa is passed in rather than re-read via
        # self.pressure_pa: step() already computes it from the same
        # unchanged pre-step state (self.pressure_pa/self.temperature_k are
        # pure functions of state.total_mass_kg/internal_energy_j, and
        # nothing between step()'s entry and this call mutates either) --
        # profiled as a measurable share of PulsejetSimulator.step()'s cost
        # across the 700K+ steps a typical design-optimize run makes.
        state = self.state
        config = self.config
        ready = (
            state.pending_heat_release_j <= 1e-9
            and state.time_s - state.last_ignition_time_s >= config.minimum_cycle_period_s
            and chamber_pressure_pa
            <= config.ignition_pressure_ratio_max * self.inlet_total_pressure_pa
            and state.fresh_air_mass_kg
            >= config.minimum_fresh_air_fraction * self.initial_air_reference_mass_kg
            and state.unburned_fuel_mass_kg > 1e-9
        )
        if not ready:
            return None

        burnable_fuel_kg = min(
            state.unburned_fuel_mass_kg,
            state.fresh_air_mass_kg / self.fuel.stoichiometric_air_fuel_ratio,
        )
        if burnable_fuel_kg <= 1e-9:
            return None
        state.unburned_fuel_mass_kg -= burnable_fuel_kg
        state.fresh_air_mass_kg -= burnable_fuel_kg * self.fuel.stoichiometric_air_fuel_ratio
        state.pending_heat_release_j += (
            burnable_fuel_kg
            * self.fuel.lower_heating_value_j_per_kg
            * config.combustion_efficiency
        )
        state.burn_time_remaining_s = config.burn_duration_s
        state.last_ignition_time_s = state.time_s
        state.cycle_count += 1
        state.cumulative_fuel_burned_kg += burnable_fuel_kg
        return "ignition"

    def step(self, time_step_s: float) -> PulsejetSample:
        if time_step_s <= 0.0:
            raise ValueError("time step must be positive")
        state = self.state
        config = self.config
        if self.side_inlet_ram_recovery_active:
            # Coupled to Part 1's inertance model (see module-level comment
            # block and side_inlet_ram_recovery_ratio above): recompute the
            # recovery ratio each step from the *previous* step's captured
            # mass flow (the inertance ODE's own state, semi-implicit/lagged
            # the same way that ODE's own Euler integration already is) --
            # not a separately assumed flow rate. Quiescent parts of the cycle
            # (near-zero flow) pull recovery toward 0.50; peak refill flow
            # pulls it toward 0.90-0.95.
            if self.freestream_velocity_m_per_s > 1e-9:
                mass_flow_coefficient = state.inlet_mass_flow_kg_per_s / (
                    self.atmosphere.density_kg_per_m3
                    * self.freestream_velocity_m_per_s
                    * self.selector.available_area_m2
                )
            else:
                mass_flow_coefficient = 0.0
            recovery_ratio = side_inlet_ram_recovery_ratio(mass_flow_coefficient)
            ideal_total_pressure_pa = (
                self.atmosphere.pressure_pa + recovery_ratio * self.ram_pressure_rise_pa
            )
            self.inlet_total_pressure_pa = (
                ideal_total_pressure_pa * self.selector.pulsejet_total_pressure_recovery
            )
        chamber_pressure_pa = self.pressure_pa
        chamber_temperature_k = self.temperature_k
        event = self._ignite_if_ready(chamber_pressure_pa)
        # Fluid-inertance inlet model (see module-level comment block) --
        # replaces the old instantaneous compressible_orifice_mass_flow call.
        # P_up (NASA nomenclature) is taken as the current chamber pressure
        # directly (reed valve dynamics idealized away, no separate valve-
        # head pressure node -- see that comment block for why).
        if self.inlet_total_pressure_pa > chamber_pressure_pa:
            # Idealized check valve: open, integrate the inertance ODE
            # (Euler, matching NASA/TM-2008-215432's own integration scheme
            # for eq. 2-4, one first-order step per pulsejet timestep).
            mass_flow_rate_of_change_kg_per_s2 = (
                self.inertance_duct_area_m2
                / self.inertance_duct_length_m
                * (self.inlet_total_pressure_pa - chamber_pressure_pa)
            )
            state.inlet_mass_flow_kg_per_s = max(
                0.0,
                state.inlet_mass_flow_kg_per_s
                + mass_flow_rate_of_change_kg_per_s2 * time_step_s,
            )
        else:
            # Closed: pressure differential opposes inflow. A check valve
            # prevents backflow and, idealized as instantaneous, also removes
            # the duct's stored momentum rather than letting it coast through
            # a reversal -- flow resumes from rest once the valve reopens.
            state.inlet_mass_flow_kg_per_s = 0.0
        inlet_air_mass_flow_kg_per_s = state.inlet_mass_flow_kg_per_s
        nozzle_result = fixed_cd_nozzle(
            chamber_pressure_pa,
            chamber_temperature_k,
            self.atmosphere.pressure_pa,
            self.nozzle.throat_area_m2,
            self.nozzle.exit_area_m2,
            self.nozzle.discharge_coefficient,
            config.gamma,
            config.gas_constant_j_per_kg_k,
        )

        requested_outflow_kg = nozzle_result.mass_flow_kg_per_s * time_step_s
        actual_outflow_kg = min(requested_outflow_kg, 0.35 * state.total_mass_kg)
        exhaust_scale = (
            actual_outflow_kg / requested_outflow_kg if requested_outflow_kg > 0.0 else 0.0
        )
        exhaust_mass_flow_kg_per_s = actual_outflow_kg / time_step_s

        air_in_kg = inlet_air_mass_flow_kg_per_s * time_step_s
        fuel_mass_flow_kg_per_s = (
            inlet_air_mass_flow_kg_per_s
            * config.target_equivalence_ratio
            / self.fuel.stoichiometric_air_fuel_ratio
        )
        fuel_in_kg = fuel_mass_flow_kg_per_s * time_step_s

        original_mass_kg = max(state.total_mass_kg, 1e-12)
        fresh_air_out_kg = actual_outflow_kg * state.fresh_air_mass_kg / original_mass_kg
        unburned_fuel_out_kg = (
            actual_outflow_kg * state.unburned_fuel_mass_kg / original_mass_kg
        )

        energy_in_j = (
            air_in_kg
            * _INLET_AIR_GAMMA
            * _INLET_AIR_GAS_CONSTANT_J_PER_KG_K
            / (_INLET_AIR_GAMMA - 1.0)
            * self.inlet_total_temperature_k
            + fuel_in_kg
            * _FUEL_SENSIBLE_SPECIFIC_HEAT_J_PER_KG_K
            * self.atmosphere.temperature_k
        )
        energy_out_j = actual_outflow_kg * self.cp_j_per_kg_k * chamber_temperature_k

        state.total_mass_kg += air_in_kg + fuel_in_kg - actual_outflow_kg
        state.fresh_air_mass_kg += air_in_kg - fresh_air_out_kg
        state.unburned_fuel_mass_kg += fuel_in_kg - unburned_fuel_out_kg
        state.cumulative_fuel_injected_kg += fuel_in_kg
        state.cumulative_air_ingested_kg += air_in_kg
        state.cumulative_exhaust_discharged_kg += actual_outflow_kg
        state.cumulative_inlet_enthalpy_j += energy_in_j
        state.cumulative_exhaust_enthalpy_j += energy_out_j
        state.internal_energy_j += energy_in_j - energy_out_j

        heat_release_j = 0.0
        if state.pending_heat_release_j > 0.0 and state.burn_time_remaining_s > 0.0:
            burn_fraction = min(time_step_s / state.burn_time_remaining_s, 1.0)
            heat_release_j = state.pending_heat_release_j * burn_fraction
            state.pending_heat_release_j -= heat_release_j
            state.burn_time_remaining_s = max(
                state.burn_time_remaining_s - time_step_s, 0.0
            )
            state.internal_energy_j += heat_release_j
            state.cumulative_combustion_heat_added_j += heat_release_j

        wall_heat_transfer_j = (
            config.wall_heat_transfer_w_per_k
            * (chamber_temperature_k - config.wall_temperature_k)
            * time_step_s
        )
        if wall_heat_transfer_j > 0.0:
            wall_heat_transfer_j = min(wall_heat_transfer_j, 0.25 * state.internal_energy_j)
            state.internal_energy_j -= wall_heat_transfer_j
            state.cumulative_heat_rejected_j += wall_heat_transfer_j
            state.cumulative_wall_heat_rejected_j += wall_heat_transfer_j

        state.total_mass_kg = max(state.total_mass_kg, 1e-9)
        state.fresh_air_mass_kg = max(state.fresh_air_mass_kg, 0.0)
        state.unburned_fuel_mass_kg = max(state.unburned_fuel_mass_kg, 0.0)
        minimum_energy_j = state.total_mass_kg * self.cv_j_per_kg_k * 100.0
        if state.internal_energy_j < minimum_energy_j:
            numerical_energy_added_j = minimum_energy_j - state.internal_energy_j
            state.internal_energy_j = minimum_energy_j
            state.cumulative_numerical_energy_added_j += numerical_energy_added_j

        maximum_energy_j = (
            state.total_mass_kg * self.cv_j_per_kg_k * config.maximum_gas_temperature_k
        )
        if state.internal_energy_j > maximum_energy_j:
            rejected_j = state.internal_energy_j - maximum_energy_j
            state.internal_energy_j = maximum_energy_j
            state.cumulative_heat_rejected_j += rejected_j
            state.cumulative_temperature_limit_heat_rejected_j += rejected_j

        state.time_s += time_step_s
        gross_thrust_n = nozzle_result.gross_thrust_n * exhaust_scale
        inlet_momentum_drag_n = (
            inlet_air_mass_flow_kg_per_s * self.freestream_velocity_m_per_s
        )
        net_thrust_n = gross_thrust_n - inlet_momentum_drag_n
        if heat_release_j > 0.0:
            phase = "combustion"
        elif exhaust_mass_flow_kg_per_s > inlet_air_mass_flow_kg_per_s:
            phase = "blowdown"
        elif inlet_air_mass_flow_kg_per_s > 0.0:
            phase = "refill"
        else:
            phase = "dwell"

        return PulsejetSample(
            time_s=state.time_s,
            chamber_pressure_pa=self.pressure_pa,
            chamber_temperature_k=self.temperature_k,
            inlet_air_mass_flow_kg_per_s=inlet_air_mass_flow_kg_per_s,
            fuel_mass_flow_kg_per_s=fuel_mass_flow_kg_per_s,
            exhaust_mass_flow_kg_per_s=exhaust_mass_flow_kg_per_s,
            gross_thrust_n=gross_thrust_n,
            inlet_momentum_drag_n=inlet_momentum_drag_n,
            net_thrust_n=net_thrust_n,
            phase=phase,
            event=event,
            cycle_count=state.cycle_count,
            nozzle_warning=nozzle_result.warning,
        )

    def run(self, duration_s: float, time_step_s: float) -> list[PulsejetSample]:
        if duration_s <= 0.0:
            raise ValueError("duration must be positive")
        samples: list[PulsejetSample] = []
        while self.state.time_s < duration_s - 0.5 * time_step_s:
            samples.append(self.step(min(time_step_s, duration_s - self.state.time_s)))
        return samples

    def conservation_audit(self) -> PulsejetConservationAudit:
        state = self.state
        fuel_injected_after_start_kg = (
            state.cumulative_fuel_injected_kg - self.initial_fuel_mass_kg
        )
        expected_final_mass_kg = (
            self.initial_total_mass_kg
            + state.cumulative_air_ingested_kg
            + fuel_injected_after_start_kg
            - state.cumulative_exhaust_discharged_kg
        )
        mass_residual_kg = state.total_mass_kg - expected_final_mass_kg
        mass_scale_kg = max(
            abs(self.initial_total_mass_kg)
            + abs(state.cumulative_air_ingested_kg)
            + abs(fuel_injected_after_start_kg)
            + abs(state.cumulative_exhaust_discharged_kg),
            1e-12,
        )

        expected_final_energy_j = (
            self.initial_internal_energy_j
            + state.cumulative_inlet_enthalpy_j
            - state.cumulative_exhaust_enthalpy_j
            + state.cumulative_combustion_heat_added_j
            - state.cumulative_heat_rejected_j
            + state.cumulative_numerical_energy_added_j
        )
        energy_residual_j = state.internal_energy_j - expected_final_energy_j
        energy_scale_j = max(
            abs(self.initial_internal_energy_j)
            + abs(state.cumulative_inlet_enthalpy_j)
            + abs(state.cumulative_exhaust_enthalpy_j)
            + abs(state.cumulative_combustion_heat_added_j)
            + abs(state.cumulative_heat_rejected_j)
            + abs(state.cumulative_numerical_energy_added_j),
            1e-12,
        )
        return PulsejetConservationAudit(
            initial_total_mass_kg=self.initial_total_mass_kg,
            final_total_mass_kg=state.total_mass_kg,
            cumulative_air_ingested_kg=state.cumulative_air_ingested_kg,
            cumulative_fuel_injected_after_start_kg=fuel_injected_after_start_kg,
            cumulative_exhaust_discharged_kg=state.cumulative_exhaust_discharged_kg,
            mass_balance_residual_kg=mass_residual_kg,
            relative_mass_balance_residual=mass_residual_kg / mass_scale_kg,
            initial_internal_energy_j=self.initial_internal_energy_j,
            final_internal_energy_j=state.internal_energy_j,
            cumulative_inlet_enthalpy_j=state.cumulative_inlet_enthalpy_j,
            cumulative_exhaust_enthalpy_j=state.cumulative_exhaust_enthalpy_j,
            cumulative_combustion_heat_added_j=state.cumulative_combustion_heat_added_j,
            cumulative_heat_rejected_j=state.cumulative_heat_rejected_j,
            cumulative_wall_heat_rejected_j=(
                state.cumulative_wall_heat_rejected_j
            ),
            cumulative_temperature_limit_heat_rejected_j=(
                state.cumulative_temperature_limit_heat_rejected_j
            ),
            cumulative_numerical_energy_added_j=(
                state.cumulative_numerical_energy_added_j
            ),
            energy_balance_residual_j=energy_residual_j,
            relative_energy_balance_residual=energy_residual_j / energy_scale_j,
        )


def summarize_pulsejet(
    samples: list[PulsejetSample],
    *,
    minimum_time_s: float = 0.0,
) -> PulsejetSummary:
    """Average equal-step samples at or after an explicit startup cutoff."""

    if not samples:
        raise ValueError("at least one sample is required")
    if minimum_time_s < 0.0:
        raise ValueError("summary minimum time cannot be negative")
    window = [sample for sample in samples if sample.time_s >= minimum_time_s]
    if not window:
        raise ValueError("summary window begins after the final sample")
    duration_s = window[-1].time_s - window[0].time_s
    mean_net_thrust_n = fmean(sample.net_thrust_n for sample in window)
    mean_fuel_mass_flow_kg_per_s = fmean(
        sample.fuel_mass_flow_kg_per_s for sample in window
    )
    specific_impulse_s = (
        mean_net_thrust_n / (mean_fuel_mass_flow_kg_per_s * G0_M_PER_S2)
        if mean_fuel_mass_flow_kg_per_s > 1e-12
        else 0.0
    )
    return PulsejetSummary(
        window_start_s=window[0].time_s,
        window_end_s=window[-1].time_s,
        duration_s=max(duration_s, 0.0),
        completed_cycles=sum(sample.event == "ignition" for sample in window),
        mean_gross_thrust_n=fmean(sample.gross_thrust_n for sample in window),
        peak_gross_thrust_n=max(sample.gross_thrust_n for sample in window),
        mean_net_thrust_n=mean_net_thrust_n,
        peak_net_thrust_n=max(sample.net_thrust_n for sample in window),
        mean_fuel_mass_flow_kg_per_s=mean_fuel_mass_flow_kg_per_s,
        peak_chamber_pressure_pa=max(sample.chamber_pressure_pa for sample in window),
        peak_chamber_temperature_k=max(
            sample.chamber_temperature_k for sample in window
        ),
        final_chamber_pressure_pa=window[-1].chamber_pressure_pa,
        specific_impulse_s=specific_impulse_s,
    )


# Fixed-window averaging (summarize_pulsejet above, warmup_s/measurement_s)
# has a real artifact: as Mach steps and the real cycle period shifts, the
# same wall-clock window catches a different, non-integer number of cycles at
# each point, producing broad sawtooth jaggedness in any Mach sweep -- this is
# also structurally the same class of bug as the original Gate 2 fast/full-
# fidelity mismatch (propulsion_map.py's fast_pulsejet_time_step_s vs.
# trajectory.py's full pulsejet_table_time_step_s could silently disagree).
# Per cycle_based_averaging_fix.md (2026-08-08): detect real ignition-cycle
# boundaries (already computed internally -- PulsejetState.last_ignition_time_s
# / PulsejetSample.event == "ignition", no separate detection algorithm
# needed) and run a convergence-driven cycle count instead of a fixed time
# window. Tolerance and the cycle cap are fixed internal constants, not
# caller-adjustable parameters -- the actual point of this fix: two callers
# asking the same (case, altitude, mach) question can no longer disagree
# based on a timing knob they each picked independently.
#
# time_step_s (dt) is a separate concern -- numerical integration step size,
# not cycle counting -- and stays caller-adjustable for now, pending its own
# convergence study (cycle_based_averaging_fix.md step 7).
_CYCLE_AVERAGE_STARTUP_TRANSIENT_CYCLES = 2
"""Discard this many completed cycles at the start of the run before
accumulating the average -- the engine has not necessarily reached its
repeating limit cycle immediately after simulation start (t=0 initial
condition is a quiescent, pre-primed chamber, not a representative mid-run
state)."""
_CYCLE_AVERAGE_MAXIMUM_CYCLES = 100
"""Cap on post-transient cycles accumulated before giving up on convergence.
Raised from an initial 30 (2026-08-08, user decision) after direct
instrumentation (docs/design_convergence.md, "Sawtooth root cause") found the
pulsejet genuinely has a real, physical damped cycle-to-cycle oscillation --
consecutive cycles trading off strong/weak (e.g. one Mach point's raw per-
cycle net thrust ran 33 -> 338 -> 103 -> 257 -> 128 -> 226 N and was still
visibly alternating after 13 cycles) as each ignition's leftover chamber
state feeds the next cycle's refill. How fast this decays varies a lot by
Mach; 30 cycles was cutting many points off mid-oscillation. Explicit user
preference: report honest non-convergence over an averaged value from a
model that has not actually reached steady state, so this cap exists only to
bound worst-case cost, not as a target cycle count."""
_CYCLE_AVERAGE_RELATIVE_TOLERANCE = 1e-3
_CYCLE_AVERAGE_THRUST_ABSOLUTE_FLOOR_N = 0.05
"""Convergence is checked as max(relative tolerance, this absolute floor),
applied identically to both the running-mean-stability check and the raw
per-cycle swing check below, so the check stays meaningful even when thrust
is itself near zero (e.g. in the ramjet-transition trough) rather than
demanding an ever-tighter absolute change as the mean approaches zero."""
_CYCLE_AVERAGE_SWING_WINDOW_CYCLES = 4
"""Convergence requires the running mean to have stopped moving (see below)
*and* the raw (unaveraged) per-cycle net thrust to have actually settled --
max minus min over the trailing window of this many post-transient cycles,
within tolerance. A mean-only check is not enough: because each new cycle's
influence on a cumulative mean shrinks as roughly 1/N, the running mean can
stop moving by tolerance well before the underlying oscillation has actually
decayed, especially since it is a period-2-like alternation (strong/weak
cycles) -- a window of 4 spans two full periods of that alternation, so a
persistent (not yet decayed) alternation cannot pass this check by chance
landing on a same-phase pair."""
_CYCLE_AVERAGE_CONSECUTIVE_CONVERGED_UPDATES_REQUIRED = 3
"""Require this many consecutive post-transient cycle updates with *both*
checks above inside tolerance before declaring convergence -- guards against
one coincidentally small change being mistaken for having actually settled."""
_CYCLE_AVERAGE_MAXIMUM_SIMULATED_TIME_S = 400.0
"""Safety net only, not a normal stopping condition: a candidate that
genuinely never ignites (e.g. no fuel reaching the chamber) would otherwise
never complete a single cycle boundary and loop forever. Set well above any
legitimate _CYCLE_AVERAGE_MAXIMUM_CYCLES-cycle duration even at the slowest
observed real cycle rate (~1s/cycle at low Mach, docs/design_convergence.md)
-- same role as the old fixed-window path's _MAXIMUM_PULSEJET_SIMULATION_S,
scaled up for this per-cycle approach's larger legitimate runtime."""


@dataclass(frozen=True)
class PulsejetCycleAverage:
    """A convergence-driven cycle average -- see module comment above.

    Replaces a fixed-warmup_s/measurement_s-window PulsejetSummary for any
    caller that wants a robust steady-operating-point average rather than a
    raw sample dump.
    """

    mean_net_thrust_n: float
    mean_gross_thrust_n: float
    peak_net_thrust_n: float
    peak_gross_thrust_n: float
    mean_fuel_mass_flow_kg_per_s: float
    mean_inlet_air_mass_flow_kg_per_s: float
    specific_impulse_s: float
    peak_chamber_pressure_pa: float
    peak_chamber_temperature_k: float
    inlet_total_pressure_pa: float
    """The simulator's own (possibly Mach- and, for a side inlet, cycle-
    mass-flow-dependent) inlet total pressure -- same value
    `PulsejetSimulator.inlet_total_pressure_pa` exposes, read once at the end
    of the run for installed-recovery reporting."""
    averaged_cycles: int
    """Post-transient cycles actually folded into the running average
    (excludes the discarded startup cycles)."""
    converged: bool
    """False if _CYCLE_AVERAGE_MAXIMUM_CYCLES was hit without the running
    average settling inside tolerance -- the reported means are still the
    best available estimate at that point, not withheld, but callers should
    surface this (see propulsion_map.py's cycle_average_did_not_converge
    validity flag) rather than silently trust an unconverged value."""
    numerical_reference_only: bool = True


def run_pulsejet_to_converged_cycle_average(
    config: PulsejetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
    time_step_s: float,
    *,
    _maximum_cycles_override: int | None = None,
    _maximum_simulated_time_s_override: float | None = None,
) -> PulsejetCycleAverage:
    """Run to a converged per-cycle average, detecting real cycle boundaries.

    See module comment block above (cycle_based_averaging_fix.md) for the
    rationale. Each ignition event both closes out the cycle that was
    accumulating and opens the next one -- `PulsejetSample.event == "ignition"`
    is read directly off the simulator's own trigger, not re-detected.

    The two leading-underscore parameters started as `find_converged_time_
    step`'s own internal-only overrides (dt_convergence_solver_spec.md,
    2026-08-08: comparing converged-average thrust across different `dt`
    values only means something if each inner value is *actually* converged,
    not capped-and-flagged-as-possibly-wrong). `propulsion_map.py` now also
    passes them, resolved from `pulsejet_cycle_bounds_for_fidelity` -- found
    (2026-08-08) that the cycle/time cap needs the same tier-dependent
    treatment as dt itself: one global cap generous enough for
    PULSEJET_FIDELITY_FULL's verification needs made a single pathological
    search candidate (e.g. an oversized chamber_volume_m3) cost up to ~2.4
    hours across one evaluate_design call. These are still not raw caller-
    adjustable floats -- every real call site resolves them from one of the
    two named fidelity tiers, never an arbitrary value.
    """

    maximum_cycles = (
        _CYCLE_AVERAGE_MAXIMUM_CYCLES
        if _maximum_cycles_override is None
        else _maximum_cycles_override
    )
    maximum_simulated_time_s = (
        _CYCLE_AVERAGE_MAXIMUM_SIMULATED_TIME_S
        if _maximum_simulated_time_s_override is None
        else _maximum_simulated_time_s_override
    )
    simulator = PulsejetSimulator(config, selector, nozzle, fuel, altitude_m, mach)

    current_cycle: list[PulsejetSample] = []
    completed_cycles_total = 0
    per_cycle_net_thrust_n: list[float] = []
    per_cycle_gross_thrust_n: list[float] = []
    per_cycle_fuel_flow_kg_per_s: list[float] = []
    per_cycle_inlet_air_flow_kg_per_s: list[float] = []
    peak_chamber_pressure_pa = 0.0
    peak_chamber_temperature_k = 0.0
    peak_net_thrust_n = 0.0
    peak_gross_thrust_n = 0.0
    running_net_thrust_mean_n: float | None = None
    consecutive_converged_updates = 0
    converged = False

    while (
        completed_cycles_total - _CYCLE_AVERAGE_STARTUP_TRANSIENT_CYCLES
        < maximum_cycles
        and simulator.state.time_s < maximum_simulated_time_s
    ):
        sample = simulator.step(time_step_s)
        cycle_boundary = sample.event == "ignition" and current_cycle
        if cycle_boundary:
            completed_cycles_total += 1
            if completed_cycles_total > _CYCLE_AVERAGE_STARTUP_TRANSIENT_CYCLES:
                # Peaks tracked from post-transient cycles only, same
                # discard-the-startup-transient principle as the thrust/fuel
                # averages below -- not from every sample seen so far.
                peak_chamber_pressure_pa = max(
                    peak_chamber_pressure_pa,
                    max(s.chamber_pressure_pa for s in current_cycle),
                )
                peak_chamber_temperature_k = max(
                    peak_chamber_temperature_k,
                    max(s.chamber_temperature_k for s in current_cycle),
                )
                peak_net_thrust_n = max(
                    peak_net_thrust_n, max(s.net_thrust_n for s in current_cycle)
                )
                peak_gross_thrust_n = max(
                    peak_gross_thrust_n, max(s.gross_thrust_n for s in current_cycle)
                )
                per_cycle_net_thrust_n.append(fmean(s.net_thrust_n for s in current_cycle))
                per_cycle_gross_thrust_n.append(fmean(s.gross_thrust_n for s in current_cycle))
                per_cycle_fuel_flow_kg_per_s.append(
                    fmean(s.fuel_mass_flow_kg_per_s for s in current_cycle)
                )
                per_cycle_inlet_air_flow_kg_per_s.append(
                    fmean(s.inlet_air_mass_flow_kg_per_s for s in current_cycle)
                )
                new_running_mean_n = fmean(per_cycle_net_thrust_n)
                if running_net_thrust_mean_n is not None:
                    tolerance_n = max(
                        _CYCLE_AVERAGE_RELATIVE_TOLERANCE * abs(running_net_thrust_mean_n),
                        _CYCLE_AVERAGE_THRUST_ABSOLUTE_FLOOR_N,
                    )
                    mean_is_stable = (
                        abs(new_running_mean_n - running_net_thrust_mean_n) <= tolerance_n
                    )
                    # Raw-swing check: the running mean alone is not a
                    # reliable convergence signal for this engine (see
                    # _CYCLE_AVERAGE_SWING_WINDOW_CYCLES comment above) --
                    # also require the actual per-cycle values, not just
                    # their cumulative average, to have stopped moving.
                    raw_swing_is_settled = False
                    if len(per_cycle_net_thrust_n) >= _CYCLE_AVERAGE_SWING_WINDOW_CYCLES:
                        window = per_cycle_net_thrust_n[-_CYCLE_AVERAGE_SWING_WINDOW_CYCLES:]
                        raw_swing_is_settled = (max(window) - min(window)) <= tolerance_n
                    if mean_is_stable and raw_swing_is_settled:
                        consecutive_converged_updates += 1
                    else:
                        consecutive_converged_updates = 0
                running_net_thrust_mean_n = new_running_mean_n
                if (
                    consecutive_converged_updates
                    >= _CYCLE_AVERAGE_CONSECUTIVE_CONVERGED_UPDATES_REQUIRED
                ):
                    converged = True
            current_cycle = [sample]
        else:
            current_cycle.append(sample)
        if converged:
            break

    averaged_cycles = len(per_cycle_net_thrust_n)
    if averaged_cycles == 0:
        # Never completed a single post-transient cycle within the cap --
        # report an honest zero, flagged unconverged, rather than raising
        # (a candidate that never ignites is a real, expected case elsewhere
        # in this codebase's search space, not a programming error here).
        return PulsejetCycleAverage(
            mean_net_thrust_n=0.0,
            mean_gross_thrust_n=0.0,
            peak_net_thrust_n=peak_net_thrust_n,
            peak_gross_thrust_n=peak_gross_thrust_n,
            mean_fuel_mass_flow_kg_per_s=0.0,
            mean_inlet_air_mass_flow_kg_per_s=0.0,
            specific_impulse_s=0.0,
            peak_chamber_pressure_pa=peak_chamber_pressure_pa,
            peak_chamber_temperature_k=peak_chamber_temperature_k,
            inlet_total_pressure_pa=simulator.inlet_total_pressure_pa,
            averaged_cycles=0,
            converged=False,
        )

    mean_net_thrust_n = fmean(per_cycle_net_thrust_n)
    mean_gross_thrust_n = fmean(per_cycle_gross_thrust_n)
    mean_fuel_mass_flow_kg_per_s = fmean(per_cycle_fuel_flow_kg_per_s)
    mean_inlet_air_mass_flow_kg_per_s = fmean(per_cycle_inlet_air_flow_kg_per_s)
    specific_impulse_s = (
        mean_net_thrust_n / (mean_fuel_mass_flow_kg_per_s * G0_M_PER_S2)
        if mean_fuel_mass_flow_kg_per_s > 1e-12
        else 0.0
    )
    return PulsejetCycleAverage(
        mean_net_thrust_n=mean_net_thrust_n,
        mean_gross_thrust_n=mean_gross_thrust_n,
        peak_net_thrust_n=peak_net_thrust_n,
        peak_gross_thrust_n=peak_gross_thrust_n,
        mean_fuel_mass_flow_kg_per_s=mean_fuel_mass_flow_kg_per_s,
        mean_inlet_air_mass_flow_kg_per_s=mean_inlet_air_mass_flow_kg_per_s,
        specific_impulse_s=specific_impulse_s,
        inlet_total_pressure_pa=simulator.inlet_total_pressure_pa,
        peak_chamber_pressure_pa=peak_chamber_pressure_pa,
        peak_chamber_temperature_k=peak_chamber_temperature_k,
        averaged_cycles=averaged_cycles,
        converged=converged,
    )


# dt_convergence_solver_spec.md (2026-08-08): the cycle-based averaging fix
# above retired warmup_s/measurement_s as a caller-adjustable knob two
# callers could disagree on. time_step_s (dt) is the other half of the
# original Gate 2 fidelity bug -- find_converged_time_step below determines
# the finest dt genuinely needed (via a real nested-convergence solve, not a
# one-off manual check) so it too can become a fixed internal constant.
_DT_SOLVER_INNER_CYCLE_CAP = 50
"""Safety bound for this solver's own inner evaluations -- deliberately
*smaller* than the production _CYCLE_AVERAGE_MAXIMUM_CYCLES (100), reversing
this module's first attempt (which used 3x production, 300). Direct
measurement (docs/design_convergence.md, "dt-convergence solver cost
finding") found the strict swing-based convergence check is itself fragile
to dt: one halving away from the production default (5e-5s -> 2.5e-5s) took
a fast-converging point (14 cycles at 5e-5s) to 300 cycles and 265s without
ever settling, even though the net thrust barely moved (188.65N vs 188.42N);
even a 75-cycle cap still cost 73s on that same halving without converging.
Chasing genuine convergence at every dt level is not practical at this
system's actual numerical behavior -- per user decision (optimize for speed,
fail fast rather than grind), reporting "this dt did not settle within a
reasonable cycle count" is more useful than spending a minute or more per
halving on a check that may not settle at some dt values at all.
inner_converged=False on a halving record is itself the real, honestly-
reported finding here, not a problem to engineer around by raising this cap
further."""
_DT_SOLVER_INNER_SIMULATED_TIME_CAP_S = 150.0
"""Companion safety bound to _DT_SOLVER_INNER_CYCLE_CAP, same role as
_CYCLE_AVERAGE_MAXIMUM_SIMULATED_TIME_S plays for the production cap --
lowered alongside the cycle cap for the same fail-fast reasoning."""
_DT_SOLVER_RELATIVE_TOLERANCE = 1e-3
_DT_SOLVER_ABSOLUTE_TOLERANCE_N = 0.05
"""Same numeric values as the cycle-average's own convergence tolerance
(_CYCLE_AVERAGE_RELATIVE_TOLERANCE / _CYCLE_AVERAGE_THRUST_ABSOLUTE_FLOOR_N)
-- deliberate, not copied by accident: there is no reason to resolve dt
sensitivity to a finer precision than the reported thrust value is already
only trusted to. Kept as separate named constants (not literally reused)
in case that reasoning needs revisiting independently of the cycle-average
tolerance later."""
_DT_SOLVER_CONSECUTIVE_STABLE_HALVINGS_REQUIRED = 3
_DT_SOLVER_MAXIMUM_HALVINGS = 5
"""dt shrinks by 2**5 = 32x from the starting value if convergence is never
reached. Reduced from an initial 8 (256x) for the same cost reasoning as
_DT_SOLVER_INNER_CYCLE_CAP: measurement showed the inner convergence check
already destabilizes on the *first* halving away from the production dt, so
pushing depth further mainly adds cost (each level's inner evaluation is both
more expensive per cycle at finer dt and more likely to hit its own cap)
without changing what's being learned. Hitting this cap is still a real
finding (this regime is numerically pathological at any practical dt), not a
sign the cap itself was set too low."""


@dataclass(frozen=True)
class DtConvergenceTestPoint:
    """One (geometry, altitude, Mach) point to probe for dt sensitivity."""

    label: str
    config: PulsejetConfig
    selector: SelectorConfig
    nozzle: NozzleConfig
    fuel: Fuel
    altitude_m: float
    mach: float


@dataclass(frozen=True)
class DtConvergenceHalvingRecord:
    time_step_s: float
    mean_net_thrust_n: float
    inner_cycles: int
    inner_converged: bool
    """False if even this solver's own extended inner cap
    (_DT_SOLVER_INNER_CYCLE_CAP/_DT_SOLVER_INNER_SIMULATED_TIME_CAP_S) was
    hit -- this halving's value is not trustworthy and cannot count toward
    declaring dt convergence (see find_converged_time_step)."""


@dataclass(frozen=True)
class DtConvergencePointResult:
    label: str
    mach: float
    halvings: tuple[DtConvergenceHalvingRecord, ...]
    converged_time_step_s: float | None
    """The coarsest dt at which the value had already stabilized -- i.e. the
    dt _DT_SOLVER_CONSECUTIVE_STABLE_HALVINGS_REQUIRED steps back from
    wherever the stable streak was confirmed, not the finest dt tested (using
    a finer dt than necessary only costs runtime for no accuracy benefit).
    None if the halving cap was hit, or every halving's inner loop failed to
    converge, without ever confirming a stable streak."""
    halving_cap_hit: bool
    inner_loop_needed_extended_cycles: bool
    """True if any halving at this point needed more post-transient cycles
    than the production cap (_CYCLE_AVERAGE_MAXIMUM_CYCLES) to converge --
    flags a numerically/physically fragile spot even though this solver's own
    inner loop does not cap there."""


@dataclass(frozen=True)
class DtConvergenceReport:
    point_results: tuple[DtConvergencePointResult, ...]
    recommended_time_step_s: float | None
    """The finest (smallest) converged_time_step_s across all points that did
    converge -- the conservative choice, since a dt fine enough for the most
    dt-sensitive point is fine enough for every other point too. None if no
    tested point converged at all (a real finding to report, not silently
    fall back to some default)."""
    unconverged_point_labels: tuple[str, ...]
    wall_clock_cost_s: float


def find_converged_time_step(
    test_points: Sequence[DtConvergenceTestPoint],
    *,
    starting_time_step_s: float = 5e-5,
    on_halving: Callable[[str, DtConvergenceHalvingRecord], None] | None = None,
) -> DtConvergenceReport:
    """Determine the finest dt genuinely needed across representative points.

    See dt_convergence_solver_spec.md (2026-08-08) for the full rationale.
    Two nested convergence loops: for each test point, repeatedly halve dt
    from ``starting_time_step_s``, evaluating a genuinely-converged (not
    production-capped) cycle average at each dt via
    ``run_pulsejet_to_converged_cycle_average``'s solver-only override
    parameters, until the net thrust result stops changing by more than
    tolerance for ``_DT_SOLVER_CONSECUTIVE_STABLE_HALVINGS_REQUIRED``
    consecutive halvings (or the halving cap is hit first). Callers should
    pass a representative test set spanning Mach regimes and geometry
    candidates -- this function does not choose that set itself, since what
    counts as "representative" is a judgment call belonging to the caller
    (see e.g. scripts/dt_convergence_study.py for one such set), and a single
    hardcoded set here would go stale as soon as the search moves to a
    different design region.

    ``on_halving``, if given, is called ``(point.label, record)`` after every
    halving completes -- this solver can run for minutes per halving (see
    _DT_SOLVER_INNER_CYCLE_CAP's docstring), so a caller running it
    unattended likely wants real-time progress rather than only a report
    after everything finishes.
    """

    start_time = time.monotonic()
    point_results: list[DtConvergencePointResult] = []
    for point in test_points:
        halvings: list[DtConvergenceHalvingRecord] = []
        time_step_s = starting_time_step_s
        consecutive_stable = 0
        converged_time_step_s: float | None = None
        halving_cap_hit = False
        inner_loop_needed_extended_cycles = False
        for _ in range(_DT_SOLVER_MAXIMUM_HALVINGS + 1):
            result = run_pulsejet_to_converged_cycle_average(
                point.config,
                point.selector,
                point.nozzle,
                point.fuel,
                point.altitude_m,
                point.mach,
                time_step_s,
                _maximum_cycles_override=_DT_SOLVER_INNER_CYCLE_CAP,
                _maximum_simulated_time_s_override=_DT_SOLVER_INNER_SIMULATED_TIME_CAP_S,
            )
            if result.averaged_cycles > _CYCLE_AVERAGE_MAXIMUM_CYCLES:
                inner_loop_needed_extended_cycles = True
            record = DtConvergenceHalvingRecord(
                time_step_s=time_step_s,
                mean_net_thrust_n=result.mean_net_thrust_n,
                inner_cycles=result.averaged_cycles,
                inner_converged=result.converged,
            )
            halvings.append(record)
            if on_halving is not None:
                on_halving(point.label, record)
            if not result.converged:
                # This dt's own value is not trustworthy -- cannot count
                # toward the stable streak (dt_convergence_solver_spec.md:
                # "don't force a cap -- report that back explicitly ...
                # rather than silently accepting a possibly-unconverged
                # value into the dt comparison").
                consecutive_stable = 0
            elif len(halvings) >= 2 and halvings[-2].inner_converged:
                previous = halvings[-2].mean_net_thrust_n
                tolerance_n = max(
                    _DT_SOLVER_RELATIVE_TOLERANCE * abs(previous),
                    _DT_SOLVER_ABSOLUTE_TOLERANCE_N,
                )
                if abs(result.mean_net_thrust_n - previous) <= tolerance_n:
                    consecutive_stable += 1
                else:
                    consecutive_stable = 0
            if consecutive_stable >= _DT_SOLVER_CONSECUTIVE_STABLE_HALVINGS_REQUIRED:
                converged_time_step_s = halvings[
                    -_DT_SOLVER_CONSECUTIVE_STABLE_HALVINGS_REQUIRED
                ].time_step_s
                break
            time_step_s /= 2.0
        else:
            halving_cap_hit = True
        point_results.append(
            DtConvergencePointResult(
                label=point.label,
                mach=point.mach,
                halvings=tuple(halvings),
                converged_time_step_s=converged_time_step_s,
                halving_cap_hit=halving_cap_hit,
                inner_loop_needed_extended_cycles=inner_loop_needed_extended_cycles,
            )
        )

    converged_time_steps_s = [
        result.converged_time_step_s
        for result in point_results
        if result.converged_time_step_s is not None
    ]
    unconverged_point_labels = tuple(
        result.label for result in point_results if result.converged_time_step_s is None
    )
    return DtConvergenceReport(
        point_results=tuple(point_results),
        recommended_time_step_s=(
            min(converged_time_steps_s) if converged_time_steps_s else None
        ),
        unconverged_point_labels=unconverged_point_labels,
        wall_clock_cost_s=time.monotonic() - start_time,
    )
