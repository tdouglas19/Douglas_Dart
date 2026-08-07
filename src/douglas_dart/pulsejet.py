"""Hybrid-event, zero-dimensional pulsejet chamber model."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import fmean

from .atmosphere import G0_M_PER_S2, Atmosphere, standard_atmosphere
from .compressible import (
    compressible_orifice_mass_flow,
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
# quantitative side-inlet correlation, only that qualitative statement -- this
# fraction (how much of the Mach-dependent ram-pressure rise a side inlet is
# credited with) is therefore an UNSOURCED engineering placeholder, not a value
# from the reference or literature. Tune or replace once real data exists.
SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION = 0.15


def pulsejet_cycle_mode(mach: float, inlet_type: str) -> str:
    """Return which idealized pulsejet cycle regime applies (sec. 2.2, 2.4).

    A straight inlet's charge genuinely pre-compresses as flight speed increases:
    Lenoir (no pre-compression) at zero/near-zero Mach, shifting toward Humphrey
    (real pre-compression before constant-volume heat addition) as Mach rises. A
    side inlet stays close to Lenoir at any speed. This is diagnostic/reporting
    only -- ``PulsejetSimulator`` itself is a numerical mass/energy-conservation
    model, not a closed-form cycle calculation, so it does not branch on this
    label; instead it reproduces the same physical trend through the Mach-
    dependent inlet stagnation pressure computed in ``__init__`` (damped for side
    inlets via ``SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION``).
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
        # rise); a side inlet shows little pre-compression at any speed, so only a
        # small, unsourced fraction of that rise is credited (see
        # SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION).
        ideal_total_pressure_pa = stagnation_pressure(self.atmosphere.pressure_pa, mach)
        if selector.inlet_type == "side":
            ram_pressure_rise_pa = ideal_total_pressure_pa - self.atmosphere.pressure_pa
            ideal_total_pressure_pa = (
                self.atmosphere.pressure_pa
                + SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION * ram_pressure_rise_pa
            )
        self.cycle_mode = pulsejet_cycle_mode(mach, selector.inlet_type)
        self.inlet_total_pressure_pa = (
            ideal_total_pressure_pa * selector.pulsejet_total_pressure_recovery
        )
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

    def _ignite_if_ready(self) -> str | None:
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
        state = self.state
        config = self.config
        ready = (
            state.pending_heat_release_j <= 1e-9
            and state.time_s - state.last_ignition_time_s >= config.minimum_cycle_period_s
            and self.pressure_pa
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
        event = self._ignite_if_ready()

        chamber_pressure_pa = self.pressure_pa
        chamber_temperature_k = self.temperature_k
        inlet_air_mass_flow_kg_per_s, _ = compressible_orifice_mass_flow(
            self.inlet_total_pressure_pa,
            self.inlet_total_temperature_k,
            chamber_pressure_pa,
            self.selector.available_area_m2,
            self.selector.discharge_coefficient,
            _INLET_AIR_GAMMA,
            _INLET_AIR_GAS_CONSTANT_J_PER_KG_K,
        )
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
