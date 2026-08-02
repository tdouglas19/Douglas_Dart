"""Hybrid-event, zero-dimensional pulsejet chamber model."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import fmean

from .atmosphere import Atmosphere, standard_atmosphere
from .compressible import (
    compressible_orifice_mass_flow,
    fixed_cd_nozzle,
    stagnation_pressure,
    stagnation_temperature,
)
from .config import Fuel, NozzleConfig, PulsejetConfig, SelectorConfig


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
    cumulative_heat_rejected_j: float


@dataclass(frozen=True)
class PulsejetSample:
    time_s: float
    chamber_pressure_pa: float
    chamber_temperature_k: float
    inlet_air_mass_flow_kg_per_s: float
    fuel_mass_flow_kg_per_s: float
    exhaust_mass_flow_kg_per_s: float
    gross_thrust_n: float
    phase: str
    event: str | None
    cycle_count: int
    nozzle_warning: str | None


@dataclass(frozen=True)
class PulsejetSummary:
    duration_s: float
    completed_cycles: int
    mean_gross_thrust_n: float
    peak_gross_thrust_n: float
    mean_fuel_mass_flow_kg_per_s: float
    peak_chamber_pressure_pa: float
    peak_chamber_temperature_k: float
    final_chamber_pressure_pa: float
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
        self.inlet_total_temperature_k = stagnation_temperature(
            self.atmosphere.temperature_k, mach
        )
        ideal_total_pressure_pa = stagnation_pressure(self.atmosphere.pressure_pa, mach)
        # Recovery is applied to ram rise above ambient, avoiding an artificial loss at M=0.
        self.inlet_total_pressure_pa = self.atmosphere.pressure_pa + (
            ideal_total_pressure_pa - self.atmosphere.pressure_pa
        ) * selector.total_pressure_recovery
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
        self.state = PulsejetState(
            time_s=0.0,
            total_mass_kg=total_mass_kg,
            fresh_air_mass_kg=self.initial_air_reference_mass_kg,
            unburned_fuel_mass_kg=initial_fuel_mass_kg,
            internal_energy_j=total_mass_kg
            * self.cv_j_per_kg_k
            * self.atmosphere.temperature_k,
            pending_heat_release_j=0.0,
            burn_time_remaining_s=0.0,
            last_ignition_time_s=-config.minimum_cycle_period_s,
            cycle_count=0,
            cumulative_fuel_injected_kg=initial_fuel_mass_kg,
            cumulative_fuel_burned_kg=0.0,
            cumulative_heat_rejected_j=0.0,
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
            1.4,
            287.05287,
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
            air_in_kg * 1.4 * 287.05287 / 0.4 * self.inlet_total_temperature_k
            + fuel_in_kg * 2_000.0 * self.atmosphere.temperature_k
        )
        energy_out_j = actual_outflow_kg * self.cp_j_per_kg_k * chamber_temperature_k

        state.total_mass_kg += air_in_kg + fuel_in_kg - actual_outflow_kg
        state.fresh_air_mass_kg += air_in_kg - fresh_air_out_kg
        state.unburned_fuel_mass_kg += fuel_in_kg - unburned_fuel_out_kg
        state.cumulative_fuel_injected_kg += fuel_in_kg
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

        wall_heat_transfer_j = (
            config.wall_heat_transfer_w_per_k
            * (chamber_temperature_k - config.wall_temperature_k)
            * time_step_s
        )
        if wall_heat_transfer_j > 0.0:
            wall_heat_transfer_j = min(wall_heat_transfer_j, 0.25 * state.internal_energy_j)
            state.internal_energy_j -= wall_heat_transfer_j
            state.cumulative_heat_rejected_j += wall_heat_transfer_j

        state.total_mass_kg = max(state.total_mass_kg, 1e-9)
        state.fresh_air_mass_kg = max(state.fresh_air_mass_kg, 0.0)
        state.unburned_fuel_mass_kg = max(state.unburned_fuel_mass_kg, 0.0)
        state.internal_energy_j = max(
            state.internal_energy_j,
            state.total_mass_kg * self.cv_j_per_kg_k * 100.0,
        )

        maximum_energy_j = (
            state.total_mass_kg * self.cv_j_per_kg_k * config.maximum_gas_temperature_k
        )
        if state.internal_energy_j > maximum_energy_j:
            rejected_j = state.internal_energy_j - maximum_energy_j
            state.internal_energy_j = maximum_energy_j
            state.cumulative_heat_rejected_j += rejected_j

        state.time_s += time_step_s
        gross_thrust_n = nozzle_result.gross_thrust_n * exhaust_scale
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


def summarize_pulsejet(samples: list[PulsejetSample]) -> PulsejetSummary:
    if not samples:
        raise ValueError("at least one sample is required")
    duration_s = samples[-1].time_s - samples[0].time_s
    return PulsejetSummary(
        duration_s=max(duration_s, 0.0),
        completed_cycles=samples[-1].cycle_count,
        mean_gross_thrust_n=fmean(sample.gross_thrust_n for sample in samples),
        peak_gross_thrust_n=max(sample.gross_thrust_n for sample in samples),
        mean_fuel_mass_flow_kg_per_s=fmean(
            sample.fuel_mass_flow_kg_per_s for sample in samples
        ),
        peak_chamber_pressure_pa=max(sample.chamber_pressure_pa for sample in samples),
        peak_chamber_temperature_k=max(sample.chamber_temperature_k for sample in samples),
        final_chamber_pressure_pa=samples[-1].chamber_pressure_pa,
    )
