"""Closed-form pulsejet thrust: Helmholtz cycle frequency, constant-volume
combustion for peak chamber conditions, choked-throat peak thrust, then a
deliberately crude average-thrust approximation. No ODE, no valve dynamics.

Per the user's own framing:
- Side-mounted reed valves are assumed unconstrained (never the flow
  bottleneck) and fill the chamber at ambient *static* conditions -- no ram
  credit for forward flight speed. This means, in this model, pulsejet
  thrust does not depend on vehicle Mach at all, only on altitude (ambient
  density/pressure/temperature). That is a direct, intentional consequence
  of the "static pressure" fill assumption, not a bug -- it is exactly why
  the ramjet eventually overtakes it as Mach increases in the thrust-vs-Mach
  plot.
- The throat diameter/length double as the Helmholtz neck; chamber volume
  comes from vehicle diameter (shared with the ramjet inlet/frontal area)
  times chamber length.
- No inlet momentum-drag term is subtracted: side-inlet flow enters roughly
  perpendicular to the vehicle axis, so its axial momentum contribution is
  neglected -- net thrust is taken equal to gross tailpipe thrust.
"""

from __future__ import annotations

from math import pi, sqrt
from typing import NamedTuple

from douglas_dart.atmosphere import Atmosphere, standard_atmosphere

from .constants import (
    CP_COMB_J_PER_KG_K,
    CV_COMB_J_PER_KG_K,
    G0_M_PER_S2,
    GAMMA_COMB,
    COMBUSTION_EFFICIENCY,
    MAX_CHAMBER_TEMPERATURE_K,
    PULSEJET_PEAK_PRESSURE_RATIO,
    R_COMB_J_PER_KG_K,
    Fuel,
)

# Average thrust over a cycle is approximated as peak_thrust / 3: the pulse
# is roughly "on" (near peak blowdown) for about a third of its period, with
# the rest spent on the comparatively low-thrust intake/scavenge portion of
# the cycle. This is the crude, deliberately-simple approximation the user
# asked for -- not a fitted or sourced duty-cycle number.
AVERAGE_THRUST_DUTY_CYCLE_FACTOR = 1.0 / 3.0


class PulsejetResult(NamedTuple):
    average_thrust_n: float
    peak_thrust_n: float
    frequency_hz: float
    fuel_mass_flow_kg_per_s: float
    air_mass_flow_kg_per_s: float
    chamber_volume_m3: float
    peak_pressure_pa: float
    peak_temperature_k: float
    specific_impulse_s: float
    choked: bool


def _choked_mass_flow_coefficient(gamma: float, gas_constant_j_per_kg_k: float) -> float:
    return sqrt(gamma / gas_constant_j_per_kg_k) * (2.0 / (gamma + 1.0)) ** (
        (gamma + 1.0) / (2.0 * (gamma - 1.0))
    )


def _critical_pressure_ratio(gamma: float) -> float:
    return (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))


# GAMMA_COMB and R_COMB_J_PER_KG_K are fixed constants, never swept -- these
# were previously recomputed (sqrt + two powers each) on every single call
# to pulsejet_thrust, i.e. every powered timestep of every flight, for a
# result that's always identical. Precomputing once at import time is a
# pure speed win with no behavior change (confirmed by direct before/after
# trajectory comparison, not assumed).
_CHOKED_MASS_FLOW_COEFFICIENT = _choked_mass_flow_coefficient(GAMMA_COMB, R_COMB_J_PER_KG_K)
_CRITICAL_PRESSURE_RATIO = _critical_pressure_ratio(GAMMA_COMB)


def helmholtz_frequency_hz(
    chamber_volume_m3: float,
    neck_area_m2: float,
    neck_length_m: float,
    speed_of_sound_m_per_s: float,
) -> float:
    """f = c/(2*pi) * sqrt(A_neck / (V_chamber * L_neck)). No end correction."""

    return (speed_of_sound_m_per_s / (2.0 * pi)) * sqrt(
        neck_area_m2 / (chamber_volume_m3 * neck_length_m)
    )


def pulsejet_thrust(
    diameter_m: float,
    chamber_length_m: float,
    throat_diameter_m: float,
    throat_length_m: float,
    mach: float,  # noqa: ARG001 -- accepted for API symmetry with ramjet_thrust; unused (see module docstring)
    altitude_m: float,
    fuel: Fuel,
    atmosphere: Atmosphere | None = None,
) -> PulsejetResult:
    # `atmosphere` lets a caller that already has the Atmosphere for this
    # altitude (flight_sim.py's hot loop, which computes it once per step
    # and would otherwise have this function -- and ramjet_thrust --
    # recompute the identical value independently) skip the recomputation.
    # Standalone callers (e.g. run_demo.py's Mach sweep) just pass
    # altitude_m as before and this is computed here exactly as always.
    if atmosphere is None:
        atmosphere = standard_atmosphere(altitude_m)

    chamber_volume_m3 = (pi * diameter_m**2 / 4.0) * chamber_length_m
    neck_area_m2 = pi * throat_diameter_m**2 / 4.0

    # One fresh, ambient-static charge fills the whole chamber each cycle.
    air_mass_per_cycle_kg = atmosphere.density_kg_per_m3 * chamber_volume_m3
    fuel_mass_per_cycle_kg = air_mass_per_cycle_kg / fuel.stoichiometric_air_fuel_ratio
    total_mass_per_cycle_kg = air_mass_per_cycle_kg + fuel_mass_per_cycle_kg

    # Constant-volume (Otto-cycle-like) combustion: all heat raises internal
    # energy, so the energy balance uses cv, not cp. This still sets the hot
    # combustion-product temperature used below for exit velocity and
    # Helmholtz sound speed -- it is only *peak pressure* that departs from
    # the idealized constant-volume value (see PULSEJET_PEAK_PRESSURE_RATIO).
    heat_release_j = fuel_mass_per_cycle_kg * fuel.lower_heating_value_j_per_kg * COMBUSTION_EFFICIENCY
    peak_temperature_k = min(
        atmosphere.temperature_k + heat_release_j / (total_mass_per_cycle_kg * CV_COMB_J_PER_KG_K),
        MAX_CHAMBER_TEMPERATURE_K,
    )
    # Anchored to real hardware (see PULSEJET_PEAK_PRESSURE_RATIO), not the
    # ideal-gas P/T ratio a true constant-volume hold would imply.
    peak_pressure_pa = atmosphere.pressure_pa * PULSEJET_PEAK_PRESSURE_RATIO

    # Helmholtz frequency, using the hot post-combustion gas's speed of
    # sound (this is the gas actually oscillating through the neck).
    speed_of_sound_hot_m_per_s = sqrt(GAMMA_COMB * R_COMB_J_PER_KG_K * peak_temperature_k)
    frequency_hz = helmholtz_frequency_hz(
        chamber_volume_m3, neck_area_m2, throat_length_m, speed_of_sound_hot_m_per_s
    )

    choked = peak_pressure_pa * _CRITICAL_PRESSURE_RATIO >= atmosphere.pressure_pa
    if not choked:
        return PulsejetResult(
            0.0, 0.0, frequency_hz, 0.0, 0.0, chamber_volume_m3, peak_pressure_pa, peak_temperature_k, 0.0, False,
        )

    peak_mass_flow_kg_per_s = _CHOKED_MASS_FLOW_COEFFICIENT * neck_area_m2 * peak_pressure_pa / sqrt(peak_temperature_k)

    pressure_ratio = atmosphere.pressure_pa / peak_pressure_pa
    exit_velocity_m_per_s = sqrt(
        max(
            2.0
            * CP_COMB_J_PER_KG_K
            * peak_temperature_k
            * (1.0 - pressure_ratio ** ((GAMMA_COMB - 1.0) / GAMMA_COMB)),
            0.0,
        )
    )
    peak_thrust_n = peak_mass_flow_kg_per_s * exit_velocity_m_per_s
    average_thrust_n = peak_thrust_n * AVERAGE_THRUST_DUTY_CYCLE_FACTOR

    fuel_mass_flow_kg_per_s = fuel_mass_per_cycle_kg * frequency_hz
    air_mass_flow_kg_per_s = air_mass_per_cycle_kg * frequency_hz
    # Air-breathing convention: Isp = (average) thrust / (fuel weight flow).
    specific_impulse_s = average_thrust_n / (fuel_mass_flow_kg_per_s * G0_M_PER_S2)

    return PulsejetResult(
        average_thrust_n=average_thrust_n,
        peak_thrust_n=peak_thrust_n,
        frequency_hz=frequency_hz,
        fuel_mass_flow_kg_per_s=fuel_mass_flow_kg_per_s,
        air_mass_flow_kg_per_s=air_mass_flow_kg_per_s,
        chamber_volume_m3=chamber_volume_m3,
        peak_pressure_pa=peak_pressure_pa,
        peak_temperature_k=peak_temperature_k,
        specific_impulse_s=specific_impulse_s,
        choked=True,
    )
