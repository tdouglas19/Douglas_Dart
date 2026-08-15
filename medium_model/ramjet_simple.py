"""Closed-form ramjet thrust: capture -> stoichiometric heat release -> throat
mass flow (choked or not) -> perfectly-expanded exit velocity -> net thrust.

Every step is a single algebraic expression (no iteration, no ODE):

1. Inlet captures up to the full frontal area: mdot_captured = rho*V*A_inlet.
2. Ram compression sets inlet stagnation temperature and pressure (ideal,
   isentropic, no loss) -- P0_ram is a hard physical ceiling on chamber
   pressure, since nothing downstream (constant-area heat addition) can
   raise total pressure further. Chamber pressure is *always* this ceiling
   (see step 4) -- there is no separate lower-pressure regime.
3. Stoichiometric fuel is added; a constant-cp energy balance gives chamber
   stagnation temperature. For fixed stoichiometric fueling this comes out
   independent of the mass flow rate itself (fuel and air scale together),
   so it can be evaluated before mass flow is resolved.
4. The mass flow that actually passes through the fixed throat area at
   (P0_ram, T0_chamber) is found with the standard compressible-orifice
   relation (douglas_dart.compressible.compressible_orifice_mass_flow),
   which itself branches choked/unchoked on the pressure ratio -- there is
   no separate low-Mach cutoff here. A ramjet can therefore produce a
   little thrust well below where the throat actually chokes (typically
   ~Mach 1 for a fixed throat at sea level): ram compression raises chamber
   pressure above ambient for any Mach > 0, so there is always *some*
   pressure differential to drive unchoked flow, just a small one at low
   Mach. Actual mass flow is capped at min(captured, throat capacity) --
   an undersized throat spills the excess rather than implying an
   unphysical chamber pressure. (Spillage drag on the un-ingested air is
   not modeled -- a known simplification.)
5. A perfectly-expanded nozzle (Pe = Pambient) gives exit velocity directly
   from isentropic expansion -- this formula doesn't care whether the throat
   itself is choked, it is the general expansion from (P0,T0) to Pe; thrust
   is the standard mdot*Ve - mdot_air*V, with zero pressure term since
   Pe = Pambient by assumption.
"""

from __future__ import annotations

from math import pi, sqrt
from typing import NamedTuple

from douglas_dart.atmosphere import Atmosphere, standard_atmosphere
from douglas_dart.compressible import compressible_orifice_mass_flow

from .constants import (
    CP_COMB_J_PER_KG_K,
    G0_M_PER_S2,
    GAMMA_AIR,
    GAMMA_COMB,
    COMBUSTION_EFFICIENCY,
    MAX_CHAMBER_TEMPERATURE_K,
    R_COMB_J_PER_KG_K,
    RAMJET_LIGHTOFF_RAMP_MACH,
    RAMJET_MIN_LIGHTOFF_MACH,
    Fuel,
)


class RamjetResult(NamedTuple):
    net_thrust_n: float
    gross_thrust_n: float
    air_mass_flow_kg_per_s: float
    fuel_mass_flow_kg_per_s: float
    captured_air_mass_flow_kg_per_s: float
    spilled: bool
    chamber_total_pressure_pa: float
    chamber_total_temperature_k: float
    exit_velocity_m_per_s: float
    specific_impulse_s: float
    choked: bool
    lit: bool = True
    """False below RAMJET_MIN_LIGHTOFF_MACH (constants.py): flameholding is
    not viable, so thrust and fuel flow are gated to zero (with a narrow
    linear ramp just above the minimum to keep the closed form continuous).
    Placeholder constant from douglas_dart's RamjetConfig until the planned
    first-principles ramjet-fp model derives the real minimum viable speed."""


def ramjet_thrust(
    diameter_m: float,
    throat_diameter_m: float,
    mach: float,
    altitude_m: float,
    fuel: Fuel,
    atmosphere: Atmosphere | None = None,
    lightoff_mach: float | None = None,
    allow_light: bool = True,
) -> RamjetResult:
    # lightoff_mach / allow_light (2026-08-13, V4): the lightoff decision is
    # no longer a bare module constant. `lightoff_mach` overrides
    # RAMJET_MIN_LIGHTOFF_MACH per call so a campaign can sweep the gate
    # without monkeypatching; `allow_light=False` vetoes the light entirely
    # regardless of Mach, which is how flight_sim's RamjetStart enforces
    # "light in the dive, not in the climb" (a Mach gate alone cannot say
    # WHERE in the trajectory it fires -- see docs/v3_learnings_for_v4.md
    # section 3.3, where the same gate is worth 0.052 g or 0.331 g depending
    # only on which phase it lands in). Both default to the pre-V4 behaviour,
    # so every V2/V3 re-fly is bit-identical.
    # See pulsejet_thrust's matching parameter for why: lets flight_sim.py's
    # hot loop pass its already-computed Atmosphere instead of this
    # function independently recomputing the identical value every step.
    if atmosphere is None:
        atmosphere = standard_atmosphere(altitude_m)
    velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
    inlet_area_m2 = pi * diameter_m**2 / 4.0
    throat_area_m2 = pi * throat_diameter_m**2 / 4.0

    captured_air_mass_flow_kg_per_s = atmosphere.density_kg_per_m3 * velocity_m_per_s * inlet_area_m2
    if captured_air_mass_flow_kg_per_s <= 0.0:
        # No forward speed -> no capture -> a ramjet makes no static thrust.
        return RamjetResult(
            0.0, 0.0, 0.0, 0.0, 0.0, False,
            atmosphere.pressure_pa, atmosphere.temperature_k, 0.0, 0.0, False,
        )

    # Ram compression (ideal, isentropic, no loss): sets both the inlet
    # stagnation temperature and the hard ceiling on chamber pressure --
    # nothing downstream can raise total pressure further.
    temperature_ratio = 1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach**2
    inlet_total_temperature_k = atmosphere.temperature_k * temperature_ratio
    ram_pressure_ceiling_pa = atmosphere.pressure_pa * temperature_ratio ** (GAMMA_AIR / (GAMMA_AIR - 1.0))

    # Stoichiometric fueling and a constant-cp energy balance for chamber T0.
    # Written per unit mass of air so it is independent of how much air
    # actually ends up flowing through the engine (see module docstring,
    # step 3) -- needed below since that amount isn't known yet when the
    # throat is the limiting constraint.
    fuel_air_ratio = 1.0 / fuel.stoichiometric_air_fuel_ratio
    heat_release_j_per_kg_air = fuel_air_ratio * fuel.lower_heating_value_j_per_kg * COMBUSTION_EFFICIENCY
    chamber_total_temperature_k = min(
        inlet_total_temperature_k
        + heat_release_j_per_kg_air / ((1.0 + fuel_air_ratio) * CP_COMB_J_PER_KG_K),
        MAX_CHAMBER_TEMPERATURE_K,
    )

    # Mass flow the throat can actually pass at the ram-ceiling pressure --
    # choked or not, see module docstring step 4. Chamber pressure is always
    # the ram ceiling; an undersized throat caps *mass flow*, not pressure.
    captured_total_mass_flow_kg_per_s = captured_air_mass_flow_kg_per_s * (1.0 + fuel_air_ratio)
    throat_capacity_kg_per_s, choked = compressible_orifice_mass_flow(
        upstream_pressure_pa=ram_pressure_ceiling_pa,
        upstream_temperature_k=chamber_total_temperature_k,
        downstream_pressure_pa=atmosphere.pressure_pa,
        area_m2=throat_area_m2,
        discharge_coefficient=1.0,
        gamma=GAMMA_COMB,
        gas_constant_j_per_kg_k=R_COMB_J_PER_KG_K,
    )
    total_mass_flow_kg_per_s = min(captured_total_mass_flow_kg_per_s, throat_capacity_kg_per_s)
    if total_mass_flow_kg_per_s <= 0.0:
        # Ram ceiling at or below ambient (only possible right at Mach 0,
        # already handled above) or a throat capacity of exactly zero.
        return RamjetResult(
            0.0, 0.0, 0.0, 0.0, captured_air_mass_flow_kg_per_s, False,
            ram_pressure_ceiling_pa, chamber_total_temperature_k, 0.0, 0.0, choked,
        )
    spilled = total_mass_flow_kg_per_s < captured_total_mass_flow_kg_per_s
    chamber_total_pressure_pa = ram_pressure_ceiling_pa

    air_mass_flow_kg_per_s = total_mass_flow_kg_per_s / (1.0 + fuel_air_ratio)
    fuel_mass_flow_kg_per_s = total_mass_flow_kg_per_s - air_mass_flow_kg_per_s

    pressure_ratio = atmosphere.pressure_pa / chamber_total_pressure_pa
    exit_velocity_m_per_s = sqrt(
        max(
            2.0
            * CP_COMB_J_PER_KG_K
            * chamber_total_temperature_k
            * (1.0 - pressure_ratio ** ((GAMMA_COMB - 1.0) / GAMMA_COMB)),
            0.0,
        )
    )

    gross_thrust_n = total_mass_flow_kg_per_s * exit_velocity_m_per_s
    # Momentum drag only on air actually ingested -- spilled air (if any)
    # never enters the engine. Additive drag on the spilled stream itself is
    # not modeled (see module docstring, step 4).
    net_thrust_n = gross_thrust_n - air_mass_flow_kg_per_s * velocity_m_per_s

    # Minimum-viable-speed gate (see RamjetResult.lit): zero below the
    # lightoff Mach, linear ramp over RAMJET_LIGHTOFF_RAMP_MACH above it so
    # the closed form stays continuous. One comparison + one multiply.
    gate_mach = (RAMJET_MIN_LIGHTOFF_MACH if lightoff_mach is None
                 else lightoff_mach)
    lightoff_factor = (mach - gate_mach) / RAMJET_LIGHTOFF_RAMP_MACH
    lightoff_factor = min(max(lightoff_factor, 0.0), 1.0)
    if not allow_light:
        lightoff_factor = 0.0
    lit = lightoff_factor > 0.0
    net_thrust_n *= lightoff_factor
    gross_thrust_n *= lightoff_factor
    fuel_mass_flow_kg_per_s *= lightoff_factor

    # Air-breathing convention: Isp = net thrust / (fuel weight flow), since
    # only the fuel is carried onboard (the oxidizer is ambient air).
    specific_impulse_s = (
        max(net_thrust_n, 0.0) / (fuel_mass_flow_kg_per_s * G0_M_PER_S2)
        if fuel_mass_flow_kg_per_s > 0.0
        else 0.0
    )

    return RamjetResult(
        net_thrust_n=net_thrust_n,
        gross_thrust_n=gross_thrust_n,
        air_mass_flow_kg_per_s=air_mass_flow_kg_per_s,
        fuel_mass_flow_kg_per_s=fuel_mass_flow_kg_per_s,
        captured_air_mass_flow_kg_per_s=captured_air_mass_flow_kg_per_s,
        spilled=spilled,
        chamber_total_pressure_pa=chamber_total_pressure_pa,
        chamber_total_temperature_k=chamber_total_temperature_k,
        exit_velocity_m_per_s=exit_velocity_m_per_s,
        specific_impulse_s=specific_impulse_s,
        choked=choked,
        lit=lit,
    )
