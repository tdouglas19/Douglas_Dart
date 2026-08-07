"""Steady low-order ramjet cycle with explicit operability and flow-balance flags."""

from __future__ import annotations

from dataclasses import dataclass

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .compressible import (
    fixed_cd_nozzle,
    normal_shock_total_pressure_ratio,
    stagnation_pressure,
    stagnation_temperature,
)
from .config import Fuel, NozzleConfig, RamjetConfig, SelectorConfig


def ideal_inlet_shock_recovery(mach: float, gamma: float) -> float:
    """Return the idealized (loss-free duct) inlet total-pressure recovery.

    docs/pulsejet_ramjet_governing_equations.md sec. 2.2 warns that a common early-stage-sizing-code
    oversimplification is a single flat pi_d across the whole Mach range. This is
    already Mach-dependent (verified, not changed): a stationary normal shock
    below/at Mach 1 is loss-free (1.0), and above Mach 1 the loss follows the
    normal-shock stagnation-pressure-ratio relation. It is a single-normal-shock
    model, not a multi-oblique-shock MIL-E-5008B-style correlation -- adequate at
    this vehicle's transonic/low-supersonic design range (~Mach 1.0-1.2), but it
    would need replacing with a real shock-train correlation before extending to
    higher supersonic Mach numbers.

    Below Mach 1 there is no shock, so an idealized inlet has no theoretical
    stagnation-pressure loss (``1.0``); above Mach 1, the loss is exactly the
    stationary normal-shock total-pressure ratio at the local freestream Mach
    number, reusing this repository's own tested normal-shock relation
    (``normal_shock_total_pressure_ratio``) rather than an empirical curve fit.

    This captures only the idealized shock-system loss. It is deliberately NOT the
    vehicle's actual installed recovery: duct friction, bends, boundary-layer
    bleed, and this vehicle's own pulsejet/ramjet selector losses are real
    additional losses, captured separately by the configured
    ``SelectorConfig.ramjet_total_pressure_recovery`` installed-efficiency factor
    that multiplies this ideal term (see ``evaluate_ramjet``).
    """

    if mach < 0.0:
        raise ValueError("Mach cannot be negative")
    if mach <= 1.0:
        return 1.0
    return normal_shock_total_pressure_ratio(mach, gamma)


@dataclass(frozen=True)
class RamjetResult:
    mach: float
    potential_captured_air_mass_flow_kg_per_s: float
    air_mass_flow_kg_per_s: float
    fuel_mass_flow_kg_per_s: float
    fuel_air_ratio: float
    ideal_inlet_shock_recovery: float
    installed_total_pressure_recovery: float
    combustor_inlet_total_pressure_pa: float
    combustor_exit_total_pressure_pa: float
    combustor_inlet_total_temperature_k: float
    combustor_exit_total_temperature_k: float
    nozzle_capacity_kg_per_s: float
    nozzle_mass_flow_residual_fraction: float
    inlet_spillage_fraction: float
    inlet_momentum_drag_n: float
    gross_thrust_n: float
    net_thrust_n: float
    specific_thrust_n_s_per_kg_air: float
    specific_impulse_s: float
    self_sustaining_candidate: bool
    status: tuple[str, ...]


def evaluate_ramjet(
    config: RamjetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
) -> RamjetResult:
    if mach < 0.0:
        raise ValueError("Mach cannot be negative")
    atmosphere = standard_atmosphere(altitude_m)
    velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
    potential_air_mass_flow_kg_per_s = (
        atmosphere.density_kg_per_m3
        * velocity_m_per_s
        * selector.available_area_m2
        * config.mass_capture_coefficient
    )
    inlet_total_temperature_k = stagnation_temperature(atmosphere.temperature_k, mach)
    ideal_total_pressure_pa = stagnation_pressure(atmosphere.pressure_pa, mach)
    shock_recovery = ideal_inlet_shock_recovery(mach, config.gamma)
    installed_total_pressure_recovery = shock_recovery * selector.ramjet_total_pressure_recovery
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.3: pi_b = combustor_total_pressure_loss_fraction
    # complement, a nonzero configured loss (not assumed = 1 / lossless).
    inlet_total_pressure_pa = ideal_total_pressure_pa * installed_total_pressure_recovery
    combustor_exit_pressure_pa = inlet_total_pressure_pa * (
        1.0 - config.combustor_total_pressure_loss_fraction
    )
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.3, Brayton-cycle combustor energy balance:
    # f = cp*(Tt3 - Tt2) / (eta_b * h_PR - cp*Tt3). Already matches the reference
    # exactly -- verified, not changed.
    cp_j_per_kg_k = config.gamma * config.gas_constant_j_per_kg_k / (config.gamma - 1.0)
    target_temperature_k = config.target_combustor_exit_temperature_k
    numerator = cp_j_per_kg_k * max(target_temperature_k - inlet_total_temperature_k, 0.0)
    denominator = (
        config.combustor_efficiency * fuel.lower_heating_value_j_per_kg
        - cp_j_per_kg_k * target_temperature_k
    )
    fuel_air_ratio = numerator / denominator if denominator > 0.0 else 0.0
    potential_fuel_mass_flow_kg_per_s = potential_air_mass_flow_kg_per_s * fuel_air_ratio
    demanded_nozzle_mass_flow_kg_per_s = (
        potential_air_mass_flow_kg_per_s + potential_fuel_mass_flow_kg_per_s
    )

    nozzle_result = fixed_cd_nozzle(
        combustor_exit_pressure_pa,
        target_temperature_k,
        atmosphere.pressure_pa,
        nozzle.throat_area_m2,
        nozzle.exit_area_m2,
        nozzle.discharge_coefficient,
        config.gamma,
        config.gas_constant_j_per_kg_k,
    )

    # docs/pulsejet_ramjet_governing_equations.md sec. 2.5: "if the nozzle's mass-flow capacity ...
    # is less than the engine's ingested mass flow, inlet spillage must
    # increase; if nozzle capacity exceeds engine mass flow, the inlet must go
    # supercritical, reducing delivered stagnation pressure -- both cases
    # require an iterative balance, not a single-pass calculation."
    #
    # Subcritical (nozzle_result.mass_flow_kg_per_s < demanded): the existing
    # inlet-spillage treatment below already satisfies this branch structurally
    # -- spillage happens upstream of the inlet and does not itself change the
    # captured stream's stagnation pressure, so no iteration is needed here.
    #
    # Supercritical (nozzle_result.mass_flow_kg_per_s >= demanded, i.e. the
    # fixed nozzle geometry could pass more than the inlet is actually
    # delivering): iterate an additional recovery penalty, driven by how far
    # capacity exceeds demand (oversupply_ratio), against nozzle capacity until
    # that ratio converges, rather than a single post-hoc thrust scaling. Note
    # the penalty cannot be driven by a capacity/demand "fill fraction" here,
    # since min(demand, capacity)/demand saturates at exactly 1.0 throughout
    # this whole branch by construction and so carries no information about
    # how oversized the nozzle capacity actually is.
    # Convergence criterion: |delta oversupply_ratio| < 1e-6, capped at 30
    # iterations (this low-order model converges in a handful of iterations in
    # practice; the cap only guards against a pathological configuration).
    if (
        demanded_nozzle_mass_flow_kg_per_s > 1e-12
        and nozzle_result.mass_flow_kg_per_s >= demanded_nozzle_mass_flow_kg_per_s
    ):
        oversupply_ratio = (
            nozzle_result.mass_flow_kg_per_s / demanded_nozzle_mass_flow_kg_per_s - 1.0
        )
        for _ in range(30):
            recovery_penalty = config.supercritical_recovery_penalty_coefficient * min(
                oversupply_ratio, 1.0
            )
            penalized_recovery = installed_total_pressure_recovery * (1.0 - recovery_penalty)
            inlet_total_pressure_pa = ideal_total_pressure_pa * penalized_recovery
            combustor_exit_pressure_pa = inlet_total_pressure_pa * (
                1.0 - config.combustor_total_pressure_loss_fraction
            )
            nozzle_result = fixed_cd_nozzle(
                combustor_exit_pressure_pa,
                target_temperature_k,
                atmosphere.pressure_pa,
                nozzle.throat_area_m2,
                nozzle.exit_area_m2,
                nozzle.discharge_coefficient,
                config.gamma,
                config.gas_constant_j_per_kg_k,
            )
            new_oversupply_ratio = max(
                0.0,
                nozzle_result.mass_flow_kg_per_s / demanded_nozzle_mass_flow_kg_per_s - 1.0,
            )
            if abs(new_oversupply_ratio - oversupply_ratio) < 1e-6:
                oversupply_ratio = new_oversupply_ratio
                break
            oversupply_ratio = new_oversupply_ratio

    if demanded_nozzle_mass_flow_kg_per_s > 1e-12:
        residual_fraction = (
            nozzle_result.mass_flow_kg_per_s - demanded_nozzle_mass_flow_kg_per_s
        ) / demanded_nozzle_mass_flow_kg_per_s
    else:
        residual_fraction = 0.0

    actual_nozzle_mass_flow_kg_per_s = min(
        demanded_nozzle_mass_flow_kg_per_s, nozzle_result.mass_flow_kg_per_s
    )
    air_mass_flow_kg_per_s = actual_nozzle_mass_flow_kg_per_s / (1.0 + fuel_air_ratio)
    fuel_mass_flow_kg_per_s = air_mass_flow_kg_per_s * fuel_air_ratio
    inlet_spillage_fraction = (
        1.0 - air_mass_flow_kg_per_s / potential_air_mass_flow_kg_per_s
        if potential_air_mass_flow_kg_per_s > 1e-12
        else 0.0
    )
    nozzle_flow_scale = (
        actual_nozzle_mass_flow_kg_per_s / nozzle_result.mass_flow_kg_per_s
        if nozzle_result.mass_flow_kg_per_s > 1e-12
        else 0.0
    )
    # Any remaining mismatch after the supercritical iteration above (or the
    # unmodified subcritical/throat-limited case) is still scaled the same way;
    # in the converged supercritical case this scale factor is now consistent
    # with the penalized combustor_exit_pressure_pa, not the unpenalized
    # first-pass value.
    gross_thrust_n = nozzle_result.gross_thrust_n * nozzle_flow_scale
    net_thrust_n = gross_thrust_n - air_mass_flow_kg_per_s * velocity_m_per_s
    specific_thrust = (
        net_thrust_n / air_mass_flow_kg_per_s if air_mass_flow_kg_per_s > 1e-12 else 0.0
    )
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.5: I_sp = F_net / (mdot_fuel * g0).
    specific_impulse_s = (
        net_thrust_n / (fuel_mass_flow_kg_per_s * G0_M_PER_S2)
        if fuel_mass_flow_kg_per_s > 1e-12
        else 0.0
    )

    status: list[str] = []
    if mach < config.minimum_lightoff_test_mach:
        status.append("below_configured_lightoff_test_mach")
    elif mach < config.minimum_self_sustaining_mach:
        status.append("lightoff_test_only_below_self_sustaining_mach")
    if mach < config.minimum_self_sustaining_mach:
        status.append("below_configured_self_sustaining_mach")
    if combustor_exit_pressure_pa <= atmosphere.pressure_pa:
        status.append("insufficient_nozzle_pressure_ratio")
    if residual_fraction < -0.20:
        # A throat-limited flowpath can operate only if the external inlet spills
        # the uncaptured streamtube and the resulting back-pressure remains stable.
        # This is an unresolved inlet-matching requirement, but it is not the same
        # failure mode as an under-fed nozzle at the assumed combustor pressure.
        status.append("fixed_nozzle_requires_inlet_spillage_coupling")
    elif residual_fraction > 0.20:
        status.append("fixed_nozzle_underfed_pressure_match_required")
    if nozzle_result.warning:
        status.append(nozzle_result.warning)
    self_sustaining_candidate = not any(
        flag
        in {
            "below_configured_self_sustaining_mach",
            "insufficient_nozzle_pressure_ratio",
            "fixed_nozzle_underfed_pressure_match_required",
        }
        for flag in status
    )
    return RamjetResult(
        mach=mach,
        potential_captured_air_mass_flow_kg_per_s=potential_air_mass_flow_kg_per_s,
        air_mass_flow_kg_per_s=air_mass_flow_kg_per_s,
        fuel_mass_flow_kg_per_s=fuel_mass_flow_kg_per_s,
        fuel_air_ratio=fuel_air_ratio,
        ideal_inlet_shock_recovery=shock_recovery,
        installed_total_pressure_recovery=installed_total_pressure_recovery,
        combustor_inlet_total_pressure_pa=inlet_total_pressure_pa,
        combustor_exit_total_pressure_pa=combustor_exit_pressure_pa,
        combustor_inlet_total_temperature_k=inlet_total_temperature_k,
        combustor_exit_total_temperature_k=target_temperature_k,
        nozzle_capacity_kg_per_s=nozzle_result.mass_flow_kg_per_s,
        nozzle_mass_flow_residual_fraction=residual_fraction,
        inlet_spillage_fraction=inlet_spillage_fraction,
        inlet_momentum_drag_n=air_mass_flow_kg_per_s * velocity_m_per_s,
        gross_thrust_n=gross_thrust_n,
        net_thrust_n=net_thrust_n,
        specific_thrust_n_s_per_kg_air=specific_thrust,
        specific_impulse_s=specific_impulse_s,
        self_sustaining_candidate=self_sustaining_candidate,
        status=tuple(status),
    )
