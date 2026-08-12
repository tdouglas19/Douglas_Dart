"""Steady low-order ramjet cycle with explicit operability and flow-balance flags."""

from __future__ import annotations

from dataclasses import dataclass, replace

from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .compressible import (
    fixed_cd_nozzle,
    stagnation_pressure,
    stagnation_temperature,
)
from .config import Fuel, NozzleConfig, RamjetConfig, SelectorConfig
from .gas_properties import real_gas_gamma, real_gas_specific_heat_j_per_kg_k


def ideal_inlet_shock_recovery(mach: float) -> float:
    """Return the idealized inlet total-pressure recovery via the MIL-E-5008B correlation.

    docs/pulsejet_ramjet_governing_equations.md sec. 2.2 warns that a common early-stage-sizing-code
    oversimplification is a single flat pi_d across the whole Mach range. This is
    Mach-dependent: loss-free (1.0) at/below Mach 1, and above Mach 1 follows the
    MIL-E-5008B empirical inlet-recovery schedule -- docs/ramjet_enginesim_comparison.md
    "Difference 1", cross-checked against NASA Glenn Research Center's EngineSim
    (`Turbo.java`'s default "Mil Spec Recovery" inlet option). This models a real
    multi-oblique-shock inlet system rather than a single normal shock -- the prior
    single-normal-shock treatment here matched this closely at this vehicle's
    transonic/low-supersonic design range (~Mach 1.0-1.2) but diverged fast above it
    (confirmed ~4 points low at Mach 1.5, ~20 points low at Mach 2.0), which is
    exactly the replacement this function's own comment used to call for.

    This captures only the idealized inlet-recovery schedule. It is deliberately NOT
    the vehicle's actual installed recovery: duct friction, bends, boundary-layer
    bleed, and this vehicle's own pulsejet/ramjet selector losses are real
    additional losses, captured separately by the configured
    ``SelectorConfig.ramjet_total_pressure_recovery`` installed-efficiency factor
    that multiplies this ideal term (see ``evaluate_ramjet``).
    """

    if mach < 0.0:
        raise ValueError("Mach cannot be negative")
    if mach <= 1.0:
        return 1.0
    return 1.0 - 0.075 * (mach - 1.0) ** 1.35


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
    nozzle_flow_regime: str
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
    result = _solve_ramjet_cycle(config, selector, nozzle, fuel, altitude_m, mach)
    # Below the configured lightoff Mach, the vehicle's own selector logic
    # (trajectory.py) never fires the ramjet at all -- it stays on pulsejet
    # power until mach >= minimum_lightoff_test_mach (itself now derived by
    # derive_lightoff_mach() below, not hand-picked). Solving the full
    # combustion/nozzle cycle down here anyway produces a real but
    # operationally meaningless negative-thrust trough right where the fixed
    # nozzle first starts to pass any flow: momentum drag on that first
    # trickle of throughflow is proportional to mass flow times the full
    # (non-vanishing) freestream velocity, while gross thrust is proportional
    # to mass flow times exit velocity -- and both mass flow and exit
    # velocity are vanishing together, so gross thrust is second-order in the
    # same small quantity that makes drag only first-order. Net thrust is
    # therefore briefly, genuinely negative in an idealized 1D sense, purely
    # from evaluating a condition the engine is never actually commanded to
    # run at (confirmed by direct sweep, 2026-08-10 session). Report zero
    # thrust and fuel flow instead, matching the operational reality that
    # fuel simply isn't scheduled to an unlit engine below this threshold.
    # Inlet aerodynamics (capture, recovery, duct pressure) are still real
    # and reported -- only the combustion/nozzle/thrust fields are zeroed.
    if mach < config.minimum_lightoff_test_mach:
        return replace(
            result,
            air_mass_flow_kg_per_s=0.0,
            fuel_mass_flow_kg_per_s=0.0,
            fuel_air_ratio=0.0,
            combustor_exit_total_temperature_k=result.combustor_inlet_total_temperature_k,
            nozzle_capacity_kg_per_s=0.0,
            nozzle_flow_regime="not_attempted_below_lightoff_mach",
            nozzle_mass_flow_residual_fraction=0.0,
            inlet_spillage_fraction=1.0,
            inlet_momentum_drag_n=0.0,
            gross_thrust_n=0.0,
            net_thrust_n=0.0,
            specific_thrust_n_s_per_kg_air=0.0,
            specific_impulse_s=0.0,
            self_sustaining_candidate=False,
            status=(
                "below_configured_lightoff_test_mach",
                "below_configured_self_sustaining_mach",
                "ramjet_not_attempted_below_lightoff_mach",
            ),
        )
    return result


def _solve_ramjet_cycle(
    config: RamjetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    mach: float,
) -> RamjetResult:
    """Solve the full combustion/nozzle cycle at any Mach, ignoring lightoff gating.

    Deliberately independent of ``config.minimum_lightoff_test_mach`` -- this is
    the raw physics ``evaluate_ramjet`` gates and ``derive_lightoff_mach`` scans
    to find that gate's value in the first place. A function that both used and
    derived the same threshold would be circular.
    """

    atmosphere = standard_atmosphere(altitude_m)
    velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
    # Uses the full circular intake area, not selector.available_area_m2
    # (open_fraction-scaled) -- the open_fraction/mutually-exclusive-half-
    # intake rule is a pulsejet-only assumption now (config.py's
    # SelectorConfig docstring, docs/assumptions.md). When the selector is in
    # ramjet mode the pulsejet path is fully closed, so nothing requires
    # capping the ramjet's captured area at half the intake.
    potential_air_mass_flow_kg_per_s = (
        atmosphere.density_kg_per_m3
        * velocity_m_per_s
        * selector.circular_area_m2
        * config.mass_capture_coefficient
    )
    inlet_total_temperature_k = stagnation_temperature(atmosphere.temperature_k, mach)
    ideal_total_pressure_pa = stagnation_pressure(atmosphere.pressure_pa, mach)
    shock_recovery = ideal_inlet_shock_recovery(mach)
    installed_total_pressure_recovery = shock_recovery * selector.ramjet_total_pressure_recovery
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.3: pi_b = combustor_total_pressure_loss_fraction
    # complement, a nonzero configured loss (not assumed = 1 / lossless).
    inlet_total_pressure_pa = ideal_total_pressure_pa * installed_total_pressure_recovery
    combustor_exit_pressure_pa = inlet_total_pressure_pa * (
        1.0 - config.combustor_total_pressure_loss_fraction
    )
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.3, Brayton-cycle combustor energy
    # balance: f = cp*(Tt3 - Tt2) / (eta_b * h_PR - cp*Tt3), rearranged to solve for Tt3
    # (combustor exit temperature) given f instead of f given Tt3 (2026-08-10 session) --
    # same energy balance, same reference, opposite unknown. fuel_air_ratio is now the
    # input, computed the same way pulsejet.py already computes its own fuel metering
    # (pulsejet.py's PulsejetSimulator.step(): fuel = air * equivalence_ratio /
    # stoichiometric_air_fuel_ratio) rather than backed out of a fixed target
    # temperature -- RamjetConfig.target_equivalence_ratio mirrors
    # PulsejetConfig.target_equivalence_ratio in mechanism, though not in value (0.60
    # lean vs. pulsejet's stoichiometric 1.00; see RamjetConfig's docstring). cp is
    # still evaluated at the combustor-inlet temperature via the real-gas curve fit
    # (docs/ramjet_enginesim_comparison.md "Difference 2") rather than held fixed --
    # unchanged by this rearrangement.
    fuel_air_ratio = config.target_equivalence_ratio / fuel.stoichiometric_air_fuel_ratio
    combustor_inlet_cp_j_per_kg_k = real_gas_specific_heat_j_per_kg_k(inlet_total_temperature_k)
    combustor_exit_temperature_k = (
        fuel_air_ratio * config.combustor_efficiency * fuel.lower_heating_value_j_per_kg
        + combustor_inlet_cp_j_per_kg_k * inlet_total_temperature_k
    ) / (combustor_inlet_cp_j_per_kg_k * (1.0 + fuel_air_ratio))
    potential_fuel_mass_flow_kg_per_s = potential_air_mass_flow_kg_per_s * fuel_air_ratio
    demanded_nozzle_mass_flow_kg_per_s = (
        potential_air_mass_flow_kg_per_s + potential_fuel_mass_flow_kg_per_s
    )

    # docs/ramjet_enginesim_comparison.md "Difference 2": the nozzle expansion uses
    # gamma evaluated at the hot combustor-exit/nozzle-inlet temperature (matching
    # EngineSim's game = getGama(tt[5])), not the same constant gamma used for the
    # cold freestream/inlet above -- combustion products have a meaningfully lower
    # gamma than cold air (~1.30 vs. ~1.40 at this vehicle's combustor temperatures).
    exhaust_gamma = real_gas_gamma(combustor_exit_temperature_k)
    nozzle_result = fixed_cd_nozzle(
        combustor_exit_pressure_pa,
        combustor_exit_temperature_k,
        atmosphere.pressure_pa,
        nozzle.throat_area_m2,
        nozzle.exit_area_m2,
        nozzle.discharge_coefficient,
        exhaust_gamma,
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
                combustor_exit_temperature_k,
                atmosphere.pressure_pa,
                nozzle.throat_area_m2,
                nozzle.exit_area_m2,
                nozzle.discharge_coefficient,
                exhaust_gamma,
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

    # No lightoff gating here by design -- see this function's docstring.
    # evaluate_ramjet() applies that gate on top of this result; the "below
    # self-sustaining Mach" flag below is a separate, always-computed
    # comparison against a still-configured (not derived) threshold.
    status: list[str] = []
    if mach < config.minimum_self_sustaining_mach:
        status.append("lightoff_test_only_below_self_sustaining_mach")
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
        combustor_exit_total_temperature_k=combustor_exit_temperature_k,
        nozzle_capacity_kg_per_s=nozzle_result.mass_flow_kg_per_s,
        nozzle_flow_regime=nozzle_result.regime,
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


def derive_lightoff_mach(
    config: RamjetConfig,
    selector: SelectorConfig,
    nozzle: NozzleConfig,
    fuel: Fuel,
    altitude_m: float,
    *,
    coarse_step_mach: float = 0.005,
) -> float:
    """Derive the Mach at which this engine's own net thrust first sustains positive.

    Replaces what used to be a hand-picked ``minimum_lightoff_test_mach`` YAML
    constant (its own former docstring: "no patent in the switchable-engine
    lineage gives a quantitative transition law... an engineering assumption,
    not a sourced value") with a number read directly off the physics: the
    Mach above which ``_solve_ramjet_cycle``'s net thrust never goes
    non-positive again, up to ``config.minimum_self_sustaining_mach`` (still a
    separate, hand-configured value -- unaffected by this function, and used
    here only as the natural upper search bound).

    This is engine-only: net thrust crossing zero, not net thrust exceeding
    the vehicle's drag. A fixed nozzle's thrust curve typically has a
    negative trough right where it first starts passing flow (see
    ``evaluate_ramjet``'s docstring) before climbing through zero and staying
    positive -- a coarse forward scan finds the *last* non-positive sample in
    range (correctly skipping past that trough, not stopping at its first,
    spurious zero-crossing), then bisects to a precise root just above it.

    Raises ``ValueError`` if net thrust is already positive at Mach 0 (nothing
    to derive) or never sustains positive within
    ``[0, minimum_self_sustaining_mach]`` (an infeasible configuration --
    surfaced here rather than silently returning a meaningless number).
    """

    if coarse_step_mach <= 0.0:
        raise ValueError("coarse_step_mach must be positive")
    upper_bound_mach = config.minimum_self_sustaining_mach

    def net_thrust_at(mach: float) -> float:
        return _solve_ramjet_cycle(config, selector, nozzle, fuel, altitude_m, mach).net_thrust_n

    last_nonpositive_mach: float | None = 0.0 if net_thrust_at(0.0) <= 0.0 else None
    mach = 0.0
    while mach < upper_bound_mach:
        mach = min(mach + coarse_step_mach, upper_bound_mach)
        if net_thrust_at(mach) <= 0.0:
            last_nonpositive_mach = mach

    if last_nonpositive_mach is None:
        raise ValueError(
            "ramjet net thrust is already positive at Mach 0 -- no lightoff crossing to derive"
        )
    if last_nonpositive_mach >= upper_bound_mach:
        raise ValueError(
            "ramjet net thrust never sustains positive within "
            f"[0, minimum_self_sustaining_mach={upper_bound_mach}] -- infeasible configuration"
        )

    low_mach = last_nonpositive_mach
    high_mach = min(low_mach + coarse_step_mach, upper_bound_mach)
    for _ in range(60):
        if high_mach - low_mach < 1e-9:
            break
        mid_mach = 0.5 * (low_mach + high_mach)
        if net_thrust_at(mid_mach) <= 0.0:
            low_mach = mid_mach
        else:
            high_mach = mid_mach
    return high_mach
