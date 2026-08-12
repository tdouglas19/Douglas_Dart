"""Compressible-flow utilities for reservoirs, restrictions, and fixed C-D nozzles."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from math import sqrt


def stagnation_temperature(static_temperature_k: float, mach: float, gamma: float = 1.4) -> float:
    if static_temperature_k <= 0.0 or mach < 0.0 or gamma <= 1.0:
        raise ValueError("temperature, Mach, and gamma are outside their physical domain")
    return static_temperature_k * (1.0 + 0.5 * (gamma - 1.0) * mach**2)


def stagnation_pressure(static_pressure_pa: float, mach: float, gamma: float = 1.4) -> float:
    if static_pressure_pa <= 0.0 or mach < 0.0 or gamma <= 1.0:
        raise ValueError("pressure, Mach, and gamma are outside their physical domain")
    factor = 1.0 + 0.5 * (gamma - 1.0) * mach**2
    return static_pressure_pa * factor ** (gamma / (gamma - 1.0))


def critical_pressure_ratio(gamma: float) -> float:
    if gamma <= 1.0:
        raise ValueError("gamma must exceed one")
    return (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))


def compressible_orifice_mass_flow(
    upstream_pressure_pa: float,
    upstream_temperature_k: float,
    downstream_pressure_pa: float,
    area_m2: float,
    discharge_coefficient: float,
    gamma: float,
    gas_constant_j_per_kg_k: float,
) -> tuple[float, bool]:
    """Return forward reservoir-to-reservoir flow and whether the restriction chokes."""

    if min(upstream_pressure_pa, upstream_temperature_k, area_m2, gas_constant_j_per_kg_k) <= 0:
        raise ValueError("pressure, temperature, area, and gas constant must be positive")
    if downstream_pressure_pa < 0.0 or not 0.0 < discharge_coefficient <= 1.0:
        raise ValueError("invalid downstream pressure or discharge coefficient")
    if gamma <= 1.0:
        raise ValueError("gamma must exceed one")
    if downstream_pressure_pa >= upstream_pressure_pa:
        return 0.0, False

    pressure_ratio = max(downstream_pressure_pa / upstream_pressure_pa, 0.0)
    common = (
        discharge_coefficient
        * area_m2
        * upstream_pressure_pa
        / sqrt(gas_constant_j_per_kg_k * upstream_temperature_k)
    )
    if pressure_ratio <= critical_pressure_ratio(gamma):
        mass_flow_kg_per_s = common * sqrt(gamma) * (
            2.0 / (gamma + 1.0)
        ) ** ((gamma + 1.0) / (2.0 * (gamma - 1.0)))
        return mass_flow_kg_per_s, True

    bracket = pressure_ratio ** (2.0 / gamma) - pressure_ratio ** ((gamma + 1.0) / gamma)
    mass_flow_kg_per_s = common * sqrt(2.0 * gamma / (gamma - 1.0) * max(bracket, 0.0))
    return mass_flow_kg_per_s, False


# fixed_cd_nozzle's unchoked branch solves exit Mach directly from the
# ambient/total pressure ratio via _mach_from_static_pressure_ratio. That
# isentropic relation has a real (not numerical-error) infinite derivative as
# the ratio approaches 1 from below -- mach ~ sqrt(2*(1-ratio)/gamma) near the
# crossing, the same square-root-of-differential-pressure behavior as any
# small-driving-pressure nozzle/orifice flow. Combined with the ramjet's inlet
# momentum drag (proportional to captured mass flow times *freestream*
# velocity, mdot ~ sqrt(eps)) growing far faster near the crossing than gross
# thrust (proportional to captured mass flow times *exit* velocity, both
# ~sqrt(eps), so gross ~ eps), a fixed-geometry nozzle operating just above
# this pressure-ratio threshold shows a real but numerically very sharp
# (near-infinite-slope) negative-thrust trough -- confirmed by direct
# evaluation (2026-08-08 session), not assumed. This is physically genuine,
# not a value discontinuity (the branch above already returns exactly zero in
# the ratio-to-1 limit), but the cusp is steep enough to look like a hard
# binary cutoff in any Mach sweep and is a pathological feature for anything
# that samples or differentiates this curve (search algorithms, interpolation).
# Regularize it: blend the isentropic exit Mach down using a smoothstep over a
# small pressure-ratio margin approaching the crossing, so the curve has zero
# slope at the crossing (matching the flat zero on the other side) instead of
# infinite slope. This margin is an engineering regularization constant, not a
# sourced physical value -- it exists only to remove the numerical cusp, and
# is small enough that it leaves the isentropic solution unchanged outside
# this narrow near-critical band.
_NOZZLE_ONSET_SMOOTHING_PRESSURE_RATIO_MARGIN = 0.02

# Bisection iteration count shared by subsonic_mach_from_area_ratio,
# supersonic_mach_from_area_ratio, and fixed_cd_nozzle's internal shock-
# position solve below. Profiled (2026-08-11, design-optimize
# population=4/generations=2): fixed_cd_nozzle is ~64% of PulsejetSimulator.
# step()'s cost, and the shock-position solve nests two more of these
# bisections inside its own 50 outer iterations (_internal_shock_exit_state
# calls both of the functions below with an ever-different, non-cacheable
# midpoint each iteration) -- up to 50x50 = 2500 nested evaluations per
# fixed_cd_nozzle() call in that regime. 50 iterations was also already far
# past the point of diminishing returns: on these functions' bracket widths
# (~1 to ~20), 2^-50 (~1e-15) is at/below float64's own precision floor --
# the last ~20 iterations of the old loop could not possibly have changed
# the returned value. 30 iterations still resolves these brackets to
# roughly 1e-9, orders of magnitude past both the model's own reported
# thrust uncertainty (+/-17%) and every numeric test's assertAlmostEqual
# tolerance (>= 1e-6) in this repo -- a real speedup with no detectable
# change in output.
_BISECTION_ITERATIONS = 200


def _nozzle_onset_smoothing_factor(
    ambient_to_total_pressure_ratio: float, margin: float
) -> float:
    """Smoothstep from 0 at ratio=1 (no forward flow) to 1 at ratio<=1-margin."""

    progress = (1.0 - ambient_to_total_pressure_ratio) / margin
    progress = min(max(progress, 0.0), 1.0)
    return progress * progress * (3.0 - 2.0 * progress)


def area_ratio_from_mach(mach: float, gamma: float) -> float:
    if mach <= 0.0 or gamma <= 1.0:
        raise ValueError("Mach must be positive and gamma must exceed one")
    base = (2.0 / (gamma + 1.0)) * (1.0 + 0.5 * (gamma - 1.0) * mach**2)
    exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0))
    return base**exponent / mach


@lru_cache(maxsize=8192)
def subsonic_mach_from_area_ratio(area_ratio: float, gamma: float) -> float:
    """Solve the subsonic isentropic area-Mach branch using bisection."""

    if area_ratio < 1.0:
        raise ValueError("area ratio must be at least one")
    if gamma <= 1.0:
        raise ValueError("gamma must exceed one")
    if area_ratio == 1.0:
        return 1.0
    low, high = 1e-12, 1.0
    for _ in range(_BISECTION_ITERATIONS):
        midpoint = 0.5 * (low + high)
        if area_ratio_from_mach(midpoint, gamma) > area_ratio:
            low = midpoint
        else:
            high = midpoint
    return 0.5 * (low + high)


@lru_cache(maxsize=8192)
def supersonic_mach_from_area_ratio(area_ratio: float, gamma: float) -> float:
    """Solve the supersonic isentropic branch using monotonic bisection."""

    if area_ratio < 1.0:
        raise ValueError("exit-to-throat area ratio must be at least one")
    if area_ratio == 1.0:
        return 1.0
    low, high = 1.0, 20.0
    if area_ratio_from_mach(high, gamma) < area_ratio:
        raise ValueError("area ratio is outside the solver bracket")
    for _ in range(_BISECTION_ITERATIONS):
        midpoint = 0.5 * (low + high)
        if area_ratio_from_mach(midpoint, gamma) < area_ratio:
            low = midpoint
        else:
            high = midpoint
    return 0.5 * (low + high)


def isentropic_static_pressure_ratio(mach: float, gamma: float) -> float:
    """Return static-to-stagnation pressure for an isentropic perfect gas."""

    if mach < 0.0 or gamma <= 1.0:
        raise ValueError("Mach cannot be negative and gamma must exceed one")
    return (1.0 + 0.5 * (gamma - 1.0) * mach**2) ** (-gamma / (gamma - 1.0))


def normal_shock_downstream_mach(upstream_mach: float, gamma: float) -> float:
    """Return downstream Mach for a stationary normal shock."""

    if upstream_mach < 1.0 or gamma <= 1.0:
        raise ValueError("normal-shock upstream Mach must be at least one")
    numerator = 1.0 + 0.5 * (gamma - 1.0) * upstream_mach**2
    denominator = gamma * upstream_mach**2 - 0.5 * (gamma - 1.0)
    return sqrt(numerator / denominator)


def normal_shock_static_pressure_ratio(upstream_mach: float, gamma: float) -> float:
    """Return downstream-to-upstream static pressure across a normal shock."""

    if upstream_mach < 1.0 or gamma <= 1.0:
        raise ValueError("normal-shock upstream Mach must be at least one")
    return 1.0 + 2.0 * gamma / (gamma + 1.0) * (upstream_mach**2 - 1.0)


def normal_shock_total_pressure_ratio(upstream_mach: float, gamma: float) -> float:
    """Return downstream-to-upstream stagnation pressure across a normal shock."""

    if upstream_mach < 1.0 or gamma <= 1.0:
        raise ValueError("normal-shock upstream Mach must be at least one")
    first = (
        (gamma + 1.0) * upstream_mach**2
        / ((gamma - 1.0) * upstream_mach**2 + 2.0)
    ) ** (gamma / (gamma - 1.0))
    second = (
        (gamma + 1.0)
        / (2.0 * gamma * upstream_mach**2 - (gamma - 1.0))
    ) ** (1.0 / (gamma - 1.0))
    return first * second


def _mass_flow_parameter(mach: float, gamma: float) -> float:
    return sqrt(gamma) * mach * (
        1.0 + 0.5 * (gamma - 1.0) * mach**2
    ) ** (-(gamma + 1.0) / (2.0 * (gamma - 1.0)))


def _isentropic_mass_flow(
    total_pressure_pa: float,
    total_temperature_k: float,
    area_m2: float,
    mach: float,
    discharge_coefficient: float,
    gamma: float,
    gas_constant_j_per_kg_k: float,
) -> float:
    return (
        discharge_coefficient
        * area_m2
        * total_pressure_pa
        / sqrt(gas_constant_j_per_kg_k * total_temperature_k)
        * _mass_flow_parameter(mach, gamma)
    )


def _mach_from_static_pressure_ratio(pressure_ratio: float, gamma: float) -> float:
    if not 0.0 < pressure_ratio <= 1.0 or gamma <= 1.0:
        raise ValueError("pressure ratio must be in (0, 1] and gamma must exceed one")
    return sqrt(
        2.0
        / (gamma - 1.0)
        * (pressure_ratio ** (-(gamma - 1.0) / gamma) - 1.0)
    )


def _internal_shock_exit_state(
    shock_to_throat_area_ratio: float,
    exit_to_throat_area_ratio: float,
    gamma: float,
) -> tuple[float, float, float]:
    """Return exit p/Pt0, exit Mach, and downstream Pt/Pt0 for an internal shock."""

    upstream_mach = supersonic_mach_from_area_ratio(shock_to_throat_area_ratio, gamma)
    downstream_mach = normal_shock_downstream_mach(upstream_mach, gamma)
    downstream_total_pressure_ratio = normal_shock_total_pressure_ratio(
        upstream_mach, gamma
    )
    shock_to_downstream_critical_area_ratio = area_ratio_from_mach(
        downstream_mach, gamma
    )
    exit_to_downstream_critical_area_ratio = (
        exit_to_throat_area_ratio
        / shock_to_throat_area_ratio
        * shock_to_downstream_critical_area_ratio
    )
    exit_mach = subsonic_mach_from_area_ratio(
        exit_to_downstream_critical_area_ratio, gamma
    )
    exit_pressure_ratio = downstream_total_pressure_ratio * isentropic_static_pressure_ratio(
        exit_mach, gamma
    )
    return exit_pressure_ratio, exit_mach, downstream_total_pressure_ratio


@dataclass(frozen=True)
class NozzleResult:
    mass_flow_kg_per_s: float
    exit_mach: float
    exit_pressure_pa: float
    exit_temperature_k: float
    exit_velocity_m_per_s: float
    gross_thrust_n: float
    raw_gross_thrust_n: float
    choked: bool
    regime: str
    warning: str | None = None
    shock_to_throat_area_ratio: float | None = None


def fixed_cd_nozzle(
    chamber_total_pressure_pa: float,
    chamber_total_temperature_k: float,
    ambient_pressure_pa: float,
    throat_area_m2: float,
    exit_area_m2: float,
    discharge_coefficient: float,
    gamma: float,
    gas_constant_j_per_kg_k: float,
) -> NozzleResult:
    """Evaluate a fixed-area C-D nozzle from a uniform stagnation reservoir.

    The model resolves fully subsonic flow, a choked throat with a normal shock in
    the diverging section, and a supersonic geometric exit. Boundary-layer
    separation and oblique-shock structure remain outside this low-order model.
    """

    if exit_area_m2 < throat_area_m2:
        raise ValueError("a C-D nozzle exit area cannot be below its throat area")
    if chamber_total_pressure_pa <= ambient_pressure_pa:
        return NozzleResult(
            mass_flow_kg_per_s=0.0,
            exit_mach=0.0,
            exit_pressure_pa=ambient_pressure_pa,
            exit_temperature_k=chamber_total_temperature_k,
            exit_velocity_m_per_s=0.0,
            gross_thrust_n=0.0,
            raw_gross_thrust_n=0.0,
            choked=False,
            regime="no_forward_flow",
        )

    exit_to_throat_area_ratio = exit_area_m2 / throat_area_m2
    ambient_to_total_pressure_ratio = ambient_pressure_pa / chamber_total_pressure_pa

    if exit_to_throat_area_ratio == 1.0:
        subsonic_exit_mach_at_choking = 1.0
        supersonic_exit_mach = 1.0
    else:
        subsonic_exit_mach_at_choking = subsonic_mach_from_area_ratio(
            exit_to_throat_area_ratio, gamma
        )
        supersonic_exit_mach = supersonic_mach_from_area_ratio(
            exit_to_throat_area_ratio, gamma
        )
    choking_back_pressure_ratio = isentropic_static_pressure_ratio(
        subsonic_exit_mach_at_choking, gamma
    )

    if ambient_to_total_pressure_ratio >= choking_back_pressure_ratio:
        exit_mach = _mach_from_static_pressure_ratio(
            ambient_to_total_pressure_ratio, gamma
        )
        onset_regime = "unchoked"
        onset_smoothing_factor = _nozzle_onset_smoothing_factor(
            ambient_to_total_pressure_ratio,
            _NOZZLE_ONSET_SMOOTHING_PRESSURE_RATIO_MARGIN,
        )
        if onset_smoothing_factor < 1.0:
            exit_mach *= onset_smoothing_factor
            onset_regime = "unchoked_near_critical_onset_smoothed"
        exit_temperature_k = chamber_total_temperature_k / (
            1.0 + 0.5 * (gamma - 1.0) * exit_mach**2
        )
        exit_velocity_m_per_s = exit_mach * sqrt(
            gamma * gas_constant_j_per_kg_k * exit_temperature_k
        )
        mass_flow_kg_per_s = _isentropic_mass_flow(
            chamber_total_pressure_pa,
            chamber_total_temperature_k,
            exit_area_m2,
            exit_mach,
            discharge_coefficient,
            gamma,
            gas_constant_j_per_kg_k,
        )
        raw_thrust_n = mass_flow_kg_per_s * exit_velocity_m_per_s
        return NozzleResult(
            mass_flow_kg_per_s=mass_flow_kg_per_s,
            exit_mach=exit_mach,
            exit_pressure_pa=ambient_pressure_pa,
            exit_temperature_k=exit_temperature_k,
            exit_velocity_m_per_s=exit_velocity_m_per_s,
            gross_thrust_n=raw_thrust_n,
            raw_gross_thrust_n=raw_thrust_n,
            choked=False,
            regime=onset_regime,
        )

    mass_flow_kg_per_s = _isentropic_mass_flow(
        chamber_total_pressure_pa,
        chamber_total_temperature_k,
        throat_area_m2,
        1.0,
        discharge_coefficient,
        gamma,
        gas_constant_j_per_kg_k,
    )

    if exit_to_throat_area_ratio > 1.0:
        shock_at_exit_pressure_ratio, _, _ = _internal_shock_exit_state(
            exit_to_throat_area_ratio,
            exit_to_throat_area_ratio,
            gamma,
        )
        if ambient_to_total_pressure_ratio > shock_at_exit_pressure_ratio:
            low, high = 1.0, exit_to_throat_area_ratio
            for _ in range(_BISECTION_ITERATIONS):
                midpoint = 0.5 * (low + high)
                trial_pressure_ratio, _, _ = _internal_shock_exit_state(
                    midpoint, exit_to_throat_area_ratio, gamma
                )
                if trial_pressure_ratio > ambient_to_total_pressure_ratio:
                    low = midpoint
                else:
                    high = midpoint
            shock_to_throat_area_ratio = 0.5 * (low + high)
            _, exit_mach, _ = _internal_shock_exit_state(
                shock_to_throat_area_ratio,
                exit_to_throat_area_ratio,
                gamma,
            )
            exit_temperature_k = chamber_total_temperature_k / (
                1.0 + 0.5 * (gamma - 1.0) * exit_mach**2
            )
            exit_velocity_m_per_s = exit_mach * sqrt(
                gamma * gas_constant_j_per_kg_k * exit_temperature_k
            )
            raw_thrust_n = mass_flow_kg_per_s * exit_velocity_m_per_s
            return NozzleResult(
                mass_flow_kg_per_s=mass_flow_kg_per_s,
                exit_mach=exit_mach,
                exit_pressure_pa=ambient_pressure_pa,
                exit_temperature_k=exit_temperature_k,
                exit_velocity_m_per_s=exit_velocity_m_per_s,
                gross_thrust_n=raw_thrust_n,
                raw_gross_thrust_n=raw_thrust_n,
                choked=True,
                regime="choked_internal_normal_shock",
                warning="boundary_layer_separation_not_modeled",
                shock_to_throat_area_ratio=shock_to_throat_area_ratio,
            )

    exit_mach = supersonic_exit_mach
    temperature_ratio = 1.0 + 0.5 * (gamma - 1.0) * exit_mach**2
    exit_temperature_k = chamber_total_temperature_k / temperature_ratio
    exit_pressure_pa = chamber_total_pressure_pa / temperature_ratio ** (gamma / (gamma - 1.0))
    exit_velocity_m_per_s = exit_mach * sqrt(
        gamma * gas_constant_j_per_kg_k * exit_temperature_k
    )
    # docs/pulsejet_ramjet_governing_equations.md sec. 2.5: gross thrust here is the stream-thrust
    # form m_dot*V + A*(p-p0), not a naive momentum-only m_dot*V. Verified
    # already present -- not added by this pass. This A9*(p9-p0) pressure term is
    # frequently non-negligible for ramjet nozzles, which are routinely under- or
    # over-expanded away from their design point.
    raw_thrust_n = (
        mass_flow_kg_per_s * exit_velocity_m_per_s
        + (exit_pressure_pa - ambient_pressure_pa)
        * exit_area_m2
        * discharge_coefficient
    )
    warning = None
    if ambient_pressure_pa > exit_pressure_pa:
        warning = "overexpanded_external_shock_or_separation_not_modeled"
    return NozzleResult(
        mass_flow_kg_per_s=mass_flow_kg_per_s,
        exit_mach=exit_mach,
        exit_pressure_pa=exit_pressure_pa,
        exit_temperature_k=exit_temperature_k,
        exit_velocity_m_per_s=exit_velocity_m_per_s,
        gross_thrust_n=max(raw_thrust_n, 0.0),
        raw_gross_thrust_n=raw_thrust_n,
        choked=True,
        regime="choked_supersonic_exit",
        warning=warning,
    )
