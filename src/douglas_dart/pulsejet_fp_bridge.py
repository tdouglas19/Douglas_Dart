"""Bridge to pulsejet-fp: the first-principles transient valved-pulsejet
model (sibling repo, installed editable as ``pulsejet_fp``) that serves as
the PRIMARY pulsejet query source for the propulsion map.

Why this model is primary (vs. the native 0D ``pulsejet.py`` and the
pulsejet-km transcription): it resolves the physics both alternatives were
audited as lacking -- quasi-1D tailpipe wave dynamics (the "liquid piston"),
an inertial reed-valve ODE derived from beam theory, and Arrhenius
heat-release phasing (Rayleigh-criterion coupling emerges instead of being
fitted). Its verification record lives in the sibling repo:
``pulsejet-fp/architecture.md`` and ``docs/derivation.md`` (exact-Riemann
solver validation, machine-precision conservation, dual independent thrust
formulations agreeing <1%, grid-convergence study, adversarial audit).
The vehicle-relevant configuration -- side-mounted inlets ingesting
boundary-layer air at static pressure -- is modeled natively
(derivation.md #8c) and was demonstrated to operate across M 0-0.9 with
coincident cold-start/operating branches.

Geometry mapping: the native ``PulsejetConfig`` is 0D -- its only geometric
handle is ``chamber_volume_m3`` (a live optimizer search variable). Each
case's engine is therefore derived by scaling the validated FP-1 reference
design isometrically to the case's chamber volume (lengths and diameters
by s = (V/V_ref)^(1/3); petal COUNT by s^2 at fixed, hardware-plausible
petal size so valve-area fraction and reed dynamics are preserved; plenum
volume by s^3). Every point produced this way carries the
``pulsejet_fp_geometry_scaled_from_chamber_volume`` validity flag so the
provenance is visible downstream.

Known, deliberately-flagged fidelity limits (see the sibling repo's
assumption registry): adiabatic walls make thrust/TSFC optimistic
(A6), and the fuel is modeled as premixed propane-air regardless of the
case's Fuel config (LHV differs from kerosene by ~7%) -- both carried as
unconditional validity flags, following the same honest-labeling pattern
as ``_PULSEJET_KM_THRUST_LOW_BIAS_FLAG``.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from functools import lru_cache
from typing import TYPE_CHECKING, Any


def pulsejet_fp_primary_enabled() -> bool:
    """PULSEJET_MODE tries pulsejet-fp first unless this kill-switch is set.

    ``DOUGLAS_DART_DISABLE_PULSEJET_FP=1`` restores the pre-2026-08-12
    dispatch (km-guarded -> native). Intended for the legacy test suite and
    for triaging a suspected sibling-model regression -- NOT a fidelity
    tier. Each transient FP point costs ~30-90 s wall (memoized per unique
    (spec, mach, altitude, fidelity)), which is the deliberate price of the
    first-principles primary; high-volume inner search loops that cannot
    afford it should set this switch explicitly and visibly rather than
    silently receiving 0D physics."""

    return os.environ.get("DOUGLAS_DART_DISABLE_PULSEJET_FP", "") not in ("1", "true", "yes")

if TYPE_CHECKING:  # lazy, like the pulsejet-km bridge
    from pulsejet_fp import ThrustResult
    from .config import ReferenceCase

# ---------------------------------------------------------------------------
# FP-1 reference design constants (must match pulsejet_fp.query.reference_*;
# asserted against the live package in tests/test_pulsejet_fp_bridge.py so
# drift in the sibling repo is caught, not silently absorbed).
# ---------------------------------------------------------------------------
_FP1_CHAMBER_DIAMETER_M = 0.078
_FP1_CHAMBER_LENGTH_M = 0.150
_FP1_CONE_LENGTH_M = 0.130
_FP1_TAILPIPE_DIAMETER_M = 0.042
_FP1_TAILPIPE_LENGTH_M = 0.620
_FP1_N_PETALS = 9
_FP1_BL_MOMENTUM_FRACTION = 0.6

PULSEJET_FP_ADIABATIC_OPTIMISM_FLAG = "pulsejet_fp_adiabatic_walls_thrust_optimistic_see_pulsejet_fp_derivation_a6"
PULSEJET_FP_PROPANE_SURROGATE_FLAG = "pulsejet_fp_fuel_modeled_as_premixed_propane_air"
PULSEJET_FP_SCALED_GEOMETRY_FLAG = "pulsejet_fp_geometry_scaled_from_chamber_volume"


def _fp1_chamber_zone_volume_m3() -> float:
    """Chamber + cone volume of the FP-1 reference (cylinder + frustum)."""
    r_c = 0.5 * _FP1_CHAMBER_DIAMETER_M
    r_t = 0.5 * _FP1_TAILPIPE_DIAMETER_M
    cyl = math.pi * r_c * r_c * _FP1_CHAMBER_LENGTH_M
    frustum = (math.pi * _FP1_CONE_LENGTH_M / 3.0) * (r_c * r_c + r_c * r_t + r_t * r_t)
    return cyl + frustum


@dataclass(frozen=True)
class PulsejetFpSpec:
    """Hashable engine specification handed to pulsejet-fp (frozen so the
    query is memoizable, mirroring ``_run_pulsejet_simulation``)."""

    chamber_diameter_m: float
    chamber_length_m: float
    cone_length_m: float
    tailpipe_diameter_m: float
    tailpipe_length_m: float
    petal_scale: float
    """Isometric valve-petal scale factor. Every petal dimension (length,
    width, thickness, port area, lift stop, seat preload) scales by this,
    petal COUNT stays at FP-1's 9. Euler-Bernoulli similarity then gives
    f_n proportional to 1/s -- exactly the engine cycle-frequency scaling --
    plus scale-invariant cracking pressure and proportional lift, so the
    valve's validated dimensionless dynamics carry over unchanged. (The
    first integration attempt scaled COUNT at fixed petal size instead;
    the broken similarity -- relative f_n 7.4x vs the validated 2.6x,
    relatively-choked lift stop, relatively-short strain-quench zone --
    let the vehicle-scale engine start but decay to the dead state.)"""
    equivalence_ratio: float
    intake_scale: float
    n_petals: int = _FP1_N_PETALS
    bl_momentum_fraction: float = _FP1_BL_MOMENTUM_FRACTION
    geometry_derived_from_chamber_volume: bool = True


def derive_pulsejet_fp_spec(case: "ReferenceCase") -> PulsejetFpSpec:
    """Scale the validated FP-1 design to this case's chamber volume.

    Isometric scaling s = (V_case / V_FP1)^(1/3) on EVERY dimension -- duct,
    intake, and valve petals alike (see ``PulsejetFpSpec.petal_scale`` for
    why petal-isometric scaling is what preserves the validated valve
    dynamics; valve-area fraction is automatically preserved since both
    curtain area and head area scale as s^2). Equivalence ratio comes from
    the case's own pulsejet config, clipped to the premixed-model range.
    """

    scale = (case.pulsejet.chamber_volume_m3 / _fp1_chamber_zone_volume_m3()) ** (1.0 / 3.0)
    phi = min(max(case.pulsejet.target_equivalence_ratio, 0.6), 1.4)
    return PulsejetFpSpec(
        chamber_diameter_m=_FP1_CHAMBER_DIAMETER_M * scale,
        chamber_length_m=_FP1_CHAMBER_LENGTH_M * scale,
        cone_length_m=_FP1_CONE_LENGTH_M * scale,
        tailpipe_diameter_m=_FP1_TAILPIPE_DIAMETER_M * scale,
        tailpipe_length_m=_FP1_TAILPIPE_LENGTH_M * scale,
        petal_scale=scale,
        equivalence_ratio=phi,
        intake_scale=scale,
    )


# fidelity -> (n_cells, t_end cap seconds). The online convergence stop
# (pulsejet_fp stop_when_converged) means the cap only binds for
# quenched/non-converging points; converged points stop themselves.
_FP_NUMERICS_BY_FIDELITY: dict[str, tuple[int, float]] = {
    "fast": (150, 0.60),
    "full": (200, 0.90),
}


@lru_cache(maxsize=256)
def run_pulsejet_fp_query(
    spec: PulsejetFpSpec,
    mach: float,
    altitude_m: float,
    pulsejet_fidelity: str,
) -> "ThrustResult":
    """Run pulsejet-fp's transient sim to a converged limit cycle.

    Pure and deterministic given these arguments (the sibling model has no
    RNG and bans wall-clock reads), so memoization is safe -- same
    rationale as ``_run_pulsejet_simulation``. Lazy import keeps
    pulsejet-fp an optional dependency until the mode is actually used.
    """

    from pulsejet_fp import (IntakeDesign, Numerics, PetalValveDesign,
                             pulsejet_thrust, reference_gas,
                             reference_valve)
    from pulsejet_fp.geometry import EngineGeometry

    if pulsejet_fidelity not in _FP_NUMERICS_BY_FIDELITY:
        raise ValueError(f"unknown pulsejet fidelity: {pulsejet_fidelity!r}")
    n_cells, t_end = _FP_NUMERICS_BY_FIDELITY[pulsejet_fidelity]

    geom = EngineGeometry(
        chamber_diameter=spec.chamber_diameter_m,
        chamber_length=spec.chamber_length_m,
        cone_length=spec.cone_length_m,
        tailpipe_diameter=spec.tailpipe_diameter_m,
        tailpipe_length=spec.tailpipe_length_m,
    )
    ref_valve = reference_valve()
    ps = spec.petal_scale
    valve = PetalValveDesign(
        n_petals=spec.n_petals,
        petal_length=ref_valve.petal_length * ps,
        petal_width=ref_valve.petal_width * ps,
        petal_thickness=ref_valve.petal_thickness * ps,
        port_area=ref_valve.port_area * ps * ps,
        youngs_modulus=ref_valve.youngs_modulus,
        density=ref_valve.density,
        damping_ratio=ref_valve.damping_ratio,
        restitution=ref_valve.restitution,
        max_lift=ref_valve.max_lift * ps,
        seat_preload=ref_valve.seat_preload * ps,
    )
    intake = IntakeDesign(
        duct_length=0.060 * spec.intake_scale,
        duct_diameter=0.050 * spec.intake_scale,
        plenum_volume=5e-5 * spec.intake_scale ** 3,
        orientation="side",
        bl_momentum_fraction=spec.bl_momentum_fraction,
    )
    return pulsejet_thrust(
        mach=mach,
        altitude_m=altitude_m,
        gas=reference_gas(phi=spec.equivalence_ratio),
        geom=geom,
        valve=valve,
        intake=intake,
        numerics=Numerics(n_cells=n_cells),
        t_end=t_end,
        stop_when_converged=True,
    )


def pulsejet_fp_result_is_trustworthy(result: "ThrustResult") -> bool:
    """Structural guards before PULSEJET_MODE prefers a pulsejet-fp answer
    over the native simulator: a genuinely converged limit cycle with
    positive net thrust, combustion actually driving the oscillation
    (positive Rayleigh index), and both independent thrust formulations in
    agreement (<15% -- the momentum-closure identity normally closes <1%,
    so a wide miss signals a broken run, not roundoff)."""

    if result.status != "converged":
        return False
    if not (result.thrust_n > 0.0):
        return False
    if not (result.rayleigh_index > 0.0):
        return False
    if math.isfinite(result.thrust_surface_n) and abs(
        result.thrust_surface_n - result.thrust_n
    ) > 0.15 * max(abs(result.thrust_n), 1.0):
        return False
    return True


def pulsejet_fp_momentum_drag_n(
    result: "ThrustResult",
    mach: float,
    altitude_m: float,
    bl_momentum_fraction: float,
) -> float:
    """Cycle-mean inlet momentum drag the FP model already charged inside
    its net thrust (side inlet: swallowed mixture x k_bl x u_inf), so the
    map can report gross = net + drag with the same bookkeeping."""

    from pulsejet_fp.atmosphere import ambient

    p_a, T_a = ambient(altitude_m)
    a_a = math.sqrt(1.4 * 287.05 * T_a)
    mdot_mix = result.mdot_air_kg_s + result.mdot_fuel_kg_s
    if not math.isfinite(mdot_mix):
        return 0.0
    return mdot_mix * bl_momentum_fraction * mach * a_a
