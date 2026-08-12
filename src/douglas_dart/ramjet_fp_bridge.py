"""Bridge to ramjet-fp: the first-principles unsteady quasi-1D ramjet model
(sibling repo, installed editable as ``ramjet_fp``) that serves as the
PRIMARY ramjet query source for the propulsion map (Gate 2) and feeds the
mission solver's ramjet phase through a lazy interpolation table (Gate 3).

Why this model is primary (vs. the native steady 0D ``ramjet.py``): it
derives everything the native model configures -- inlet recovery from the
Rankine-Hugoniot jump conditions instead of the MIL-E-5008B schedule,
combustion completeness from resolved flame physics instead of a configured
``combustor_efficiency``, choking from the conservation laws instead of a
discharge coefficient -- and it resolves the two behaviors the 0D model
cannot represent at all: flameholder blow-off (the Damkohler/strain
stability limit, so "the ramjet is lit" becomes a computed fact rather than
a Mach threshold) and the large-amplitude combustion oscillation this
vehicle's deeply-subcritical shared-nozzle configuration produces at every
lit operating point. Its verification record lives in the sibling repo:
``ramjet-fp/architecture.md`` and ``docs/derivation.md`` (exact-solution
battery computed in-test from first principles, machine-precision
conservation, dual independent thrust formulations closing to 0.02-5%,
grid study +-1%, end-to-end JIT equivalence).

Findings the consumer MUST know (ramjet-fp architecture.md #4-#8):

- At the vehicle configs' ramjet ``target_equivalence_ratio`` 0.60 the
  fully-premixed flame CANNOT hold at any Mach -- the model reports
  ``blown_off`` with cold-throughflow drag. Near-stoichiometric fueling
  (phi ~ 1.0) is required, and the lean limit tightens with Mach and
  altitude (phi_min 0.87 at M 0.4 SL -> ~1.0 in the M 0.6-0.9 pinch).
- Every lit point is a bounded chugging limit cycle (amplitude 0.7-1.5x
  the mean, 500-1200 Hz). Cycle-MEAN thrust/fuel flow are reported; the
  amplitude is carried as a validity flag, not hidden.
- Cold relight has an altitude-dependent no-go hole (M 0.7-0.8 at 1500 m
  widening to M 0.8-1.1 at 6000 m; none at sea level). A continuously
  carried flame transits the band at <=~1000 m and survives climb at
  M >= 1.1 -- the mission-profile implication Gate 3 now sees.

Geometry mapping is DIRECT (unlike the pulsejet bridge's isometric
scaling): the case config carries the real flowpath -- intake diameter,
body-limited combustor, shared nozzle throat and exit ratio, phi. Only the
flameholder is scaled from the validated RJ-1 proportions (the vehicle has
no configured gutter yet), capped so the gutter is never the choke point
ahead of the shared nozzle throat; both derivations carry visible flags.

Known, deliberately-flagged fidelity limits (sibling assumption registry):
adiabatic walls (A6) make thrust/TSFC and flame stability optimistic;
fully-premixed uniform charge (A13) makes lean blow-off conservative vs a
piloted/stratified injector; the achieved total-pressure recovery is an
OUTPUT diagnostic, so scenario recovery overrides do not apply to
FP-sourced points (flagged).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from functools import lru_cache
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # lazy, like the pulsejet-fp bridge
    from ramjet_fp import RamjetResult
    from .config import ReferenceCase


def ramjet_fp_primary_enabled() -> bool:
    """RAMJET_MODE tries ramjet-fp first unless this kill-switch is set.

    ``DOUGLAS_DART_DISABLE_RAMJET_FP=1`` restores the native steady-0D
    dispatch. Intended for the legacy test suite and for triaging a
    suspected sibling-model regression -- NOT a fidelity tier. Each FP
    point costs ~20-120 s wall (memoized per unique (spec, mach, altitude,
    fidelity)); high-volume inner search loops that cannot afford the
    lazy-table build (~12-16 points per unique geometry) should set this
    switch explicitly and visibly rather than silently receiving 0D
    physics -- the same deliberate price as the pulsejet-fp primary."""

    return os.environ.get("DOUGLAS_DART_DISABLE_RAMJET_FP", "") not in ("1", "true", "yes")


# ---------------------------------------------------------------------------
# RJ-1 flameholder proportions (must match ramjet_fp.query.reference_*;
# asserted against the live package in tests/test_ramjet_fp_bridge.py so
# sibling drift is caught, not silently absorbed).
# ---------------------------------------------------------------------------
_RJ1_COMBUSTOR_DIAMETER_M = 0.190
_RJ1_GUTTER_RADIUS_FRACTION = 0.060 / 0.190      # r_g / D_c
_RJ1_GUTTER_WIDTH_FRACTION = 0.025 / 0.190       # w_g / D_c
_RJ1_BODY_LENGTH_M = 2.30                        # candidate-A body the RJ-1
_RJ1_X_COMBUSTOR_START = 0.30                    # axial stations scale with
_RJ1_X_FLAMEHOLDER = 0.45                        # body_length / 2.30
_RJ1_X_COMBUSTOR_END = 1.30
_RJ1_X_THROAT = 1.55
_RJ1_X_EXIT = 1.62

# gutter must never be the choke point ahead of the shared nozzle throat
_GUTTER_MIN_AREA_MARGIN_OVER_THROAT = 1.15
_COMBUSTOR_WALL_ALLOWANCE_M = 0.015

RAMJET_FP_ADIABATIC_OPTIMISM_FLAG = (
    "ramjet_fp_adiabatic_walls_thrust_and_stability_optimistic_a6"
)
RAMJET_FP_PREMIXED_CONSERVATIVE_FLAG = (
    "ramjet_fp_fully_premixed_lean_blowoff_conservative_a13_a27"
)
RAMJET_FP_RECOVERY_OUTPUT_FLAG = "ramjet_fp_recovery_is_derived_output_not_input"
RAMJET_FP_FLAMEHOLDER_SCALED_FLAG = "ramjet_fp_flameholder_scaled_from_rj1_proportions"
RAMJET_FP_GUTTER_CAPPED_FLAG = "ramjet_fp_gutter_capped_by_nozzle_throat_margin"
RAMJET_FP_OSCILLATORY_FLAG = "ramjet_fp_large_combustion_oscillation_cycle_mean_reported"
RAMJET_FP_BLOWN_OFF_FLAG = "ramjet_fp_flame_blown_off_cold_throughflow_drag"
RAMJET_FP_FALLBACK_FLAG = "ramjet_fp_primary_rejected_fell_back_to_native"
RAMJET_FP_RECOVERY_OVERRIDE_NA_FLAG = (
    "ramjet_fp_scenario_recovery_override_not_applicable"
)


@dataclass(frozen=True)
class RamjetFpSpec:
    """Hashable engine specification handed to ramjet-fp (frozen so the
    query is memoizable, mirroring the pulsejet bridge)."""

    lip_diameter_m: float
    combustor_diameter_m: float
    throat_diameter_m: float
    exit_area_ratio: float
    x_combustor_start_m: float
    x_flameholder_m: float
    x_combustor_end_m: float
    x_throat_m: float
    x_exit_m: float
    gutter_radius_m: float
    gutter_width_m: float
    equivalence_ratio: float
    gutter_capped_by_throat: bool = False

    @property
    def gutter_frontal_area_m2(self) -> float:
        return 2.0 * math.pi * self.gutter_radius_m * self.gutter_width_m

    @property
    def gutter_shear_perimeter_m(self) -> float:
        return 4.0 * math.pi * self.gutter_radius_m


def derive_ramjet_fp_spec(case: "ReferenceCase") -> RamjetFpSpec:
    """Map the case's real flowpath dimensions onto a ramjet-fp engine.

    Direct where the config carries the dimension (intake, nozzle, phi);
    derived-with-flags where it does not: combustor diameter = body
    diameter minus a wall allowance (floored above the throat so the
    nozzle stays convergent), axial stations scaled with body length from
    the RJ-1 layout, and the flameholder scaled from RJ-1's proportions
    but capped so the gutter's minimum flow area keeps a
    >=15% margin over the shared nozzle throat (a gutter that chokes ahead
    of the throat would be a different engine, not a bigger flameholder).
    """

    lip = case.selector.circular_intake_diameter_m
    throat = case.nozzle.throat_diameter_m
    combustor = max(case.vehicle.body_diameter_m - _COMBUSTOR_WALL_ALLOWANCE_M,
                    1.10 * throat)

    s_x = case.vehicle.body_length_m / _RJ1_BODY_LENGTH_M
    a_c = 0.25 * math.pi * combustor * combustor
    a_th = 0.25 * math.pi * throat * throat

    r_g = _RJ1_GUTTER_RADIUS_FRACTION * combustor
    w_g = _RJ1_GUTTER_WIDTH_FRACTION * combustor
    # cap: A_c - A_gutter >= margin * A_throat
    a_g_max = a_c - _GUTTER_MIN_AREA_MARGIN_OVER_THROAT * a_th
    capped = False
    if 2.0 * math.pi * r_g * w_g > a_g_max:
        capped = True
        w_g = max(a_g_max / (2.0 * math.pi * r_g), 1e-3)

    phi = min(max(case.ramjet.target_equivalence_ratio, 0.3), 1.4)
    return RamjetFpSpec(
        lip_diameter_m=lip,
        combustor_diameter_m=combustor,
        throat_diameter_m=throat,
        exit_area_ratio=case.nozzle.exit_to_throat_area_ratio,
        x_combustor_start_m=_RJ1_X_COMBUSTOR_START * s_x,
        x_flameholder_m=_RJ1_X_FLAMEHOLDER * s_x,
        x_combustor_end_m=_RJ1_X_COMBUSTOR_END * s_x,
        x_throat_m=_RJ1_X_THROAT * s_x,
        x_exit_m=_RJ1_X_EXIT * s_x,
        gutter_radius_m=r_g,
        gutter_width_m=w_g,
        equivalence_ratio=phi,
        gutter_capped_by_throat=capped,
    )


# fidelity -> (n_cells, t_end cap seconds); the steady/oscillatory early
# stop means the cap binds only for slow-converging points.
_FP_NUMERICS_BY_FIDELITY: dict[str, tuple[int, float]] = {
    "fast": (162, 0.30),
    "full": (324, 0.40),
}


@lru_cache(maxsize=512)
def run_ramjet_fp_query(
    spec: RamjetFpSpec,
    mach: float,
    altitude_m: float,
    ramjet_fidelity: str,
) -> "RamjetResult":
    """Run ramjet-fp's transient sim to its attractor (steady, oscillatory
    limit cycle, or blown off -- all honest, reportable outcomes).

    Pure and deterministic given these arguments (the sibling model has no
    RNG and bans wall-clock reads), so memoization is safe. Lazy import
    keeps ramjet-fp an optional dependency until the mode is used."""

    from ramjet_fp import (FlameholderDesign, Numerics, RamjetGeometry,
                           ramjet_thrust, reference_gas)

    if ramjet_fidelity not in _FP_NUMERICS_BY_FIDELITY:
        raise ValueError(f"unknown ramjet fidelity: {ramjet_fidelity!r}")
    n_cells, t_end = _FP_NUMERICS_BY_FIDELITY[ramjet_fidelity]

    geom = RamjetGeometry(
        lip_diameter=spec.lip_diameter_m,
        combustor_diameter=spec.combustor_diameter_m,
        throat_diameter=spec.throat_diameter_m,
        exit_area_ratio=spec.exit_area_ratio,
        x_combustor_start=spec.x_combustor_start_m,
        x_combustor_end=spec.x_combustor_end_m,
        x_throat=spec.x_throat_m,
        x_exit=spec.x_exit_m,
    )
    fh = FlameholderDesign(
        x_fh=spec.x_flameholder_m,
        gutter_width=spec.gutter_width_m,
        frontal_area=spec.gutter_frontal_area_m2,
        shear_perimeter=spec.gutter_shear_perimeter_m,
    )
    return ramjet_thrust(
        mach,
        altitude_m,
        gas=reference_gas(spec.equivalence_ratio),
        geom=geom,
        fh=fh,
        numerics=Numerics(n_cells=n_cells),
        t_end=t_end,
    )


def ramjet_fp_result_is_usable(result: "RamjetResult") -> bool:
    """Structural guards before RAMJET_MODE prefers a ramjet-fp answer.

    ``blown_off`` and ``no_flow`` ARE usable physics (a flame that will
    not hold, reported with the cold-throughflow drag) -- only a run that
    failed to classify (``unconverged``), produced non-finite numbers, or
    whose two independent thrust formulations disagree wildly (the eq.
    30/32 identity closes to 0.0-0.1% steady and <5% cycle-mean in the
    validation record, so a >15% miss signals a broken run) is rejected."""

    if result.status not in ("steady", "oscillatory", "blown_off", "no_flow"):
        return False
    if not math.isfinite(result.net_thrust_n):
        return False
    if not math.isfinite(result.mdot_fuel_kg_s) and result.status != "no_flow":
        return False
    if math.isfinite(result.thrust_surface_n):
        scale = max(abs(result.net_thrust_n), 5.0)
        if abs(result.thrust_surface_n - result.net_thrust_n) > 0.15 * scale:
            return False
    return True


# ---------------------------------------------------------------------------
# Gate 3: lazy bilinear mission table
# ---------------------------------------------------------------------------

_TABLE_MACH_STEP = 0.1
_TABLE_ALT_STEP_M = 1500.0


class RamjetFpMissionTable:
    """Lazy (Mach x altitude) interpolation table over the memoized query.

    The mission solver queries thrust at continuously-varying (M, alt)
    every step; a raw FP call per step is unaffordable, so this snaps to a
    0.1-Mach x 1500-m grid and computes only the corner cells the
    trajectory actually touches (~12-16 FP runs per mission, memoized
    across scenarios and repeat calls). Thrust and fuel flow are bilinear
    cycle means; ``flame_ok`` is the corner-weighted flame fraction >= 0.5
    (the blow-off boundary is sharp, so the transition band spans at most
    one cell). Same spirit as trajectory.py's pulsejet static table."""

    def __init__(self, spec: RamjetFpSpec, fidelity: str):
        self.spec = spec
        self.fidelity = fidelity

    @lru_cache(maxsize=4096)
    def _corner(self, mach_node: float, alt_node: float):
        res = run_ramjet_fp_query(self.spec, mach_node, alt_node,
                                  self.fidelity)
        flame = 1.0 if (res.flame_stable
                        and ramjet_fp_result_is_usable(res)) else 0.0
        fuel = res.mdot_fuel_kg_s if math.isfinite(res.mdot_fuel_kg_s) else 0.0
        return res.net_thrust_n, fuel, flame

    def query(self, mach: float, altitude_m: float) -> tuple[float, float, bool]:
        """(net_thrust_n, fuel_flow_kg_per_s, flame_ok), cycle means."""
        m0 = math.floor(mach / _TABLE_MACH_STEP) * _TABLE_MACH_STEP
        m0 = max(round(m0, 6), 0.0)
        m1 = round(m0 + _TABLE_MACH_STEP, 6)
        a0 = max(math.floor(altitude_m / _TABLE_ALT_STEP_M) * _TABLE_ALT_STEP_M, 0.0)
        a1 = a0 + _TABLE_ALT_STEP_M
        tm = (mach - m0) / _TABLE_MACH_STEP
        ta = (altitude_m - a0) / _TABLE_ALT_STEP_M
        tm = min(max(tm, 0.0), 1.0)
        ta = min(max(ta, 0.0), 1.0)

        c00 = self._corner(m0, a0)
        c10 = self._corner(m1, a0)
        c01 = self._corner(m0, a1)
        c11 = self._corner(m1, a1)
        out = []
        for k in range(3):
            v0 = c00[k] * (1 - tm) + c10[k] * tm
            v1 = c01[k] * (1 - tm) + c11[k] * tm
            out.append(v0 * (1 - ta) + v1 * ta)
        return out[0], out[1], out[2] >= 0.5


@lru_cache(maxsize=16)
def get_ramjet_fp_mission_table(spec: RamjetFpSpec,
                                fidelity: str) -> RamjetFpMissionTable:
    """One shared lazy table per (engine spec, fidelity) per process."""

    return RamjetFpMissionTable(spec, fidelity)
