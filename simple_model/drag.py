"""Closed-form vehicle drag: parasitic (frontal-area) + induced (span-efficiency).

Both pieces are single algebraic expressions, no iteration.
"""

from __future__ import annotations

from math import cos, pi, sqrt
from typing import NamedTuple

from .constants import (
    CD0_FRONTAL,
    CD0_WING,
    CL_MAX,
    OSWALD_EFFICIENCY,
    TRANSONIC_ONSET_MACH,
    TRANSONIC_PEAK_CD0_MULTIPLIER,
    TRANSONIC_PEAK_MACH,
    WING_ASPECT_RATIO,
)


def cd0_transonic_multiplier(mach: float) -> float:
    """Mach-dependent multiplier on the body CD0: flat subsonic, quadratic
    transonic rise to a peak at TRANSONIC_PEAK_MACH, then a decaying
    supersonic wave-drag tail ~ (M_peak/M)^2. Continuous at both joints,
    three float branches -- no lookup table, no iteration."""

    if mach <= TRANSONIC_ONSET_MACH:
        return 1.0
    rise = TRANSONIC_PEAK_CD0_MULTIPLIER - 1.0
    if mach < TRANSONIC_PEAK_MACH:
        s = (mach - TRANSONIC_ONSET_MACH) / (TRANSONIC_PEAK_MACH - TRANSONIC_ONSET_MACH)
        return 1.0 + rise * s * s
    return 1.0 + rise * (TRANSONIC_PEAK_MACH / mach) ** 2


class DragResult(NamedTuple):
    parasitic_n: float
    wing_parasitic_n: float
    induced_n: float
    total_n: float
    required_lift_n: float


def parasitic_drag_n(
    diameter_m: float,
    dynamic_pressure_pa: float,
    cd0: float = CD0_FRONTAL,
) -> float:
    """Zero-lift drag from frontal area alone: D0 = CD0 * q * (pi*D^2/4)."""

    frontal_area_m2 = pi * diameter_m**2 / 4.0
    return cd0 * dynamic_pressure_pa * frontal_area_m2


def wing_parasitic_drag_n(
    wingspan_m: float,
    dynamic_pressure_pa: float,
    aspect_ratio: float = WING_ASPECT_RATIO,
    cd0_wing: float = CD0_WING,
) -> float:
    """Zero-lift wing drag: D0_wing = CD0_wing * q * S, S = b^2/AR.

    Same backed-out reference area as stall_speed_m_per_s (see its docstring
    for why this model has no reference area of its own).
    """

    reference_area_m2 = wingspan_m**2 / aspect_ratio
    return cd0_wing * dynamic_pressure_pa * reference_area_m2


def induced_drag_n(
    required_lift_n: float,
    dynamic_pressure_pa: float,
    wingspan_m: float,
    oswald_efficiency: float = OSWALD_EFFICIENCY,
) -> float:
    """Finite-span induced drag: Di = L^2 / (q * pi * b^2 * e).

    Standard result for D_i = q*S*CL^2/(pi*AR*e) once CL = L/(q*S) and
    AR = b^2/S are substituted in -- the reference area S cancels, leaving
    only wingspan as the geometric input, per the user's own derivation.
    """

    if dynamic_pressure_pa <= 0.0:
        return 0.0
    return required_lift_n**2 / (dynamic_pressure_pa * pi * wingspan_m**2 * oswald_efficiency)


def total_drag_n(
    diameter_m: float,
    wingspan_m: float,
    velocity_m_per_s: float,
    air_density_kg_per_m3: float,
    mass_kg: float,
    flight_path_angle_rad: float,
    gravity_m_per_s2: float,
    cd0: float = CD0_FRONTAL,
    oswald_efficiency: float = OSWALD_EFFICIENCY,
    aspect_ratio: float = WING_ASPECT_RATIO,
    cd0_wing: float = CD0_WING,
    mach: float | None = None,
) -> DragResult:
    """Total drag at one flight state.

    Required lift assumes quasi-steady flight: lift balances the weight
    component perpendicular to the velocity vector (thrust taken as aligned
    with velocity), i.e. L = m*g*cos(flight_path_angle).

    `mach`, when given, applies the transonic/supersonic wave-drag rise to
    the body CD0 (cd0_transonic_multiplier). Wings are left un-multiplied
    (thin surfaces; body wave drag dominates for this layout).
    """

    dynamic_pressure_pa = 0.5 * air_density_kg_per_m3 * velocity_m_per_s**2
    required_lift_n = mass_kg * gravity_m_per_s2 * cos(flight_path_angle_rad)
    effective_cd0 = cd0 if mach is None else cd0 * cd0_transonic_multiplier(mach)
    parasitic_n = parasitic_drag_n(diameter_m, dynamic_pressure_pa, effective_cd0)
    wing_parasitic_n = wing_parasitic_drag_n(wingspan_m, dynamic_pressure_pa, aspect_ratio, cd0_wing)
    induced_n = induced_drag_n(required_lift_n, dynamic_pressure_pa, wingspan_m, oswald_efficiency)
    return DragResult(
        parasitic_n=parasitic_n,
        wing_parasitic_n=wing_parasitic_n,
        induced_n=induced_n,
        total_n=parasitic_n + wing_parasitic_n + induced_n,
        required_lift_n=required_lift_n,
    )


def stall_speed_m_per_s(
    wingspan_m: float,
    mass_kg: float,
    air_density_kg_per_m3: float,
    gravity_m_per_s2: float,
    aspect_ratio: float = WING_ASPECT_RATIO,
    cl_max: float = CL_MAX,
) -> float:
    """V at which level flight (L = W) requires exactly CL_max.

    The induced-drag formula above only ever needs wingspan (the reference
    area cancels out algebraically), which is elegant but means this model
    has no reference wing area of its own to define a stall speed with.
    One is backed out here from wingspan via an assumed aspect ratio
    (S = b^2 / AR) rather than adding a 7th sweep variable -- both
    WING_ASPECT_RATIO and CL_MAX are simple, tunable placeholders, not
    sourced values.
    """

    reference_area_m2 = wingspan_m**2 / aspect_ratio
    weight_n = mass_kg * gravity_m_per_s2
    return sqrt(2.0 * weight_n / (air_density_kg_per_m3 * reference_area_m2 * cl_max))
