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


# --- Wing-concept model (2026-08-12) ---------------------------------------
# Everything below stays single-expression closed-form. A WingConcept
# generalizes the four fixed wing constants; wing_efficiency/wing drag/stall
# all reduce EXACTLY to the legacy constants for DEFAULT_WING_CONCEPT
# (rectangular AR=3 unswept, cl_max 1.0, cd0 0.02 -> e 0.80), verified in
# tests/test_simple_model.py.

from math import cos as _cos, radians as _radians

from .constants import (
    AIRFOILS,
    Airfoil,
    OSWALD_BASE_E,
    OSWALD_TAPER_MIN_DELTA_AT,
    OSWALD_TAPER_PENALTY,
    WING_CRITICAL_NORMAL_MACH,
    WING_WAVE_K,
    WING_WAVE_ONSET_WIDTH,
)


class WingConcept(NamedTuple):
    span_m: float
    aspect_ratio: float
    taper_ratio: float      # tip/root chord, 1.0 = rectangular
    sweep_deg: float        # leading-edge sweep
    airfoil: Airfoil

    @property
    def reference_area_m2(self) -> float:
        return self.span_m ** 2 / self.aspect_ratio

    @property
    def oswald_e(self) -> float:
        """Lifting-line-flavored span efficiency: best near taper ~0.35,
        quadratic penalty away from it (see constants.py anchors)."""
        return OSWALD_BASE_E - OSWALD_TAPER_PENALTY * (
            self.taper_ratio - OSWALD_TAPER_MIN_DELTA_AT
        ) ** 2

    @property
    def cl_max_effective(self) -> float:
        """Sweep decimates usable lift: simple-sweep-theory cos^2 on the
        normal-component dynamic pressure."""
        return self.airfoil.cl_max * _cos(_radians(self.sweep_deg)) ** 2


DEFAULT_WING_CONCEPT_FOR = None  # sentinel docs; use default_wing_concept()


def default_wing_concept(span_m: float) -> WingConcept:
    """The legacy fixed wing as a concept: rectangular, unswept, AR=3,
    'naca_symmetric' (cl_max 1.0, cd0 0.02). oswald_e comes out 0.80."""
    return WingConcept(span_m, WING_ASPECT_RATIO, 1.0, 0.0,
                       AIRFOILS["naca_symmetric"])


def wing_wave_drag_coefficient(concept: WingConcept, mach: float) -> float:
    """Supersonic thin-wing wave drag on the wing reference area:
    K (t/c)^2 / sqrt(Mn^2 - 1), onset blended over WING_WAVE_ONSET_WIDTH in
    leading-edge-normal Mach Mn = M cos(sweep). Zero below onset."""
    m_normal = mach * _cos(_radians(concept.sweep_deg))
    onset = WING_CRITICAL_NORMAL_MACH
    if m_normal <= onset:
        return 0.0
    tc = concept.airfoil.thickness_ratio
    # Linear supersonic thin-wing theory diverges as Mn -> 1+ (the
    # 1/sqrt(Mn^2 - 1) singularity); real transonic wave drag stays finite,
    # so the factor is capped at its Mn ~= 1.17 value (argument floor 0.36)
    # -- the standard regularization. Quadratic ramp from onset up to that
    # capped peak, then the capped supersonic tail. (An earlier version
    # kept a 1e-6 floor here: cd_wave ~ 3.6 at exactly Mach 1 -- a ~60 kN
    # phantom drag wall that silently made every transonic mission
    # infeasible. Pinned by test_simple_model.)
    peak_factor = 1.0 / sqrt(0.36)
    if m_normal < onset + WING_WAVE_ONSET_WIDTH:
        s = (m_normal - onset) / WING_WAVE_ONSET_WIDTH
        return WING_WAVE_K * tc * tc * peak_factor * s * s
    return WING_WAVE_K * tc * tc / sqrt(max(m_normal * m_normal - 1.0, 0.36))


def wing_concept_drag_n(
    concept: WingConcept,
    dynamic_pressure_pa: float,
    required_lift_n: float,
    mach: float | None = None,
) -> tuple[float, float]:
    """(parasitic+wave, induced) drag of a WingConcept. Same algebra as the
    legacy functions with the concept's own area/e/profile numbers."""
    area = concept.reference_area_m2
    cd0 = concept.airfoil.cd0_wing
    if mach is not None:
        cd0 = cd0 + wing_wave_drag_coefficient(concept, mach)
    parasitic = cd0 * dynamic_pressure_pa * area
    if dynamic_pressure_pa <= 0.0:
        return parasitic, 0.0
    induced = required_lift_n ** 2 / (
        dynamic_pressure_pa * pi * concept.span_m ** 2 * concept.oswald_e
    )
    return parasitic, induced


def stall_speed_concept_m_per_s(
    concept: WingConcept,
    mass_kg: float,
    air_density_kg_per_m3: float,
    gravity_m_per_s2: float,
) -> float:
    """Stall speed with the concept's own area and sweep-effective CL_max."""
    weight_n = mass_kg * gravity_m_per_s2
    return sqrt(
        2.0 * weight_n
        / (air_density_kg_per_m3 * concept.reference_area_m2 * concept.cl_max_effective)
    )
