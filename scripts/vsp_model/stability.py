"""Static and dynamic stability from VSPAERO output plus the mass model.

Two layers, and the distinction matters:

* STATIC margins come straight out of the solver. ``Cm_alpha``/``CL_alpha`` give a
  neutral point and a static margin; ``Cn_beta`` and ``Cl_beta`` give directional
  and lateral stiffness. These depend only on geometry and the moment reference.
* DYNAMIC modes need mass and inertia, which this project does not have -- see
  :mod:`mass_cg`, where every station is either derived from flowpath geometry or
  chosen outright. Short-period and dutch-roll numbers below are only as good as
  those station assignments, and the ballast station alone moves the CG by
  hundreds of millimetres.

Sign conventions are the usual ones: ``Cm_alpha < 0`` is pitch-stable,
``Cn_beta > 0`` directionally stable, ``Cl_beta < 0`` laterally stable.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt

from mass_cg import MassProperties
from vehicle_geometry import VehicleGeometry

# Sign to apply when a derivative is read off the force/moment history rather
# than the stability result.
#
# OpenVSP's body axes put x AFT, so its CMx (roll) and CMz (yaw) are the
# negatives of the standard aircraft-convention Cl and Cn; pitch is unaffected.
# Measured directly on this model: CMz_Beta = +3.6116 while CMn_Beta = -3.6116,
# and CMx_Beta = +0.3254 while CMl_Beta = -0.3254, with CMy_Alpha = CMm_Alpha =
# +6.7560 to the last digit. Without this correction the two lateral stability
# criteria come out with their signs flipped and an unstable vehicle reads as
# stable.
_AXIS_SIGN = {
    "CMxtot": -1.0,  # -> Cl, standard convention
    "CMytot": +1.0,  # -> Cm
    "CMztot": -1.0,  # -> Cn
    "CStot": +1.0,
}

# VSPAERO stability-result column names -> the derivative they represent.
# The solver names a derivative "<coefficient>_<perturbation>". CMl/CMm/CMn are
# already in the standard convention, so they are preferred over CMx/CMy/CMz.
_DERIVATIVE_ALIASES = {
    "CL_alpha": ("CL_Alpha", "CLtot_Alpha", "CFz_Alpha"),
    "CD_alpha": ("CD_Alpha", "CDtot_Alpha"),
    "Cm_alpha": ("CMm_Alpha", "CMy_Alpha"),
    "Cm_q": ("CMm_q", "CMy_q"),
    "CL_q": ("CL_q", "CFz_q"),
    "CY_beta": ("CS_Beta", "CFy_Beta"),
    # No CMz_/CMx_ fallbacks for the roll and yaw derivatives: those are OpenVSP
    # body-axis and carry the opposite sign (see _AXIS_SIGN). A missing value is
    # better than a confidently inverted stability verdict.
    "Cn_beta": ("CMn_Beta",),
    "Cl_beta": ("CMl_Beta",),
    "Cn_r": ("CMn_r",),
    "Cl_p": ("CMl_p",),
    "Cn_p": ("CMn_p",),
    "Cl_r": ("CMl_r",),
}


@dataclass(frozen=True)
class StabilityDerivatives:
    """Derivatives at one flight condition, per radian."""

    mach: float
    alpha_deg: float
    source: str
    values: dict[str, float]

    def get(self, name: str) -> float | None:
        return self.values.get(name)


@dataclass(frozen=True)
class StaticStability:
    mach: float
    cl_alpha: float
    cm_alpha: float
    neutral_point_x_m: float
    static_margin: float
    """Fraction of the reference chord. Positive is stable."""
    cn_beta: float | None
    cl_beta: float | None
    cy_beta: float | None
    cg_x_m: float
    reference_chord_m: float

    @property
    def pitch_stable(self) -> bool:
        return self.cm_alpha < 0.0

    @property
    def directionally_stable(self) -> bool:
        return self.cn_beta is not None and self.cn_beta > 0.0

    @property
    def laterally_stable(self) -> bool:
        return self.cl_beta is not None and self.cl_beta < 0.0


@dataclass(frozen=True)
class DynamicModes:
    mach: float
    altitude_m: float
    true_airspeed_m_s: float
    dynamic_pressure_pa: float
    short_period_rad_s: float | None
    short_period_damping: float | None
    dutch_roll_rad_s: float | None
    dutch_roll_damping: float | None
    roll_time_constant_s: float | None
    caveats: tuple[str, ...]


def _finite_difference(x_values: list[float], y_values: list[float]) -> float:
    """Least-squares slope. Uses every point rather than just the endpoints, so
    one noisy solve does not set the derivative on its own."""

    n = len(x_values)
    if n < 2:
        raise ValueError("need at least two points to take a derivative")
    mean_x = sum(x_values) / n
    mean_y = sum(y_values) / n
    denominator = sum((x - mean_x) ** 2 for x in x_values)
    if denominator <= 0.0:
        raise ValueError("derivative points are all at the same abscissa")
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(x_values, y_values)) / denominator


def derivatives_from_points(
    points, mach: float, *, alpha_window_deg: float = 6.0
) -> StabilityDerivatives:
    """Build derivatives by differencing a plain (non-stability) sweep.

    This is the fallback when a stability-mode run is not available, and it is
    also the cross-check on one: the same ``Cm_alpha`` computed two ways.
    Restricted to a small alpha window so the slope is the linear-range one and
    not contaminated by the low-aspect-ratio wing's nonlinear lift further out.
    """

    at_mach = [p for p in points if abs(p.mach - mach) < 1e-9]
    if not at_mach:
        raise ValueError(f"no points at Mach {mach}")

    pitch = sorted(
        (p for p in at_mach if abs(p.beta_deg) < 1e-9 and abs(p.alpha_deg) <= alpha_window_deg),
        key=lambda p: p.alpha_deg,
    )
    values: dict[str, float] = {}
    if len(pitch) >= 2:
        alpha_rad = [p.alpha_deg * pi / 180.0 for p in pitch]
        for key, field, sign in (
            ("CL_alpha", "CLtot", 1.0),
            ("CD_alpha", "CDtot", 1.0),
            ("Cm_alpha", "CMytot", _AXIS_SIGN["CMytot"]),
        ):
            series = [p.coefficients.get(field) for p in pitch]
            if all(v is not None for v in series):
                values[key] = sign * _finite_difference(alpha_rad, series)

    # Sideslip: take the alpha closest to zero that has more than one beta.
    by_alpha: dict[float, list] = {}
    for p in at_mach:
        by_alpha.setdefault(p.alpha_deg, []).append(p)
    sideslip = sorted(
        (group for group in by_alpha.values() if len({p.beta_deg for p in group}) >= 2),
        key=lambda group: abs(group[0].alpha_deg),
    )
    if sideslip:
        group = sorted(sideslip[0], key=lambda p: p.beta_deg)
        beta_rad = [p.beta_deg * pi / 180.0 for p in group]
        for key, field in (
            ("CY_beta", "CStot"),
            ("Cn_beta", "CMztot"),
            ("Cl_beta", "CMxtot"),
        ):
            series = [p.coefficients.get(field) for p in group]
            if all(v is not None for v in series):
                values[key] = _AXIS_SIGN[field] * _finite_difference(beta_rad, series)

    return StabilityDerivatives(
        mach=mach, alpha_deg=0.0, source="finite difference of sweep points", values=values
    )


def derivatives_from_stability_run(point) -> StabilityDerivatives:
    """Map a stability-mode point's raw derivative columns onto known names."""

    raw = point.derivatives
    values: dict[str, float] = {}
    for canonical, aliases in _DERIVATIVE_ALIASES.items():
        for alias in aliases:
            if alias in raw:
                values[canonical] = raw[alias]
                break
    return StabilityDerivatives(
        mach=point.mach,
        alpha_deg=point.alpha_deg,
        source="VSPAERO stability mode",
        values=values,
    )


def static_stability(
    derivatives: StabilityDerivatives, geometry: VehicleGeometry
) -> StaticStability:
    """Neutral point and static margin.

    ``x_np = x_cg - (Cm_alpha / CL_alpha) * cref`` is the definition: the station
    at which the moment derivative would vanish. Static margin is that offset in
    chords, positive aft of the CG.
    """

    cl_alpha = derivatives.get("CL_alpha")
    cm_alpha = derivatives.get("Cm_alpha")
    if cl_alpha is None or cm_alpha is None:
        raise ValueError(
            "CL_alpha and Cm_alpha are both required for a static margin; got "
            f"{sorted(derivatives.values)}"
        )
    if abs(cl_alpha) < 1e-9:
        raise ValueError("CL_alpha is ~0; the neutral point is undefined")

    references = geometry.references
    static_margin = -cm_alpha / cl_alpha
    return StaticStability(
        mach=derivatives.mach,
        cl_alpha=cl_alpha,
        cm_alpha=cm_alpha,
        neutral_point_x_m=references.cg_x_m + static_margin * references.chord_m,
        static_margin=static_margin,
        cn_beta=derivatives.get("Cn_beta"),
        cl_beta=derivatives.get("Cl_beta"),
        cy_beta=derivatives.get("CY_beta"),
        cg_x_m=references.cg_x_m,
        reference_chord_m=references.chord_m,
    )


def _atmosphere(altitude_m: float) -> tuple[float, float]:
    """ISA density and speed of sound, troposphere only."""

    temperature = 288.15 - 0.0065 * altitude_m
    pressure = 101325.0 * (temperature / 288.15) ** 5.25588
    density = pressure / (287.053 * temperature)
    return density, sqrt(1.4 * 287.053 * temperature)


def dynamic_modes(
    derivatives: StabilityDerivatives,
    geometry: VehicleGeometry,
    mass: MassProperties,
    altitude_m: float,
) -> DynamicModes:
    """Short-period, dutch-roll and roll-subsidence estimates.

    Standard decoupled approximations, not a full 6-DOF eigenvalue solve: with
    inertias built on assigned stations, the extra precision of a full solve
    would be false precision. What these ARE good for is telling a stable
    configuration from an unstable one and showing which way a change moves it.
    """

    density, speed_of_sound = _atmosphere(altitude_m)
    velocity = derivatives.mach * speed_of_sound
    q_bar = 0.5 * density * velocity**2

    references = geometry.references
    s_ref, c_ref, b_ref = references.area_m2, references.chord_m, references.span_m
    caveats: list[str] = [
        "Inertias come from assigned mass stations; the project has no CG model.",
        "Decoupled modal approximations, not a 6-DOF eigenvalue solve.",
    ]

    cl_alpha = derivatives.get("CL_alpha")
    cm_alpha = derivatives.get("Cm_alpha")
    cm_q = derivatives.get("Cm_q")

    short_period = damping_sp = None
    if cl_alpha is not None and cm_alpha is not None and velocity > 1.0:
        z_alpha = -q_bar * s_ref * cl_alpha / mass.mass_kg
        m_alpha = q_bar * s_ref * c_ref * cm_alpha / mass.i_yy_kg_m2
        m_q = (
            q_bar * s_ref * c_ref**2 * cm_q / (2.0 * velocity * mass.i_yy_kg_m2)
            if cm_q is not None
            else 0.0
        )
        if cm_q is None:
            caveats.append("Cm_q unavailable: short-period damping excludes pitch rate.")
        omega_squared = z_alpha * m_q / velocity - m_alpha
        if omega_squared > 0.0:
            short_period = sqrt(omega_squared)
            damping_sp = -(m_q + z_alpha / velocity) / (2.0 * short_period)
        else:
            caveats.append(
                "Short-period frequency is imaginary: the pitch mode is divergent, "
                "not oscillatory (statically unstable at this condition)."
            )

    cn_beta = derivatives.get("Cn_beta")
    cn_r = derivatives.get("Cn_r")
    cy_beta = derivatives.get("CY_beta")
    dutch_roll = damping_dr = None
    if cn_beta is not None and velocity > 1.0:
        omega_squared = q_bar * s_ref * b_ref * cn_beta / mass.i_zz_kg_m2
        if omega_squared > 0.0:
            dutch_roll = sqrt(omega_squared)
            n_r = (
                q_bar * s_ref * b_ref**2 * cn_r / (2.0 * velocity * mass.i_zz_kg_m2)
                if cn_r is not None
                else 0.0
            )
            y_beta = (
                q_bar * s_ref * cy_beta / (mass.mass_kg * velocity)
                if cy_beta is not None
                else 0.0
            )
            damping_dr = -(n_r + y_beta) / (2.0 * dutch_roll)
            if cn_r is None:
                caveats.append("Cn_r unavailable: dutch-roll damping excludes yaw rate.")
        else:
            caveats.append(
                "Cn_beta <= 0: the vehicle is directionally UNSTABLE, so there is "
                "no dutch-roll oscillation to report."
            )

    cl_p = derivatives.get("Cl_p")
    roll_tau = None
    if cl_p is not None and cl_p < 0.0 and velocity > 1.0:
        roll_tau = (
            -2.0 * velocity * mass.i_xx_kg_m2 / (q_bar * s_ref * b_ref**2 * cl_p)
        )

    return DynamicModes(
        mach=derivatives.mach,
        altitude_m=altitude_m,
        true_airspeed_m_s=velocity,
        dynamic_pressure_pa=q_bar,
        short_period_rad_s=short_period,
        short_period_damping=damping_sp,
        dutch_roll_rad_s=dutch_roll,
        dutch_roll_damping=damping_dr,
        roll_time_constant_s=roll_tau,
        caveats=tuple(caveats),
    )


def describe_static(static: StaticStability) -> str:
    def verdict(ok: bool) -> str:
        return "STABLE" if ok else "UNSTABLE"

    lines = [f"Static stability at Mach {static.mach:.2f}", ""]
    lines.append(f"  CL_alpha        {static.cl_alpha:+8.4f} /rad")
    lines.append(
        f"  Cm_alpha        {static.cm_alpha:+8.4f} /rad   {verdict(static.pitch_stable)} in pitch"
    )
    lines.append(f"  neutral point   {static.neutral_point_x_m * 1e3:8.1f} mm from nose")
    lines.append(f"  CG              {static.cg_x_m * 1e3:8.1f} mm from nose")
    lines.append(
        f"  static margin   {static.static_margin:+8.3f} cref "
        f"({static.static_margin * static.reference_chord_m * 1e3:+.1f} mm)"
    )
    if static.cn_beta is not None:
        lines.append(
            f"  Cn_beta         {static.cn_beta:+8.4f} /rad   "
            f"{verdict(static.directionally_stable)} directionally"
        )
    if static.cl_beta is not None:
        lines.append(
            f"  Cl_beta         {static.cl_beta:+8.4f} /rad   "
            f"{verdict(static.laterally_stable)} laterally"
        )
    return "\n".join(lines)


def describe_dynamic(modes: DynamicModes) -> str:
    lines = [
        f"Dynamic modes at Mach {modes.mach:.2f}, {modes.altitude_m:.0f} m "
        f"(V={modes.true_airspeed_m_s:.1f} m/s, q={modes.dynamic_pressure_pa:.0f} Pa)",
        "",
    ]

    def row(label: str, freq: float | None, damping: float | None) -> str:
        if freq is None:
            return f"  {label:<16} not computable"
        period = 2.0 * pi / freq
        zeta = f"{damping:+.3f}" if damping is not None else "  n/a"
        return (
            f"  {label:<16} wn={freq:6.2f} rad/s  T={period:5.2f} s  zeta={zeta}"
        )

    lines.append(row("short period", modes.short_period_rad_s, modes.short_period_damping))
    lines.append(row("dutch roll", modes.dutch_roll_rad_s, modes.dutch_roll_damping))
    if modes.roll_time_constant_s is not None:
        lines.append(f"  {'roll subsidence':<16} tau={modes.roll_time_constant_s:.3f} s")
    else:
        lines.append(f"  {'roll subsidence':<16} not computable")
    if modes.caveats:
        lines.append("")
        for caveat in modes.caveats:
            lines.append(f"  ! {caveat}")
    return "\n".join(lines)
