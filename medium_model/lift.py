"""Finite-wing lift: lift-curve slope, CLmax vs Mach, Oswald factor, stall.

medium_model's step-2 fidelity upgrade, the lift-side companion to
``drag_buildup.py``.  It exists because induced drag is the *binding*
term on this vehicle -- ~69 N of the ~71 N total at the 40 m/s release
condition -- and induced drag is set entirely by lift fidelity:

    D_i = L^2 / (q * pi * b^2 * e)

The ancestor model supplies ``e`` from a single quadratic in taper ratio
(``drag.WingConcept.oswald_e``, e = 0.85 - 0.118 (lambda-0.35)^2, no
aspect-ratio, no Mach, no leading-edge-suction physics) and ``CL_max``
from one fixed airfoil constant times cos^2(sweep).  Neither carries the
two effects that actually decide this airframe:

  * **Aspect ratio 1.86.**  At that AR the full-leading-edge-suction
    induced drag (K = 1/(pi AR e_theo) = 0.172) and the zero-suction
    limit (K = 1/CL_alpha = 0.407) differ by 2.4x.  Which one the wing
    is near is worth more than every other term in this module put
    together.
  * **Maximum lift is alpha-limited, not stall-limited.**  A low-AR wing
    does not reach its section CL_max; it runs out of usable angle of
    attack first.  Treating cl_max_2D * cos^2(sweep) as attainable
    overstates CL_max by ~35 % here.

EMPIRICISM POLICY (identical to drag_buildup.py, and deliberately
different from the FP engine repos): vehicle aerodynamics cannot be
derived end-to-end without CFD, so this module uses standard engineering
correlations, each cited where it enters, each with its validity range
stated.  Nothing here touches pulsejet-fp / ramjet-fp, which remain
clean-room first-principles.

Where a term degenerates outside its fitted range it says so
(``trustworthy`` flags, ``mach_regime()``) instead of extrapolating
silently.  Two knobs -- ``leading_edge_suction`` and
``fuselage_diameter_ratio`` -- are load-bearing enough that they are
exposed, defaulted conservatively, and reported term-by-term
(``oswald_efficiency_breakdown``) rather than buried.

References
  - Polhamus, E.C., "A Simple Method of Estimating the Subsonic Lift and
    Damping in Roll of Sweptback Wings", NACA TN 1862 / TN 3911; the
    same expression is USAF DATCOM 4.1.3.2 and Raymer, *Aircraft
    Design*, eq. 12.6.  Valid for ALL aspect ratios: it degenerates to
    R.T. Jones' slender-wing limit pi*AR/2 as AR -> 0 and to the 2-D
    value 2*pi*eta as AR -> infinity, which is why it is used here
    rather than the lifting-line form (AR >~ 4 only).
  - Jones, R.T., "Properties of Low-Aspect-Ratio Pointed Wings at Speeds
    Below and Above the Speed of Sound", NACA TR 835 (slender-wing
    limit).
  - Puckett, A.E. & Stewart, H.J., "Aerodynamic Performance of Delta
    Wings at Supersonic Speeds", JAS 1947; Ackeret linearized supersonic
    theory (2-D slope 4/beta, rectangular-wing tip correction).
  - Hoerner, S.F., *Fluid-Dynamic Lift* / *Fluid-Dynamic Drag*: induced
    drag factor delta(lambda, AR), fuselage lift-carryover loss.
  - Nita, M. & Scholz, D., "Estimating the Oswald Factor from Basic
    Aircraft Geometrical Parameters", DLRK 2012 -- closed-form fit to
    Hoerner's delta charts, plus the fuselage and viscous factors.
  - Raymer, *Aircraft Design*: leading-edge-suction method for
    drag-due-to-lift (eq. 12.48-12.53), CL_max sweep correction
    (eq. 12.15).
  - Polhamus, E.C., "A Concept of the Vortex Lift of Sharp-Edge Delta
    Wings Based on a Leading-Edge-Suction Analogy", NASA TN D-3767 --
    the nonlinear vortex-lift term, available but OFF by default (see
    ``wing_clmax``); omitting it is the conservative direction.

Pure functions, no global state, no iteration.  (``stall_speed`` makes
one explicit, documented algebraic Mach-correction pass -- not a loop;
see its docstring.)
"""
from __future__ import annotations

from math import atan, cos, degrees, exp, pi, radians, sin, sqrt, tan
from typing import NamedTuple

from .constants import G0_M_PER_S2

# ---------------------------------------------------------------------------
# Regime boundaries and defaults (module constants, not mutable state)
# ---------------------------------------------------------------------------

SUBSONIC_VALID_MAX_MACH = 0.85
"""Above this the Prandtl-Glauert-based subsonic forms stop being usable
(beta -> 0) and the real curves are governed by shock/boundary-layer
interaction, which no algebraic form captures."""

SUPERSONIC_VALID_MIN_MACH = 1.05
"""Below this the linearized supersonic forms are singular/invalid."""

CLMAX_PLATEAU_MACH = 0.25
"""Below this, compressibility has no measurable effect on maximum lift."""

CLMAX_TRUSTED_MAX_MACH = 0.80
"""Above this the CL_max-vs-Mach correlation is NOT trustworthy: maximum
lift there is set by shock-induced separation / buffet onset, not by the
peak-suction scaling the correlation is built on.  Thin sections (this
vehicle's are t/c 0.03-0.04) push the real limit toward 0.85; the 9 %
'naca_symmetric' option pulls it toward 0.70.  0.80 is the compromise
this module refuses to extrapolate past silently."""

DEFAULT_AIRFOIL_EFFICIENCY = 0.97
"""eta = cl_alpha_2D / (2 pi).  Thin-airfoil theory gives 2 pi exactly;
real sections run 0.95-1.0 of it.  Standard DATCOM practice."""

DEFAULT_LEADING_EDGE_SUCTION = 0.85
"""Fraction of theoretical leading-edge suction actually realized in
attached subsonic flow (Raymer's leading-edge-suction method: 0.85-0.95
for a well-designed round-nose wing near its design CL, less for a sharp
or thin LE, less again at high CL).  THE dominant uncertainty in this
module at AR ~ 1.9 -- see ``oswald_efficiency``."""

DEFAULT_VISCOUS_FACTOR = 0.85
"""k_e,D0: lift-dependent VISCOUS drag, i.e. the growth of profile drag
with incidence, which the zero-lift build-up in drag_buildup.py does not
contain.  Nita & Scholz report a fleet mean of 0.804 for jet transports;
a clean single-wing dart with no pylons or flap tracks should sit at the
optimistic end of the 0.80-0.95 band.  0.85 chosen."""

DEFAULT_ALPHA_MAX_DEG = 25.0
"""Usable angle of attack for a thin low-AR wing.  Such wings do not
stall abruptly -- flow stays attached-plus-vortex well past the 2-D
stall angle -- so max lift is set by how much alpha the vehicle can trim
and hold, not by section stall.  25 deg is a conventional limit and is
also roughly where the alpha-limit and the 2-D-referenced estimate cross
(AR ~ 2.8 for this planform), so the mechanism switch is visible in the
tables rather than hidden."""


class ClmaxResult(NamedTuple):
    clmax: float
    mechanism: str
    trustworthy: bool
    note: str


class OswaldBreakdown(NamedTuple):
    e: float
    e_theoretical: float
    delta: float
    suction_fraction: float
    k_full_suction: float
    k_zero_suction: float
    k_effective: float
    k_fuselage: float
    k_viscous: float
    trustworthy: bool


class StallResult(NamedTuple):
    speed_m_per_s: float
    mach: float
    clmax: float
    mechanism: str
    compressibility_corrected: bool
    trustworthy: bool


# ---------------------------------------------------------------------------
# Geometry / compressibility helpers
# ---------------------------------------------------------------------------

def mach_regime(mach: float) -> str:
    """Which branch of this module governs, named honestly."""
    if mach <= SUBSONIC_VALID_MAX_MACH:
        return "subsonic"
    if mach < SUPERSONIC_VALID_MIN_MACH:
        return "transonic(bridged)"
    return "supersonic"


def prandtl_glauert_beta(mach: float) -> float:
    """|1 - M^2|^(1/2), floored away from the M = 1 singularity.

    The floor (0.30, i.e. M = 0.954 or 1.044) is a regularization, not
    physics -- exactly the same treatment drag.py already applies to the
    thin-wing wave-drag singularity.  Callers in the transonic band get
    the bridged branch, not this value.
    """
    return max(sqrt(abs(1.0 - mach * mach)), 0.30)


def sweep_at_chord_fraction(sweep_deg: float, aspect_ratio: float,
                            taper_ratio: float,
                            from_chord_fraction: float = 0.0,
                            to_chord_fraction: float = 0.5) -> float:
    """Convert sweep measured at one chord station to another.

        tan(L_n) = tan(L_m) - (4/AR) (n - m) (1 - lambda)/(1 + lambda)

    Exact for a straight-tapered planform (pure trapezoid geometry, no
    correlation involved).  Defaults convert leading-edge sweep to
    mid-chord sweep, which is what the Polhamus/DATCOM slope wants.
    """
    ar = max(aspect_ratio, 1.0e-6)
    lam = min(max(taper_ratio, 0.0), 1.0)
    shift = (4.0 / ar) * (to_chord_fraction - from_chord_fraction) \
        * (1.0 - lam) / (1.0 + lam)
    return degrees(atan(tan(radians(sweep_deg)) - shift))


# ---------------------------------------------------------------------------
# 1. Lift-curve slope
# ---------------------------------------------------------------------------

def _subsonic_lift_curve_slope(aspect_ratio: float, sweep_half_chord_deg: float,
                               mach: float, airfoil_efficiency: float) -> float:
    ar = max(aspect_ratio, 1.0e-6)
    beta = prandtl_glauert_beta(min(mach, SUBSONIC_VALID_MAX_MACH))
    eta = max(airfoil_efficiency, 1.0e-3)
    tan_half = tan(radians(sweep_half_chord_deg))
    inner = (ar * beta / eta) ** 2 * (1.0 + tan_half * tan_half / (beta * beta))
    return 2.0 * pi * ar / (2.0 + sqrt(4.0 + inner))


def _supersonic_lift_curve_slope(aspect_ratio: float, mach: float) -> float:
    ar = max(aspect_ratio, 1.0e-6)
    beta = prandtl_glauert_beta(max(mach, SUPERSONIC_VALID_MIN_MACH))
    # Ackeret 2-D slope with the linearized rectangular-wing tip loss:
    # the two tip Mach cones cover a fraction 1/(2 AR beta) of the
    # planform and carry no lift (Puckett & Stewart).  Valid AR*beta >= 1;
    # below that the cones overlap and the wing behaves as a slender one,
    # so the slender-wing value pi*AR/2 is used as a floor.
    tip_corrected = (4.0 / beta) * (1.0 - 1.0 / (2.0 * ar * beta))
    slender_floor = pi * ar / 2.0
    return max(tip_corrected, slender_floor)


def finite_wing_lift_curve_slope(
    aspect_ratio: float,
    sweep_deg: float,
    mach: float,
    *,
    taper_ratio: float = 1.0,
    airfoil_efficiency: float = DEFAULT_AIRFOIL_EFFICIENCY,
    sweep_is_leading_edge: bool = True,
) -> float:
    """Finite-wing lift-curve slope CL_alpha, per RADIAN.

    Subsonic branch -- Polhamus / DATCOM 4.1.3.2 (Raymer eq. 12.6):

        CL_a = 2 pi AR / (2 + sqrt(4 + (AR beta / eta)^2
                                     (1 + tan^2(L_c/2) / beta^2)))

    with beta = sqrt(1 - M^2), eta = cl_a,2D/(2 pi), L_c/2 the MID-CHORD
    sweep.  This form is chosen over lifting-line + Prandtl-Glauert
    precisely because of this vehicle's AR ~ 1.9: it is exact in both
    asymptotes (pi*AR/2 as AR -> 0, R.T. Jones slender-wing; 2*pi*eta as
    AR -> infinity, thin-airfoil) and is the standard low-AR method.
    Lifting-line would be a ~30 % error at AR 1.9.

    Supersonic branch (M >= 1.05) -- Ackeret 4/beta with the linearized
    rectangular tip correction, floored at the slender-wing value.

    Transonic band 0.85 < M < 1.05 -- NOT TRUSTWORTHY, and not silently
    faked either: the two branches are linearly bridged so the model
    stays continuous, and ``mach_regime()`` reports "transonic(bridged)".
    The real slope peaks near M = 1 ABOVE both endpoints, so the bridge
    is mildly conservative (under-predicts lift, over-predicts the alpha
    needed, and hence over-predicts drag-due-to-lift).

    Sweep is taken as leading-edge sweep by default (that is what
    ``WingConcept.sweep_deg`` means in this repo) and converted exactly.
    """
    if mach <= SUBSONIC_VALID_MAX_MACH:
        sweep_half = (sweep_at_chord_fraction(sweep_deg, aspect_ratio,
                                              taper_ratio, 0.0, 0.5)
                      if sweep_is_leading_edge else sweep_deg)
        return _subsonic_lift_curve_slope(aspect_ratio, sweep_half, mach,
                                          airfoil_efficiency)
    if mach >= SUPERSONIC_VALID_MIN_MACH:
        return _supersonic_lift_curve_slope(aspect_ratio, mach)

    sweep_half = (sweep_at_chord_fraction(sweep_deg, aspect_ratio, taper_ratio,
                                          0.0, 0.5)
                  if sweep_is_leading_edge else sweep_deg)
    low = _subsonic_lift_curve_slope(aspect_ratio, sweep_half,
                                     SUBSONIC_VALID_MAX_MACH,
                                     airfoil_efficiency)
    high = _supersonic_lift_curve_slope(aspect_ratio, SUPERSONIC_VALID_MIN_MACH)
    s = (mach - SUBSONIC_VALID_MAX_MACH) / (SUPERSONIC_VALID_MIN_MACH
                                            - SUBSONIC_VALID_MAX_MACH)
    return low + (high - low) * s


# ---------------------------------------------------------------------------
# 2. Maximum lift vs Mach
# ---------------------------------------------------------------------------

def clmax_vs_mach(clmax_low_speed: float, mach: float) -> float:
    """Maximum section-referenced lift coefficient at Mach ``mach``.

    Mechanism: the peak suction on the upper surface scales as 1/beta
    (Prandtl-Glauert), so holding the peak suction at the fixed level
    that triggers separation gives

        CL_max(M) = CL_max,lowspeed * beta(M) / beta(M_plateau)

    i.e. CL_max falls roughly as sqrt(1 - M^2) once compressibility
    bites.  Flat below M = 0.25 (no measurable effect there).  This
    reproduces the shape of measured 2-D CL_max-vs-Mach data (typically
    ~15-20 % down by M 0.6, ~40 % down by M 0.8).

    VALIDITY -- stated rather than extrapolated: above
    ``CLMAX_TRUSTED_MAX_MACH`` (0.80) the governing mechanism changes to
    shock-induced separation / buffet onset, which this scaling does not
    model, and beta -> 0 makes the form collapse.  Above 0.80 the value
    is HELD FLAT at its M = 0.80 level and
    ``clmax_vs_mach_detail(...).trustworthy`` is False.  Above M ~ 1
    there is no classical stall at all: usable lift is set by attainable
    angle of attack, so use ``wing_clmax``/``stall_speed``, which apply
    the alpha limit through CL_alpha(M).

    A tempting alternative -- the critical-pressure bound (CL at which
    peak suction first reaches Cp_crit) -- was evaluated and rejected: it
    is the onset-of-supercritical-flow boundary, not a lift limit, and
    collapses to ~12 % of the low-speed value by M 0.6, far faster than
    any measured CL_max.
    """
    return clmax_vs_mach_detail(clmax_low_speed, mach).clmax


def clmax_vs_mach_detail(clmax_low_speed: float, mach: float) -> ClmaxResult:
    """``clmax_vs_mach`` plus the mechanism and the honesty flag."""
    beta_plateau = prandtl_glauert_beta(CLMAX_PLATEAU_MACH)
    m = abs(mach)
    if m <= CLMAX_PLATEAU_MACH:
        return ClmaxResult(clmax_low_speed, "incompressible", True,
                           "flat: compressibility immaterial below M 0.25")
    if m <= CLMAX_TRUSTED_MAX_MACH:
        cl = clmax_low_speed * prandtl_glauert_beta(m) / beta_plateau
        return ClmaxResult(cl, "peak-suction/PG", True,
                           "CL_max ~ sqrt(1-M^2) peak-suction scaling")
    held = clmax_low_speed * prandtl_glauert_beta(CLMAX_TRUSTED_MAX_MACH) \
        / beta_plateau
    if m < 1.0:
        note = ("HELD at the M 0.80 value: above it maximum lift is set by "
                "shock-induced separation / buffet, not by this scaling")
    else:
        note = ("NO classical stall supersonically: usable lift is "
                "alpha-limited -- use wing_clmax()/stall_speed()")
    return ClmaxResult(held, "held (out of range)", False, note)


def wing_clmax(
    airfoil_clmax: float,
    aspect_ratio: float,
    taper_ratio: float,
    sweep_deg: float,
    mach: float,
    *,
    alpha_max_deg: float = DEFAULT_ALPHA_MAX_DEG,
    vortex_lift_kv: float = 0.0,
    airfoil_efficiency: float = DEFAULT_AIRFOIL_EFFICIENCY,
) -> ClmaxResult:
    """Attainable wing CL_max: the LESSER of the section-stall estimate
    and the angle-of-attack limit.

    Section-stall branch (binds for AR >~ 2.8 on this planform) --
    Raymer eq. 12.15:

        CL_max,wing = 0.9 * cl_max,2D(M) * cos(L_c/4)

    The 0.9 is the spanwise-loading/finite-span penalty; the cosine is
    simple sweep theory.  (For the V3a wing this lands within 5 % of the
    ancestor model's cl_max * cos^2(L_LE), so the two agree where the
    ancestor was defensible.)

    Alpha-limit branch (binds at AR ~ 1.9-2.8, i.e. THIS vehicle) -- a
    low-AR wing never reaches its section CL_max; it runs out of usable
    incidence first:

        CL_max = CL_alpha(M) * sin(a_max) cos^2(a_max)   [+ vortex lift]

    The sin*cos^2 resolution is the potential-flow term of Polhamus'
    leading-edge-suction analogy (NASA TN D-3767), which is the correct
    large-alpha form of the linear slope.

    ``vortex_lift_kv`` optionally adds that analogy's nonlinear term,
    Kv cos(a) sin^2(a).  It is 0.0 by DEFAULT and deliberately so: Kv is
    a chart lookup (~2.5-3.5 for sharp-edged AR 1-2 planforms) that this
    module will not invent, and omitting it UNDER-predicts CL_max, i.e.
    over-predicts stall speed.  The conservative direction.  With
    Kv = 3.0 the V3a wing gains ~0.45 CL at 25 deg -- large enough that
    it is worth a real chart lookup before any design decision leans on
    the stall margin.
    """
    section = clmax_vs_mach_detail(airfoil_clmax, mach)
    sweep_quarter = sweep_at_chord_fraction(sweep_deg, aspect_ratio,
                                            taper_ratio, 0.0, 0.25)
    cl_section = 0.9 * section.clmax * cos(radians(sweep_quarter))

    a = radians(alpha_max_deg)
    cl_alpha = finite_wing_lift_curve_slope(
        aspect_ratio, sweep_deg, mach, taper_ratio=taper_ratio,
        airfoil_efficiency=airfoil_efficiency)
    cl_alpha_limited = (cl_alpha * sin(a) * cos(a) ** 2
                        + vortex_lift_kv * cos(a) * sin(a) ** 2)

    if cl_alpha_limited <= cl_section:
        return ClmaxResult(
            cl_alpha_limited, f"alpha-limited @ {alpha_max_deg:.0f} deg",
            section.trustworthy,
            "low-AR wing: usable incidence, not section stall, sets CL_max"
            + ("" if vortex_lift_kv else " (vortex lift NOT credited)"))
    return ClmaxResult(cl_section, "section-stall (2D x sweep)",
                       section.trustworthy, section.note)


# ---------------------------------------------------------------------------
# 3. Oswald span efficiency
# ---------------------------------------------------------------------------

def glauert_induced_drag_factor(aspect_ratio: float, taper_ratio: float,
                                sweep_deg: float) -> float:
    """Glauert/Hoerner induced-drag factor delta, so e_theoretical =
    1/(1 + delta).

    Closed-form fit to Hoerner's delta(lambda, AR) charts (Nita &
    Scholz 2012):

        d_lambda = -0.357 + 0.45 exp(0.0375 * L_c/4[deg])
        lambda_eff = lambda - d_lambda
        f = 0.0524 l^4 - 0.15 l^3 + 0.1659 l^2 - 0.0706 l + 0.0119
        delta = f(lambda_eff) * AR

    The sweep shift is what makes a swept wing want a smaller taper
    ratio; f is minimal near lambda_eff ~ 0.35-0.45 (nearest to elliptic
    loading), which is the same optimum the ancestor's hand-fitted
    quadratic encodes.  Note delta ~ AR: at AR 1.9 the theoretical span
    efficiency is essentially 1.0, so at this AR the Oswald factor is
    NOT set by planform shape -- it is set by leading-edge suction and
    by the fuselage.  Fitted for L_c/4 <~ 30 deg; this wing is 3.7 deg.
    """
    sweep_quarter = sweep_at_chord_fraction(sweep_deg, aspect_ratio,
                                            taper_ratio, 0.0, 0.25)
    d_lambda = -0.357 + 0.45 * exp(0.0375 * sweep_quarter)
    lam = taper_ratio - d_lambda
    f = (0.0524 * lam ** 4 - 0.15 * lam ** 3 + 0.1659 * lam ** 2
         - 0.0706 * lam + 0.0119)
    return max(f, 0.0) * max(aspect_ratio, 0.0)


def leading_edge_suction_fraction(mach: float, sweep_deg: float,
                                  subsonic_value: float
                                  = DEFAULT_LEADING_EDGE_SUCTION) -> float:
    """Fraction of theoretical leading-edge suction actually realized.

    Subsonic attached flow recovers most of it (Raymer's leading-edge
    suction method: 0.85-0.95 round-nose, less for a sharp/thin LE).
    Once the component of Mach NORMAL to the leading edge goes
    supersonic the LE shock destroys the suction peak and drag-due-to-
    lift approaches the zero-suction limit CL^2/CL_alpha -- so the
    fraction is ramped to zero between M_n = 0.90 and M_n = 1.10.

    This vehicle's 13 deg of sweep buys essentially nothing here
    (cos 13.4 = 0.973), which is itself worth knowing: the suction
    collapse happens right where the mission's transonic push does.
    """
    m_normal = abs(mach) * cos(radians(sweep_deg))
    if m_normal <= 0.90:
        return min(max(subsonic_value, 0.0), 1.0)
    if m_normal >= 1.10:
        return 0.0
    s = (m_normal - 0.90) / 0.20
    return min(max(subsonic_value, 0.0), 1.0) * (1.0 - s)


def oswald_efficiency(
    aspect_ratio: float,
    taper_ratio: float,
    sweep_deg: float,
    mach: float,
    *,
    fuselage_diameter_ratio: float = 0.0,
    viscous_factor: float = DEFAULT_VISCOUS_FACTOR,
    leading_edge_suction: float | None = None,
    airfoil_efficiency: float = DEFAULT_AIRFOIL_EFFICIENCY,
) -> float:
    """Oswald span efficiency e for D_i = L^2 / (q pi b^2 e).

    Four multiplicative pieces, in decreasing order of leverage on THIS
    vehicle (see ``oswald_efficiency_breakdown`` for the term-by-term
    numbers):

    1. **Leading-edge suction** (dominant at low AR).  Raymer's method:
       the drag-due-to-lift factor K is blended between the full-suction
       and zero-suction limits,

           K = s / (pi AR e_theo)  +  (1 - s) / CL_alpha

       At AR 1.9 those two limits differ by 2.4x, so this single
       fraction moves induced drag more than every planform effect
       combined.  It also carries the entire Mach dependence: it decays
       to zero as the leading-edge-normal Mach goes supersonic, which is
       why e falls transonically here.
    2. **Fuselage lift carry-over loss** (Hoerner; Nita & Scholz):
       k_F = 1 - 2 (d_fus/b)^2.  DEFAULT OFF (ratio 0.0) and that is a
       deliberate, flagged choice: V3a's d/b = 0.42 would cost 36 %, but
       the correlation is fitted for d/b <~ 0.2 and it assumes the body
       carries NO lift.  A body 42 % of the span carries a great deal
       (slender-body carry-over), in the opposite direction and of the
       same order.  Resolving that needs CFD; until then this term is
       exposed, bracketed, and off rather than quietly applied.
    3. **Lift-dependent viscous drag** k_D0 = ``viscous_factor``: the
       growth of profile drag with incidence, which the zero-lift
       build-up in drag_buildup.py genuinely does not contain.
    4. **Planform** e_theo = 1/(1 + delta), Glauert/Hoerner -- nearly
       1.0 at this AR, i.e. almost irrelevant here.

    NOT used: Nita & Scholz's Mach factor k_e,M = a((M/0.3)-1)^b + 1.
    It is fitted to transport cruise (M 0.7-0.85) and, checked
    numerically, goes NEGATIVE by M 0.85 -- unusable for a vehicle whose
    whole mission is the transonic push.  The suction collapse above
    supplies the transonic loss instead, with a stated mechanism.
    """
    return oswald_efficiency_breakdown(
        aspect_ratio, taper_ratio, sweep_deg, mach,
        fuselage_diameter_ratio=fuselage_diameter_ratio,
        viscous_factor=viscous_factor,
        leading_edge_suction=leading_edge_suction,
        airfoil_efficiency=airfoil_efficiency).e


def oswald_efficiency_breakdown(
    aspect_ratio: float,
    taper_ratio: float,
    sweep_deg: float,
    mach: float,
    *,
    fuselage_diameter_ratio: float = 0.0,
    viscous_factor: float = DEFAULT_VISCOUS_FACTOR,
    leading_edge_suction: float | None = None,
    airfoil_efficiency: float = DEFAULT_AIRFOIL_EFFICIENCY,
) -> OswaldBreakdown:
    """``oswald_efficiency`` with every factor exposed, so the leverage
    of each assumption stays visible instead of collapsing into one
    number."""
    ar = max(aspect_ratio, 1.0e-6)
    delta = glauert_induced_drag_factor(ar, taper_ratio, sweep_deg)
    e_theo = 1.0 / (1.0 + delta)

    cl_alpha = finite_wing_lift_curve_slope(
        ar, sweep_deg, mach, taper_ratio=taper_ratio,
        airfoil_efficiency=airfoil_efficiency)
    k_full = 1.0 / (pi * ar * e_theo)
    k_zero = 1.0 / max(cl_alpha, 1.0e-6)
    s = (leading_edge_suction_fraction(mach, sweep_deg)
         if leading_edge_suction is None
         else min(max(leading_edge_suction, 0.0), 1.0))
    k_eff = s * k_full + (1.0 - s) * k_zero

    k_fus = 1.0 - 2.0 * fuselage_diameter_ratio ** 2
    k_fus = min(max(k_fus, 0.05), 1.0)
    k_vis = min(max(viscous_factor, 0.05), 1.0)

    e = (1.0 / (pi * ar * k_eff)) * k_fus * k_vis
    return OswaldBreakdown(min(e, 1.0), e_theo, delta, s, k_full, k_zero,
                           k_eff, k_fus, k_vis,
                           mach_regime(mach) != "transonic(bridged)")


def induced_drag_n(lift_n: float, dynamic_pressure_pa: float, span_m: float,
                   oswald_e: float) -> float:
    """D_i = L^2 / (q pi b^2 e) -- identical algebra to drag.py's, given
    so a caller can substitute this module's ``e`` as a drop-in."""
    if dynamic_pressure_pa <= 0.0 or span_m <= 0.0 or oswald_e <= 0.0:
        return 0.0
    return lift_n ** 2 / (dynamic_pressure_pa * pi * span_m ** 2 * oswald_e)


# ---------------------------------------------------------------------------
# 4. Stall speed
# ---------------------------------------------------------------------------

def stall_speed(
    mass_kg: float,
    wing_area_m2: float,
    air_density_kg_per_m3: float,
    airfoil_clmax: float,
    aspect_ratio: float,
    taper_ratio: float,
    sweep_deg: float,
    *,
    speed_of_sound_m_per_s: float = 340.294,
    load_factor: float = 1.0,
    gravity_m_per_s2: float = G0_M_PER_S2,
    alpha_max_deg: float = DEFAULT_ALPHA_MAX_DEG,
    vortex_lift_kv: float = 0.0,
) -> float:
    """Speed at which L = n W requires the full attainable CL_max."""
    return stall_speed_detail(
        mass_kg, wing_area_m2, air_density_kg_per_m3, airfoil_clmax,
        aspect_ratio, taper_ratio, sweep_deg,
        speed_of_sound_m_per_s=speed_of_sound_m_per_s,
        load_factor=load_factor, gravity_m_per_s2=gravity_m_per_s2,
        alpha_max_deg=alpha_max_deg,
        vortex_lift_kv=vortex_lift_kv).speed_m_per_s


def stall_speed_detail(
    mass_kg: float,
    wing_area_m2: float,
    air_density_kg_per_m3: float,
    airfoil_clmax: float,
    aspect_ratio: float,
    taper_ratio: float,
    sweep_deg: float,
    *,
    speed_of_sound_m_per_s: float = 340.294,
    load_factor: float = 1.0,
    gravity_m_per_s2: float = G0_M_PER_S2,
    alpha_max_deg: float = DEFAULT_ALPHA_MAX_DEG,
    vortex_lift_kv: float = 0.0,
) -> StallResult:
    """Stall speed using ``wing_clmax`` -- i.e. sweep, aspect ratio, the
    alpha limit and compressibility all included.

    CL_max depends on Mach and Mach depends on the answer, which would
    normally mean iteration.  It does not here, and deliberately: the
    speed is evaluated ONCE at the incompressible-plateau CL_max, then
    corrected ONCE at the resulting Mach.  Two algebraic passes, fixed
    cost, no loop -- the repo's no-iteration rule intact.  The
    correction is inert for this vehicle (stall sits near M 0.15, well
    inside the M <= 0.25 plateau where CL_max is flat), so the second
    pass changes nothing until someone flies this at altitude or under
    load factor; ``compressibility_corrected`` reports whether it did.
    """
    weight_n = max(load_factor, 0.0) * mass_kg * gravity_m_per_s2
    if wing_area_m2 <= 0.0 or air_density_kg_per_m3 <= 0.0 or weight_n <= 0.0:
        return StallResult(float("inf"), 0.0, 0.0, "degenerate", False, False)

    def speed_for(clmax: float) -> float:
        return sqrt(2.0 * weight_n
                    / (air_density_kg_per_m3 * wing_area_m2 * max(clmax, 1e-9)))

    first = wing_clmax(airfoil_clmax, aspect_ratio, taper_ratio, sweep_deg,
                       0.0, alpha_max_deg=alpha_max_deg,
                       vortex_lift_kv=vortex_lift_kv)
    v0 = speed_for(first.clmax)
    m0 = v0 / max(speed_of_sound_m_per_s, 1.0e-6)
    if m0 <= CLMAX_PLATEAU_MACH:
        return StallResult(v0, m0, first.clmax, first.mechanism, False,
                           first.trustworthy)
    second = wing_clmax(airfoil_clmax, aspect_ratio, taper_ratio, sweep_deg,
                        m0, alpha_max_deg=alpha_max_deg,
                        vortex_lift_kv=vortex_lift_kv)
    v1 = speed_for(second.clmax)
    return StallResult(v1, v1 / max(speed_of_sound_m_per_s, 1.0e-6),
                       second.clmax, second.mechanism, True,
                       second.trustworthy)


# ---------------------------------------------------------------------------
# Inspection table
# ---------------------------------------------------------------------------

if __name__ == "__main__":  # pragma: no cover - inspection aid
    ARS = (1.9, 2.4, 3.0, 3.5, 4.0)
    MACHS = (0.1, 0.3, 0.5, 0.7, 0.9)

    # V3a wing (docs/v3a_medium_model/design.json)
    V3A_SPAN = 0.532526287202532
    V3A_AR = 1.8595845502152375
    V3A_TAPER = 0.5120481632935137
    V3A_SWEEP = 13.4220600837535
    V3A_CLMAX2D = 1.20          # AIRFOILS['thin_cambered']
    V3A_BODY_D = 0.22521729709177246
    V3A_AREA = V3A_SPAN ** 2 / V3A_AR
    WET_MASS = 22.6796185       # 50 lb
    RHO_SL, A_SL = 1.225, 340.294

    print("Wing lift module -- medium_model/lift.py")
    print(f"taper 0.5, LE sweep 13.4 deg, eta {DEFAULT_AIRFOIL_EFFICIENCY}\n")

    print("CL_alpha [per rad]   (2-D thin-airfoil limit = 6.28; "
          "slender limit pi*AR/2)")
    print("  AR  | " + " ".join(f"M{m:<5.1f}" for m in MACHS)
          + " |  pi*AR/2")
    for ar in ARS:
        row = " ".join(
            f"{finite_wing_lift_curve_slope(ar, 13.4, m, taper_ratio=0.5):<6.3f}"
            for m in MACHS)
        print(f" {ar:4.1f} | {row} |  {pi * ar / 2:.3f}")

    print("\nCL_alpha [per deg]")
    print("  AR  | " + " ".join(f"M{m:<6.1f}" for m in MACHS))
    for ar in ARS:
        row = " ".join(
            f"{radians(finite_wing_lift_curve_slope(ar, 13.4, m, taper_ratio=0.5)):<7.4f}"
            for m in MACHS)
        print(f" {ar:4.1f} | {row}")

    print("\nOswald e (defaults: s_LE 0.85, k_visc 0.85, NO fuselage term)")
    print("  AR  | " + " ".join(f"M{m:<5.1f}" for m in MACHS)
          + " | legacy e")
    for ar in ARS:
        row = " ".join(f"{oswald_efficiency(ar, 0.5, 13.4, m):<6.3f}"
                       for m in MACHS)
        legacy = 0.85 - 0.118 * (0.5 - 0.35) ** 2
        print(f" {ar:4.1f} | {row} |  {legacy:.3f}")

    print("\nOswald term-by-term at M 0.3 (shows what actually moves it)")
    print("  AR  | e_theo  K_full  K_zero   s     K_eff   e")
    for ar in ARS:
        b = oswald_efficiency_breakdown(ar, 0.5, 13.4, 0.3)
        print(f" {ar:4.1f} | {b.e_theoretical:6.4f}  {b.k_full_suction:6.4f}"
              f"  {b.k_zero_suction:6.4f}  {b.suction_fraction:4.2f}"
              f"  {b.k_effective:6.4f}  {b.e:5.3f}")

    print("\nCL_max (airfoil cl_max 1.20, alpha_max 25 deg, no vortex lift)")
    print("  AR  | " + " ".join(f"M{m:<5.1f}" for m in MACHS)
          + " | mechanism @ M0.3")
    for ar in ARS:
        row = " ".join(f"{wing_clmax(1.2, ar, 0.5, 13.4, m).clmax:<6.3f}"
                       for m in MACHS)
        mech = wing_clmax(1.2, ar, 0.5, 13.4, 0.3).mechanism
        print(f" {ar:4.1f} | {row} | {mech}")

    print("\n2-D CL_max vs Mach (clmax_vs_mach, cl_max_ls = 1.20)")
    for m in (0.1, 0.25, 0.4, 0.6, 0.8, 0.85, 0.9, 1.1):
        r = clmax_vs_mach_detail(1.2, m)
        flag = "ok " if r.trustworthy else "!! "
        print(f"  M {m:4.2f}  CLmax {r.clmax:5.3f}  {flag}{r.mechanism:22s}"
              f" {r.note}")

    print("\n--- V3a wing, this module vs the ancestor model ---")
    legacy_e = 0.85 - 0.118 * (V3A_TAPER - 0.35) ** 2
    legacy_clmax = V3A_CLMAX2D * cos(radians(V3A_SWEEP)) ** 2
    legacy_vs = sqrt(2 * WET_MASS * G0_M_PER_S2
                     / (RHO_SL * V3A_AREA * legacy_clmax))
    new_e = oswald_efficiency(V3A_AR, V3A_TAPER, V3A_SWEEP, 0.12)
    new_e_fus = oswald_efficiency(V3A_AR, V3A_TAPER, V3A_SWEEP, 0.12,
                                  fuselage_diameter_ratio=V3A_BODY_D / V3A_SPAN)
    st = stall_speed_detail(WET_MASS, V3A_AREA, RHO_SL, V3A_CLMAX2D, V3A_AR,
                            V3A_TAPER, V3A_SWEEP)
    st_v = stall_speed_detail(WET_MASS, V3A_AREA, RHO_SL, V3A_CLMAX2D, V3A_AR,
                              V3A_TAPER, V3A_SWEEP, vortex_lift_kv=3.0)
    lift_n = WET_MASS * G0_M_PER_S2
    q40 = 0.5 * RHO_SL * 40.0 ** 2
    print(f"  S = {V3A_AREA:.4f} m2, b = {V3A_SPAN:.3f} m, AR = {V3A_AR:.3f},"
          f" d_fus/b = {V3A_BODY_D / V3A_SPAN:.3f}")
    print(f"  e            legacy {legacy_e:5.3f} -> new {new_e:5.3f}"
          f"   (with fuselage term: {new_e_fus:5.3f})")
    print(f"  CL_max       legacy {legacy_clmax:5.3f} -> new {st.clmax:5.3f}"
          f"   [{st.mechanism}]")
    print(f"  V_stall SL   legacy {legacy_vs:5.2f} -> new"
          f" {st.speed_m_per_s:5.2f} m/s  (with Kv=3 vortex lift:"
          f" {st_v.speed_m_per_s:5.2f})")
    print(f"  D_induced at 40 m/s, level: legacy"
          f" {induced_drag_n(lift_n, q40, V3A_SPAN, legacy_e):6.1f} N -> new"
          f" {induced_drag_n(lift_n, q40, V3A_SPAN, new_e):6.1f} N"
          f"  (fuselage term: "
          f"{induced_drag_n(lift_n, q40, V3A_SPAN, new_e_fus):6.1f} N)")
    print(f"  CL required at 40 m/s = "
          f"{lift_n / (q40 * V3A_AREA):.3f}  (vs CL_max {st.clmax:.3f})")

    print("\n--- sanity checks ---")
    checks = []
    slender = finite_wing_lift_curve_slope(0.05, 0.0, 0.1, taper_ratio=1.0)
    checks.append(("AR->0 approaches Jones pi*AR/2",
                   abs(slender / (pi * 0.05 / 2) - 1.0) < 0.02))
    twod = finite_wing_lift_curve_slope(400.0, 0.0, 0.05, taper_ratio=1.0)
    checks.append(("AR->inf approaches 2*pi*eta",
                   abs(twod / (2 * pi * DEFAULT_AIRFOIL_EFFICIENCY) - 1.0)
                   < 0.02))
    checks.append(("CL_alpha rises with AR", all(
        finite_wing_lift_curve_slope(a, 13.4, 0.3, taper_ratio=0.5)
        < finite_wing_lift_curve_slope(b, 13.4, 0.3, taper_ratio=0.5)
        for a, b in zip(ARS, ARS[1:]))))
    checks.append(("CL_alpha rises with Mach (subsonic)", all(
        finite_wing_lift_curve_slope(1.9, 13.4, a, taper_ratio=0.5)
        < finite_wing_lift_curve_slope(1.9, 13.4, b, taper_ratio=0.5)
        for a, b in zip(MACHS, MACHS[1:]))))
    checks.append(("e <= 1 everywhere", all(
        oswald_efficiency(ar, t, sw, m) <= 1.0
        for ar in ARS for t in (0.2, 0.5, 1.0) for sw in (0.0, 13.4, 30.0)
        for m in (0.1, 0.5, 0.9, 1.2))))
    checks.append(("e falls through the transonic",
                   oswald_efficiency(1.9, 0.5, 13.4, 1.2)
                   < oswald_efficiency(1.9, 0.5, 13.4, 0.3)))
    checks.append(("CL_max falls with Mach", all(
        clmax_vs_mach(1.2, a) >= clmax_vs_mach(1.2, b)
        for a, b in zip(MACHS, MACHS[1:]))))
    checks.append(("taper 0.35-0.45 is the e optimum",
                   oswald_efficiency(8.0, 0.4, 0.0, 0.2)
                   > max(oswald_efficiency(8.0, 1.0, 0.0, 0.2),
                         oswald_efficiency(8.0, 0.1, 0.0, 0.2))))
    checks.append(("stall speed scales as 1/sqrt(rho)", abs(
        stall_speed(WET_MASS, V3A_AREA, RHO_SL / 2, V3A_CLMAX2D, V3A_AR,
                    V3A_TAPER, V3A_SWEEP)
        / stall_speed(WET_MASS, V3A_AREA, RHO_SL, V3A_CLMAX2D, V3A_AR,
                      V3A_TAPER, V3A_SWEEP) - sqrt(2.0)) < 1e-9))
    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
