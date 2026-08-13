"""Component drag build-up -- medium_model's step-1 fidelity upgrade.

Replaces the ancestor's single flat ``CD0_FRONTAL`` (an explicitly
unsourced placeholder, run for the V2 campaign at 0.10 -- one third of its
own 0.30 default, and just below its own stated 0.125-0.15 feasibility
boundary) with a build-up whose terms each have a physical driver and a
citation.

EMPIRICISM POLICY (deliberate, and different from the FP engine repos):
vehicle aerodynamics cannot be derived end-to-end without CFD, so this
module uses standard engineering correlations, each cited where it enters,
with its uncertainty stated. The first-principles engine internals
(pulsejet-fp / ramjet-fp) remain strictly clean-room; nothing here touches
them. Where a term CAN be derived rather than correlated -- notably
spillage/additive drag, which is a momentum-theorem statement -- it is
derived, and that is noted.

Terms, and why each one earns its place on THIS vehicle:

* **Skin friction** -- the flight sweeps Re ~ 1e6 to ~2e7, over which the
  turbulent flat-plate C_f falls by roughly a factor of two. A constant
  CD0 cannot represent that at all.
* **Form drag** -- pressure drag from thickness, as a factor on friction,
  keyed to the real fineness ratio.
* **Base drag, split engine-on / engine-off** -- 140 of V2's 170 s are
  unpowered glide behind a blunt base, and base drag is what sets glide
  range, i.e. the "lands 2 m from launch" result. With the engine lit the
  exhaust fills the duct exit and only the annulus around it is base.
* **Wave drag** -- the mission is decided in the transonic push, exactly
  where a multiplier on a placeholder is weakest. Keyed to volume and
  fineness via the Sears-Haack minimum with a shape penalty.
* **Spillage / additive drag** -- at ~55 % spillage this is a real force
  that is currently charged to NOBODY: the FP engine models exclude it by
  assumption (engine thrust, not installed force), and the ancestor
  vehicle model does not know it exists.

References (conventional aerodynamic build-up):
  - Prandtl-Schlichting turbulent flat-plate skin friction; compressibility
    factor after van Driest / Frankl-Voishel (Schlichting, *Boundary Layer
    Theory*).
  - Body and wing form factors: Hoerner, *Fluid-Dynamic Drag*; Raymer,
    *Aircraft Design*, component build-up.
  - Base pressure on a blunt-based body of revolution: Hoerner.
  - Sears-Haack minimum wave drag for a slender body of revolution.
  - Additive/pre-entry drag: momentum theorem on the capture streamtube
    (derived here, not correlated); cowl-lip suction recovery is the one
    empirical factor in that term.
"""
from __future__ import annotations

from math import cos, log10, pi, radians, sqrt
from typing import NamedTuple

# ---------------------------------------------------------------------------
# Air properties
# ---------------------------------------------------------------------------

_SUTHERLAND_MU0 = 1.716e-5     # Pa s at T0
_SUTHERLAND_T0 = 273.15        # K
_SUTHERLAND_S = 110.4          # K


def dynamic_viscosity_pa_s(temperature_k: float) -> float:
    """Sutherland's law -- viscosity is what makes friction Reynolds- and
    hence altitude/speed-dependent, which a constant CD0 discards."""
    t = max(temperature_k, 1.0)
    return (_SUTHERLAND_MU0 * (t / _SUTHERLAND_T0) ** 1.5
            * (_SUTHERLAND_T0 + _SUTHERLAND_S) / (t + _SUTHERLAND_S))


def reynolds_number(density_kg_per_m3: float, velocity_m_per_s: float,
                    length_m: float, temperature_k: float) -> float:
    mu = dynamic_viscosity_pa_s(temperature_k)
    return max(density_kg_per_m3 * abs(velocity_m_per_s) * length_m / mu, 1.0)


# ---------------------------------------------------------------------------
# Skin friction
# ---------------------------------------------------------------------------

def skin_friction_coefficient(reynolds: float, mach: float) -> float:
    """Incompressible Prandtl-Schlichting turbulent flat plate,

        C_f = 0.455 / (log10 Re)^2.58

    with a compressibility factor (Frankl-Voishel form) applied to account
    for the reduced density in a heated boundary layer at speed:

        C_f,comp = C_f / (1 + 0.144 M^2)^0.65

    Fully-turbulent is assumed: at Re > 1e6 over a body with an inlet, a
    valve grid and surface joints, laminar run is negligible and assuming
    it would be optimistic.
    """
    re = max(reynolds, 1.0e3)
    cf_incompressible = 0.455 / (log10(re) ** 2.58)
    return cf_incompressible / (1.0 + 0.144 * mach * mach) ** 0.65


def body_form_factor(fineness_ratio: float) -> float:
    """Pressure (form) drag as a multiplier on friction for a body of
    revolution -- Hoerner/Raymer:  FF = 1 + 60/f^3 + f/400.

    A short, fat body pays heavily; a slender one approaches the flat-plate
    value. This is precisely the geometric sensitivity the flat CD0 hides.
    """
    f = max(fineness_ratio, 1.0)
    return 1.0 + 60.0 / (f ** 3) + f / 400.0


def wing_form_factor(thickness_ratio: float, sweep_deg: float,
                     mach: float) -> float:
    """Raymer component form factor for a lifting surface (subsonic form;
    above M ~ 1 wave drag takes over and is accounted separately)."""
    tc = max(thickness_ratio, 1.0e-3)
    m = min(mach, 0.95)
    cos_sweep = max(cos(radians(sweep_deg)), 0.2)
    return ((1.0 + 0.6 / 0.3 * tc + 100.0 * tc ** 4)
            * (1.34 * m ** 0.18 * cos_sweep ** 0.28))


# ---------------------------------------------------------------------------
# Base drag
# ---------------------------------------------------------------------------

def base_pressure_coefficient(mach: float) -> float:
    """Base pressure coefficient for a blunt-based body of revolution.

    Subsonic values cluster near Cp_b ~ -0.15 (Hoerner); the magnitude
    grows through the transonic rise to ~-0.30 near M 1.2, then relaxes
    supersonically as the base expansion weakens, bounded below by the
    perfect-vacuum limit -2/(gamma M^2).

    Continuous by construction at both joints -- an earlier piecewise form
    jumped by 2x at M = 0.8, which put a step in the drag exactly where
    this mission does its transonic push.
    """
    if mach <= 0.8:
        return -0.15
    vacuum_limit = -2.0 / (1.4 * max(mach, 0.1) ** 2)
    if mach <= 1.2:
        cp = -0.15 - 0.15 * (mach - 0.8) / 0.4
    else:
        cp = -0.30 * (1.2 / mach) ** 2
    return max(cp, vacuum_limit)


def base_drag_n(base_area_m2: float, dynamic_pressure_pa: float,
                mach: float) -> float:
    """D_base = -Cp_b * q * A_base  (Cp_b < 0, so this is a drag)."""
    if base_area_m2 <= 0.0:
        return 0.0
    return -base_pressure_coefficient(mach) * dynamic_pressure_pa * base_area_m2


BOATTAIL_HALF_ANGLE_DEG = 8.0
"""ASSUMPTION, and a load-bearing one -- flagged for review.

The ancestor model has no aft-body shape at all: it carries a nose/tail
length allowance for MASS but computes drag from frontal area only. Base
drag, however, depends entirely on how much of that frontal area is still
there at the tail. Treating the aft end as a flat annulus (no boattail)
made base drag the single largest term in the whole build-up, which is an
artifact of assuming the vehicle ends in a cliff.

8 deg is the conventional upper bound for an attached (non-separating)
boattail; steeper separates and loses the benefit. With the model's own
1.0-diameter tail allowance this closes a 280 mm body to ~200 mm before
the base. Base area is reported so the assumption's leverage stays
visible, and it is worth a sensitivity pass.
"""


def base_diameter_m(body_diameter_m: float, tail_length_m: float,
                    duct_exit_diameter_m: float,
                    boattail_half_angle_deg: float = BOATTAIL_HALF_ANGLE_DEG
                    ) -> float:
    """Diameter at the aft end after the boattail closes down, floored at
    the duct exit (the aft body cannot close inside its own nozzle)."""
    from math import tan
    shrink = 2.0 * tail_length_m * tan(radians(boattail_half_angle_deg))
    return max(body_diameter_m - shrink, duct_exit_diameter_m)


def base_area_m2(base_diameter: float, duct_exit_diameter_m: float,
                 engine_on: bool) -> float:
    """The area actually behaving as a base.

    Engine ON: exhaust fills the duct exit, so only the annulus between
    the boattail base and the exit is base. Engine OFF: the dead duct exit
    is base as well -- the distinction that matters because 140 of this
    mission's 170 s are an unpowered glide.
    """
    base = pi * base_diameter ** 2 / 4.0
    exit_a = pi * duct_exit_diameter_m ** 2 / 4.0
    return max(base - exit_a, 0.0) if engine_on else base


# ---------------------------------------------------------------------------
# Wave drag
# ---------------------------------------------------------------------------

def wave_drag_coefficient(mach: float, body_diameter_m: float,
                          body_length_m: float,
                          shape_penalty: float = 1.75) -> float:
    """Wave-drag coefficient on FRONTAL area for a slender body.

    Baseline is the Sears-Haack minimum for a body of the same length and
    maximum area,
        C_Dw,min = (81 pi^4 / 512) * (A_max / L^2) / (pi/4)   [frontal-ref]
    which for a real (non-optimal) nose/boattail shape is multiplied by a
    penalty factor (1.75 default; a well-shaped ogive-cylinder-boattail
    runs ~1.5-2x the Sears-Haack minimum).

    Mach shaping: zero below the drag-divergence Mach, a smooth rise to a
    peak just above M = 1, then the (M^2 - 1)^-1/2-like decay of linearised
    supersonic theory. Transonic peak location and the divergence Mach are
    the empirical part of this term.
    """
    if mach <= 0.85:
        return 0.0
    d_over_l = max(body_diameter_m, 1e-6) / max(body_length_m, 1e-6)
    # Sears-Haack minimum, referenced to frontal area
    cdw_min = (81.0 * pi ** 4 / 512.0) * (pi / 4.0) * d_over_l ** 2
    peak = cdw_min * shape_penalty
    if mach < 1.2:
        s = (mach - 0.85) / (1.2 - 0.85)
        return peak * s * s
    return peak * 1.2 / mach


# ---------------------------------------------------------------------------
# Inlet spillage / additive drag  (DERIVED, not correlated)
# ---------------------------------------------------------------------------

def spillage_drag_n(captured_mass_flow_kg_per_s: float,
                    lip_area_m2: float,
                    density_kg_per_m3: float,
                    velocity_m_per_s: float,
                    cowl_suction_recovery: float = 0.85) -> float:
    """Pre-entry (additive) drag of the spilled streamtube.

    Momentum theorem on the capture streamtube between the free stream and
    the cowl lip. The engine swallows only A_0 = mdot / (rho_inf V_inf) of
    the lip area A_lip; the remainder is turned around the cowl. The flow
    ahead of the lip decelerates, so the streamtube surface carries a net
    rearward force

        D_add = (A_lip - A_0) * q * K_spill

    where the bracketed area is the spilled capture area and K_spill
    follows from the momentum deficit of the turned flow. Taking the
    subsonic-spillage momentum balance with the lip static pressure
    approaching stagnation as A_0/A_lip -> 0 gives K_spill -> 1 in the
    fully-spilled limit and 0 when the inlet swallows its full capture
    area; the linear-in-area form below is that balance.

    ``cowl_suction_recovery`` is the empirical factor, and on THIS vehicle
    it is doing a lot of work -- read before trusting the number.

    A podded nacelle turns its spilled flow sharply around a cowl lip and
    recovers roughly half the additive drag as leading-edge suction
    (Hoerner; ESDU inlet data), i.e. recovery ~ 0.5. **This vehicle's
    inlet is the NOSE of the body, not a pod.** Its spilled air does not
    get turned around an isolated cowl -- it simply becomes the external
    flow over the forebody, whose friction, form and wave drag are
    ALREADY counted above. Charging full additive drag on top of that
    double-counts the same air.

    The default is therefore 0.85: most of the pre-entry force is already
    represented in the forebody terms, and what remains is the residual
    from the streamtube being larger than the lip. Separating the two
    properly needs CFD; until then this term is reported SEPARATELY in
    DragBuildup so its leverage stays visible, and the campaign brackets
    it (0.0 = charge it fully, 1.0 = assume the forebody terms already
    contain it). At ~90 % spillage the bracket spans a large force, and
    saying so is more honest than picking a number quietly.

    Independent of the factor, the term itself is real and is currently
    charged to NOBODY in the model chain: ramjet-fp reports engine thrust
    and explicitly excludes installed inlet drag (its assumption A18), and
    the ancestor vehicle model has no concept of it.
    """
    if lip_area_m2 <= 0.0 or velocity_m_per_s <= 1.0:
        return 0.0
    q = 0.5 * density_kg_per_m3 * velocity_m_per_s ** 2
    a0 = captured_mass_flow_kg_per_s / max(
        density_kg_per_m3 * velocity_m_per_s, 1e-9)
    spilled_area = max(lip_area_m2 - a0, 0.0)
    return spilled_area * q * (1.0 - cowl_suction_recovery)


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------

class DragBuildup(NamedTuple):
    friction_n: float
    form_n: float
    base_n: float
    wave_n: float
    spillage_n: float
    wing_profile_n: float
    induced_n: float
    total_n: float
    required_lift_n: float
    cd0_equivalent_frontal: float
    """The flat frontal-referenced CD0 this build-up is EQUIVALENT to at
    this condition -- the number to compare against the ancestor's 0.10
    placeholder, and the headline of the drag attribution row."""


def body_wetted_area_m2(diameter_m: float, body_length_m: float) -> float:
    """Cylinder side area; nose/tail curvature raises and closes this to
    within a few percent for the fineness ratios in this class."""
    return pi * diameter_m * body_length_m


def total_drag_buildup(
    *,
    diameter_m: float,
    body_length_m: float,
    duct_exit_diameter_m: float,
    tail_length_m: float,
    wing_reference_area_m2: float,
    wing_thickness_ratio: float,
    wing_sweep_deg: float,
    velocity_m_per_s: float,
    density_kg_per_m3: float,
    temperature_k: float,
    mach: float,
    required_lift_n: float,
    oswald_efficiency: float,
    wing_aspect_ratio: float,
    engine_on: bool,
    captured_mass_flow_kg_per_s: float = 0.0,
    lip_area_m2: float = 0.0,
    cowl_suction_recovery: float = 0.85,
) -> DragBuildup:
    """Full component build-up at one flight condition.

    Single algebraic pass, no iteration -- the ancestor's "no ODE fanciness
    inside a physics call" discipline is preserved; what changes is that
    each term now has a physical driver instead of one flat constant.
    """
    q = 0.5 * density_kg_per_m3 * velocity_m_per_s ** 2
    frontal_area = pi * diameter_m ** 2 / 4.0
    if q <= 0.0 or velocity_m_per_s <= 0.0:
        return DragBuildup(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           required_lift_n, 0.0)

    # -- friction + form, body ------------------------------------------
    re_body = reynolds_number(density_kg_per_m3, velocity_m_per_s,
                              body_length_m, temperature_k)
    cf_body = skin_friction_coefficient(re_body, mach)
    s_wet_body = body_wetted_area_m2(diameter_m, body_length_m)
    friction_body = cf_body * q * s_wet_body
    ff_body = body_form_factor(body_length_m / max(diameter_m, 1e-9))
    form_body = friction_body * (ff_body - 1.0)

    # -- friction + form, wing (wetted ~ 2x planform) --------------------
    mean_chord = sqrt(max(wing_reference_area_m2, 1e-9)
                      / max(wing_aspect_ratio, 1e-9))
    re_wing = reynolds_number(density_kg_per_m3, velocity_m_per_s,
                              mean_chord, temperature_k)
    cf_wing = skin_friction_coefficient(re_wing, mach)
    s_wet_wing = 2.0 * wing_reference_area_m2
    ff_wing = wing_form_factor(wing_thickness_ratio, wing_sweep_deg, mach)
    wing_profile = cf_wing * ff_wing * q * s_wet_wing

    # -- base, wave, spillage -------------------------------------------
    d_base = base_diameter_m(diameter_m, tail_length_m, duct_exit_diameter_m)
    base = base_drag_n(base_area_m2(d_base, duct_exit_diameter_m,
                                    engine_on), q, mach)
    wave = wave_drag_coefficient(mach, diameter_m, body_length_m) \
        * q * frontal_area
    spill = spillage_drag_n(captured_mass_flow_kg_per_s, lip_area_m2,
                            density_kg_per_m3, velocity_m_per_s,
                            cowl_suction_recovery)
    if cowl_suction_recovery >= 1.0:
        spill = 0.0        # forebody terms assumed to contain it entirely

    # -- induced (unchanged form; lift is the driver) --------------------
    if wing_reference_area_m2 > 0.0 and required_lift_n != 0.0:
        induced = (required_lift_n ** 2) / (
            q * pi * oswald_efficiency * wing_aspect_ratio
            * wing_reference_area_m2)
    else:
        induced = 0.0

    total = (friction_body + form_body + base + wave + spill
             + wing_profile + induced)
    cd0_equiv = ((friction_body + form_body + base + wave + spill
                  + wing_profile) / (q * frontal_area)) if frontal_area > 0 \
        else 0.0
    return DragBuildup(friction_body, form_body, base, wave, spill,
                       wing_profile, induced, total, required_lift_n,
                       cd0_equiv)
