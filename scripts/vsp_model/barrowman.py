"""Analytic static stability for a body-plus-fins dart (Barrowman method).

    PYTHONPATH=scripts/vsp_model .venv/Scripts/python scripts/vsp_model/barrowman.py

WHY THIS EXISTS. VSPAERO could not size this tail. Three independent attempts
failed: the vortex-lattice solve does not converge at the fin/body junction (and
refining the mesh made it worse -- 0 of 4 mount depths converged at the finest
tessellation), the panel method converges but returns intermittent wild outliers
(Cn_beta = -13.95 against +0.49 and +0.36 from sibling solves of the same
geometry), and a slender-body cross-check that appeared to validate the baseline
turned out to be a coincidence under proper decomposition.

Barrowman is the right tool for this configuration and always was: a slender
axisymmetric body with fins, subsonic, small angle of attack. It is analytic, has
no mesh to be degenerate, and every term is separately checkable.

REFERENCES. Barrowman, "The Practical Calculation of the Aerodynamic
Characteristics of Slender Finned Vehicles" (1967); the fin normal-force slope
and body transition terms are the standard forms.

WHAT IT DOES NOT DO. Inviscid, linear, subsonic. No viscous body crossflow lift
(real and destabilizing on a fineness-10.6 body at incidence, so this is
optimistic), no wing/fin interference beyond the classic body-fin factor, no
compressibility beyond an optional Prandtl-Glauert correction, no dynamic
derivatives. It gives CN_alpha and a centre of pressure, which is what sizing a
tail requires.

ONE ADAPTATION, and it matters. Classic Barrowman gives a closed nose
CN_alpha = 2. This vehicle's nose is an OPEN INLET, which does not generate that
normal force -- the flow enters the duct rather than turning around a closed
body. The external cowl is treated instead as a conical transition from the lip
diameter to the body diameter, which is the standard treatment for a ducted
forebody. Using the closed-nose term here would add a large spurious
destabilizing force at the very front of the vehicle.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from math import cos, radians, sin, sqrt, tan
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from geometry_inputs import GeometryInputs  # noqa: E402
from sizing_input import DEFAULT_SIZING_JSON, SizingInput  # noqa: E402


@dataclass(frozen=True)
class Contribution:
    """One term's normal-force slope and centre of pressure."""

    name: str
    cn_alpha: float
    """Per radian, referenced to the BODY cross-sectional area."""
    x_cp_m: float

    def moment_slope(self, x_cg_m: float, reference_length_m: float) -> float:
        """Contribution to Cm_alpha about the CG. Negative is stabilising."""

        return -self.cn_alpha * (self.x_cp_m - x_cg_m) / reference_length_m


def transition_cn_alpha(
    fore_diameter_m: float, aft_diameter_m: float, reference_diameter_m: float
) -> float:
    """Normal-force slope of a conical body transition.

    ``CN_alpha = 2[(d_aft/d_ref)^2 - (d_fore/d_ref)^2]``. Positive for an
    expansion (nose cowl), negative for a contraction (boattail).
    """

    return 2.0 * (
        (aft_diameter_m / reference_diameter_m) ** 2
        - (fore_diameter_m / reference_diameter_m) ** 2
    )


def transition_x_cp_m(
    start_x_m: float, length_m: float, fore_diameter_m: float, aft_diameter_m: float
) -> float:
    """Centre of pressure of a conical transition (Barrowman)."""

    if abs(aft_diameter_m - fore_diameter_m) < 1e-12:
        return start_x_m + 0.5 * length_m
    ratio = fore_diameter_m / aft_diameter_m
    return start_x_m + (length_m / 3.0) * (1.0 + (1.0 - ratio) / (1.0 - ratio**2))


def fin_cn_alpha(
    panel_count: int,
    semispan_m: float,
    root_chord_m: float,
    tip_chord_m: float,
    sweep_le_deg: float,
    body_radius_m: float,
    mach: float = 0.0,
) -> float:
    """Normal-force slope of a set of ``panel_count`` fins, with body interference.

    Barrowman's fin term, referenced to body cross-sectional area:

        CN_a = 4 N (s/d)^2 / (1 + sqrt(1 + (2 L / (c_r + c_t))^2))

    where ``L`` is the mid-chord sweep length. The body-fin interference factor
    ``K = 1 + r/(s + r)`` accounts for the fin carrying some of the body's flow.

    Prandtl-Glauert is applied to the effective span, which is the usual
    subsonic compressibility treatment for this method.
    """

    if panel_count <= 0 or semispan_m <= 0.0:
        return 0.0
    diameter = 2.0 * body_radius_m
    beta = sqrt(max(1.0 - mach * mach, 1e-6))
    effective_semispan = semispan_m / beta

    # Mid-chord sweep length: the span-wise distance along the mid-chord line.
    mid_chord_offset = (
        semispan_m * tan(radians(sweep_le_deg)) + 0.5 * (tip_chord_m - root_chord_m)
    )
    mid_chord_length = sqrt(semispan_m**2 + mid_chord_offset**2)

    denominator = 1.0 + sqrt(
        1.0 + (2.0 * mid_chord_length / (root_chord_m + tip_chord_m)) ** 2
    )
    base = 4.0 * panel_count * (effective_semispan / diameter) ** 2 / denominator
    interference = 1.0 + body_radius_m / (semispan_m + body_radius_m)
    return base * interference


def fin_x_cp_m(
    root_le_x_m: float,
    root_chord_m: float,
    tip_chord_m: float,
    semispan_m: float,
    sweep_le_deg: float,
) -> float:
    """Centre of pressure of a trapezoidal fin (Barrowman)."""

    sweep_offset = semispan_m * tan(radians(sweep_le_deg))
    chord_sum = root_chord_m + tip_chord_m
    sweep_term = sweep_offset * (root_chord_m + 2.0 * tip_chord_m) / (3.0 * chord_sum)
    chord_term = (
        chord_sum - root_chord_m * tip_chord_m / chord_sum
    ) / 6.0
    return root_le_x_m + sweep_term + chord_term


@dataclass(frozen=True)
class RollResult:
    """Roll behaviour of a cruciform finned body.

    ROLL HAS NO STATIC MARGIN, and quoting one would be meaningless. An
    axisymmetric body with symmetric fins has no restoring moment in roll angle
    at all -- roll is neutrally stable by construction. What decides whether the
    vehicle is well behaved in roll is DAMPING (``Cl_p`` < 0, always true for
    fins) and how fast that damping acts, i.e. the roll-mode time constant.

    The design risk in roll is therefore not divergence but roll RATE: fin
    misalignment, asymmetric fillets or a bent panel drive a steady roll that the
    damping only limits, it does not remove. A short time constant means the
    vehicle reaches that steady rate quickly.
    """

    cl_p: float
    """Roll damping derivative, per radian of wing-tip helix angle. Negative."""
    roll_time_constant_s: float | None
    i_xx_kg_m2: float
    dynamic_pressure_pa: float
    velocity_m_s: float
    fin_misalignment_deg: float
    steady_roll_rate_deg_s: float | None
    """Steady roll rate from a commanded fin misalignment -- the number that
    actually matters, since damping cannot null it."""


def roll_damping(geometry, mach: float = 0.5) -> float:
    """``Cl_p`` by strip theory over the exposed fin panels.

    Each spanwise strip at radius y sees local incidence ``p*y/V``; its lift acts
    at moment arm y, so the integrand is ``c(y) * y^2``. Wing panels are included
    -- they damp roll exactly as fins do.
    """

    from vehicle_geometry import build_fin_panels, build_wing_panels

    sizing, inputs = geometry.sizing, geometry.inputs
    s_ref, b_ref = geometry.references.area_m2, geometry.references.span_m
    frontal = sizing.frontal_area_m2
    body_radius = 0.5 * sizing.body_diameter_m

    total = 0.0
    panels = list(build_wing_panels(sizing, inputs)) + list(build_fin_panels(sizing, inputs))
    for panel in panels:
        count = panel.panel_count
        area = panel.exposed_area_m2
        if area <= 0.0:
            continue
        cn_alpha = fin_cn_alpha(
            1, panel.semispan_m, panel.root_chord_m, panel.tip_chord_m,
            panel.le_sweep_deg, body_radius, mach,
        )
        # Integrate c(y) y^2 across the exposed span, chord linear root->tip.
        root, tip, span = panel.root_chord_m, panel.tip_chord_m, panel.semispan_m
        steps = 60
        integral = 0.0
        for index in range(steps):
            frac = (index + 0.5) / steps
            y = body_radius + frac * span
            chord = root + frac * (tip - root)
            integral += chord * y * y * (span / steps)
        total += count * (cn_alpha * frontal / area) * integral

    return -2.0 * total / (s_ref * b_ref**2)


def analyse_roll(
    geometry,
    mass,
    mach: float = 0.5,
    altitude_m: float = 194.0,
    fin_misalignment_deg: float = 0.25,
) -> RollResult:
    """Roll damping, roll-mode time constant, and the roll rate a small fin
    misalignment produces.

    ``fin_misalignment_deg`` defaults to 0.25 deg, a realistic build tolerance
    for a fin bonded to a skin without a spar or jig feature to key off.
    """

    from math import pi as _pi

    density, speed_of_sound = _atmosphere(altitude_m)
    velocity = mach * speed_of_sound
    q_bar = 0.5 * density * velocity**2
    s_ref, b_ref = geometry.references.area_m2, geometry.references.span_m
    cl_p = roll_damping(geometry, mach)

    tau = None
    steady_rate = None
    if cl_p < 0.0 and velocity > 1.0:
        tau = -2.0 * mass.i_xx_kg_m2 * velocity / (q_bar * s_ref * b_ref**2 * cl_p)
        # A misaligned fin set makes a rolling moment Cl_delta * delta; at steady
        # state that is balanced by damping: p = -Cl_delta*delta / Cl_p * 2V/b.
        cl_delta = _roll_control_power(geometry, mach)
        delta = fin_misalignment_deg * _pi / 180.0
        helix = -cl_delta * delta / cl_p
        steady_rate = helix * 2.0 * velocity / b_ref * 180.0 / _pi
    return RollResult(
        cl_p=cl_p,
        roll_time_constant_s=tau,
        i_xx_kg_m2=mass.i_xx_kg_m2,
        dynamic_pressure_pa=q_bar,
        velocity_m_s=velocity,
        fin_misalignment_deg=fin_misalignment_deg,
        steady_roll_rate_deg_s=steady_rate,
    )


def _roll_control_power(geometry, mach: float) -> float:
    """``Cl_delta``: rolling moment from all fins deflected together by one radian."""

    from vehicle_geometry import build_fin_panels

    sizing, inputs = geometry.sizing, geometry.inputs
    s_ref, b_ref = geometry.references.area_m2, geometry.references.span_m
    frontal = sizing.frontal_area_m2
    body_radius = 0.5 * sizing.body_diameter_m

    total = 0.0
    for panel in build_fin_panels(sizing, inputs):
        cn_alpha = fin_cn_alpha(
            1, panel.semispan_m, panel.root_chord_m, panel.tip_chord_m,
            panel.le_sweep_deg, body_radius, mach,
        )
        arm = body_radius + panel.semispan_m / 3.0 * (
            (panel.root_chord_m + 2.0 * panel.tip_chord_m)
            / (panel.root_chord_m + panel.tip_chord_m)
        )
        total += cn_alpha * frontal * arm
    return total / (s_ref * b_ref)


def _atmosphere(altitude_m: float) -> tuple[float, float]:
    temperature = 288.15 - 0.0065 * altitude_m
    pressure = 101325.0 * (temperature / 288.15) ** 5.25588
    return pressure / (287.053 * temperature), sqrt(1.4 * 287.053 * temperature)


@dataclass(frozen=True)
class StabilityResult:
    contributions: tuple[Contribution, ...]
    cn_alpha_total: float
    x_cp_m: float
    x_cg_m: float
    reference_length_m: float
    axis: str

    @property
    def static_margin_calibres(self) -> float:
        """(x_cp - x_cg) / body diameter. Positive is stable.

        Calibres, not reference chords: it is the convention for finned bodies
        and 1-2 calibres is the usual target."""

        return (self.x_cp_m - self.x_cg_m) / self.reference_length_m

    @property
    def stable(self) -> bool:
        return self.x_cp_m > self.x_cg_m


def solve_axis(contributions: list[Contribution], x_cg_m: float,
               reference_length_m: float, axis: str) -> StabilityResult:
    total = sum(c.cn_alpha for c in contributions)
    if abs(total) < 1e-12:
        x_cp = x_cg_m
    else:
        x_cp = sum(c.cn_alpha * c.x_cp_m for c in contributions) / total
    return StabilityResult(
        contributions=tuple(contributions),
        cn_alpha_total=total,
        x_cp_m=x_cp,
        x_cg_m=x_cg_m,
        reference_length_m=reference_length_m,
        axis=axis,
    )


def body_contributions(
    sizing: SizingInput, inputs: GeometryInputs, aft_diameter_m: float
) -> list[Contribution]:
    """Nose cowl and boattail, as conical transitions.

    NO closed-nose term -- see the module docstring. The cowl expands from the
    open lip to the body diameter and the boattail contracts to the base; both
    are transitions.
    """

    d_ref = sizing.body_diameter_m
    lip = sizing.inlet_lip_diameter_m
    nose_length = sizing.nose_fairing_length_m

    cowl = Contribution(
        name="nose cowl (open inlet, lip->body)",
        cn_alpha=transition_cn_alpha(lip, d_ref, d_ref),
        x_cp_m=transition_x_cp_m(0.0, nose_length, lip, d_ref),
    )
    boattail = Contribution(
        name="boattail (body->base)",
        cn_alpha=transition_cn_alpha(d_ref, aft_diameter_m, d_ref),
        x_cp_m=transition_x_cp_m(
            sizing.boattail_start_x_m, sizing.boattail_length_m, d_ref, aft_diameter_m
        ),
    )
    return [cowl, boattail]


def analyse(
    sizing: SizingInput,
    inputs: GeometryInputs,
    x_cg_m: float,
    mach: float = 0.5,
    aft_diameter_m: float | None = None,
) -> dict[str, StabilityResult]:
    """Static stability in pitch and yaw for the configured vehicle."""

    from vehicle_geometry import build_body_stations, build_fin_panels, build_wing_panels

    if aft_diameter_m is None:
        aft_diameter_m = build_body_stations(sizing, inputs)[-1].diameter_m
    body_radius = 0.5 * sizing.body_diameter_m

    pitch = body_contributions(sizing, inputs, aft_diameter_m)
    yaw = body_contributions(sizing, inputs, aft_diameter_m)

    # --- Wing: a 2-panel horizontal surface. Pitch only.
    for panel in build_wing_panels(sizing, inputs):
        cn = fin_cn_alpha(
            2, panel.semispan_m, panel.root_chord_m, panel.tip_chord_m,
            panel.le_sweep_deg, body_radius, mach,
        )
        pitch.append(
            Contribution(
                "wing",
                cn,
                fin_x_cp_m(panel.root_le_x_m, panel.root_chord_m, panel.tip_chord_m,
                           panel.semispan_m, panel.le_sweep_deg),
            )
        )

    # --- Fins: each panel projected onto the axis it actually works.
    # A panel clocked at theta contributes cos^2(theta) of its normal force to
    # yaw and sin^2(theta) to pitch -- which is exactly why a cruciform splits
    # evenly and a vertical-biased tail does not.
    for panel in build_fin_panels(sizing, inputs):
        cn = fin_cn_alpha(
            1, panel.semispan_m, panel.root_chord_m, panel.tip_chord_m,
            panel.le_sweep_deg, body_radius, mach,
        )
        x_cp = fin_x_cp_m(panel.root_le_x_m, panel.root_chord_m, panel.tip_chord_m,
                          panel.semispan_m, panel.le_sweep_deg)
        # Clocking 0 deg puts the panel in the HORIZONTAL plane (extending in +Y,
        # like the wing), so its lift acts along Z and works PITCH. Clocking
        # 90 deg puts it vertical (extending in +Z), so its lift acts along Y and
        # works YAW. Hence pitch takes cos^2 and yaw takes sin^2 -- getting these
        # the wrong way round is invisible on a 45 deg cruciform, where both are
        # 0.5, and inverts the answer on any biased tail.
        theta = radians(panel.clocking_deg)
        pitch_share, yaw_share = cos(theta) ** 2, sin(theta) ** 2
        if pitch_share > 1e-9:
            pitch.append(Contribution(f"{panel.name} (pitch)", cn * pitch_share, x_cp))
        if yaw_share > 1e-9:
            yaw.append(Contribution(f"{panel.name} (yaw)", cn * yaw_share, x_cp))

    return {
        "pitch": solve_axis(pitch, x_cg_m, sizing.body_diameter_m, "pitch"),
        "yaw": solve_axis(yaw, x_cg_m, sizing.body_diameter_m, "yaw"),
    }


def describe(result: StabilityResult) -> str:
    lines = [
        f"{result.axis.upper()}  (CN_alpha referenced to body frontal area)",
        f"  {'term':<34}{'CN_alpha':>10}{'x_cp (mm)':>12}",
    ]
    for c in result.contributions:
        lines.append(f"  {c.name:<34}{c.cn_alpha:>10.4f}{c.x_cp_m * 1e3:>12.1f}")
    lines.append(f"  {'TOTAL':<34}{result.cn_alpha_total:>10.4f}{result.x_cp_m * 1e3:>12.1f}")
    lines.append(
        f"  CG {result.x_cg_m * 1e3:.1f} mm -> static margin "
        f"{result.static_margin_calibres:+.3f} calibres  "
        f"[{'STABLE' if result.stable else 'UNSTABLE'}]"
    )
    return "\n".join(lines)


def main() -> int:
    import mass_cg

    sizing = SizingInput.from_json(DEFAULT_SIZING_JSON)
    inputs = GeometryInputs()
    mass = mass_cg.vehicle_mass_properties(sizing, inputs)
    results = analyse(sizing, inputs, mass.cg_x_m, mach=0.5)
    print(f"Douglas Dart V4 -- Barrowman static stability, M 0.50")
    print(f"CG {mass.cg_x_m * 1e3:.1f} mm at release, body diameter "
          f"{sizing.body_diameter_m * 1e3:.1f} mm\n")
    for axis in ("pitch", "yaw"):
        print(describe(results[axis]))
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
