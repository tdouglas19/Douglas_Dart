"""Control surface sizing: partial-chord flap, or all-moving surface?

    PYTHONPATH=scripts/vsp_model .venv/Scripts/python scripts/vsp_model/control.py

A deliberately small answer to a binary question. The vehicle's static margin is
stiff (~3 calibres), so every commanded manoeuvre needs a real deflection to hold
it; the only question is whether a hinged flap has the authority or whether the
whole surface has to move.

METHOD. Thin-aerofoil flap effectiveness

    tau = 1 - (theta_f - sin theta_f) / pi ,   cos theta_f = 2 (Cf/C) - 1

is the fraction of the surface's own lift slope that a trailing-edge flap
achieves per degree of deflection. An all-moving surface has tau = 1 by
definition. Real flaps reach roughly 85% of the theoretical value once viscous
effects at the hinge line are counted, which is applied here.

DEFLECTION LIMITS. An all-moving surface stalls as a whole, so it is limited to
about the section stall angle (~15 deg). A flap only changes camber and keeps
working further, to about 25 deg, though effectiveness tails off above ~20 deg.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from math import acos, cos, degrees, pi, radians, sin
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

VISCOUS_KNOCKDOWN = 0.85
"""Fraction of thin-aerofoil flap effectiveness a real hinged surface achieves."""

ALL_MOVING_LIMIT_DEG = 15.0
"""An all-moving surface stalls bodily, so its useful range is the section stall
angle. Past this the surface is stalled and control reverses or is lost."""

FLAP_LIMIT_DEG = 25.0
"""A flap keeps producing lift further than an all-moving surface because it is
changing camber, not incidence -- but effectiveness falls off above ~20 deg."""


def flap_effectiveness(chord_fraction: float) -> float:
    """``tau`` for a trailing-edge flap of the given chord fraction."""

    if chord_fraction >= 0.999:
        return 1.0
    theta = acos(2.0 * chord_fraction - 1.0)
    return VISCOUS_KNOCKDOWN * (1.0 - (theta - sin(theta)) / pi)


@dataclass(frozen=True)
class ControlCase:
    name: str
    required_deflection_deg: float
    """Deflection an ALL-MOVING surface would need."""


@dataclass(frozen=True)
class ControlVerdict:
    chord_fraction: float
    effectiveness: float
    limit_deg: float
    cases: tuple[tuple[str, float, bool], ...]

    @property
    def all_cases_fit(self) -> bool:
        return all(ok for _, _, ok in self.cases)

    @property
    def label(self) -> str:
        return (
            "all-moving"
            if self.chord_fraction >= 0.999
            else f"{self.chord_fraction * 100:.0f}% chord flap"
        )


def evaluate(cases: list[ControlCase], chord_fraction: float) -> ControlVerdict:
    tau = flap_effectiveness(chord_fraction)
    limit = ALL_MOVING_LIMIT_DEG if chord_fraction >= 0.999 else FLAP_LIMIT_DEG
    rows = []
    for case in cases:
        needed = case.required_deflection_deg / tau
        rows.append((case.name, needed, needed <= limit))
    return ControlVerdict(chord_fraction, tau, limit, tuple(rows))


def required_deflections(sizing, inputs, mass, mach: float = 0.5) -> list[ControlCase]:
    """Deflection each design manoeuvre needs from an all-moving surface."""

    import barrowman
    from vehicle_geometry import build_vehicle_geometry

    d = sizing.body_diameter_m
    frontal = sizing.frontal_area_m2
    s_ref = 0.152499
    geometry = build_vehicle_geometry(sizing, inputs, cg_x_m=mass.cg_x_m)
    results = barrowman.analyse(sizing, inputs, mass.cg_x_m, mach=mach)
    pitch = results["pitch"]
    cn_alpha = pitch.cn_alpha_total
    cm_alpha = -cn_alpha * (pitch.x_cp_m - mass.cg_x_m) / d
    fin_cn = sum(c.cn_alpha for c in pitch.contributions if c.name.startswith("Fin"))
    fin_x = next(c.x_cp_m for c in pitch.contributions if c.name.startswith("Fin"))
    arm = (fin_x - mass.cg_x_m) / d
    cl_alpha = cn_alpha * frontal / s_ref

    def deflection_for(load_factor: float, velocity: float, mass_kg: float) -> float:
        q = 0.5 * 1.225 * velocity**2
        lift = load_factor * mass_kg * 9.80665
        alpha = lift / (q * s_ref) / cl_alpha
        return degrees(abs(cm_alpha) * alpha / (fin_cn * arm))

    return [
        ControlCase("pull-out 3 g @ M0.49", deflection_for(3.0, 166.0, 20.5)),
        ControlCase("spiral climb 30 deg bank", deflection_for(1.155, 95.0, 22.0)),
        ControlCase("pull-out 6 g (structural cap)", deflection_for(6.0, 166.0, 20.5)),
        ControlCase("glide trim, 10.8 deg alpha", degrees(abs(cm_alpha) * radians(10.8) / (fin_cn * arm))),
    ]


def describe(cases: list[ControlCase], fractions=(0.25, 0.40, 1.0)) -> str:
    verdicts = [evaluate(cases, f) for f in fractions]
    lines = ["Control surface: flap or all-moving?", ""]
    header = f"  {'manoeuvre':<32}" + "".join(f"{v.label:>18}" for v in verdicts)
    lines.append(header)
    lines.append(
        f"  {'(deflection needed)':<32}"
        + "".join(f"{'tau ' + format(v.effectiveness, '.2f'):>18}" for v in verdicts)
    )
    for index, case in enumerate(cases):
        row = f"  {case.name:<32}"
        for verdict in verdicts:
            _, needed, ok = verdict.cases[index]
            row += f"{format(needed, '.1f') + ' deg' + ('' if ok else ' X'):>18}"
        lines.append(row)
    lines.append("")
    lines.append(
        f"  limits: flap {FLAP_LIMIT_DEG:.0f} deg, all-moving {ALL_MOVING_LIMIT_DEG:.0f} deg"
        "   (X = beyond limit)"
    )
    workable = [v for v in verdicts if v.all_cases_fit]
    if workable:
        best = min(workable, key=lambda v: v.chord_fraction)
        lines.append(f"  VERDICT: {best.label} is the smallest option that covers every case.")
    else:
        lines.append(
            "  VERDICT: no option covers every case -- the binding manoeuvre needs "
            "either less static margin or a larger surface."
        )
    return "\n".join(lines)


def main() -> int:
    from dataclasses import replace

    import mass_cg
    from geometry_inputs import GeometryInputs
    from sizing_input import DEFAULT_SIZING_JSON, SizingInput

    sizing = SizingInput.from_json(DEFAULT_SIZING_JSON)
    inputs = replace(
        GeometryInputs(),
        fin_area_ratio=0.80,
        wing_root_le_x_m=1.60,
        fin_aspect_ratio=2.1,
        fin_thickness_to_chord=0.08,
        fin_clocking_deg_explicit=(60.0, 120.0),
    )
    mass = mass_cg.vehicle_mass_properties(sizing, inputs)
    print(describe(required_deflections(sizing, inputs, mass)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
