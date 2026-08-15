"""Fin flutter check — a structural gate on the aerodynamic design.

    PYTHONPATH=scripts/vsp_model .venv/Scripts/python scripts/vsp_model/flutter.py

WHY THIS IS IN THE PIPELINE. Aspect ratio is the cheapest lever on yaw stiffness
per unit fin area, so an aero-only optimiser drives AR up and t/c down without
limit. Flutter speed goes as ``sqrt((t/c)^3 / A^3)``, so that is precisely the
direction that destroys the fin. An AR 3.0, t/c 0.04 CFRP fin on this vehicle
flutters at **Mach 0.79** against a Mach 1.10 requirement -- it departs during
the dive. Nothing else in this project models flutter, root bending, or fin
attachment, so without this check there is nothing to stop the aero from
specifying an unflyable fin.

METHOD. The classical flat-plate fin flutter criterion (NACA TN 4197 form, the
standard rocketry/missile expression):

    V_f = a * sqrt( G / [ 1.337 A^3 P (lambda + 1) / (2 (A + 2) (t/c)^3) ] )

with ``A`` the exposed panel aspect ratio, ``lambda`` its taper ratio, ``P`` the
ambient static pressure, ``a`` the speed of sound and ``G`` the fin's shear
modulus. Low altitude is the WORST case because ``P`` appears in the
denominator -- and this vehicle reaches its peak Mach at ~194 m.

WHAT IT IS NOT. A flat-plate criterion, panel flutter only. It does not cover
fin-body coupling, bending-torsion divergence, attachment compliance (which
lowers the real flutter speed, sometimes a lot), or aeroelastic amplification
short of flutter. Treat a pass here as necessary, never sufficient, and treat
the margin as the design variable rather than the raw number.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from math import sqrt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

SHEAR_MODULUS_PA = {
    "cfrp": 5.0e9,
    "aluminium": 26.0e9,
    "titanium": 44.0e9,
    "steel": 79.0e9,
}
"""In-plane shear modulus. CFRP is quoted for a quasi-isotropic laminate; a
unidirectional layup is far worse in shear and must not be assumed here."""

REQUIRED_FLUTTER_MARGIN = 1.5
"""Flutter Mach / max flight Mach. 1.5 is the conventional minimum; the fin is a
single-point failure whose loss ends the flight, so this is not a place to trim."""

DEFAULT_MATERIAL = "cfrp"
"""Fins are carbon fibre (user directive, 2026-08-14).

This is the worst case the table offers: a quasi-isotropic CFRP laminate has
roughly a fifth the shear modulus of aluminium, and flutter speed goes as
sqrt(G). A fin that is comfortable in aluminium can be unflyable in CFRP at the
same planform."""

BONDED_ROOT_FACTOR = 0.85
"""Derate for a fin bonded and filleted to the skin, with NO through spar
(user directive, 2026-08-14).

The classical criterion assumes a RIGID root. A surface-bonded root is a
compliant spring instead: root rotation adds to the torsional degree of freedom
that drives flutter, so the real onset is below the flat-plate figure. 0.85 is a
deliberately mild, conventional allowance for that compliance -- the true value
depends on the bond line and fillet radius and can be considerably worse. It is
applied on top of, not instead of, the 1.5x margin.

The same constraint has a second consequence this module does not model: the
root bending moment must pass entirely through the bond and fillet. A high
aspect ratio fin has a large root moment, so AR is limited by the JOINT as well
as by flutter, and nothing in this project sizes that joint."""


def standard_atmosphere(altitude_m: float) -> tuple[float, float]:
    """ISA pressure (Pa) and speed of sound (m/s), troposphere."""

    temperature = 288.15 - 0.0065 * altitude_m
    pressure = 101325.0 * (temperature / 288.15) ** 5.25588
    return pressure, sqrt(1.4 * 287.053 * temperature)


@dataclass(frozen=True)
class FlutterResult:
    surface: str
    material: str
    aspect_ratio: float
    taper_ratio: float
    thickness_to_chord: float
    altitude_m: float
    flutter_mach: float
    """Already derated for root attachment -- see BONDED_ROOT_FACTOR."""
    required_mach: float
    attachment_factor: float = BONDED_ROOT_FACTOR

    @property
    def margin(self) -> float:
        return self.flutter_mach / self.required_mach if self.required_mach else float("inf")

    @property
    def passes(self) -> bool:
        return self.margin >= REQUIRED_FLUTTER_MARGIN

    @property
    def verdict(self) -> str:
        if self.flutter_mach < self.required_mach:
            return "FAIL - flutters below max flight Mach"
        return "PASS" if self.passes else "MARGINAL"


def flutter_mach(
    aspect_ratio: float,
    taper_ratio: float,
    thickness_to_chord: float,
    shear_modulus_pa: float,
    altitude_m: float,
    attachment_factor: float = BONDED_ROOT_FACTOR,
) -> float:
    """Flutter Mach of a fin at the given altitude, derated for root compliance."""

    pressure, speed_of_sound = standard_atmosphere(altitude_m)
    stiffness_demand = (
        1.337
        * aspect_ratio**3
        * pressure
        * (taper_ratio + 1.0)
        / (2.0 * (aspect_ratio + 2.0) * thickness_to_chord**3)
    )
    return attachment_factor * sqrt(shear_modulus_pa / stiffness_demand)


def check_panel(
    name: str,
    semispan_m: float,
    root_chord_m: float,
    tip_chord_m: float,
    thickness_to_chord: float,
    material: str,
    altitude_m: float,
    required_mach: float,
    attachment_factor: float = BONDED_ROOT_FACTOR,
) -> FlutterResult:
    panel_area = 0.5 * (root_chord_m + tip_chord_m) * semispan_m
    aspect_ratio = semispan_m**2 / panel_area
    taper = tip_chord_m / root_chord_m
    return FlutterResult(
        surface=name,
        material=material,
        aspect_ratio=aspect_ratio,
        taper_ratio=taper,
        thickness_to_chord=thickness_to_chord,
        altitude_m=altitude_m,
        flutter_mach=flutter_mach(
            aspect_ratio, taper, thickness_to_chord,
            SHEAR_MODULUS_PA[material], altitude_m, attachment_factor,
        ),
        required_mach=required_mach,
        attachment_factor=attachment_factor,
    )


def check_vehicle(
    geometry,
    material: str = DEFAULT_MATERIAL,
    altitude_m: float = 194.0,
    required_mach: float = 1.10,
) -> list[FlutterResult]:
    """Flutter check every lifting panel on the vehicle.

    Defaults are the V4 cutoff condition: Mach 1.100 at 194 m, which is the
    worst case because low altitude means high dynamic pressure for a given Mach.
    """

    results = []
    for panel in geometry.all_panels:
        results.append(
            check_panel(
                panel.name, panel.semispan_m, panel.root_chord_m, panel.tip_chord_m,
                panel.thickness_to_chord, material, altitude_m, required_mach,
            )
        )
    return results


def required_thickness_to_chord(
    aspect_ratio: float,
    taper_ratio: float,
    material: str,
    altitude_m: float,
    required_mach: float,
    margin: float = REQUIRED_FLUTTER_MARGIN,
    attachment_factor: float = BONDED_ROOT_FACTOR,
) -> float:
    """Minimum t/c that meets the flutter margin — the actionable output.

    Inverting for t/c rather than reporting a failure tells the designer what to
    change, and t/c is nearly free aerodynamically compared with losing fin area.
    """

    pressure, speed_of_sound = standard_atmosphere(altitude_m)
    target_speed = margin * required_mach * speed_of_sound / attachment_factor
    # V_f^2 = a^2 G (t/c)^3 * 2(A+2) / (1.337 A^3 P (lam+1))  -> solve for (t/c)^3
    cubed = (
        target_speed**2
        * 1.337
        * aspect_ratio**3
        * pressure
        * (taper_ratio + 1.0)
        / (speed_of_sound**2 * SHEAR_MODULUS_PA[material] * 2.0 * (aspect_ratio + 2.0))
    )
    return cubed ** (1.0 / 3.0)


def describe(results: list[FlutterResult]) -> str:
    if not results:
        return "Flutter: no lifting panels."
    first = results[0]
    lines = [
        f"Fin flutter ({first.material}, bonded root x{first.attachment_factor:.2f}, "
        f"{first.altitude_m:.0f} m, "
        f"need Mach >= {REQUIRED_FLUTTER_MARGIN:.1f} x {first.required_mach:.2f} "
        f"= {REQUIRED_FLUTTER_MARGIN * first.required_mach:.2f})",
        f"  {'surface':<14}{'AR':>6}{'t/c':>7}{'M_flutter':>11}{'margin':>9}  verdict",
    ]
    for result in results:
        lines.append(
            f"  {result.surface:<14}{result.aspect_ratio:>6.2f}"
            f"{result.thickness_to_chord:>7.3f}{result.flutter_mach:>11.2f}"
            f"{result.margin:>9.2f}  {result.verdict}"
        )
    worst = min(results, key=lambda r: r.margin)
    if not worst.passes:
        needed = required_thickness_to_chord(
            worst.aspect_ratio, worst.taper_ratio, worst.material,
            worst.altitude_m, worst.required_mach,
        )
        lines.append("")
        lines.append(
            f"  {worst.surface} is the binding panel. Raising its t/c to "
            f"{needed:.3f} would meet the {REQUIRED_FLUTTER_MARGIN:.1f}x margin."
        )
    return "\n".join(lines)


def main() -> int:
    from dataclasses import replace

    import mass_cg
    from geometry_inputs import GeometryInputs
    from sizing_input import DEFAULT_SIZING_JSON, SizingInput
    from vehicle_geometry import build_vehicle_geometry

    sizing = SizingInput.from_json(DEFAULT_SIZING_JSON)
    inputs = GeometryInputs()
    mass = mass_cg.vehicle_mass_properties(sizing, inputs)
    geometry = build_vehicle_geometry(sizing, inputs, cg_x_m=mass.cg_x_m)
    print(describe(check_vehicle(geometry)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
