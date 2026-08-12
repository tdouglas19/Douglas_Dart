"""Render a labeled axisymmetric side-profile cross section of the vehicle.

OpenVSP is the source of truth for the external mold line and flow-through
engine behavior (see `openvsp_geometry.py`), but it does not model or lay out
internal components -- there is no internal-layout model anywhere in this
codebase (no combustor station, no fuel-tank subdivision, no avionics
placement). This script draws two different kinds of lines and is careful to
keep them visually distinct:

- SOLID outlines: the real, configured outer mold line and flow-through
  geometry, reusing `openvsp_geometry.py`'s own tested station functions
  (`body_stations`, `shell_stations`, `mount_radius_m`, `RamInletConfig`'s cowl
  ratios) rather than re-deriving the geometry independently.
- DASHED/HATCHED schematic blocks: illustrative internal component
  *placeholders* (selector/combustor, fuel tank, nozzle duct) sized from
  configured volumes/diameters where possible, but NOT positioned by any real
  internal-layout model. Treat their axial position as illustrative only.

Run: python scripts/vehicle_cross_section.py [--config path] [--output path.svg]
"""

from __future__ import annotations

import argparse
import sys
from math import pi, sqrt
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

from douglas_dart.config import ReferenceCase, load_reference_case
from douglas_dart.openvsp_geometry import body_stations, mount_radius_m, shell_stations

# Palette placeholders swapped for CSS var() references by the dashboard build
# step (see docs/governing_equations_integration.md-adjacent dashboard script);
# kept as literal hex here so this script also produces a standalone,
# self-contained SVG on its own.
_INK = "#172229"
_MUTED = "#5b6b78"
_ACCENT = "#c97a2e"
_ACCENT2 = "#2f8f86"
_GRID = "#d3dade"
_PANEL = "#ffffff"


def _schematic_chamber_diameter_m(volume_m3: float, length_m: float) -> float:
    area_m2 = volume_m3 / max(length_m, 1e-6)
    return 2.0 * sqrt(max(area_m2, 0.0) / pi)


# The real vehicle is a ~2.4 m long, ~0.21 m diameter body -- an ~11:1
# fineness ratio that renders as an unreadable sliver at true aspect ratio.
# Radii (not lengths) are exaggerated by this factor purely for legibility;
# the figure caption states this explicitly so it is never mistaken for the
# real, slender proportions (see docs/design_convergence.md's fineness_ratio).
RADIAL_EXAGGERATION = 3.2


def render(case: ReferenceCase) -> plt.Figure:
    geometry = case.geometry
    vehicle = case.vehicle
    selector = case.selector
    nozzle = case.nozzle
    ram_inlet = geometry.ram_inlet
    k = RADIAL_EXAGGERATION

    stations = body_stations(case)
    shell = shell_stations(case)

    intake_d = selector.circular_intake_diameter_m
    ram_length = ram_inlet.length_m(intake_d)
    lip_d = ram_inlet.lip_diameter_m(intake_d)

    fig, ax = plt.subplots(figsize=(12, 5.4), dpi=150)
    fig.patch.set_alpha(0.0)
    ax.set_facecolor("none")

    callouts: list[tuple[float, float, str]] = []  # (x, y, legend text), numbered on the plot

    def mark(x: float, y: float, text: str) -> None:
        index = len(callouts) + 1
        ax.annotate(
            str(index),
            xy=(x, y),
            fontsize=7,
            color=_PANEL,
            ha="center",
            va="center",
            zorder=10,
            bbox=dict(boxstyle="circle,pad=0.15", facecolor=_INK, edgecolor="none"),
        )
        callouts.append((x, y, text))

    # --- Real, configured outer mold line (SOLID) --------------------------
    body_x = [s.x_location_m for s in stations]
    body_r = [s.diameter_m / 2.0 * k for s in stations]
    # Ram-inlet lip fairing ahead of the nose, exactly the linear fairing
    # openvsp_geometry.py's _configure_ram_inlet itself uses.
    inlet_x = [-ram_length, 0.0]
    inlet_r = [lip_d / 2.0 * k, intake_d / 2.0 * k]
    full_x = inlet_x + body_x
    full_r = inlet_r + body_r
    ax.plot(full_x, full_r, color=_INK, linewidth=1.8, zorder=5, solid_joinstyle="round")
    ax.plot(full_x, [-r for r in full_r], color=_INK, linewidth=1.8, zorder=5, solid_joinstyle="round")

    # --- Fin-can shell (SOLID, real configured annulus) ---------------------
    shell_x = [s.x_location_m for s in shell]
    shell_r = [s.diameter_m / 2.0 * k for s in shell]
    ax.plot(shell_x, shell_r, color=_ACCENT2, linewidth=1.3, zorder=4)
    ax.plot(shell_x, [-r for r in shell_r], color=_ACCENT2, linewidth=1.3, zorder=4)

    # --- Lifting surfaces / fins (SOLID, real configured span+chord+x) -----
    surface_specs = (
        ("Lifting surface", geometry.lifting_surface, geometry.lifting_surface_count),
        ("Fin", geometry.fin, geometry.fin_count),
    )
    for label, planform, count in surface_specs:
        if count == 0:
            continue
        root_r = mount_radius_m(case, planform.x_location_m) * k
        tip_r = root_r + planform.exposed_semispan_m * k
        x0 = planform.x_location_m
        x1 = x0 + planform.root_chord_m
        for sign in (1.0, -1.0):
            ax.plot([x0, x1], [sign * root_r, sign * root_r], color=_ACCENT, linewidth=1.1, zorder=6)
            ax.plot([x0, x0], [sign * root_r, sign * tip_r], color=_ACCENT, linewidth=0.9, linestyle=(0, (2, 2)), zorder=6)
        mark(x0 + planform.root_chord_m * 0.5, root_r, f"{label}: x={x0:.2f} m, root chord {planform.root_chord_m*1000:.0f} mm, {count}x")

    # --- Schematic internal components (DASHED, illustrative only) ---------
    schematic_kwargs = dict(edgecolor=_MUTED, facecolor="none", linewidth=1.0, linestyle=(0, (4, 3)), zorder=3)

    chamber_length = max(geometry.forebody_transition_length_m * 0.85, 0.05)
    chamber_d = min(
        _schematic_chamber_diameter_m(case.pulsejet.chamber_volume_m3, chamber_length),
        vehicle.body_diameter_m * 0.9,
    )
    chamber_x0 = 0.02
    ax.add_patch(
        plt.Rectangle((chamber_x0, -chamber_d / 2.0 * k), chamber_length, chamber_d * k, **schematic_kwargs)
    )
    mark(chamber_x0 + chamber_length / 2.0, 0.0, "Selector + pulsejet/ramjet combustion chamber -- SCHEMATIC placement, sized from chamber_volume_m3 only")

    fuel_tank_x0 = geometry.shell.start_x_m + geometry.shell.forward_taper_length_m + 0.02
    fuel_tank_x1 = geometry.shell.end_x_m - geometry.shell.aft_taper_length_m - 0.02
    inner_r = vehicle.body_diameter_m / 2.0 * k
    outer_r = inner_r + geometry.shell.radial_offset_m * k
    if fuel_tank_x1 > fuel_tank_x0:
        for sign in (1.0, -1.0):
            ax.add_patch(
                plt.Rectangle(
                    (fuel_tank_x0, sign * inner_r if sign > 0 else -outer_r),
                    fuel_tank_x1 - fuel_tank_x0,
                    outer_r - inner_r,
                    hatch="///",
                    **{**schematic_kwargs, "edgecolor": _ACCENT2, "linewidth": 0.6},
                )
            )
        mark(fuel_tank_x0 + (fuel_tank_x1 - fuel_tank_x0) * 0.5, outer_r * 0.5, "Fuel tank / avionics annulus -- SCHEMATIC, real shell envelope but no internal subdivision model")

    throat_r = nozzle.throat_diameter_m / 2.0 * k
    exit_r = nozzle.throat_diameter_m * sqrt(nozzle.exit_to_throat_area_ratio) / 2.0 * k
    nozzle_x0 = geometry.aft_taper_start_m
    nozzle_x1 = vehicle.body_length_m
    throat_x = nozzle_x0 + (nozzle_x1 - nozzle_x0) * 0.35
    for sign in (1.0, -1.0):
        ax.plot(
            [nozzle_x0, throat_x, nozzle_x1],
            [sign * throat_r * 1.3, sign * throat_r, sign * exit_r],
            color=_MUTED,
            linewidth=1.0,
            linestyle=(0, (4, 3)),
            zorder=3,
        )
    mark(throat_x, 0.0, f"Nozzle duct -- SCHEMATIC contour; throat {nozzle.throat_diameter_m*1000:.0f} mm / exit {nozzle.throat_diameter_m*sqrt(nozzle.exit_to_throat_area_ratio)*1000:.0f} mm diameters are real")

    # --- Dimension callouts (kept to the essentials, outside the profile) --
    def dim(x0, x1, y, text):
        ax.annotate("", xy=(x1, y), xytext=(x0, y), arrowprops=dict(arrowstyle="<->", color=_MUTED, linewidth=0.8))
        ax.text((x0 + x1) / 2.0, y + 0.02, text, fontsize=7.5, color=_MUTED, ha="center")

    top = max(outer_r, body_r[1]) + 0.30
    dim(-ram_length, vehicle.body_length_m, top, f"true overall length {vehicle.body_length_m + ram_length:.2f} m (radii shown at {k:.1f}x for legibility)")
    ax.text(
        vehicle.body_length_m * 0.5, outer_r + 0.08,
        f"true body diameter {vehicle.body_diameter_m*1000:.0f} mm",
        fontsize=7.5, color=_INK, ha="center",
    )
    ax.text(
        -ram_length * 0.5, inlet_r[0] + 0.09,
        f"true intake {intake_d*1000:.0f} mm",
        fontsize=7.5, color=_INK, ha="center",
    )

    # --- Numbered legend, printed below the profile to avoid overlap -------
    legend_y0 = -(top + 0.05)
    for index, (_x, _y, text) in enumerate(callouts, start=1):
        ax.text(
            -ram_length - 0.02,
            legend_y0 - 0.16 * (index - 1),
            f"{index}. {text}",
            fontsize=7,
            color=_INK,
            ha="left",
            va="top",
        )

    ax.set_xlim(-ram_length - 0.05, vehicle.body_length_m + 0.1)
    ax.set_ylim(legend_y0 - 0.16 * len(callouts) - 0.05, top + 0.1)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout(pad=0.2)
    return fig


def render_svg_string(case: ReferenceCase) -> str:
    import io

    fig = render(case)
    buffer = io.StringIO()
    fig.savefig(buffer, format="svg", transparent=True)
    plt.close(fig)
    svg = buffer.getvalue()
    for literal, token in (
        (_INK, "var(--ink)"),
        (_MUTED, "var(--muted)"),
        (_ACCENT, "var(--accent)"),
        (_ACCENT2, "var(--accent2)"),
        (_GRID, "var(--grid)"),
    ):
        svg = svg.replace(literal, token)
    return svg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/shared_nozzle_candidate_b.yaml")
    parser.add_argument("--fuels", default=None)
    parser.add_argument("--output", default="docs/vehicle_cross_section.svg")
    args = parser.parse_args()

    case = load_reference_case(ROOT / args.config, args.fuels)
    fig = render(case)
    output_path = ROOT / args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, format="svg", transparent=True)
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
