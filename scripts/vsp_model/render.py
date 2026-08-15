"""Render three views of the geometry VSPAERO actually solves.

    .venv/Scripts/python scripts/vsp_model/render.py <model.vsp3> [outdir]

Reads the ``.vspgeom`` triangulated surface that ``VSPAEROComputeGeometry``
writes -- NOT the station table this repo used to build the model. That
distinction is the whole point: a plot drawn from my own inputs would restate my
assumptions back at me, whereas the mesh shows what the solver was handed,
including anything the builder got wrong on the way.

(OpenVSP's own ``ScreenGrab`` needs an initialised GUI/OpenGL context and
silently writes nothing headless, which is why this reads the mesh instead.)
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402


def read_vspgeom(path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    """Parse the node/triangle blocks of a vspgeom v3 file."""

    tokens = path.read_text(encoding="utf-8", errors="ignore").split("\n")
    # Line 0: "# vspgeom v3", line 1: mesh count, line 2: "<nodes> <tris> <parts>"
    counts = tokens[2].split()
    node_count, tri_count = int(counts[0]), int(counts[1])

    nodes: list[tuple[float, float, float]] = []
    index = 3
    while len(nodes) < node_count and index < len(tokens):
        parts = tokens[index].split()
        if len(parts) >= 3:
            nodes.append((float(parts[0]), float(parts[1]), float(parts[2])))
        index += 1

    # Faces are QUADS as well as triangles in this format ("4 i1 i2 i3 i4"), and
    # the block is preceded by a repeat of the face count on its own line.
    faces: list[tuple[int, ...]] = []
    while len(faces) < tri_count and index < len(tokens):
        parts = tokens[index].split()
        index += 1
        if len(parts) < 4 or parts[0] not in ("3", "4"):
            continue
        try:
            count = int(parts[0])
            faces.append(tuple(int(v) - 1 for v in parts[1 : count + 1]))
        except ValueError:
            continue
    return nodes, faces


def _panel(axes, nodes, faces, horiz: int, vert: int, title: str) -> None:
    """Draw the mesh projected onto two axes, painters-sorted for depth."""

    depth_axis = ({0, 1, 2} - {horiz, vert}).pop()
    faces = sorted(
        faces,
        key=lambda f: sum(nodes[i][depth_axis] for i in f) / len(f),
    )
    patches = []
    for face in faces:
        patches.append([(nodes[i][horiz], nodes[i][vert]) for i in face])
    for pts in patches:
        axes.add_patch(
            Polygon(pts, closed=True, facecolor="#8fb8de", edgecolor="#31506b",
                    linewidth=0.12, alpha=0.85)
        )
    hs = [n[horiz] for n in nodes]
    vs = [n[vert] for n in nodes]
    pad = 0.05 * (max(hs) - min(hs))
    axes.set_xlim(min(hs) - pad, max(hs) + pad)
    axes.set_ylim(min(vs) - pad, max(vs) + pad)
    axes.set_aspect("equal")
    axes.set_title(title, fontsize=10)
    axes.grid(alpha=0.25, linewidth=0.4)


def _mark_stations(axes, marks: dict, vertical_extent: float) -> None:
    """Overlay CG and centre-of-pressure stations on a side/top view.

    CG is drawn in the conventional quartered-circle form so it reads as a CG at
    a glance, and the CP markers sit on the same axis so the static margin is the
    visible gap between them -- which is the whole point of putting them on the
    same picture.
    """

    from matplotlib.patches import Circle, Wedge

    radius = 0.028 * vertical_extent
    cg = marks.get("cg_x_m")
    if cg is not None:
        for start in (0, 180):
            axes.add_patch(Wedge((cg, 0.0), radius, start, start + 90,
                                 facecolor="black", edgecolor="black", zorder=12))
        for start in (90, 270):
            axes.add_patch(Wedge((cg, 0.0), radius, start, start + 90,
                                 facecolor="white", edgecolor="black", zorder=12))
        axes.add_patch(Circle((cg, 0.0), radius, fill=False, edgecolor="black",
                              linewidth=1.1, zorder=13))
        axes.annotate(f"CG {cg * 1e3:.0f}", (cg, -radius * 1.6),
                      ha="center", va="top", fontsize=8, zorder=14)

    for key, colour, label in (
        ("cp_pitch_x_m", "crimson", "CP pitch"),
        ("cp_yaw_x_m", "seagreen", "CP yaw"),
    ):
        station = marks.get(key)
        if station is None:
            continue
        axes.axvline(station, color=colour, linewidth=1.3, linestyle="--",
                     alpha=0.9, zorder=11)
        axes.annotate(f"{label} {station * 1e3:.0f}", (station, vertical_extent * 0.62),
                      rotation=90, ha="right", va="top", fontsize=7.5,
                      color=colour, zorder=14)

    if cg is not None and marks.get("cp_pitch_x_m") is not None:
        axes.annotate(
            "", xy=(marks["cp_pitch_x_m"], -vertical_extent * 0.55),
            xytext=(cg, -vertical_extent * 0.55),
            arrowprops=dict(arrowstyle="<->", color="crimson", lw=1.1), zorder=12,
        )
        mid = 0.5 * (cg + marks["cp_pitch_x_m"])
        axes.annotate(
            f"static margin {marks.get('sm_pitch_cal', float('nan')):+.2f} cal",
            (mid, -vertical_extent * 0.60), ha="center", va="top",
            fontsize=8, color="crimson", zorder=14,
        )


def render(model_path: str | Path, output_dir: str | Path, marks: dict | None = None) -> Path:
    model_path = Path(model_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    mesh = model_path.with_suffix(".vspgeom")
    if not mesh.is_file():
        raise FileNotFoundError(
            f"{mesh} not found -- run a VSPAERO grid first so "
            "VSPAEROComputeGeometry writes the mesh."
        )
    nodes, faces = read_vspgeom(mesh)

    marks = marks or {}
    z_extent = max(n[2] for n in nodes) - min(n[2] for n in nodes)
    y_extent = max(n[1] for n in nodes) - min(n[1] for n in nodes)

    figure = plt.figure(figsize=(13.0, 8.6))
    grid = figure.add_gridspec(2, 2, height_ratios=[1.0, 1.35])
    side = figure.add_subplot(grid[0, :])
    _panel(side, nodes, faces, 0, 2, "SIDE  (x aft, z up)")
    _mark_stations(side, marks, z_extent)
    top = figure.add_subplot(grid[1, 0])
    _panel(top, nodes, faces, 0, 1, "TOP  (x aft, y starboard)")
    _mark_stations(top, marks, y_extent)
    _panel(figure.add_subplot(grid[1, 1]), nodes, faces, 1, 2,
           "FRONT  (y starboard, z up)")
    subtitle = marks.get("subtitle", "")
    figure.suptitle(
        f"Douglas Dart V4 — solver mesh from {mesh.name}  "
        f"({len(nodes)} nodes, {len(faces)} faces)"
        + ("\n" + subtitle if subtitle else ""),
        fontsize=11,
    )
    figure.tight_layout()
    path = output_dir / "geometry_views.png"
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return path


def main() -> int:
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    model = Path(sys.argv[1])
    outdir = Path(sys.argv[2]) if len(sys.argv) > 2 else model.parent
    print(f"wrote {render(model, outdir)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
