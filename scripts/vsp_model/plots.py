"""Plot the VSPAERO polars written by ``run_vsp_model.py``.

    .venv/Scripts/python scripts/vsp_model/plots.py [output_dir]

Reads ``aero_points.csv`` and writes PNGs next to it. Vortex-lattice and panel
points are drawn distinctly rather than merged -- they are different solutions of
the same geometry, and the panel grid exists to check the VLM one.
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DEFAULT_OUTPUT_DIR = Path("out_vsp_model/v4")


def read_points(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key, value in list(row.items()):
            if key in ("grid", "method", "source"):
                continue
            try:
                row[key] = float(value) if value not in ("", None) else None
            except (TypeError, ValueError):
                row[key] = None
    return rows


def _by_mach(rows: list[dict], method: str, beta: float = 0.0) -> dict[float, list[dict]]:
    grouped: dict[float, list[dict]] = defaultdict(list)
    for row in rows:
        if row["method"] != method:
            continue
        if row.get("beta_deg") is None or abs(row["beta_deg"] - beta) > 1e-9:
            continue
        grouped[row["mach"]].append(row)
    for series in grouped.values():
        series.sort(key=lambda r: r["alpha_deg"])
    return dict(sorted(grouped.items()))


def plot_polars(rows: list[dict], output_dir: Path) -> list[Path]:
    written: list[Path] = []
    vlm = _by_mach(rows, "vortex_lattice")
    panel = _by_mach(rows, "panel")
    if not vlm:
        return written

    colours = plt.cm.viridis([i / max(len(vlm) - 1, 1) for i in range(len(vlm))])

    panels = [
        ("CLtot", "alpha_deg", r"$\alpha$ (deg)", r"$C_L$", "lift_curve"),
        ("CMytot", "alpha_deg", r"$\alpha$ (deg)", r"$C_{m}$ (body axis)", "pitching_moment"),
    ]
    for field, xfield, xlabel, ylabel, name in panels:
        figure, axes = plt.subplots(figsize=(7.0, 4.6))
        for colour, (mach, series) in zip(colours, vlm.items()):
            xs = [r[xfield] for r in series if r.get(field) is not None]
            ys = [r[field] for r in series if r.get(field) is not None]
            if xs:
                axes.plot(xs, ys, "-o", ms=3, color=colour, label=f"VLM M{mach:.2f}")
        for mach, series in panel.items():
            xs = [r[xfield] for r in series if r.get(field) is not None]
            ys = [r[field] for r in series if r.get(field) is not None]
            if xs:
                axes.plot(xs, ys, "s--", ms=6, mfc="none", color="crimson",
                          label=f"panel M{mach:.2f}")
        axes.axhline(0.0, lw=0.6, color="0.6")
        axes.axvline(0.0, lw=0.6, color="0.6")
        axes.set_xlabel(xlabel)
        axes.set_ylabel(ylabel)
        axes.grid(alpha=0.3)
        axes.legend(fontsize=7, ncol=2)
        if field == "CMytot":
            axes.set_title(
                "Pitching moment about the CG — positive slope is UNSTABLE",
                fontsize=10,
            )
        else:
            axes.set_title("Lift curve (Sref = wing trapezoid 0.15250 m$^2$)", fontsize=10)
        figure.tight_layout()
        path = output_dir / f"{name}.png"
        figure.savefig(path, dpi=140)
        plt.close(figure)
        written.append(path)

    # Drag polar. Labelled hard, because this CD is not the vehicle's drag.
    figure, axes = plt.subplots(figsize=(7.0, 4.6))
    for colour, (mach, series) in zip(colours, vlm.items()):
        xs = [r["CDtot"] for r in series if r.get("CDtot") is not None]
        ys = [r["CLtot"] for r in series if r.get("CDtot") is not None]
        if xs:
            axes.plot(xs, ys, "-o", ms=3, color=colour, label=f"VLM M{mach:.2f}")
    axes.set_xlabel(r"$C_D$ (inviscid only)")
    axes.set_ylabel(r"$C_L$")
    axes.grid(alpha=0.3)
    axes.legend(fontsize=7, ncol=2)
    axes.set_title(
        "Drag polar — INVISCID. No skin friction, no base drag (80.9 cm$^2$\n"
        "annular base), no spillage. Not the vehicle's drag; see drag_buildup.py.",
        fontsize=9,
    )
    figure.tight_layout()
    path = output_dir / "drag_polar.png"
    figure.savefig(path, dpi=140)
    plt.close(figure)
    written.append(path)
    return written


def plot_mach_trends(rows: list[dict], output_dir: Path) -> list[Path]:
    """CL_alpha and Cm_alpha against Mach, from the VLM grid."""

    vlm = _by_mach(rows, "vortex_lattice")
    machs, cl_alpha, cm_alpha = [], [], []
    for mach, series in vlm.items():
        usable = [r for r in series if abs(r["alpha_deg"]) <= 6.0]
        if len(usable) < 2:
            continue
        alphas = [r["alpha_deg"] * 3.141592653589793 / 180.0 for r in usable]
        mean_a = sum(alphas) / len(alphas)
        denominator = sum((a - mean_a) ** 2 for a in alphas)
        if denominator <= 0:
            continue

        def slope(field: str) -> float:
            values = [r[field] for r in usable]
            mean_v = sum(values) / len(values)
            return sum((a - mean_a) * (v - mean_v) for a, v in zip(alphas, values)) / denominator

        machs.append(mach)
        cl_alpha.append(slope("CLtot"))
        cm_alpha.append(slope("CMytot"))

    if not machs:
        return []

    figure, axes = plt.subplots(figsize=(7.0, 4.6))
    axes.plot(machs, cl_alpha, "-o", label=r"$C_{L\alpha}$")
    axes.plot(machs, cm_alpha, "-s", color="crimson", label=r"$C_{m\alpha}$")
    axes.axhline(0.0, lw=0.8, color="0.4")
    axes.set_xlabel("Mach")
    axes.set_ylabel("per radian")
    axes.grid(alpha=0.3)
    axes.legend()
    axes.set_title(
        r"$C_{m\alpha} > 0$ at every Mach: the airframe is statically unstable in pitch",
        fontsize=10,
    )
    figure.tight_layout()
    path = output_dir / "mach_trends.png"
    figure.savefig(path, dpi=140)
    plt.close(figure)
    return [path]


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    output_dir = Path(argv[0]) if argv else DEFAULT_OUTPUT_DIR
    csv_path = output_dir / "aero_points.csv"
    if not csv_path.is_file():
        raise SystemExit(f"no aero_points.csv in {output_dir}")

    rows = read_points(csv_path)
    written = plot_polars(rows, output_dir) + plot_mach_trends(rows, output_dir)
    for path in written:
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
