"""Build the Douglas Dart V4 OpenVSP model and run VSPAERO against it.

    .venv/Scripts/python scripts/vsp_model/run_vsp_model.py

Everything lands in one output directory: the ``.vsp3``, the raw solver files,
tidy CSVs, and a README recording exactly which inputs produced them.

    --geometry-only     build the .vsp3 and skip the solver entirely
    --grids a,b         run only the named sweep grids
    --wing-le 1.2       override the wing root LE station (m)
    --fin-area-ratio r  override total fin area / wing reference area
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mass_cg  # noqa: E402
import stability  # noqa: E402
import vehicle_geometry as vg  # noqa: E402
from geometry_inputs import GeometryInputs  # noqa: E402
from sizing_input import DEFAULT_SIZING_JSON, SizingInput  # noqa: E402
from vsp_aero import (  # noqa: E402
    run_sweep,
    write_points_csv,
    write_run_summary,
)
from vsp_build import build_model, verify_model  # noqa: E402

DEFAULT_OUTPUT_DIR = Path("out_vsp_model/v4")
MODEL_FILENAME = "douglas_dart_v4.vsp3"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sizing-json", type=Path, default=DEFAULT_SIZING_JSON)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--geometry-only", action="store_true")
    parser.add_argument(
        "--reanalyse",
        action="store_true",
        help="rebuild stability.md/.csv from an existing aero_points.csv without "
        "re-running the solver; use after changing the analysis, not the geometry",
    )
    parser.add_argument(
        "--grids",
        default="",
        help="comma-separated grid names to run; default is every configured grid",
    )
    parser.add_argument("--wing-le", type=float, default=None)
    parser.add_argument("--fin-area-ratio", type=float, default=None)
    parser.add_argument("--fin-aspect-ratio", type=float, default=None)
    parser.add_argument("--fin-le-sweep-deg", type=float, default=None)
    parser.add_argument("--fin-thickness-to-chord", type=float, default=None)
    parser.add_argument("--fin-clocking-offset-deg", type=float, default=None)
    parser.add_argument("--fin-vertical-fraction", type=float, default=None)
    parser.add_argument("--ballast-x", type=float, default=None)
    parser.add_argument(
        "--stability-altitude",
        type=float,
        default=163.5,
        help="altitude for the dynamic-mode estimates; default is the V4 ramjet "
        "light-off altitude",
    )
    return parser.parse_args(argv)


def _write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_geometry_csvs(geometry: vg.VehicleGeometry, output_dir: Path) -> None:
    _write_csv(
        output_dir / "geometry_stations.csv",
        [
            {
                "index": i,
                "x_m": round(s.x_m, 6),
                "x_mm": round(s.x_m * 1e3, 3),
                "diameter_m": round(s.diameter_m, 6),
                "diameter_mm": round(s.diameter_m * 1e3, 3),
            }
            for i, s in enumerate(geometry.body)
        ],
        ["index", "x_m", "x_mm", "diameter_m", "diameter_mm"],
    )

    panel_fields = [
        "name",
        "panel_count",
        "clocking_deg",
        "mirror_plane",
        "root_le_x_m",
        "root_te_x_m",
        "mount_radius_m",
        "root_chord_m",
        "tip_chord_m",
        "semispan_m",
        "le_sweep_deg",
        "dihedral_deg",
        "incidence_deg",
        "thickness_to_chord",
        "camber",
        "exposed_area_m2",
        "mac_m",
        "quarter_chord_x_m",
    ]
    _write_csv(
        output_dir / "geometry_panels.csv",
        [
            {
                "name": p.name,
                "panel_count": p.panel_count,
                "clocking_deg": p.clocking_deg,
                "mirror_plane": p.mirror_plane or "",
                "root_le_x_m": round(p.root_le_x_m, 6),
                "root_te_x_m": round(p.root_te_x_m, 6),
                "mount_radius_m": round(p.mount_radius_m, 6),
                "root_chord_m": round(p.root_chord_m, 6),
                "tip_chord_m": round(p.tip_chord_m, 6),
                "semispan_m": round(p.semispan_m, 6),
                "le_sweep_deg": round(p.le_sweep_deg, 4),
                "dihedral_deg": p.dihedral_deg,
                "incidence_deg": p.incidence_deg,
                "thickness_to_chord": p.thickness_to_chord,
                "camber": p.camber,
                "exposed_area_m2": round(p.exposed_area_m2, 6),
                "mac_m": round(p.mean_aerodynamic_chord_m, 6),
                "quarter_chord_x_m": round(p.quarter_chord_x_m, 6),
            }
            for p in geometry.all_panels
        ],
        panel_fields,
    )


def write_mass_csv(
    conditions: list[mass_cg.MassProperties], output_dir: Path
) -> None:
    """Per-item mass and station for each fuel condition.

    Both release and burnout are written because the tank is an annulus around
    the tailpipe, well aft of the dry CG: the vehicle's CG moves FORWARD as it
    burns, so static margin is a range, not a number.
    """

    rows: list[dict] = []
    for properties in conditions:
        rows.extend(
            {
                "condition": properties.condition,
                "item": item.name,
                "mass_kg": round(item.mass_kg, 6),
                "x_m": round(item.x_m, 6),
                "x_mm": round(item.x_m * 1e3, 2),
                "length_m": round(item.length_m, 6),
                "radius_m": round(item.radius_m, 6),
                "station_source": item.station_source,
                "basis": item.basis,
            }
            for item in sorted(properties.items, key=lambda i: i.x_m)
        )
        rows.append(
            {
                "condition": properties.condition,
                "item": "TOTAL",
                "mass_kg": round(properties.mass_kg, 6),
                "x_m": round(properties.cg_x_m, 6),
                "x_mm": round(properties.cg_x_m * 1e3, 2),
                "length_m": "",
                "radius_m": "",
                "station_source": "",
                "basis": (
                    f"CG; Ixx={properties.i_xx_kg_m2:.4f} "
                    f"Iyy={properties.i_yy_kg_m2:.4f} "
                    f"Izz={properties.i_zz_kg_m2:.4f} kg m^2"
                ),
            }
        )
    _write_csv(
        output_dir / "mass_properties.csv",
        rows,
        [
            "condition",
            "item",
            "mass_kg",
            "x_m",
            "x_mm",
            "length_m",
            "radius_m",
            "station_source",
            "basis",
        ],
    )


def write_barrowman_report(sizing, inputs, mass, burnout, output_dir: Path) -> str:
    """Analytic static stability, written as the PRIMARY stability result.

    VSPAERO is no longer the authority on this vehicle's stability: its
    vortex-lattice solve does not converge at the fin/body junction and its panel
    solve returns intermittent outliers (see README). Barrowman is analytic, is
    the standard method for a slender finned body, and its body term agrees with
    an independent slender-body integral to 5.2%. The VSPAERO grid is still run,
    but as a cross-check with a stated disagreement, not as the answer.
    """

    import barrowman

    lines = ["Barrowman static stability (PRIMARY -- see README on why not VSPAERO)", ""]
    rows: list[dict] = []
    for properties, label in ((mass, "release"), (burnout, "burnout")):
        results = barrowman.analyse(sizing, inputs, properties.cg_x_m, mach=0.5)
        pitch, yaw = results["pitch"], results["yaw"]
        lines.append(
            f"  {label:<8} CG {properties.cg_x_m * 1e3:7.1f} mm   "
            f"pitch SM {pitch.static_margin_calibres:+6.3f} cal "
            f"[{'STABLE' if pitch.stable else 'UNSTABLE'}]   "
            f"yaw SM {yaw.static_margin_calibres:+6.3f} cal "
            f"[{'STABLE' if yaw.stable else 'UNSTABLE'}]"
        )
        for axis, result in results.items():
            rows.append(
                {
                    "condition": label,
                    "axis": axis,
                    "cg_x_m": round(properties.cg_x_m, 6),
                    "x_cp_m": round(result.x_cp_m, 6),
                    "cn_alpha_total": round(result.cn_alpha_total, 6),
                    "static_margin_calibres": round(result.static_margin_calibres, 6),
                    "stable": result.stable,
                }
            )
    lines.append("")
    lines.append("  Calibres, not reference chords: the convention for finned bodies.")
    lines.append(
        "  Inviscid and linear -- no viscous body crossflow, which is destabilising,"
    )
    lines.append("  so these are optimistic. Treat 1-2 calibres as the working target.")
    _write_csv(output_dir / "stability_barrowman.csv", rows, list(rows[0]))
    return "\n".join(lines)


def write_flutter_and_roll(sizing, inputs, geometry, mass, output_dir: Path) -> str:
    """Fin flutter and roll behaviour — the two checks the aero alone cannot make.

    Flutter is a hard structural GATE, not an objective: aspect ratio is the
    cheapest lever on yaw stiffness, and it is also the fastest route to losing
    the fin. Roll has no static margin to report (an axisymmetric body with
    symmetric fins is neutrally stable in roll by construction), so what is
    reported is damping and the roll rate a realistic fin misalignment produces.
    """

    import barrowman
    import flutter

    results = flutter.check_vehicle(geometry)
    roll = barrowman.analyse_roll(geometry, mass)
    lines = [flutter.describe(results), ""]
    lines.append("Roll (no static margin exists — see RollResult)")
    lines.append(f"  Cl_p                {roll.cl_p:+8.4f} /rad   (negative = damped)")
    if roll.roll_time_constant_s is not None:
        lines.append(f"  roll time constant  {roll.roll_time_constant_s:8.3f} s")
    if roll.steady_roll_rate_deg_s is not None:
        lines.append(
            f"  steady roll rate    {roll.steady_roll_rate_deg_s:8.1f} deg/s  "
            f"from {roll.fin_misalignment_deg:.2f} deg of fin misalignment"
        )
    lines.append(
        "  Damping cannot null a misalignment-driven roll, only limit it. With"
    )
    lines.append(
        "  fins bonded to the skin and no spar or jig feature to key off, build"
    )
    lines.append("  tolerance is the design driver here, not aerodynamics.")

    rows = [
        {
            "surface": r.surface, "material": r.material,
            "aspect_ratio": round(r.aspect_ratio, 4),
            "thickness_to_chord": r.thickness_to_chord,
            "attachment_factor": r.attachment_factor,
            "flutter_mach": round(r.flutter_mach, 4),
            "required_mach": r.required_mach,
            "margin": round(r.margin, 4), "verdict": r.verdict,
        }
        for r in results
    ]
    rows.append(
        {
            "surface": "ROLL", "material": "", "aspect_ratio": "",
            "thickness_to_chord": "", "attachment_factor": "",
            "flutter_mach": "", "required_mach": "",
            "margin": round(roll.cl_p, 6),
            "verdict": (
                f"Cl_p; tau={roll.roll_time_constant_s:.3f}s; "
                f"{roll.steady_roll_rate_deg_s:.1f} deg/s at "
                f"{roll.fin_misalignment_deg:.2f} deg misalign"
                if roll.roll_time_constant_s else "Cl_p"
            ),
        }
    )
    _write_csv(output_dir / "flutter_and_roll.csv", rows, list(rows[0]))
    return "\n".join(lines)


def render_with_marks(sizing, inputs, geometry, mass, output_dir: Path) -> None:
    """Draw the solver mesh with CG and centre-of-pressure superimposed."""

    import barrowman
    import render as render_module

    model = output_dir / MODEL_FILENAME
    if not model.with_suffix(".vspgeom").is_file():
        return  # no mesh yet; a geometry-only run has not called the solver
    results = barrowman.analyse(sizing, inputs, mass.cg_x_m, mach=0.5)
    marks = {
        "cg_x_m": mass.cg_x_m,
        "cp_pitch_x_m": results["pitch"].x_cp_m,
        "cp_yaw_x_m": results["yaw"].x_cp_m,
        "sm_pitch_cal": results["pitch"].static_margin_calibres,
        "subtitle": (
            f"CG {mass.cg_x_m * 1e3:.0f} mm   "
            f"pitch SM {results['pitch'].static_margin_calibres:+.2f} cal   "
            f"yaw SM {results['yaw'].static_margin_calibres:+.2f} cal   (Barrowman)"
        ),
    }
    print(f"wrote {render_module.render(model, output_dir, marks)}")


def analyse_stability(
    results, geometry: vg.VehicleGeometry, mass: mass_cg.MassProperties, altitude_m: float
) -> tuple[str, list[dict]]:
    """Static margins per Mach, plus dynamic modes wherever the data allows."""

    all_points = [p for r in results for p in r.points]
    sections: list[str] = []
    rows: list[dict] = []

    # Group by METHOD first. Panel and vortex-lattice points exist at the same
    # Mach by design (the panel grid is the cross-check on the VLM one), and
    # fitting a slope across both would average two different solutions instead
    # of comparing them.
    methods = sorted({p.method for p in all_points})
    for method in methods:
        method_points = [p for p in all_points if p.method == method]
        for mach in sorted({p.mach for p in method_points}):
            row_section, row = _analyse_condition(
                method_points, method, mach, geometry, mass, altitude_m
            )
            if row_section:
                sections.append(row_section)
            if row:
                rows.append(row)
    return "\n".join(sections), rows


def _analyse_condition(
    method_points, method: str, mach: float, geometry, mass, altitude_m: float
) -> tuple[str, dict | None]:
    sections: list[str] = []
    try:
        derivatives = stability.derivatives_from_points(method_points, mach)
    except ValueError:
        return "", None

    # Two independent estimates of the same derivatives: differencing the sweep
    # points, and VSPAERO's own stability mode. They are NOT interchangeable --
    # measured disagreement on this model is roughly 1.8x on CL_alpha and 2x on
    # Cm_alpha -- so both are reported and the disagreement is flagged rather
    # than one being quietly preferred. The merged set (native first) is what
    # feeds the margins, because the rate derivatives only exist there.
    finite_difference = derivatives
    native = sorted(
        (p for p in method_points if p.derivatives and abs(p.mach - mach) < 1e-9),
        key=lambda p: abs(p.alpha_deg),
    )
    native_derivatives = (
        stability.derivatives_from_stability_run(native[0]) if native else None
    )
    if native_derivatives:
        # FINITE DIFFERENCE WINS on anything both can produce. Measured: at
        # M 0.50 the stability mode returns Cm_alpha +0.3958 where differencing
        # the 8-point alpha sweep gives +3.4450 -- an 8.7x disagreement, and the
        # finite-difference value is the one that lies on a smooth Mach trend
        # (+3.20, +3.65, +3.30, +3.45, +3.62 across the grid) while the
        # stability-mode value is an isolated outlier. Native is kept only for
        # the rate derivatives (Cm_q, Cn_r, Cl_p), which differencing a static
        # sweep cannot produce at all.
        merged = dict(native_derivatives.values)
        merged.update(finite_difference.values)
        derivatives = stability.StabilityDerivatives(
            mach=mach,
            alpha_deg=native[0].alpha_deg,
            source=f"{method}: finite difference, rate derivatives from stability mode",
            values=merged,
        )
    else:
        derivatives = stability.StabilityDerivatives(
            mach=mach,
            alpha_deg=0.0,
            source=f"{method}: finite difference of sweep points",
            values=finite_difference.values,
        )
    try:
        static = stability.static_stability(derivatives, geometry)
    except ValueError as exc:
        return f"[{method}] Mach {mach:.2f}: {exc}\n", None

    sections.append(f"[{method}] " + stability.describe_static(static))
    comparison = _compare_estimates(finite_difference, native_derivatives)
    if comparison:
        sections.append("")
        sections.append(comparison)
    modes = stability.dynamic_modes(derivatives, geometry, mass, altitude_m)
    sections.append("")
    sections.append(stability.describe_dynamic(modes))
    sections.append("")
    return (
        "\n".join(sections),
        {
            "method": method,
            "mach": mach,
            "source": derivatives.source,
            "CL_alpha_finite_difference": _round(finite_difference.get("CL_alpha")),
            "Cm_alpha_finite_difference": _round(finite_difference.get("Cm_alpha")),
            "CL_alpha_stability_mode": _round(
                native_derivatives.get("CL_alpha") if native_derivatives else None
            ),
            "Cm_alpha_stability_mode": _round(
                native_derivatives.get("Cm_alpha") if native_derivatives else None
            ),
            "CL_alpha": round(static.cl_alpha, 6),
            "Cm_alpha": round(static.cm_alpha, 6),
            "Cn_beta": _round(static.cn_beta),
            "Cl_beta": _round(static.cl_beta),
            "CY_beta": _round(static.cy_beta),
            "cg_x_m": round(static.cg_x_m, 6),
            "neutral_point_x_m": round(static.neutral_point_x_m, 6),
            "static_margin_cref": round(static.static_margin, 6),
            "pitch_stable": static.pitch_stable,
            "directionally_stable": static.directionally_stable,
            "short_period_rad_s": _round(modes.short_period_rad_s),
            "short_period_damping": _round(modes.short_period_damping),
            "dutch_roll_rad_s": _round(modes.dutch_roll_rad_s),
            "dutch_roll_damping": _round(modes.dutch_roll_damping),
            "roll_tau_s": _round(modes.roll_time_constant_s),
        },
    )


def _round(value: float | None, digits: int = 6):
    return "" if value is None else round(value, digits)


def _compare_estimates(
    finite_difference: stability.StabilityDerivatives,
    native: stability.StabilityDerivatives | None,
) -> str:
    """Show both estimates of the shared derivatives side by side.

    They come from genuinely different calculations -- a slope fitted across the
    commanded alpha sweep, versus VSPAERO perturbing its own converged solution --
    and on this model they do not agree well. That is information about how much
    to trust the margins, so it is printed rather than resolved by picking one.
    """

    if native is None:
        return ""
    lines: list[str] = []
    for name in ("CL_alpha", "Cm_alpha", "CY_beta", "Cn_beta", "Cl_beta"):
        a, b = finite_difference.get(name), native.get(name)
        if a is None or b is None:
            continue
        if abs(b) > 1e-9:
            ratio = f"{a / b:5.2f}x"
        else:
            ratio = "  n/a"
        flag = ""
        if abs(a) > 1e-6 and abs(b) > 1e-6 and not 0.8 <= abs(a / b) <= 1.25:
            flag = "  <-- DISAGREE"
        lines.append(
            f"  {name:<10} finite-difference {a:+8.4f}   stability-mode {b:+8.4f}"
            f"   ratio {ratio}{flag}"
        )
    if not lines:
        return ""
    return "  Two independent estimates of the same derivatives:\n" + "\n".join(lines)


def load_points_csv(path: Path):
    """Rebuild AeroPoints from a written aero_points.csv.

    Lets the analysis be re-run against solver output that already cost an hour,
    which is the difference between fixing a reporting bug and re-flying the grid.
    """

    from vsp_aero import AeroPoint  # local: keeps the import cost off --geometry-only

    fixed = {"grid", "method", "mach", "alpha_deg", "beta_deg", "solve_s"}
    points = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            coefficients: dict[str, float] = {}
            derivatives: dict[str, float] = {}
            for key, raw in row.items():
                if key in fixed or raw in ("", None):
                    continue
                try:
                    value = float(raw)
                except ValueError:
                    continue
                if key.startswith("d_"):
                    derivatives[key[2:]] = value
                else:
                    coefficients[key] = value
            points.append(
                AeroPoint(
                    grid=row["grid"],
                    method=row["method"],
                    mach=float(row["mach"]),
                    alpha_deg=float(row["alpha_deg"]),
                    beta_deg=float(row["beta_deg"]),
                    solve_seconds=float(row.get("solve_s") or 0.0),
                    coefficients=coefficients,
                    derivatives=derivatives,
                )
            )
    return points


class _StoredResults:
    """Minimal stand-in for SweepResult, so analyse_stability is unchanged."""

    def __init__(self, points):
        self.points = points


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    sizing = SizingInput.from_json(args.sizing_json)
    inputs = GeometryInputs()
    overrides: dict[str, object] = {}
    for flag, field in (
        ("wing_le", "wing_root_le_x_m"),
        ("fin_area_ratio", "fin_area_ratio"),
        ("fin_aspect_ratio", "fin_aspect_ratio"),
        ("fin_le_sweep_deg", "fin_le_sweep_deg"),
        ("fin_thickness_to_chord", "fin_thickness_to_chord"),
        ("fin_clocking_offset_deg", "fin_clocking_offset_deg"),
        ("fin_vertical_fraction", "fin_vertical_fraction"),
    ):
        value = getattr(args, flag, None)
        if value is not None:
            overrides[field] = value
    if overrides:
        inputs = replace(inputs, **overrides)

    mass = mass_cg.vehicle_mass_properties(
        sizing, inputs, ballast_x_m=args.ballast_x
    )
    burnout = mass_cg.vehicle_mass_properties(
        sizing, inputs, ballast_x_m=args.ballast_x, fuel_fraction=0.0
    )
    # Moments are referenced to the RELEASE CG. Burnout is reported alongside so
    # the static-margin shift over the burn is visible rather than implied.
    geometry = vg.build_vehicle_geometry(sizing, inputs, cg_x_m=mass.cg_x_m)

    if args.reanalyse:
        points_csv = output_dir / "aero_points.csv"
        if not points_csv.is_file():
            raise SystemExit(f"--reanalyse needs an existing {points_csv}")
        results = [_StoredResults(load_points_csv(points_csv))]
        report, rows = analyse_stability(
            results, geometry, mass, args.stability_altitude
        )
        (output_dir / "stability.md").write_text(
            "# Douglas Dart V4 -- stability from VSPAERO\n\n```\n" + report + "\n```\n",
            encoding="utf-8",
        )
        if rows:
            _write_csv(output_dir / "stability.csv", rows, list(rows[0]))
        print(report)
        print(f"\nRe-analysed {len(results[0].points)} stored points; no solver run.")
        return 0

    print(vg.describe(geometry))
    print()
    print(mass_cg.describe(mass, sizing.body_length_m))
    print()
    print(
        f"CG travel over the burn: {mass.cg_x_m * 1e3:.1f} mm at release -> "
        f"{burnout.cg_x_m * 1e3:.1f} mm at burnout "
        f"({(mass.cg_x_m - burnout.cg_x_m) / geometry.references.chord_m:+.3f} cref "
        "forward; the tank is aft of the dry CG)"
    )
    print()

    model_path = output_dir / MODEL_FILENAME
    build = build_model(geometry, model_path)
    verification = verify_model(model_path, geometry)
    print(f"Model written: {model_path}  ({model_path.stat().st_size / 1024:.0f} kB)")
    print(f"Read-back OK : {', '.join(verification['geom_names'])}")
    print()

    write_geometry_csvs(geometry, output_dir)
    write_mass_csv([mass, burnout], output_dir)
    print(write_barrowman_report(sizing, inputs, mass, burnout, output_dir))
    print()
    print(write_flutter_and_roll(sizing, inputs, geometry, mass, output_dir))
    print()
    render_with_marks(sizing, inputs, geometry, mass, output_dir)

    provenance = {
        "sizing_source": sizing.source_path,
        "sizing_git_sha": sizing.git_sha,
        "openvsp_version": build.openvsp_version,
        "model_file": MODEL_FILENAME,
        "geometry_inputs": inputs.to_dict(),
        "reference_quantities": asdict(geometry.references),
        "mass_properties": {
            "condition": mass.condition,
            "mass_kg": mass.mass_kg,
            "cg_x_m": mass.cg_x_m,
            "i_xx_kg_m2": mass.i_xx_kg_m2,
            "i_yy_kg_m2": mass.i_yy_kg_m2,
            "i_zz_kg_m2": mass.i_zz_kg_m2,
        },
        "body_stations_m": [
            {"x_m": s.x_m, "diameter_m": s.diameter_m} for s in geometry.body
        ],
    }

    if args.geometry_only:
        (output_dir / "inputs.json").write_text(
            json.dumps(provenance, indent=2), encoding="utf-8"
        )
        print("Geometry only -- solver skipped.")
        return 0

    wanted = {name.strip() for name in args.grids.split(",") if name.strip()}
    grids = [g for g in inputs.sweeps if not wanted or g.name in wanted]
    if not grids:
        raise SystemExit(f"no grid matched {sorted(wanted)}")

    results = []
    for grid in grids:
        print(
            f"VSPAERO grid '{grid.name}' -- {grid.method}, {grid.point_count()} points"
            + (" (stability mode)" if grid.stability else "")
        )
        results.append(run_sweep(geometry, model_path, grid, output_dir))
        # Rewrite after EVERY grid, not just at the end. A stability-mode grid can
        # take an order of magnitude longer per point than a plain one, and losing
        # an hour of solved points to an interrupted run is avoidable.
        write_points_csv(results, output_dir / "aero_points.csv")
        print()

    points_csv = write_points_csv(results, output_dir / "aero_points.csv")
    provenance["sweeps"] = write_run_summary(results)
    (output_dir / "inputs.json").write_text(
        json.dumps(provenance, indent=2), encoding="utf-8"
    )

    report, rows = analyse_stability(results, geometry, mass, args.stability_altitude)
    (output_dir / "stability.md").write_text(
        "# Douglas Dart V4 -- stability from VSPAERO\n\n```\n" + report + "\n```\n",
        encoding="utf-8",
    )
    if rows:
        _write_csv(output_dir / "stability.csv", rows, list(rows[0]))
    print(report)
    print()
    print(f"Wrote {points_csv} and {len(rows)} stability rows to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
