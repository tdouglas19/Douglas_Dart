"""Search wing station and fin area for a statically stable Douglas Dart V4.

    .venv/Scripts/python scripts/vsp_model/sweep_stability.py

The baseline airframe is badly unstable in BOTH axes. Measured over the full
78-point baseline grid, the neutral point sits at ~330 mm from the nose against a
CG at 922 mm -- a static margin of **-2.0 reference chords**, near-constant from
M 0.20 to M 0.80 -- and ``Cn_beta`` is about **-2.2 /rad** where stability needs
it positive. That is the slender body's Munk moment, and neither the 533 mm wing
sitting 184 mm behind the CG nor the baseline fin comes close to countering it.

WHY THIS IS CHEAP. The neutral point is a property of the AERODYNAMICS ALONE --
moving the CG moves the moment reference, not the point where the moment
derivative vanishes. So each geometry needs exactly one solver evaluation, and
the CG (i.e. the 4.68 kg ballast station, the only large free mass in the
vehicle) is then solved analytically against it:

    x_np   = x_ref + (-Cm_alpha / CL_alpha) * cref          [one solve]
    SM(cg) = (x_np - x_cg) / cref                           [free, any cg]

Same trick for the directional axis, referencing Cn_beta/CY_beta to bref.

Results stream to CSV as they complete, so an interrupted run keeps its work.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mass_cg  # noqa: E402
import stability  # noqa: E402
from geometry_inputs import GeometryInputs, SweepGrid  # noqa: E402
from sizing_input import DEFAULT_SIZING_JSON, SizingInput  # noqa: E402
from vehicle_geometry import build_vehicle_geometry  # noqa: E402
from vsp_aero import run_sweep  # noqa: E402
from vsp_build import build_model  # noqa: E402

DEFAULT_OUTPUT_DIR = Path("out_vsp_model/v4_stability_sweep")

# Wing root LE stations, metres from the nose tip. Bounded below by the nose
# cowl (ends 428 mm) and above by the need to keep the 309 mm root chord clear
# of the boattail (starts 2060 mm).
WING_STATIONS_M = (1.00, 1.30, 1.60)

# Total exposed fin area / wing reference area. The baseline 0.35 is nowhere
# near enough; the arm from the CG to the fins is ~3.6 chords, so countering
# Cm_alpha = +3.3 /rad needs roughly 0.9 /rad of fin CL_alpha, which is several
# times the baseline fin. The range runs well past "sensible" on purpose --
# finding that nothing in it closes is itself the answer.
FIN_AREA_RATIOS = (0.35, 0.80, 1.30, 1.80, 2.30)

# One Mach only. Static margin is not strongly Mach-dependent below the
# transonic, and the point here is to rank configurations, not to build a
# polar -- that is what run_vsp_model.py does for the chosen one.
SEARCH_MACH = 0.50

TARGET_STATIC_MARGIN = 0.10
"""Static margin to aim for, in reference chords. 5-15% is the conventional
band for a stable but manoeuvrable airframe."""


@dataclass
class SweepPoint:
    wing_le_x_m: float
    fin_area_ratio: float
    ok: bool
    note: str = ""
    cl_alpha: float | None = None
    cm_alpha: float | None = None
    cy_beta: float | None = None
    cn_beta: float | None = None
    cl_beta: float | None = None
    reference_cg_x_m: float | None = None
    neutral_point_x_m: float | None = None
    yaw_neutral_point_x_m: float | None = None
    static_margin_at_release: float | None = None
    static_margin_at_burnout: float | None = None
    ballast_x_for_target_m: float | None = None
    ballast_feasible: bool | None = None
    fin_semispan_m: float | None = None
    fin_root_chord_m: float | None = None
    total_span_m: float | None = None
    mount_overlap_m: float | None = None
    overlap_attempts: int = 0


def search_grid(mach: float) -> SweepGrid:
    """Minimum set of points that pins both axes.

    Four alphas for the pitch slope, and two extra betas at alpha = 0 for the
    lateral one (beta = 0 at alpha = 0 is already in the pitch set, so this is
    six solves, not twelve).
    """

    return SweepGrid(
        name="search",
        method="vortex_lattice",
        mach_values=(mach,),
        alpha_deg_values=(-2.0, 0.0, 2.0, 4.0),
        beta_deg_values=(0.0,),
        wake_iterations=2,
    )


def sideslip_grid(mach: float) -> SweepGrid:
    return SweepGrid(
        name="search_beta",
        method="vortex_lattice",
        mach_values=(mach,),
        alpha_deg_values=(0.0,),
        beta_deg_values=(-4.0, 4.0),
        wake_iterations=2,
    )


def solve_ballast_station_m(
    sizing: SizingInput,
    inputs: GeometryInputs,
    neutral_point_x_m: float,
    target_margin: float,
    reference_chord_m: float,
) -> tuple[float | None, bool]:
    """Ballast station that puts the CG where the target margin wants it.

    Only the ballast moves; every other item keeps its assigned station. Because
    the CG is linear in the ballast station, this inverts directly instead of
    iterating.

    Returns ``(station, feasible)``. Feasible means the station lies inside the
    body, between the nose tip and the boattail start.
    """

    required_cg = neutral_point_x_m - target_margin * reference_chord_m

    # CG is linear in ballast station: evaluate two stations and invert.
    probe_a, probe_b = 0.20, 1.20
    cg_a = mass_cg.vehicle_mass_properties(sizing, inputs, ballast_x_m=probe_a).cg_x_m
    cg_b = mass_cg.vehicle_mass_properties(sizing, inputs, ballast_x_m=probe_b).cg_x_m
    slope = (cg_b - cg_a) / (probe_b - probe_a)
    if abs(slope) < 1e-9:
        return None, False
    station = probe_a + (required_cg - cg_a) / slope
    feasible = 0.0 <= station <= sizing.boattail_start_x_m
    return station, feasible


OVERLAP_ATTEMPTS_M = (0.006, 0.0075, 0.009, 0.0105, 0.012, 0.0135, 0.015)
"""Root interpenetrations to try, in order, until the solve converges.

NOT a tolerance ladder -- a jitter. The mixed thick/thin solve fails by returning
NaN from GMRES when a surface root intersects the body badly, and the failure is
**non-monotonic in depth**: measured on the fin_area_ratio 0.80 geometry, 6 mm
fails, 10 mm solves, 14 mm fails and 20 mm fails, while the much larger
ratio 2.30 fin solves at 6 mm. That is the signature of a mesh-intersection
degeneracy -- the root landing on a body tessellation feature -- not of a depth
threshold, so there is no rule to derive and no single value that is safe for
every planform. Perturbing the depth moves the intersection off the degeneracy.

The chosen depth is recorded per configuration so the geometry stays reproducible.
"""


def evaluate(
    sizing: SizingInput,
    base_inputs: GeometryInputs,
    wing_le_x_m: float,
    fin_area_ratio: float,
    output_dir: Path,
    mach: float,
    overlap_m: float | None = None,
    extra: dict | None = None,
) -> SweepPoint:
    inputs = replace(
        base_inputs,
        wing_root_le_x_m=wing_le_x_m,
        fin_area_ratio=fin_area_ratio,
        **({"min_radial_mount_overlap_m": overlap_m} if overlap_m is not None else {}),
        **(extra or {}),
    )
    point = SweepPoint(wing_le_x_m=wing_le_x_m, fin_area_ratio=fin_area_ratio, ok=False)

    try:
        mass = mass_cg.vehicle_mass_properties(sizing, inputs)
        burnout = mass_cg.vehicle_mass_properties(sizing, inputs, fuel_fraction=0.0)
        geometry = build_vehicle_geometry(sizing, inputs, cg_x_m=mass.cg_x_m)
    except ValueError as exc:
        # Geometry itself is invalid (e.g. the fin root runs off the barrel).
        point.note = f"geometry rejected: {exc}"
        return point

    fin = geometry.fin_panels[0] if geometry.fin_panels else None
    point.fin_semispan_m = fin.semispan_m if fin else 0.0
    point.fin_root_chord_m = fin.root_chord_m if fin else 0.0
    point.total_span_m = (
        2.0 * (fin.mount_radius_m + fin.semispan_m) if fin else sizing.body_diameter_m
    )

    model_path = output_dir / "models" / (
        f"wing{int(wing_le_x_m * 1000)}_fin{int(fin_area_ratio * 100)}.vsp3"
    )
    try:
        build_model(geometry, model_path)
        pitch = run_sweep(geometry, model_path, search_grid(mach), output_dir / "_solver", progress=False)
        yaw = run_sweep(geometry, model_path, sideslip_grid(mach), output_dir / "_solver", progress=False)
    except Exception as exc:  # noqa: BLE001 -- one bad config must not end the sweep
        point.note = f"{type(exc).__name__}: {str(exc)[:160]}"
        return point

    all_points = list(pitch.points) + list(yaw.points)
    try:
        derivatives = stability.derivatives_from_points(all_points, mach)
    except ValueError as exc:
        point.note = f"derivatives unavailable: {exc}"
        return point

    cl_alpha = derivatives.get("CL_alpha")
    cm_alpha = derivatives.get("Cm_alpha")
    if cl_alpha is None or cm_alpha is None or abs(cl_alpha) < 1e-9:
        point.note = "CL_alpha/Cm_alpha not resolvable"
        return point

    references = geometry.references
    cref, bref = references.chord_m, references.span_m
    point.ok = True
    point.cl_alpha = cl_alpha
    point.cm_alpha = cm_alpha
    point.cy_beta = derivatives.get("CY_beta")
    point.cn_beta = derivatives.get("Cn_beta")
    point.cl_beta = derivatives.get("Cl_beta")
    point.reference_cg_x_m = references.cg_x_m
    point.neutral_point_x_m = references.cg_x_m + (-cm_alpha / cl_alpha) * cref
    point.static_margin_at_release = (point.neutral_point_x_m - mass.cg_x_m) / cref
    point.static_margin_at_burnout = (point.neutral_point_x_m - burnout.cg_x_m) / cref

    if point.cn_beta is not None and point.cy_beta is not None and abs(point.cy_beta) > 1e-9:
        # With station s measured AFT from the nose, a side force at s_a gives
        # Cn = -CY * (s_a - s_ref) / bref, so s_a = s_ref - Cn_beta*bref/CY_beta.
        # A stable vehicle has Cn_beta > 0 and CY_beta < 0, which puts s_a aft of
        # the reference -- the sign check that catches getting this backwards.
        point.yaw_neutral_point_x_m = references.cg_x_m - (
            point.cn_beta / point.cy_beta
        ) * bref

    station, feasible = solve_ballast_station_m(
        sizing, inputs, point.neutral_point_x_m, TARGET_STATIC_MARGIN, cref
    )
    point.ballast_x_for_target_m = station
    point.ballast_feasible = feasible
    return point


SUBPROCESS_TIMEOUT_S = 900

_SINGLE_MARKER = "__SWEEP_POINT__="
"""Prefix the child uses to hand one result back on stdout.

A marker rather than plain stdout because VSPAERO writes a great deal of its own
output to the same stream, unterminated.
"""


def _evaluate_isolated(
    wing_le_x_m: float,
    fin_area_ratio: float,
    output_dir: Path,
    mach: float,
    overlap_m: float,
    sizing_json: Path,
    extra: dict | None = None,
) -> SweepPoint:
    """Evaluate one configuration in a SUBPROCESS.

    Necessary, not defensive. Some root/body intersections make OpenVSP's native
    mesher kill the interpreter outright -- no exception, no traceback, nothing a
    ``try/except`` can catch (measured: wing 1.00 / fin 1.40 at a 9.0 mm mount
    depth). In-process, that ends the entire sweep at whichever configuration
    happens to trip it. Isolated, it costs one configuration and the sweep
    carries on, which is the difference between a sweep that survives unattended
    and one that does not.

    Also bounds the runtime: a hung solve is failed rather than waited on forever.
    """

    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--single",
        "--wing-le", repr(wing_le_x_m),
        "--fin-ratio", repr(fin_area_ratio),
        "--overlap", repr(overlap_m),
        "--mach", repr(mach),
        "--output-dir", str(output_dir),
        "--sizing-json", str(sizing_json),
    ]
    for key, value in (extra or {}).items():
        command += [f"--set-{key.replace('_', '-')}", repr(value)]
    point = SweepPoint(wing_le_x_m=wing_le_x_m, fin_area_ratio=fin_area_ratio, ok=False)
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=SUBPROCESS_TIMEOUT_S,
            cwd=str(Path.cwd()),
        )
    except subprocess.TimeoutExpired:
        point.note = f"timed out after {SUBPROCESS_TIMEOUT_S}s"
        return point

    # Search the WHOLE stream, not line starts: VSPAERO writes to the same stdout
    # and leaves its last line unterminated, so the marker frequently ends up
    # concatenated mid-line behind solver output.
    index = completed.stdout.rfind(_SINGLE_MARKER)
    if index >= 0:
        tail = completed.stdout[index + len(_SINGLE_MARKER):]
        payload = json.loads(tail.splitlines()[0])
        return SweepPoint(**payload)
    # No marker: the child died before it could report. That is the hard-crash
    # mode, and it is the reason this runs out of process at all.
    point.note = (
        f"child process produced no result (exit {completed.returncode}) -- "
        "the OpenVSP mesher most likely killed it; treated as a diverged solve"
    )
    return point


CONSENSUS_DEPTHS_M = (0.006, 0.009, 0.012, 0.015)


def evaluate_consensus(
    sizing: SizingInput,
    base_inputs: GeometryInputs,
    wing_le_x_m: float,
    fin_area_ratio: float,
    output_dir: Path,
    mach: float,
    sizing_json: Path,
    extra: dict | None = None,
) -> tuple[SweepPoint, dict]:
    """Solve the SAME geometry at several root depths and take the median.

    Necessary because this solver fails intermittently rather than
    systematically. Measured on one fixed geometry: the VLM converged at 1 of 4
    depths (and refining the mesh made that WORSE -- 0 of 4 at the finest), while
    the panel method converged at 3 of 3 but returned one wild outlier
    (Cn_beta = -13.95 against +0.49 and +0.36 from its siblings).

    The solves that *do* succeed agree with each other and across methods, so the
    signal is real and the failures are sporadic. A median over several mount
    depths -- a parameter with no aerodynamic meaning -- rejects the outliers
    without hand-picking, and the spread across survivors is reported so a
    configuration resting on one lucky solve is visible as such.
    """

    from statistics import median

    good: list[SweepPoint] = []
    for overlap in CONSENSUS_DEPTHS_M:
        point = _evaluate_isolated(
            wing_le_x_m, fin_area_ratio, output_dir, mach, overlap, sizing_json, extra
        )
        if point.ok:
            point.mount_overlap_m = overlap
            good.append(point)

    if not good:
        failed = SweepPoint(wing_le_x_m=wing_le_x_m, fin_area_ratio=fin_area_ratio, ok=False)
        failed.note = f"no converged solve across {len(CONSENSUS_DEPTHS_M)} mount depths"
        return failed, {"n": 0}

    def spread(attr: str) -> float:
        values = [getattr(p, attr) for p in good if getattr(p, attr) is not None]
        return max(values) - min(values) if len(values) >= 2 else float("nan")

    # Median on each derivative independently: an outlier is usually wrong in one
    # axis, not all of them, so rejecting per-quantity keeps more good data than
    # discarding a whole solve.
    consensus = SweepPoint(
        wing_le_x_m=wing_le_x_m, fin_area_ratio=fin_area_ratio, ok=True
    )
    for attr in (
        "cl_alpha", "cm_alpha", "cy_beta", "cn_beta", "cl_beta",
        "reference_cg_x_m", "neutral_point_x_m", "static_margin_at_release",
        "static_margin_at_burnout", "fin_semispan_m", "fin_root_chord_m",
        "total_span_m",
    ):
        values = [getattr(p, attr) for p in good if getattr(p, attr) is not None]
        if values:
            setattr(consensus, attr, median(values))
    consensus.overlap_attempts = len(good)
    consensus.note = f"median of {len(good)}/{len(CONSENSUS_DEPTHS_M)} converged"
    return consensus, {
        "n": len(good),
        "cm_alpha_spread": spread("cm_alpha"),
        "cn_beta_spread": spread("cn_beta"),
        "cl_alpha_spread": spread("cl_alpha"),
    }


def evaluate_with_retry(
    sizing: SizingInput,
    base_inputs: GeometryInputs,
    wing_le_x_m: float,
    fin_area_ratio: float,
    output_dir: Path,
    mach: float,
    sizing_json: Path,
) -> SweepPoint:
    """Try successive root depths, each in its own process, until one converges.

    See :data:`OVERLAP_ATTEMPTS_M` for why the depth is jittered rather than
    tuned. Only a solver failure is retried; a rejected geometry is returned
    immediately, because re-running it at a different depth would fail the same
    way more slowly.
    """

    retryable = ("non-finite", "implausible", "diverged", "timed out", "no result")
    last = None
    for attempt, overlap in enumerate(OVERLAP_ATTEMPTS_M, start=1):
        point = _evaluate_isolated(
            wing_le_x_m, fin_area_ratio, output_dir, mach, overlap, sizing_json
        )
        point.mount_overlap_m = overlap
        point.overlap_attempts = attempt
        if point.ok:
            return point
        last = point
        if not any(k in point.note for k in retryable):
            return point
    return last


_FIELDS = [
    "wing_le_x_m",
    "fin_area_ratio",
    "ok",
    "mount_overlap_m",
    "overlap_attempts",
    "cl_alpha",
    "cm_alpha",
    "cy_beta",
    "cn_beta",
    "cl_beta",
    "reference_cg_x_m",
    "neutral_point_x_m",
    "yaw_neutral_point_x_m",
    "static_margin_at_release",
    "static_margin_at_burnout",
    "ballast_x_for_target_m",
    "ballast_feasible",
    "fin_semispan_m",
    "fin_root_chord_m",
    "total_span_m",
    "note",
]


def _row(point: SweepPoint) -> dict:
    def fmt(value):
        return "" if value is None else (round(value, 6) if isinstance(value, float) else value)

    return {field: fmt(getattr(point, field)) for field in _FIELDS}


def load_sweep_csv(path: Path) -> list[SweepPoint]:
    """Rebuild SweepPoints from a written sweep.csv.

    Lets the summary and plots be regenerated after changing how results are
    interpreted, without re-running an hour of solver.
    """

    points: list[SweepPoint] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            kwargs: dict = {}
            for field in _FIELDS:
                raw = row.get(field, "")
                if raw in ("", None):
                    kwargs[field] = None
                    continue
                if field == "ok" or field == "ballast_feasible":
                    kwargs[field] = raw == "True"
                elif field in ("note",):
                    kwargs[field] = raw
                elif field == "overlap_attempts":
                    kwargs[field] = int(float(raw))
                else:
                    kwargs[field] = float(raw)
            kwargs["ok"] = bool(kwargs.get("ok"))
            kwargs["note"] = kwargs.get("note") or ""
            kwargs["overlap_attempts"] = kwargs.get("overlap_attempts") or 0
            points.append(SweepPoint(**kwargs))
    return points


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sizing-json", type=Path, default=DEFAULT_SIZING_JSON)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--mach", type=float, default=SEARCH_MACH)
    parser.add_argument(
        "--summarise-only",
        action="store_true",
        help="rebuild summary.md and the plot from an existing sweep.csv",
    )
    parser.add_argument(
        "--single",
        action="store_true",
        help="internal: evaluate ONE configuration and print it as JSON. The "
        "sweep uses this to run each configuration out of process, because a bad "
        "root intersection can kill the interpreter outright.",
    )
    parser.add_argument("--wing-le", type=float, default=None)
    parser.add_argument("--fin-ratio", type=float, default=None)
    parser.add_argument("--overlap", type=float, default=None)
    parser.add_argument("--set-fuselage-tessellation", type=int, default=None)
    parser.add_argument("--set-surface-tessellation", type=int, default=None)
    parser.add_argument("--set-fin-vertical-fraction", type=float, default=None)
    parser.add_argument("--set-fin-aspect-ratio", type=float, default=None)
    parser.add_argument("--set-fin-le-sweep-deg", type=float, default=None)
    parser.add_argument("--set-fin-clocking-offset-deg", type=float, default=None)
    parser.add_argument("--set-fin-root-te-x-m", type=float, default=None)
    parser.add_argument(
        "--fin-ratios",
        default="",
        help="comma-separated fin area ratios; default is the built-in range",
    )
    parser.add_argument(
        "--wing-stations",
        default="",
        help="comma-separated wing root LE stations in metres",
    )
    args = parser.parse_args(argv)

    fin_ratios = (
        tuple(float(v) for v in args.fin_ratios.split(",") if v.strip())
        or FIN_AREA_RATIOS
    )
    wing_stations = (
        tuple(float(v) for v in args.wing_stations.split(",") if v.strip())
        or WING_STATIONS_M
    )

    if args.single:
        sizing = SizingInput.from_json(args.sizing_json)
        extra = {
            k[len('set_'):]: v
            for k, v in vars(args).items()
            if k.startswith('set_') and v is not None
        }
        point = evaluate(
            sizing,
            GeometryInputs(),
            args.wing_le,
            args.fin_ratio,
            args.output_dir,
            args.mach,
            args.overlap,
            extra,
        )
        # Flush explicitly: the parent reads stdout, and VSPAERO has been writing
        # to the same stream throughout.
        print("\n" + _SINGLE_MARKER + json.dumps(asdict(point)), flush=True)
        return 0

    if args.summarise_only:
        sizing = SizingInput.from_json(args.sizing_json)
        results = load_sweep_csv(args.output_dir / "sweep.csv")
        (args.output_dir / "summary.md").write_text(
            _summarise(results, sizing, args.mach), encoding="utf-8"
        )
        plot_sweep(results, args.output_dir)
        print(f"Rebuilt {args.output_dir / 'summary.md'} from {len(results)} rows")
        return 0

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    sizing = SizingInput.from_json(args.sizing_json)
    base_inputs = GeometryInputs()

    csv_path = output_dir / "sweep.csv"
    total = len(wing_stations) * len(fin_ratios)
    print(
        f"Stability sweep: {len(wing_stations)} wing stations x "
        f"{len(fin_ratios)} fin ratios = {total} configurations, "
        f"6 solves each, at Mach {args.mach}"
    )
    print(f"Target static margin {TARGET_STATIC_MARGIN:+.2f} cref\n")

    results: list[SweepPoint] = []
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=_FIELDS)
        writer.writeheader()
        handle.flush()
        index = 0
        for wing_le in wing_stations:
            for fin_ratio in fin_ratios:
                index += 1
                point = evaluate_with_retry(
                    sizing, base_inputs, wing_le, fin_ratio, output_dir,
                    args.mach, args.sizing_json,
                )
                results.append(point)
                writer.writerow(_row(point))
                handle.flush()  # stream, so an interrupted sweep keeps its work
                if point.ok:
                    print(
                        f"[{index:>2}/{total}] wing {wing_le:.2f} m  fin {fin_ratio:.2f}  "
                        f"x_np {point.neutral_point_x_m * 1e3:7.1f} mm  "
                        f"ov {point.mount_overlap_m * 1e3:4.1f}mm  "
                        f"SM {point.static_margin_at_release:+7.3f} release / "
                        f"{point.static_margin_at_burnout:+7.3f} burnout  "
                        f"Cn_b {point.cn_beta:+.4f}",
                        flush=True,
                    )
                else:
                    print(
                        f"[{index:>2}/{total}] wing {wing_le:.2f} m  fin {fin_ratio:.2f}  "
                        f"FAILED: {point.note}",
                        flush=True,
                    )

    (output_dir / "summary.md").write_text(
        _summarise(results, sizing, args.mach), encoding="utf-8"
    )
    print(f"\nWrote {csv_path} and {output_dir / 'summary.md'}")
    return 0


def plot_sweep(results: list[SweepPoint], output_dir: Path) -> Path | None:
    """Static margin and Cn_beta against fin area, one line per wing station."""

    ok = [p for p in results if p.ok]
    if not ok:
        return None
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    stations = sorted({p.wing_le_x_m for p in ok})
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(7.2, 7.0), sharex=True)
    for station in stations:
        series = sorted(
            (p for p in ok if abs(p.wing_le_x_m - station) < 1e-9),
            key=lambda p: p.fin_area_ratio,
        )
        ratios = [p.fin_area_ratio for p in series]
        top.plot(
            ratios,
            [p.static_margin_at_release for p in series],
            "-o", ms=4, label=f"wing LE {station:.2f} m",
        )
        bottom.plot(
            ratios, [p.cn_beta for p in series], "-o", ms=4,
            label=f"wing LE {station:.2f} m",
        )

    top.axhline(0.0, lw=1.0, color="0.4")
    top.axhline(TARGET_STATIC_MARGIN, lw=0.8, ls="--", color="seagreen")
    top.set_ylabel("static margin (cref)")
    top.set_title(
        "Pitch: stable above 0. Fin area and wing station both work.", fontsize=10
    )
    top.grid(alpha=0.3)
    top.legend(fontsize=8)

    bottom.axhline(0.0, lw=1.0, color="0.4")
    bottom.set_xlabel("total fin area / wing reference area")
    bottom.set_ylabel(r"$C_{n\beta}$ (/rad)")
    bottom.set_title(
        "Yaw: stable above 0, and nearly independent of wing station —\n"
        "fin area alone sizes this axis, and it is the binding constraint.",
        fontsize=10,
    )
    bottom.grid(alpha=0.3)
    bottom.legend(fontsize=8)

    figure.tight_layout()
    path = output_dir / "stability_sweep.png"
    figure.savefig(path, dpi=140)
    plt.close(figure)
    return path


def _summarise(results: list[SweepPoint], sizing: SizingInput, mach: float) -> str:
    ok = [p for p in results if p.ok]
    stable = [
        p
        for p in ok
        if p.static_margin_at_release is not None
        and p.static_margin_at_release > 0.0
        and p.static_margin_at_burnout is not None
        and p.static_margin_at_burnout > 0.0
    ]
    lines = [
        "# Douglas Dart V4 — wing station / fin area stability sweep",
        "",
        f"Source: `{sizing.source_path}` (git `{sizing.git_sha[:7]}`). "
        f"Vortex-lattice, Mach {mach:.2f}, 6 solves per configuration.",
        "",
        f"{len(ok)}/{len(results)} configurations solved; "
        f"**{len(stable)}** are pitch-stable at BOTH release and burnout.",
        "",
        "Static margin is quoted in reference chords (cref = 296.3 mm), positive",
        "stable. Both fuel states are shown because the tank is an annulus around",
        "the tailpipe, well aft of the dry CG, so the CG moves ~81 mm forward over",
        "the burn and the margin moves with it.",
        "",
        "| wing LE (m) | fin ratio | fin span (mm) | x_np (mm) | SM release | SM burnout | Cn_beta | ballast for +0.10 SM |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for p in results:
        if not p.ok:
            lines.append(
                f"| {p.wing_le_x_m:.2f} | {p.fin_area_ratio:.2f} | — | — | — | — | — | "
                f"{p.note[:60]} |"
            )
            continue
        ballast = (
            f"{p.ballast_x_for_target_m * 1e3:.0f} mm"
            + ("" if p.ballast_feasible else " (OUTSIDE BODY)")
            if p.ballast_x_for_target_m is not None
            else "—"
        )
        lines.append(
            f"| {p.wing_le_x_m:.2f} | {p.fin_area_ratio:.2f} | "
            f"{p.total_span_m * 1e3:.0f} | {p.neutral_point_x_m * 1e3:.1f} | "
            f"{p.static_margin_at_release:+.3f} | {p.static_margin_at_burnout:+.3f} | "
            f"{'' if p.cn_beta is None else f'{p.cn_beta:+.4f}'} | {ballast} |"
        )

    lines.append("")
    lines.append(
        "> ## DO NOT SIZE ANYTHING FROM THIS TABLE YET\n"
        ">\n"
        "> These derivatives are **mesh-dependent to the point of changing the "
        "answer**. Holding the aerodynamics completely fixed (wing LE 1.00 m, "
        "fin ratio 1.40, M 0.50) and varying only the surface-root mount depth "
        "-- a meshing artifact with no physical meaning -- gives:\n"
        ">\n"
        "> | mount depth | static margin | Cn_beta |\n"
        "> |---|---|---|\n"
        "> | 6.0 mm | +0.391 | +0.656 |\n"
        "> | 10.5 mm | -1.885 | -6.066 |\n"
        "> | 13.5 mm | +0.072 | -0.080 |\n"
        ">\n"
        "> `Cm_alpha` changes sign; static margin spans 2.3 chords. Four of "
        "seven depths did not converge at all. So a single solve does not "
        "determine stability for a given geometry, and this sweep was partly "
        "measuring fin area and partly measuring which mesh each configuration "
        "landed on -- configurations that needed a retry got a different mesh "
        "from their neighbours, which is the likely source of the fin 1.50 "
        "outlier.\n"
        ">\n"
        "> **Prerequisite:** a mesh-convergence study at the fin/body junction "
        "(raise `surface_tessellation` and `fuselage_tessellation` until the "
        "derivatives stop responding to mount depth). See `README.md`.\n"
    )
    lines.append("")
    # Directional stability is the binding constraint, and it is set almost
    # entirely by fin area -- Cn_beta barely moves with wing station, which is
    # the expected physics and a useful check on the run.
    directional = [p for p in stable if p.cn_beta is not None and p.cn_beta > 0.0]
    if directional:
        # The right pick is the SMALLEST fin that closes both axes, not the
        # largest static margin: excess margin is excess fin mass and drag, and
        # pitch margin can afterwards be trimmed down to target with ballast
        # alone (see the ballast column). Ties broken by the least over-stable.
        best = min(
            directional,
            key=lambda p: (p.fin_area_ratio, abs(p.static_margin_at_release - TARGET_STATIC_MARGIN)),
        )
        lines.append(
            f"**Smallest configuration stable in BOTH axes:** wing LE "
            f"{best.wing_le_x_m:.2f} m, fin ratio {best.fin_area_ratio:.2f} "
            f"({best.fin_area_ratio * 0.152499:.3f} m^2 of fin, span "
            f"{best.total_span_m * 1e3:.0f} mm) — static margin "
            f"{best.static_margin_at_release:+.3f} release / "
            f"{best.static_margin_at_burnout:+.3f} burnout, "
            f"Cn_beta {best.cn_beta:+.3f}."
        )
        if best.ballast_x_for_target_m is not None and best.ballast_feasible:
            lines.append("")
            lines.append(
                f"That configuration is over-stable in pitch; moving the 4.68 kg "
                f"ballast to {best.ballast_x_for_target_m * 1e3:.0f} mm trims it to "
                f"the {TARGET_STATIC_MARGIN:+.2f} cref target without touching the "
                "aerodynamics."
            )
        pitch_only = [p for p in stable if p not in directional]
        if pitch_only:
            lines.append("")
            lines.append(
                f"{len(pitch_only)} further configurations are pitch-stable but "
                "still directionally UNSTABLE. Cn_beta is set almost entirely by "
                "fin area and is nearly independent of wing station, so yaw -- not "
                "pitch -- is what sizes the tail on this vehicle."
            )
    elif stable:
        lines.append(
            "Configurations are pitch-stable but **none is directionally stable**: "
            "every Cn_beta in the set is negative. Fin area sizes the yaw axis and "
            "the range does not reach far enough."
        )
    else:
        lines.append(
            "**No configuration in this range is statically stable.** The neutral "
            "point stays ahead of the CG throughout, which is the slender body's "
            "Munk moment outrunning everything the surfaces can do at these sizes. "
            "The levers left are a longer tail arm, a much larger fin, moving the "
            "CG forward beyond what ballast placement allows, or accepting an "
            "unstable airframe with active control."
        )
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
