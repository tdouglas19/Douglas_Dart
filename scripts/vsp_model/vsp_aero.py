"""Run VSPAERO point sweeps against a generated model and export CSV.

Every Mach/alpha/beta combination is executed as its own single-point run rather
than as a VSPAERO start/end/count sweep: the Mach list is deliberately
nonuniform, and a swept run would linearly interpolate between the endpoints and
analyse conditions nobody asked for.

WHAT THE NUMBERS ARE. VSPAERO here is an inviscid potential solution. ``CDtot``
contains induced and (in panel mode) volume-wave drag -- it does NOT contain skin
friction, base drag, or the inlet spillage/additive term. The trajectory model's
own build-up (``medium_model/drag_buildup.py``) supplies all three, and the
annular base this vehicle carries at the boattail is a large one. Do not
substitute a VSPAERO CD for the build-up's total.
"""

from __future__ import annotations

import csv
import os
import shutil
import time
from math import isfinite
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from geometry_inputs import SweepGrid
from openvsp_env import check_errors, clear_errors, load_openvsp
from vsp_build import THICK_SET_NAME, THIN_SET_NAME
from vehicle_geometry import VehicleGeometry

# Force/moment coefficients pulled from every point when present.
_FORCE_FIELDS = (
    "CLtot",
    "CDtot",
    "CDi",
    "CDo",
    "CStot",
    "CFx",
    "CFy",
    "CFz",
    "CMx",
    "CMy",
    "CMz",
    "CMxtot",
    "CMytot",
    "CMztot",
    "L/D",
    "E",
)

# Stability-mode derivative names. VSPAERO emits one column per derivative in a
# separate stability result; missing ones are simply left blank.
_STABILITY_FIELDS = (
    "CFx",
    "CFy",
    "CFz",
    "CMx",
    "CMy",
    "CMz",
    "CL",
    "CD",
    "CS",
    "CMl",
    "CMm",
    "CMn",
)


@dataclass
class AeroPoint:
    grid: str
    method: str
    mach: float
    alpha_deg: float
    beta_deg: float
    solve_seconds: float
    coefficients: dict[str, float] = field(default_factory=dict)
    derivatives: dict[str, float] = field(default_factory=dict)

    def row(self) -> dict[str, Any]:
        row: dict[str, Any] = {
            "grid": self.grid,
            "method": self.method,
            "mach": self.mach,
            "alpha_deg": self.alpha_deg,
            "beta_deg": self.beta_deg,
            "solve_s": round(self.solve_seconds, 2),
        }
        row.update({k: v for k, v in self.coefficients.items()})
        row.update({f"d_{k}": v for k, v in self.derivatives.items()})
        return row


@dataclass
class SweepResult:
    grid: SweepGrid
    points: tuple[AeroPoint, ...]
    compute_geometry_id: str
    wall_seconds: float
    solver_files: tuple[str, ...]


def _solver_threads() -> int:
    """Threads to hand VSPAERO.

    Leave two cores for the rest of the machine -- these runs are long enough
    that starving the interactive session is a real cost.
    """

    cores = os.cpu_count() or 4
    return max(1, min(cores - 2, 16))


def _set_geometry_sets(vsp: Any, analysis: str, method: str) -> None:
    """Assign thick and thin geometry for the chosen method.

    This build exposes no ``AnalysisMethod`` input and no ``PANEL`` /
    ``VORTEX_LATTICE`` constants at all -- confirmed on the installed 3.51.2
    Windows build. The geometry sets ARE the selector.

    ``vortex_lattice`` uses the two named sets ``vsp_build`` writes into the
    model: the fuselage thick, the lifting panels thin. Sweeping everything into
    one set is wrong and fails SILENTLY -- with the whole model in the thin set,
    the bare body at alpha = 4 deg returned CL = 0.0000 and CS = -0.0567,
    because a fuselage solved as a zero-thickness lifting sheet puts its normal
    force on the wrong axis. Verified by rebuilding with the split.

    ``panel`` puts everything in the thick set, which is correct: the panel
    method meshes real surfaces and has no thin representation to fall back on.
    """

    if method == "panel":
        thick, thin = vsp.SET_ALL, vsp.SET_NONE
    elif method == "vortex_lattice":
        thick = vsp.GetSetIndex(THICK_SET_NAME)
        thin = vsp.GetSetIndex(THIN_SET_NAME)
        if thick < 0 or thin < 0:
            raise RuntimeError(
                f"the model has no {THICK_SET_NAME!r}/{THIN_SET_NAME!r} sets. It "
                "was built by an older version of vsp_build; rebuild it, because "
                "a vortex-lattice run without the split silently mis-solves the body."
            )
    else:
        raise ValueError(f"unsupported VSPAERO method: {method!r}")
    vsp.SetIntAnalysisInput(analysis, "GeomSet", [thick], 0)
    vsp.SetIntAnalysisInput(analysis, "ThinGeomSet", [thin], 0)


def _set_point_inputs(
    vsp: Any,
    analysis: str,
    geometry: VehicleGeometry,
    grid: SweepGrid,
    mach: float,
    alpha_deg: float,
    beta_deg: float,
) -> None:
    references = geometry.references
    vsp.SetIntAnalysisInput(analysis, "RefFlag", [vsp.MANUAL_REF], 0)
    vsp.SetDoubleAnalysisInput(analysis, "Sref", [references.area_m2], 0)
    vsp.SetDoubleAnalysisInput(analysis, "bref", [references.span_m], 0)
    vsp.SetDoubleAnalysisInput(analysis, "cref", [references.chord_m], 0)
    vsp.SetDoubleAnalysisInput(analysis, "Xcg", [references.cg_x_m], 0)
    vsp.SetDoubleAnalysisInput(analysis, "Ycg", [0.0], 0)
    vsp.SetDoubleAnalysisInput(analysis, "Zcg", [0.0], 0)
    # Symmetry is a Boolean here. A sideslip point needs the full model, and the
    # cruciform fins make the vehicle asymmetric about the wing plane anyway.
    vsp.SetIntAnalysisInput(analysis, "Symmetry", [False], 0)
    vsp.SetIntAnalysisInput(analysis, "WakeNumIter", [grid.wake_iterations], 0)
    vsp.SetIntAnalysisInput(analysis, "NCPU", [_solver_threads()], 0)
    if grid.stability:
        vsp.SetIntAnalysisInput(analysis, "UnsteadyType", [vsp.STABILITY_DEFAULT], 0)
    else:
        vsp.SetIntAnalysisInput(analysis, "UnsteadyType", [vsp.STABILITY_OFF], 0)

    for prefix, value in (("Alpha", alpha_deg), ("Beta", beta_deg), ("Mach", mach)):
        vsp.SetDoubleAnalysisInput(analysis, f"{prefix}Start", [value], 0)
        vsp.SetDoubleAnalysisInput(analysis, f"{prefix}End", [value], 0)
        vsp.SetIntAnalysisInput(analysis, f"{prefix}Npts", [1], 0)


def _result_ids_after(vsp: Any, sweep_id: str) -> tuple[str, ...]:
    ids = tuple(str(v) for v in vsp.GetStringResults(sweep_id, "ResultsVec", 0))
    if not ids:
        raise RuntimeError("VSPAERO returned no results for a configured point")
    return ids


def _latest_result_containing(
    vsp: Any, result_ids: tuple[str, ...], marker: str
) -> str | None:
    """Newest result whose data names include ``marker``.

    ``ResultsVec`` accumulates across the WHOLE session rather than resetting per
    call, and one sweep appends several entries (a force/moment history plus
    rotor-style and CpSlice summaries that exist even with no propellers or
    slices configured). Neither "exactly one" nor "the last one" is safe;
    scanning backward for the entry that actually carries the wanted field is.
    """

    for candidate in reversed(result_ids):
        names = {str(v) for v in vsp.GetAllDataNames(candidate)}
        if marker in names:
            return candidate
    return None


COEFFICIENT_SANITY_BOUND = 100.0
"""Magnitude past which a returned coefficient is a broken solve, not a result.

Force and moment coefficients on this vehicle are O(1) -- CL peaks near 0.4, Cm
near 0.9. A partially-diverged VSPAERO solve, though, can return large FINITE
garbage rather than NaN: measured on one sweep configuration, CL came back as
-2.3e7 with the run reporting success and every value finite. 100 is ~2 orders
of magnitude above anything physical here and ~5 below the observed failure, so
it separates them without any risk of rejecting a real point.
"""


def _reject_implausible(
    coefficients: dict[str, float], mach: float, alpha_deg: float, beta_deg: float
) -> None:
    """Refuse to record a diverged or partially-diverged solve.

    VSPAERO reports divergence by returning bad numbers, not by failing. Two
    distinct modes have been seen on this model, both with a normal exit code:

    * fully diverged -- GMRES prints ``Red: nan`` and every coefficient is NaN;
    * partially diverged -- finite but absurd values (CL = -2.3e7).

    The second is the more dangerous, because it survives a naive isfinite check
    and lands in a CSV looking like data. Both are caught here.
    """

    nonfinite = sorted(n for n, v in coefficients.items() if not isfinite(v))
    if nonfinite:
        raise RuntimeError(
            f"VSPAERO returned non-finite {', '.join(nonfinite)} at M{mach} "
            f"alpha={alpha_deg} beta={beta_deg}. The solve diverged -- GMRES "
            "returns NaN without raising. The usual cause is a lifting-surface "
            "root intersecting the body badly; see "
            "sweep_stability.OVERLAP_ATTEMPTS_M."
        )
    absurd = sorted(
        f"{n}={v:.4g}"
        for n, v in coefficients.items()
        if abs(v) > COEFFICIENT_SANITY_BOUND
    )
    if absurd:
        raise RuntimeError(
            f"VSPAERO returned implausible {', '.join(absurd)} at M{mach} "
            f"alpha={alpha_deg} beta={beta_deg} (bound "
            f"{COEFFICIENT_SANITY_BOUND:g}). The solve partially diverged: the "
            "values are finite but not physical. Same remedy as a NaN -- perturb "
            "the surface root depth."
        )


def _read_final(vsp: Any, result_id: str, name: str) -> float | None:
    names = {str(v) for v in vsp.GetAllDataNames(result_id)}
    if name not in names:
        return None
    values = list(vsp.GetDoubleResults(result_id, name, 0))
    # The history carries one row per wake iteration; the converged value is last.
    return float(values[-1]) if values else None


def run_sweep(
    geometry: VehicleGeometry,
    model_path: str | Path,
    grid: SweepGrid,
    output_dir: str | Path,
    *,
    progress: bool = True,
) -> SweepResult:
    """Execute one grid and return its points."""

    model_path = Path(model_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    if not model_path.is_file():
        raise FileNotFoundError(f"OpenVSP model does not exist: {model_path}")

    vsp = load_openvsp()
    vsp.VSPRenew()
    vsp.ClearVSPModel()
    clear_errors(vsp)
    vsp.ReadVSPFile(str(model_path))
    vsp.Update()
    check_errors(vsp, "loading the model for VSPAERO")

    compute = "VSPAEROComputeGeometry"
    vsp.SetAnalysisInputDefaults(compute)
    _set_geometry_sets(vsp, compute, grid.method)
    compute_id = str(vsp.ExecAnalysis(compute))
    check_errors(vsp, "VSPAEROComputeGeometry")
    if not compute_id:
        raise RuntimeError("VSPAEROComputeGeometry returned no results ID")

    started = time.perf_counter()
    points: list[AeroPoint] = []
    total = grid.point_count()
    index = 0
    for mach in grid.mach_values:
        for beta_deg in grid.beta_deg_values:
            for alpha_deg in grid.alpha_deg_values:
                index += 1
                point_started = time.perf_counter()
                sweep = "VSPAEROSweep"
                vsp.SetAnalysisInputDefaults(sweep)
                _set_geometry_sets(vsp, sweep, grid.method)
                _set_point_inputs(
                    vsp, sweep, geometry, grid, mach, alpha_deg, beta_deg
                )
                vsp.Update()
                sweep_id = str(vsp.ExecAnalysis(sweep))
                check_errors(vsp, f"VSPAEROSweep at M{mach} a{alpha_deg} b{beta_deg}")
                if not sweep_id:
                    raise RuntimeError("VSPAEROSweep returned no results ID")

                result_ids = _result_ids_after(vsp, sweep_id)
                history_id = _latest_result_containing(vsp, result_ids, "CLtot")
                if history_id is None:
                    raise RuntimeError(
                        "no VSPAERO result carries 'CLtot'; got " + ", ".join(result_ids)
                    )
                coefficients = {
                    name: value
                    for name in _FORCE_FIELDS
                    if (value := _read_final(vsp, history_id, name)) is not None
                }

                _reject_implausible(coefficients, mach, alpha_deg, beta_deg)

                derivatives: dict[str, float] = {}
                if grid.stability:
                    derivatives = _read_stability(vsp, result_ids)

                elapsed = time.perf_counter() - point_started
                points.append(
                    AeroPoint(
                        grid=grid.name,
                        method=grid.method,
                        mach=mach,
                        alpha_deg=alpha_deg,
                        beta_deg=beta_deg,
                        solve_seconds=elapsed,
                        coefficients=coefficients,
                        derivatives=derivatives,
                    )
                )
                if progress:
                    cl = coefficients.get("CLtot")
                    cd = coefficients.get("CDtot")
                    cm = coefficients.get("CMytot", coefficients.get("CMy"))
                    # Leading newline on purpose: VSPAERO writes its own progress
                    # to the same stream without terminating the last line, so an
                    # unprefixed print gets concatenated onto solver output and
                    # becomes ungreppable.
                    print(
                        f"\n[{index:>3}/{total}] {grid.name:<14} M{mach:.2f} "
                        f"a{alpha_deg:+5.1f} b{beta_deg:+5.1f}  "
                        f"CL={_fmt(cl)} CD={_fmt(cd)} CMy={_fmt(cm)}  {elapsed:5.1f}s",
                        flush=True,
                    )

    solver_files = _collect_solver_files(model_path, output_dir, grid.name)
    return SweepResult(
        grid=grid,
        points=tuple(points),
        compute_geometry_id=compute_id,
        wall_seconds=time.perf_counter() - started,
        solver_files=solver_files,
    )


def _fmt(value: float | None) -> str:
    return "   n/a" if value is None else f"{value:+.4f}"


def _read_stability(vsp: Any, result_ids: tuple[str, ...]) -> dict[str, float]:
    """Pull the rate/angle derivatives out of the stability result.

    VSPAERO's stability mode names its derivative columns per base coefficient
    and perturbation, e.g. ``CMm`` w.r.t. ``Alpha``. The layout varies between
    builds, so this reads whatever derivative-shaped entries the result actually
    carries instead of assuming a fixed set.
    """

    stability_id = _latest_result_containing(vsp, result_ids, "CMm_Alpha")
    if stability_id is None:
        stability_id = _latest_result_containing(vsp, result_ids, "CMm_Beta")
    if stability_id is None:
        return {}
    derivatives: dict[str, float] = {}
    for name in vsp.GetAllDataNames(stability_id):
        name = str(name)
        if "_" not in name:
            continue
        base, _, wrt = name.partition("_")
        if base not in _STABILITY_FIELDS:
            continue
        if wrt not in ("Alpha", "Beta", "p", "q", "r", "Mach", "U"):
            continue
        value = _read_final(vsp, stability_id, name)
        if value is not None:
            derivatives[name] = value
    return derivatives


def _collect_solver_files(
    model_path: Path, output_dir: Path, grid_name: str
) -> tuple[str, ...]:
    """Move VSPAERO's own output files into this grid's folder.

    The solver writes its .polar, .history, .stab, .lod and the mesh it actually
    solved beside the .vsp3, named after the model. They are the primary evidence
    for a run, so they are kept per grid.

    MOVED, not copied, and that matters: the names collide between grids, so
    leaving them in place would both let the next grid overwrite them and make
    this function sweep the previous grid's files into the next grid's folder.
    Each is regenerated by that grid's own VSPAEROComputeGeometry.
    """

    stem = model_path.stem
    destination = output_dir / "solver" / grid_name
    destination.mkdir(parents=True, exist_ok=True)
    kept: list[str] = []
    for candidate in sorted(model_path.parent.iterdir()):
        if not candidate.is_file() or candidate.suffix == ".vsp3":
            continue
        if not candidate.name.startswith(stem):
            continue
        target = destination / candidate.name
        target.unlink(missing_ok=True)
        shutil.move(str(candidate), str(target))
        kept.append(str(target))
    return tuple(kept)


def write_points_csv(results: list[SweepResult], path: str | Path) -> Path:
    """One tidy CSV across every grid, columns unioned."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [point.row() for result in results for point in result.points]
    if not rows:
        raise ValueError("no VSPAERO points to write")

    leading = ["grid", "method", "mach", "alpha_deg", "beta_deg", "solve_s"]
    extra = sorted({key for row in rows for key in row if key not in leading})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=leading + extra)
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_run_summary(results: list[SweepResult]) -> list[dict[str, Any]]:
    return [
        {
            "grid": result.grid.name,
            "method": result.grid.method,
            "points": len(result.points),
            "wall_seconds": round(result.wall_seconds, 1),
            "stability": result.grid.stability,
            "grid_definition": asdict(result.grid),
            "solver_files": [Path(f).name for f in result.solver_files],
        }
        for result in results
    ]
