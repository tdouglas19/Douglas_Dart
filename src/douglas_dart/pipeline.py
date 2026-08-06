"""End-to-end analysis pipeline for the Douglas Dart reference configuration.

The pipeline calls the same model functions exposed by the individual CLI commands.
It writes provenance, machine-readable results, CSV tables, plots, the OpenVSP model,
and, when requested and available, a live VSPAERO sweep into one output tree.
"""

from __future__ import annotations

import csv
import json
import math
import platform
import shutil
import subprocess
import traceback
from dataclasses import asdict, dataclass, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from time import perf_counter
from typing import Any, Callable, Iterable, Sequence

from .config import load_fuels, load_reference_case
from .fuel_trade import fuel_performance_trade
from .jsbsim_model import validate_with_jsbsim, write_jsbsim_aircraft
from .openvsp_geometry import build_openvsp_geometry
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet
from .robustness import run_robustness_trade
from .sensitivity import (
    pulsejet_local_sensitivities,
    ramjet_local_sensitivities,
    rank_by_net_thrust_sensitivity,
)
from .sizing import (
    peak_mach_altitude_trade_sweep,
    peak_mach_diameter_trade_sweep,
    ramjet_handoff_sweep,
    select_minimum_feasible_shared_nozzle,
    shared_nozzle_feasibility_bounds,
    shared_nozzle_trade_sweep,
)
from .trajectory import ADVERSE_SCENARIO, NOMINAL_SCENARIO, simulate_mission
from .vspaero import run_vspaero_sweep


@dataclass(frozen=True)
class PipelineStageRecord:
    name: str
    status: str
    required: bool
    duration_s: float
    output_files: tuple[str, ...]
    message: str | None = None


@dataclass(frozen=True)
class PipelineRunSummary:
    case_name: str
    config_path: str
    fuels_path: str
    output_directory: str
    openvsp_model_path: str
    started_at_utc: str
    completed_at_utc: str
    successful: bool
    stages: tuple[PipelineStageRecord, ...]
    note: str = (
        "The pipeline executes every currently implemented low-order analysis, "
        "including the phase-based nominal/adverse mission trajectory integrator "
        "and JSBSim aircraft-model generation with a live load-and-run check."
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _git_provenance(repository_root: Path) -> dict[str, Any]:
    """Return revision metadata without making pipeline execution depend on Git."""

    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repository_root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=repository_root,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return {"git_commit_sha": commit, "git_worktree_dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"git_commit_sha": None, "git_worktree_dirty": None}


def _json_ready(value: Any) -> Any:
    if is_dataclass(value):
        return _json_ready(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _write_json(path: Path, value: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_ready(value), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return path


def _row_mapping(value: Any) -> dict[str, Any]:
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, dict):
        return dict(value)
    raise TypeError(f"CSV rows must be dataclasses or mappings, got {type(value)!r}")


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, (tuple, list, set, dict)):
        return json.dumps(_json_ready(value), sort_keys=True)
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def _write_csv(path: Path, rows: Iterable[Any]) -> Path:
    materialized = [_row_mapping(row) for row in rows]
    if not materialized:
        raise ValueError(f"cannot write empty CSV: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(materialized[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in materialized:
            writer.writerow({key: _csv_value(row.get(key)) for key in fieldnames})
    return path


def _load_pyplot() -> Any:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Plot generation requires matplotlib. Install the project dependencies "
            "with 'python -m pip install -e .'."
        ) from exc
    return plt


def _save_plot(plt: Any, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=180, bbox_inches="tight")
    plt.close()
    return path


def _plot_pulsejet(samples: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    time_s = [point.time_s for point in samples]

    plt.figure(figsize=(9.0, 5.2))
    plt.plot(time_s, [point.gross_thrust_n for point in samples], label="Gross thrust")
    plt.plot(time_s, [point.net_thrust_n for point in samples], label="Net thrust")
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust (N)")
    plt.title("Pulsejet thrust history")
    plt.grid(True, alpha=0.25)
    plt.legend()
    paths.append(_save_plot(plt, plot_dir / "pulsejet_thrust_history.png"))

    plt.figure(figsize=(9.0, 5.2))
    plt.plot(time_s, [point.chamber_pressure_pa / 1000.0 for point in samples])
    plt.xlabel("Time (s)")
    plt.ylabel("Chamber pressure (kPa)")
    plt.title("Pulsejet chamber pressure")
    plt.grid(True, alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "pulsejet_chamber_pressure.png"))

    plt.figure(figsize=(9.0, 5.2))
    plt.plot(time_s, [point.chamber_temperature_k for point in samples])
    plt.xlabel("Time (s)")
    plt.ylabel("Chamber temperature (K)")
    plt.title("Pulsejet chamber temperature")
    plt.grid(True, alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "pulsejet_chamber_temperature.png"))
    return paths


def _plot_ramjet_handoff(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    mach = [point.mach for point in points]

    plt.figure(figsize=(8.0, 5.0))
    plt.plot(
        mach,
        [point.current_net_thrust_n for point in points],
        marker="o",
        label="Configured nozzle",
    )
    plt.plot(
        mach,
        [point.matched_net_thrust_n for point in points],
        marker="o",
        label="Flow-matched throat",
    )
    plt.xlabel("Mach")
    plt.ylabel("Net thrust (N)")
    plt.title("Ramjet handoff thrust")
    plt.grid(True, alpha=0.25)
    plt.legend()
    paths.append(_save_plot(plt, plot_dir / "ramjet_handoff_thrust.png"))

    plt.figure(figsize=(8.0, 5.0))
    plt.plot(
        mach,
        [1000.0 * point.required_matched_throat_diameter_m for point in points],
        marker="o",
        label="Required matched throat",
    )
    plt.plot(
        mach,
        [1000.0 * point.nominal_body_diameter_m for point in points],
        linestyle="--",
        label="Nominal body diameter",
    )
    plt.xlabel("Mach")
    plt.ylabel("Diameter (mm)")
    plt.title("Ramjet flow-match throat requirement")
    plt.grid(True, alpha=0.25)
    plt.legend()
    paths.append(_save_plot(plt, plot_dir / "ramjet_handoff_throat.png"))
    return paths


def _plot_diameter_trade(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    diameter_mm = [1000.0 * point.body_diameter_m for point in points]

    plt.figure(figsize=(8.0, 5.0))
    plt.plot(
        diameter_mm,
        [point.thrust_margin_against_scaled_drag_target_n for point in points],
        marker="o",
    )
    plt.axhline(0.0, linewidth=1.0)
    plt.xlabel("Body diameter (mm)")
    plt.ylabel("Ramjet thrust margin (N)")
    plt.title("Body diameter versus peak-Mach thrust margin")
    plt.grid(True, alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "diameter_trade_thrust_margin.png"))

    plt.figure(figsize=(8.0, 5.0))
    plt.plot(
        diameter_mm,
        [
            point.fuel_limited_peak_mach_hold_duration_s
            if point.fuel_limited_peak_mach_hold_duration_s is not None
            else float("nan")
            for point in points
        ],
        marker="o",
    )
    plt.xlabel("Body diameter (mm)")
    plt.ylabel("Fuel-limited hold duration (s)")
    plt.title("Body diameter versus static peak-Mach hold")
    plt.grid(True, alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "diameter_trade_hold_duration.png"))
    return paths


def _plot_shared_nozzle(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    groups: dict[tuple[float, float], list[Any]] = {}
    for point in points:
        groups.setdefault(
            (point.body_diameter_m, point.exit_to_throat_area_ratio), []
        ).append(point)

    plt.figure(figsize=(8.5, 5.2))
    for (body_diameter_m, area_ratio), group in sorted(groups.items()):
        group = sorted(group, key=lambda point: point.throat_diameter_m)
        plt.plot(
            [1000.0 * point.throat_diameter_m for point in group],
            [point.ramjet_derated_thrust_margin_n for point in group],
            marker="o",
            label=f"D={1000.0 * body_diameter_m:.0f} mm, Ae/At={area_ratio:.2f}",
        )
    plt.axhline(0.0, linewidth=1.0)
    plt.xlabel("Throat diameter (mm)")
    plt.ylabel("15%-derated thrust margin (N)")
    plt.title("Shared-nozzle ramjet margin")
    plt.grid(True, alpha=0.25)
    plt.legend(fontsize="small")
    paths.append(_save_plot(plt, plot_dir / "shared_nozzle_thrust_margin.png"))

    plt.figure(figsize=(8.5, 5.2))
    for (body_diameter_m, area_ratio), group in sorted(groups.items()):
        group = sorted(group, key=lambda point: point.throat_diameter_m)
        plt.plot(
            [1000.0 * point.throat_diameter_m for point in group],
            [100.0 * point.ramjet_inlet_spillage_fraction for point in group],
            marker="o",
            label=f"D={1000.0 * body_diameter_m:.0f} mm, Ae/At={area_ratio:.2f}",
        )
    plt.xlabel("Throat diameter (mm)")
    plt.ylabel("Ramjet inlet spillage (%)")
    plt.title("Shared-nozzle spillage trade")
    plt.grid(True, alpha=0.25)
    plt.legend(fontsize="small")
    paths.append(_save_plot(plt, plot_dir / "shared_nozzle_spillage.png"))
    return paths


def _plot_altitude_trade(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    altitude_km = [point.altitude_m / 1000.0 for point in points]

    plt.figure(figsize=(8.0, 5.0))
    plt.plot(
        altitude_km,
        [point.ramjet_derated_thrust_margin_n for point in points],
        marker="o",
    )
    plt.axhline(0.0, linewidth=1.0)
    plt.xlabel("Altitude MSL (km)")
    plt.ylabel("15%-derated thrust margin (N)")
    plt.title("Peak-Mach altitude trade")
    plt.grid(True, alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "altitude_trade_thrust_margin.png"))

    plt.figure(figsize=(8.0, 5.0))
    plt.plot(
        altitude_km,
        [
            point.ramjet_fuel_limited_hold_duration_s
            if point.ramjet_fuel_limited_hold_duration_s is not None
            else float("nan")
            for point in points
        ],
        marker="o",
    )
    plt.xlabel("Altitude MSL (km)")
    plt.ylabel("Fuel-limited hold duration (s)")
    plt.title("Altitude versus static peak-Mach hold")
    plt.grid(True, alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "altitude_trade_hold_duration.png"))
    return paths


def _plot_fuel_trade(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    labels = [point.display_name for point in points]
    positions = list(range(len(points)))

    plt.figure(figsize=(9.0, max(4.5, 0.55 * len(points) + 2.0)))
    plt.barh(positions, [point.ramjet_net_thrust_n for point in points])
    plt.yticks(positions, labels)
    plt.xlabel("Ramjet net thrust (N)")
    plt.title("Fuel performance trade: ramjet thrust")
    plt.grid(True, axis="x", alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "fuel_trade_ramjet_thrust.png"))

    plt.figure(figsize=(9.0, max(4.5, 0.55 * len(points) + 2.0)))
    plt.barh(positions, [point.loaded_fuel_volume_l for point in points])
    plt.yticks(positions, labels)
    plt.xlabel("Loaded fuel volume (L)")
    plt.title("Fuel performance trade: tank volume before hardware allowance")
    plt.grid(True, axis="x", alpha=0.25)
    paths.append(_save_plot(plt, plot_dir / "fuel_trade_loaded_volume.png"))
    return paths


def _plot_sensitivities(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    labels = [f"{point.engine}: {point.variable}" for point in points]
    positions = list(range(len(points)))
    plt.figure(figsize=(10.5, max(5.0, 0.42 * len(points) + 2.0)))
    plt.barh(positions, [point.net_thrust_normalized_slope for point in points])
    plt.yticks(positions, labels)
    plt.axvline(0.0, linewidth=1.0)
    plt.xlabel("Normalized net-thrust sensitivity")
    plt.title("Local propulsion key-variable sensitivities")
    plt.grid(True, axis="x", alpha=0.25)
    return [_save_plot(plt, plot_dir / "key_variable_sensitivities.png")]


def _plot_vspaero(points: Sequence[Any], plot_dir: Path) -> list[Path]:
    plt = _load_pyplot()
    paths: list[Path] = []
    zero_beta = [point for point in points if abs(point.beta_deg) <= 1e-12]
    mach_values = sorted({point.mach for point in zero_beta})

    plt.figure(figsize=(8.5, 5.2))
    for mach in mach_values:
        group = sorted(
            (point for point in zero_beta if point.mach == mach),
            key=lambda point: point.alpha_deg,
        )
        plt.plot(
            [point.alpha_deg for point in group],
            [point.lift_coefficient for point in group],
            marker="o",
            label=f"Mach {mach:.2f}",
        )
    plt.xlabel("Angle of attack (deg)")
    plt.ylabel("Lift coefficient")
    plt.title("VSPAERO lift curves at zero sideslip")
    plt.grid(True, alpha=0.25)
    plt.legend()
    paths.append(_save_plot(plt, plot_dir / "vspaero_lift_curves.png"))

    plt.figure(figsize=(8.5, 5.2))
    for mach in mach_values:
        group = sorted(
            (point for point in zero_beta if point.mach == mach),
            key=lambda point: point.lift_coefficient,
        )
        plt.plot(
            [point.drag_coefficient_inviscid for point in group],
            [point.lift_coefficient for point in group],
            marker="o",
            label=f"Mach {mach:.2f}",
        )
    plt.xlabel("Inviscid drag coefficient")
    plt.ylabel("Lift coefficient")
    plt.title("VSPAERO inviscid drag polar at zero sideslip")
    plt.grid(True, alpha=0.25)
    plt.legend()
    paths.append(_save_plot(plt, plot_dir / "vspaero_drag_polar.png"))
    return paths


def _write_summary_markdown(
    path: Path,
    *,
    case: Any,
    stage_records: Sequence[PipelineStageRecord],
    pulsejet_summary: Any | None,
    ramjet_result: Any | None,
    convergence: Any | None,
    selected_nozzle: Any | None,
    robustness_result: Any | None,
) -> Path:
    lines = [
        f"# Douglas Dart analysis run: {case.name}",
        "",
        "This report is generated by the repository pipeline. Numerical outputs remain",
        "low-order engineering references until the validation gates in `docs/validation.md` close.",
        "",
        "## Pipeline status",
        "",
        "| Stage | Status | Required | Duration (s) |",
        "|---|---:|---:|---:|",
    ]
    for record in stage_records:
        lines.append(
            f"| {record.name} | {record.status} | {'yes' if record.required else 'no'} | "
            f"{record.duration_s:.3f} |"
        )

    lines.extend(["", "## Key numerical outputs", ""])
    if pulsejet_summary is not None:
        lines.extend(
            [
                f"- Pulsejet steady-window mean net thrust: {pulsejet_summary.mean_net_thrust_n:.1f} N",
                f"- Pulsejet peak chamber pressure: {pulsejet_summary.peak_chamber_pressure_pa / 1000.0:.1f} kPa",
                f"- Pulsejet completed cycles: {pulsejet_summary.completed_cycles}",
            ]
        )
    if ramjet_result is not None:
        lines.extend(
            [
                f"- Ramjet net thrust at Mach {case.mission.peak_mach:.2f}: {ramjet_result.net_thrust_n:.1f} N",
                f"- Ramjet inlet spillage: {100.0 * ramjet_result.inlet_spillage_fraction:.1f}%",
            ]
        )
    if convergence is not None:
        lines.extend(
            [
                f"- Minimum packageable body diameter: {1000.0 * convergence.minimum_packageable_body_diameter_m:.2f} mm",
                f"- Maximum body diameter under derated drag budget: {1000.0 * convergence.maximum_body_diameter_for_derated_drag_budget_m:.2f} mm",
            ]
        )
    if selected_nozzle is not None:
        lines.extend(
            [
                f"- Nominal-only grid selected feasible throat: {1000.0 * selected_nozzle.throat_diameter_m:.1f} mm",
                f"- Selected exit/throat area ratio: {selected_nozzle.exit_to_throat_area_ratio:.2f}",
            ]
        )
    else:
        lines.append("- No feasible point was selected from the configured shared-nozzle grid.")
    if robustness_result is not None:
        selected_robust = robustness_result.selected_candidate
        lines.extend(
            [
                f"- Component mass estimate / high-side total: {robustness_result.mass_budget.current_total_mass_kg:.1f} / {robustness_result.mass_budget.high_total_mass_kg:.1f} kg",
            ]
        )
        if selected_robust is not None:
            lines.append(
                "- Robustness-selected body / throat: "
                f"{1000.0 * selected_robust.body_diameter_m:.0f} / "
                f"{1000.0 * selected_robust.throat_diameter_m:.0f} mm"
            )
            for scenario in selected_robust.scenarios:
                lines.append(
                    f"- {scenario.scenario.title()} excess thrust: "
                    f"{scenario.excess_thrust_n:.1f} N"
                )

    lines.extend(
        [
            "",
            "## Coverage note",
            "",
            "The pipeline runs every currently implemented low-order propulsion, sizing, fuel, sensitivity, phase-based mission trajectory, JSBSim aircraft-model, OpenVSP, and optional VSPAERO analysis. Solver-backed aerodynamic tables (replacing the Mach-indexed drag-area proxy) are still pending.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def run_all_analyses(
    config_path: str | Path = "configs/shared_nozzle_candidate_b.yaml",
    *,
    fuels_path: str | Path | None = None,
    robustness_path: str | Path = "configs/robustness_candidate_b.yaml",
    output_directory: str | Path = "results/generated/shared_nozzle_candidate_b",
    openvsp_model_path: str | Path = "openvsp/generated/shared_nozzle_candidate_b.vsp3",
    body_diameter_max_m: float = 0.300,
    body_diameter_step_m: float = 0.005,
    altitude_min_m: float = 3000.0,
    altitude_max_m: float = 6500.0,
    altitude_step_m: float = 500.0,
    propulsion_derate_fraction: float = 0.15,
    run_openvsp: bool = True,
    run_vspaero: bool = True,
    run_trajectory: bool = True,
    run_jsbsim: bool = True,
    jsbsim_check: bool = True,
    jsbsim_root: str | Path = "jsbsim/generated",
) -> PipelineRunSummary:
    """Run all currently implemented analyses and write a complete result package."""

    started_at_utc = _utc_now()
    config_path = Path(config_path)
    robustness_path = Path(robustness_path)
    resolved_fuels_path = (
        Path(fuels_path) if fuels_path is not None else config_path.with_name("fuels.yaml")
    )
    output_directory = Path(output_directory)
    openvsp_model_path = Path(openvsp_model_path)
    jsbsim_root = Path(jsbsim_root)
    json_dir = output_directory / "json"
    csv_dir = output_directory / "csv"
    plot_dir = output_directory / "plots"
    log_dir = output_directory / "logs"
    input_dir = output_directory / "inputs"
    for directory in (json_dir, csv_dir, plot_dir, log_dir, input_dir):
        directory.mkdir(parents=True, exist_ok=True)

    case = load_reference_case(config_path, resolved_fuels_path)
    shutil.copy2(config_path, input_dir / config_path.name)
    shutil.copy2(resolved_fuels_path, input_dir / resolved_fuels_path.name)
    shutil.copy2(robustness_path, input_dir / robustness_path.name)
    environment_record = {
        "generated_at_utc": started_at_utc,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "config_path": str(config_path),
        "fuels_path": str(resolved_fuels_path),
        "robustness_path": str(robustness_path),
    }
    environment_record.update(
        _git_provenance(Path(__file__).resolve().parents[2])
    )
    _write_json(
        output_directory / "environment.json",
        environment_record,
    )

    stage_records: list[PipelineStageRecord] = []
    pulsejet_summary = None
    ramjet_result = None
    convergence = None
    selected_nozzle = None
    robustness_result = None

    def execute(
        name: str,
        required: bool,
        function: Callable[[], Sequence[Path] | Path | None],
    ) -> bool:
        start = perf_counter()
        try:
            outputs = function()
            if outputs is None:
                output_paths: tuple[Path, ...] = ()
            elif isinstance(outputs, Path):
                output_paths = (outputs,)
            else:
                output_paths = tuple(outputs)
            stage_records.append(
                PipelineStageRecord(
                    name=name,
                    status="completed",
                    required=required,
                    duration_s=perf_counter() - start,
                    output_files=tuple(str(path) for path in output_paths),
                )
            )
            return True
        except Exception as exc:
            error_path = log_dir / f"{name}.txt"
            error_path.write_text(traceback.format_exc(), encoding="utf-8")
            stage_records.append(
                PipelineStageRecord(
                    name=name,
                    status="failed",
                    required=required,
                    duration_s=perf_counter() - start,
                    output_files=(str(error_path),),
                    message=str(exc),
                )
            )
            return False

    pulsejet_samples: list[Any] = []

    def pulsejet_stage() -> Sequence[Path]:
        nonlocal pulsejet_summary, pulsejet_samples
        simulator = PulsejetSimulator(
            case.pulsejet,
            case.selector,
            case.nozzle,
            case.fuel,
            case.altitude_m,
            case.mach,
        )
        duration_s = (
            case.simulation.pulsejet_steady_warmup_s
            + case.simulation.pulsejet_steady_measurement_s
        )
        pulsejet_samples = simulator.run(duration_s, case.simulation.time_step_s)
        pulsejet_summary = summarize_pulsejet(
            pulsejet_samples,
            minimum_time_s=case.simulation.pulsejet_steady_warmup_s,
        )
        csv_path = _write_csv(csv_dir / "pulsejet_timeseries.csv", pulsejet_samples)
        json_path = _write_json(
            json_dir / "pulsejet_summary.json",
            {
                "summary": pulsejet_summary,
                "conservation_audit": simulator.conservation_audit(),
            },
        )
        return (csv_path, json_path)

    execute("pulsejet", True, pulsejet_stage)

    def ramjet_point_stage() -> Path:
        nonlocal ramjet_result
        ramjet_result = evaluate_ramjet(
            case.ramjet,
            case.selector,
            case.nozzle,
            case.fuel,
            case.mission.speed_run_altitude_msl_m,
            case.mission.peak_mach,
        )
        return _write_json(json_dir / "ramjet_peak_mach.json", ramjet_result)

    execute("ramjet_peak_mach", True, ramjet_point_stage)

    handoff_points: list[Any] = []

    def handoff_stage() -> Sequence[Path]:
        nonlocal handoff_points
        handoff_points = ramjet_handoff_sweep(
            case,
            case.mission.speed_run_altitude_msl_m,
            case.ramjet.minimum_lightoff_test_mach,
            case.mission.peak_mach,
            0.05,
        )
        return (
            _write_csv(csv_dir / "ramjet_handoff_sweep.csv", handoff_points),
            _write_json(json_dir / "ramjet_handoff_sweep.json", handoff_points),
        )

    execute("ramjet_handoff_sweep", True, handoff_stage)

    diameter_points: list[Any] = []

    def diameter_stage() -> Sequence[Path]:
        nonlocal diameter_points
        diameter_points = peak_mach_diameter_trade_sweep(
            case,
            case.vehicle.body_diameter_m,
            body_diameter_max_m,
            body_diameter_step_m,
        )
        return (
            _write_csv(csv_dir / "diameter_trade.csv", diameter_points),
            _write_json(json_dir / "diameter_trade.json", diameter_points),
        )

    execute("diameter_trade", True, diameter_stage)

    shared_nozzle_points: list[Any] = []

    def shared_nozzle_stage() -> Sequence[Path]:
        nonlocal shared_nozzle_points, selected_nozzle
        nominal_throat_m = case.nozzle.throat_diameter_m
        throat_values = tuple(
            sorted(
                {
                    round(max(0.001, nominal_throat_m + offset), 6)
                    for offset in (-0.020, -0.010, 0.0, 0.010, 0.020)
                }
            )
        )
        area_ratios = tuple(
            sorted(
                {
                    round(case.nozzle.exit_to_throat_area_ratio, 6),
                    1.10,
                    1.20,
                }
            )
        )
        body_values = tuple(
            sorted(
                {
                    round(case.vehicle.body_diameter_m, 6),
                    round(case.vehicle.body_diameter_m + 0.005, 6),
                }
            )
        )
        shared_nozzle_points = shared_nozzle_trade_sweep(
            case,
            body_diameters_m=body_values,
            throat_diameters_m=throat_values,
            exit_to_throat_area_ratios=area_ratios,
            propulsion_derate_fraction=propulsion_derate_fraction,
            pulsejet_warmup_s=case.simulation.pulsejet_steady_warmup_s,
            pulsejet_measurement_s=case.simulation.pulsejet_steady_measurement_s,
            pulsejet_time_step_s=case.simulation.time_step_s,
        )
        selected_nozzle = select_minimum_feasible_shared_nozzle(shared_nozzle_points)
        return (
            _write_csv(csv_dir / "shared_nozzle_trade.csv", shared_nozzle_points),
            _write_json(
                json_dir / "shared_nozzle_trade.json",
                {
                    "selection_rule": (
                        "smallest feasible body, then throat, then exit-to-throat area ratio"
                    ),
                    "selected": selected_nozzle,
                    "points": shared_nozzle_points,
                },
            ),
        )

    execute("shared_nozzle_trade", True, shared_nozzle_stage)

    def convergence_stage() -> Path:
        nonlocal convergence
        convergence = shared_nozzle_feasibility_bounds(
            case,
            propulsion_derate_fraction=propulsion_derate_fraction,
        )
        return _write_json(json_dir / "design_convergence.json", convergence)

    execute("design_convergence", True, convergence_stage)

    def robustness_stage() -> Sequence[Path]:
        nonlocal robustness_result
        robustness_result = run_robustness_trade(case, robustness_path)
        return (
            _write_csv(
                csv_dir / "robustness_trade.csv",
                robustness_result.points,
            ),
            _write_json(
                json_dir / "robustness_trade.json",
                robustness_result,
            ),
        )

    execute("robustness_trade", True, robustness_stage)

    fuel_points: list[Any] = []

    def fuel_stage() -> Sequence[Path]:
        nonlocal fuel_points
        fuel_points = fuel_performance_trade(
            case,
            load_fuels(resolved_fuels_path),
            propulsion_derate_fraction=propulsion_derate_fraction,
        )
        return (
            _write_csv(csv_dir / "fuel_trade.csv", fuel_points),
            _write_json(json_dir / "fuel_trade.json", fuel_points),
        )

    execute("fuel_trade", True, fuel_stage)

    altitude_points: list[Any] = []

    def altitude_stage() -> Sequence[Path]:
        nonlocal altitude_points
        altitude_points = peak_mach_altitude_trade_sweep(
            case,
            altitude_min_m,
            altitude_max_m,
            altitude_step_m,
            propulsion_derate_fraction=propulsion_derate_fraction,
        )
        return (
            _write_csv(csv_dir / "altitude_trade.csv", altitude_points),
            _write_json(json_dir / "altitude_trade.json", altitude_points),
        )

    execute("altitude_trade", True, altitude_stage)

    sensitivity_points: list[Any] = []

    def sensitivity_stage() -> Sequence[Path]:
        nonlocal sensitivity_points
        points = list(
            pulsejet_local_sensitivities(
                case,
                perturbation_fraction=0.10,
                warmup_s=case.simulation.pulsejet_steady_warmup_s,
                measurement_s=case.simulation.pulsejet_steady_measurement_s,
                time_step_s=case.simulation.time_step_s,
            )
        )
        points.extend(
            ramjet_local_sensitivities(case, perturbation_fraction=0.10)
        )
        sensitivity_points = rank_by_net_thrust_sensitivity(points)
        return (
            _write_csv(csv_dir / "key_variables.csv", sensitivity_points),
            _write_json(json_dir / "key_variables.json", sensitivity_points),
        )

    execute("key_variables", True, sensitivity_stage)

    trajectory_results: dict[str, Any] = {}

    def trajectory_stage() -> Sequence[Path]:
        nonlocal trajectory_results
        trajectory_results = {
            "nominal": simulate_mission(case, NOMINAL_SCENARIO),
            "adverse": simulate_mission(case, ADVERSE_SCENARIO),
        }
        return (
            _write_json(json_dir / "mission_trajectory.json", trajectory_results),
        )

    if run_trajectory:
        execute("mission_trajectory", True, trajectory_stage)
    else:
        stage_records.append(
            PipelineStageRecord(
                name="mission_trajectory",
                status="skipped",
                required=False,
                duration_s=0.0,
                output_files=(),
                message="disabled by run_trajectory=False",
            )
        )

    def jsbsim_stage() -> Sequence[Path]:
        outputs: list[Path] = []
        checks: dict[str, Any] = {}
        for scenario in (NOMINAL_SCENARIO, ADVERSE_SCENARIO):
            output_path, summary = write_jsbsim_aircraft(
                case, jsbsim_root / "aircraft", scenario
            )
            outputs.append(
                _write_json(
                    json_dir / f"jsbsim_{scenario.name}_aircraft_summary.json", summary
                )
            )
            if jsbsim_check:
                checks[scenario.name] = validate_with_jsbsim(
                    jsbsim_root,
                    output_path.parent.name,
                    altitude_m=case.mission.speed_run_altitude_msl_m,
                    mach=case.mission.peak_mach,
                    run_seconds=3.0,
                )
        if checks:
            outputs.append(_write_json(json_dir / "jsbsim_load_checks.json", checks))
        return outputs

    if run_jsbsim:
        execute("jsbsim_model", True, jsbsim_stage)
    else:
        stage_records.append(
            PipelineStageRecord(
                name="jsbsim_model",
                status="skipped",
                required=False,
                duration_s=0.0,
                output_files=(),
                message="disabled by run_jsbsim=False",
            )
        )

    def plots_stage() -> Sequence[Path]:
        plot_paths: list[Path] = []
        if pulsejet_samples:
            plot_paths.extend(_plot_pulsejet(pulsejet_samples, plot_dir))
        if handoff_points:
            plot_paths.extend(_plot_ramjet_handoff(handoff_points, plot_dir))
        if diameter_points:
            plot_paths.extend(_plot_diameter_trade(diameter_points, plot_dir))
        if shared_nozzle_points:
            plot_paths.extend(_plot_shared_nozzle(shared_nozzle_points, plot_dir))
        if altitude_points:
            plot_paths.extend(_plot_altitude_trade(altitude_points, plot_dir))
        if fuel_points:
            plot_paths.extend(_plot_fuel_trade(fuel_points, plot_dir))
        if sensitivity_points:
            plot_paths.extend(_plot_sensitivities(sensitivity_points, plot_dir))
        return plot_paths

    execute("plots", True, plots_stage)

    openvsp_succeeded = not run_openvsp
    if run_openvsp:

        def openvsp_stage() -> Sequence[Path]:
            summary = build_openvsp_geometry(case, openvsp_model_path)
            return (
                openvsp_model_path,
                _write_json(json_dir / "openvsp_geometry.json", summary),
            )

        openvsp_succeeded = execute("openvsp_geometry", True, openvsp_stage)
    else:
        stage_records.append(
            PipelineStageRecord(
                name="openvsp_geometry",
                status="skipped",
                required=False,
                duration_s=0.0,
                output_files=(),
                message="disabled by --skip-openvsp",
            )
        )

    if run_vspaero and openvsp_succeeded:

        def vspaero_stage() -> Sequence[Path]:
            summary = run_vspaero_sweep(case, openvsp_model_path)
            points = list(summary.points)
            csv_path = _write_csv(csv_dir / "vspaero_sweep.csv", points)
            json_path = _write_json(json_dir / "vspaero_sweep.json", summary)
            plot_paths = _plot_vspaero(points, plot_dir)
            return (csv_path, json_path, *plot_paths)

        execute("vspaero_sweep", True, vspaero_stage)
    elif run_vspaero:
        stage_records.append(
            PipelineStageRecord(
                name="vspaero_sweep",
                status="blocked",
                required=True,
                duration_s=0.0,
                output_files=(),
                message="OpenVSP geometry stage did not complete.",
            )
        )
    else:
        stage_records.append(
            PipelineStageRecord(
                name="vspaero_sweep",
                status="skipped",
                required=False,
                duration_s=0.0,
                output_files=(),
                message="disabled by --skip-vspaero",
            )
        )

    report_path = _write_summary_markdown(
        output_directory / "summary.md",
        case=case,
        stage_records=stage_records,
        pulsejet_summary=pulsejet_summary,
        ramjet_result=ramjet_result,
        convergence=convergence,
        selected_nozzle=selected_nozzle,
        robustness_result=robustness_result,
    )
    stage_records.append(
        PipelineStageRecord(
            name="summary_report",
            status="completed",
            required=True,
            duration_s=0.0,
            output_files=(str(report_path),),
        )
    )

    successful = all(
        record.status == "completed" for record in stage_records if record.required
    )
    summary = PipelineRunSummary(
        case_name=case.name,
        config_path=str(config_path),
        fuels_path=str(resolved_fuels_path),
        output_directory=str(output_directory),
        openvsp_model_path=str(openvsp_model_path),
        started_at_utc=started_at_utc,
        completed_at_utc=_utc_now(),
        successful=successful,
        stages=tuple(stage_records),
    )
    _write_json(output_directory / "manifest.json", summary)
    return summary
