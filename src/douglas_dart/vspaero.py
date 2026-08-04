"""Headless VSPAERO point sweeps for a generated OpenVSP vehicle.

The configured Mach values are intentionally executed as individual points. OpenVSP's
``VSPAEROSweep`` accepts start/end/count inputs, which would otherwise interpolate the
nonuniform Mach list and silently analyze different conditions than requested.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import ReferenceCase
from .openvsp_geometry import (
    _check_openvsp_errors,
    load_openvsp_api,
    validate_openvsp_api,
    vspaero_reference_quantities,
)


@dataclass(frozen=True)
class VSPAeroPoint:
    mach: float
    alpha_deg: float
    beta_deg: float
    lift_coefficient: float
    drag_coefficient_inviscid: float
    side_force_coefficient: float
    rolling_moment_coefficient: float
    pitching_moment_coefficient: float
    yawing_moment_coefficient: float
    sweep_results_id: str
    history_results_id: str
    status: tuple[str, ...]
    external_aerodynamics_only: bool = True


@dataclass(frozen=True)
class VSPAeroSweepSummary:
    case_name: str
    openvsp_version: str
    analysis_method: str
    model_path: str
    compute_geometry_results_id: str
    reference_area_m2: float
    reference_span_m: float
    reference_chord_m: float
    reference_cg_x_m: float
    points: tuple[VSPAeroPoint, ...]
    live_solver_run: bool = True


_VSPAERO_FUNCTIONS = (
    "ExecAnalysis",
    "GetAllDataNames",
    "GetDoubleResults",
    "GetStringResults",
    "ReadVSPFile",
    "SetAnalysisInputDefaults",
    "SetDoubleAnalysisInput",
    "SetIntAnalysisInput",
)

_VSPAERO_CONSTANTS = (
    "MANUAL_REF",
    "PANEL",
    "SET_ALL",
    "SET_NONE",
    "VORTEX_LATTICE",
)


def validate_vspaero_api(vsp: Any, expected_version: str) -> str:
    version = validate_openvsp_api(vsp, expected_version)
    missing = [
        name
        for name in (*_VSPAERO_FUNCTIONS, *_VSPAERO_CONSTANTS)
        if not hasattr(vsp, name)
    ]
    if missing:
        raise RuntimeError(
            "The OpenVSP API does not expose the required VSPAERO interface; missing: "
            + ", ".join(sorted(missing))
        )
    return version


def _set_geometry_sets(vsp: Any, analysis: str, method: str) -> None:
    if method == "panel":
        thick_set = vsp.SET_ALL
        thin_set = vsp.SET_NONE
    elif method == "vortex_lattice":
        thick_set = vsp.SET_NONE
        thin_set = vsp.SET_ALL
    else:
        raise ValueError(f"unsupported VSPAERO method: {method}")
    vsp.SetIntAnalysisInput(analysis, "GeomSet", [thick_set], 0)
    vsp.SetIntAnalysisInput(analysis, "ThinGeomSet", [thin_set], 0)


def _set_sweep_analysis_method(vsp: Any, analysis: str, method: str) -> None:
    if method == "panel":
        analysis_method = vsp.PANEL
    elif method == "vortex_lattice":
        analysis_method = vsp.VORTEX_LATTICE
    else:
        raise ValueError(f"unsupported VSPAERO method: {method}")
    vsp.SetIntAnalysisInput(analysis, "AnalysisMethod", [analysis_method], 0)


def _set_single_point_inputs(
    vsp: Any,
    analysis: str,
    case: ReferenceCase,
    mach: float,
    alpha_deg: float,
    beta_deg: float,
) -> None:
    references = vspaero_reference_quantities(case)
    vsp.SetIntAnalysisInput(analysis, "RefFlag", [vsp.MANUAL_REF], 0)
    vsp.SetDoubleAnalysisInput(analysis, "Sref", [references.area_m2], 0)
    vsp.SetDoubleAnalysisInput(analysis, "bref", [references.span_m], 0)
    vsp.SetDoubleAnalysisInput(
        analysis,
        "cref",
        [references.mean_aerodynamic_chord_m],
        0,
    )
    vsp.SetDoubleAnalysisInput(
        analysis,
        "Xcg",
        [case.geometry.reference_cg_x_m],
        0,
    )
    vsp.SetDoubleAnalysisInput(analysis, "Ycg", [0.0], 0)
    vsp.SetDoubleAnalysisInput(analysis, "Zcg", [0.0], 0)
    # VSPAERO's Symmetry input is Boolean; sideslip points require the full model.
    vsp.SetIntAnalysisInput(analysis, "Symmetry", [False], 0)
    vsp.SetIntAnalysisInput(
        analysis,
        "WakeNumIter",
        [case.geometry.wake_iterations],
        0,
    )

    for prefix, value in (
        ("Alpha", alpha_deg),
        ("Beta", beta_deg),
        ("Mach", mach),
    ):
        vsp.SetDoubleAnalysisInput(analysis, f"{prefix}Start", [value], 0)
        vsp.SetDoubleAnalysisInput(analysis, f"{prefix}End", [value], 0)
        vsp.SetIntAnalysisInput(analysis, f"{prefix}Npts", [1], 0)


def _final_result_value(vsp: Any, history_id: str, name: str) -> float:
    available_names = set(str(value) for value in vsp.GetAllDataNames(history_id))
    if name not in available_names:
        raise RuntimeError(
            f"VSPAERO history result {history_id!r} does not contain {name!r}"
        )
    values = list(vsp.GetDoubleResults(history_id, name, 0))
    if not values:
        raise RuntimeError(f"VSPAERO history result {name!r} is empty")
    return float(values[-1])


def run_vspaero_sweep(
    case: ReferenceCase,
    model_path: str | Path,
    *,
    vsp: Any | None = None,
) -> VSPAeroSweepSummary:
    """Run every explicitly configured Mach/alpha/beta combination."""

    vsp = load_openvsp_api(case.geometry.api_version) if vsp is None else vsp
    version = validate_vspaero_api(vsp, case.geometry.api_version)
    model_path = Path(model_path)
    if not model_path.exists():
        # Require a generated model so an empty analysis cannot look successful.
        raise FileNotFoundError(f"OpenVSP model does not exist: {model_path}")

    if hasattr(vsp, "VSPRenew"):
        vsp.VSPRenew()
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(str(model_path))
    vsp.Update()
    _check_openvsp_errors(vsp)

    compute_analysis = "VSPAEROComputeGeometry"
    vsp.SetAnalysisInputDefaults(compute_analysis)
    # OpenVSP 3.51.2 exposes geometry sets, but not AnalysisMethod, on the
    # VSPAEROComputeGeometry analysis. AnalysisMethod belongs to VSPAEROSweep.
    _set_geometry_sets(
        vsp,
        compute_analysis,
        case.geometry.analysis_method,
    )
    compute_geometry_results_id = str(vsp.ExecAnalysis(compute_analysis))
    if not compute_geometry_results_id:
        raise RuntimeError("VSPAEROComputeGeometry returned no results ID")
    _check_openvsp_errors(vsp)

    sweep_analysis = "VSPAEROSweep"
    points: list[VSPAeroPoint] = []
    for mach in case.geometry.mach_values:
        for beta_deg in case.geometry.beta_deg_values:
            for alpha_deg in case.geometry.alpha_deg_values:
                vsp.SetAnalysisInputDefaults(sweep_analysis)
                _set_geometry_sets(
                    vsp,
                    sweep_analysis,
                    case.geometry.analysis_method,
                )
                _set_sweep_analysis_method(
                    vsp,
                    sweep_analysis,
                    case.geometry.analysis_method,
                )
                _set_single_point_inputs(
                    vsp,
                    sweep_analysis,
                    case,
                    mach,
                    alpha_deg,
                    beta_deg,
                )
                vsp.Update()
                sweep_id = str(vsp.ExecAnalysis(sweep_analysis))
                if not sweep_id:
                    raise RuntimeError("VSPAEROSweep returned no results ID")
                result_ids = tuple(
                    str(value) for value in vsp.GetStringResults(sweep_id, "ResultsVec", 0)
                )
                if len(result_ids) != 1:
                    raise RuntimeError(
                        "A configured single-point VSPAERO run did not return exactly "
                        f"one history result (got {len(result_ids)})"
                    )
                history_id = result_ids[0]
                points.append(
                    VSPAeroPoint(
                        mach=mach,
                        alpha_deg=alpha_deg,
                        beta_deg=beta_deg,
                        lift_coefficient=_final_result_value(
                            vsp, history_id, "CLtot"
                        ),
                        drag_coefficient_inviscid=_final_result_value(
                            vsp, history_id, "CDtot"
                        ),
                        side_force_coefficient=_final_result_value(
                            vsp, history_id, "CStot"
                        ),
                        rolling_moment_coefficient=_final_result_value(
                            vsp, history_id, "CMxtot"
                        ),
                        pitching_moment_coefficient=_final_result_value(
                            vsp, history_id, "CMytot"
                        ),
                        yawing_moment_coefficient=_final_result_value(
                            vsp, history_id, "CMztot"
                        ),
                        sweep_results_id=sweep_id,
                        history_results_id=history_id,
                        status=(
                            "inviscid_external_aerodynamics_only",
                            "augment_drag_with_viscous_wave_and_base_drag_models",
                        ),
                    )
                )
                _check_openvsp_errors(vsp)

    references = vspaero_reference_quantities(case)
    return VSPAeroSweepSummary(
        case_name=case.name,
        openvsp_version=version,
        analysis_method=case.geometry.analysis_method,
        model_path=str(model_path),
        compute_geometry_results_id=compute_geometry_results_id,
        reference_area_m2=references.area_m2,
        reference_span_m=references.span_m,
        reference_chord_m=references.mean_aerodynamic_chord_m,
        reference_cg_x_m=case.geometry.reference_cg_x_m,
        points=tuple(points),
    )
