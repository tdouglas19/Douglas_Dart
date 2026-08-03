"""Parameter-driven OpenVSP geometry for the external Douglas Dart mold line.

OpenVSP is an optional runtime dependency distributed with the OpenVSP application.
The functions in this module accept an API object explicitly so their call contract can
be tested without pretending that a unit test is a successful OpenVSP/VSPAERO run.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from math import cos, radians, sin, sqrt
from pathlib import Path
from types import ModuleType
from typing import Any

from .config import ReferenceCase, SurfacePlanformConfig


@dataclass(frozen=True)
class BodyStation:
    x_location_m: float
    diameter_m: float


@dataclass(frozen=True)
class AerodynamicReferenceQuantities:
    area_m2: float
    span_m: float
    mean_aerodynamic_chord_m: float


@dataclass(frozen=True)
class OpenVSPGeometrySummary:
    case_name: str
    openvsp_version: str
    output_path: str
    body_id: str
    lifting_surface_ids: tuple[str, ...]
    fin_ids: tuple[str, ...]
    body_stations: tuple[BodyStation, ...]
    lifting_reference_area_m2: float
    lifting_reference_span_m: float
    lifting_reference_mean_aerodynamic_chord_m: float
    flowthrough_open_end_configuration_requested: bool
    numerical_geometry_only: bool = True


_REQUIRED_FUNCTIONS = (
    "AddGeom",
    "ChangeXSecShape",
    "ClearVSPModel",
    "GetNumXSec",
    "GetVSPVersion",
    "GetXSec",
    "GetXSecParm",
    "GetXSecSurf",
    "SetDriverGroup",
    "SetGeomName",
    "SetParmVal",
    "Update",
    "WriteVSPFile",
)

_REQUIRED_CONSTANTS = (
    "ENGINE_GEOM_FLOWTHROUGH",
    "ENGINE_GEOM_INLET_OUTLET",
    "ENGINE_LOC_INDEX",
    "ENGINE_MODE_FLOWTHROUGH",
    "ROOTC_WSECT_DRIVER",
    "SET_ALL",
    "SPAN_WSECT_DRIVER",
    "SYM_NONE",
    "TIPC_WSECT_DRIVER",
    "XS_CIRCLE",
)


def load_openvsp_api(expected_version: str) -> ModuleType:
    """Import and validate the OpenVSP Python API supplied with OpenVSP."""

    try:
        vsp = import_module("openvsp")
    except ImportError as exc:
        raise RuntimeError(
            "OpenVSP's Python API is not importable. Install OpenVSP and use the "
            "Python package shipped with the matching OpenVSP release."
        ) from exc
    validate_openvsp_api(vsp, expected_version)
    return vsp


def validate_openvsp_api(vsp: Any, expected_version: str) -> str:
    """Reject namespace-only or version-mismatched ``openvsp`` installations."""

    missing = [
        name
        for name in (*_REQUIRED_FUNCTIONS, *_REQUIRED_CONSTANTS)
        if not hasattr(vsp, name)
    ]
    if missing:
        raise RuntimeError(
            "The imported 'openvsp' module is not a functional OpenVSP API; missing: "
            + ", ".join(sorted(missing))
        )
    actual_version = str(vsp.GetVSPVersion())
    if expected_version not in actual_version:
        raise RuntimeError(
            f"OpenVSP API version mismatch: expected {expected_version}, got "
            f"{actual_version}. Use the Python bindings shipped with that OpenVSP build."
        )
    return actual_version


def body_stations(case: ReferenceCase) -> tuple[BodyStation, ...]:
    """Return the five circular outer-mold-line stations used by OpenVSP."""

    geometry = case.geometry
    length_m = case.vehicle.body_length_m
    exit_diameter_m = case.nozzle.throat_diameter_m * sqrt(
        case.nozzle.exit_to_throat_area_ratio
    )
    constant_body_midpoint_m = 0.5 * (
        geometry.forebody_transition_length_m + geometry.aft_taper_start_m
    )
    return (
        BodyStation(0.0, case.selector.circular_intake_diameter_m),
        BodyStation(
            geometry.forebody_transition_length_m,
            case.vehicle.body_diameter_m,
        ),
        BodyStation(constant_body_midpoint_m, case.vehicle.body_diameter_m),
        BodyStation(geometry.aft_taper_start_m, case.vehicle.body_diameter_m),
        BodyStation(length_m, exit_diameter_m),
    )


def clocking_angles_deg(count: int, offset_deg: float) -> tuple[float, ...]:
    if count <= 0:
        return ()
    return tuple((offset_deg + index * 360.0 / count) % 360.0 for index in range(count))


def trapezoid_mean_aerodynamic_chord_m(planform: SurfacePlanformConfig) -> float:
    taper_ratio = planform.tip_chord_m / planform.root_chord_m
    return (
        2.0
        / 3.0
        * planform.root_chord_m
        * (1.0 + taper_ratio + taper_ratio**2)
        / (1.0 + taper_ratio)
    )


def vspaero_reference_quantities(
    case: ReferenceCase,
) -> AerodynamicReferenceQuantities:
    """Return total exposed area, tip-to-tip span, and trapezoid MAC."""

    planform = case.geometry.lifting_surface
    area_m2 = (
        case.geometry.lifting_surface_count * planform.exposed_area_per_surface_m2
    )
    span_m = (
        case.vehicle.body_diameter_m + 2.0 * planform.exposed_semispan_m
        if case.geometry.lifting_surface_count
        else case.vehicle.body_diameter_m
    )
    return AerodynamicReferenceQuantities(
        area_m2=area_m2,
        span_m=span_m,
        mean_aerodynamic_chord_m=trapezoid_mean_aerodynamic_chord_m(planform),
    )


def _set_xsec_parm(vsp: Any, xsec_id: str, name: str, value: float) -> None:
    parm_id = vsp.GetXSecParm(xsec_id, name)
    vsp.SetParmVal(parm_id, float(value))


def _check_openvsp_errors(vsp: Any) -> None:
    if not hasattr(vsp, "GetNumTotalErrors") or not hasattr(vsp, "PopLastError"):
        return
    errors: list[str] = []
    while vsp.GetNumTotalErrors() > 0:
        error = vsp.PopLastError()
        if hasattr(error, "GetErrorString"):
            errors.append(str(error.GetErrorString()))
        else:
            errors.append(str(error))
    if errors:
        raise RuntimeError("OpenVSP reported API errors: " + " | ".join(errors))


def _configure_flowthrough_body(
    vsp: Any,
    case: ReferenceCase,
) -> tuple[str, tuple[BodyStation, ...]]:
    stations = body_stations(case)
    body_id = str(vsp.AddGeom("FUSELAGE", ""))
    if not body_id:
        raise RuntimeError("OpenVSP failed to create the fuselage geometry")
    vsp.SetGeomName(body_id, "Douglas_Dart_Flowthrough_Body")
    vsp.SetParmVal(body_id, "Length", "Design", case.vehicle.body_length_m)
    vsp.SetParmVal(
        body_id,
        "Tess_W",
        "Shape",
        float(case.geometry.fuselage_tessellation),
    )

    xsec_surf_id = str(vsp.GetXSecSurf(body_id, 0))
    if vsp.GetNumXSec(xsec_surf_id) != len(stations):
        raise RuntimeError("The expected five default fuselage cross-sections are unavailable")

    for index, station in enumerate(stations):
        vsp.ChangeXSecShape(xsec_surf_id, index, vsp.XS_CIRCLE)
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        _set_xsec_parm(
            vsp,
            xsec_id,
            "XLocPercent",
            station.x_location_m / case.vehicle.body_length_m,
        )
        _set_xsec_parm(vsp, xsec_id, "Circle_Diameter", station.diameter_m)

    last_index = len(stations) - 1
    engine_values = {
        "GeomIOType": vsp.ENGINE_GEOM_INLET_OUTLET,
        "GeomInType": vsp.ENGINE_GEOM_FLOWTHROUGH,
        "InletFaceMode": vsp.ENGINE_LOC_INDEX,
        "InletLipMode": vsp.ENGINE_LOC_INDEX,
        "OutletFaceMode": vsp.ENGINE_LOC_INDEX,
        "OutletLipMode": vsp.ENGINE_LOC_INDEX,
        "InletFaceIndex": 0,
        "InletLipIndex": 0,
        "OutletFaceIndex": last_index,
        "OutletLipIndex": last_index,
        "InletModeType": vsp.ENGINE_MODE_FLOWTHROUGH,
    }
    for name, value in engine_values.items():
        vsp.SetParmVal(body_id, name, "EngineModel", float(value))
    return body_id, stations


def _configure_radial_surface(
    vsp: Any,
    case: ReferenceCase,
    planform: SurfacePlanformConfig,
    name: str,
    clocking_angle_deg: float,
) -> str:
    surface_id = str(vsp.AddGeom("WING", ""))
    if not surface_id:
        raise RuntimeError(f"OpenVSP failed to create {name}")
    vsp.SetGeomName(surface_id, name)
    vsp.SetParmVal(surface_id, "Sym_Planar_Flag", "Sym", float(vsp.SYM_NONE))

    clocking_angle_rad = radians(clocking_angle_deg)
    body_radius_m = 0.5 * case.vehicle.body_diameter_m
    vsp.SetParmVal(surface_id, "X_Rel_Location", "XForm", planform.x_location_m)
    vsp.SetParmVal(
        surface_id,
        "Y_Rel_Location",
        "XForm",
        body_radius_m * cos(clocking_angle_rad),
    )
    vsp.SetParmVal(
        surface_id,
        "Z_Rel_Location",
        "XForm",
        body_radius_m * sin(clocking_angle_rad),
    )
    vsp.SetParmVal(surface_id, "X_Rel_Rotation", "XForm", clocking_angle_deg)

    vsp.SetDriverGroup(
        surface_id,
        1,
        vsp.SPAN_WSECT_DRIVER,
        vsp.ROOTC_WSECT_DRIVER,
        vsp.TIPC_WSECT_DRIVER,
    )
    vsp.SetParmVal(surface_id, "Span", "XSec_1", planform.exposed_semispan_m)
    vsp.SetParmVal(surface_id, "Root_Chord", "XSec_1", planform.root_chord_m)
    vsp.SetParmVal(surface_id, "Tip_Chord", "XSec_1", planform.tip_chord_m)
    vsp.SetParmVal(surface_id, "Sweep", "XSec_1", planform.sweep_deg)
    vsp.SetParmVal(surface_id, "Sweep_Location", "XSec_1", 0.25)
    for xsec_curve_index in (0, 1):
        vsp.SetParmVal(
            surface_id,
            "ThickChord",
            f"XSecCurve_{xsec_curve_index}",
            planform.thickness_to_chord,
        )
    vsp.SetParmVal(
        surface_id,
        "SectTess_U",
        "XSec_1",
        float(case.geometry.surface_tessellation),
    )
    vsp.SetParmVal(
        surface_id,
        "Tess_W",
        "Shape",
        float(2 * case.geometry.surface_tessellation + 1),
    )
    return surface_id


def build_openvsp_geometry(
    case: ReferenceCase,
    output_path: str | Path,
    *,
    vsp: Any | None = None,
) -> OpenVSPGeometrySummary:
    """Build and save the configured external geometry through the OpenVSP API."""

    vsp = load_openvsp_api(case.geometry.api_version) if vsp is None else vsp
    actual_version = validate_openvsp_api(vsp, case.geometry.api_version)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if hasattr(vsp, "VSPRenew"):
        vsp.VSPRenew()
    vsp.ClearVSPModel()
    body_id, stations = _configure_flowthrough_body(vsp, case)

    lifting_ids = tuple(
        _configure_radial_surface(
            vsp,
            case,
            case.geometry.lifting_surface,
            f"Lifting_Surface_{index + 1}",
            angle_deg,
        )
        for index, angle_deg in enumerate(
            clocking_angles_deg(
                case.geometry.lifting_surface_count,
                case.geometry.lifting_surface_clocking_offset_deg,
            )
        )
    )
    fin_ids = tuple(
        _configure_radial_surface(
            vsp,
            case,
            case.geometry.fin,
            f"Fin_{index + 1}",
            angle_deg,
        )
        for index, angle_deg in enumerate(
            clocking_angles_deg(
                case.geometry.fin_count,
                case.geometry.fin_clocking_offset_deg,
            )
        )
    )
    vsp.Update()
    _check_openvsp_errors(vsp)
    vsp.WriteVSPFile(str(output_path), vsp.SET_ALL)
    _check_openvsp_errors(vsp)
    if not output_path.is_file():
        raise RuntimeError(
            f"OpenVSP reported no error but did not write the model: {output_path}"
        )

    references = vspaero_reference_quantities(case)
    return OpenVSPGeometrySummary(
        case_name=case.name,
        openvsp_version=actual_version,
        output_path=str(output_path),
        body_id=body_id,
        lifting_surface_ids=lifting_ids,
        fin_ids=fin_ids,
        body_stations=stations,
        lifting_reference_area_m2=references.area_m2,
        lifting_reference_span_m=references.span_m,
        lifting_reference_mean_aerodynamic_chord_m=(
            references.mean_aerodynamic_chord_m
        ),
        flowthrough_open_end_configuration_requested=True,
    )
