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

from .config import ExternalShellConfig, RamInletConfig, ReferenceCase, SurfacePlanformConfig


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
    shell_id: str
    ram_inlet_id: str
    lifting_surface_ids: tuple[str, ...]
    fin_ids: tuple[str, ...]
    body_stations: tuple[BodyStation, ...]
    shell_stations: tuple[BodyStation, ...]
    shell_outer_diameter_m: float
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


def body_diameter_at_x_m(case: ReferenceCase, x_location_m: float) -> float:
    """Interpolate the inner flow-through body's outer mold line at any station.

    ``body_stations`` defines the body as a piecewise-linear profile (forebody
    taper-in, constant section, aft taper-out to the nozzle exit). Any geometry
    that fairs into the body -- the fin-can shell in particular -- must fair to
    this local diameter, not the constant mid-body diameter, once it extends into
    the forebody or aft taper regions.
    """

    stations = body_stations(case)
    clamped_x = min(max(x_location_m, stations[0].x_location_m), stations[-1].x_location_m)
    for lower, upper in zip(stations, stations[1:]):
        if lower.x_location_m <= clamped_x <= upper.x_location_m:
            span = upper.x_location_m - lower.x_location_m
            if span <= 0.0:
                return lower.diameter_m
            fraction = (clamped_x - lower.x_location_m) / span
            return lower.diameter_m + fraction * (upper.diameter_m - lower.diameter_m)
    return stations[-1].diameter_m


def shell_stations(case: ReferenceCase) -> tuple[BodyStation, ...]:
    """Return the five stations of the annular fin-can shroud outer mold line.

    The shroud fairs from the *local* inner flow-through body diameter (not the
    constant mid-body diameter -- the shell may extend into the aft taper to keep a
    fin/wing root chord entirely on its constant-diameter span, see
    ``ReferenceCase.__post_init__``) up to the shell outer diameter (inner body
    diameter plus twice the configured radial annulus offset) over the forward
    taper, holds that diameter, and fairs back down aft. It is a second,
    independent FUSELAGE surface, not a blend of the inner body.
    """

    shell = case.geometry.shell
    outer_diameter_m = case.vehicle.body_diameter_m + 2.0 * shell.radial_offset_m
    midpoint_m = 0.5 * (
        (shell.start_x_m + shell.forward_taper_length_m)
        + (shell.end_x_m - shell.aft_taper_length_m)
    )
    return (
        BodyStation(shell.start_x_m, body_diameter_at_x_m(case, shell.start_x_m)),
        BodyStation(shell.start_x_m + shell.forward_taper_length_m, outer_diameter_m),
        BodyStation(midpoint_m, outer_diameter_m),
        BodyStation(shell.end_x_m - shell.aft_taper_length_m, outer_diameter_m),
        BodyStation(shell.end_x_m, body_diameter_at_x_m(case, shell.end_x_m)),
    )


def shell_outer_diameter_m(case: ReferenceCase) -> float:
    return case.vehicle.body_diameter_m + 2.0 * case.geometry.shell.radial_offset_m


def mount_radius_m(case: ReferenceCase, x_location_m: float) -> float:
    """Return the fin-can mount radius for a radial surface at one axial station.

    Surfaces that fall within the external shell's constant-diameter span mount to
    the shell outer mold line (the "fin can"); surfaces outside that span fall back
    to the inner flow-through body radius.
    """

    shell = case.geometry.shell
    if shell.start_x_m <= x_location_m <= shell.end_x_m:
        return 0.5 * shell_outer_diameter_m(case)
    return 0.5 * case.vehicle.body_diameter_m


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


def _set_xsec_parm_if_present(vsp: Any, xsec_id: str, name: str, value: float) -> None:
    """Set an optional cross-section parameter, tolerating a missing skinning parm.

    Real OpenVSP returns an empty parm ID for a name that does not exist on the
    current cross-section shape; the required-function fake used in tests always
    returns a synthetic (truthy) ID, so this stays exercised there too.
    """

    parm_id = vsp.GetXSecParm(xsec_id, name)
    if not parm_id:
        return
    vsp.SetParmVal(parm_id, float(value))


def _flatten_xsec_skinning(vsp: Any, xsec_surf_id: str, station_count: int) -> None:
    """Zero the top/right skinning skew and strength on every circular cross-section.

    OpenVSP fuselage cross-sections carry per-side ("Top"/"Right"/"Bottom"/"Left")
    skinning continuity angle and strength parameters. Left at nonzero or asymmetric
    defaults they can skew an otherwise circular mold line. This keeps every station
    a plain, symmetric surface of revolution so the body stays flowing and
    aerodynamic rather than picking up an unintended top/side kink.
    """

    for index in range(station_count):
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        _set_xsec_parm_if_present(vsp, xsec_id, "AllSym", 1.0)
        for side in ("Top", "Bottom", "Left", "Right"):
            _set_xsec_parm_if_present(vsp, xsec_id, f"{side}LAngle", 0.0)
            _set_xsec_parm_if_present(vsp, xsec_id, f"{side}LStrength", 0.0)
            _set_xsec_parm_if_present(vsp, xsec_id, f"{side}RAngle", 0.0)
            _set_xsec_parm_if_present(vsp, xsec_id, f"{side}RStrength", 0.0)


def _normalize_rotation_deg(clocking_angle_deg: float) -> float:
    """Fold a [0, 360) clocking angle into OpenVSP's [-180, 180] XRot range.

    ``X_Rel_Rotation`` only accepts -180 to 180 degrees, so a 225 degree fin station
    must be commanded as -135 and a 315 degree station as -45.
    """

    normalized = ((clocking_angle_deg + 180.0) % 360.0) - 180.0
    if normalized <= -180.0:
        normalized += 360.0
    return normalized


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
    _flatten_xsec_skinning(vsp, xsec_surf_id, len(stations))

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


def _radial_mount_overlap_m(planform: SurfacePlanformConfig) -> float:
    """Root-mount radial interpenetration, sized from the surface's own root airfoil.

    A root mounted exactly tangent to the body/shell surface (zero overlap) produces
    a degenerate sliver panel face at the tangent line during VSPAERO's real
    triangulation ("PGFace Invalid in Triangulate_DBA"), confirmed against the real
    installed API. Half the root airfoil's own maximum thickness is a natural,
    geometry-scaled interpenetration depth -- enough to guarantee a genuine
    intersection for this specific surface, rather than one fixed absolute value
    that could be too shallow for a thick/large-chord surface or unnecessarily deep
    for a thin one.
    """

    return 0.5 * planform.thickness_to_chord * planform.root_chord_m


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
    # A root mounted exactly tangent to the body/shell surface (zero radial overlap)
    # produces a degenerate sliver panel face at the tangent line during VSPAERO's
    # real triangulation ("PGFace Invalid in Triangulate_DBA"), confirmed against the
    # real installed API. A small commanded interpenetration forces a genuine
    # intersection instead of exact tangency.
    mount_radius = mount_radius_m(case, planform.x_location_m) - _radial_mount_overlap_m(
        planform
    )
    vsp.SetParmVal(surface_id, "X_Rel_Location", "XForm", planform.x_location_m)
    vsp.SetParmVal(
        surface_id,
        "Y_Rel_Location",
        "XForm",
        mount_radius * cos(clocking_angle_rad),
    )
    vsp.SetParmVal(
        surface_id,
        "Z_Rel_Location",
        "XForm",
        mount_radius * sin(clocking_angle_rad),
    )
    vsp.SetParmVal(
        surface_id,
        "X_Rel_Rotation",
        "XForm",
        _normalize_rotation_deg(clocking_angle_deg),
    )

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


def _configure_external_shell(
    vsp: Any,
    case: ReferenceCase,
) -> tuple[str, tuple[BodyStation, ...]]:
    """Build the annular fin-can shroud as a second, independent FUSELAGE surface.

    Houses the fuel tank, avionics, and auxiliary systems in the annulus between this
    outer mold line and the inner engine flow path; fins and lifting surfaces mount
    to it (see :func:`mount_radius_m`).
    """

    stations = shell_stations(case)
    shell_id = str(vsp.AddGeom("FUSELAGE", ""))
    if not shell_id:
        raise RuntimeError("OpenVSP failed to create the external shell geometry")
    vsp.SetGeomName(shell_id, "Douglas_Dart_External_Shell_FinCan")
    vsp.SetParmVal(shell_id, "Length", "Design", case.vehicle.body_length_m)
    vsp.SetParmVal(
        shell_id,
        "Tess_W",
        "Shape",
        float(case.geometry.shell.tessellation),
    )

    xsec_surf_id = str(vsp.GetXSecSurf(shell_id, 0))
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
    _flatten_xsec_skinning(vsp, xsec_surf_id, len(stations))
    return shell_id, stations


def _configure_ram_inlet(vsp: Any, case: ReferenceCase) -> str:
    """Build the ram-air inlet as a centered, flow-through lip duct ahead of the nose.

    The primary body's nose station is already an open flow-through face, but a
    single circular opening does not represent the mutually exclusive
    pulsejet/ramjet selector's real inlet housing. This duct is a short,
    axisymmetric extension mounted on the vehicle centerline immediately ahead of
    the main body (spanning ``x in [-length_m, 0]``), fairing from a slightly larger
    external lip diameter down to exactly the main body's nose-opening diameter, and
    is explicitly flagged with its own OpenVSP inlet/outlet flow-through engine
    parameters -- the same pattern used for the main body -- rather than being a
    solid, off-axis decorative bump. ``length_m`` and the lip diameter are both
    derived from the actual intake diameter via ``RamInletConfig``'s cowl ratios,
    not set as independent absolute dimensions.
    """

    ram_inlet = case.geometry.ram_inlet
    nose_diameter_m = case.selector.circular_intake_diameter_m
    length_m = ram_inlet.length_m(nose_diameter_m)
    lip_diameter_m = ram_inlet.lip_diameter_m(nose_diameter_m)

    duct_id = str(vsp.AddGeom("FUSELAGE", ""))
    if not duct_id:
        raise RuntimeError("OpenVSP failed to create the ram-air inlet geometry")
    vsp.SetGeomName(duct_id, "Douglas_Dart_Ram_Air_Inlet_Duct")
    vsp.SetParmVal(duct_id, "Length", "Design", length_m)
    vsp.SetParmVal(duct_id, "Sym_Planar_Flag", "Sym", 0.0)

    # Centered on the vehicle axis and mounted immediately ahead of the main body's
    # nose station (x=0), not offset radially or clocked to one side.
    vsp.SetParmVal(duct_id, "X_Rel_Location", "XForm", -length_m)
    vsp.SetParmVal(duct_id, "Y_Rel_Location", "XForm", 0.0)
    vsp.SetParmVal(duct_id, "Z_Rel_Location", "XForm", 0.0)
    vsp.SetParmVal(duct_id, "X_Rel_Rotation", "XForm", 0.0)

    xsec_surf_id = str(vsp.GetXSecSurf(duct_id, 0))
    station_count = vsp.GetNumXSec(xsec_surf_id)
    last_index = station_count - 1
    for index in range(station_count):
        vsp.ChangeXSecShape(xsec_surf_id, index, vsp.XS_CIRCLE)
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        fraction = index / max(last_index, 1)
        # Fair linearly from the external lip diameter at the forward face down to
        # exactly the main body's nose-opening diameter at the aft face, so the duct
        # blends into the body with no step.
        diameter_m = lip_diameter_m + fraction * (nose_diameter_m - lip_diameter_m)
        _set_xsec_parm(vsp, xsec_id, "XLocPercent", fraction)
        _set_xsec_parm(vsp, xsec_id, "Circle_Diameter", diameter_m)
    _flatten_xsec_skinning(vsp, xsec_surf_id, station_count)

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
        vsp.SetParmVal(duct_id, name, "EngineModel", float(value))
    return duct_id


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
    shell_id, shell_station_tuple = _configure_external_shell(vsp, case)
    ram_inlet_id = _configure_ram_inlet(vsp, case)

    lifting_ids = tuple(
        _configure_radial_surface(
            vsp,
            case,
            case.geometry.lifting_surface,
            # Not "Lifting_Surface_N": a real installed OpenVSP 3.51.2 Windows build
            # crashes VSPAEROComputeGeometry's native panel mesher whenever a geom
            # name ends in the literal substring "_Surface" (reproduced
            # deterministically; "_Surface_X" and every other tested variant that
            # does not end the string in "_Surface" is unaffected). Confirmed via
            # bisection against the real API, not the unit-test fake.
            f"Lifting_Panel_{index + 1}",
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
        shell_id=shell_id,
        ram_inlet_id=ram_inlet_id,
        lifting_surface_ids=lifting_ids,
        fin_ids=fin_ids,
        body_stations=stations,
        shell_stations=shell_station_tuple,
        shell_outer_diameter_m=shell_outer_diameter_m(case),
        lifting_reference_area_m2=references.area_m2,
        lifting_reference_span_m=references.span_m,
        lifting_reference_mean_aerodynamic_chord_m=(
            references.mean_aerodynamic_chord_m
        ),
        flowthrough_open_end_configuration_requested=True,
    )
