"""Turn a :class:`VehicleGeometry` into a real ``.vsp3`` through the OpenVSP API.

Nothing is decided here -- :mod:`vehicle_geometry` has already resolved every
dimension. This module only translates.

Several non-obvious workarounds below are load-bearing and were established
against this same installed 3.51.2 Windows build (docs/openvsp_real_api_findings.md):

* No geom name may END in the literal ``_Surface``: it kills the native panel
  mesher, taking the whole Python process down with no exception.
* Surface roots need a commanded radial interpenetration. Exact tangency makes
  a degenerate sliver face during triangulation.
* Fuselage cross-sections carry per-side skinning angle/strength parameters that
  default nonzero; left alone they skew an otherwise circular mold line.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
from typing import Any

from openvsp_env import check_errors, clear_errors, load_openvsp
from vehicle_geometry import BodyStation, SurfacePanel, VehicleGeometry

BODY_GEOM_NAME = "Douglas_Dart_V4_Body"

# VSPAERO splits a configuration into THICK geometry (solved with surface
# panels) and THIN geometry (solved as zero-thickness lifting sheets). The
# assignment is not cosmetic: a fuselage placed in the thin set is solved as a
# lifting sheet and returns garbage -- measured here, the bare body at alpha = 4
# deg produced CL = 0.0000 and a spurious CS = -0.0567, i.e. its entire normal
# force came out on the wrong axis. These two user sets carry the split, and are
# written into the .vsp3 so the GUI shows the same grouping.
THICK_SET_NAME = "VSPAERO_Thick_Bodies"
THIN_SET_NAME = "VSPAERO_Thin_Surfaces"
_THICK_SET_INDEX = 3  # vsp.SET_FIRST_USER
_THIN_SET_INDEX = 4


@dataclass(frozen=True)
class BuildResult:
    model_path: str
    openvsp_version: str
    body_id: str
    wing_ids: tuple[str, ...]
    fin_ids: tuple[str, ...]
    body_station_count: int


def _normalise_rotation_deg(angle_deg: float) -> float:
    """Fold a [0, 360) clocking angle into OpenVSP's [-180, 180] XRot range.

    ``X_Rel_Rotation`` rejects anything outside that band, so a 225 deg fin has
    to be commanded as -135 and a 315 deg fin as -45.
    """

    folded = ((angle_deg + 180.0) % 360.0) - 180.0
    return folded + 360.0 if folded <= -180.0 else folded


def _set_xsec_parm(
    vsp: Any, xsec_id: str, name: str, value: float, *, required: bool = True
) -> None:
    """Set a cross-section parameter.

    OpenVSP returns an empty parm ID for a name the current cross-section shape
    does not have, rather than raising. ``required=False`` is for parameters that
    genuinely vary by shape (the per-side skinning controls); everything else
    must exist, or a silent typo becomes a silently wrong mold line.
    """

    parm_id = vsp.GetXSecParm(xsec_id, name)
    if not parm_id:
        if required:
            raise RuntimeError(
                f"cross-section {xsec_id} has no parameter {name!r}; the shape is "
                "not what this code assumes"
            )
        return
    vsp.SetParmVal(parm_id, float(value))


def _flatten_skinning(vsp: Any, xsec_surf_id: str, station_count: int) -> None:
    """Zero the per-side skinning skew so each station stays a true circle."""

    for index in range(station_count):
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        _set_xsec_parm(vsp, xsec_id, "AllSym", 1.0, required=False)
        for side in ("Top", "Bottom", "Left", "Right"):
            for suffix in ("LAngle", "LStrength", "RAngle", "RStrength", "LSlew", "RSlew"):
                _set_xsec_parm(vsp, xsec_id, f"{side}{suffix}", 0.0, required=False)


def _build_body(vsp: Any, geometry: VehicleGeometry) -> str:
    """The flow-through duct: open at the nose lip, open at the aft face.

    A single FUSELAGE whose outer mold line IS the external shape. Marking it
    flow-through means OpenVSP opens both end faces instead of capping them, so
    the nose is a real inlet rather than a flat 116 mm disc -- which in a panel
    method would otherwise read as a large blunt base.

    LIMITATION, stated because it is invisible in the output: the internal duct
    OpenVSP creates is the interior of this mold line, which balloons to the full
    214 mm body. The real flowpath necks to a 115.6 mm tailpipe. Internal duct
    losses and true capture are therefore NOT represented -- the trajectory model
    accounts for spillage and additive drag separately, and this solution is
    inviscid external aerodynamics only.
    """

    stations = geometry.body
    body_length = geometry.sizing.body_length_m

    body_id = str(vsp.AddGeom("FUSELAGE", ""))
    if not body_id:
        raise RuntimeError("OpenVSP failed to create the body fuselage")
    vsp.SetGeomName(body_id, BODY_GEOM_NAME)
    vsp.SetParmVal(body_id, "Length", "Design", body_length)
    vsp.SetParmVal(
        body_id, "Tess_W", "Shape", float(geometry.inputs.fuselage_tessellation)
    )

    xsec_surf_id = str(vsp.GetXSecSurf(body_id, 0))
    # A default fuselage has 5 cross-sections; add until it has as many as the
    # derived mold line needs. InsertXSec appends after the given index.
    while vsp.GetNumXSec(xsec_surf_id) < len(stations):
        vsp.InsertXSec(body_id, vsp.GetNumXSec(xsec_surf_id) - 2, vsp.XS_CIRCLE)
    while vsp.GetNumXSec(xsec_surf_id) > len(stations):
        vsp.CutXSec(body_id, vsp.GetNumXSec(xsec_surf_id) - 2)
    if vsp.GetNumXSec(xsec_surf_id) != len(stations):
        raise RuntimeError(
            f"could not set the fuselage to {len(stations)} cross-sections "
            f"(got {vsp.GetNumXSec(xsec_surf_id)})"
        )

    _write_stations(vsp, xsec_surf_id, stations, body_length)
    _flatten_skinning(vsp, xsec_surf_id, len(stations))
    _mark_flowthrough(vsp, body_id, len(stations) - 1)
    return body_id


def _write_stations(
    vsp: Any,
    xsec_surf_id: str,
    stations: tuple[BodyStation, ...],
    body_length_m: float,
) -> None:
    for index, station in enumerate(stations):
        vsp.ChangeXSecShape(xsec_surf_id, index, vsp.XS_CIRCLE)
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        # XLocPercent is a fraction of the geom's Length, not an absolute station.
        _set_xsec_parm(
            vsp, xsec_id, "XLocPercent", station.x_m / body_length_m
        )
        _set_xsec_parm(vsp, xsec_id, "Circle_Diameter", station.diameter_m)


def _mark_flowthrough(vsp: Any, body_id: str, last_index: int) -> None:
    """Flag the geom as an inlet/outlet flow-through duct."""

    for name, value in (
        ("GeomIOType", vsp.ENGINE_GEOM_INLET_OUTLET),
        ("GeomInType", vsp.ENGINE_GEOM_FLOWTHROUGH),
        ("InletFaceMode", vsp.ENGINE_LOC_INDEX),
        ("InletLipMode", vsp.ENGINE_LOC_INDEX),
        ("OutletFaceMode", vsp.ENGINE_LOC_INDEX),
        ("OutletLipMode", vsp.ENGINE_LOC_INDEX),
        ("InletFaceIndex", 0),
        ("InletLipIndex", 0),
        ("OutletFaceIndex", last_index),
        ("OutletLipIndex", last_index),
        ("InletModeType", vsp.ENGINE_MODE_FLOWTHROUGH),
    ):
        vsp.SetParmVal(body_id, name, "EngineModel", float(value))


def _build_panel(vsp: Any, geometry: VehicleGeometry, panel: SurfacePanel) -> str:
    """One radially-mounted lifting panel.

    Each panel is its own unsymmetrised WING geom rather than one symmetric wing:
    a symmetry shortcut would suppress exactly the asymmetric loads a sideslip or
    roll case exists to measure.
    """

    roll_deg = _normalise_rotation_deg(panel.clocking_deg + panel.dihedral_deg)
    if panel.camber and panel.mirror_plane is None and abs(roll_deg) > 90.0:
        # A roll past 90 deg maps +Z to -Z, so it would mount this panel with its
        # camber inverted -- silently, since the geometry is otherwise correct.
        raise ValueError(
            f"{panel.name} is cambered ({panel.camber:.3f}) and rolled "
            f"{roll_deg:.1f} deg, which would invert its section. Give it a "
            "mirror_plane instead, or make the section symmetric."
        )

    surface_id = str(vsp.AddGeom("WING", ""))
    if not surface_id:
        raise RuntimeError(f"OpenVSP failed to create {panel.name}")
    vsp.SetGeomName(surface_id, panel.name)
    symmetry = {None: vsp.SYM_NONE, "XZ": vsp.SYM_XZ, "XY": vsp.SYM_XY}[
        panel.mirror_plane
    ]
    vsp.SetParmVal(surface_id, "Sym_Planar_Flag", "Sym", float(symmetry))

    clock_rad = radians(panel.clocking_deg)
    vsp.SetParmVal(surface_id, "X_Rel_Location", "XForm", panel.root_le_x_m)
    vsp.SetParmVal(
        surface_id, "Y_Rel_Location", "XForm", panel.mount_radius_m * cos(clock_rad)
    )
    vsp.SetParmVal(
        surface_id, "Z_Rel_Location", "XForm", panel.mount_radius_m * sin(clock_rad)
    )
    vsp.SetParmVal(surface_id, "X_Rel_Rotation", "XForm", roll_deg)
    vsp.SetParmVal(surface_id, "Y_Rel_Rotation", "XForm", -panel.incidence_deg)

    vsp.SetDriverGroup(
        surface_id, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER
    )
    vsp.SetParmVal(surface_id, "Span", "XSec_1", panel.semispan_m)
    vsp.SetParmVal(surface_id, "Root_Chord", "XSec_1", panel.root_chord_m)
    vsp.SetParmVal(surface_id, "Tip_Chord", "XSec_1", panel.tip_chord_m)
    # Sweep measured at the LEADING edge (Sweep_Location = 0) so the geom origin
    # is unambiguously the root leading edge and the built planform matches the
    # station arithmetic in vehicle_geometry exactly.
    vsp.SetParmVal(surface_id, "Sweep_Location", "XSec_1", 0.0)
    vsp.SetParmVal(surface_id, "Sweep", "XSec_1", panel.le_sweep_deg)
    vsp.SetParmVal(surface_id, "Dihedral", "XSec_1", 0.0)
    vsp.SetParmVal(
        surface_id, "SectTess_U", "XSec_1", float(geometry.inputs.surface_tessellation)
    )
    vsp.SetParmVal(
        surface_id,
        "Tess_W",
        "Shape",
        float(2 * geometry.inputs.surface_tessellation + 1),
    )

    # Airfoil parameters live on the cross-section itself, reachable only through
    # GetXSecParm. Addressing them as a geom container group ("XSecCurve_0") is
    # what the older code in src/douglas_dart did, and every one of those writes
    # fails with "Can't Find Parm" -- invisibly, because this build's error stack
    # is only reachable via ErrorMgrSingleton.
    xsec_surf_id = str(vsp.GetXSecSurf(surface_id, 0))
    for index in range(vsp.GetNumXSec(xsec_surf_id)):
        vsp.ChangeXSecShape(xsec_surf_id, index, vsp.XS_FOUR_SERIES)
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        # MAX_CAMB: give the camber as a thickness fraction, not as a design CL.
        _set_xsec_parm(vsp, xsec_id, "CamberInputFlag", float(vsp.MAX_CAMB))
        _set_xsec_parm(vsp, xsec_id, "ThickChord", panel.thickness_to_chord)
        _set_xsec_parm(vsp, xsec_id, "Camber", panel.camber)
        _set_xsec_parm(vsp, xsec_id, "CamberLoc", panel.camber_location)
    return surface_id


def _assign_solver_sets(
    vsp: Any, body_id: str, surface_ids: tuple[str, ...]
) -> None:
    """Put the body in the thick set and every lifting panel in the thin set.

    See the module constants for why this split is load-bearing. Named so the
    grouping is legible in the OpenVSP GUI rather than being two anonymous user
    sets someone later reuses for something else.
    """

    vsp.SetSetName(_THICK_SET_INDEX, THICK_SET_NAME)
    vsp.SetSetName(_THIN_SET_INDEX, THIN_SET_NAME)
    vsp.SetSetFlag(body_id, _THICK_SET_INDEX, True)
    vsp.SetSetFlag(body_id, _THIN_SET_INDEX, False)
    for surface_id in surface_ids:
        vsp.SetSetFlag(surface_id, _THICK_SET_INDEX, False)
        vsp.SetSetFlag(surface_id, _THIN_SET_INDEX, True)


def build_model(geometry: VehicleGeometry, output_path: str | Path) -> BuildResult:
    """Build the vehicle and write it to ``output_path``."""

    vsp = load_openvsp()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    vsp.VSPRenew()
    vsp.ClearVSPModel()
    clear_errors(vsp)

    body_id = _build_body(vsp, geometry)
    check_errors(vsp, "body construction")
    wing_ids = tuple(_build_panel(vsp, geometry, p) for p in geometry.wing_panels)
    fin_ids = tuple(_build_panel(vsp, geometry, p) for p in geometry.fin_panels)
    check_errors(vsp, "lifting-surface construction")

    _assign_solver_sets(vsp, body_id, wing_ids + fin_ids)
    check_errors(vsp, "solver set assignment")

    vsp.Update()
    check_errors(vsp, "Update")

    vsp.WriteVSPFile(str(output_path), vsp.SET_ALL)
    check_errors(vsp, "WriteVSPFile")
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(
            f"OpenVSP reported no error but wrote no usable model: {output_path}"
        )

    return BuildResult(
        model_path=str(output_path),
        openvsp_version=str(vsp.GetVSPVersion()),
        body_id=body_id,
        wing_ids=wing_ids,
        fin_ids=fin_ids,
        body_station_count=len(geometry.body),
    )


def verify_model(model_path: str | Path, geometry: VehicleGeometry) -> dict[str, Any]:
    """Read the written file back and check it against the intended geometry.

    A separate load, not a query of the live session: the point is to prove the
    artifact on disk is what will be analysed, not that the builder's in-memory
    state was right.
    """

    vsp = load_openvsp()
    vsp.VSPRenew()
    vsp.ClearVSPModel()
    clear_errors(vsp)
    vsp.ReadVSPFile(str(model_path))
    vsp.Update()
    check_errors(vsp, 'read-back')

    geom_ids = list(vsp.FindGeoms())
    if not geom_ids:
        raise RuntimeError(f"no geometry loaded back from {model_path}")
    names = [str(vsp.GetGeomName(gid)) for gid in geom_ids]

    expected = (
        [BODY_GEOM_NAME]
        + [p.name for p in geometry.wing_panels]
        + [p.name for p in geometry.fin_panels]
    )
    missing = [name for name in expected if name not in names]
    if missing:
        raise RuntimeError(f"model is missing geoms: {missing} (found {names})")
    bad_names = [name for name in names if name.endswith("_Surface")]
    if bad_names:
        raise RuntimeError(
            f"geom names ending in '_Surface' crash the panel mesher: {bad_names}"
        )

    body_id = geom_ids[names.index(BODY_GEOM_NAME)]
    xsec_surf_id = str(vsp.GetXSecSurf(body_id, 0))
    built_stations = []
    body_length = geometry.sizing.body_length_m
    for index in range(vsp.GetNumXSec(xsec_surf_id)):
        xsec_id = str(vsp.GetXSec(xsec_surf_id, index))
        x_fraction = vsp.GetParmVal(vsp.GetXSecParm(xsec_id, "XLocPercent"))
        diameter = vsp.GetParmVal(vsp.GetXSecParm(xsec_id, "Circle_Diameter"))
        built_stations.append((x_fraction * body_length, diameter))

    for (built_x, built_d), intended in zip(built_stations, geometry.body):
        if abs(built_x - intended.x_m) > 1e-6 or abs(built_d - intended.diameter_m) > 1e-6:
            raise RuntimeError(
                f"body station mismatch after read-back: file has "
                f"(x={built_x:.6f}, d={built_d:.6f}), intended "
                f"(x={intended.x_m:.6f}, d={intended.diameter_m:.6f})"
            )

    return {
        "model_path": str(model_path),
        "openvsp_version": str(vsp.GetVSPVersion()),
        "geom_names": names,
        "body_station_count": len(built_stations),
        "body_stations_m": [
            {"x_m": x, "diameter_m": d} for x, d in built_stations
        ],
    }
