"""Derive the complete outer mold line from the sizing export plus our choices.

Pure geometry: no OpenVSP, no solver, no file I/O. Everything :mod:`vsp_build`
hands to the API is computed here first, so the mold line can be checked,
printed and diffed without a running OpenVSP.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, degrees, pi, radians, sin, sqrt, tan

from geometry_inputs import GeometryInputs, cubic_fair_fraction, normalised_fair_slope
from sizing_input import SizingInput


@dataclass(frozen=True)
class BodyStation:
    x_m: float
    diameter_m: float

    @property
    def radius_m(self) -> float:
        return 0.5 * self.diameter_m


@dataclass(frozen=True)
class SurfacePanel:
    """One exposed lifting panel, mounted radially on the body.

    Chords and semispan are EXPOSED values measured from the mount radius
    outward -- the part of the surface that exists as geometry. Reference areas
    stay theoretical (see :class:`ReferenceQuantities`).
    """

    name: str
    root_le_x_m: float
    mount_radius_m: float
    clocking_deg: float
    root_chord_m: float
    tip_chord_m: float
    semispan_m: float
    le_sweep_deg: float
    dihedral_deg: float
    incidence_deg: float
    thickness_to_chord: float
    camber: float
    camber_location: float
    mirror_plane: str | None = None
    """``"XZ"`` builds the opposite-side panel by REFLECTION instead of by a
    second geom rolled 180 deg.

    This is not cosmetic. A 180 deg roll maps +Z to -Z, so it mounts the panel on
    the correct side with its camber upside down -- the two halves of a cambered
    wing then fight each other, and the model quietly reports a rolling moment
    and a lift deficit at zero sideslip. A symmetric section is immune, which is
    why the fins can be rolled but the wing cannot.
    """

    @property
    def panel_count(self) -> int:
        """Physical panels this entry builds: 2 when mirrored, 1 otherwise."""

        return 2 if self.mirror_plane else 1

    @property
    def exposed_area_m2(self) -> float:
        """Area of ONE panel. Multiply by :attr:`panel_count` for the pair."""

        return 0.5 * (self.root_chord_m + self.tip_chord_m) * self.semispan_m

    @property
    def root_te_x_m(self) -> float:
        return self.root_le_x_m + self.root_chord_m

    @property
    def tip_le_x_m(self) -> float:
        return self.root_le_x_m + self.semispan_m * tan(radians(self.le_sweep_deg))

    @property
    def mean_aerodynamic_chord_m(self) -> float:
        taper = self.tip_chord_m / self.root_chord_m
        return (
            2.0 / 3.0 * self.root_chord_m * (1.0 + taper + taper**2) / (1.0 + taper)
        )

    @property
    def mac_le_x_m(self) -> float:
        """Axial station of the exposed panel's own MAC leading edge."""

        taper = self.tip_chord_m / self.root_chord_m
        y_mac = self.semispan_m / 3.0 * (1.0 + 2.0 * taper) / (1.0 + taper)
        return self.root_le_x_m + y_mac * tan(radians(self.le_sweep_deg))

    @property
    def quarter_chord_x_m(self) -> float:
        """Axial station of the panel's MAC quarter-chord -- its lift centroid
        to a first approximation, and what the static-margin solve moves."""

        return self.mac_le_x_m + 0.25 * self.mean_aerodynamic_chord_m


@dataclass(frozen=True)
class ReferenceQuantities:
    """VSPAERO reference values.

    Deliberately the THEORETICAL wing trapezoid, not the exposed panel area: it
    is what the trajectory model used to produce every CL and CD it flew with, so
    coefficients out of VSPAERO stay directly comparable to it.
    """

    area_m2: float
    span_m: float
    chord_m: float
    cg_x_m: float
    frontal_area_m2: float

    @property
    def frontal_referencing_factor(self) -> float:
        """Multiply a wing-referenced coefficient by this to get the
        frontal-area-referenced form the trajectory model's CD0 uses."""

        return self.area_m2 / self.frontal_area_m2


@dataclass(frozen=True)
class VehicleGeometry:
    sizing: SizingInput
    inputs: GeometryInputs
    body: tuple[BodyStation, ...]
    wing_panels: tuple[SurfacePanel, ...]
    fin_panels: tuple[SurfacePanel, ...]
    references: ReferenceQuantities

    @property
    def inlet_diameter_m(self) -> float:
        return self.body[0].diameter_m

    @property
    def exit_diameter_m(self) -> float:
        return self.body[-1].diameter_m

    @property
    def base_annulus_area_m2(self) -> float:
        """Annular base left around the nozzle where the boattail stops.

        Zero when the boattail closes onto the nozzle. Non-zero at the flown
        8 deg half-angle, and it is where the drag build-up charges base drag --
        a term VSPAERO's inviscid solution does NOT contain.
        """

        base = pi * self.exit_diameter_m**2 / 4.0
        nozzle = pi * self.sizing.nozzle_exit_diameter_m**2 / 4.0
        return max(base - nozzle, 0.0)

    def body_diameter_at_x_m(self, x_m: float) -> float:
        """Piecewise-linear OML diameter, for checking surface root mounts."""

        stations = self.body
        clamped = min(max(x_m, stations[0].x_m), stations[-1].x_m)
        for lower, upper in zip(stations, stations[1:]):
            if lower.x_m <= clamped <= upper.x_m:
                span = upper.x_m - lower.x_m
                if span <= 0.0:
                    return lower.diameter_m
                f = (clamped - lower.x_m) / span
                return lower.diameter_m + f * (upper.diameter_m - lower.diameter_m)
        return stations[-1].diameter_m

    @property
    def all_panels(self) -> tuple[SurfacePanel, ...]:
        return self.wing_panels + self.fin_panels


# ---------------------------------------------------------------------------
# Body
# ---------------------------------------------------------------------------


def build_body_stations(
    sizing: SizingInput, inputs: GeometryInputs
) -> tuple[BodyStation, ...]:
    """Nose cowl -> constant barrel -> boattail, as circular stations.

    The body is a single flow-through duct: open at the nose lip, open at the
    aft face. That is the same picture the flown drag build-up takes ('this
    vehicle's inlet is the NOSE of the body, not a pod').
    """

    body_radius = 0.5 * sizing.body_diameter_m
    lip_radius = 0.5 * sizing.inlet_lip_diameter_m
    nose_length = sizing.nose_fairing_length_m

    stations: list[BodyStation] = []

    # --- Nose cowl: cubic, commanded lip angle, tangent to the barrel -----
    nose_slope = normalised_fair_slope(
        inputs.nose_lip_half_angle_deg, nose_length, body_radius - lip_radius
    )
    for index in range(inputs.nose_station_count):
        u = index / (inputs.nose_station_count - 1)
        radius = lip_radius + (body_radius - lip_radius) * cubic_fair_fraction(
            u, nose_slope
        )
        stations.append(BodyStation(u * nose_length, 2.0 * radius))

    # --- Constant barrel --------------------------------------------------
    # One interior station so the skinning spline cannot bow the cylinder
    # between the two fairings.
    barrel_mid_x = 0.5 * (nose_length + sizing.boattail_start_x_m)
    stations.append(BodyStation(barrel_mid_x, sizing.body_diameter_m))
    stations.append(BodyStation(sizing.boattail_start_x_m, sizing.body_diameter_m))

    # --- Boattail ---------------------------------------------------------
    exit_radius = _boattail_exit_radius_m(sizing, inputs)
    boattail_length = sizing.boattail_length_m
    boattail_slope = normalised_fair_slope(
        inputs.boattail_half_angle_deg or 0.0, boattail_length, body_radius - exit_radius
    )
    # Read the cubic in reverse: u = 0 at the AFT face (commanded closure angle)
    # and u = 1 at the barrel (tangent), so the boattail leaves the cylinder
    # without a shoulder.
    for index in range(1, inputs.boattail_station_count):
        u = 1.0 - index / (inputs.boattail_station_count - 1)
        radius = exit_radius + (body_radius - exit_radius) * cubic_fair_fraction(
            u, boattail_slope
        )
        x = sizing.boattail_start_x_m + (1.0 - u) * boattail_length
        stations.append(BodyStation(x, 2.0 * radius))

    _validate_body(stations, sizing)
    return tuple(stations)


def _boattail_exit_radius_m(sizing: SizingInput, inputs: GeometryInputs) -> float:
    """Aft OML radius, floored at the nozzle -- the body cannot close inside
    its own exhaust.

    Mirrors ``medium_model/drag_buildup.base_diameter_m`` exactly, so the VSP
    mold line and the drag the vehicle was flown with describe one aft body.
    """

    nozzle_radius = 0.5 * sizing.nozzle_exit_diameter_m
    if inputs.boattail_half_angle_deg is None:
        return nozzle_radius
    shrink = sizing.boattail_length_m * tan(radians(inputs.boattail_half_angle_deg))
    return max(0.5 * sizing.body_diameter_m - shrink, nozzle_radius)


def _validate_body(stations: list[BodyStation], sizing: SizingInput) -> None:
    problems: list[str] = []
    for lower, upper in zip(stations, stations[1:]):
        if upper.x_m <= lower.x_m:
            problems.append(
                f"body stations are not strictly increasing in x: "
                f"{lower.x_m:.6f} then {upper.x_m:.6f}"
            )
    if abs(stations[-1].x_m - sizing.body_length_m) > 1e-9:
        problems.append(
            f"body ends at {stations[-1].x_m:.6f} m, not the sized "
            f"{sizing.body_length_m:.6f} m"
        )
    if max(s.diameter_m for s in stations) > sizing.body_diameter_m + 1e-9:
        problems.append("a body station is wider than the sized body diameter")
    if problems:
        raise ValueError("derived body mold line is invalid:\n  - " + "\n  - ".join(problems))


# ---------------------------------------------------------------------------
# Lifting surfaces
# ---------------------------------------------------------------------------


MEASURED_MIN_OVERLAP_M = 0.006
"""Floor on root interpenetration, measured not guessed.

A root mounted exactly tangent to the body produces a degenerate sliver face
during VSPAERO's triangulation ('PGFace Invalid in Triangulate_DBA'), which is
why any overlap is commanded at all. But half the root airfoil's own thickness
-- the natural geometric scale -- is not enough for a thin surface:

| fin root overlap | mixed thick/thin VLM result |
|---|---|
| 2.9 mm (t/c 0.04, 145 mm root) | NaN from GMRES iteration 0 |
| 5.8 mm (t/c 0.08, same planform) | solves, CS = -0.0011 |
| 6.1 mm (the wing, t/c 0.04, 309 mm root) | always solved |

The failure has no error message and no warning: GMRES simply returns NaN and
every coefficient is NaN. 6 mm is the shallowest depth demonstrated to solve on
this configuration. See ``scripts/vsp_model/README.md``.
"""


def radial_mount_overlap_m(
    root_chord_m: float,
    thickness_to_chord: float,
    minimum_m: float | None = None,
) -> float:
    """Radial interpenetration of a surface root into the body.

    Half the root airfoil's maximum thickness scales the overlap to the surface,
    floored at :data:`MEASURED_MIN_OVERLAP_M` so a thin surface cannot mount too
    shallowly for the solver.
    """

    floor = MEASURED_MIN_OVERLAP_M if minimum_m is None else minimum_m
    return max(0.5 * thickness_to_chord * root_chord_m, floor)


def clocking_angles_deg(count: int, offset_deg: float) -> tuple[float, ...]:
    if count <= 0:
        return ()
    return tuple((offset_deg + i * 360.0 / count) % 360.0 for i in range(count))


def build_wing_panels(
    sizing: SizingInput, inputs: GeometryInputs
) -> tuple[SurfacePanel, ...]:
    """Two exposed panels cut from the sizing model's theoretical trapezoid.

    The sizing span (532.5 mm) is tip-to-tip THROUGH the body, so the panel that
    physically exists starts at the body surface. Chord at the mount radius is
    read off the same straight taper line, which keeps the built surface exactly
    on the theoretical planform rather than approximating it.
    """

    # Area scale holds aspect ratio and taper, so linear dimensions go as sqrt.
    linear = sqrt(max(inputs.wing_area_scale, 1e-9))
    semispan_theoretical = 0.5 * sizing.wing_span_m * linear
    body_radius = 0.5 * sizing.body_diameter_m
    root_theoretical = sizing.wing_root_chord_m * linear
    tip_theoretical = sizing.wing_tip_chord_m * linear

    def chord_at(y_m: float) -> float:
        f = y_m / semispan_theoretical
        return root_theoretical + f * (tip_theoretical - root_theoretical)

    overlap = radial_mount_overlap_m(
        chord_at(body_radius),
        sizing.wing_thickness_to_chord,
        inputs.min_radial_mount_overlap_m,
    )
    mount_radius = body_radius - overlap
    root_chord = chord_at(mount_radius)
    le_sweep = sizing.wing_le_sweep_deg
    # The trapezoid's centreline LE is the placement input; the panel's own root
    # LE is that station swept out to the mount radius.
    root_le_x = inputs.wing_root_le_x_m + mount_radius * tan(radians(le_sweep))

    # ONE entry, mirrored about XZ -- see SurfacePanel.mirror_plane for why the
    # port panel is not simply a second geom rolled 180 degrees.
    # PACKAGING CONSTRAINT (from the V4 simulator session, 2026-08-14): the
    # combustion chamber occupies 428.0-817.3 mm at the FULL body diameter -- V4
    # deleted the annular gap, so the chamber wall is the skin. No spar can cross
    # it. Everything aft of 817.3 mm is tank annulus, which a spar can pass
    # through. This is a real structural limit, not an aerodynamic preference,
    # and it is the only hard bound on wing station the project has.
    chamber_start, chamber_end = sizing.chamber_start_x_m, sizing.chamber_end_x_m
    root_te_x = inputs.wing_root_le_x_m + root_theoretical
    if inputs.wing_root_le_x_m < chamber_end and root_te_x > chamber_start:
        raise ValueError(
            f"wing root chord {inputs.wing_root_le_x_m * 1e3:.1f}-"
            f"{root_te_x * 1e3:.1f} mm crosses the combustion chamber "
            f"({chamber_start * 1e3:.1f}-{chamber_end * 1e3:.1f} mm), which sits "
            "at the full body diameter with no annular gap -- no spar can pass "
            "through it. Move the wing aft of the chamber or forward of it."
        )

    return (
        SurfacePanel(
            name="Wing_Panel",
            root_le_x_m=root_le_x,
            mount_radius_m=mount_radius,
            clocking_deg=0.0,
            root_chord_m=root_chord,
            tip_chord_m=tip_theoretical,
            semispan_m=semispan_theoretical - mount_radius,
            le_sweep_deg=le_sweep,
            dihedral_deg=inputs.wing_dihedral_deg,
            incidence_deg=inputs.wing_incidence_deg,
            thickness_to_chord=sizing.wing_thickness_to_chord,
            camber=inputs.wing_camber,
            camber_location=inputs.wing_camber_location,
            mirror_plane="XZ",
        ),
    )


def _fin_panel(
    sizing: SizingInput,
    inputs: GeometryInputs,
    name: str,
    panel_area_m2: float,
    clocking_deg: float,
) -> SurfacePanel:
    """One fin panel of a commanded area, at a commanded clocking."""

    semispan = sqrt(inputs.fin_aspect_ratio * panel_area_m2)
    mean_chord = panel_area_m2 / semispan
    root_chord = 2.0 * mean_chord / (1.0 + inputs.fin_taper_ratio)
    overlap = radial_mount_overlap_m(
        root_chord, inputs.fin_thickness_to_chord, inputs.min_radial_mount_overlap_m
    )
    root_te_x = (
        inputs.fin_root_te_x_m
        if inputs.fin_root_te_x_m is not None
        else sizing.boattail_start_x_m
    )
    return SurfacePanel(
        name=name,
        root_le_x_m=root_te_x - root_chord,
        mount_radius_m=0.5 * sizing.body_diameter_m - overlap,
        clocking_deg=clocking_deg,
        root_chord_m=root_chord,
        tip_chord_m=root_chord * inputs.fin_taper_ratio,
        semispan_m=semispan,
        le_sweep_deg=inputs.fin_le_sweep_deg,
        dihedral_deg=0.0,
        incidence_deg=0.0,
        thickness_to_chord=inputs.fin_thickness_to_chord,
        camber=0.0,
        camber_location=0.4,
    )


def _build_biased_fin_pairs(
    sizing: SizingInput, inputs: GeometryInputs, total_area_m2: float
) -> tuple[SurfacePanel, ...]:
    """A ``+`` tail whose vertical and horizontal pairs carry unequal area.

    See ``GeometryInputs.fin_vertical_fraction``. The vertical pair sits at
    90/270 deg and the horizontal pair at 0/180 deg, so each pair works one axis
    cleanly instead of splitting cos^2(45) into both.
    """

    fraction = min(max(inputs.fin_vertical_fraction, 0.0), 1.0)
    vertical_each = 0.5 * fraction * total_area_m2
    horizontal_each = 0.5 * (1.0 - fraction) * total_area_m2

    panels: list[SurfacePanel] = []
    for index, clocking in enumerate((90.0, 270.0)):
        if vertical_each > 1e-9:
            panels.append(
                _fin_panel(sizing, inputs, f"Fin_V{index + 1}", vertical_each, clocking)
            )
    for index, clocking in enumerate((0.0, 180.0)):
        if horizontal_each > 1e-9:
            panels.append(
                _fin_panel(sizing, inputs, f"Fin_H{index + 1}", horizontal_each, clocking)
            )
    if not panels:
        return ()
    _validate_fin_station(sizing, panels[0])
    return tuple(panels)


def _validate_fin_station(sizing: SizingInput, panel: SurfacePanel) -> None:
    barrel_start, barrel_end = sizing.nose_fairing_length_m, sizing.boattail_start_x_m
    if panel.root_le_x_m < barrel_start - 1e-9 or panel.root_te_x_m > barrel_end + 1e-9:
        raise ValueError(
            f"fin root chord {panel.root_le_x_m:.4f}-{panel.root_te_x_m:.4f} m runs "
            f"off the constant-diameter barrel ({barrel_start:.4f}-{barrel_end:.4f} m)"
        )


def build_fin_panels(
    sizing: SizingInput, inputs: GeometryInputs
) -> tuple[SurfacePanel, ...]:
    """Cruciform fins, sized from an area ratio rather than read from the model.

    There are NO tail surfaces anywhere in the sizing chain -- no area, no volume
    coefficient, no drag, no mass. Every number here is ours.
    """

    if inputs.fin_count == 0 or inputs.fin_area_ratio <= 0.0:
        return ()

    total_area = inputs.fin_area_ratio * sizing.wing_reference_area_m2
    if inputs.fin_count == 4 and abs(inputs.fin_vertical_fraction - 0.5) > 1e-9:
        return _build_biased_fin_pairs(sizing, inputs, total_area)
    count = (
        len(inputs.fin_clocking_deg_explicit)
        if inputs.fin_clocking_deg_explicit is not None
        else inputs.fin_count
    )
    panel_area = total_area / count
    # AR of one exposed panel, treating its exposed height as the span.
    semispan = sqrt(inputs.fin_aspect_ratio * panel_area)
    mean_chord = panel_area / semispan
    root_chord = 2.0 * mean_chord / (1.0 + inputs.fin_taper_ratio)
    tip_chord = root_chord * inputs.fin_taper_ratio

    overlap = radial_mount_overlap_m(
        root_chord, inputs.fin_thickness_to_chord, inputs.min_radial_mount_overlap_m
    )
    mount_radius = 0.5 * sizing.body_diameter_m - overlap

    root_te_x = (
        inputs.fin_root_te_x_m
        if inputs.fin_root_te_x_m is not None
        else sizing.boattail_start_x_m
    )
    root_le_x = root_te_x - root_chord

    barrel_start = sizing.nose_fairing_length_m
    barrel_end = sizing.boattail_start_x_m
    if root_le_x < barrel_start - 1e-9 or root_te_x > barrel_end + 1e-9:
        raise ValueError(
            f"fin root chord {root_le_x:.4f}-{root_te_x:.4f} m runs off the "
            f"constant-diameter barrel ({barrel_start:.4f}-{barrel_end:.4f} m). "
            "A root that overhangs a taper touches the surface at one end and "
            "opens a gap at the other."
        )

    clocking = (
        inputs.fin_clocking_deg_explicit
        if inputs.fin_clocking_deg_explicit is not None
        else clocking_angles_deg(inputs.fin_count, inputs.fin_clocking_offset_deg)
    )
    return tuple(
        SurfacePanel(
            name=f"Fin_{index + 1}",
            root_le_x_m=root_le_x,
            mount_radius_m=mount_radius,
            clocking_deg=angle,
            root_chord_m=root_chord,
            tip_chord_m=tip_chord,
            semispan_m=semispan,
            le_sweep_deg=inputs.fin_le_sweep_deg,
            dihedral_deg=0.0,
            incidence_deg=0.0,
            thickness_to_chord=inputs.fin_thickness_to_chord,
            # Symmetric section: a cambered fin would trim the vehicle in roll/yaw.
            camber=0.0,
            camber_location=0.4,
        )
        for index, angle in enumerate(clocking)
    )


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------


def build_vehicle_geometry(
    sizing: SizingInput, inputs: GeometryInputs, cg_x_m: float | None = None
) -> VehicleGeometry:
    """Assemble the full mold line.

    ``cg_x_m`` overrides both ``inputs.reference_cg_x_m`` and the mass model, so
    a stability solve can re-reference moments without rebuilding anything else.
    """

    body = build_body_stations(sizing, inputs)
    wing_panels = build_wing_panels(sizing, inputs)
    fin_panels = build_fin_panels(sizing, inputs)

    resolved_cg = cg_x_m if cg_x_m is not None else inputs.reference_cg_x_m
    if resolved_cg is None:
        from mass_cg import vehicle_mass_properties  # local import: optional dependency

        resolved_cg = vehicle_mass_properties(sizing, inputs).cg_x_m

    references = ReferenceQuantities(
        area_m2=sizing.wing_reference_area_m2,
        span_m=sizing.wing_span_m,
        chord_m=sizing.wing_mean_aerodynamic_chord_m,
        cg_x_m=float(resolved_cg),
        frontal_area_m2=sizing.frontal_area_m2,
    )
    return VehicleGeometry(
        sizing=sizing,
        inputs=inputs,
        body=body,
        wing_panels=wing_panels,
        fin_panels=fin_panels,
        references=references,
    )


def describe(geometry: VehicleGeometry) -> str:
    """Human-readable dump of the derived mold line."""

    lines: list[str] = []
    sizing = geometry.sizing
    lines.append(f"Source          : {sizing.source_path} (git {sizing.git_sha[:7]})")
    lines.append(
        f"Body            : {sizing.body_diameter_m * 1e3:.1f} mm dia x "
        f"{sizing.body_length_m * 1e3:.1f} mm long"
    )
    lines.append(
        f"Inlet lip       : {geometry.inlet_diameter_m * 1e3:.1f} mm  (open, flow-through)"
    )
    lines.append(
        f"Aft OML face    : {geometry.exit_diameter_m * 1e3:.1f} mm  "
        f"(nozzle {sizing.nozzle_exit_diameter_m * 1e3:.1f} mm, "
        f"annular base {geometry.base_annulus_area_m2 * 1e4:.1f} cm^2)"
    )
    lines.append("")
    lines.append("Body stations (x mm, dia mm):")
    for station in geometry.body:
        lines.append(f"   {station.x_m * 1e3:8.1f}   {station.diameter_m * 1e3:7.1f}")
    lines.append("")
    for panel in geometry.all_panels:
        mirror = f" mirror={panel.mirror_plane}" if panel.mirror_plane else ""
        lines.append(
            f"{panel.name:<14}: root LE x={panel.root_le_x_m * 1e3:7.1f} mm  "
            f"c_r={panel.root_chord_m * 1e3:6.1f}  c_t={panel.tip_chord_m * 1e3:6.1f}  "
            f"b_exp={panel.semispan_m * 1e3:6.1f}  clock={panel.clocking_deg:5.1f} deg  "
            f"S_exp={panel.exposed_area_m2 * 1e4:6.1f} cm^2 x{panel.panel_count}  "
            f"c/4 x={panel.quarter_chord_x_m * 1e3:7.1f} mm{mirror}"
        )
    references = geometry.references
    lines.append("")
    lines.append(
        f"Reference       : Sref={references.area_m2:.6f} m^2  "
        f"bref={references.span_m:.4f} m  cref={references.chord_m:.4f} m  "
        f"Xcg={references.cg_x_m:.4f} m"
    )
    lines.append(
        f"Frontal factor  : x{references.frontal_referencing_factor:.4f} converts a "
        "wing-referenced coefficient to the model's frontal-area convention"
    )
    return "\n".join(lines)
