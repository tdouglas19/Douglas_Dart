"""Everything the OML needs that the sizing model does NOT produce.

The sizing chain is a point-mass trajectory model. It has no CG, no moment arms,
no stability terms and no tail surfaces, so a whole layer of real geometry is a
free choice rather than a model output. That layer lives here, in one place, with
a default and a stated reason for every value -- so a reader can tell at a glance
what the trajectory model decided and what we did.

Read ``README.md`` in this directory for the architecture these numbers describe.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import radians, tan
from typing import Any

# ---------------------------------------------------------------------------
# Sweep grid
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SweepGrid:
    """One VSPAERO run's Mach/alpha/beta grid and solver settings.

    Points are executed individually rather than as a VSPAERO start/end/count
    sweep: the Mach list is nonuniform, and VSPAERO would linearly interpolate
    between the endpoints and silently analyse conditions nobody asked for.
    """

    name: str
    method: str  # "vortex_lattice" | "panel"
    mach_values: tuple[float, ...]
    alpha_deg_values: tuple[float, ...]
    beta_deg_values: tuple[float, ...] = (0.0,)
    wake_iterations: int = 3
    stability: bool = False
    """Run VSPAERO's stability mode, which adds the rate derivatives
    (Cmq, Cnr, Clp, ...) needed for dynamic modes. Costs an extra solve per
    point, so it is off for the bulk grid and on for a small dedicated grid."""

    def point_count(self) -> int:
        return (
            len(self.mach_values) * len(self.alpha_deg_values) * len(self.beta_deg_values)
        )


# The mission runs M 0.12 -> 1.10. VSPAERO's linear methods are invalid through
# the transonic (the Prandtl-Glauert factor is singular at M = 1), so the grid
# stops at 0.80 and the trajectory model keeps its own transonic multiplier.
DEFAULT_VLM_GRID = SweepGrid(
    name="vlm_subsonic",
    method="vortex_lattice",
    mach_values=(0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80),
    alpha_deg_values=(-4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0),
    beta_deg_values=(0.0,),
    wake_iterations=3,
)

# Sideslip needs the asymmetric solution, so these points are separate rather
# than an extra axis on the grid above (which would multiply every alpha by
# every beta for no benefit -- Cn_beta is wanted at small alpha).
DEFAULT_VLM_SIDESLIP_GRID = SweepGrid(
    name="vlm_sideslip",
    method="vortex_lattice",
    mach_values=(0.30, 0.60),
    alpha_deg_values=(0.0, 4.0),
    beta_deg_values=(-4.0, 0.0, 4.0),
    wake_iterations=3,
)

# Stability derivatives at the two conditions that decide the airframe: the
# dive-exit / ramjet-light point (M ~ 0.49) and the cruise-out drag strip.
DEFAULT_STABILITY_GRID = SweepGrid(
    name="stability",
    method="vortex_lattice",
    mach_values=(0.30, 0.50, 0.80),
    alpha_deg_values=(0.0, 4.0),
    beta_deg_values=(0.0,),
    wake_iterations=3,
    stability=True,
)

# Panel points cost ~1-3 min each on this geometry (docs/openvsp_real_api_findings.md).
# This grid exists to check the VLM against the thick flow-through body, not to
# replace it.
DEFAULT_PANEL_GRID = SweepGrid(
    name="panel_spotcheck",
    method="panel",
    mach_values=(0.30, 0.60),
    alpha_deg_values=(0.0, 4.0),
    beta_deg_values=(0.0,),
    wake_iterations=3,
)


# ---------------------------------------------------------------------------
# Geometry choices
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GeometryInputs:
    """Non-sized geometry: profiles, surface placement, fins, tessellation."""

    # --- Nose cowl -------------------------------------------------------
    nose_lip_half_angle_deg: float = 12.0
    """External flare angle at the inlet lip.

    The OML fairs from the 115.6 mm inlet lip out to the 214 mm barrel over the
    sizing model's 428 mm nose fairing. The model gives that LENGTH and nothing
    else, so the profile is ours. A cubic with a commanded lip angle and zero
    slope at the barrel gives a cowl that opens quickly then flattens tangent to
    the body -- the conventional subsonic cowl shape, and the one the drag
    build-up's cowl-suction term assumes ('a rounded subsonic lip on a long,
    gently-expanding cowl'). The mean slope over the fairing is only 6.6 deg, so
    12 deg at the lip is a mild flare, not a blunt face.
    """

    nose_station_count: int = 7
    """Cross-sections used to resolve the nose cowl curve (>= 3)."""

    # --- Boattail --------------------------------------------------------
    boattail_half_angle_deg: float = 8.0
    """Matches ``medium_model/drag_buildup.BOATTAIL_HALF_ANGLE_DEG``.

    DELIBERATELY NOT the sizing JSON's ``boattail_exit_diameter_m``. That field
    just repeats the nozzle diameter, which over the 214 mm tail length implies a
    13 deg half-angle -- past the ~8 deg attached-flow limit the drag model itself
    calls the upper bound, and past what the vehicle was actually FLOWN with. At
    8 deg the body closes to 153.9 mm and carries an annular base around the
    115.6 mm nozzle, which is what the flown build-up charges base drag on.
    Set to ``None`` to close the OML onto the nozzle diameter instead.
    """

    boattail_station_count: int = 5

    # --- Wing placement --------------------------------------------------
    wing_root_le_x_m: float = 1.000
    """Body station of the wing root chord's leading edge.

    Free choice: the sizing model is point-mass and has no CG or moment arms, so
    it cannot produce this. 1.000 m puts the root chord at 1.000-1.379 m, clear
    of the nose fairing (ends 0.428 m) and the boattail (starts 2.060 m).
    ``stability.py`` solves for this against a static-margin target.
    """

    wing_area_scale: float = 1.0
    """Scale on the V4 wing PLANFORM AREA, holding aspect ratio and taper.

    1.0 is the sizing model's wing exactly. Above 1.0 departs from the V4 freeze
    and must be handed back to the trajectory model, because wing area drives
    stall speed, mass (6.0 kg/m^2) and drag -- none of which this repo prices.
    Span scales as sqrt(scale), chords likewise.
    """

    wing_dihedral_deg: float = 0.0
    wing_incidence_deg: float = 0.0
    """Zero in the sizing model because they are UNMODELLED, not because they
    were chosen (the JSON's own wing note says so). Kept zero here so the VSP
    model reproduces the flown planform exactly; both are live knobs."""

    wing_camber: float = 0.02
    wing_camber_location: float = 0.40
    """The sizing model's airfoil is the key 'thin_cambered' with t/c 0.04 and
    cl_max_2d 1.2 -- a label, not ordinates. A NACA 4-series with 2% camber at
    40% chord reproduces that cl_max at this thickness and is what gets built."""

    # --- Fins ------------------------------------------------------------
    fin_count: int = 4
    fin_clocking_offset_deg: float = 45.0
    """Cruciform, clocked 45 deg from the wing plane so the fins sit between the
    wing panels rather than in their wake (user decision)."""

    fin_area_ratio: float = 0.35
    """Total exposed fin area (all four panels) as a fraction of the wing
    reference area. 0.35 -> 0.0534 m^2 total, 0.0133 m^2 per panel.

    KNOWN TOO SMALL. With this fin the measured Cm_alpha is +3.3 /rad at M 0.30,
    i.e. the airframe is strongly pitch-UNSTABLE: the slender body's Munk moment
    dominates and neither the wing nor this fin comes close to countering it.
    The value is kept as the documented starting point rather than quietly
    replaced, because it is what the vehicle looks like with the sizing model's
    own wing and a conventionally-proportioned tail. Run
    ``sweep_stability.py`` for the search over this and the wing station."""

    fin_clocking_deg_explicit: tuple[float, ...] | None = None
    """Explicit clocking angle per fin panel, overriding the evenly-spaced set.

    Needed for any tail that is NOT rotationally symmetric -- in particular a
    V-tail, whose whole point is to put every panel in the upper half so nothing
    projects below the body on a belly landing. Clocking is measured from the +Y
    axis, so 90 deg is straight up and any angle in (0, 180) clears the ground.
    """

    fin_vertical_fraction: float = 0.5
    """Share of the total fin area given to the VERTICAL pair.

    Only meaningful with ``fin_clocking_offset_deg = 0`` (a ``+`` arrangement,
    vertical pair at 90/270 deg, horizontal pair at 0/180 deg). 0.5 reproduces a
    symmetric cruciform; 1.0 is a vertical-only tail.

    WHY THIS EXISTS. A cruciform is rotationally invariant -- X and + give the
    same yaw stiffness for the same total area, because each 45 deg panel
    contributes cos^2(45) = 1/2 to each axis. Biasing the area is NOT invariant:
    two vertical panels of total area S give twice the yaw stiffness of four
    cruciform panels of total area S. On this vehicle yaw is the binding axis and
    the wing (moved aft) carries pitch, so area spent on horizontal fins is
    largely wasted against the constraint that actually binds.
    """

    fin_aspect_ratio: float = 1.20
    fin_taper_ratio: float = 0.45
    fin_le_sweep_deg: float = 35.0
    fin_thickness_to_chord: float = 0.04
    fin_root_te_x_m: float | None = None
    """Body station of the fin root TRAILING edge. ``None`` -> the boattail start,
    which is the furthest aft a root chord can sit and still land entirely on the
    constant-diameter barrel (a root running onto the boattail opens a gap at its
    aft end -- a real defect found in the GUI on the previous vehicle, see
    docs/openvsp_real_api_findings.md)."""

    # --- Surface root mounting -------------------------------------------
    min_radial_mount_overlap_m: float | None = None
    """Floor on how deep a surface root is buried in the body.

    The natural scale is the root airfoil's own half-thickness, but for a thin
    fin that is far too shallow: at t/c 0.04 and a 145 mm root chord it gives
    2.9 mm, and VSPAERO's mixed thick-body/thin-surface solve then returns NaN
    from GMRES iteration 0 -- no warning, no error, just NaN. Doubling it to
    5.8 mm solves cleanly, and the wing (6.1 mm by the same rule) never failed.
    ``None`` uses the measured threshold from :func:`resolved_min_overlap_m`.
    """

    # --- Reference quantities -------------------------------------------
    reference_cg_x_m: float | None = None
    """Moment reference station. ``None`` -> the mass model's computed CG
    (``mass_cg.py``). Pitching moment is meaningless without this, and the sizing
    model has no CG at all, so it is built rather than read."""

    # --- Meshing ---------------------------------------------------------
    fuselage_tessellation: int = 33
    surface_tessellation: int = 17

    # --- Sweeps ----------------------------------------------------------
    sweeps: tuple[SweepGrid, ...] = (
        DEFAULT_VLM_GRID,
        DEFAULT_VLM_SIDESLIP_GRID,
        DEFAULT_STABILITY_GRID,
        DEFAULT_PANEL_GRID,
    )

    notes: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.nose_station_count < 3:
            raise ValueError("nose_station_count must be at least 3")
        if self.boattail_station_count < 3:
            raise ValueError("boattail_station_count must be at least 3")
        if self.fin_count not in (0, 2, 3, 4, 6, 8):
            raise ValueError(f"unsupported fin_count: {self.fin_count}")
        if self.fin_area_ratio < 0.0:
            raise ValueError("fin_area_ratio must be non-negative")
        if self.fin_aspect_ratio <= 0.0:
            raise ValueError("fin_aspect_ratio must be positive")
        if not 0.0 < self.fin_taper_ratio <= 1.0:
            raise ValueError("fin_taper_ratio must be in (0, 1]")
        if self.boattail_half_angle_deg is not None and not (
            0.0 < self.boattail_half_angle_deg < 45.0
        ):
            raise ValueError("boattail_half_angle_deg must be in (0, 45)")

    def to_dict(self) -> dict[str, Any]:
        """Provenance payload written next to every generated model."""

        payload = asdict(self)
        payload["sweeps"] = [asdict(grid) for grid in self.sweeps]
        return payload


def cubic_fair_fraction(u: float, end_slope_normalised: float) -> float:
    """Cubic blend h(u) with h(0)=0, h(1)=1, h'(1)=0 and h'(0) commanded.

    Used for both the nose cowl (opens at a commanded lip angle, tangent to the
    barrel) and the boattail read in reverse. ``end_slope_normalised`` is the
    starting slope in units where the whole fairing spans 1.0 in both axes.

    Monotonic for ``end_slope_normalised`` in [0, 3]; above 3 the curve reverses
    and would produce a bulged, self-shadowing mold line, so it is clamped by
    :func:`normalised_fair_slope` before it gets here.
    """

    c = end_slope_normalised
    a = c - 2.0
    b = 3.0 - 2.0 * c
    return a * u**3 + b * u**2 + c * u


def normalised_fair_slope(
    half_angle_deg: float, fairing_length_m: float, radius_change_m: float
) -> float:
    """Convert a physical end half-angle into the cubic's normalised slope.

    Clamped to the monotonic range [0, 3]; at the top of that range the fairing
    is as blunt as a single cubic can be without bulging.
    """

    if radius_change_m <= 0.0 or fairing_length_m <= 0.0:
        return 0.0
    slope = fairing_length_m * tan(radians(half_angle_deg)) / radius_change_m
    return min(max(slope, 0.0), 3.0)
