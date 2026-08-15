"""Analytic 2D geometry kernel for the Douglas Dart integrated propulsion system.

The vehicle is represented as an ordered list of typed ``Segment`` objects on
the (x, r) half-plane -- NOT as a sampled point cloud.  Every segment carries
exact endpoints, a closed-form ``r_at(x)``, and a provenance tag saying whether
it came from the model or was invented.

Why analytic and not points-every-mm:

  * Every station break in this vehicle is at a non-integer mm (817.307,
    2059.607, 2273.607).  A 1 mm grid aliases all of them.
  * The duct wall is 1.0 mm and the skin 1.5 mm.  A 1 mm sample grid cannot
    resolve the wall it is meant to draw.
  * The 1068 mm tailpipe needs two points, not 1068.

Sampling is therefore an EXPORT (``sample_uniform``), derived from the model,
never the model itself.

Input is ``out_medium_model/v4_export/structural_dimensions.json`` plus
``assumptions.json`` in this package.  Nothing else -- no imports of
``medium_model``, so this tool cannot silently drift with the flight code.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent.parent

DEFAULT_DIMS = _REPO / "out_medium_model" / "v4_export" / "structural_dimensions.json"
DEFAULT_ASSUMPTIONS = _HERE / "assumptions.json"

MM = 1000.0  # metres -> millimetres, the drawing unit throughout


# ==========================================================================
# provenance
# ==========================================================================

JSON = "JSON"          # straight out of structural_dimensions.json
DERIVED = "DERIVED"    # closed-form consequence of JSON values only
ASSUMED = "ASSUMED"    # invented here; lives in assumptions.json


# ==========================================================================
# curve laws  (t in [0, 1] along the segment)
# ==========================================================================

def _line(t: float) -> float:
    return t


def _smoothstep(t: float) -> float:
    """Zero slope at BOTH ends.  Used where a duct must not dump area
    immediately behind a lip."""
    return t * t * (3.0 - 2.0 * t)


def _parabolic_tangent(t: float) -> float:
    """Zero slope at the FAR end only -- meets a cylinder tangentially."""
    return 1.0 - (1.0 - t) ** 2


CURVES: dict[str, Callable[[float], float]] = {
    "line": _line,
    "cone": _line,              # alias; a cone is a line in (x, r)
    "smoothstep": _smoothstep,
    "parabolic_tangent": _parabolic_tangent,
}


# ==========================================================================
# Segment
# ==========================================================================

@dataclass(frozen=True)
class Segment:
    """One analytic piece of one contour, in METRES."""

    name: str
    chain: str          # which contour this belongs to
    curve: str          # key into CURVES, or "custom" when law is given
    x0: float
    x1: float
    r0: float
    r1: float
    source: str         # JSON / DERIVED / ASSUMED
    detail: str = ""    # the specific key or rule
    # Closed-form r(x) for shapes that depend on another contour (the
    # annular diffuser wall depends on the centrebody).  Still analytic --
    # just not expressible as a curve law on t alone.
    law: Callable[[float], float] | None = field(
        default=None, repr=False, compare=False)

    # ---- evaluation ----------------------------------------------------
    @property
    def length(self) -> float:
        return self.x1 - self.x0

    def r_at(self, x: float) -> float:
        """Exact radius at station x (clamped to the segment)."""
        if self.length <= 0.0:
            return self.r1
        if x <= self.x0:
            return self.r0
        if x >= self.x1:
            return self.r1
        if self.law is not None:
            return self.law(x)
        if self.curve not in CURVES:
            raise KeyError(
                f"segment '{self.name}' has curve '{self.curve}', which is "
                f"neither a known curve law {sorted(CURVES)} nor accompanied "
                f"by a closed-form 'law'")
        t = (x - self.x0) / self.length
        return self.r0 + (self.r1 - self.r0) * CURVES[self.curve](t)

    def contains(self, x: float, tol: float = 1e-12) -> bool:
        return (self.x0 - tol) <= x <= (self.x1 + tol)

    def is_straight(self) -> bool:
        return self.law is None and self.curve in ("line", "cone")

    @property
    def half_angle_deg(self) -> float:
        """Cone half-angle; meaningful only for straight segments."""
        if self.length <= 0.0:
            return 90.0
        return math.degrees(math.atan2(self.r1 - self.r0, self.length))

    # ---- sampling ------------------------------------------------------
    def sample(self, curve_step: float = 0.00025) -> tuple[list[float], list[float]]:
        """Adaptive: 2 points on a straight, ~curve_step spacing on a curve.
        Endpoints are always exact."""
        if self.length <= 0.0:
            return [self.x0], [self.r0]
        if self.is_straight():
            return [self.x0, self.x1], [self.r0, self.r1]
        n = max(8, int(math.ceil(self.length / curve_step)))
        xs = [self.x0 + self.length * i / n for i in range(n + 1)]
        return xs, [self.r_at(x) for x in xs]

    # ---- integrals -----------------------------------------------------
    def volume(self, panels: int = 512) -> float:
        """Volume of revolution, integral of pi r^2 dx.  Exact frustum
        formula on straights, Simpson elsewhere."""
        if self.length <= 0.0:
            return 0.0
        if self.is_straight():
            return (math.pi * self.length / 3.0) * (
                self.r0 ** 2 + self.r0 * self.r1 + self.r1 ** 2)
        n = panels if panels % 2 == 0 else panels + 1
        h = self.length / n
        total = 0.0
        for i in range(n + 1):
            x = self.x0 + i * h
            w = 1.0 if i in (0, n) else (4.0 if i % 2 else 2.0)
            total += w * math.pi * self.r_at(x) ** 2
        return total * h / 3.0

    def lateral_area(self, panels: int = 512) -> float:
        """Wetted area of revolution, integral of 2 pi r ds."""
        if self.length <= 0.0:
            return 0.0
        if self.is_straight():
            slant = math.hypot(self.length, self.r1 - self.r0)
            return math.pi * (self.r0 + self.r1) * slant
        n = panels
        total, xp, rp = 0.0, self.x0, self.r0
        for i in range(1, n + 1):
            x = self.x0 + self.length * i / n
            r = self.r_at(x)
            total += math.pi * (r + rp) * math.hypot(x - xp, r - rp)
            xp, rp = x, r
        return total


@dataclass
class Chain:
    """An ordered, continuous list of segments forming one contour."""

    name: str
    label: str
    segments: list[Segment] = field(default_factory=list)

    @property
    def x0(self) -> float:
        return self.segments[0].x0

    @property
    def x1(self) -> float:
        return self.segments[-1].x1

    def r_at(self, x: float) -> float | None:
        if not self.segments or not (self.x0 <= x <= self.x1):
            return None
        for seg in self.segments:
            if seg.contains(x):
                return seg.r_at(x)
        return None

    def segment_at(self, x: float) -> Segment | None:
        for seg in self.segments:
            if seg.contains(x):
                return seg
        return None

    def polyline(self, curve_step: float = 0.00025
                 ) -> tuple[list[float], list[float]]:
        """Adaptive polyline for drawing.  Duplicate joint points dropped."""
        xs: list[float] = []
        rs: list[float] = []
        for seg in self.segments:
            sx, sr = seg.sample(curve_step)
            if xs and abs(sx[0] - xs[-1]) < 1e-12 and abs(sr[0] - rs[-1]) < 1e-12:
                sx, sr = sx[1:], sr[1:]
            xs.extend(sx)
            rs.extend(sr)
        return xs, rs

    def sample_uniform(self, step: float = 0.001,
                       breakpoints: Sequence[float] = ()
                       ) -> tuple[list[float], list[float]]:
        """Uniform sample -- the 'point every mm' export.  Derived, not the
        model.

        The grid does NOT land on the real breakpoints (817.307, 991.523,
        2273.607 ...), which is precisely why this is an export and not the
        representation.  Any ``breakpoints`` inside the chain are therefore
        MERGED into the grid, so the exact stations appear as rows instead of
        being straddled by two approximate ones.
        """
        n = int(math.floor((self.x1 - self.x0) / step))
        grid = [self.x0 + i * step for i in range(n + 1)]
        grid.append(self.x1)
        grid.extend(x for x in breakpoints if self.x0 < x < self.x1)

        xs: list[float] = []
        for x in sorted(grid):
            if not xs or abs(x - xs[-1]) > 1e-12:
                xs.append(x)
        return xs, [self.r_at(x) for x in xs]

    def volume(self) -> float:
        return sum(s.volume() for s in self.segments)

    def lateral_area(self) -> float:
        return sum(s.lateral_area() for s in self.segments)


@dataclass(frozen=True)
class Station:
    """A named axial breakpoint, at full precision."""

    name: str
    x: float
    source: str
    detail: str = ""


# ==========================================================================
# the vehicle
# ==========================================================================

@dataclass
class Vehicle:
    dims: dict
    assumptions: dict
    chains: dict[str, Chain]
    stations: list[Station]
    scalars: dict[str, tuple[float, str, str]]   # name -> (value, unit, source)

    # ---- convenience ---------------------------------------------------
    @property
    def d_body(self) -> float:
        return self.dims["body"]["diameter_m"]

    @property
    def a_body(self) -> float:
        return math.pi * self.d_body ** 2 / 4.0

    @property
    def x_max(self) -> float:
        return self.dims["body"]["overall_length_m"]

    def flow_radius(self, x: float) -> float | None:
        """Outer wetted radius of the duct: the cowl inner wall forward of
        the chamber head, the flowpath line aft of it."""
        r = self.chains["flow"].r_at(x)
        if r is not None:
            return r
        return self.chains["cowl_inner"].r_at(x)

    def flow_area(self, x: float) -> float | None:
        """Net through-flow area at x -- annulus where the centrebody is in
        the way, full circle elsewhere.  Strut blockage NOT included (there
        are no struts in this model)."""
        r = self.flow_radius(x)
        if r is None:
            return None
        a = math.pi * r ** 2
        cb = self.chains["centrebody"].r_at(x)
        if cb is not None:
            a -= math.pi * cb ** 2
        return max(a, 0.0)

    def region_at(self, x: float) -> str:
        for lo, hi, name in self._regions:
            if lo <= x <= hi:
                return name
        return "outside body"

    @property
    def _regions(self) -> list[tuple[float, float, str]]:
        s = {st.name: st.x for st in self.stations}
        return [
            (0.0, s["cowl_lip"], "SPIKE (external compression)"),
            (s["cowl_lip"], s["chamber_start"], "ANNULAR SUBSONIC DIFFUSER"),
            (s["chamber_start"], s["chamber_end"], "COMBUSTION CHAMBER"),
            (s["chamber_end"], s["cone_end"], "CONTRACTION CONE"),
            (s["cone_end"], s["nozzle_throat"], "TAILPIPE (= nozzle throat)"),
            (s["nozzle_throat"], s["nozzle_exit"], "DIVERGENT NOZZLE"),
        ]

    def station(self, name: str) -> float:
        for st in self.stations:
            if st.name == name:
                return st.x
        raise KeyError(name)


# ==========================================================================
# build
# ==========================================================================

def load_inputs(dims_path: Path | str = DEFAULT_DIMS,
                assumptions_path: Path | str = DEFAULT_ASSUMPTIONS
                ) -> tuple[dict, dict]:
    with open(dims_path, "r", encoding="utf-8") as fh:
        dims = json.load(fh)
    with open(assumptions_path, "r", encoding="utf-8") as fh:
        assumptions = json.load(fh)
    return dims, assumptions


def _check_wall_readings(dims: dict, walls: dict) -> None:
    """Re-derive, from the JSON, that 214 mm is an outer surface and
    115.638 mm is a flow diameter.

    These are not assumptions and not preferences -- the export states both
    facts twice, in a form that can be checked.  If a future export ever
    changes its convention this raises instead of silently drawing a vehicle
    with the wrong wall on the wrong side.
    """
    body, flow = dims["body"], dims["internal_flowpath"]

    a_frontal = math.pi * body["diameter_m"] ** 2 / 4.0
    if walls["body_diameter_is"] == "oml" and not math.isclose(
            a_frontal, body["frontal_area_m2"], rel_tol=1e-9):
        raise ValueError(
            "body.diameter_m is read as an OML, but it does not reproduce "
            f"body.frontal_area_m2 ({a_frontal:.9f} vs "
            f"{body['frontal_area_m2']:.9f})")

    frac = (flow["tailpipe_diameter_m"] / body["diameter_m"]) ** 2
    if walls["tailpipe_diameter_is"] == "flow_id" and not math.isclose(
            frac, flow["throat_to_body_area_fraction"], rel_tol=1e-9):
        raise ValueError(
            "internal_flowpath.tailpipe_diameter_m is read as a FLOW "
            "diameter, but it does not reproduce "
            f"throat_to_body_area_fraction ({frac:.9f} vs "
            f"{flow['throat_to_body_area_fraction']:.9f})")


def build_vehicle(dims: dict, assumptions: dict) -> Vehicle:
    """Assemble every contour from the JSON + the assumptions file."""
    body = dims["body"]
    st_in = dims["stations_from_nose_tip_m"]
    flow = dims["internal_flowpath"]
    mat = dims["materials_and_gauges"]
    inl = assumptions["inlet"]
    walls = assumptions["walls"]

    # ---- given, exactly ------------------------------------------------
    d_body = body["diameter_m"]
    r_body = 0.5 * d_body                       # OML: frontal_area proves it
    x_nose_end = st_in["nose_fairing_end__barrel_start"]
    x_cham0 = st_in["combustion_chamber_start"]
    x_cham1 = st_in["combustion_chamber_end__tailpipe_start"]
    x_pipe1 = st_in["tailpipe_end"]
    x_exit = st_in["nozzle_exit__body_end"]

    t_skin = mat["skin_gauge_m"]                # 1.5 mm CFRP
    t_duct = mat["duct_wall_thickness_m"]       # 1.0 mm steel

    # How the two source diameters are read.  Neither is a free choice --
    # the data proves each one, and they disagree with each other:
    #   frontal_area_m2 == pi/4 * 0.214^2          -> 214 mm is an OML
    #   throat_to_body_area_fraction == (115.638/214)^2
    #                                              -> 115.638 mm is a FLOW dia
    # _check_wall_readings() re-verifies both against the JSON at build time,
    # so this cannot rot if the export changes.
    _check_wall_readings(dims, walls)

    d_throat = flow["tailpipe_diameter_m"]       # a FLOW diameter
    r_pipe = 0.5 * d_throat
    r_pipe_od = r_pipe + t_duct

    # chamber: 214 is the OML, so the flow ID is inboard of the full stack
    r_cham = r_body - (t_skin + t_duct)
    r_skin_id = r_body - t_skin

    # ---- centrebody / inlet (ASSUMED) ----------------------------------
    cb_half = inl["centrebody_cone_half_angle_deg"]
    r_cb = 0.5 * inl["centrebody_max_diameter_frac_of_body"] * d_body
    L_spike = r_cb / math.tan(math.radians(cb_half))
    x_lip = L_spike                              # lip in the shoulder plane
    L_cb_cyl = inl["centrebody_cylinder_length_m"]
    x_cb_cyl_end = x_lip + L_cb_cyl

    # Capture rule.  The centrebody is the DESIGN VARIABLE (sized for
    # avionics, user directive) and the lip is SOLVED from it:
    #     pi (r_lip^2 - r_cb^2) = A_capture   ->   r_lip = sqrt(r_cb^2 + A/pi)
    rule = inl["capture_area_rule"]
    if rule == "annulus_equals_throat_area":
        # the original rule: match the nozzle throat.  The flight model has
        # since shown this oversizes the inlet 2.67x -- kept only so the
        # earlier drawing can be reproduced.
        a_capture = math.pi * r_pipe ** 2
    elif rule == "annulus_equals_required_mass_flow":
        if "inlet" not in dims:
            raise ValueError(
                "capture_area_rule 'annulus_equals_required_mass_flow' needs "
                "an 'inlet' block in structural_dimensions.json (added by the "
                "flight model 2026-08-14); re-pull the export")
        a_capture = dims["inlet"]["implied_capture_area_m2"]
    else:
        raise ValueError(
            f"unknown inlet.capture_area_rule {rule!r}; implemented rules are "
            "'annulus_equals_required_mass_flow' and "
            "'annulus_equals_throat_area'")
    r_lip = math.sqrt(r_cb ** 2 + a_capture / math.pi)
    t_lip = inl["cowl_lip_thickness_m"]
    if r_lip + t_lip >= r_body:
        raise ValueError(
            f"cowl lip OD {2 * (r_lip + t_lip) * MM:.1f} mm does not fit "
            f"inside the {d_body * MM:.1f} mm body")

    # ---- chamber -> tailpipe cone (ASSUMED angle, DERIVED length) -------
    cone_half = assumptions["chamber_to_tailpipe_cone"]["half_angle_deg"]
    L_cone = (r_cham - r_pipe) / math.tan(math.radians(cone_half))
    x_cone1 = x_cham1 + L_cone

    # ---- divergent nozzle (ASSUMED) -------------------------------------
    # The model has no divergence: its nozzle_exit_diameter_m equals the
    # tailpipe diameter, so what it calls the exit is really the THROAT.
    # Expansion is therefore added AFT of the throat and the throat itself
    # never moves -- throat area, throat_to_body_area_fraction and the inlet
    # capture rule are all untouched by anything in this block.
    noz = assumptions["divergent_nozzle"]
    exp_ar = noz["expansion_area_ratio"]
    div_half = noz["half_angle_deg"]
    if exp_ar < 1.0:
        raise ValueError(
            f"divergent_nozzle.expansion_area_ratio is {exp_ar}, i.e. a "
            "CONTRACTION aft of the throat; it must be >= 1.0")
    r_exit = r_pipe * math.sqrt(exp_ar)
    L_div = ((r_exit - r_pipe) / math.tan(math.radians(div_half))
             if exp_ar > 1.0 else 0.0)
    x_throat = x_exit - L_div
    if x_throat <= x_cone1:
        raise ValueError(
            f"the divergent section ({L_div * MM:.1f} mm at "
            f"{div_half:.1f} deg) is longer than the tailpipe; reduce "
            "expansion_area_ratio or open half_angle_deg")
    if r_exit + t_duct > r_body:
        raise ValueError(
            f"expansion_area_ratio {exp_ar} gives an exit OD of "
            f"{2 * (r_exit + t_duct) * MM:.1f} mm, wider than the "
            f"{d_body * MM:.1f} mm body")

    # ---- boattail (DERIVED from the JSON's length and base diameter) ----
    # Corrected export, 2026-08-14.  The old field echoed the nozzle FLOW
    # diameter, which the flight model never used as an outer mould line and
    # which left no room for the duct wall.  The real aft body closes at
    # exactly 8.0 deg to 153.849 mm and does NOT close onto the nozzle: an
    # annular base remains, and base drag on it is a flown term.  The max()
    # is a containment guard only -- it no longer binds.
    r_base = max(0.5 * body["boattail_exit_diameter_m"], r_exit + t_duct)
    bt_half = math.degrees(math.atan2(r_body - r_base, x_exit - x_pipe1))

    def seg(**kw) -> Segment:
        return Segment(**kw)

    # ---- chain: centrebody ---------------------------------------------
    centrebody = Chain("centrebody", "centrebody (ASSUMED)", [
        seg(name="spike cone", chain="centrebody", curve="cone",
            x0=0.0, x1=x_lip, r0=0.0, r1=r_cb, source=ASSUMED,
            detail=f"{cb_half:.1f} deg half-angle cone"),
        seg(name="centrebody barrel", chain="centrebody", curve="line",
            x0=x_lip, x1=x_cb_cyl_end, r0=r_cb, r1=r_cb, source=ASSUMED,
            detail="payload / avionics volume"),
        seg(name="centrebody closure", chain="centrebody", curve="smoothstep",
            x0=x_cb_cyl_end, x1=x_cham0, r0=r_cb, r1=0.0, source=ASSUMED,
            detail="closes out at the chamber head"),
    ])

    def centrebody_radius(x: float) -> float:
        r = centrebody.r_at(x)
        return 0.0 if r is None else r

    # ---- chain: cowl inner (the annular diffuser outer wall) ------------
    # "constant_area_angle" shapes the wall so the EQUIVALENT-CONE radius
    # sqrt(A/pi) grows linearly, i.e. a uniform equivalent diffuser angle
    # over the whole intake.  Because the duct is annular, the wall depends
    # on the centrebody:  r_cowl(x) = sqrt(R_eq(x)^2 + r_cb(x)^2).
    # Both endpoints fall out exactly right, so this is a drop-in for the
    # simple t-laws and strictly better behaved than any of them.
    #
    # Diffuser policy.  With the inlet correctly sized to the engine's real
    # air demand the area ratio to the chamber head is 8.7, which NO wall
    # shape can diffuse gently over a 275 mm nose -- it works out at a 14.1
    # deg equivalent cone, far past separation.  The physical answer is the
    # one a ramjet with a flameholder uses anyway: diffuse at a sane angle as
    # far as the length allows, then DUMP into the combustor through a
    # sudden expansion.  'full' keeps the old behaviour for comparison.
    cowl_curve = inl["cowl_inner_curve"]
    policy = inl["diffuser_policy"]
    target = inl["diffuser_target_half_angle_deg"]
    _L = x_cham0 - x_lip
    _R0 = math.sqrt(a_capture / math.pi)          # equivalent capture radius

    if policy == "full":
        _R1 = r_cham                               # diffuse the whole way
    elif policy == "constant_angle_then_dump":
        _R1 = min(_R0 + _L * math.tan(math.radians(target)), r_cham)
    else:
        raise ValueError(
            f"unknown inlet.diffuser_policy {policy!r}; implemented policies "
            "are 'constant_angle_then_dump' and 'full'")
    r_dump = _R1                                   # cowl inner at the head
    dump_area_ratio = (r_cham / r_dump) ** 2

    cowl_law = None
    if cowl_curve == "constant_area_angle":
        def cowl_law(x: float, _R0=_R0, _R1=_R1, _L=_L, _x0=x_lip) -> float:
            t = (x - _x0) / _L
            r_eq = _R0 + (_R1 - _R0) * t
            return math.sqrt(r_eq ** 2 + centrebody_radius(x) ** 2)

    cowl_detail = (
        f"{cowl_curve}: r(x) = sqrt(R_eq(x)^2 + r_cb(x)^2), R_eq linear "
        f"{_R0 * MM:.3f} -> {_R1 * MM:.3f} mm over {_L * MM:.3f} mm"
        if cowl_law else f"{cowl_curve} lip -> chamber head")
    cowl_inner = Chain("cowl_inner", "cowl inner / diffuser (ASSUMED)", [
        seg(name="diffuser", chain="cowl_inner", curve=cowl_curve,
            x0=x_lip, x1=x_cham0, r0=r_lip, r1=r_dump, source=ASSUMED,
            detail=cowl_detail, law=cowl_law),
    ])

    # ---- chain: flow (chamber head -> nozzle exit) ----------------------
    flow_chain = Chain("flow", "flowpath wetted line", [
        seg(name="chamber", chain="flow", curve="line",
            x0=x_cham0, x1=x_cham1, r0=r_cham, r1=r_cham, source=DERIVED,
            detail="body OML minus skin+duct stackup"),
        seg(name="contraction cone", chain="flow", curve="cone",
            x0=x_cham1, x1=x_cone1, r0=r_cham, r1=r_pipe, source=ASSUMED,
            detail=f"{cone_half:.1f} deg half-angle (model gives it ZERO length)"),
        seg(name="tailpipe", chain="flow", curve="line",
            x0=x_cone1, x1=x_throat, r0=r_pipe, r1=r_pipe, source=JSON,
            detail="dia is JSON (internal_flowpath.tailpipe_diameter_m) and "
                   "IS the nozzle throat; its START station follows the "
                   "ASSUMED cone and its end the ASSUMED divergence"),
        seg(name="divergent nozzle", chain="flow", curve="cone",
            x0=x_throat, x1=x_exit, r0=r_pipe, r1=r_exit, source=ASSUMED,
            detail=f"area ratio {exp_ar:.3f} at {div_half:.1f} deg half-angle "
                   f"(the model has NO divergence)"),
    ])

    # the same line, offset outboard by the duct wall
    duct_od = Chain("duct_od", "duct outer wall (steel)", [
        seg(name="chamber liner OD", chain="duct_od", curve="line",
            x0=x_cham0, x1=x_cham1, r0=r_cham + t_duct, r1=r_cham + t_duct,
            source=DERIVED, detail=f"+{t_duct * MM:.1f} mm steel"),
        seg(name="cone OD", chain="duct_od", curve="cone",
            x0=x_cham1, x1=x_cone1, r0=r_cham + t_duct, r1=r_pipe_od,
            source=DERIVED, detail=f"+{t_duct * MM:.1f} mm steel"),
        seg(name="tailpipe OD", chain="duct_od", curve="line",
            x0=x_cone1, x1=x_throat, r0=r_pipe_od, r1=r_pipe_od,
            source=DERIVED, detail=f"+{t_duct * MM:.1f} mm steel"),
        seg(name="divergent nozzle OD", chain="duct_od", curve="cone",
            x0=x_throat, x1=x_exit, r0=r_pipe_od, r1=r_exit + t_duct,
            source=DERIVED, detail=f"+{t_duct * MM:.1f} mm steel"),
    ])

    # ---- chain: OML aft of the lip -------------------------------------
    oml = Chain("oml", "outer mould line", [
        seg(name="cowl outer", chain="oml", curve=inl["cowl_outer_curve"],
            x0=x_lip, x1=x_nose_end, r0=r_lip + t_lip, r1=r_body,
            source=ASSUMED,
            detail="length is JSON (nose_fairing_length_m); profile is not"),
        seg(name="barrel", chain="oml", curve="line",
            x0=x_nose_end, x1=x_pipe1, r0=r_body, r1=r_body, source=JSON,
            detail="body.constant_diameter_barrel_length_m"),
        seg(name="boattail", chain="oml", curve="cone",
            x0=x_pipe1, x1=x_exit, r0=r_body, r1=r_base, source=DERIVED,
            detail=f"{bt_half:.2f} deg half-angle from JSON length + exit dia"),
    ])

    # skin inner surface, for the fuel annulus
    skin_id = Chain("skin_id", "skin inner surface", [
        seg(name="skin ID barrel", chain="skin_id", curve="line",
            x0=x_nose_end, x1=x_pipe1, r0=r_skin_id, r1=r_skin_id,
            source=DERIVED, detail=f"-{t_skin * MM:.1f} mm CFRP"),
        # the skin lands ON the duct at the base -- it cannot pass inside it
        seg(name="skin ID boattail", chain="skin_id", curve="cone",
            x0=x_pipe1, x1=x_exit, r0=r_skin_id,
            r1=max(r_base - t_skin, r_exit + t_duct), source=DERIVED,
            detail=f"-{t_skin * MM:.1f} mm CFRP, clamped to the duct OD"),
    ])

    chains = {c.name: c for c in
              (centrebody, cowl_inner, flow_chain, duct_od, oml, skin_id)}

    # ---- stations -------------------------------------------------------
    stations = [
        Station("nose_tip", 0.0, JSON, "stations_from_nose_tip_m.nose_tip"),
        Station("cowl_lip", x_lip, ASSUMED,
                "spike shoulder plane = capture plane"),
        Station("centrebody_max_dia_end", x_cb_cyl_end, ASSUMED,
                "end of the centrebody barrel"),
        Station("nose_fairing_end", x_nose_end, JSON,
                "nose_fairing_end__barrel_start"),
        Station("chamber_start", x_cham0, JSON, "combustion_chamber_start"),
        Station("chamber_end", x_cham1, JSON,
                "combustion_chamber_end__tailpipe_start"),
        Station("cone_end", x_cone1, ASSUMED,
                f"chamber_end + {L_cone * MM:.1f} mm at {cone_half:.1f} deg"),
        Station("tailpipe_end", x_pipe1, JSON, "tailpipe_end / boattail_start"),
        Station("nozzle_throat", x_throat, ASSUMED,
                f"nozzle_exit - {L_div * MM:.1f} mm of divergence "
                f"(area ratio {exp_ar:.3f} at {div_half:.1f} deg)"),
        Station("nozzle_exit", x_exit, JSON, "nozzle_exit__body_end"),
    ]
    stations.sort(key=lambda s: s.x)

    # ---- scalars worth carrying ----------------------------------------
    a_throat = math.pi * r_pipe ** 2
    a_cham_flow = math.pi * r_cham ** 2
    a_base = math.pi * (r_base ** 2 - (r_exit + t_duct) ** 2)
    v_cb = centrebody.volume()
    scalars = {
        "body diameter (OML)": (d_body * MM, "mm", JSON),
        "overall length": (x_exit * MM, "mm", JSON),
        "fineness ratio": (x_exit / d_body, "-", JSON),
        "spike half-angle": (cb_half, "deg", ASSUMED),
        "spike length (tip -> lip plane)": (L_spike * MM, "mm", ASSUMED),
        "centrebody max diameter": (2 * r_cb * MM, "mm", ASSUMED),
        "centrebody volume (avionics)": (v_cb * 1e3, "L", ASSUMED),
        "cowl lip diameter": (2 * r_lip * MM, "mm", ASSUMED),
        "annular capture area": (a_capture * 1e4, "cm2",
                                 JSON if rule.endswith("mass_flow") else ASSUMED),
        "throat area (model)": (a_throat * 1e4, "cm2", JSON),
        "capture / throat area": (a_capture / a_throat, "-", DERIVED),
        "chamber flow diameter": (2 * r_cham * MM, "mm", DERIVED),
        "chamber flow area": (a_cham_flow * 1e4, "cm2", DERIVED),
        "diffuser area ratio (chamber/capture)":
            (a_cham_flow / a_capture, "-", DERIVED),
        "diffuser exit dia (dump plane)": (2 * r_dump * MM, "mm", ASSUMED),
        "dump sudden-expansion area ratio": (dump_area_ratio, "-", ASSUMED),
        "annular base area (as drawn)": (a_base * 1e4, "cm2", DERIVED),
        "annular base area (export, engine on)":
            (body.get("annular_base_area_engine_on_m2", float("nan")) * 1e4,
             "cm2", JSON),
        "contraction cone half-angle": (cone_half, "deg", ASSUMED),
        "contraction cone length": (L_cone * MM, "mm", ASSUMED),
        "tailpipe length (as drawn)": ((x_exit - x_cone1) * MM, "mm", DERIVED),
        "tailpipe length (model)": (flow["tailpipe_length_m"] * MM, "mm", JSON),
        "nozzle throat diameter": (2 * r_pipe * MM, "mm", JSON),
        "nozzle expansion area ratio": (exp_ar, "-", ASSUMED),
        "nozzle exit flow diameter": (2 * r_exit * MM, "mm", ASSUMED),
        "divergent half-angle": (div_half, "deg", ASSUMED),
        "divergent length": (L_div * MM, "mm", ASSUMED),
        "boattail half-angle": (bt_half, "deg", DERIVED),
        "boattail base OD": (2 * r_base * MM, "mm", JSON),
        "throat / body area fraction": ((d_throat / d_body) ** 2, "-", JSON),
    }

    return Vehicle(dims=dims, assumptions=assumptions, chains=chains,
                   stations=stations, scalars=scalars)


# ==========================================================================
# fuel annulus + model-vs-as-drawn reconciliation
# ==========================================================================

def annulus_volume(v: Vehicle, x0: float, x1: float, panels: int = 2000
                   ) -> float:
    """True volume between the duct OD and the skin ID over [x0, x1]."""
    duct, skin = v.chains["duct_od"], v.chains["skin_id"]
    h = (x1 - x0) / panels
    total = 0.0
    for i in range(panels):
        x = x0 + (i + 0.5) * h
        ro = skin.r_at(x)
        ri = duct.r_at(x)
        if ro is None or ri is None:
            continue
        total += math.pi * max(ro ** 2 - ri ** 2, 0.0) * h
    return total


def equivalent_radius(area: float) -> float:
    """Radius of the circle with the same area -- the standard way to talk
    about an ANNULAR duct in conical-diffuser terms."""
    return math.sqrt(max(area, 0.0) / math.pi)


def diffuser_diagnostics(v: Vehicle, panels: int = 800) -> dict:
    """Equivalent-conical-diffuser angle through the annular intake.

    The separation rule of thumb for a subsonic diffuser is an equivalent
    CONE half-angle of about 7 deg; past ~10 deg you should expect separation
    and a total-pressure penalty the model does not carry.  This is geometry
    only -- there is no boundary-layer calculation anywhere in this repo.
    """
    x0 = v.station("cowl_lip")
    x1 = v.station("chamber_start")

    def diffuser_area(x: float) -> float:
        """Annulus between the cowl inner wall and the centrebody.  NOT
        ``flow_area``: that dispatches to the chamber at x1, so sampling it
        across the dump plane reports the DUMP as if it were diffuser wall
        angle (an 89 deg spike in the last panel)."""
        r_o = v.chains["cowl_inner"].r_at(x)
        r_i = v.chains["centrebody"].r_at(x) or 0.0
        return math.pi * max(r_o ** 2 - r_i ** 2, 0.0)

    a0, a1 = diffuser_area(x0), diffuser_area(x1)

    worst_deg, worst_x = 0.0, x0
    h = (x1 - x0) / panels
    for i in range(panels):
        xa, xb = x0 + i * h, x0 + (i + 1) * h
        deg = math.degrees(math.atan2(
            equivalent_radius(diffuser_area(xb))
            - equivalent_radius(diffuser_area(xa)), h))
        if deg > worst_deg:
            worst_deg, worst_x = deg, 0.5 * (xa + xb)

    d_req = equivalent_radius(a1) - equivalent_radius(a0)
    mean_deg = math.degrees(math.atan2(d_req, x1 - x0))

    # With both areas fixed, length is the ONLY lever left on the angle.
    # Report what it would take to reach the rule-of-thumb limit, so the
    # trade is a number rather than a shrug.  Under the dump policy the
    # diffuser already sits AT the target and the leftover area ratio is
    # taken by the dump instead, so this is the 'full' policy's question.
    target = v.assumptions["inlet"]["diffuser_target_half_angle_deg"]
    a_head = math.pi * v.chains["flow"].segments[0].r0 ** 2
    d_full = equivalent_radius(a_head) - equivalent_radius(a0)
    l_for_target = d_full / math.tan(math.radians(target))

    return {
        "policy": v.assumptions["inlet"]["diffuser_policy"],
        "length_m": x1 - x0,
        "area_in_m2": a0,
        "area_out_m2": a1,
        "area_ratio": a1 / a0,
        "area_ratio_to_chamber_head": a_head / a0,
        "dump_area_ratio": a_head / a1,
        "mean_half_angle_deg": mean_deg,
        "max_half_angle_deg": worst_deg,
        "max_half_angle_x_m": worst_x,
        "target_half_angle_deg": target,
        "length_for_target_m": l_for_target,
        "extra_length_for_target_m": l_for_target - (x1 - x0),
    }


# --------------------------------------------------------------------------
# quasi-1D nozzle relations (isentropic, calorically perfect)
# --------------------------------------------------------------------------

def _p0_over_p(mach: float, gamma: float) -> float:
    return (1.0 + 0.5 * (gamma - 1.0) * mach * mach) ** (gamma / (gamma - 1.0))


def _area_ratio(mach: float, gamma: float) -> float:
    """A/A* for isentropic flow."""
    g = gamma
    return (1.0 / mach) * ((2.0 / (g + 1.0))
                           * (1.0 + 0.5 * (g - 1.0) * mach * mach)
                           ) ** ((g + 1.0) / (2.0 * (g - 1.0)))


def _mach_from_area_ratio(ar: float, gamma: float) -> float:
    """Supersonic branch of A/A* = ar, by bisection."""
    if ar <= 1.0:
        return 1.0
    lo, hi = 1.0, 10.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if _area_ratio(mid, gamma) < ar:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def nozzle_diagnostics(v: Vehicle) -> dict:
    """Is the divergent section doing anything, and is it the right size?

    Quasi-1D and isentropic: no boundary layer, no heat loss, no unsteadiness
    (which for a PULSEJET is a real omission -- its cycle is unsteady by
    construction, and a fixed divergence cannot be right through a cycle).
    Good enough to size a cone; not a performance prediction.
    """
    noz = v.assumptions["divergent_nozzle"]
    g = noz["gamma_hot"]
    mach = noz["design_mach"]
    rec = noz["total_pressure_recovery"]
    ar = noz["expansion_area_ratio"]

    # freestream stagnation uses cold-air gamma; the expansion uses hot gamma
    p0_over_pa = rec * _p0_over_p(mach, 1.4)
    choke_pr = ((g + 1.0) / 2.0) ** (g / (g - 1.0))
    choked = p0_over_pa >= choke_pr

    # flight Mach at which the nozzle first chokes
    lo, hi = 0.05, 5.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if rec * _p0_over_p(mid, 1.4) < choke_pr:
            lo = mid
        else:
            hi = mid
    choke_mach = 0.5 * (lo + hi)

    out = {
        "gamma": g, "design_mach": mach, "recovery": rec,
        "area_ratio": ar,
        "p0_over_pa": p0_over_pa,
        "choke_pressure_ratio": choke_pr,
        "choked": choked,
        "choking_mach": choke_mach,
        "optimum_area_ratio": None,
        "exit_mach": None,
        "pe_over_pa": None,
    }
    if choked:
        me_opt = math.sqrt(
            (p0_over_pa ** ((g - 1.0) / g) - 1.0) * 2.0 / (g - 1.0))
        out["optimum_area_ratio"] = _area_ratio(me_opt, g)
        me = _mach_from_area_ratio(ar, g)
        out["exit_mach"] = me
        out["pe_over_pa"] = p0_over_pa / _p0_over_p(me, g)
    return out


def expansion_trade(v: Vehicle,
                    ratios: Sequence[float] = (1.0, 1.05, 1.10, 1.20, 1.40,
                                               1.724)) -> list[dict]:
    """Area ratio -> exit diameter, divergent length, and the ANNULAR BASE
    AREA it leaves.

    The boattail angle used to be the column here, on the theory that a
    bigger exit let the aft body close less steeply.  The corrected export
    (2026-08-14) killed that: the boattail is fixed at 8.0 deg to a 153.849
    mm base and does not close onto the nozzle at all, so expansion cannot
    move it.  What expansion DOES do is eat into the annular base -- and
    base drag on that annulus is a real term in the flown build-up.
    """
    body = v.dims["body"]
    noz = v.assumptions["divergent_nozzle"]
    r_base = 0.5 * body["boattail_exit_diameter_m"]
    r_th = 0.5 * v.dims["internal_flowpath"]["tailpipe_diameter_m"]
    t_duct = v.dims["materials_and_gauges"]["duct_wall_thickness_m"]
    half = math.radians(noz["half_angle_deg"])
    a_base0 = math.pi * (r_base ** 2 - (r_th + t_duct) ** 2)

    rows = []
    for ar in ratios:
        r_e = r_th * math.sqrt(ar)
        a_base = math.pi * (r_base ** 2 - (r_e + t_duct) ** 2)
        rows.append({
            "area_ratio": ar,
            "exit_diameter_m": 2 * r_e,
            "divergent_length_m": (r_e - r_th) / math.tan(half),
            "base_area_m2": a_base,
            "base_area_change_frac": a_base / a_base0 - 1.0,
            "fits": r_e + t_duct <= r_base,
        })
    return rows


def reconciliation(v: Vehicle) -> list[tuple[str, float, float, str, str]]:
    """(quantity, model value, as-drawn value, unit, why they differ)."""
    flow = v.dims["internal_flowpath"]
    fuel = v.dims["fuel_system"]
    x_cham0 = v.station("chamber_start")
    x_cham1 = v.station("chamber_end")
    x_pipe1 = v.station("tailpipe_end")
    x_exit = v.station("nozzle_exit")

    cham_seg = v.chains["flow"].segments[0]
    v_cham = cham_seg.volume()

    v_ann_barrel = annulus_volume(v, x_cham1, x_pipe1)
    v_ann_boattail = annulus_volume(v, x_pipe1, x_exit)

    frac = fuel["usable_volume_fraction_of_annulus"]
    rho = fuel["fuel_density_kg_m3"]

    rows = [
        ("chamber flow diameter", flow["chamber_diameter_m"] * MM,
         2 * cham_seg.r0 * MM, "mm",
         "model quotes the OML; as-drawn subtracts the 1.5 mm skin + 1.0 mm liner"),
        ("chamber volume", flow["chamber_volume_m3"] * 1e3,
         v_cham * 1e3, "L", "same cause"),
        ("tailpipe length", flow["tailpipe_length_m"] * MM,
         (x_exit - v.station("cone_end")) * MM, "mm",
         "as-drawn gives the contraction cone a real length and runs the pipe "
         "to the exit plane"),
        ("fuel annulus volume", fuel["annulus_volume_m3"] * 1e3,
         v_ann_barrel * 1e3, "L",
         "the cone is fatter than the pipe, and true walls shrink the gap"),
        ("fuel capacity", fuel["tank_capacity_kg"],
         frac * v_ann_barrel * rho, "kg",
         f"{frac:.0%} of the annulus, same rule as the model"),
        ("nozzle exit flow diameter",
         v.dims["internal_flowpath"]["nozzle_exit_diameter_m"] * MM,
         2 * v.chains["flow"].segments[-1].r1 * MM, "mm",
         "the model has NO divergence -- what it calls the nozzle exit is "
         "the THROAT, which is unchanged; this is the added expansion"),
        ("annular base area",
         v.dims["body"].get("annular_base_area_engine_on_m2", 0.0) * 1e4,
         v.scalars["annular base area (as drawn)"][0], "cm2",
         "the export's base annulus assumes no nozzle expansion; the "
         "divergent exit eats into it, which REDUCES base drag"),
        ("fuel loaded", fuel["fuel_loaded_kg"], fuel["fuel_loaded_kg"], "kg",
         "unchanged -- an input, not a geometry output"),
        ("spare annulus aft of the barrel", 0.0, v_ann_boattail * 1e3, "L",
         "the model's tank stops at the boattail start; this volume exists "
         "but is not counted"),
    ]
    return rows


# ==========================================================================
# exports
# ==========================================================================

def export_stations_csv(v: Vehicle, path: Path) -> None:
    lines = ["station,x_m,x_mm,x_in,source,detail"]
    for st in v.stations:
        lines.append(f"{st.name},{st.x!r},{st.x * MM!r},"
                     f"{st.x / 0.0254!r},{st.source},\"{st.detail}\"")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _station_names_by_x(v: Vehicle) -> dict[float, str]:
    """Stations keyed by rounded x.  Two stations can share an x (the nose
    fairing ends exactly where the chamber starts), so names are JOINED --
    keying by a plain dict would silently drop one of them."""
    out: dict[float, list[str]] = {}
    for st in v.stations:
        out.setdefault(round(st.x, 9), []).append(st.name)
    return {k: "|".join(names) for k, names in out.items()}


def export_contour_csv(v: Vehicle, path: Path, step: float = 0.001) -> None:
    """The 'point every mm' export, with the exact stations merged in."""
    breaks = [st.x for st in v.stations]
    by_x = _station_names_by_x(v)
    lines = ["chain,x_mm,r_mm,diameter_mm,segment,source,station"]
    for name in ("oml", "flow", "duct_od", "skin_id", "centrebody",
                 "cowl_inner"):
        ch = v.chains[name]
        xs, rs = ch.sample_uniform(step, breaks)
        for x, r in zip(xs, rs):
            seg = ch.segment_at(x)
            lines.append(
                f"{name},{x * MM:.4f},{r * MM:.5f},{2 * r * MM:.5f},"
                f"{seg.name if seg else ''},{seg.source if seg else ''},"
                f"{by_x.get(round(x, 9), '')}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_area_csv(v: Vehicle, path: Path, step: float = 0.001) -> None:
    """Flow area distribution -- the number an engine person actually wants.

    The named stations are merged into the grid, so the area at exactly
    817.307 mm is a row rather than something to interpolate for.
    """
    a_body = v.a_body
    a_th = math.pi * (
        0.5 * v.dims["internal_flowpath"]["tailpipe_diameter_m"]) ** 2
    by_x = _station_names_by_x(v)

    n = int(math.floor(v.x_max / step))
    grid = [i * step for i in range(n + 1)] + [v.x_max]
    grid.extend(st.x for st in v.stations)
    xs: list[float] = []
    for x in sorted(grid):
        if not xs or abs(x - xs[-1]) > 1e-12:
            xs.append(x)

    lines = ["x_mm,region,flow_radius_mm,flow_area_cm2,area_over_body,"
             "area_over_throat,equivalent_radius_mm,station"]
    for x in xs:
        a, r = v.flow_area(x), v.flow_radius(x)
        if a is None or r is None:
            continue
        lines.append(
            f"{x * MM:.4f},{v.region_at(x)},{r * MM:.5f},{a * 1e4:.5f},"
            f"{a / a_body:.6f},{a / a_th:.6f},"
            f"{equivalent_radius(a) * MM:.5f},{by_x.get(round(x, 9), '')}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_geometry_json(v: Vehicle, path: Path) -> None:
    """The segment model itself, for other tools to consume.

    Self-describing rather than re-loadable: a segment shaped by a closed-form
    ``law`` (the annular diffuser) stores the law's NAME and its constants in
    ``detail``, not a serialised function.  Rebuilding from the JSON means
    re-applying the documented formula.  Rebuilding from the source is cheaper
    anyway -- ``build_vehicle`` takes well under a second.
    """
    payload = {
        "provenance": {
            "dims_source": str(DEFAULT_DIMS.relative_to(_REPO)).replace("\\", "/"),
            "git_sha": v.dims["provenance"]["git_sha"],
            "units": "metres in 'segments'; the viewer works in mm",
        },
        "stations": [
            {"name": s.name, "x_m": s.x, "source": s.source, "detail": s.detail}
            for s in v.stations
        ],
        "chains": {
            name: {
                "label": ch.label,
                "segments": [
                    {"name": s.name, "curve": s.curve, "x0_m": s.x0,
                     "x1_m": s.x1, "r0_m": s.r0, "r1_m": s.r1,
                     "source": s.source, "detail": s.detail}
                    for s in ch.segments
                ],
            }
            for name, ch in v.chains.items()
        },
        "scalars": {k: {"value": val, "unit": u, "source": src}
                    for k, (val, u, src) in v.scalars.items()},
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
