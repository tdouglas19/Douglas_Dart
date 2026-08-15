"""To-scale 2D longitudinal cross-section of the combined-cycle dart.

Draws a side-view section (mirrored about the centreline) of a frozen
medium_model design.json: outer mould line, the shared pulsejet/ramjet
flowpath inside it, the reed-valve and flameholder stations, the fuel
annulus, and the lifting-surface stations -- every feature dimensioned
with the number the model actually uses.

Nothing here is a new geometric assumption where the model already has
one.  Every station is derived from:

  * ``design.json``            -- diameter, throat diameter, chamber and
                                 tail-section lengths, wing concept
  * ``medium_model.constants`` -- NOSE_LENGTH_DIAMETERS (2.0),
                                 TAIL_LENGTH_DIAMETERS (1.0), gauges
  * ``medium_model.fp_spec``   -- chamber diameter 0.95 D, cone fraction
                                 0.130/0.750 of the tail section,
                                 flameholder gutter radius/width, x_fh
  * ``medium_model.drag_buildup`` -- 8 deg boattail -> base diameter
  * ``medium_model.mission``   -- fuel annulus volume and usable fraction

The three places a POSITION had to be invented (the model carries no
longitudinal station for them) are drawn in outline only and labelled
"ASSUMED": the internal diffuser shape between lip and chamber head, the
wing root station, and the fin root station.  Everything else is the
model's own geometry.

Usage
-----
    MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_cross_section.py [--design PATH] [--name TAG]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")                                   # noqa: E402
import matplotlib.pyplot as plt                         # noqa: E402
from matplotlib.patches import Polygon, Rectangle       # noqa: E402

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from medium_model import constants as C                 # noqa: E402
from medium_model import mission                        # noqa: E402
from medium_model.design import load_frozen_design      # noqa: E402
from medium_model.drag_buildup import (BOATTAIL_HALF_ANGLE_DEG,  # noqa: E402
                                       base_diameter_m)
from medium_model.fp_spec import spec_from_geometry     # noqa: E402
from medium_model.mass_model import duct_wall_thickness_m  # noqa: E402

DEFAULT_DESIGN = _REPO / "docs" / "v3a_medium_model" / "design.json"
OUT_DIR = _REPO / "out_medium_model"

# --- greyscale-safe palette (printed mono, this must still read) ----------
INK = "#101010"
SKIN_FILL = "#e8e8e8"      # structure between skin and duct
GAS_FILL = "#ffffff"       # the flowpath itself
CHAMBER_FILL = "#c9c9c9"   # combustion volume (pulsejet chamber + gutter)
FUEL_FILL = "#b4b4b4"
ACCENT = "#404040"
ASSUMED = "#7a7a7a"


# ==========================================================================
# geometry
# ==========================================================================

def build_stations(design):
    """Every axial station and radius the drawing needs, in metres,
    measured from the inlet lip plane (x = 0)."""
    g = design.geometry
    spec = spec_from_geometry(g)
    pj = spec.pulsejet_geometry()

    d = g.diameter_m
    r_body = 0.5 * d
    r_lip = 0.5 * g.throat_diameter_m
    r_cham = 0.5 * pj.chamber_diameter          # 0.95 D
    r_pipe = 0.5 * pj.tailpipe_diameter         # = throat diameter

    nose_len = C.NOSE_LENGTH_DIAMETERS * d
    tail_len = C.TAIL_LENGTH_DIAMETERS * d
    duct_len = g.chamber_length_m + g.throat_length_m
    overall = duct_len + C.NOSE_TAIL_LENGTH_DIAMETERS * d

    x_duct0 = nose_len                                  # chamber head
    x_cham1 = x_duct0 + g.chamber_length_m              # cone start
    x_cone1 = x_cham1 + pj.cone_length                  # cone end
    x_exit = x_cham1 + g.throat_length_m                # nozzle exit plane
    x_base = overall                                    # boattail base

    r_base = 0.5 * base_diameter_m(d, tail_len, g.throat_diameter_m)

    fh = spec.ramjet_flameholder()
    rj = spec.ramjet_geometry()
    x_fh = x_duct0 + fh.x_fh
    r_gutter = 0.060 / 0.190 * spec.combustor_diameter_m

    return dict(
        spec=spec, pj=pj, fh=fh, rj=rj,
        d=d, r_body=r_body, r_lip=r_lip, r_cham=r_cham, r_pipe=r_pipe,
        r_base=r_base, r_gutter=r_gutter, w_gutter=fh.gutter_width,
        nose_len=nose_len, tail_len=tail_len, duct_len=duct_len,
        overall=overall, cone_len=pj.cone_length,
        pipe_len=pj.tailpipe_length, chamber_len=g.chamber_length_m,
        tailsec_len=g.throat_length_m,
        x_duct0=x_duct0, x_cham1=x_cham1, x_cone1=x_cone1,
        x_exit=x_exit, x_base=x_base, x_fh=x_fh,
        t_duct=duct_wall_thickness_m(d), t_skin=C.CFRP_MIN_GAUGE_M,
    )


def _nose_outer(s, n=140):
    """Cowl lip -> body radius, parabolic and tangent to the cylinder."""
    xs = [s["nose_len"] * i / n for i in range(n + 1)]
    dr = s["r_body"] - s["r_lip"]
    rs = [s["r_lip"] + dr * (1.0 - (1.0 - x / s["nose_len"]) ** 2) for x in xs]
    return xs, rs


def _diffuser_inner(s, n=140):
    """ASSUMED shape: lip -> chamber head, smoothstep (zero slope both ends)."""
    xs = [s["nose_len"] * i / n for i in range(n + 1)]
    dr = s["r_cham"] - s["r_lip"]
    out = []
    for x in xs:
        t = x / s["nose_len"]
        out.append(s["r_lip"] + dr * t * t * (3.0 - 2.0 * t))
    return xs, out


def duct_region(s, x) -> str:
    """Which piece of the PULSEJET flowpath station x lands in -- the two
    engines' station maps are independent, so this is worth stating."""
    if x <= s["nose_len"]:
        return "intake diffuser"
    if x <= s["x_cham1"]:
        return "chamber"
    if x <= s["x_cone1"]:
        return "cone"
    return "tailpipe"


def duct_radius(s, x):
    """Inner radius of the shared flowpath at station x."""
    if x <= s["nose_len"]:
        xs, rs = _diffuser_inner(s)
        for i in range(len(xs) - 1):
            if xs[i] <= x <= xs[i + 1]:
                f = (x - xs[i]) / max(xs[i + 1] - xs[i], 1e-12)
                return rs[i] + f * (rs[i + 1] - rs[i])
        return s["r_cham"]
    if x <= s["x_cham1"]:
        return s["r_cham"]
    if x <= s["x_cone1"]:
        f = (x - s["x_cham1"]) / s["cone_len"]
        return s["r_cham"] + f * (s["r_pipe"] - s["r_cham"])
    return s["r_pipe"]


# ==========================================================================
# drawing helpers -- everything is mirrored about y = 0
# ==========================================================================

def mirror_band(ax, xs, r_outer, r_inner, **kw):
    """Fill the band between +-r_outer and +-r_inner on both sides."""
    up = list(zip(xs, r_outer)) + list(zip(reversed(xs), reversed(r_inner)))
    dn = [(x, -y) for x, y in up]
    for pts in (up, dn):
        ax.add_patch(Polygon(pts, closed=True, **kw))


def mirror_fill(ax, xs, rs, **kw):
    """Fill the solid of revolution between -r and +r."""
    pts = list(zip(xs, rs)) + list(zip(reversed(xs), [-r for r in reversed(rs)]))
    ax.add_patch(Polygon(pts, closed=True, **kw))


def mirror_line(ax, xs, rs, **kw):
    ax.plot(xs, rs, **kw)
    kw2 = dict(kw)
    kw2.pop("label", None)
    ax.plot(xs, [-r for r in rs], **kw2)


def dim_bar(ax, x0, x1, y, text, *, color=INK, fs=8.0, tick=0.012,
            va="bottom", dy=0.006, x_text=None, ha="center"):
    ax.annotate("", xy=(x0, y), xytext=(x1, y),
                arrowprops=dict(arrowstyle="<->", color=color, lw=0.9,
                                shrinkA=0, shrinkB=0))
    ax.plot([x0, x0], [y - tick, y + tick], color=color, lw=0.9)
    ax.plot([x1, x1], [y - tick, y + tick], color=color, lw=0.9)
    ax.text(0.5 * (x0 + x1) if x_text is None else x_text,
            y + (dy if va == "bottom" else -dy), text,
            ha=ha, va=va, fontsize=fs, color=color)


def vdim_bar(ax, x, r, text, *, color=INK, fs=8.0, tick=0.012):
    ax.annotate("", xy=(x, -r), xytext=(x, r),
                arrowprops=dict(arrowstyle="<->", color=color, lw=0.9,
                                shrinkA=0, shrinkB=0))
    for y in (-r, r):
        ax.plot([x - tick, x + tick], [y, y], color=color, lw=0.9)
    ax.text(x + 0.014, 0.0, text, ha="left", va="center", fontsize=fs,
            color=color, rotation=90, zorder=11,
            bbox=dict(facecolor="white", edgecolor="none", pad=1.2,
                      alpha=0.85))


def callout(ax, x_feat, y_feat, x_txt, y_txt, text, *, color=INK, fs=7.8,
            ha="center", weight="normal"):
    ax.annotate(text, xy=(x_feat, y_feat), xytext=(x_txt, y_txt),
                ha=ha, va="bottom", fontsize=fs, color=color, weight=weight,
                arrowprops=dict(arrowstyle="-", color=color, lw=0.7,
                                shrinkA=1, shrinkB=1,
                                connectionstyle="arc3,rad=0.0"))


# ==========================================================================
# the figure
# ==========================================================================

def render(design, s, out_path, wing_quarter_chord_frac=0.50):
    g, w = design.geometry, design.wing
    mm = 1000.0

    fig, ax = plt.subplots(figsize=(19.0, 9.9))
    ax.set_aspect("equal")
    ax.axis("off")

    # ---- outer mould line -------------------------------------------
    xs_n, rs_n = _nose_outer(s)
    xs_out = xs_n + [s["x_exit"], s["x_base"]]
    rs_out = rs_n + [s["r_body"], s["r_base"]]
    mirror_fill(ax, xs_out, rs_out, facecolor=SKIN_FILL, edgecolor="none",
                zorder=1)

    # ---- fuel annulus: duct <-> skin over the whole tail section -----
    xs_fa, ro_fa, ri_fa = [], [], []
    n = 120
    for i in range(n + 1):
        x = s["x_cham1"] + (s["x_exit"] - s["x_cham1"]) * i / n
        xs_fa.append(x)
        ro_fa.append(s["r_body"] - s["t_skin"])
        ri_fa.append(duct_radius(s, x) + s["t_duct"])
    mirror_band(ax, xs_fa, ro_fa, ri_fa, facecolor=FUEL_FILL,
                edgecolor=ACCENT, lw=0.5, hatch="///", zorder=2)

    # ---- the flowpath (gas volume) -----------------------------------
    xs_d, rs_d = _diffuser_inner(s)
    xs_duct = xs_d + [s["x_cham1"], s["x_cone1"], s["x_exit"]]
    rs_duct = rs_d + [s["r_cham"], s["r_pipe"], s["r_pipe"]]
    mirror_fill(ax, xs_duct, rs_duct, facecolor=GAS_FILL, edgecolor="none",
                zorder=3)

    # combustion volume: pulsejet chamber (the ramjet burns in the same duct)
    ax.add_patch(Rectangle((s["x_duct0"], -s["r_cham"]),
                           s["chamber_len"], 2 * s["r_cham"],
                           facecolor=CHAMBER_FILL, edgecolor="none",
                           zorder=3.5))

    mirror_line(ax, xs_duct, rs_duct, color=INK, lw=2.0, zorder=6,
                solid_joinstyle="miter")

    # jetpipe carried through the boattail to the base plane: the drag
    # model puts the duct exit AT the base (base_area_m2 subtracts the
    # exit area from the base annulus), but the length budget ends the
    # duct one tail-length forward of it.  Drawn dashed, not solid.
    mirror_line(ax, [s["x_exit"], s["x_base"]], [s["r_pipe"], s["r_pipe"]],
                color=ACCENT, lw=1.3, ls=(0, (5, 3)), zorder=6)

    # ---- outer skin line, drawn last so it sits on top ---------------
    mirror_line(ax, xs_out, rs_out, color=INK, lw=2.4, zorder=7)
    ax.plot([s["x_base"], s["x_base"]], [-s["r_base"], s["r_base"]],
            color=INK, lw=2.4, zorder=7)
    ax.plot([0.0, 0.0], [-s["r_lip"], s["r_lip"]], color=INK, lw=1.0,
            ls=":", zorder=7)

    # centreline
    ax.plot([-0.05, s["x_base"] + 0.06], [0, 0], color=ACCENT, lw=0.8,
            ls=(0, (9, 4, 1.5, 4)), zorder=8)

    # ---- reed-valve station (chamber head bulkhead) ------------------
    ax.add_patch(Rectangle((s["x_duct0"] - 0.008, -s["r_cham"]),
                           0.016, 2 * s["r_cham"], facecolor="#8c8c8c",
                           edgecolor=INK, lw=1.0, hatch="xx", zorder=9))
    for k in range(-4, 5):                       # 9 petals, schematic ticks
        yy = k * s["r_cham"] / 5.0
        ax.plot([s["x_duct0"] - 0.008, s["x_duct0"] + 0.020], [yy, yy * 0.86],
                color=INK, lw=0.7, zorder=9.5)

    # Side-mounted valve inlet runners (orientation="side", pulsejet-fp #8c).
    # Drawn OUTBOARD of the skin, to scale, because they do not fit inside
    # it: the chamber is 0.95 D, leaving only (D - 0.95 D)/2 of radial gap.
    intake = s["spec"].pulsejet_intake()
    gap = s["r_body"] - s["r_cham"]
    for sgn in (1, -1):
        x0 = s["x_duct0"] - intake.duct_length
        ax.add_patch(Rectangle((x0, sgn * s["r_body"]),
                               intake.duct_length,
                               sgn * intake.duct_diameter,
                               facecolor="#d8d8d8", edgecolor=ASSUMED,
                               lw=1.1, ls="--", hatch="..", zorder=5))
        ax.plot([s["x_duct0"], s["x_duct0"]],
                [sgn * s["r_body"], sgn * (s["r_body"] + 0.4 * gap)],
                color=ASSUMED, lw=1.0, ls="--", zorder=5)

    # ---- ramjet flameholder gutter ----------------------------------
    r_local = duct_radius(s, s["x_fh"])
    for sgn in (1, -1):
        ax.add_patch(Rectangle(
            (s["x_fh"] - 0.5 * s["w_gutter"], sgn * s["r_gutter"]),
            s["w_gutter"], sgn * 0.5 * s["w_gutter"],
            facecolor="#5a5a5a", edgecolor=INK, lw=0.8, zorder=9))
        ax.plot([s["x_fh"], s["x_fh"]], [sgn * s["r_gutter"], sgn * r_local],
                color=INK, lw=1.0, zorder=9)

    # ---- ramjet-fp's OWN station map, on the centreline ---------------
    # RamjetGeometry places the combustor at 0.18-0.80 of the duct and its
    # throat at 0.95: those stations do NOT line up with the pulsejet's
    # chamber/cone/tailpipe breaks in the same duct. Marked so the
    # disagreement is visible rather than buried in fp_spec.
    rj_x = [("x_cb0", s["x_duct0"] + s["rj"].x_combustor_start),
            ("x_cb1", s["x_duct0"] + s["rj"].x_combustor_end),
            ("x_th", s["x_duct0"] + s["rj"].x_throat)]
    for lab, xv in rj_x:
        ax.plot([xv], [0.0], marker="v", ms=6.0, color=ACCENT, zorder=10,
                clip_on=False)
        ax.text(xv, -0.011, f"RJ {lab}", ha="center", va="top", fontsize=6.4,
                color=ACCENT, zorder=10)
    ax.text(0.5 * (rj_x[0][1] + rj_x[1][1]) + 0.10, 0.012,
            "ramjet-fp combustor 0.18-0.80 x duct, throat 0.95 x duct",
            ha="center", va="bottom", fontsize=6.8, color=ACCENT, zorder=10,
            bbox=dict(facecolor="white", edgecolor="none", pad=1.2,
                      alpha=0.85))

    # ---- lifting surfaces (STATIONS ASSUMED) -------------------------
    area = w.reference_area_m2
    c_root = 2.0 * area / (w.span_m * (1.0 + w.taper_ratio))
    c_tip = w.taper_ratio * c_root
    t_wing = w.airfoil.thickness_ratio * c_root
    x_w_le = wing_quarter_chord_frac * s["overall"] - 0.25 * c_root
    for sgn in (1, -1):
        ax.add_patch(Polygon(
            [(x_w_le, sgn * s["r_body"]),
             (x_w_le + c_root, sgn * s["r_body"]),
             (x_w_le + c_root, sgn * (s["r_body"] + 0.35 * t_wing)),
             (x_w_le + 0.15 * c_root, sgn * (s["r_body"] + t_wing))],
            closed=True, facecolor="#dcdcdc", edgecolor=ASSUMED, lw=1.2,
            ls="--", zorder=8))

    c_fin = s["tail_len"]
    x_f_te = s["x_base"]
    for sgn in (1, -1):
        ax.add_patch(Polygon(
            [(x_f_te - c_fin, sgn * (s["r_body"] - 0.006)),
             (x_f_te, sgn * s["r_base"]),
             (x_f_te, sgn * (s["r_base"] + 0.055)),
             (x_f_te - 0.55 * c_fin, sgn * (s["r_body"] + 0.010))],
            closed=True, facecolor="#ececec", edgecolor=ASSUMED, lw=1.2,
            ls="--", zorder=8))

    # ==================================================================
    # dimensions and callouts
    # ==================================================================
    # dimension rows sit clear of the outboard valve-inlet scoops
    y_seg = -(s["r_body"] + intake.duct_diameter + 0.038)
    y_all = -(s["r_body"] + intake.duct_diameter + 0.128)
    dim_bar(ax, 0.0, s["nose_len"], y_seg,
            f"nose fairing 2.0 D\n{s['nose_len']*mm:.1f} mm", va="top")
    dim_bar(ax, s["x_duct0"], s["x_cham1"], y_seg,
            f"chamber\n{s['chamber_len']*mm:.1f} mm", va="top")
    dim_bar(ax, s["x_cham1"], s["x_cone1"], y_seg,
            f"cone\n{s['cone_len']*mm:.1f} mm", va="top")
    dim_bar(ax, s["x_cone1"], s["x_exit"], y_seg,
            f"tailpipe {s['pipe_len']*mm:.1f} mm  "
            f"(tail/chamber-dia = {s['pipe_len']/(2*s['r_cham']):.2f})",
            va="top")
    dim_bar(ax, s["x_exit"], s["x_base"], y_seg - 0.058,
            f"aft body 1.0 D = {s['tail_len']*mm:.1f} mm",
            va="top", x_text=s["x_base"] + 0.02, ha="left")
    dim_bar(ax, s["x_duct0"], s["x_exit"], y_seg - 0.058,
            f"duct (chamber + tail section) = {s['duct_len']*mm:.1f} mm",
            va="top", fs=8.5)
    dim_bar(ax, 0.0, s["x_base"], y_all,
            f"OVERALL LENGTH  {s['overall']*mm:.1f} mm  "
            f"({s['overall']/s['d']:.2f} body diameters)",
            va="top", fs=10.5)
    for x in (0.0, s["nose_len"], s["x_cham1"], s["x_cone1"], s["x_exit"],
              s["x_base"]):
        ax.plot([x, x], [y_all, -s["r_body"] - 0.004], color=ACCENT, lw=0.5,
                ls=":", zorder=0)

    vdim_bar(ax, s["x_cham1"] + 0.75 * s["cone_len"], s["r_body"],
             f"body dia {s['d']*mm:.1f} mm")
    vdim_bar(ax, s["x_duct0"] + 0.30 * s["chamber_len"], s["r_cham"],
             f"chamber dia {2*s['r_cham']*mm:.1f} mm", color=ACCENT, fs=7.5)
    vdim_bar(ax, s["x_cone1"] + 0.55 * s["pipe_len"], s["r_pipe"],
             f"throat dia {2*s['r_pipe']*mm:.1f} mm", color=ACCENT, fs=7.5)
    vdim_bar(ax, s["x_base"] + 0.055, s["r_base"],
             f"base dia {2*s['r_base']*mm:.1f} mm", color=ACCENT, fs=7.5)

    # ---- upper callouts ---------------------------------------------
    a_lip = math.pi * g.throat_diameter_m ** 2 / 4.0
    a_cham = math.pi * (2 * s["r_cham"]) ** 2 / 4.0
    port_txt = ""
    try:
        from pulsejet_fp import reference_valve
        v = s["spec"].pulsejet_valve(reference_valve())
        a_port = v.n_petals * v.port_area
        port_txt = (f"\n{v.n_petals} petals {v.petal_length*mm:.0f} x "
                    f"{v.petal_width*mm:.0f} mm, port area "
                    f"{a_port*1e4:.0f} cm2 = {100*a_port/a_cham:.0f}% A_chamber")
    except Exception:                                    # pragma: no cover
        port_txt = "\n(petal data unavailable: pulsejet_fp not importable)"

    y1, y2, y3 = 0.290, 0.372, 0.454
    callout(ax, 0.002, 0.55 * s["r_lip"], 0.03, y2,
            f"INLET LIP / capture plane\ndia {g.throat_diameter_m*mm:.1f} mm "
            f"(= throat dia), A = {a_lip*1e4:.0f} cm2", ha="left")
    callout(ax, 0.42 * s["nose_len"], -0.50 * s["r_body"],
            0.32 * s["nose_len"], -(s["r_body"] + 0.052),
            "ASSUMED internal diffuser\n(the model carries no duct shape)",
            color=ASSUMED, ha="center")
    callout(ax, s["x_duct0"], 0.90 * s["r_cham"], s["x_duct0"] + 0.02, y3,
            f"REED-VALVE STATION (chamber head)\nx = {s['x_duct0']*mm:.1f} mm"
            + port_txt, ha="left")
    callout(ax, s["x_duct0"] - 0.5 * intake.duct_length,
            s["r_body"] + intake.duct_diameter,
            s["x_duct0"] - 0.5 * intake.duct_length, y1,
            f"SIDE VALVE INLETS {intake.duct_diameter*mm:.0f} mm dia x "
            f"{intake.duct_length*mm:.0f} mm\n"
            f"(boundary-layer air at static p)\n"
            f"drawn OUTBOARD -- skin/chamber gap\n"
            f"is only {gap*mm:.1f} mm, they do not fit inside",
            color=ASSUMED, ha="center")
    callout(ax, s["x_duct0"] + 0.62 * s["chamber_len"], 0.30 * s["r_cham"],
            s["x_duct0"] + 0.30 * s["chamber_len"], y2,
            f"PULSEJET CHAMBER  dia {2*s['r_cham']*mm:.1f} mm (0.95 D) x "
            f"{s['chamber_len']*mm:.1f} mm\n= ramjet combustor "
            f"(shared flowpath)", ha="center")
    callout(ax, s["x_fh"], s["r_gutter"] + 0.5 * s["w_gutter"],
            s["x_fh"] + 0.02, y2,
            f"RAMJET FLAMEHOLDER  x = {s['x_fh']*mm:.1f} mm (0.28 x duct)\n"
            f"gutter r = {s['r_gutter']*mm:.1f} mm, w = {s['w_gutter']*mm:.1f}"
            f" mm, A_frontal = {s['fh'].frontal_area*1e4:.0f} cm2\n"
            f"-- lands in the pulsejet {duct_region(s, s['x_fh']).upper()}",
            ha="left")

    v_ann_model = mission.annular_volume_m3(g.diameter_m, g.throat_diameter_m,
                                            g.throat_length_m)
    cap_kg = (mission.FUEL_VOLUME_FRACTION_OF_ANNULUS * v_ann_model
              * g.fuel.density_kg_per_m3)
    callout(ax, 0.5 * (s["x_cone1"] + s["x_exit"]),
            0.5 * (s["r_pipe"] + s["r_body"]),
            0.5 * (s["x_cone1"] + s["x_exit"]) + 0.02, y3,
            f"FUEL ANNULUS (duct <-> skin over the tail section)\n"
            f"model volume {v_ann_model*1e3:.1f} L, "
            f"{100*mission.FUEL_VOLUME_FRACTION_OF_ANNULUS:.0f}% usable -> "
            f"{cap_kg:.2f} kg {g.fuel.key} capacity\n"
            f"loaded on this design: {design.loaded_fuel_kg:.2f} kg "
            f"(burn limit {design.burn_limit_kg:.2f} kg)", ha="center")
    callout(ax, s["x_exit"], 0.55 * s["r_pipe"], s["x_exit"] + 0.02, y3,
            f"NOZZLE EXIT dia {2*s['r_pipe']*mm:.1f} mm\n"
            f"jetpipe (dashed) carried to the base plane:\n"
            f"the drag model's base area is the annulus around it",
            ha="left")
    x_bt = s["x_exit"] + 0.45 * s["tail_len"]
    callout(ax, x_bt, -(s["r_body"] - 0.45 * (s["r_body"] - s["r_base"])),
            s["x_base"] - 0.46, -(s["r_body"] + 0.052),
            f"boattail {BOATTAIL_HALF_ANGLE_DEG:.0f} deg half-angle "
            f"-> base dia {2*s['r_base']*mm:.1f} mm", ha="center", color=ACCENT)
    callout(ax, x_w_le + 0.5 * c_root, s["r_body"] + 0.7 * t_wing,
            x_w_le + 0.5 * c_root, y1,
            f"WING STATION -- ASSUMED (1/4-chord at "
            f"{100*wing_quarter_chord_frac:.0f}% of length)\n"
            f"root {c_root*mm:.0f} mm, tip {c_tip*mm:.0f} mm, span "
            f"{w.span_m*mm:.0f} mm, AR {w.aspect_ratio:.2f},\n"
            f"sweep {w.sweep_deg:.1f} deg, {w.airfoil.key}, "
            f"S = {area:.4f} m2  (shown edge-on)",
            color=ASSUMED, ha="center")
    callout(ax, x_f_te - 0.45 * c_fin, s["r_body"] + 0.012,
            s["x_base"] + 0.14, y1,
            "FIN STATION -- ASSUMED\n(no fin geometry in the model; fins\n"
            "live inside STRUCTURAL_OVERHEAD_FRACTION = 0.35)",
            color=ASSUMED, ha="right")

    # ---- legend for the fills ---------------------------------------
    lx, ly = 0.02, y_all - 0.108
    items = [(GAS_FILL, None, "flowpath (gas)"),
             (CHAMBER_FILL, None, "combustion volume"),
             (FUEL_FILL, "///", "fuel annulus"),
             (SKIN_FILL, None, "structure"),
             ("#8c8c8c", "xx", "reed-valve station"),
             ("#5a5a5a", None, "flameholder gutter")]
    for i, (fc, hh, lab) in enumerate(items):
        x = lx + i * 0.33
        ax.add_patch(Rectangle((x, ly), 0.045, 0.030, facecolor=fc,
                               edgecolor=INK, lw=0.8, hatch=hh))
        ax.text(x + 0.055, ly + 0.015, lab, ha="left", va="center", fontsize=8)

    ax.text(lx, ly - 0.022,
            f"RJ ticks = ramjet-fp's own station map (RamjetGeometry): its "
            f"combustor and throat do NOT line up with the pulsejet's "
            f"chamber / cone / tailpipe breaks in the same duct.    "
            f"Duct wall {s['t_duct']*mm:.1f} mm steel, skin "
            f"{s['t_skin']*mm:.1f} mm CFRP -- wall lines are the only thing "
            f"not to scale.",
            ha="left", va="top", fontsize=7.8, color=ACCENT)

    # ---- title -------------------------------------------------------
    ax.set_title(
        f"{design.name}\n"
        f"body dia {s['d']*mm:.1f} mm  |  overall {s['overall']*mm:.1f} mm  |  "
        f"fineness {s['overall']/s['d']:.2f}  |  duct {s['duct_len']*mm:.1f} mm"
        f"  |  tail/chamber-dia {s['pipe_len']/(2*s['r_cham']):.2f}  |  "
        f"throat dia {2*s['r_pipe']*mm:.1f} mm "
        f"(A_th/A_body = {(2*s['r_pipe']/s['d'])**2:.3f})\n"
        f"longitudinal section, to scale, mirrored about the centreline",
        fontsize=12.5, linespacing=1.5, pad=14)

    ax.set_xlim(-0.10, s["x_base"] + 0.16)
    ax.set_ylim(ly - 0.075, y3 + 0.125)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, facecolor="white")
    plt.close(fig)
    return dict(c_root=c_root, c_tip=c_tip, area=area, a_lip=a_lip,
                a_cham=a_cham, v_ann_model=v_ann_model, cap_kg=cap_kg)


# ==========================================================================

def true_annulus_volume_m3(s):
    """Geometric annulus volume including the cone (the model's own
    ``annular_volume_m3`` treats the whole tail section as throat-sized)."""
    n, total = 400, 0.0
    x0, x1 = s["x_cham1"], s["x_exit"]
    dx = (x1 - x0) / n
    for i in range(n):
        x = x0 + (i + 0.5) * dx
        r = duct_radius(s, x)
        total += math.pi * (s["r_body"] ** 2 - r ** 2) * dx
    return total


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--design", default=str(DEFAULT_DESIGN))
    p.add_argument("--name", default=None)
    p.add_argument("--wing-quarter-chord-frac", type=float, default=0.50)
    args = p.parse_args()

    dpath = Path(args.design).resolve()
    design = load_frozen_design(dpath)
    s = build_stations(design)
    tag = args.name or dpath.parent.name
    out = OUT_DIR / f"cross_section_{tag}.png"
    extra = render(design, s, out, args.wing_quarter_chord_frac)

    g, mm = design.geometry, 1000.0
    rows = [
        ("design.json diameter_m", g.diameter_m * mm, "mm"),
        ("design.json throat_diameter_m", g.throat_diameter_m * mm, "mm"),
        ("design.json chamber_length_m", g.chamber_length_m * mm, "mm"),
        ("design.json throat_length_m (tail section)",
         g.throat_length_m * mm, "mm"),
        ("chamber dia = 0.95 D", 2 * s["r_cham"] * mm, "mm"),
        ("cone length = 0.1733 x tail section", s["cone_len"] * mm, "mm"),
        ("tailpipe length", s["pipe_len"] * mm, "mm"),
        ("tail/chamber-dia ratio", s["pipe_len"] / (2 * s["r_cham"]), "-"),
        ("nose fairing = 2.0 D", s["nose_len"] * mm, "mm"),
        ("aft body = 1.0 D", s["tail_len"] * mm, "mm"),
        ("duct length (chamber + tail section)", s["duct_len"] * mm, "mm"),
        ("OVERALL LENGTH", s["overall"] * mm, "mm"),
        ("fineness ratio", s["overall"] / g.diameter_m, "-"),
        ("base dia (8 deg boattail)", 2 * s["r_base"] * mm, "mm"),
        ("x reed-valve station", s["x_duct0"] * mm, "mm"),
        ("x flameholder (0.28 x duct)", s["x_fh"] * mm, "mm"),
        ("  -> local duct radius there", duct_radius(s, s["x_fh"]) * mm, "mm"),
        (f"  -> lands in the pulsejet {duct_region(s, s['x_fh'])}",
         float("nan"), ""),
        ("flameholder gutter radius", s["r_gutter"] * mm, "mm"),
        ("flameholder gutter width", s["w_gutter"] * mm, "mm"),
        ("flameholder frontal area", s["fh"].frontal_area * 1e4, "cm2"),
        ("x nozzle exit / duct end", s["x_exit"] * mm, "mm"),
        ("x base plane", s["x_base"] * mm, "mm"),
        ("wing root chord", extra["c_root"] * mm, "mm"),
        ("wing tip chord", extra["c_tip"] * mm, "mm"),
        ("wing reference area", extra["area"], "m2"),
        ("fuel annulus volume (model formula)",
         extra["v_ann_model"] * 1e3, "L"),
        ("fuel annulus volume (true, cone included)",
         true_annulus_volume_m3(s) * 1e3, "L"),
        ("fuel capacity @ 50% annulus", extra["cap_kg"], "kg"),
        ("fuel actually loaded", design.loaded_fuel_kg, "kg"),
    ]
    w0 = max(len(r[0]) for r in rows)
    print(f"\n{design.name}\n{'-' * (w0 + 20)}")
    for label, val, unit in rows:
        if val != val:                       # NaN -> annotation-only row
            print(label)
        else:
            print(f"{label:<{w0}}  {val:12.4f}  {unit}")
    print(f"{'-' * (w0 + 20)}")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
