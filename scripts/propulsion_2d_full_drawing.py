"""Full 2D propulsion section WITH the translating inlet shut-off.

Composes the experimental forward end (pitot-static boom, fixed fairing,
annular slot, translating sleeve) onto the frozen aft end (chamber, reed
valves, flameholder, contraction cone, tailpipe, divergent nozzle, boattail)
and draws the whole vehicle to scale.

    .venv/Scripts/python scripts/propulsion_2d_full_drawing.py [--show]

The frozen tool is untouched; this only reads from it.

Fidelity notes
--------------
* The duct is CLAMPED so it can never leave the outer mould line, and the
  clamp is checked and reported.  An earlier revision let the diffuser wall
  poke ~2.5 mm outside the cowl over x 300-315 mm: the sleeve rises 7.08 mm
  there (that rise IS the seal stroke), so to hold capture area the outer
  wall had to gain 6.6 mm in 7 mm while the cowl only grew 1.3 mm.  Fixed by
  giving the cowl a fast-rising lip.
* Flameholder, reed valves and the pitot-static boom are all ASSUMED -- the
  export carries no geometry for any of them.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from scripts.propulsion_2d.geometry import (  # noqa: E402
    MM, Chain, Segment, build_vehicle, load_inputs)
from scripts.propulsion_2d.translating_inlet import (  # noqa: E402
    DIFFUSER_ANGLE_DEG, SPAR_RADIUS_M, from_vehicle)

OUT = _REPO / "out_medium_model" / "propulsion_2d"

INK, ACCENT, GHOST = "#101010", "#404040", "#8a8a8a"
SKIN, GAS, CHAMBER, FUEL = "#e9e9e9", "#ffffff", "#c9c9c9", "#b6b6b6"
OPEN_C, SEAL_C, SLOT_C = "#1f5fa8", "#a83232", "#009060"
FH_C, RV_C, PROBE_C = "#5a5a5a", "#7a4a9a", "#1a6a6a"

# ---- ASSUMED features (no geometry for any of these in the export) -------
PROBE_LEN_M = 0.160          # nose boom ahead of the fairing tip
PROBE_DIA_M = 0.010
PROBE_STATIC_FRAC = 0.56     # static ports at 0.56 L = 9 probe diameters aft
COWL_LIP_RISE_M = 0.040      # cowl reaches body radius this far aft of slot
COWL_LIP_EXP = 0.28          # fast-rising lip: f(t) = t**EXP
SEAL_FAYING_M = 0.012        # axial land of flat-on-flat seal contact
FH_FRAC_OF_CHAMBER = 0.28    # V-gutter station, fraction of chamber length
FH_RADIUS_FRAC = 0.60        # gutter mid-radius / chamber flow radius
FH_WIDTH_M = 0.025           # gutter width (projected)
RV_X_FRAC = 0.075            # reed-valve port, fraction of chamber length aft
RV_PORT_LEN_M = 0.070        # axial length of the port in the chamber wall
RV_RUNNER_DIA_M = 0.057      # side runner diameter
RV_RUNNER_LEN_M = 0.105


def compose(v, t):
    """Forward (new) + aft (frozen) contours, plus derived internals."""
    x_slot, x_head = t.x_slot_m, v.station("chamber_start")
    r_body = 0.5 * v.d_body
    land = t.stroke_m
    t_wall = v.dims["materials_and_gauges"]["duct_wall_thickness_m"]

    r_eq0 = math.sqrt(t.capture_area_m2 / math.pi)
    r_eq1 = r_eq0 + t.diffuser_length_m * math.tan(
        math.radians(DIFFUSER_ANGLE_DEG))

    # ---- cowl: fast-rising lip so the duct has somewhere to go ----------
    r_lip_out = t.r_lip_m + t.cowl_lip_thickness_m
    x_lip_top = min(x_slot + COWL_LIP_RISE_M, x_head)

    def cowl_outer_law(x, _x0=x_slot, _L=x_lip_top - x_slot,
                       _r0=r_lip_out, _r1=r_body):
        s = min(max((x - _x0) / _L, 0.0), 1.0)
        return _r0 + (_r1 - _r0) * (s ** COWL_LIP_EXP)

    oml = Chain("oml", "outer mould line", [
        Segment("pitot-static boom", "oml", "line", -PROBE_LEN_M, 0.0,
                0.5 * PROBE_DIA_M, 0.5 * PROBE_DIA_M, "ASSUMED",
                f"{PROBE_LEN_M * MM:.0f} mm nose boom, "
                f"{PROBE_DIA_M * MM:.0f} mm dia"),
        Segment("fixed nose fairing", "oml", "cone", 0.0, x_slot,
                0.5 * PROBE_DIA_M, t.r_fore_m, "ASSUMED",
                "FIXED, carries the avionics"),
        Segment("cowl lip", "oml", "custom", x_slot, x_lip_top,
                r_lip_out, r_body, "ASSUMED",
                f"fast lip, t**{COWL_LIP_EXP}", law=cowl_outer_law),
        Segment("cowl barrel", "oml", "line", x_lip_top, x_head,
                r_body, r_body, "ASSUMED", ""),
    ] + [s for s in v.chains["oml"].segments if s.x0 >= x_head - 1e-9])

    tan_s = math.tan(math.radians(t.seal_half_angle_deg))
    fay = t.faying_land_m
    r_fay_top = t.r_lip_m + fay * tan_s

    def seat_line(x):
        """The 45 deg seat, and the sleeve's matching face at full stroke."""
        return t.r_lip_m + (x - x_slot) * tan_s

    def sleeve(stroke):
        dx = -stroke
        return Chain("sleeve", "translating sleeve", [
            Segment("seal cone", "sleeve", "cone", x_slot + dx,
                    x_slot + land + dx, t.r_fore_m, t.r_lip_m, "ASSUMED",
                    "rises one stroke length from the fairing rim"),
            Segment("faying land", "sleeve", "cone", x_slot + land + dx,
                    x_slot + land + fay + dx, t.r_lip_m, r_fay_top, "ASSUMED",
                    f"{fay * MM:.0f} mm of flat-on-flat contact at full "
                    f"stroke"),
            Segment("closure", "sleeve", "smoothstep",
                    x_slot + land + fay + dx, x_head, r_fay_top,
                    SPAR_RADIUS_M, "ASSUMED", ""),
        ])

    sleeve_open = sleeve(0.0)

    # ---- diffuser outer wall, solved then CLAMPED inside the OML -------
    def cowl_inner_law(x, _R0=r_eq0, _R1=r_eq1, _x0=x_slot,
                       _L=t.diffuser_length_m):
        r_eq = _R0 + (_R1 - _R0) * (x - _x0) / _L
        r_in = sleeve_open.r_at(x) or SPAR_RADIUS_M
        want = math.sqrt(r_eq ** 2 + r_in ** 2)
        if x <= x_slot + fay:              # the seat itself, so it can mate
            want = max(want, seat_line(x))
        return min(want, cowl_outer_law(x) - t_wall)   # never leave the OML

    cowl_inner = Chain("cowl_inner", "seat + diffuser", [
        Segment("seat + diffuser", "cowl_inner", "custom", x_slot, x_head,
                t.r_lip_m, cowl_inner_law(x_head), "ASSUMED",
                f"uniform {DIFFUSER_ANGLE_DEG:.0f} deg equivalent cone, "
                f"clamped to the OML", law=cowl_inner_law),
    ])

    spar = Chain("spar", "spar", [
        Segment("spar", "spar", "line", 0.0, x_head, SPAR_RADIUS_M,
                SPAR_RADIUS_M, "ASSUMED", "fairing support + actuator"),
    ])

    # ---- internals ------------------------------------------------------
    x_c0, x_c1 = x_head, v.station("chamber_end")
    L_ch = x_c1 - x_c0
    r_ch = v.chains["flow"].segments[0].r0
    fh = dict(x=x_c0 + FH_FRAC_OF_CHAMBER * L_ch, r=FH_RADIUS_FRAC * r_ch,
              w=FH_WIDTH_M)
    fh["blockage"] = 2 * math.pi * fh["r"] * fh["w"] / (math.pi * r_ch ** 2)
    rv = dict(x=x_c0 + RV_X_FRAC * L_ch, length=RV_PORT_LEN_M,
              dia=RV_RUNNER_DIA_M, runner=RV_RUNNER_LEN_M)
    rv["port_area"] = 2 * math.pi * (RV_RUNNER_DIA_M / 2) ** 2
    rv["port_frac"] = rv["port_area"] / (math.pi * r_ch ** 2)

    a_dump = math.pi * (cowl_inner_law(x_head) ** 2 - SPAR_RADIUS_M ** 2)
    return dict(oml=oml, cowl_inner=cowl_inner, spar=spar,
                sleeve_open=sleeve_open, sleeve_shut=sleeve(t.stroke_m),
                a_dump=a_dump, fh=fh, rv=rv, x_lip_top=x_lip_top,
                cowl_outer_law=cowl_outer_law, seat_line=seat_line)


def flow_area(v, t, c, x):
    x_head = v.station("chamber_start")
    if x < t.x_slot_m:
        return None
    if x < x_head:
        ro = c["cowl_inner"].r_at(x)
        ri = max(c["sleeve_open"].r_at(x) or SPAR_RADIUS_M, SPAR_RADIUS_M)
        return math.pi * max(ro ** 2 - ri ** 2, 0.0)
    return v.flow_area(x)


def containment_report(v, t, c) -> dict:
    """Nothing may be drawn outside the OML.  Verify, do not assume."""
    # Compare against the COWL OUTER law, not the OML chain.  At exactly
    # x_slot the chain still reports the fairing rim, and that discontinuity
    # IS the slot -- measuring against it would flag the inlet as a fault.
    worst, worst_x, amin, amin_x = 0.0, 0.0, 1e9, 0.0
    n = 4000
    x0, x1 = t.x_slot_m, v.station("chamber_start")
    t_wall = v.dims["materials_and_gauges"]["duct_wall_thickness_m"]
    for i in range(n + 1):
        x = x0 + (x1 - x0) * i / n
        over = c["cowl_inner"].r_at(x) - (c["cowl_outer_law"](x) - t_wall)
        if over > worst:
            worst, worst_x = over, x
        a = flow_area(v, t, c, x)
        if a is not None and a < amin:
            amin, amin_x = a, x
    return dict(max_overshoot_m=worst, at_x_m=worst_x,
                min_area_m2=amin, min_area_x_m=amin_x)


# ==========================================================================

def draw(v, t, c, path: Path, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    x_slot, x_head = t.x_slot_m, v.station("chamber_start")
    x_cham1 = v.station("chamber_end")
    x_cone1 = v.station("cone_end")
    x_pipe1 = v.station("tailpipe_end")
    x_thr = v.station("nozzle_throat")
    x_exit = v.station("nozzle_exit")
    rb = 0.5 * v.d_body * MM
    r_cham = v.chains["flow"].segments[0].r0 * MM
    fh, rv = c["fh"], c["rv"]

    stations = [("nose tip", 0.0), ("SLOT", x_slot),
                ("chamber head / dump", x_head), ("chamber end", x_cham1),
                ("cone end", x_cone1), ("tailpipe end", x_pipe1),
                ("throat", x_thr), ("nozzle exit", x_exit)]
    tier_of, prev, tier, deep = {}, None, 0, 0
    for _, xv in stations:
        xm = xv * MM
        tier = (tier + 1) % 2 if (prev is not None
                                  and xm - prev < 0.025 * x_exit * MM) else 0
        prev, tier_of[round(xm, 4)] = xm, tier
        deep = max(deep, tier)

    y_seg = -(rb + 100.0 + deep * 62.0)
    y_all = y_seg - 82.0
    xlim = (-PROBE_LEN_M * MM - 55.0, x_exit * MM + 60.0)
    ylim = (y_all - 48.0, rb + 150.0 + deep * 62.0)
    h_sec = 17.0 * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    fig_h = h_sec + 1.5 + 2.5 + 0.78 + 0.5 + 0.7

    fig = plt.figure(figsize=(17.0, fig_h))
    gs = fig.add_gridspec(
        3, 1, height_ratios=[h_sec, 1.5, 2.5], hspace=0.42,
        left=0.035, right=0.985, top=1.0 - 0.78 / fig_h, bottom=0.5 / fig_h)
    ax = fig.add_subplot(gs[0])
    axa = fig.add_subplot(gs[1], sharex=ax)
    axd = fig.add_subplot(gs[2])
    ax.set_aspect("equal")

    def mfill(a, xs, rs, **kw):
        a.add_patch(Polygon(list(zip(xs, rs)) + [(x, -r) for x, r in
                                                 zip(reversed(xs),
                                                     reversed(rs))],
                            closed=True, **kw))

    def mband(a, xs, ro, ri, **kw):
        up = list(zip(xs, ro)) + list(zip(reversed(xs), reversed(ri)))
        for pts in (up, [(x, -y) for x, y in up]):
            a.add_patch(Polygon(pts, closed=True, **kw))

    def mline(a, xs, rs, **kw):
        a.plot(xs, rs, **kw)
        kw2 = {k: val for k, val in kw.items() if k != "label"}
        a.plot(xs, [-r for r in rs], **kw2)

    def poly(chain, step=0.0002):
        xs, rs = chain.polyline(step)
        return [x * MM for x in xs], [r * MM for r in rs]

    def paint(a, detail=False):
        xo, ro = poly(c["oml"])
        mfill(a, xo, ro, facecolor=SKIN, edgecolor="none", zorder=1)
        n = 300
        xs_d = [x_slot + (x_head - x_slot) * i / n for i in range(n + 1)]
        mband(a, [x * MM for x in xs_d],
              [c["cowl_inner"].r_at(x) * MM for x in xs_d],
              [max(c["sleeve_open"].r_at(x) or SPAR_RADIUS_M,
                   SPAR_RADIUS_M) * MM for x in xs_d],
              facecolor=GAS, edgecolor="none", zorder=2)
        xf, rf = poly(v.chains["flow"])
        mfill(a, xf, rf, facecolor=GAS, edgecolor="none", zorder=2)
        a.add_patch(Rectangle((x_head * MM, -r_cham),
                              (x_cham1 - x_head) * MM, 2 * r_cham,
                              facecolor=CHAMBER, edgecolor="none", zorder=2.5))
        if not detail:
            xs_a = [x_cham1 + (x_pipe1 - x_cham1) * i / 240
                    for i in range(241)]
            mband(a, [x * MM for x in xs_a],
                  [v.chains["skin_id"].r_at(x) * MM for x in xs_a],
                  [v.chains["duct_od"].r_at(x) * MM for x in xs_a],
                  facecolor=FUEL, edgecolor=ACCENT, lw=0.5, hatch="///",
                  zorder=3)
        xsp, rsp = poly(c["spar"])
        mfill(a, xsp, rsp, facecolor="#d0d0d0", edgecolor=ACCENT, lw=0.8,
              zorder=3.5)
        for key, col, ls, lab in (
                ("sleeve_open", OPEN_C, "-",
                 "sleeve RETRACTED — ramjet inlet OPEN"),
                ("sleeve_shut", SEAL_C, "--",
                 "sleeve EXTENDED — ramjet inlet SEALED")):
            xs, rs = poly(c[key])
            sp = SPAR_RADIUS_M * MM
            up = list(zip(xs, rs)) + [(xs[-1], sp), (xs[0], sp)]
            first = True
            for pts in (up, [(x, -y) for x, y in up]):
                a.add_patch(Polygon(pts, closed=True, facecolor=col,
                                    alpha=0.24, edgecolor=col, lw=1.6, ls=ls,
                                    zorder=4,
                                    label=lab if (first and not detail)
                                    else None))
                first = False
        if not detail:
            for nm in ("duct_od", "skin_id"):
                xw, rw = poly(v.chains[nm])
                mline(a, xw, rw, color=ACCENT, lw=0.8, zorder=5)
        mline(a, xf, rf, color=INK, lw=1.9, zorder=6)
        xci, rci = poly(c["cowl_inner"])
        mline(a, xci, rci, color=GHOST, lw=1.5, ls="--", zorder=6)
        mline(a, xo, ro, color=INK, lw=2.3, zorder=7)
        for sgn in (1, -1):
            a.plot([x_head * MM] * 2,
                   [sgn * c["cowl_inner"].r_at(x_head) * MM, sgn * r_cham],
                   color=GHOST, lw=1.6, ls="--", zorder=7)
            a.plot([x_exit * MM] * 2,
                   [sgn * v.chains["duct_od"].r_at(x_exit) * MM,
                    sgn * v.chains["oml"].segments[-1].r1 * MM],
                   color=INK, lw=2.3, zorder=7)
            a.plot([x_slot * MM] * 2,
                   [sgn * t.r_fore_m * MM, sgn * t.r_lip_m * MM],
                   color=SLOT_C, lw=3.4, solid_capstyle="butt", zorder=8)

    paint(ax)

    # ---- pitot-static boom ---------------------------------------------
    x_st = (-PROBE_LEN_M + PROBE_STATIC_FRAC * PROBE_LEN_M) * MM
    ax.plot([-PROBE_LEN_M * MM], [0], marker="o", ms=3.4, color=PROBE_C,
            zorder=9)
    for sgn in (1, -1):
        ax.plot([x_st, x_st], [sgn * 0.5 * PROBE_DIA_M * MM,
                               sgn * (0.5 * PROBE_DIA_M * MM + 3.5)],
                color=PROBE_C, lw=1.6, zorder=9)

    # ---- reed valves: external side runners + petal packs ---------------
    r_sk = rb
    for sgn in (1, -1):
        ax.add_patch(Rectangle(
            (rv["x"] * MM, sgn * r_sk), rv["runner"] * MM,
            sgn * rv["dia"] * MM, facecolor="#e2dcec", edgecolor=RV_C,
            lw=1.3, ls="--", zorder=5.5))
        ax.plot([rv["x"] * MM, rv["x"] * MM],
                [sgn * (r_cham), sgn * r_sk], color=RV_C, lw=2.0, zorder=6.5)
        for k in range(5):
            yy = sgn * (r_cham - 3 - k * (r_cham * 0.09))
            ax.plot([rv["x"] * MM, rv["x"] * MM + rv["length"] * MM * 0.45],
                    [yy, yy * 0.93], color=RV_C, lw=0.9, zorder=6.6)

    # ---- ramjet flameholder: annular V-gutter ---------------------------
    for sgn in (1, -1):
        xg, rg, w = fh["x"] * MM, sgn * fh["r"] * MM, fh["w"] * MM
        ax.add_patch(Polygon([(xg - 0.5 * w, rg), (xg + 0.5 * w, rg + sgn * w),
                              (xg + 0.5 * w, rg - sgn * w)], closed=True,
                             facecolor=FH_C, edgecolor=INK, lw=0.9, zorder=6.8))
        ax.plot([xg, xg], [rg, sgn * r_cham], color=FH_C, lw=1.1, zorder=6.7)

    ax.plot(list(xlim), [0, 0], color=ACCENT, lw=0.7,
            ls=(0, (9, 4, 1.5, 4)), zorder=8)

    # ---- stations -------------------------------------------------------
    for name, xv in stations:
        xm = xv * MM
        col = SLOT_C if name == "SLOT" else ACCENT
        ax.axvline(xm, color=col, lw=0.6, ls=":", zorder=0.5)
        dy = 16 + tier_of[round(xm, 4)] * 62
        ax.text(xm, rb + dy, name, rotation=90, ha="center", va="bottom",
                fontsize=6.8, color=col,
                weight="bold" if name == "SLOT" else "normal")
        ax.text(xm, -rb - dy, f"{xm:.1f}", rotation=90, ha="center",
                va="top", fontsize=6.6, color=col)

    # ---- callouts, all placed clear of the body -------------------------
    def call(x_feat, y_feat, x_txt, y_txt, text, col, ha="center", fs=7.4):
        ax.annotate(text, xy=(x_feat, y_feat), xytext=(x_txt, y_txt),
                    fontsize=fs, color=col, ha=ha, va="center", zorder=12,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                              edgecolor=col, lw=0.7, alpha=0.95),
                    arrowprops=dict(arrowstyle="->", color=col, lw=0.9))

    y_hi = rb + 116 + deep * 62
    call(-PROBE_LEN_M * MM, 0, -PROBE_LEN_M * MM + 30, y_hi,
         f"PITOT-STATIC BOOM\n{PROBE_LEN_M * MM:.0f} × "
         f"{PROBE_DIA_M * MM:.0f} mm dia\ntotal at tip, statics "
         f"{PROBE_STATIC_FRAC * PROBE_LEN_M * MM:.0f} mm aft "
         f"({PROBE_STATIC_FRAC * PROBE_LEN_M / PROBE_DIA_M:.0f} d)",
         PROBE_C, ha="left")
    call(rv["x"] * MM + 20, rb + rv["dia"] * MM * 0.5, 760, y_hi,
         f"REED VALVES — pulsejet side inlets\n"
         f"2 × {rv['dia'] * MM:.0f} mm runners = "
         f"{rv['port_area'] * 1e4:.0f} cm² port "
         f"({100 * rv['port_frac']:.0f}% of chamber)\n"
         f"runners drawn OUTBOARD: they do not fit in a "
         f"{2 * rb:.0f} mm skin", RV_C)
    call(fh["x"] * MM, fh["r"] * MM, 1520, y_hi,
         f"RAMJET FLAMEHOLDER — annular V-gutter\n"
         f"x {fh['x'] * MM:.0f} mm ({FH_FRAC_OF_CHAMBER:.0%} into the "
         f"chamber), r {fh['r'] * MM:.0f} mm\n"
         f"{fh['w'] * MM:.0f} mm gutter, "
         f"{100 * fh['blockage']:.0f}% blockage", FH_C)

    # ---- dimensions -----------------------------------------------------
    def dim(x0, x1, y, text, fs=7.4):
        ax.annotate("", xy=(x0, y), xytext=(x1, y),
                    arrowprops=dict(arrowstyle="<->", color=ACCENT, lw=0.8,
                                    shrinkA=0, shrinkB=0))
        for xx in (x0, x1):
            ax.plot([xx, xx], [y - 5, y + 5], color=ACCENT, lw=0.8)
        ax.text(0.5 * (x0 + x1), y - 9, text, ha="center", va="top",
                fontsize=fs, color=ACCENT)

    for a, b, lab in [(-PROBE_LEN_M * MM, 0, "boom"),
                      (0, x_slot * MM, "fixed fairing"),
                      (x_slot * MM, x_head * MM, "diffuser"),
                      (x_head * MM, x_cham1 * MM, "chamber"),
                      (x_cham1 * MM, x_cone1 * MM, "cone"),
                      (x_cone1 * MM, x_thr * MM, "tailpipe"),
                      (x_pipe1 * MM, x_exit * MM, "boattail")]:
        dim(a, b, y_seg, f"{lab}\n{b - a:.1f}")
    dim(0, x_exit * MM, y_all,
        f"BODY {x_exit * MM:.1f} mm  (fineness {x_exit / v.d_body:.2f})   |   "
        f"with boom {(x_exit + PROBE_LEN_M) * MM:.1f} mm", fs=9.0)

    def vdim(x, r, text, fs=7.2):
        ax.annotate("", xy=(x, -r), xytext=(x, r),
                    arrowprops=dict(arrowstyle="<->", color=ACCENT, lw=0.8,
                                    shrinkA=0, shrinkB=0))
        ax.text(x + 8, 0, text, ha="left", va="center", fontsize=fs,
                color=ACCENT, rotation=90, zorder=11,
                bbox=dict(facecolor="white", edgecolor="none", pad=1.0,
                          alpha=0.9))

    vdim((x_head + 0.62 * (x_cham1 - x_head)) * MM, r_cham,
         f"chamber flow dia {2 * r_cham:.1f}")
    vdim((x_head + 0.86 * (x_cham1 - x_head)) * MM, rb, f"body dia {2 * rb:.1f}")
    r_th = v.chains["flow"].r_at(x_thr) * MM
    vdim((x_cone1 + 0.11 * (x_pipe1 - x_cone1)) * MM, r_th,
         f"throat dia {2 * r_th:.2f}")
    vdim(x_exit * MM, v.chains["flow"].r_at(x_exit) * MM,
         f"exit dia {2 * v.chains['flow'].r_at(x_exit) * MM:.2f}", fs=6.8)

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_yticks([])
    for sp in ("top", "right", "left", "bottom"):
        ax.spines[sp].set_visible(False)
    ax.tick_params(labelbottom=False, bottom=False)
    # inside the tailpipe: the only genuinely empty region on the drawing
    ax.legend(loc="center", fontsize=8, framealpha=0.95,
              bbox_to_anchor=(0.82, 0.556))

    # ---- area panel -----------------------------------------------------
    xs, ys = [], []
    for i in range(2000):
        x = x_slot + (x_exit - x_slot) * i / 1999
        a = flow_area(v, t, c, x)
        if a is not None:
            xs.append(x * MM)
            ys.append(a * 1e4)
    a_th = math.pi * (0.5 * v.dims["internal_flowpath"]
                      ["tailpipe_diameter_m"]) ** 2 * 1e4
    axa.fill_between(xs, 0, ys, color="#eeeeee", zorder=1)
    axa.plot(xs, ys, color=INK, lw=1.7, zorder=3)
    axa.axhline(a_th, color=ACCENT, lw=0.8, ls="--", zorder=2)
    axa.text(xlim[1] - 10, a_th, "A_throat", fontsize=7, va="center",
             ha="right", color=ACCENT,
             bbox=dict(facecolor="white", edgecolor="none", pad=1.0))
    for xv, col in ((x_slot, SLOT_C), (fh["x"], FH_C), (rv["x"], RV_C)):
        axa.axvline(xv * MM, color=col, lw=0.9, ls=":")
    axa.annotate(f"capture {t.capture_area_m2 * 1e4:.1f} cm²",
                 xy=(x_slot * MM, t.capture_area_m2 * 1e4),
                 xytext=(x_slot * MM + 120, 190), fontsize=7.5, color=SLOT_C,
                 arrowprops=dict(arrowstyle="->", color=SLOT_C, lw=0.8))
    axa.annotate(f"DUMP {c['a_dump'] * 1e4:.0f} → "
                 f"{math.pi * (r_cham / MM) ** 2 * 1e4:.0f} cm²",
                 xy=(x_head * MM, 0.5 * (c["a_dump"] * 1e4 + 343)),
                 xytext=(x_head * MM + 300, 275), fontsize=7.5, color=GHOST,
                 arrowprops=dict(arrowstyle="->", color=GHOST, lw=0.8))
    axa.set_ylabel("flow area  [cm$^2$]", fontsize=8.5)
    axa.set_xlabel("station from nose tip  [mm]", fontsize=8.5)
    axa.tick_params(labelsize=8)
    axa.set_ylim(0, max(ys) * 1.18)
    axa.grid(True, lw=0.4, color="#dddddd", zorder=0)
    for sp in ("top", "right"):
        axa.spines[sp].set_visible(False)

    # ---- detail panel: the valve, its own axes so nothing overlaps ------
    axd.set_aspect("equal")
    paint(axd, detail=True)
    rfm, rlm = t.r_fore_m * MM, t.r_lip_m * MM
    axd.annotate("", xy=(x_slot * MM, rfm - 7),
                 xytext=((x_slot - t.stroke_m) * MM, rfm - 7),
                 arrowprops=dict(arrowstyle="<->", color=SEAL_C, lw=1.7))
    axd.text(x_slot * MM - 0.5 * t.stroke_m * MM, rfm - 9,
             f"STROKE {t.stroke_m * MM:.2f} mm", color=SEAL_C, fontsize=9,
             ha="center", va="top", weight="bold")
    axd.annotate(f"SLOT {t.gap_m * MM:.2f} mm radial\n"
                 f"{t.capture_area_m2 * 1e4:.1f} cm² capture",
                 xy=(x_slot * MM, 0.5 * (rfm + rlm)),
                 xytext=(x_slot * MM - 34, rlm + 20), fontsize=8,
                 color=SLOT_C, ha="center", weight="bold",
                 arrowprops=dict(arrowstyle="->", color=SLOT_C, lw=1.0))
    fay_mid = x_slot * MM + 0.5 * t.faying_land_m * MM
    axd.plot([x_slot * MM, (x_slot + t.faying_land_m) * MM],
             [rlm, t.faying_seat_outer_r_m * MM], color=SEAL_C, lw=3.2,
             solid_capstyle="butt", zorder=9)
    axd.annotate(f"{t.seal_half_angle_deg:.0f}° FAYING LAND\n"
                 f"{t.faying_land_m * MM:.0f} mm axial → "
                 f"{t.faying_slant_m * MM:.1f} mm slant\n"
                 f"{t.faying_area_m2 * 1e4:.0f} cm² contact",
                 xy=(fay_mid,
                     0.5 * (rlm + t.faying_seat_outer_r_m * MM)),
                 xytext=(x_slot * MM + 33, rlm + 6), fontsize=7.5,
                 color=SEAL_C, ha="left", va="center", weight="bold",
                 arrowprops=dict(arrowstyle="->", color=SEAL_C, lw=0.9))
    axd.set_xlim(x_slot * MM - 44, x_slot * MM + 66)
    axd.set_ylim(rfm - 28, rlm + 34)
    axd.set_yticks([])
    axd.set_xlabel("station from nose tip  [mm]", fontsize=8.5)
    axd.tick_params(labelsize=8)
    for sp in ("top", "right", "left"):
        axd.spines[sp].set_visible(False)
    axd.set_title("VALVE DETAIL — upper half, to scale", fontsize=9,
                  color=SEAL_C)

    ax.set_title(
        "Douglas Dart V4 — integrated propulsion with translating inlet "
        "shut-off, longitudinal section, to scale, mirrored\n"
        f"body {v.d_body * MM:.1f} mm  |  capture "
        f"{t.capture_area_m2 * 1e4:.1f} cm² at r {t.r_lip_m * MM:.1f} mm  |  "
        f"valve stroke {t.stroke_m * MM:.2f} mm  |  seal faying "
        f"{t.faying_area_m2 * 1e4:.0f} cm²  |  dump "
        f"{v.scalars['chamber flow area'][0] / (c['a_dump'] * 1e4):.2f}×  |  "
        f"flameholder {100 * fh['blockage']:.0f}% blockage  |  "
        f"boattail {-v.chains['oml'].segments[-1].half_angle_deg:.2f}°\n"
        "outer mould line identical in both valve states — only the sleeve "
        "and spar move.   Boom, reed valves and flameholder are ASSUMED.",
        fontsize=9.8, linespacing=1.5, pad=8)

    path.parent.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg", "pdf"):
        fig.savefig(path.with_suffix("." + ext), facecolor="white",
                    dpi=180 if ext == "png" else None)
        print(f"  wrote {path.with_suffix('.' + ext)}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--show", action="store_true")
    p.add_argument("--x-slot-mm", type=float, default=300.0)
    p.add_argument("--seal-deg", type=float, default=45.0)
    p.add_argument("--lip-frac", type=float, default=0.78)
    p.add_argument("--faying-mm", type=float, default=SEAL_FAYING_M * MM)
    args = p.parse_args(argv)

    dims, assumptions = load_inputs()
    v = build_vehicle(dims, assumptions)
    t = from_vehicle(v, args.x_slot_mm / MM,
                     args.lip_frac * 0.5 * v.d_body, args.seal_deg,
                     faying_land_m=args.faying_mm / MM)
    c = compose(v, t)
    rep = containment_report(v, t, c)

    print("\nFULL SECTION with translating inlet")
    print(f"  slot          {t.x_slot_m * MM:8.1f} mm at r "
          f"{t.r_lip_m * MM:.2f} mm   gap {t.gap_m * MM:.2f} mm")
    print(f"  stroke        {t.stroke_m * MM:8.2f} mm "
          f"({t.seal_half_angle_deg:.0f}° seat)")
    print(f"  capture       {t.capture_area_m2 * 1e4:8.3f} cm2")
    print(f"  dump plane    {c['a_dump'] * 1e4:8.2f} cm2 -> chamber "
          f"{v.scalars['chamber flow area'][0]:.2f} cm2  "
          f"({v.scalars['chamber flow area'][0] / (c['a_dump'] * 1e4):.2f}x)")
    print(f"  nose volume   {t.forebody_volume_m3 * 1e3:8.2f} L")
    print(f"  SEAL faying   {t.faying_land_m * MM:8.1f} mm land -> "
          f"{t.faying_slant_m * MM:.1f} mm slant, "
          f"{t.faying_area_m2 * 1e4:.1f} cm2 contact")
    print(f"    seat runs r {t.r_lip_m * MM:.2f} -> "
          f"{t.faying_seat_outer_r_m * MM:.2f} mm; headroom to the "
          f"{v.d_body * MM / 2:.1f} mm body: "
          f"{(0.5 * v.d_body - t.faying_seat_outer_r_m) * MM:.2f} mm")
    print(f"  pitot boom    {PROBE_LEN_M * MM:8.1f} mm x "
          f"{PROBE_DIA_M * MM:.0f} mm dia, statics at "
          f"{PROBE_STATIC_FRAC * PROBE_LEN_M / PROBE_DIA_M:.0f} probe dia")
    print(f"  reed valves   2 x {c['rv']['dia'] * MM:.0f} mm runners = "
          f"{c['rv']['port_area'] * 1e4:.1f} cm2 "
          f"({100 * c['rv']['port_frac']:.0f}% of chamber area)")
    print(f"  flameholder   x {c['fh']['x'] * MM:.1f} mm, r "
          f"{c['fh']['r'] * MM:.1f} mm, {100 * c['fh']['blockage']:.0f}% "
          f"blockage")
    print("\n  CONTAINMENT CHECK")
    over = rep["max_overshoot_m"] * MM
    print(f"    max duct overshoot beyond the OML   {over:8.4f} mm"
          f"   {'OK' if over <= 1e-6 else '<-- STILL VIOLATING'}")
    print(f"    min flow area through the diffuser  "
          f"{rep['min_area_m2'] * 1e4:8.2f} cm2 at x "
          f"{rep['min_area_x_m'] * MM:.1f} mm")
    if rep["min_area_m2"] < t.capture_area_m2 - 1e-9:
        print(f"    -> NECKS below capture ({t.capture_area_m2 * 1e4:.2f} "
              f"cm2): the real throat is the neck, not the slot")
    else:
        print("    -> no neck: the slot is the minimum area, as intended")
    draw(v, t, c, OUT / "propulsion_2d_full_translating", args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
