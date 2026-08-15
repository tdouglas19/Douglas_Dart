"""Interactive 2D propulsion section viewer -- the MATLAB .fig analogue.

A live matplotlib window, not a rendered image:

  * zoom / pan with the standard toolbar; the geometry underneath is
    analytic, so zooming reveals the 1.0 mm duct wall rather than blurring it
  * live cursor readout in the toolbar: station, radius, diameter, local flow
    area, A/A_body, A/A_throat and which region of the engine you are in
  * LEFT-CLICK pins a datatip with exact values, snapping to the nearest
    contour and to a named station when one is within tolerance
  * RIGHT-CLICK removes the nearest datatip
  * number keys toggle layers, 's' saves vector SVG + PDF + PNG, 'h' help

Everything ASSUMED is drawn dashed grey.  Everything solid came from
structural_dimensions.json or is a closed-form consequence of it.
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

from .geometry import ASSUMED, MM, Vehicle

# --- greyscale-safe palette ----------------------------------------------
INK = "#101010"
ACCENT = "#404040"
GHOST = "#8a8a8a"          # ASSUMED geometry
SKIN_FILL = "#e9e9e9"
GAS_FILL = "#ffffff"
CHAMBER_FILL = "#c9c9c9"
FUEL_FILL = "#b6b6b6"
CB_FILL = "#d5d5d5"
PIN_FILL = "#fffbe6"

SNAP_TOL_MM = 6.0          # station snap distance for datatips
FIG_WIDTH_IN = 17.0
AREA_PANEL_IN = 1.55

# stations share x in this vehicle (the nose fairing ends exactly where the
# chamber starts), so the label row is built from merged, shortened names
SHORT_NAME = {
    "nose_tip": "nose tip",
    "cowl_lip": "cowl lip",
    "centrebody_max_dia_end": "cb barrel end",
    "nose_fairing_end": "fairing end",
    "chamber_start": "chamber start",
    "chamber_end": "chamber end",
    "cone_end": "cone end",
    "tailpipe_end": "tailpipe end",
    "nozzle_throat": "throat",
    "nozzle_exit": "nozzle exit",
}


def _mirror_fill(ax, xs, rs, **kw):
    pts = list(zip(xs, rs)) + [(x, -r) for x, r in zip(reversed(xs),
                                                       reversed(rs))]
    ax.add_patch(Polygon(pts, closed=True, **kw))
    return ax.patches[-1]


def _mirror_line(ax, xs, rs, **kw):
    (ln,) = ax.plot(xs, rs, **kw)
    kw2 = {k: val for k, val in kw.items() if k != "label"}
    ax.plot(xs, [-r for r in rs], **kw2)
    return ln


def _band_artists(ax, xs, r_out, r_in, **kw):
    """Fill the annulus between r_in and r_out on BOTH sides of the axis."""
    up = list(zip(xs, r_out)) + list(zip(reversed(xs), reversed(r_in)))
    return [ax.add_patch(Polygon(pts, closed=True, **kw))
            for pts in (up, [(x, -y) for x, y in up])]


# ==========================================================================
# viewer
# ==========================================================================

class SectionViewer:

    LAYERS = ["oml", "centrebody", "diffuser", "flow", "walls", "fuel",
              "stations", "dims"]

    def __init__(self, v: Vehicle, out_dir: Path):
        self.v = v
        self.out_dir = out_dir
        self.pins: list = []
        self.visible = {k: True for k in self.LAYERS}
        self.artists: dict[str, list] = {k: [] for k in self.LAYERS}

        self.a_body = v.a_body * 1e4
        self.a_throat = math.pi * (
            0.5 * v.dims["internal_flowpath"]["tailpipe_diameter_m"]) ** 2 * 1e4

        # ---- layout, derived from the content so no dead band appears ----
        r_top = 0.5 * v.d_body * MM
        self.r_top = r_top

        # Stations too close to label side by side get staggered outward.
        # Work the tiers out FIRST so the dimension rows can be pushed clear
        # of the deepest one instead of being written over.
        self.station_tier = self._station_tiers()
        self.tier_step = 62.0
        deepest = max(self.station_tier.values(), default=0)

        self.y_seg = -(r_top + 95.0 + deepest * self.tier_step)
        self.y_all = self.y_seg - 82.0      # overall-length row
        self.xlim = (-45.0, v.x_max * MM + 55.0)
        self.ylim = (self.y_all - 48.0, r_top + 225.0)

        # the section keeps equal aspect, so its axes height is FIXED by the
        # data limits.  Size the figure to that instead of leaving a dead
        # band above and below the drawing.
        x_span = self.xlim[1] - self.xlim[0]
        y_span = self.ylim[1] - self.ylim[0]
        h_sec = FIG_WIDTH_IN * y_span / x_span
        title_in, xlabel_in, gap_in = 0.66, 0.55, 0.26
        fig_h = h_sec + AREA_PANEL_IN + title_in + xlabel_in + gap_in

        self.fig, (self.ax, self.axa) = plt.subplots(
            2, 1, figsize=(FIG_WIDTH_IN, fig_h), sharex=True,
            gridspec_kw=dict(height_ratios=[h_sec, AREA_PANEL_IN],
                             hspace=gap_in / (0.5 * (h_sec + AREA_PANEL_IN)),
                             left=0.035, right=0.985,
                             top=1.0 - title_in / fig_h,
                             bottom=xlabel_in / fig_h))

        self._draw_section()
        self._draw_area()
        self._wire()

    def _station_tiers(self) -> dict[float, int]:
        """Label tier per station-x (mm).  Stations closer together than they
        can be labelled side by side alternate outward -- the divergent
        nozzle is only ~10 mm long on a 2274 mm vehicle, so 'throat' and
        'nozzle exit' would otherwise print on top of each other."""
        xs = sorted({round(st.x * MM, 4) for st in self.v.stations})
        crowd = 0.025 * self.v.x_max * MM
        tiers: dict[float, int] = {}
        prev, tier = None, 0
        for x in xs:
            tier = (tier + 1) % 2 if (prev is not None
                                      and x - prev < crowd) else 0
            tiers[x] = tier
            prev = x
        return tiers

    # ---------------------------------------------------------------- draw
    def _reg(self, layer, *artists):
        for a in artists:
            if a is not None:
                self.artists[layer].append(a)

    def _draw_section(self):
        v, ax = self.v, self.ax
        ch = v.chains
        x_lip = v.station("cowl_lip")
        x_cham0 = v.station("chamber_start")
        x_cham1 = v.station("chamber_end")
        x_cone1 = v.station("cone_end")
        x_pipe1 = v.station("tailpipe_end")
        x_throat = v.station("nozzle_throat")
        x_exit = v.station("nozzle_exit")
        r_top = self.r_top

        ax.set_aspect("equal")
        ax.set_facecolor("white")

        # ---- outer mould line, filled solid then hollowed out -----------
        xs_o, rs_o = ch["oml"].polyline()
        xs_o = [x * MM for x in xs_o]
        rs_o = [r * MM for r in rs_o]
        self._reg("oml", _mirror_fill(ax, xs_o, rs_o, facecolor=SKIN_FILL,
                                      edgecolor="none", zorder=1))

        # ---- annular subsonic diffuser (gas) ----------------------------
        n = 240
        xs_d = [x_lip + (x_cham0 - x_lip) * i / n for i in range(n + 1)]
        self._reg("diffuser", *_band_artists(
            ax, [x * MM for x in xs_d],
            [ch["cowl_inner"].r_at(x) * MM for x in xs_d],
            [ch["centrebody"].r_at(x) * MM for x in xs_d],
            facecolor=GAS_FILL, edgecolor="none", zorder=2))

        # ---- chamber + cone + tailpipe (gas) ----------------------------
        xs_f, rs_f = ch["flow"].polyline()
        xs_f = [x * MM for x in xs_f]
        rs_f = [r * MM for r in rs_f]
        self._reg("flow", _mirror_fill(ax, xs_f, rs_f, facecolor=GAS_FILL,
                                       edgecolor="none", zorder=2))
        r_cham = ch["flow"].segments[0].r0 * MM
        self._reg("flow", ax.add_patch(Rectangle(
            (x_cham0 * MM, -r_cham), (x_cham1 - x_cham0) * MM, 2 * r_cham,
            facecolor=CHAMBER_FILL, edgecolor="none", zorder=2.5)))

        # ---- fuel annulus: duct OD <-> skin ID over the model's extent ---
        n = 260
        xs_a = [x_cham1 + (x_pipe1 - x_cham1) * i / n for i in range(n + 1)]
        self._reg("fuel", *_band_artists(
            ax, [x * MM for x in xs_a],
            [ch["skin_id"].r_at(x) * MM for x in xs_a],
            [ch["duct_od"].r_at(x) * MM for x in xs_a],
            facecolor=FUEL_FILL, edgecolor=ACCENT, lw=0.5, hatch="///",
            zorder=3))

        # ---- centrebody, solid, on top ----------------------------------
        xs_c, rs_c = ch["centrebody"].polyline()
        self._reg("centrebody", _mirror_fill(
            ax, [x * MM for x in xs_c], [r * MM for r in rs_c],
            facecolor=CB_FILL, edgecolor=GHOST, lw=1.4, ls="--", zorder=4))

        # ---- wall lines --------------------------------------------------
        for nm in ("duct_od", "skin_id"):
            xs_w, rs_w = ch[nm].polyline()
            self._reg("walls", _mirror_line(
                ax, [x * MM for x in xs_w], [r * MM for r in rs_w],
                color=ACCENT, lw=0.8, zorder=5))

        # ---- flowpath + cowl inner lines --------------------------------
        self._reg("flow", _mirror_line(ax, xs_f, rs_f, color=INK, lw=1.9,
                                       zorder=6))
        xs_ci, rs_ci = ch["cowl_inner"].polyline()
        self._reg("diffuser", _mirror_line(
            ax, [x * MM for x in xs_ci], [r * MM for r in rs_ci],
            color=GHOST, lw=1.6, ls="--", zorder=6))

        # ---- OML line, drawn last so it sits on top ---------------------
        self._reg("oml", _mirror_line(ax, xs_o, rs_o, color=INK, lw=2.3,
                                      zorder=7))

        # capture plane
        r_lip = ch["cowl_inner"].segments[0].r0 * MM
        r_cb_lip = ch["centrebody"].r_at(x_lip) * MM
        for sgn in (1, -1):
            self._reg("diffuser", ax.plot(
                [x_lip * MM, x_lip * MM], [sgn * r_cb_lip, sgn * r_lip],
                color=GHOST, lw=1.0, ls=":", zorder=7)[0])

        # dump plane: the diffuser wall stops short of the chamber wall and
        # the flow expands suddenly.  A real feature, so draw the step.
        r_dump = ch["cowl_inner"].segments[-1].r1 * MM
        if r_dump < r_cham - 1e-9:
            for sgn in (1, -1):
                self._reg("diffuser", ax.plot(
                    [x_cham0 * MM, x_cham0 * MM],
                    [sgn * r_dump, sgn * r_cham], color=GHOST, lw=1.6,
                    ls="--", zorder=7)[0])
            self._reg("diffuser", ax.annotate(
                f"DUMP {2 * r_dump:.0f} -> {2 * r_cham:.0f} mm",
                xy=(x_cham0 * MM, 0.5 * (r_dump + r_cham)),
                xytext=(x_cham0 * MM - 30, r_top + 8), fontsize=6.6,
                color=GHOST, ha="right", va="bottom",
                arrowprops=dict(arrowstyle="->", color=GHOST, lw=0.7)))

        # annular base: the boattail does NOT close onto the nozzle
        r_base = ch["oml"].segments[-1].r1 * MM
        r_duct_ex = ch["duct_od"].r_at(x_exit) * MM
        for sgn in (1, -1):
            self._reg("oml", ax.plot(
                [x_exit * MM, x_exit * MM], [sgn * r_duct_ex, sgn * r_base],
                color=INK, lw=2.3, zorder=7)[0])

        ax.plot(list(self.xlim), [0, 0], color=ACCENT, lw=0.7,
                ls=(0, (9, 4, 1.5, 4)), zorder=8)

        # ---- stations: merge names that share an x ----------------------
        merged: dict[float, list] = {}
        for st in v.stations:
            key = round(st.x * MM, 4)
            merged.setdefault(key, []).append(st)
        for xmm, group in sorted(merged.items()):
            src = ASSUMED if all(s.source == ASSUMED for s in group) else ACCENT
            col = GHOST if src is ASSUMED else ACCENT
            name = " / ".join(SHORT_NAME.get(s.name, s.name) for s in group)
            dy = 16 + self.station_tier.get(round(xmm, 4), 0) * self.tier_step
            self._reg("stations",
                      ax.axvline(xmm, color=col, lw=0.6, ls=":", zorder=0.5),
                      ax.text(xmm, r_top + dy, name, rotation=90,
                              ha="center", va="bottom", fontsize=6.8,
                              color=col, zorder=9),
                      ax.text(xmm, -r_top - dy, f"{xmm:.1f}", rotation=90,
                              ha="center", va="top", fontsize=6.6,
                              color=col, zorder=9))

        # ---- dimension rows ----------------------------------------------
        for a, b, lab in [
                (0, x_lip * MM, "spike"),
                (x_lip * MM, x_cham0 * MM, "diffuser"),
                (x_cham0 * MM, x_cham1 * MM, "chamber"),
                (x_cham1 * MM, x_cone1 * MM, "cone"),
                (x_cone1 * MM, x_throat * MM, "tailpipe"),
                (x_pipe1 * MM, x_exit * MM, "boattail")]:
            self._reg("dims", *self._dim(ax, a, b, self.y_seg,
                                         f"{lab}\n{b - a:.1f}", fs=7.4))
        self._reg("dims", *self._dim(
            ax, 0, x_exit * MM, self.y_all,
            f"OVERALL {x_exit * MM:.1f} mm   (fineness "
            f"{x_exit / v.d_body:.2f})", fs=9.0))

        # ---- diameter dimensions, each in a station where it is real -----
        self._reg("dims", *self._vdim(
            ax, (x_cham0 + 0.32 * (x_cham1 - x_cham0)) * MM, r_cham,
            f"chamber flow dia {2 * r_cham:.1f}"))
        self._reg("dims", *self._vdim(
            ax, (x_cham0 + 0.72 * (x_cham1 - x_cham0)) * MM, r_top,
            f"body dia {2 * r_top:.1f}"))
        r_th = ch["flow"].r_at(x_throat) * MM
        self._reg("dims", *self._vdim(
            ax, (x_cone1 + 0.16 * (x_pipe1 - x_cone1)) * MM, r_th,
            f"throat dia {2 * r_th:.2f}"))
        r_ex = ch["flow"].r_at(x_exit) * MM
        self._reg("dims", *self._vdim(
            ax, x_exit * MM, r_ex, f"exit dia {2 * r_ex:.2f}", fs=6.8))
        self._reg("dims", *self._vdim(
            ax, x_lip * MM, r_lip, f"lip dia {2 * r_lip:.1f}", fs=6.8,
            side=-1))

        ax.set_xlim(*self.xlim)
        ax.set_ylim(*self.ylim)
        ax.set_yticks([])
        for sp in ("top", "right", "left", "bottom"):
            ax.spines[sp].set_visible(False)

        sc = v.scalars
        ax.set_title(
            "Douglas Dart V4 -- integrated propulsion, longitudinal section, "
            "to scale, mirrored about the centreline\n"
            f"body {sc['body diameter (OML)'][0]:.1f} mm  |  overall "
            f"{sc['overall length'][0]:.1f} mm  |  capture "
            f"{sc['annular capture area'][0]:.1f} cm2 = "
            f"{sc['capture / throat area'][0]:.2f} x throat  |  diffuser AR "
            f"{sc['diffuser area ratio (chamber/capture)'][0]:.2f}  |  "
            f"boattail {sc['boattail half-angle'][0]:.2f} deg\n"
            "dashed grey = ASSUMED (not in the model)     "
            "left-click pins a datatip, right-click removes, "
            "keys 1-8 toggle layers, 'h' for help",
            fontsize=9.8, linespacing=1.5, pad=8)

    def _draw_area(self):
        v, ax = self.v, self.axa
        x_lip, x_exit = v.station("cowl_lip"), v.station("nozzle_exit")
        xs, ys = [], []
        n = 1600
        for i in range(n + 1):
            x = x_lip + (x_exit - x_lip) * i / n
            a = v.flow_area(x)
            if a is not None:
                xs.append(x * MM)
                ys.append(a * 1e4)
        ax.fill_between(xs, 0, ys, color="#eeeeee", zorder=1)
        ax.plot(xs, ys, color=INK, lw=1.7, zorder=3)
        for val, lab, col, ls in ((self.a_throat, "A_throat", ACCENT, "--"),
                                  (self.a_body, "A_body", GHOST, ":")):
            ax.axhline(val, color=col, lw=0.8, ls=ls, zorder=2)
            ax.text(self.xlim[1] - 8, val, lab, fontsize=7, va="center",
                    ha="right", color=col, zorder=4,
                    bbox=dict(facecolor="white", edgecolor="none", pad=1.0))
        for st in v.stations:
            ax.axvline(st.x * MM, color=GHOST, lw=0.5, ls=":", zorder=0.5)
        ax.set_ylabel("flow area  [cm$^2$]", fontsize=8.5)
        ax.set_xlabel("station from nose tip  [mm]", fontsize=8.5)
        ax.tick_params(labelsize=8)
        ax.set_ylim(0, max(ys) * 1.16)
        ax.grid(True, lw=0.4, color="#dddddd", zorder=0)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)

    # ------------------------------------------------------- annotations
    @staticmethod
    def _dim(ax, x0, x1, y, text, *, fs=7.5):
        out = [ax.annotate("", xy=(x0, y), xytext=(x1, y),
                           arrowprops=dict(arrowstyle="<->", color=ACCENT,
                                           lw=0.8, shrinkA=0, shrinkB=0))]
        for x in (x0, x1):
            out.append(ax.plot([x, x], [y - 5, y + 5], color=ACCENT,
                               lw=0.8)[0])
        out.append(ax.text(0.5 * (x0 + x1), y - 9, text, ha="center",
                           va="top", fontsize=fs, color=ACCENT))
        return out

    @staticmethod
    def _vdim(ax, x, r, text, *, fs=7.2, side=1):
        out = [ax.annotate("", xy=(x, -r), xytext=(x, r),
                           arrowprops=dict(arrowstyle="<->", color=ACCENT,
                                           lw=0.8, shrinkA=0, shrinkB=0))]
        out.append(ax.text(x + 7 * side, 0, text,
                           ha="left" if side > 0 else "right", va="center",
                           fontsize=fs, color=ACCENT, rotation=90, zorder=11,
                           bbox=dict(facecolor="white", edgecolor="none",
                                     pad=1.0, alpha=0.86)))
        return out

    # ------------------------------------------------------------ readout
    def _describe(self, x_mm: float, y_mm: float) -> str:
        v = self.v
        x = x_mm / MM
        if not (0.0 <= x <= v.x_max):
            return f"x = {x_mm:8.2f} mm   (outside the body)"
        parts = [f"x = {x_mm:8.2f} mm", f"r = {abs(y_mm):7.2f} mm",
                 v.region_at(x)]
        a = v.flow_area(x)
        if a is not None:
            r = v.flow_radius(x)
            parts += [f"duct dia {2 * r * MM:6.2f}",
                      f"A {a * 1e4:7.2f} cm2",
                      f"A/A_body {a * 1e4 / self.a_body:5.3f}",
                      f"A/A_th {a * 1e4 / self.a_throat:5.3f}"]
        cb = v.chains["centrebody"].r_at(x)
        if cb:
            parts.append(f"centrebody dia {2 * cb * MM:.2f}")
        return "   |   ".join(parts)

    PICK_RADIUS_PX = 45.0
    CURVE_PICK_SAMPLES = 600

    def _nearest_point(self, x_mm, y_mm):
        """Nearest point ON a contour -- not merely on a drawing vertex.

        Distance is measured in DISPLAY space so the pick feels the same at
        any zoom or aspect.  Straight segments are projected onto exactly
        (the axes transform is affine, so the display-space parameter is the
        data-space parameter); curves are densely sampled.  Sampling the
        drawing polyline instead would make the long straights unpickable --
        the 1068 mm tailpipe has only two vertices.
        """
        trans = self.ax.transData.transform
        px, py = trans((x_mm, y_mm))
        sgn = 1.0 if y_mm >= 0 else -1.0
        best = None

        for name, ch in self.v.chains.items():
            for seg in ch.segments:
                if seg.is_straight():
                    ax0, ay0 = trans((seg.x0 * MM, sgn * seg.r0 * MM))
                    ax1, ay1 = trans((seg.x1 * MM, sgn * seg.r1 * MM))
                    dx, dy = ax1 - ax0, ay1 - ay0
                    den = dx * dx + dy * dy
                    t = 0.0 if den == 0.0 else (
                        ((px - ax0) * dx + (py - ay0) * dy) / den)
                    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                    d = math.hypot(ax0 + t * dx - px, ay0 + t * dy - py)
                    xd = seg.x0 + t * seg.length
                    rd = seg.r_at(xd)
                else:
                    d = xd = rd = None
                    n = self.CURVE_PICK_SAMPLES
                    for i in range(n + 1):
                        xx = seg.x0 + seg.length * i / n
                        rr = seg.r_at(xx)
                        tx, ty = trans((xx * MM, sgn * rr * MM))
                        dd = math.hypot(tx - px, ty - py)
                        if d is None or dd < d:
                            d, xd, rd = dd, xx, rr
                if best is None or d < best[0]:
                    best = (d, name, xd * MM, sgn * rd * MM, seg)

        if best is None or best[0] > self.PICK_RADIUS_PX:
            return None
        return best[1:]

    def _pin(self, x_mm, y_mm):
        near = self._nearest_point(x_mm, y_mm)
        if near is None:
            return
        chain, px, py, seg = near

        snap = ""
        for st in self.v.stations:
            if abs(st.x * MM - px) <= SNAP_TOL_MM:
                r = self.v.chains[chain].r_at(st.x)
                px = st.x * MM
                py = math.copysign((r or 0.0) * MM, py if py else 1.0)
                snap = f"\nSTATION {st.name}  [{st.source}]"
                break

        a = self.v.flow_area(px / MM)
        lines = [f"{chain}: {seg.name if seg else '?'}"
                 f"  [{seg.source if seg else '?'}]",
                 f"x = {px:.3f} mm   ({px / 25.4:.4f} in)",
                 f"r = {abs(py):.3f} mm   dia = {2 * abs(py):.3f} mm"]
        if a is not None:
            lines.append(f"A_flow = {a * 1e4:.2f} cm2   "
                         f"A/A_th = {a * 1e4 / self.a_throat:.3f}")
        if seg is not None and seg.is_straight() and abs(seg.r1 - seg.r0) > 1e-9:
            lines.append(f"local half-angle = {seg.half_angle_deg:.2f} deg")
        if seg is not None and seg.detail:
            lines.append(seg.detail)

        off = 62 if py >= 0 else -62
        ann = self.ax.annotate(
            "\n".join(lines) + snap, xy=(px, py),
            xytext=(px + 26, py + off), fontsize=7.0, ha="left",
            va="bottom" if py >= 0 else "top", zorder=30,
            bbox=dict(boxstyle="round,pad=0.42", facecolor=PIN_FILL,
                      edgecolor=INK, lw=0.7, alpha=0.96),
            arrowprops=dict(arrowstyle="->", color=INK, lw=0.8,
                            shrinkA=0, shrinkB=2))
        self.pins.append((ann, px, py))
        self.fig.canvas.draw_idle()

    def _unpin(self, x_mm, y_mm):
        if not self.pins:
            return
        i = min(range(len(self.pins)),
                key=lambda k: math.hypot(self.pins[k][1] - x_mm,
                                         self.pins[k][2] - y_mm))
        self.pins.pop(i)[0].remove()
        self.fig.canvas.draw_idle()

    # --------------------------------------------------------------- wire
    def _wire(self):
        self.ax.format_coord = lambda x, y: self._describe(x, y)
        self.axa.format_coord = lambda x, y: (
            f"x = {x:8.2f} mm   |   A = {y:8.2f} cm2   |   "
            f"{self.v.region_at(max(min(x / MM, self.v.x_max), 0.0))}")
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _on_click(self, ev):
        if ev.inaxes is not self.ax or ev.xdata is None:
            return
        if self.fig.canvas.toolbar and self.fig.canvas.toolbar.mode:
            return                      # zoom / pan is active
        if ev.button == 1:
            self._pin(ev.xdata, ev.ydata)
        elif ev.button == 3:
            self._unpin(ev.xdata, ev.ydata)

    def _on_key(self, ev):
        if ev.key in [str(i) for i in range(1, len(self.LAYERS) + 1)]:
            layer = self.LAYERS[int(ev.key) - 1]
            self.visible[layer] = not self.visible[layer]
            for a in self.artists[layer]:
                a.set_visible(self.visible[layer])
            self.fig.canvas.draw_idle()
        elif ev.key == "c":
            while self.pins:
                self.pins.pop()[0].remove()
            self.fig.canvas.draw_idle()
        elif ev.key == "s":
            self.save_vector()
        elif ev.key == "h":
            print(self.help_text())

    def help_text(self) -> str:
        keys = "  ".join(f"{i + 1}={n}" for i, n in enumerate(self.LAYERS))
        return ("\n  left-click   pin a datatip (snaps to contours and to "
                "named stations)\n"
                "  right-click  remove the nearest datatip\n"
                "  c            clear all datatips\n"
                "  s            save vector SVG + PDF + PNG\n"
                "  h            this help\n"
                f"  layer keys   {keys}\n")

    def save_vector(self):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        for ext in ("svg", "pdf", "png"):
            p = self.out_dir / f"propulsion_2d_section.{ext}"
            self.fig.savefig(p, facecolor="white",
                             dpi=180 if ext == "png" else None)
            print(f"  wrote {p}")
