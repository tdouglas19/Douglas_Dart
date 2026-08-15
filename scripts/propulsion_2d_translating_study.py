"""EXPERIMENT: can a small stroke shut the ramjet inlet?

Feasibility study for the translating-centrebody inlet valve.  Prints the
trade, draws the chosen design open and sealed, and plots open-area vs
stroke.  Touches nothing the frozen 2D section draws.

    .venv/Scripts/python scripts/propulsion_2d_translating_study.py [--show]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from scripts.propulsion_2d.geometry import MM, build_vehicle, load_inputs  # noqa: E402
from scripts.propulsion_2d.translating_inlet import (  # noqa: E402
    SPAR_RADIUS_M, area_curve, build_chains, from_vehicle, sweep)

OUT = _REPO / "out_medium_model" / "propulsion_2d"

INK, ACCENT, GHOST = "#101010", "#404040", "#8a8a8a"
OPEN_C, SEAL_C = "#1f5fa8", "#a83232"


def print_baseline(v) -> None:
    cur = v.assumptions["inlet"]
    r_lip = 0.5 * v.scalars["cowl lip diameter"][0] / MM
    r_cb = 0.5 * v.scalars["centrebody max diameter"][0] / MM
    import math
    gap = r_lip - r_cb
    stroke = gap / math.tan(math.radians(cur["centrebody_cone_half_angle_deg"]))
    print("\nBASELINE (the frozen drawing, for reference)")
    print("-" * 72)
    print(f"  annulus at r ~ {0.5 * (r_lip + r_cb) * MM:6.1f} mm, "
          f"radial gap {gap * MM:5.2f} mm")
    print(f"  spike half-angle {cur['centrebody_cone_half_angle_deg']:.0f} deg"
          f"  ->  stroke to close would be {stroke * MM:6.2f} mm")
    print("  (this is what we are trying to beat)")


def print_sweep(v) -> None:
    rows = sweep(v)
    print("\nTRADE: slot station x seal angle   (lip at 0.86 x body radius)")
    print("-" * 104)
    print(f"{'x_slot':>8}{'seal':>7}{'r_lip':>8}{'r_fore':>8}{'gap':>7}"
          f"{'STROKE':>9}{'diffuser':>10}{'forebody':>10}{'nose vol':>10}"
          f"{'lip front':>11}")
    print(f"{'mm':>8}{'deg':>7}{'mm':>8}{'mm':>8}{'mm':>7}"
          f"{'mm':>9}{'mm':>10}{'deg':>10}{'L':>10}{'cm2':>11}")
    print("-" * 104)
    last_x = None
    for r in rows:
        if last_x is not None and r["x_slot_m"] != last_x:
            print()
        last_x = r["x_slot_m"]
        print(f"{r['x_slot_m'] * MM:>8.0f}{r['seal_deg']:>7.0f}"
              f"{r['r_lip_m'] * MM:>8.2f}{r['r_fore_m'] * MM:>8.2f}"
              f"{r['gap_m'] * MM:>7.2f}{r['stroke_m'] * MM:>9.2f}"
              f"{r['diffuser_len_m'] * MM:>10.1f}{r['forebody_deg']:>10.2f}"
              f"{r['forebody_vol_m3'] * 1e3:>10.2f}"
              f"{r['lip_frontal_m2'] * 1e4:>11.2f}")


def print_chosen(t) -> None:
    print("\nCHOSEN DESIGN  (slot 300 mm, 45 deg seat -- user selection)")
    print("-" * 72)
    print(f"  capture area (engine requirement)  {t.capture_area_m2 * 1e4:8.3f} cm2")
    print(f"  slot station                       {t.x_slot_m * MM:8.1f} mm")
    print(f"  cowl lip inner radius              {t.r_lip_m * MM:8.2f} mm")
    print(f"  fairing aft rim radius             {t.r_fore_m * MM:8.2f} mm")
    print(f"  annulus radial gap                 {t.gap_m * MM:8.2f} mm")
    print(f"  seal half-angle                    "
          f"{t.seal_half_angle_deg:8.1f} deg")
    print(f"  ** STROKE, open to sealed **       {t.stroke_m * MM:8.2f} mm")
    print()
    print(f"  diffuser length to chamber head    "
          f"{t.diffuser_length_m * MM:8.1f} mm")
    print(f"  fixed forebody half-angle          "
          f"{t.forebody_half_angle_deg:8.2f} deg")
    print(f"  fixed nose volume (avionics)       "
          f"{t.forebody_volume_m3 * 1e3:8.2f} L")
    print(f"  cowl lip frontal area              "
          f"{t.cowl_lip_frontal_area_m2 * 1e4:8.2f} cm2  "
          f"({100 * t.cowl_lip_frontal_fraction:.0f}% of capture)")
    print("    (the fairing-to-cowl radial offset is the SLOT, not a bluff")
    print("     step -- only the lip's own thickness presents frontal area)")

    print("\n  open area vs stroke:")
    print(f"    {'stroke mm':>10}{'area cm2':>11}{'% of open':>11}")
    for s, a, f in area_curve(t, 10):
        print(f"    {s * MM:>10.2f}{a * 1e4:>11.3f}{100 * f:>11.1f}")
    print(f"\n  90% shut at {t.stroke_for_area_fraction(0.10) * MM:.2f} mm, "
          f"99% shut at {t.stroke_for_area_fraction(0.01) * MM:.2f} mm "
          f"-- most of the closing happens early.")


def draw(t, v, path: Path, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    fig = plt.figure(figsize=(13.5, 9.4))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.15, 1.75, 1.0], hspace=0.42,
                          left=0.07, right=0.975, top=0.855, bottom=0.07)
    axo = fig.add_subplot(gs[0])     # whole forward end, context
    axd = fig.add_subplot(gs[1])     # zoomed detail of the valve
    axc = fig.add_subplot(gs[2])     # area vs stroke

    x_end = v.station("nose_fairing_end") * MM
    rb = 0.5 * v.d_body * MM

    def mirror(ax, chain, **kw):
        xs, rs = chain.polyline(0.0002)
        xs = [x * MM for x in xs]
        rs = [r * MM for r in rs]
        ax.plot(xs, rs, **kw)
        kw2 = {k: val for k, val in kw.items() if k != "label"}
        ax.plot(xs, [-r for r in rs], **kw2)

    def fill_sleeve(ax, stroke, col, ls, lab):
        ch = build_chains(t, v, stroke)["sleeve"]
        xs, rs = ch.polyline(0.0002)
        xs = [x * MM for x in xs]
        rs = [r * MM for r in rs]
        sp = SPAR_RADIUS_M * MM
        up = list(zip(xs, rs)) + [(xs[-1], sp), (xs[0], sp)]
        for pts in (up, [(x, -y) for x, y in up]):
            ax.add_patch(Polygon(pts, closed=True, facecolor=col, alpha=0.25,
                                 edgecolor=col, lw=1.7, ls=ls,
                                 label=lab, zorder=5))
            lab = None

    for ax, (x0, x1) in ((axo, (-10, x_end + 10)),
                         (axd, (t.x_slot_m * MM - 46, t.x_slot_m * MM + 46))):
        ax.set_aspect("equal")
        fixed = build_chains(t, v, 0.0)
        mirror(ax, fixed["fairing"], color=INK, lw=2.2)
        mirror(ax, fixed["cowl"], color=INK, lw=2.2)
        mirror(ax, fixed["cowl_inner"], color=GHOST, lw=1.5, ls="--")
        mirror(ax, fixed["spar"], color=ACCENT, lw=1.0)
        fill_sleeve(ax, 0.0, OPEN_C, "-", "sleeve RETRACTED — inlet OPEN")
        fill_sleeve(ax, t.stroke_m, SEAL_C, "--",
                    "sleeve EXTENDED — inlet SEALED")
        for sgn in (1, -1):
            ax.plot([t.x_slot_m * MM] * 2,
                    [sgn * t.r_fore_m * MM, sgn * t.r_lip_m * MM],
                    color="#009060", lw=3.2, solid_capstyle="butt", zorder=7)
        ax.plot([x0, x1], [0, 0], color=ACCENT, lw=0.7,
                ls=(0, (9, 4, 1.5, 4)))
        ax.set_xlim(x0, x1)
        ax.set_yticks([])
        for sp_ in ("top", "right", "left", "bottom"):
            ax.spines[sp_].set_visible(False)

    axo.set_ylim(-rb - 12, rb + 12)
    axo.add_patch(Rectangle((t.x_slot_m * MM - 46, -rb - 6), 92, 2 * rb + 12,
                            fill=False, edgecolor=SEAL_C, lw=1.0, ls=":"))
    axo.set_title("whole forward end — the valve is the boxed region",
                  fontsize=8.5, color=ACCENT)
    axo.set_xlabel("station from nose tip  [mm]", fontsize=8)
    axo.tick_params(labelsize=7.5)

    # ---- detail annotations --------------------------------------------
    xs_mm, rf, rl = t.x_slot_m * MM, t.r_fore_m * MM, t.r_lip_m * MM
    axd.set_ylim(rf - 26, rl + 26)          # one side only: the slot region
    axd.annotate(f"SLOT  {t.gap_m * MM:.2f} mm radial\n"
                 f"{t.capture_area_m2 * 1e4:.1f} cm² capture",
                 xy=(xs_mm, 0.5 * (rf + rl)), xytext=(xs_mm - 34, rl + 17),
                 fontsize=8.5, color="#009060", ha="center", weight="bold",
                 arrowprops=dict(arrowstyle="->", color="#009060", lw=1.1))
    axd.annotate("", xy=(xs_mm, rf - 9),
                 xytext=(xs_mm - t.stroke_m * MM, rf - 9),
                 arrowprops=dict(arrowstyle="<->", color=SEAL_C, lw=1.8))
    axd.text(xs_mm - 0.5 * t.stroke_m * MM, rf - 11.5,
             f"STROKE {t.stroke_m * MM:.2f} mm", color=SEAL_C, fontsize=9.5,
             ha="center", va="top", weight="bold")
    axd.annotate(f"{t.seal_half_angle_deg:.0f}° seat",
                 xy=(xs_mm + 6, rl + 6), xytext=(xs_mm + 26, rl + 15),
                 fontsize=8, color=GHOST,
                 arrowprops=dict(arrowstyle="->", color=GHOST, lw=0.8))
    axd.legend(loc="lower right", fontsize=8, framealpha=0.92)
    axd.set_title(
        f"DETAIL — slot at x {xs_mm:.0f} mm, r {rl:.1f} mm   "
        f"(upper half; drawn to scale)", fontsize=9, color=ACCENT)
    axd.set_xlabel("station from nose tip  [mm]", fontsize=8)
    axd.tick_params(labelsize=7.5)

    # ---- area vs stroke ------------------------------------------------
    curve = area_curve(t, 200)
    axc.plot([s * MM for s, _, _ in curve], [100 * f for _, _, f in curve],
             color=INK, lw=2.0)
    axc.fill_between([s * MM for s, _, _ in curve], 0,
                     [100 * f for _, _, f in curve], color="#eeeeee")
    for frac, lab in ((10.0, "90% shut"), (1.0, "99% shut")):
        s = t.stroke_for_area_fraction(frac / 100.0) * MM
        axc.axvline(s, color=ACCENT, lw=0.8, ls=":")
        axc.annotate(f"{lab}\n{s:.2f} mm", xy=(s, frac), xytext=(s - 1.1, 40),
                     fontsize=7.5, color=ACCENT, ha="right",
                     arrowprops=dict(arrowstyle="->", color=ACCENT, lw=0.7))
    axc.set_xlabel("sleeve stroke from retracted  [mm]", fontsize=9)
    axc.set_ylabel("inlet open area\n[% of capture]", fontsize=9)
    axc.set_xlim(0, t.stroke_m * MM)
    axc.set_ylim(0, 105)
    axc.grid(True, lw=0.4, color="#dddddd")
    axc.tick_params(labelsize=8)
    for sp_ in ("top", "right"):
        axc.spines[sp_].set_visible(False)

    fig.suptitle(
        "EXPERIMENT — translating centrebody as a ramjet inlet shut-off\n"
        f"slot {xs_mm:.0f} mm at r {rl:.1f} mm  |  "
        f"{t.seal_half_angle_deg:.0f}° seat  |  "
        f"STROKE {t.stroke_m * MM:.2f} mm  (baseline geometry needs 28.22)  |  "
        f"fixed nose volume {t.forebody_volume_m3 * 1e3:.2f} L\n"
        "outer mould line identical in both states — only the sleeve moves",
        fontsize=10.5, linespacing=1.5, y=0.985)

    path.parent.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg"):
        fig.savefig(path.with_suffix("." + ext), facecolor="white",
                    dpi=170 if ext == "png" else None)
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
    p.add_argument("--lip-frac", type=float, default=0.86,
                   help="cowl lip inner radius as a fraction of body radius")
    args = p.parse_args(argv)

    dims, assumptions = load_inputs()
    v = build_vehicle(dims, assumptions)

    print("\nEXPERIMENT: translating centrebody as a ramjet inlet shut-off")
    print(f"  capture requirement from the flight model: "
          f"{dims['inlet']['implied_capture_area_m2'] * 1e4:.3f} cm2")
    print_baseline(v)
    print_sweep(v)

    t = from_vehicle(v, args.x_slot_mm / MM,
                     args.lip_frac * 0.5 * v.d_body, args.seal_deg)
    print_chosen(t)
    draw(t, v, OUT / "translating_inlet_study", args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
