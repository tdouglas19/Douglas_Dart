"""CLI for the 2D propulsion section.

    .venv/Scripts/python -m scripts.propulsion_2d --show
    .venv/Scripts/python -m scripts.propulsion_2d --export
    .venv/Scripts/python -m scripts.propulsion_2d --save-vector

With no flags it prints the tables and does nothing else, which is the fast
answer to "what does the geometry say about X".
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from scripts.propulsion_2d.geometry import (  # noqa: E402
    ASSUMED, DEFAULT_ASSUMPTIONS, DEFAULT_DIMS, MM, build_vehicle,
    diffuser_diagnostics, expansion_trade, export_area_csv,
    export_contour_csv, export_geometry_json, export_stations_csv,
    load_inputs, nozzle_diagnostics, reconciliation)

OUT_DIR = _REPO / "out_medium_model" / "propulsion_2d"


# ==========================================================================
# tables
# ==========================================================================

def _rule(w: int) -> str:
    return "-" * w


def print_stations(v) -> None:
    print("\nSTATIONS  (from the nose tip)")
    print(f"{'station':<24}{'mm':>12}{'in':>10}  {'source':<8} detail")
    print(_rule(112))
    for st in v.stations:
        print(f"{st.name:<24}{st.x * MM:>12.3f}{st.x / 0.0254:>10.3f}  "
              f"{st.source:<8} {st.detail}")


def print_scalars(v) -> None:
    print("\nKEY DIMENSIONS")
    w = max(len(k) for k in v.scalars)
    print(f"{'quantity':<{w}}{'value':>13}  {'unit':<5}source")
    print(_rule(w + 28))
    for k, (val, unit, src) in v.scalars.items():
        print(f"{k:<{w}}{val:>13.4f}  {unit:<5}{src}")


def print_reconciliation(v) -> None:
    rows = reconciliation(v)
    print("\nMODEL vs AS-DRAWN  (where the two disagree, and why)")
    w = max(len(r[0]) for r in rows)
    print(f"{'quantity':<{w}}{'model':>11}{'as drawn':>11}{'delta %':>10}  "
          f"{'unit':<5}why")
    print(_rule(w + 120))
    for name, mv, dv, unit, why in rows:
        d = (100.0 * (dv - mv) / mv) if mv else float("nan")
        ds = "     --" if d != d else f"{d:>9.1f}"
        print(f"{name:<{w}}{mv:>11.3f}{dv:>11.3f}{ds}  {unit:<5}{why}")


def print_diffuser(v) -> None:
    d = diffuser_diagnostics(v)
    print(f"\nINTAKE DIFFUSER  (annular, lip -> chamber head; policy "
          f"'{d['policy']}')")
    print(_rule(78))
    print(f"  length                        {d['length_m'] * MM:9.2f} mm")
    print(f"  area in  (annular capture)    {d['area_in_m2'] * 1e4:9.2f} cm2")
    print(f"  area out (dump plane)         {d['area_out_m2'] * 1e4:9.2f} cm2")
    print(f"  area ratio ACROSS THE WALL    {d['area_ratio']:9.3f}")
    print(f"  area ratio to the chamber     "
          f"{d['area_ratio_to_chamber_head']:9.3f}"
          f"   (the rest is taken by the dump)")
    print(f"  equivalent-cone half-angle,"
          f"\n    mean                        {d['mean_half_angle_deg']:9.2f} deg")
    print(f"    max                         {d['max_half_angle_deg']:9.2f} deg"
          f"   at x = {d['max_half_angle_x_m'] * MM:.1f} mm")
    sc = v.scalars
    if sc["dump sudden-expansion area ratio"][0] > 1.0 + 1e-9:
        print(f"  -> then DUMPS into the chamber: "
              f"{sc['diffuser exit dia (dump plane)'][0]:.1f} mm -> "
              f"{sc['chamber flow diameter'][0]:.1f} mm, a sudden expansion "
              f"of {sc['dump sudden-expansion area ratio'][0]:.2f}x")
        print("     (the area distribution is genuinely DISCONTINUOUS there; "
              "that is what a dump combustor is)")
    worst = d["max_half_angle_deg"]
    verdict = ("within the ~7 deg no-separation rule of thumb"
               if worst <= d["target_half_angle_deg"] + 1e-6
               else "ABOVE the ~7 deg rule of thumb -- expect separation and a "
                    "total-pressure loss this repo does not model"
               if worst <= 10.0 else
               "WELL above the ~10 deg limit -- this diffuser will separate")
    print(f"  -> {verdict}")
    extra = d["extra_length_for_target_m"] * MM
    nose = v.dims["body"]["nose_fairing_length_m"] * MM
    print(f"  -> to diffuse ALL the way to the chamber head at "
          f"{d['target_half_angle_deg']:.0f} deg instead of dumping would "
          f"need {d['length_for_target_m'] * MM:.1f} mm,")
    print(f"     i.e. {extra:+.1f} mm of nose fairing: {nose:.1f} -> "
          f"{nose + extra:.1f} mm "
          f"({(nose + extra) / (v.d_body * MM):.2f} body diameters) -- not "
          f"worth it; dump instead.")
    print("  (geometry only; there is no boundary-layer calculation anywhere "
          "in this repo)")


def print_nozzle(v) -> None:
    n = nozzle_diagnostics(v)
    sc = v.scalars
    print("\nDIVERGENT NOZZLE  (throat -> exit)")
    print(_rule(78))
    print(f"  throat dia (UNCHANGED, = model's)  "
          f"{sc['nozzle throat diameter'][0]:9.3f} mm")
    print(f"  exit dia                           "
          f"{sc['nozzle exit flow diameter'][0]:9.3f} mm")
    print(f"  expansion area ratio               {n['area_ratio']:9.3f}")
    print(f"  divergent length @ "
          f"{sc['divergent half-angle'][0]:.0f} deg half-angle"
          f"   {sc['divergent length'][0]:9.3f} mm")
    print(f"\n  design point (ASSUMED): M {n['design_mach']:.2f}, "
          f"gamma {n['gamma']:.2f}, total-pressure recovery "
          f"{n['recovery']:.2f}")
    print(f"  available p0/pa                    {n['p0_over_pa']:9.4f}")
    print(f"  choking needs p0/pa >=             "
          f"{n['choke_pressure_ratio']:9.4f}   -> "
          f"{'CHOKED' if n['choked'] else 'NOT CHOKED'}")
    print(f"  nozzle first chokes at flight      M {n['choking_mach']:7.3f}")
    if n["choked"]:
        print(f"  optimum area ratio here            "
              f"{n['optimum_area_ratio']:9.4f}")
        print(f"  exit Mach at the built ratio       {n['exit_mach']:9.3f}")
        pe = n["pe_over_pa"]
        state = ("over-expanded" if pe < 0.98 else
                 "under-expanded" if pe > 1.02 else "matched")
        warn = "  <-- below ~0.4, expect separation" if pe < 0.4 else ""
        print(f"  exit pressure pe/pa                {pe:9.3f}   "
              f"{state}{warn}")
    print(f"\n  NOTE: the nozzle is unchoked below M {n['choking_mach']:.2f} "
          f"and the vehicle cuts off at M {n['design_mach']:.2f}, so it is")
    print("  choked only in the last sliver of the mission.  Below that a "
          "divergent section acts")
    print("  as a subsonic diffuser and REDUCES thrust.  The one thing "
          "expansion does buy is")
    print("  eating into the ANNULAR BASE, and base drag on that annulus is "
          "a flown term:")
    print(f"\n  {'area ratio':>10}  {'exit dia':>9}  {'div length':>10}  "
          f"{'base area':>11}  {'vs AR 1.0':>9}")
    for row in expansion_trade(v):
        mark = " <-- as drawn" if abs(
            row["area_ratio"] - n["area_ratio"]) < 1e-9 else ""
        if not row["fits"]:
            mark = " <-- WIDER THAN THE BASE"
        print(f"  {row['area_ratio']:>10.2f}  "
              f"{row['exit_diameter_m'] * MM:>7.2f} mm  "
              f"{row['divergent_length_m'] * MM:>8.2f} mm  "
              f"{row['base_area_m2'] * 1e4:>8.2f} cm2  "
              f"{100 * row['base_area_change_frac']:>+8.1f}%{mark}")
    print("  (quasi-1D isentropic; no boundary layer, no heat loss, and no "
          "unsteadiness --")
    print("   which for a PULSEJET is a real omission, its cycle being "
          "unsteady by construction)")


def print_assumptions(v) -> None:
    print("\nASSUMED GEOMETRY  (invented here; nothing below is in the model)")
    seen = []
    for ch in v.chains.values():
        for s in ch.segments:
            if s.source == ASSUMED:
                seen.append((f"{ch.name}/{s.name}",
                             s.x0 * MM, s.x1 * MM, s.detail))
    for st in v.stations:
        if st.source == ASSUMED:
            seen.append((f"station/{st.name}", st.x * MM, st.x * MM,
                         st.detail))
    w = max(len(r[0]) for r in seen)
    print(f"{'item':<{w}}{'x0 mm':>11}{'x1 mm':>11}  rule")
    print(_rule(w + 90))
    for name, x0, x1, detail in seen:
        print(f"{name:<{w}}{x0:>11.2f}{x1:>11.2f}  {detail}")
    print("\nNOT MODELLED AT ALL:")
    for k, txt in v.assumptions["not_modelled"].items():
        if k.startswith("_"):
            continue
        print(f"  - {k}: {txt}")


# ==========================================================================

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dims", default=str(DEFAULT_DIMS),
                   help="structural_dimensions.json to build from")
    p.add_argument("--assumptions", default=str(DEFAULT_ASSUMPTIONS))
    p.add_argument("--out-dir", default=str(OUT_DIR))
    p.add_argument("--show", action="store_true",
                   help="open the interactive viewer")
    p.add_argument("--export", action="store_true",
                   help="write stations/contour/area CSVs + geometry.json")
    p.add_argument("--save-vector", action="store_true",
                   help="write SVG + PDF without opening a window")
    p.add_argument("--contour-step-mm", type=float, default=1.0,
                   help="uniform sample step for the contour CSV export")
    p.add_argument("--quiet", action="store_true", help="skip the tables")
    args = p.parse_args(argv)

    dims, assumptions = load_inputs(args.dims, args.assumptions)
    v = build_vehicle(dims, assumptions)
    out_dir = Path(args.out_dir)

    if not args.quiet:
        print(f"\nDouglas Dart V4 -- 2D integrated propulsion section")
        print(f"  input : {Path(args.dims).relative_to(_REPO)}")
        print(f"  git   : {dims['provenance']['git_sha']}")
        print_stations(v)
        print_scalars(v)
        print_diffuser(v)
        print_nozzle(v)
        print_reconciliation(v)
        print_assumptions(v)

    if args.export:
        out_dir.mkdir(parents=True, exist_ok=True)
        step = args.contour_step_mm / MM
        export_stations_csv(v, out_dir / "stations.csv")
        export_contour_csv(v, out_dir / "contour.csv", step)
        export_area_csv(v, out_dir / "flow_area.csv", step)
        export_geometry_json(v, out_dir / "geometry.json")
        print(f"\nwrote exports to {out_dir}")
        for f in sorted(out_dir.glob("*")):
            print(f"  {f.name:<20}{f.stat().st_size / 1024:>9.1f} kB")

    if args.show or args.save_vector:
        import matplotlib
        if args.save_vector and not args.show:
            matplotlib.use("Agg")
        from scripts.propulsion_2d.viewer import SectionViewer
        import matplotlib.pyplot as plt

        viewer = SectionViewer(v, out_dir)
        if args.save_vector:
            viewer.save_vector()
        if args.show:
            print(viewer.help_text())
            plt.show()
        else:
            plt.close(viewer.fig)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
