"""v2 report: full plot set + dimensional/mass/margin table for the
overnight2 pipeline's winning design (vehicle + optimized wing concept).

Usage: python scripts/simple_model_report2.py [cd0]
Winner default: lowest composite SCORE among campaigns whose wing stage is
feasible (falls back to vehicle-only rows).
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out_simple_model"
sys.path.insert(0, str(ROOT))


def pick():
    rows = json.loads((OUT / "overnight2_summary.json").read_text())
    ok = [r for r in rows if (r.get("vehicle") or {}).get("feasible")]
    if not ok:
        raise SystemExit("no feasible campaign in overnight2_summary.json")
    if len(sys.argv) > 1:
        want = float(sys.argv[1])
        return next(r for r in ok if abs(r["cd0"] - want) < 1e-9), rows

    def keyfn(r):
        w = r.get("wing") or {}
        if w.get("feasible"):
            return w["score"]
        return r["vehicle"]["score"]

    return min(ok, key=keyfn), rows


def main() -> None:
    row, all_rows = pick()
    cd0 = row["cd0"]
    vres = row["vehicle"]
    wres = row.get("wing") or {}
    os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(cd0)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from simple_model import run_demo
    from simple_model.constants import AIRFOILS, FUELS, KG_PER_LB
    from simple_model.drag import WingConcept
    from simple_model.flight_sim import VehicleGeometry, run_flight
    from simple_model.mass_model import vehicle_dry_mass
    from simple_model.optimize import (FUEL_RESERVE_MARGIN,
                                       FUEL_VOLUME_FRACTION_OF_ANNULUS,
                                       MAX_WET_MASS_KG, MOTOR_CUTOFF_MACH,
                                       _annular_volume_m3)

    cand = vres["candidate"]
    geometry = VehicleGeometry(
        diameter_m=cand["diameter_m"], throat_diameter_m=cand["throat_diameter_m"],
        chamber_length_m=cand["chamber_length_m"], throat_length_m=cand["throat_length_m"],
        wingspan_m=cand["wingspan_m"], fuel=FUELS[cand["fuel_key"]],
    )
    if wres.get("feasible"):
        w = wres["wing"]
        concept = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                              w["sweep_deg"], AIRFOILS[w["airfoil_key"]])
    else:
        from simple_model.drag import default_wing_concept
        concept = default_wing_concept(cand["wingspan_m"])

    run_demo.GEOMETRY = geometry
    run_demo.CLIMB_ANGLE_DEG = cand["climb_angle_deg"]
    run_demo.OUT_DIR = OUT
    paths = [run_demo.plot_thrust_vs_mach()]

    tank_cap = (FUEL_VOLUME_FRACTION_OF_ANNULUS
                * _annular_volume_m3(cand["diameter_m"], cand["throat_diameter_m"],
                                     cand["throat_length_m"])
                * FUELS[cand["fuel_key"]].density_kg_per_m3)
    result = run_flight(geometry, MAX_WET_MASS_KG,
                        climb_angle_deg=cand["climb_angle_deg"],
                        motor_cutoff_mach=MOTOR_CUTOFF_MACH,
                        dt_s=0.02, max_time_s=900.0, wing_concept=concept,
                        max_fuel_burn_kg=tank_cap / (1.0 + FUEL_RESERVE_MARGIN))
    final = result.states[-1]
    fuel_loaded = (1.0 + FUEL_RESERVE_MARGIN) * final.fuel_burned_kg
    paths += [run_demo.plot_flight_profile(result),
              run_demo.plot_altitude_vs_distance(result),
              run_demo.plot_fuel_mass(result, fuel_loaded)]

    # CD0 sensitivity (score + peak T/W, wing-stage where available)
    fig, ax1 = plt.subplots(figsize=(8.5, 5))
    ax2 = ax1.twinx()
    xs, scores, tws, dead = [], [], [], []
    for r in all_rows:
        v = r.get("vehicle") or {}
        w = r.get("wing") or {}
        best = w if w.get("feasible") else (v if v.get("feasible") else None)
        if best:
            xs.append(r["cd0"]); scores.append(best["score"])
            tws.append(best["max_thrust_to_weight"])
        else:
            dead.append(r["cd0"])
    ax1.plot(xs, scores, "-o", color="#2a78d6", label="composite score (lower=better)")
    ax2.plot(xs, tws, "--s", color="#008300", label="peak T/W")
    for d in dead:
        ax1.axvline(d, color="#e34948", alpha=0.3, lw=8)
    ax1.set_xlabel("body CD0")
    ax1.set_ylabel("composite score", color="#2a78d6")
    ax2.set_ylabel("peak T/W", color="#008300")
    ax1.set_title("v2 campaign: score & peak T/W vs CD0\n(red bands: infeasible under full constraint stack)")
    ax1.grid(alpha=0.3)
    fig.tight_layout()
    p = OUT / "cd0_sensitivity_v2.png"
    fig.savefig(p, dpi=150); plt.close(fig); paths.append(p)

    mass = vehicle_dry_mass(cand["diameter_m"], cand["chamber_length_m"],
                            cand["throat_diameter_m"], cand["throat_length_m"],
                            concept.reference_area_m2, fuel_loaded)
    peak_tw = max(st.thrust_to_weight for st in result.states)
    lines = [
        "# v2 optimized vehicle + wing -- full parameter table", "",
        f"Campaign: CD0 = {cd0}; composite objective (T/W + D + length + span)", "",
        "| vehicle | value |", "|---|---|",
        f"| body diameter | {cand['diameter_m']*1e3:.0f} mm |",
        f"| throat diameter | {cand['throat_diameter_m']*1e3:.0f} mm (area frac {(cand['throat_diameter_m']/cand['diameter_m'])**2:.3f}) |",
        f"| chamber / tube length | {cand['chamber_length_m']*1e3:.0f} / {cand['throat_length_m']*1e3:.0f} mm |",
        f"| overall body length (incl. nose/tail) | {(cand['chamber_length_m']+cand['throat_length_m']+3.0*cand['diameter_m'])*1e3:.0f} mm |",
        f"| climb angle / fuel | {cand['climb_angle_deg']:.1f} deg / {cand['fuel_key']} |", "",
        "| wing concept | value |", "|---|---|",
        f"| span | {concept.span_m*1e3:.0f} mm |",
        f"| aspect ratio / taper / sweep | {concept.aspect_ratio:.2f} / {concept.taper_ratio:.2f} / {concept.sweep_deg:.0f} deg |",
        f"| airfoil | {concept.airfoil.display_name} |",
        f"| area / Oswald e / CLmax_eff | {concept.reference_area_m2:.3f} m^2 / {concept.oswald_e:.3f} / {concept.cl_max_effective:.2f} |", "",
        "| mass budget | kg |", "|---|---|",
        f"| engine duct (steel, t={mass.duct_wall_thickness_m*1e3:.1f} mm) | {mass.engine_duct_kg:.2f} |",
        f"| airframe skin (CFRP) | {mass.airframe_skin_kg:.2f} |",
        f"| wing | {mass.wing_kg:.2f} |",
        f"| avionics / tank hw / landing hw | {mass.avionics_kg:.1f} / {mass.tank_hardware_kg:.2f} / {mass.landing_hardware_kg:.1f} |",
        f"| dry total | {mass.dry_mass_kg:.2f} |",
        f"| fuel loaded (incl. reserve) | {fuel_loaded:.2f} |",
        f"| payload/ballast margin vs 50 lb | {MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded:.2f} |", "",
        "| mission | value |", "|---|---|",
        f"| peak T/W | {peak_tw:.2f} |",
        f"| composite score | {(wres if wres.get('feasible') else vres)['score']:.3f} |",
        f"| min powered thrust margin | {result.min_powered_thrust_margin:.2f} at M {result.min_margin_mach:.2f} (required >= 1.15) |",
        f"| cutoff / safe landing | {result.motor_cutoff_reached} / {result.safe_landing} |",
        f"| touchdown / stall speed | {final.velocity_m_per_s:.0f} / {final.stall_speed_m_per_s:.0f} m/s |",
        f"| flight time / distance | {final.time_s:.0f} s / {final.distance_m/1e3:.1f} km |",
    ]
    tp = OUT / "v2_optimal_design.md"
    tp.write_text("\n".join(lines))
    paths.append(tp)
    print("\n".join(lines))
    for p in paths:
        print("wrote", p)


if __name__ == "__main__":
    main()
