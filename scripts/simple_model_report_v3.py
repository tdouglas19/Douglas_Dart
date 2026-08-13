"""V3 report: the climb-dive winner's plots + parameter table, plus a
head-to-head against the frozen V2 baseline (docs/v2_frozen/design.json).

Usage: python scripts/simple_model_report_v3.py
Reads out_simple_model/v3_summary.json (from simple_model_v3_campaign.py).
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out_simple_model"
FROZEN = ROOT / "docs" / "v2_frozen" / "design.json"
sys.path.insert(0, str(ROOT))


def main() -> None:
    row = json.loads((OUT / "v3_summary.json").read_text())[0]
    if not (row.get("vehicle") or {}).get("feasible"):
        raise SystemExit("V3 campaign has no feasible vehicle")
    cd0 = row["cd0"]
    os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(cd0)

    import matplotlib
    matplotlib.use("Agg")

    from simple_model import run_demo
    from simple_model.constants import (AIRFOILS, FUELS,
                                        MIN_POWERED_ACCELERATION_G,
                                        NOSE_LENGTH_DIAMETERS,
                                        TAIL_LENGTH_DIAMETERS)
    from simple_model.drag import WingConcept, default_wing_concept
    from simple_model.flight_sim import VehicleGeometry, run_flight
    from simple_model.mass_model import vehicle_dry_mass
    from simple_model.optimize import (Candidate, FUEL_RESERVE_MARGIN,
                                       FUEL_VOLUME_FRACTION_OF_ANNULUS,
                                       MAX_WET_MASS_KG, MOTOR_CUTOFF_MACH,
                                       _annular_volume_m3)

    def fly(cand_dict, wing_dict, label):
        cand = Candidate(**cand_dict)
        geometry = VehicleGeometry(
            cand.diameter_m, cand.throat_diameter_m, cand.chamber_length_m,
            cand.throat_length_m, cand.wingspan_m, FUELS[cand.fuel_key])
        concept = (WingConcept(wing_dict["span_m"], wing_dict["aspect_ratio"],
                               wing_dict["taper_ratio"], wing_dict["sweep_deg"],
                               AIRFOILS[wing_dict["airfoil_key"]])
                   if wing_dict else default_wing_concept(cand.wingspan_m))
        tank = (FUEL_VOLUME_FRACTION_OF_ANNULUS
                * _annular_volume_m3(cand.diameter_m, cand.throat_diameter_m,
                                     cand.throat_length_m)
                * FUELS[cand.fuel_key].density_kg_per_m3)
        result = run_flight(
            geometry, MAX_WET_MASS_KG, climb_angle_deg=cand.climb_angle_deg,
            motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02, max_time_s=900.0,
            wing_concept=concept,
            max_fuel_burn_kg=tank / (1.0 + FUEL_RESERVE_MARGIN),
            return_to_launch=True, climb_dive=cand.to_climb_dive())
        return cand, concept, result, label

    v3 = fly(row["vehicle"]["candidate"],
             (row.get("wing") or {}).get("wing"), "V3")
    frozen = json.loads(FROZEN.read_text())
    v2 = fly(frozen["vehicle_candidate"], frozen["wing_concept"], "V2")

    cand, concept, result, _ = v3
    run_demo.GEOMETRY = VehicleGeometry(
        cand.diameter_m, cand.throat_diameter_m, cand.chamber_length_m,
        cand.throat_length_m, cand.wingspan_m, FUELS[cand.fuel_key])
    run_demo.CLIMB_ANGLE_DEG = cand.climb_angle_deg
    run_demo.OUT_DIR = OUT
    fin = result.states[-1]
    fuel_loaded = (1.0 + FUEL_RESERVE_MARGIN) * fin.fuel_burned_kg
    paths = [run_demo.plot_propulsion(),
             run_demo.plot_flight_profile(result, fuel_loaded)]
    for p in paths:                      # keep V3 plots beside the V2 ones
        p.rename(p.with_name(p.stem + "_v3" + p.suffix))
    paths = [p.with_name(p.stem + "_v3" + p.suffix) for p in paths]

    mass = vehicle_dry_mass(cand.diameter_m, cand.chamber_length_m,
                            cand.throat_diameter_m, cand.throat_length_m,
                            concept.reference_area_m2, fuel_loaded)
    peak_tw = max(s.thrust_to_weight for s in result.states)
    track_km = sum(abs(b.distance_m - a.distance_m)
                   for a, b in zip(result.states, result.states[1:])) / 1e3
    top_ft = (result.climb_dive_top_altitude_m or 0.0) / 0.3048

    def stat(r):
        return (max(s.thrust_to_weight for s in r.states),
                r.min_traverse_accel_g, r.min_powered_thrust_margin)

    v2_tw, v2_acc, v2_mar = stat(v2[2])
    v2_fuel = (1.0 + FUEL_RESERVE_MARGIN) * v2[2].states[-1].fuel_burned_kg
    v2_mass = vehicle_dry_mass(v2[0].diameter_m, v2[0].chamber_length_m,
                               v2[0].throat_diameter_m, v2[0].throat_length_m,
                               v2[1].reference_area_m2, v2_fuel)

    def pct(new, old):
        return f"{(new - old) / old * 100:+.0f}%"

    lines = [
        "# V3 climb-dive design -- parameter table and V2 comparison", "",
        f"Campaign: CD0 = {cd0}; climb-dive profile; acceleration gate "
        f"{MIN_POWERED_ACCELERATION_G} g on the traverse (notch + drag strip).", "",
        "## Head-to-head vs the frozen V2 baseline", "",
        "| metric | V2 (frozen) | V3 (climb-dive) | change |", "|---|---|---|---|",
        f"| peak T/W (objective) | {v2_tw:.2f} | {peak_tw:.2f} | {pct(peak_tw, v2_tw)} |",
        f"| min traverse acceleration | {v2_acc:+.3f} g | {result.min_traverse_accel_g:+.3f} g | {pct(result.min_traverse_accel_g, v2_acc)} |",
        f"| min engine-only thrust margin | {v2_mar:.2f} | {result.min_powered_thrust_margin:.2f} | {pct(result.min_powered_thrust_margin, v2_mar)} |",
        f"| body diameter | {v2[0].diameter_m*1e3:.0f} mm | {cand.diameter_m*1e3:.0f} mm | {pct(cand.diameter_m, v2[0].diameter_m)} |",
        f"| dry mass | {v2_mass.dry_mass_kg:.2f} kg | {mass.dry_mass_kg:.2f} kg | {pct(mass.dry_mass_kg, v2_mass.dry_mass_kg)} |",
        f"| payload/ballast margin | {MAX_WET_MASS_KG - v2_mass.dry_mass_kg - v2_fuel:.2f} kg | "
        f"{MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded:.2f} kg | "
        f"{pct(MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded, MAX_WET_MASS_KG - v2_mass.dry_mass_kg - v2_fuel)} |",
        "", "## V3 trajectory", "", "| phase | value |", "|---|---|",
        f"| initial climb angle | {cand.initial_climb_angle_deg:.1f} deg |",
        f"| top of climb (DERIVED from the dive) | {top_ft:.0f} ft |",
        f"| dive angle | {cand.dive_angle_deg:.1f} deg |",
        f"| pull-out floor | {cand.floor_altitude_m/0.3048:.0f} ft (hard min 400 ft) |",
        f"| drag-strip angle | {cand.climb_angle_deg:.1f} deg |",
        f"| rule (gamma >= 0, M 0.80 -> cutoff) | {'SATISFIED' if not result.rule_violated else 'VIOLATED'} |",
        "", "## Vehicle", "", "| parameter | value |", "|---|---|",
        f"| body diameter | {cand.diameter_m*1e3:.0f} mm |",
        f"| throat diameter | {cand.throat_diameter_m*1e3:.0f} mm (area frac {(cand.throat_diameter_m/cand.diameter_m)**2:.3f}) |",
        f"| chamber / tube length | {cand.chamber_length_m*1e3:.0f} / {cand.throat_length_m*1e3:.0f} mm |",
        f"| nose cone / boattail length | {NOSE_LENGTH_DIAMETERS*cand.diameter_m*1e3:.0f} / {TAIL_LENGTH_DIAMETERS*cand.diameter_m*1e3:.0f} mm |",
        f"| overall body length | {(cand.chamber_length_m+cand.throat_length_m+3.0*cand.diameter_m)*1e3:.0f} mm |",
        f"| fuel | {cand.fuel_key} |",
        "", "| wing concept | value |", "|---|---|",
        f"| span | {concept.span_m*1e3:.0f} mm |",
        f"| aspect ratio / taper / sweep | {concept.aspect_ratio:.2f} / {concept.taper_ratio:.2f} / {concept.sweep_deg:.0f} deg |",
        f"| airfoil | {concept.airfoil.display_name} |",
        "", "| mass budget | kg |", "|---|---|",
        f"| engine duct (steel, t={mass.duct_wall_thickness_m*1e3:.1f} mm) | {mass.engine_duct_kg:.2f} |",
        f"| airframe skin (CFRP) | {mass.airframe_skin_kg:.2f} |",
        f"| wing | {mass.wing_kg:.2f} |",
        f"| avionics / tank hw / landing hw | {mass.avionics_kg:.1f} / {mass.tank_hardware_kg:.2f} / {mass.landing_hardware_kg:.1f} |",
        f"| dry total | {mass.dry_mass_kg:.2f} |",
        f"| fuel loaded (incl. reserve) | {fuel_loaded:.2f} |",
        f"| payload/ballast margin vs 50 lb | {MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded:.2f} |",
        "", "| mission | value |", "|---|---|",
        f"| peak T/W | {peak_tw:.2f} |",
        f"| min traverse acceleration | {result.min_traverse_accel_g:+.3f} g at M {result.min_traverse_accel_mach:.2f} (required >= {MIN_POWERED_ACCELERATION_G}) |",
        f"| min acceleration incl. climb | {result.min_powered_accel_g:+.3f} g at M {result.min_accel_mach:.2f} (must stay > 0) |",
        f"| min engine-only thrust margin | {result.min_powered_thrust_margin:.2f} at M {result.min_margin_mach:.2f} (required >= 1.15) |",
        f"| cutoff / safe landing | {result.motor_cutoff_reached} / {result.safe_landing} |",
        f"| touchdown / stall speed | {fin.velocity_m_per_s:.0f} / {fin.stall_speed_m_per_s:.0f} m/s |",
        f"| flight time / ground track | {fin.time_s:.0f} s / {track_km:.1f} km (lands {abs(fin.distance_m):.0f} m from launch) |",
        "", "## Plots", "",
        "### Propulsion (thrust, Isp, SFC vs Mach; sea level)", "",
        "![propulsion](propulsion_v3.png)", "",
        "### Flight profile (climb / dive / drag strip / return; shading = phase)", "",
        "![flight profile](flight_profile_v3.png)",
    ]
    path = OUT / "v3_optimal_design.md"
    path.write_text("\n".join(lines))
    print("\n".join(lines))
    for p in paths + [path]:
        print("wrote", p)


if __name__ == "__main__":
    main()
