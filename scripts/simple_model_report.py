"""Report generator for the T/W-optimized vehicle: full plot set + a
dimensional-parameter table, from the overnight campaign's winning design.

Usage:  python scripts/simple_model_report.py [cd0]
        (cd0 defaults to the lowest-peak-T/W FEASIBLE campaign in
         out_simple_model/summary.json)

Reuses simple_model/run_demo.py's own plotting (thrust-vs-Mach, 7-panel
flight profile, altitude-vs-distance, fuel mass) by overriding its
module-level GEOMETRY/CLIMB_ANGLE_DEG with the winner, then adds the
CD0-sensitivity figure and writes the dimensional table. All outputs land
in out_simple_model/.

IMPORTANT: CD0 is baked into simple_model at import time, so this script
sets SIMPLE_MODEL_CD0_FRONTAL *before* importing simple_model.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out_simple_model"
sys.path.insert(0, str(ROOT))


def pick_winner():
    summary = json.loads((OUT / "summary.json").read_text())
    feasible = [row for row in summary
                if row["result"] and row["result"].get("feasible")]
    if not feasible:
        raise SystemExit("no feasible campaign in summary.json")
    if len(sys.argv) > 1:
        want = float(sys.argv[1])
        row = next(r for r in feasible if abs(r["cd0"] - want) < 1e-9)
    else:
        row = min(feasible, key=lambda r: r["result"]["max_thrust_to_weight"])
    return row, summary


def main() -> None:
    row, summary = pick_winner()
    cd0 = row["cd0"]
    cand = row["result"]["candidate"]
    os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(cd0)

    # imports AFTER the env is set (constants.py reads it at import time)
    import matplotlib
    matplotlib.use("Agg")

    from simple_model import run_demo
    from simple_model.constants import FUELS, KG_PER_LB
    from simple_model.flight_sim import VehicleGeometry, run_flight
    from simple_model.optimize import (FUEL_RESERVE_MARGIN, MAX_WET_MASS_KG,
                                       MOTOR_CUTOFF_MACH, evaluate, Candidate)

    geometry = VehicleGeometry(
        diameter_m=cand["diameter_m"],
        throat_diameter_m=cand["throat_diameter_m"],
        chamber_length_m=cand["chamber_length_m"],
        throat_length_m=cand["throat_length_m"],
        wingspan_m=cand["wingspan_m"],
        fuel=FUELS[cand["fuel_key"]],
    )
    run_demo.GEOMETRY = geometry
    run_demo.CLIMB_ANGLE_DEG = cand["climb_angle_deg"]
    run_demo.OUT_DIR = OUT

    print(f"T/W-optimal design (CD0={cd0}): {cand}")
    paths = [run_demo.plot_propulsion()]

    result = run_flight(geometry, MAX_WET_MASS_KG,
                        climb_angle_deg=cand["climb_angle_deg"],
                        motor_cutoff_mach=MOTOR_CUTOFF_MACH,
                        dt_s=0.02, max_time_s=900.0, return_to_launch=True)
    final = result.states[-1]
    fuel_loaded_kg = (1.0 + FUEL_RESERVE_MARGIN) * final.fuel_burned_kg
    paths.append(run_demo.plot_flight_profile(result, fuel_loaded_kg))

    # dimensional table
    peak_tw = max(st.thrust_to_weight for st in result.states)
    peak_mach = max(st.mach for st in result.states)
    cutoff = next((s for s in result.states if s.mode in ("decel", "glide", "flare")), final)
    lines = [
        "# T/W-optimized vehicle -- dimensional parameters",
        "",
        f"Campaign: CD0 = {cd0} (lowest-peak-T/W feasible level of the sensitivity family)",
        "",
        "| parameter | value |",
        "|---|---|",
        f"| body diameter | {geometry.diameter_m*1e3:.0f} mm |",
        f"| throat diameter | {geometry.throat_diameter_m*1e3:.0f} mm |",
        f"| throat/chamber area fraction | {(geometry.throat_diameter_m/geometry.diameter_m)**2:.3f} (operability limit 0.30) |",
        f"| chamber length | {geometry.chamber_length_m*1e3:.0f} mm |",
        f"| throat/resonance-tube length | {geometry.throat_length_m*1e3:.0f} mm |",
        f"| overall duct length (chamber+tube) | {(geometry.chamber_length_m+geometry.throat_length_m)*1e3:.0f} mm |",
        f"| wingspan | {geometry.wingspan_m*1e3:.0f} mm |",
        f"| climb angle | {cand['climb_angle_deg']:.1f} deg |",
        f"| fuel | {geometry.fuel.display_name} |",
        f"| wet mass | {MAX_WET_MASS_KG:.1f} kg (50 lb) |",
        f"| fuel loaded (incl. {FUEL_RESERVE_MARGIN:.0%} reserve) | {fuel_loaded_kg:.2f} kg |",
        f"| implied dry mass | {MAX_WET_MASS_KG - fuel_loaded_kg:.2f} kg |",
        "",
        "| mission result | value |",
        "|---|---|",
        f"| peak thrust-to-weight (objective) | {peak_tw:.2f} |",
        f"| max Mach | {peak_mach:.2f} |",
        f"| motor cutoff reached | {result.motor_cutoff_reached} |",
        f"| time to cutoff | {cutoff.time_s:.0f} s |",
        f"| safe landing | {result.safe_landing} |",
        f"| touchdown speed / stall speed | {final.velocity_m_per_s:.0f} / {final.stall_speed_m_per_s:.0f} m/s |",
        f"| total flight time | {final.time_s:.0f} s |",
        f"| ground distance | {final.distance_m/1e3:.1f} km |",
    ]
    table_path = OUT / "tw_optimal_design.md"
    table_path.write_text("\n".join(lines))
    paths.append(table_path)
    print("\n".join(lines))
    for p in paths:
        print("wrote", p)


if __name__ == "__main__":
    main()
