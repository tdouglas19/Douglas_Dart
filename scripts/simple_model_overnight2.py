"""Overnight pipeline v2 (2026-08-12): vehicle campaign -> wing campaign,
under the full constraint/objective set (thrust margin, parametric mass
budget, composite T/W + diameter + length + span objective).

Phase 1: vehicle optimization at each CD0 level (finer grid than v1).
Phase 2: for each feasible CD0, optimize the wing concept for that
         CD0's winning vehicle (fixed vehicle, 5 wing variables).
Everything lands in out_simple_model/overnight2_summary.json plus
per-run logs. Report/plots: scripts/simple_model_report.py (v2-aware).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out_simple_model"
VEHICLE_CHILD = ROOT / "scripts" / "simple_model_campaign_child.py"
WING_CHILD = ROOT / "scripts" / "simple_model_wing_child.py"
# Low-CD0-focused: with honest wing wave drag the >=0.25 levels are
CD0_VALUES = [0.25, 0.20, 0.175, 0.15, 0.125, 0.10]  # hopeless above ~0.2
N_RANDOM_VEHICLE = "60000"
N_RANDOM_WING = "8000"


def run_child(script: Path, log_path: Path, env: dict) -> dict | None:
    with open(log_path, "w") as fh:
        subprocess.run([sys.executable, str(script)], cwd=ROOT, env=env,
                       stdout=fh, stderr=subprocess.STDOUT, text=True)
    for line in reversed(log_path.read_text().splitlines()):
        if line.startswith("RESULT_JSON:"):
            return json.loads(line[len("RESULT_JSON:"):])
    return None


def main() -> None:
    OUT.mkdir(exist_ok=True)
    summary = []
    t_start = time.time()
    for cd0 in CD0_VALUES:
        tag = f"cd0_{cd0:.3f}".replace(".", "p")
        env = dict(os.environ)
        env["SIMPLE_MODEL_CD0_FRONTAL"] = str(cd0)
        env["SIMPLE_MODEL_N_RANDOM"] = N_RANDOM_VEHICLE
        # stage-1 vehicles fly a real thin wing (flat plate, the most
        # benign supersonic choice) so wing wave drag is priced in from
        # the start -- see optimize.evaluate(wing_airfoil_key=...)
        env["SIMPLE_MODEL_WING_AIRFOIL"] = "flat_plate"

        t0 = time.time()
        print(f"=== [vehicle] CD0={cd0} ===", flush=True)
        vres = run_child(VEHICLE_CHILD, OUT / f"o2_vehicle_{tag}.log", env)
        v_wall = time.time() - t0
        row = {"cd0": cd0, "vehicle": vres, "vehicle_wall_s": round(v_wall)}
        feasible = bool(vres and vres.get("feasible"))
        print(f"    vehicle: {'feasible' if feasible else 'infeasible'} "
              f"({v_wall:.0f}s)", flush=True)

        if feasible:
            wenv = dict(env)
            wenv["SIMPLE_MODEL_VEHICLE_JSON"] = json.dumps(vres["candidate"])
            wenv["SIMPLE_MODEL_WING_N_RANDOM"] = N_RANDOM_WING
            t0 = time.time()
            print(f"=== [wing] CD0={cd0} ===", flush=True)
            wres = run_child(WING_CHILD, OUT / f"o2_wing_{tag}.log", wenv)
            row["wing"] = wres
            row["wing_wall_s"] = round(time.time() - t0)
            ok = bool(wres and wres.get("feasible"))
            print(f"    wing: {'feasible' if ok else 'failed'} "
                  f"({row['wing_wall_s']}s)", flush=True)

        summary.append(row)
        (OUT / "overnight2_summary.json").write_text(json.dumps(summary, indent=2))

    print(f"=== pipeline complete in {(time.time()-t_start)/60:.0f} min ===")
    for row in summary:
        v = row.get("vehicle") or {}
        w = row.get("wing") or {}
        if v.get("feasible"):
            print(f"CD0={row['cd0']}: vehicle score={v.get('score'):.3f} "
                  f"peakT/W={v.get('max_thrust_to_weight'):.2f} "
                  f"massMargin={v.get('mass_margin_kg'):.1f}kg"
                  + (f" | wing score={w.get('score'):.3f} "
                     f"peakT/W={w.get('max_thrust_to_weight'):.2f} "
                     f"wing={w.get('wing')}" if w.get("feasible") else " | wing: none"))
        else:
            print(f"CD0={row['cd0']}: infeasible")


if __name__ == "__main__":
    main()
