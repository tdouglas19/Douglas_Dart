"""V3 campaign: vehicle + wing optimization at one CD0 with the climb-dive
profile enabled (SIMPLE_MODEL_V3=1), written alongside the V2 result so the
two can be compared directly.

Usage:  python scripts/simple_model_v3_campaign.py [cd0]   (default 0.10)

Writes out_simple_model/v3_summary.json. The V2 baseline it is compared
against lives in docs/v2_frozen/ and is NOT touched.
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
CD0 = sys.argv[1] if len(sys.argv) > 1 else "0.1"
N_RANDOM_VEHICLE = os.environ.get("SIMPLE_MODEL_N_RANDOM", "40000")
N_RANDOM_WING = os.environ.get("SIMPLE_MODEL_WING_N_RANDOM", "6000")


def run_child(script: str, log_name: str, env: dict):
    log_path = OUT / log_name
    with open(log_path, "w") as fh:
        subprocess.run([sys.executable, str(ROOT / "scripts" / script)],
                       cwd=ROOT, env=env, stdout=fh,
                       stderr=subprocess.STDOUT, text=True)
    for line in reversed(log_path.read_text().splitlines()):
        if line.startswith("RESULT_JSON:"):
            return json.loads(line[len("RESULT_JSON:"):])
    return None


def main() -> None:
    OUT.mkdir(exist_ok=True)
    env = dict(os.environ)
    env["SIMPLE_MODEL_V3"] = "1"
    env["SIMPLE_MODEL_CD0_FRONTAL"] = CD0
    env["SIMPLE_MODEL_N_RANDOM"] = N_RANDOM_VEHICLE
    env["SIMPLE_MODEL_WING_AIRFOIL"] = "flat_plate"

    t0 = time.time()
    print(f"=== [V3 vehicle] CD0={CD0} ===", flush=True)
    vres = run_child("simple_model_campaign_child.py", f"v3_vehicle_cd0_{CD0}.log", env)
    print(f"    vehicle: {'feasible' if vres and vres.get('feasible') else 'INFEASIBLE'}"
          f" ({time.time()-t0:.0f}s)", flush=True)
    row = {"cd0": float(CD0), "profile": "v3_climb_dive", "vehicle": vres}

    if vres and vres.get("feasible"):
        env["SIMPLE_MODEL_VEHICLE_JSON"] = json.dumps(vres["candidate"])
        env["SIMPLE_MODEL_WING_N_RANDOM"] = N_RANDOM_WING
        t1 = time.time()
        print("=== [V3 wing] ===", flush=True)
        wres = run_child("simple_model_wing_child.py", f"v3_wing_cd0_{CD0}.log", env)
        print(f"    wing: {'feasible' if wres and wres.get('feasible') else 'INFEASIBLE'}"
              f" ({time.time()-t1:.0f}s)", flush=True)
        row["wing"] = wres

    (OUT / "v3_summary.json").write_text(json.dumps([row], indent=1))
    print(json.dumps(row, indent=1))
    print(f"total {time.time()-t0:.0f}s -> {OUT / 'v3_summary.json'}")


if __name__ == "__main__":
    main()
