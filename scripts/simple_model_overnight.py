"""Overnight CD0-sensitivity optimization campaign for the calibrated
simple_model (2026-08-12).

Why a CD0 family: after calibrating pulsejet thrust against the
first-principles pulsejet-fp model (a consistent 4.0-4.7x reduction from
the old duty-cycle guess), overall mission feasibility now hinges on the
flat CD0_FRONTAL placeholder -- at 0.30 the pulsejet ceiling (M~0.40,
level flight) sits below ramjet lightoff (M~0.45-0.55) for EVERY geometry
(the thrust/drag ratio is scale-invariant at fixed area fractions), while
0.10-0.20 (realistic clean slender body) closes the transition gap. Each
campaign runs the full optimizer in a subprocess with the CD0 env override
so the import-time constant binds correctly.
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
CD0_VALUES = [0.30, 0.25, 0.20, 0.15, 0.10]

CHILD_SCRIPT = Path(__file__).resolve().parent / "simple_model_campaign_child.py"


def main() -> None:
    OUT.mkdir(exist_ok=True)
    summary = []
    for cd0 in CD0_VALUES:
        tag = f"cd0_{cd0:.2f}".replace(".", "p")
        log_path = OUT / f"campaign_{tag}.log"
        env = dict(os.environ)
        env["SIMPLE_MODEL_CD0_FRONTAL"] = str(cd0)
        env["SIMPLE_MODEL_N_RANDOM"] = "20000"
        t0 = time.time()
        print(f"=== campaign CD0={cd0} -> {log_path.name} ===", flush=True)
        with open(log_path, "w") as fh:
            proc = subprocess.run(
                [sys.executable, str(CHILD_SCRIPT)], cwd=ROOT, env=env,
                stdout=fh, stderr=subprocess.STDOUT, text=True,
            )
        wall = time.time() - t0
        result = None
        for line in reversed(open(log_path).read().splitlines()):
            if line.startswith("RESULT_JSON:"):
                result = json.loads(line[len("RESULT_JSON:"):])
                break
        status = "crashed/no-feasible" if result is None else (
            "feasible" if result.get("feasible") else "infeasible")
        summary.append({"cd0": cd0, "status": status, "wall_s": round(wall),
                        "result": result, "exit": proc.returncode})
        print(f"    -> {status} in {wall:.0f}s", flush=True)
        with open(OUT / "summary.json", "w") as fh:
            json.dump(summary, fh, indent=2)
    print("=== campaign family complete ===")
    for row in summary:
        r = row["result"] or {}
        tw = r.get("max_thrust_to_weight")
        print(f"CD0={row['cd0']}: {row['status']}"
              + (f", best peak T/W={tw:.3f}, candidate={r.get('candidate')}" if tw else ""))


if __name__ == "__main__":
    main()
