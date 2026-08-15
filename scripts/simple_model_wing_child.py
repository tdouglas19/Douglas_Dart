"""One wing-concept optimization for a fixed vehicle (real file for Windows
spawn). Vehicle candidate arrives as JSON via SIMPLE_MODEL_VEHICLE_JSON;
CD0 via SIMPLE_MODEL_CD0_FRONTAL (import-time, as usual)."""
from __future__ import annotations

import dataclasses
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def main() -> None:
    from simple_model.optimize import Candidate
    from simple_model.wing_optimize import optimize_wing

    cand = json.loads(os.environ["SIMPLE_MODEL_VEHICLE_JSON"])
    vehicle = Candidate(**cand)
    kwargs = {}
    if os.environ.get("SIMPLE_MODEL_WING_N_RANDOM"):
        kwargs["n_random"] = int(os.environ["SIMPLE_MODEL_WING_N_RANDOM"])
    try:
        best = optimize_wing(vehicle, log=lambda *a, **k: print(*a, flush=True, **k), **kwargs)
    except RuntimeError as exc:
        print("RESULT_JSON:" + json.dumps({"feasible": False, "error": str(exc)[:300]}))
        return
    print("RESULT_JSON:" + json.dumps({
        "feasible": True,
        "max_thrust_to_weight": best.max_thrust_to_weight,
        "score": best.score,
        "min_powered_thrust_margin": best.min_powered_thrust_margin,
        "mass_margin_kg": best.mass_margin_kg,
        "wing": dataclasses.asdict(best.wing),
    }, default=str))


if __name__ == "__main__":
    main()
