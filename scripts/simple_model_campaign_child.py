"""One optimization campaign (real file so Windows multiprocessing spawn can
re-import __main__). CD0 arrives via SIMPLE_MODEL_CD0_FRONTAL (read at
import time by simple_model.constants); optional size overrides via
SIMPLE_MODEL_N_RANDOM / SIMPLE_MODEL_N_REFINE for smoke tests."""
from __future__ import annotations

import dataclasses
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def main() -> None:
    from simple_model.optimize import optimize

    kwargs = {}
    if os.environ.get("SIMPLE_MODEL_N_RANDOM"):
        kwargs["n_random"] = int(os.environ["SIMPLE_MODEL_N_RANDOM"])
    if os.environ.get("SIMPLE_MODEL_N_REFINE"):
        kwargs["n_refine"] = int(os.environ["SIMPLE_MODEL_N_REFINE"])
    try:
        best = optimize(log=lambda *a, **k: print(*a, flush=True, **k), **kwargs)
    except RuntimeError as exc:
        print("RESULT_JSON:" + json.dumps({"feasible": False, "error": str(exc)[:400]}))
        return
    c = best.candidate
    cand = dataclasses.asdict(c) if dataclasses.is_dataclass(c) else c._asdict()
    cand["fuel"] = c.fuel.key
    print("RESULT_JSON:" + json.dumps({
        "feasible": best.feasible,
        "max_thrust_to_weight": best.max_thrust_to_weight,
        "candidate": cand,
    }, default=str))


if __name__ == "__main__":
    main()
