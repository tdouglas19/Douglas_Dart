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


def _corner_seeds():
    """Hand-derived warm-start grid around the known feasible corner (large
    diameter, max operable throat, narrow landable span band, near-level
    climb) -- see optimize()'s seed_candidates note for why random draws
    alone cannot find a ~4e-6-volume corner."""
    import itertools

    from simple_model.constants import FUELS
    from simple_model.optimize import Candidate

    seeds = []
    for D, frac, ch, tube, span, climb, fuel in itertools.product(
        (0.26, 0.28, 0.30), (0.50, 0.54), (0.40, 0.55), (0.80, 1.00),
        (0.70, 0.75, 0.80), (1.0, 2.0), FUELS.keys(),
    ):
        seeds.append(Candidate(
            diameter_m=D, throat_diameter_m=D * frac, chamber_length_m=ch,
            throat_length_m=tube, wingspan_m=span, climb_angle_deg=climb,
            fuel_key=fuel,
        ))
    return seeds


def main() -> None:
    from simple_model.optimize import optimize

    kwargs = {"seed_candidates": _corner_seeds()}
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
    print("RESULT_JSON:" + json.dumps({
        "feasible": best.feasible,
        "max_thrust_to_weight": best.max_thrust_to_weight,
        "candidate": cand,
    }, default=str))


if __name__ == "__main__":
    main()
