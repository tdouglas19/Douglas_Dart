"""V4 selection pass: rank the campaign's feasible designs by how much
pulsejet thrust they can LOSE and still light the ramjet in the dive.

Why this and not nominal payload margin: the whole V4 mission turns on
reaching a Mach gate inside the dive, and docs/v3_learnings_for_v4.md
section 4.1 is explicit that "can it reach X" is the one question a
closed-form screen gets wrong near a boundary -- the FP/closed-form traverse
ratio collapses from 0.78 to 0.05 right there. Independently, section 4.3
measured a 3.8-5.5 % thrust loss from a single grid refinement (162 -> 324
cells), and section 4.1 puts closed-form 6-16 % high on thrust outright.

So a V4 design is only interesting if it still lights after the pulsejet
gives back several percent. Nominal-thrust payload margin is the tiebreak,
not the objective.

Reads  out_simple_model/v4_campaign.json
Writes out_simple_model/v4_robust.json
"""
from __future__ import annotations

import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.simple_model_v4_baseline import (  # noqa: E402
    FP_PULSEJET_CEILING_M, V4_WING, fly, mass_block, v4_geometry)
from scripts.simple_model_v4_campaign import MAX_PEAK_LOAD_N  # noqa: E402
from simple_model.flight_sim import V3_FLOOR_ALTITUDE_M  # noqa: E402

# Steps chosen against the measured error budget, not round numbers:
# 0.962 is one grid refinement (-3.8 %), 0.945 is the worst one (-5.5 %),
# 0.92 covers a refinement plus a slice of the 6-16 % closed-form bias.
HAIRCUTS = [1.0, 0.962, 0.945, 0.92, 0.90]


def _survives(args: tuple) -> dict:
    key, scale = args
    gate, top, dive, climb, n = key
    import simple_model.pulsejet_simple as pj
    original = pj.AVERAGE_THRUST_DUTY_CYCLE_FACTOR
    pj.AVERAGE_THRUST_DUTY_CYCLE_FACTOR = original * scale
    try:
        r = fly(climb_deg=climb, dive_deg=dive, floor_m=V3_FLOOR_ALTITUDE_M,
                gate_mach=gate, pullout_n=n, top_altitude_m=top)
        mb = mass_block(r, v4_geometry(), V4_WING)
        ok = bool(r.ramjet_lit_in_dive and r.motor_cutoff_reached
                  and r.safe_landing and not r.floor_violated
                  and not r.rule_violated
                  and r.peak_load_n_total <= MAX_PEAK_LOAD_N
                  and mb["payload_margin_kg"] > 0.0)
        return {"key": list(key), "scale": scale, "ok": ok,
                "dive_exit_mach": r.dive_exit_mach,
                "lit_in_dive": r.ramjet_lit_in_dive,
                "cutoff": r.motor_cutoff_reached,
                "safe_landing": r.safe_landing,
                "peak_n_total": r.peak_load_n_total,
                "payload_margin_kg": mb["payload_margin_kg"],
                "traverse_g": r.min_traverse_accel_g,
                "peak_tw": max(s.thrust_to_weight for s in r.states)}
    finally:
        pj.AVERAGE_THRUST_DUTY_CYCLE_FACTOR = original


def main() -> None:
    data = json.loads(Path("out_simple_model/v4_campaign.json").read_text())
    feasible = [r for r in data["rows"] if r["feasible"]]
    keys = [(r["gate"], r["top_m"], r["dive_deg"], r["climb_deg"], r["pullout_n"])
            for r in feasible]
    print(f"{len(keys)} nominally-feasible designs x {len(HAIRCUTS)} thrust levels "
          f"= {len(keys)*len(HAIRCUTS)} flights")

    tasks = [(k, s) for k in keys for s in HAIRCUTS]
    workers = max(1, (os.cpu_count() or 2) - 1)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(_survives, tasks, chunksize=8))

    by_key: dict[tuple, dict] = {}
    for res in results:
        by_key.setdefault(tuple(res["key"]), {})[res["scale"]] = res

    ranked = []
    for key, per_scale in by_key.items():
        survived = [s for s in HAIRCUTS if per_scale[s]["ok"]]
        worst = min(survived) if survived else 1.01
        nominal = per_scale[1.0]
        ranked.append({
            "gate": key[0], "top_m": key[1], "dive_deg": key[2],
            "climb_deg": key[3], "pullout_n": key[4],
            "worst_scale_survived": worst,
            "thrust_loss_tolerated_pct": (1.0 - worst) * 100.0,
            "n_levels_survived": len(survived),
            "nominal": nominal,
            "per_scale": {str(s): per_scale[s] for s in HAIRCUTS},
        })

    # Rank: most thrust loss tolerated, then highest gate, then most payload.
    ranked.sort(key=lambda r: (-r["thrust_loss_tolerated_pct"], -r["gate"],
                               -r["nominal"]["payload_margin_kg"]))

    print(f"\n{'gate':>5} {'top':>6} {'dive':>5} {'climb':>6} {'n':>4} "
          f"{'-thrust%':>9} {'payload':>8} {'peakTW':>7} {'n_tot':>6} "
          f"{'trav g':>7} {'exitM':>6}")
    for r in ranked[:25]:
        nom = r["nominal"]
        print(f"{r['gate']:5.2f} {r['top_m']:6.0f} {r['dive_deg']:5.0f} "
              f"{r['climb_deg']:6.0f} {r['pullout_n']:4.1f} "
              f"{r['thrust_loss_tolerated_pct']:9.1f} "
              f"{nom['payload_margin_kg']:8.3f} {nom['peak_tw']:7.2f} "
              f"{nom['peak_n_total']:6.2f} {nom['traverse_g']:7.3f} "
              f"{nom['dive_exit_mach']:6.3f}")

    best_by_gate: dict[float, dict] = {}
    for r in ranked:
        best_by_gate.setdefault(r["gate"], r)
    print("\nMost robust design at each gate:")
    for g in sorted(best_by_gate):
        r = best_by_gate[g]
        nom = r["nominal"]
        print(f"  gate {g:.2f}: tolerates -{r['thrust_loss_tolerated_pct']:.1f}% thrust, "
              f"top {r['top_m']:.0f} m, dive {r['dive_deg']:.0f} deg, "
              f"climb {r['climb_deg']:.0f} deg, {r['pullout_n']:.1f} g nominal "
              f"-> payload {nom['payload_margin_kg']:.2f} kg, "
              f"peak n {nom['peak_n_total']:.2f}, T/W {nom['peak_tw']:.2f}")

    Path("out_simple_model/v4_robust.json").write_text(json.dumps({
        "haircuts": HAIRCUTS,
        "ranked": ranked,
        "best_by_gate": best_by_gate,
        "fp_pulsejet_ceiling_m": FP_PULSEJET_CEILING_M,
        "max_peak_load_n": MAX_PEAK_LOAD_N,
    }, indent=2))
    print("\nwrote out_simple_model/v4_robust.json")


if __name__ == "__main__":
    main()
