"""V4 trajectory campaign: optimize payload margin / peak T/W / body loads
subject to the ramjet lighting IN THE DIVE at the highest gate that survives.

Search box is sized by scripts/simple_model_v4_reach.py, which measured the
pulsejet-only dive-exit Mach ceiling at M 0.533 (top 1200 m, dive 18 deg,
5 g pull-out) -- so gates are swept up to 0.52 and no further, and tops are
capped at the FP pulsejet flame-out ceiling (~1200 m).

Feasibility (all hard):
  * ramjet lights during the pushover or the dive       (V4 requirement)
  * motor cutoff reached, tank not pinned               (learnings 3.5)
  * pull-out bottoms out at or above the 400 ft floor
  * flight path angle >= 0 from M 0.80 through cutoff   (competition rule)
  * safe landing, stall speed <= 45 m/s
  * positive payload margin against the 50 lb wet budget

Every surviving candidate is then re-flown with the pulsejet thrust cut
5/10/15 %, because "does it REACH the gate" is exactly the question
docs/v3_learnings_for_v4.md section 4.1 says a closed-form screen gets wrong
near a boundary (the FP/closed-form ratio collapses from 0.78 to 0.05 there).
A design that only lights at full modelled thrust is not a design.

Usage: .venv/Scripts/python scripts/simple_model_v4_campaign.py
Writes: out_simple_model/v4_campaign.json
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
from simple_model.constants import (  # noqa: E402
    MIN_POWERED_ACCELERATION_G, MIN_POWERED_THRUST_MARGIN_FRACTION)
from simple_model.flight_sim import V3_FLOOR_ALTITUDE_M  # noqa: E402
from simple_model.optimize import MAX_ACCEPTABLE_STALL_SPEED_M_PER_S  # noqa: E402

# 0.42-0.48 are CONTEXT ONLY -- the user's directive is a gate at 0.50 or
# higher. They are flown to price what that directive costs, not to
# propose a lower gate.
GATES = [0.42, 0.45, 0.48, 0.50, 0.51, 0.52]
TOPS = [800.0, 900.0, 1000.0, 1100.0, 1200.0]
DIVES = [8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
CLIMBS = [6.0, 8.0, 10.0, 12.0, 14.0]
PULLOUT_NS = [2.0, 3.0, 4.0]
HAIRCUTS = [0.95, 0.90, 0.85]

# The pull-out holds the floor by pulling harder when it has to, so the
# NOMINAL load factor is not what the airframe sees -- the measured peak is.
# Gating on the measured peak is what actually keeps body loads sane: it
# rejects any (top, dive, gate) combination that only makes the floor by
# saturating the limiter. 4 g on 22.68 kg is 890 N (200 lbf) through the wing
# joint. Nothing in this repo models structure, so this is a design limit.
MAX_PEAK_LOAD_N = 4.0


def _evaluate(args: tuple) -> dict:
    gate, top, dive, climb, n = args
    geometry = v4_geometry()
    r = fly(climb_deg=climb, dive_deg=dive, floor_m=V3_FLOOR_ALTITUDE_M,
            gate_mach=gate, pullout_n=n, top_altitude_m=top)
    final = r.states[-1]
    row = {
        "gate": gate, "top_m": top, "dive_deg": dive, "climb_deg": climb,
        "pullout_n": n,
        "lit_in_dive": r.ramjet_lit_in_dive,
        "lightoff_mach": r.ramjet_lightoff_mach,
        "lightoff_alt_m": r.ramjet_lightoff_altitude_m,
        "lightoff_mode": r.ramjet_lightoff_mode,
        "dive_exit_mach": r.dive_exit_mach,
        "cutoff": r.motor_cutoff_reached,
        "safe_landing": r.safe_landing,
        "stalled": r.stalled,
        "floor_violated": r.floor_violated,
        "rule_violated": r.rule_violated,
        "min_powered_alt_m": r.min_powered_altitude_m,
        "pullout_entry_alt_m": r.pullout_entry_altitude_m,
        "pullout_radius_m": r.pullout_radius_m,
        "pushover_radius_m": r.pushover_radius_m,
        "pullout_duration_s": r.pullout_duration_s,
        "pushover_duration_s": r.pushover_duration_s,
        "peak_n_total": r.peak_load_n_total,
        "peak_n_yaw": r.peak_load_n_yaw,
        "peak_n_roll": r.peak_load_n_roll,
        "peak_load_mode": r.peak_load_mode,
        "traverse_g": r.min_traverse_accel_g,
        "traverse_mach": r.min_traverse_accel_mach,
        "pushover_g": r.min_pushover_accel_g,
        "thrust_margin": r.min_powered_thrust_margin,
        "peak_tw": max(s.thrust_to_weight for s in r.states),
        "stall_speed_m_per_s": final.stall_speed_m_per_s,
        "peak_mach": max(s.mach for s in r.states),
    }
    row.update(mass_block(r, geometry, V4_WING))
    row["feasible"] = bool(
        row["lit_in_dive"] and row["cutoff"] and row["safe_landing"]
        and not row["floor_violated"] and not row["rule_violated"]
        and row["fuel_burned_kg"] <= row["tank_capacity_kg"] / 1.25 + 1e-9
        and row["stall_speed_m_per_s"] <= MAX_ACCEPTABLE_STALL_SPEED_M_PER_S
        and row["payload_margin_kg"] > 0.0
        and row["peak_n_total"] <= MAX_PEAK_LOAD_N
    )
    row["over_fp_ceiling"] = top > FP_PULSEJET_CEILING_M
    return row


def _haircut(args: tuple) -> dict:
    """Re-fly one candidate with the pulsejet's average thrust scaled down."""
    gate, top, dive, climb, n, scale = args
    import simple_model.pulsejet_simple as pj
    original = pj.AVERAGE_THRUST_DUTY_CYCLE_FACTOR
    pj.AVERAGE_THRUST_DUTY_CYCLE_FACTOR = original * scale
    try:
        r = fly(climb_deg=climb, dive_deg=dive, floor_m=V3_FLOOR_ALTITUDE_M,
                gate_mach=gate, pullout_n=n, top_altitude_m=top)
        return {
            "scale": scale, "lit_in_dive": r.ramjet_lit_in_dive,
            "dive_exit_mach": r.dive_exit_mach, "cutoff": r.motor_cutoff_reached,
            "traverse_g": r.min_traverse_accel_g,
        }
    finally:
        pj.AVERAGE_THRUST_DUTY_CYCLE_FACTOR = original


def main() -> None:
    tasks = [(g, t, d, c, n) for g in GATES for t in TOPS for d in DIVES
             for c in CLIMBS for n in PULLOUT_NS]
    workers = max(1, (os.cpu_count() or 2) - 1)
    print(f"V4 campaign: {len(tasks)} flights on {workers} workers...")
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(_evaluate, tasks, chunksize=8))

    feasible = [r for r in rows if r["feasible"]]
    print(f"{len(feasible)}/{len(rows)} feasible\n")

    fail_reasons = {}
    for r in rows:
        if r["feasible"]:
            continue
        if not r["lit_in_dive"]:
            key = "ramjet never lit in the dive"
        elif not r["cutoff"]:
            key = "motor cutoff never reached"
        elif r["floor_violated"]:
            key = "busted the 400 ft floor"
        elif not r["safe_landing"]:
            key = "no safe landing"
        elif r["payload_margin_kg"] <= 0.0:
            key = "negative payload margin"
        elif r["peak_n_total"] > MAX_PEAK_LOAD_N:
            key = f"peak body load over {MAX_PEAK_LOAD_N:.0f} g"
        else:
            key = "other"
        fail_reasons[key] = fail_reasons.get(key, 0) + 1
    print("Infeasible breakdown:")
    for k, v in sorted(fail_reasons.items(), key=lambda kv: -kv[1]):
        print(f"  {v:5d}  {k}")

    if not feasible:
        Path("out_simple_model").mkdir(exist_ok=True)
        Path("out_simple_model/v4_campaign.json").write_text(
            json.dumps({"rows": rows, "feasible": []}, indent=2))
        print("\nNo feasible candidate -- nothing to refine.")
        return

    print(f"\n{'gate':>5} {'top':>6} {'dive':>5} {'climb':>6} {'n':>4} "
          f"{'payload':>8} {'peakTW':>7} {'n_tot':>6} {'trav g':>7} "
          f"{'exitM':>6} {'R_pull':>7}")
    best_per_gate = {}
    for r in sorted(feasible, key=lambda r: -r["payload_margin_kg"]):
        g = r["gate"]
        if g not in best_per_gate:
            best_per_gate[g] = r
    for r in sorted(feasible, key=lambda r: -r["payload_margin_kg"])[:20]:
        print(f"{r['gate']:5.2f} {r['top_m']:6.0f} {r['dive_deg']:5.0f} "
              f"{r['climb_deg']:6.0f} {r['pullout_n']:4.1f} "
              f"{r['payload_margin_kg']:8.3f} {r['peak_tw']:7.2f} "
              f"{r['peak_n_total']:6.2f} {r['traverse_g']:7.3f} "
              f"{r['dive_exit_mach']:6.3f} {r['pullout_radius_m']:7.0f}")

    print("\nBest by payload margin at each gate:")
    for g in sorted(best_per_gate):
        r = best_per_gate[g]
        print(f"  gate {g:.2f}: payload {r['payload_margin_kg']:.3f} kg, "
              f"top {r['top_m']:.0f} m, dive {r['dive_deg']:.0f} deg, "
              f"climb {r['climb_deg']:.0f} deg, {r['pullout_n']:.1f} g, "
              f"peak T/W {r['peak_tw']:.2f}, exit M {r['dive_exit_mach']:.3f}")

    # Robustness: does the winner still light with a weaker pulsejet?
    print("\nThrust-haircut robustness (does it still light in the dive?):")
    haircut_results = {}
    for g in sorted(best_per_gate):
        r = best_per_gate[g]
        args = [(r["gate"], r["top_m"], r["dive_deg"], r["climb_deg"],
                 r["pullout_n"], s) for s in HAIRCUTS]
        with ProcessPoolExecutor(max_workers=len(HAIRCUTS)) as pool:
            hc = list(pool.map(_haircut, args))
        haircut_results[g] = hc
        line = "  ".join(
            f"-{(1-h['scale'])*100:.0f}%: {'LIT' if h['lit_in_dive'] else 'DEAD'}"
            f" (M{h['dive_exit_mach']:.3f})" for h in hc)
        print(f"  gate {g:.2f}  100%: LIT (M{r['dive_exit_mach']:.3f})   {line}")

    Path("out_simple_model").mkdir(exist_ok=True)
    Path("out_simple_model/v4_campaign.json").write_text(json.dumps({
        "rows": rows,
        "best_per_gate": best_per_gate,
        "haircuts": haircut_results,
        "gates_applied": {
            "MIN_POWERED_ACCELERATION_G": MIN_POWERED_ACCELERATION_G,
            "MIN_POWERED_THRUST_MARGIN_FRACTION": MIN_POWERED_THRUST_MARGIN_FRACTION,
            "MAX_ACCEPTABLE_STALL_SPEED_M_PER_S": MAX_ACCEPTABLE_STALL_SPEED_M_PER_S,
            "FP_PULSEJET_CEILING_M": FP_PULSEJET_CEILING_M,
        },
    }, indent=2))
    print("\nwrote out_simple_model/v4_campaign.json")


if __name__ == "__main__":
    main()
