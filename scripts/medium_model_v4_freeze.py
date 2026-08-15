"""Freeze the medium_model V4 result into docs/v4_medium_frozen/design.json.

ONE flight: build-up drag + first-principles engines at CONFIRM (324 cells),
chamber = body diameter, with RamjetStart.light_at_pullout. That is the V4
result (user, 2026-08-14) -- the fidelity-ladder rungs it was established
against are not part of the freeze.

Same contract as docs/v{2,3,4}_frozen: re-flyable INPUTS + the constants they
were flown under + the git SHA + the verified numbers. Built by a script
rather than hand-written so the recorded results cannot drift from the flight
they came from.

Usage: python scripts/medium_model_v4_freeze.py
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out_medium_model"
FROZEN_DIR = ROOT / "docs" / "v4_medium_frozen"
SOURCE_FREEZE = ROOT / "docs" / "v4_frozen" / "design.json"
TAG = "v4_rungc_lap"


# Recorded to 4 dp. The flight is deterministic, but the freeze is a
# CONTRACT, not a bit-comparison: a tolerance keeps it robust to a
# last-digit float change while still catching any real behavioural drift.
def r4(x):
    return None if x is None else (x if isinstance(x, (bool, str))
                                   else round(float(x), 4))


VERIFIED_FIELDS = [
    "peak_mach", "cutoff", "dive_exit_mach", "ramjet_lightoff_mach",
    "ramjet_lightoff_altitude_m", "ramjet_lightoff_time_s",
    "ramjet_lightoff_mode", "ramjet_lit_in_dive", "ramjet_light_refused",
    "peak_tw", "traverse_g", "powered_g", "pushover_g", "margin",
    "peak_load_n_total", "peak_load_n_yaw", "peak_load_n_roll",
    "peak_load_mode", "pushover_radius_m", "pullout_radius_m",
    "spiral_radius_m", "pushover_duration_s", "pullout_duration_s",
    "min_powered_altitude_m", "floor_violated", "rule_violated",
    "fuel_kg", "burn_cap_kg", "loaded_fuel_kg", "tank_dry", "flight_s",
    "stalled", "safe_landing", "lands_from_launch_m", "fp_runs",
]
PHASE_FIELDS = ["mode", "t0", "t1", "dt", "mach0", "mach1", "alt0", "alt1",
                "gamma0_deg", "gamma1_deg", "peak_load_n", "fuel_kg",
                "mean_thrust_n", "mean_drag_n", "min_accel_g"]


def main() -> None:
    src = json.loads(SOURCE_FREEZE.read_text())
    sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                         capture_output=True, text=True).stdout.strip()
    p = OUT / f"{TAG}_final.json"
    if not p.exists():
        raise SystemExit(f"missing {p} -- fly the design before freezing")
    s = json.loads(p.read_text())

    trajectory = dict(src["trajectory"])
    trajectory["ramjet_light_at_pullout"] = True

    doc = {
        "name": "V4 through medium_model -- build-up drag + first-principles "
                "engines; the ramjet lights at the pull-out, not the gate",
        "frozen_utc_date": "2026-08-14",
        "flight_dated": "2026-08-13",
        "git_sha": sha,
        "derived_from": "docs/v4_frozen/design.json (the simple_model V4 "
                        "freeze), flown UNMODIFIED -- no vehicle or "
                        "trajectory parameter was changed and no optimizer "
                        "was run. The only addition is the lightoff policy "
                        "flag below.",
        "headline": (
            "At first-principles fidelity the V4 airframe closes its mission "
            "with RamjetStart.light_at_pullout: the FP flameholder cold-lights "
            "at Mach 0.488 inside the pull-out -- below the Mach 0.50 gate the "
            "design was written around -- and the vehicle reaches Mach 1.100. "
            "The gate, not the engine, was the binding constraint. Without the "
            "override the vehicle never reaches the gate, so the ramjet is "
            "never even asked, and it burns its whole tank stranded at "
            "M ~0.44."),
        "how_to_refly": (
            "PYTHONPATH=. .venv/Scripts/python scripts/fly_frozen_v4_medium.py"
            "   -- ~35 minutes, because every propulsion re-convergence is a "
            "first-principles transient. tests/test_v4_medium_frozen.py "
            "therefore does NOT re-fly it by default: it checks the recorded "
            "numbers against the stored per-step trace "
            f"(out_medium_model/{TAG}_states.json.gz). Set "
            "DOUGLAS_DART_REFLY_FP_RUNGS=1 to force the full re-fly."),
        "vehicle": src["vehicle"],
        "wing_concept": src["wing_concept"],
        "trajectory": trajectory,
        "fidelity": {
            "drag_model": s["drag_model"],
            "propulsion": s["propulsion"],
            "n_cells": s["n_cells"],
            "tier": "CONFIRM (324 cells)",
            "chamber_diameter_fraction": s["chamber_diameter_fraction"],
            "mach_step": s["mach_step"],
            "altitude_step_m": s["altitude_step_m"],
            "dt_s": s["dt_s"],
            "fp_solves": s["fp_runs"],
        },
        "medium_model_settings": {
            "fuel_cap_rule": "90% of the design's LOADED allocation "
                             "(mission.py) -- a refinement model must not be "
                             "able to enlarge the vehicle it is evaluating",
            "chamber_diameter_fraction_note":
                "1.0 (user decision, 2026-08-13): V4's premise is that the "
                "chamber IS the body. The medium_model default is 0.95, kept "
                "so earlier FP results stay reproducible. It scales the "
                "chamber bore, the valve pack, the intake and the ramjet "
                "combustor together.",
            "ramjet_start_policy":
                "The gate decides when the ramjet is ASKED; the "
                "first-principles model keeps the right to refuse. "
                "light_at_pullout waives the Mach gate once the pull-out "
                "begins with the engine still unlit, and latches so the "
                "waiver survives into the drag strip.",
        },
        "constants_at_freeze": dict(src["constants_at_freeze"], **{
            "MEDIUM_MODEL_FUEL_BURN_FRACTION_OF_LOADED": 0.9,
            "MEDIUM_MODEL_FUEL_VOLUME_FRACTION_OF_ANNULUS": 0.5,
            "RAMJET_MIN_LIGHTOFF_MACH": 0.45,
            "RAMJET_LIGHTOFF_RAMP_MACH": 0.10,
            "RETURN_LOOP_LOAD_FACTOR": 6.0,
        }),
        "verified_mission": {f: r4(s.get(f)) for f in VERIFIED_FIELDS},
        "phases": [{k: r4(q[k]) for k in PHASE_FIELDS} for q in s["phases"]],
        "engine_events": s.get("events", []),
        "port_verification": {
            "v2_parity": "tests/test_medium_model_v2_parity.py green, "
                         "state-for-state against simple_model",
            "v3_parity": "the frozen V3 design flown through BOTH models and "
                         "compared step by step: 10,438 states, biggest "
                         "delta 0.000e+00",
            "v4_port_check": "flown with the legacy drag model and the "
                             "closed-form engines, this design reproduces "
                             "docs/v4_frozen/design.json to within 0.086%, "
                             "which is the rounding stored in that file",
        },
        "gates": {
            "peak_mach_above_1": "PASS (1.100)",
            "motor_cutoff_reached": "PASS",
            "400ft_floor_held": "PASS (flown minimum 121.74 m vs the 121.92 m "
                                "floor -- 0.18 m under, inside the 1.0 m "
                                "discretisation tolerance)",
            "peak_body_load_under_4g": "PASS for POWERED flight (3.65 g). See "
                                       "the structural finding below: the "
                                       "unpowered return loop is far higher.",
            "fuel_within_cap": "PASS (2.125 of 2.764 kg, 77%)",
            "ramjet_lights_in_the_dive": "FAIL -- it lights in the PULL-OUT "
                                         "(M 0.488, v4_pullout). That was the "
                                         "stated V4 requirement and this is a "
                                         "deliberate relaxation of it, not a "
                                         "satisfaction of it.",
            "safe_landing": "FAIL -- stalled 4.1 km from home. The "
                            "return-to-launch profile does not survive "
                            "build-up drag; independent of propulsion.",
            "min_traverse_accel_0.25g": "FAIL (-0.057 g, in the dive before "
                                        "lightoff)",
            "min_powered_accel_positive": "FAIL (-0.390 g at the top of "
                                          "climb, where the pulsejet quenches)",
            "min_powered_thrust_margin_1.15": "FAIL (0.000 -- the quench)",
        },
        "findings": [
            "The FP pulsejet QUENCHES TWICE on the climb -- M 0.333 at "
            "1038 m and M 0.356 at 948 m. The flame-out boundary at CONFIRM "
            "(324 cells) with a full-diameter chamber sits around 950-1100 m, "
            "LOWER than the ~1250 m the simple_model freeze assumed, and V4's "
            "commanded top of climb is 1100 m -- inside it.",
            "Because of that the top of climb is reached at M 0.284 instead "
            "of the closed-form M 0.381, the dive starts 0.097 Mach down, and "
            "dive exit is M 0.488 instead of 0.600 -- which is why the M 0.50 "
            "gate is never crossed and the override is what saves the "
            "mission.",
            "Across the 34 live points of the march the FP pulsejet makes "
            "90.6% of the closed-form thrust (worst live point 76.7%). The "
            "simple_model freeze's 10% thrust haircut was about right ON "
            "AVERAGE and still the wrong instrument: what strands the vehicle "
            "is the quench, which no scalar haircut can express.",
            "The ramjet lights at M 0.488 in v4_pullout, so "
            "ramjet_lit_in_dive is FALSE. The stated V4 requirement was that "
            "it light in the DIVE.",
            "The return-to-launch profile does not survive build-up drag: the "
            "half-loop bleeds the vehicle below stall and it lands 4.1 km "
            "from home. Independent of propulsion and untouched here.",
            "STRUCTURAL: the worst body load in the flight is NOT the "
            "pull-out. The unpowered return half-loop pulls a fixed "
            "RETURN_LOOP_LOAD_FACTOR = 6 g by construction, and total |n| "
            "there reaches 9.14 g. peak_load_n_total deliberately tracks "
            "POWERED steps only, so that load does not appear in it. Anything "
            "sizing structure must read the per-phase peak_load_n column.",
        ],
        "known_risks": [
            "Top of climb 1100 m is inside the measured FP flame-out band "
            "(~950-1100 m at CONFIRM). The quench is not a marginal effect: "
            "engine-only thrust margin is 0.000 and min powered acceleration "
            "-0.390 g.",
            "The M 0.488 cold light is 0.012 Mach below the gate the design "
            "was written around. It is ONE converged FP point, not a mapped "
            "boundary -- the lightoff margin below M 0.488 is unmeasured.",
            "The pull-out load factor sawtooths between ~1 and ~2 g through "
            "the drag strip: the floor-holding law responding to a stepped "
            "thrust input. Peak loads read off this flight carry that "
            "discretisation.",
            "min_traverse_accel_g, min_powered_accel_g and the engine-only "
            "thrust margin all FAIL. The last two fail on the CLIMB, which "
            "the lightoff override does not touch.",
            "Side valve runners still do not fit inside a 214 mm skin -- an "
            "external fairing is needed and its drag is in neither the flat "
            "CD0 nor the build-up.",
        ],
    }

    FROZEN_DIR.mkdir(parents=True, exist_ok=True)
    path = FROZEN_DIR / "design.json"
    path.write_text(json.dumps(doc, indent=2), encoding="utf-8")
    print(f"wrote {path}")
    v = doc["verified_mission"]
    print(f"  git sha {sha[:7]}  peak M {v['peak_mach']}  cutoff "
          f"{v['cutoff']}  ramjet M {v['ramjet_lightoff_mach']} in "
          f"{v['ramjet_lightoff_mode']}  fuel {v['fuel_kg']} kg")


if __name__ == "__main__":
    main()
