"""Re-fly the frozen medium_model V4 and print its numbers as JSON.

Standalone for the same reason as scripts/fly_frozen_v{2,3,4}.py: CD0 is baked
into medium_model.constants at IMPORT time and re-exported by drag/flight_sim,
so a test cannot reliably override it in-process once any other test module has
imported medium_model. Running the flight in its own process makes the frozen
check independent of test module import order.

COSTS ~35 MINUTES. Every propulsion re-convergence is a first-principles
transient at CONFIRM resolution (324 cells). tests/test_v4_medium_frozen.py
does not run this by default -- see its docstring.

Usage: python scripts/fly_frozen_v4_medium.py
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

FREEZE = ROOT / "docs" / "v4_medium_frozen" / "design.json"
DESIGN = json.loads(FREEZE.read_text())
os.environ["MEDIUM_MODEL_CD0_FRONTAL"] = str(
    DESIGN["constants_at_freeze"]["CD0_FRONTAL"])

from medium_model.design import fly, load_frozen_design  # noqa: E402
from medium_model.fp_propulsion import FpPropulsion  # noqa: E402
from medium_model.fp_spec import spec_from_geometry  # noqa: E402


def main() -> None:
    f = DESIGN["fidelity"]
    d = load_frozen_design(ROOT / "docs" / "v4_frozen" / "design.json")
    # The one thing added on top of the simple_model freeze: the last-chance
    # lightoff policy. Everything else is that file, unmodified.
    d = replace(d, ramjet_start=replace(d.ramjet_start,
                                        light_at_pullout=True))

    propulsion = FpPropulsion(
        spec_from_geometry(
            d.geometry,
            chamber_diameter_fraction=f["chamber_diameter_fraction"]),
        fuel="propane", lightoff_mach=d.ramjet_start.gate_mach,
        n_cells=f["n_cells"], mach_step=f["mach_step"],
        altitude_step_m=f["altitude_step_m"])

    r = fly(d, drag_model=f["drag_model"], propulsion=propulsion,
            dt_s=f["dt_s"], max_fuel_burn_kg=d.burn_limit_kg, max_time_s=600.0)
    st = r.states
    print("FLIGHT_JSON:" + json.dumps({
        "peak_mach": max(s.mach for s in st),
        "cutoff": bool(r.motor_cutoff_reached),
        "dive_exit_mach": r.dive_exit_mach,
        "ramjet_lightoff_mach": r.ramjet_lightoff_mach,
        "ramjet_lightoff_altitude_m": r.ramjet_lightoff_altitude_m,
        "ramjet_lightoff_time_s": r.ramjet_lightoff_time_s,
        "ramjet_lightoff_mode": r.ramjet_lightoff_mode,
        "ramjet_lit_in_dive": bool(r.ramjet_lit_in_dive),
        "ramjet_light_refused": bool(r.ramjet_light_refused),
        "peak_tw": max(s.thrust_to_weight for s in st),
        "traverse_g": r.min_traverse_accel_g,
        "powered_g": r.min_powered_accel_g,
        "margin": r.min_powered_thrust_margin,
        "peak_load_n_total": r.peak_load_n_total,
        "peak_load_n_yaw": r.peak_load_n_yaw,
        "peak_load_n_roll": r.peak_load_n_roll,
        "peak_load_mode": r.peak_load_mode,
        "pushover_radius_m": r.pushover_radius_m,
        "pullout_radius_m": r.pullout_radius_m,
        "spiral_radius_m": r.spiral_radius_m,
        "pushover_duration_s": r.pushover_duration_s,
        "pullout_duration_s": r.pullout_duration_s,
        "min_powered_altitude_m": r.min_powered_altitude_m,
        "floor_violated": bool(r.floor_violated),
        "rule_violated": bool(r.rule_violated),
        "fuel_kg": max(s.fuel_burned_kg for s in st),
        "burn_cap_kg": d.burn_limit_kg,
        "loaded_fuel_kg": d.loaded_fuel_kg,
        "flight_s": st[-1].time_s,
        "stalled": bool(r.stalled),
        "safe_landing": bool(r.safe_landing),
        "lands_from_launch_m": abs(st[-1].distance_m),
        "fp_runs": propulsion.n_transients,
    }))


if __name__ == "__main__":
    main()
