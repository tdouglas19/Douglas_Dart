"""Fly the frozen V2 design through medium_model's fidelity ladder and
report how far the answer moves at each step (#39).

The point is attribution, not a single number: "the answer moved X% when
drag became real" is what should drive the next design iteration.

Usage: python scripts/medium_model_attribution.py
"""
from __future__ import annotations

import json
import os
from pathlib import Path

FROZEN = Path(__file__).resolve().parent.parent / "docs" / "v2_frozen" / "design.json"
DESIGN = json.loads(FROZEN.read_text())
_CD0 = str(DESIGN["constants_at_freeze"]["CD0_FRONTAL"])
os.environ.setdefault("SIMPLE_MODEL_CD0_FRONTAL", _CD0)
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", _CD0)

from medium_model.constants import AIRFOILS, FUELS  # noqa: E402
from medium_model.drag import WingConcept  # noqa: E402
from medium_model.flight_sim import VehicleGeometry, run_flight  # noqa: E402
from medium_model.mission import (MAX_WET_MASS_KG, MOTOR_CUTOFF_MACH,  # noqa: E402
                                  usable_fuel_kg)


def fly(drag_model: str, cowl_suction_recovery: float = 0.85):
    c = DESIGN["vehicle_candidate"]
    w = DESIGN["wing_concept"]
    geometry = VehicleGeometry(
        c["diameter_m"], c["throat_diameter_m"], c["chamber_length_m"],
        c["throat_length_m"], c["wingspan_m"], FUELS[c["fuel_key"]])
    concept = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                          w["sweep_deg"], AIRFOILS[w["airfoil_key"]])
    burn = usable_fuel_kg(c["diameter_m"], c["throat_diameter_m"],
                          c["throat_length_m"],
                          FUELS[c["fuel_key"]].density_kg_per_m3)
    return run_flight(
        geometry, MAX_WET_MASS_KG, climb_angle_deg=c["climb_angle_deg"],
        motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02, max_time_s=900.0,
        wing_concept=concept, max_fuel_burn_kg=burn, return_to_launch=True,
        drag_model=drag_model,
        cowl_suction_recovery=cowl_suction_recovery)


def summarize(name: str, r) -> dict:
    peak_mach = max(s.mach for s in r.states)
    peak_tw = max(s.thrust_to_weight for s in r.states)
    return {
        "step": name,
        "peak_mach": peak_mach,
        "cutoff_reached": r.motor_cutoff_reached,
        "peak_tw": peak_tw,
        "min_accel_g": r.min_powered_accel_g,
        "min_accel_mach": r.min_accel_mach,
        "min_margin": r.min_powered_thrust_margin,
        "safe_landing": r.safe_landing,
        "stalled": r.stalled,
        "range_m": max(s.distance_m for s in r.states),
        "lands_from_launch_m": abs(r.states[-1].distance_m),
        "flight_time_s": r.states[-1].time_s,
    }


def main():
    # The spillage term's treatment on a NOSE inlet is genuinely ambiguous
    # without CFD (see drag_buildup.spillage_drag_n), so it is BRACKETED
    # rather than picked: recovery 1.0 = the forebody terms already contain
    # it, 0.85 = nominal residual, 0.5 = podded-nacelle-style full charge.
    rows = []
    rows.append(summarize("0: copy (legacy)", fly("legacy")))
    rows.append(summarize("1: buildup, no spill", fly("buildup", 1.0)))
    rows.append(summarize("1b: buildup, spill 0.85", fly("buildup", 0.85)))
    rows.append(summarize("1c: buildup, spill 0.5", fly("buildup", 0.5)))

    keys = [("peak_mach", "peak M", "{:.3f}"),
            ("cutoff_reached", "cutoff", "{}"),
            ("peak_tw", "peak T/W", "{:.2f}"),
            ("min_accel_g", "min accel g", "{:.2f}"),
            ("min_margin", "min margin", "{:.2f}"),
            ("safe_landing", "safe land", "{}"),
            ("range_m", "range m", "{:.0f}"),
            ("lands_from_launch_m", "lands from launch m", "{:.0f}"),
            ("flight_time_s", "flight s", "{:.0f}")]

    width = max(len(label) for _, label, _ in keys) + 2
    print(f"\nFrozen V2 through the medium_model fidelity ladder\n")
    header = " " * width + "".join(f"{r['step']:>26}" for r in rows)
    print(header)
    print("-" * len(header))
    for key, label, fmt in keys:
        line = f"{label:<{width}}"
        for r in rows:
            line += f"{fmt.format(r[key]):>26}"
        print(line)

    base, new = rows[0], rows[1]
    print("\nDeltas (step 1 vs step 0):")
    for key, label, _ in keys:
        a, b = base[key], new[key]
        if isinstance(a, bool) or isinstance(b, bool):
            if a != b:
                print(f"  {label}: {a} -> {b}   <-- CHANGED")
            continue
        if a:
            print(f"  {label}: {a:.3g} -> {b:.3g}  ({(b - a) / abs(a) * 100:+.1f}%)")
    out = Path("out_medium_model")
    out.mkdir(exist_ok=True)
    (out / "attribution.json").write_text(json.dumps(rows, indent=2, default=str))
    print(f"\njson: {out / 'attribution.json'}")


if __name__ == "__main__":
    main()
