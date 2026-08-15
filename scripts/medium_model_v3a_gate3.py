"""V3a through the medium_model fidelity ladder (Gate 3).

V3a = the frozen V3 climb-dive design with its tail extended from 570 to
1027 mm (tail/chamber-diameter 2.66 -> 4.80) so the pulsejet can actually
sustain a cycle -- V3 as frozen produces 0.8 N, a dead duct.

Cheap rungs first: if V3a fails on aerodynamics alone there is no point
spending FP compute on it.
"""
from __future__ import annotations

import json, os, sys
from pathlib import Path

DESIGN = "docs/v3a_medium_model/design.json"
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import fly, load_frozen_design  # noqa: E402


def summarize(name, r, extra=""):
    peak_mach = max(s.mach for s in r.states)
    return {
        "step": name, "peak_mach": peak_mach,
        "cutoff": r.motor_cutoff_reached,
        "above_mach_1": peak_mach >= 1.0,
        "peak_tw": max(s.thrust_to_weight for s in r.states),
        "min_traverse_g": r.min_traverse_accel_g,
        "min_powered_g": r.min_powered_accel_g,
        "min_margin": r.min_powered_thrust_margin,
        "rule_ok": not r.rule_violated,
        "top_alt_m": r.climb_dive_top_altitude_m,
        "fuel_kg": max(s.fuel_burned_kg for s in r.states),
        "safe_land": r.safe_landing, "stalled": r.stalled,
        "lands_from_launch_m": abs(r.states[-1].distance_m),
        "flight_s": r.states[-1].time_s, "extra": extra,
    }


def main():
    d = load_frozen_design(DESIGN)
    c = d.raw["vehicle_candidate"]
    duct = c["chamber_length_m"] + c["throat_length_m"]
    body = duct + 3 * c["diameter_m"]
    print(f"V3a: body {c['diameter_m']*1e3:.0f} mm dia x {body*1e3:.0f} mm "
          f"(fineness {body/c['diameter_m']:.2f}), duct {duct*1e3:.0f} mm")
    print(f"     climb-dive: climb {d.climb_dive.initial_climb_angle_deg:.1f} deg,"
          f" dive {d.climb_dive.dive_angle_deg:.1f} deg,"
          f" floor {d.climb_dive.floor_altitude_m:.0f} m")
    print(f"     loaded fuel {d.loaded_fuel_kg:.3f} kg -> burn limit "
          f"{d.burn_limit_kg:.3f} kg (90%)\n")

    rows = [summarize("0: legacy drag", fly(d, drag_model="legacy")),
            summarize("1: drag build-up", fly(d, drag_model="buildup"))]

    if "--fp" in sys.argv:
        from medium_model.fp_propulsion import FpPropulsion
        from medium_model.fp_spec import spec_from_geometry
        spec = spec_from_geometry(d.geometry)
        fp = FpPropulsion(spec, fuel=c["fuel_key"],
                          lightoff_mach=d.raw["constants_at_freeze"]
                          ["RAMJET_MIN_LIGHTOFF_MACH"], n_cells=162)
        r = fly(d, drag_model="buildup", propulsion=fp)
        rows.append(summarize("2: + FP propulsion", r,
                              f"{fp.n_transients} FP runs"))
        print("\nFP engine events:")
        for t, what in fp.trace.events:
            print(f"   t={t:6.2f}s  {what}")
        Path("out_medium_model").mkdir(exist_ok=True)
        Path("out_medium_model/v3a_fp_trace.json").write_text(json.dumps(
            {k: v for k, v in vars(fp.trace).items()}, indent=1, default=str))

    keys = [("peak_mach", "peak Mach", "{:.3f}"),
            ("above_mach_1", "ABOVE MACH 1", "{}"),
            ("cutoff", "cutoff reached", "{}"),
            ("peak_tw", "peak T/W", "{:.2f}"),
            ("min_traverse_g", "min traverse g", "{:.3f}"),
            ("min_powered_g", "min powered g", "{:.3f}"),
            ("min_margin", "min margin", "{:.2f}"),
            ("rule_ok", "gamma rule ok", "{}"),
            ("top_alt_m", "top of climb m", "{:.0f}"),
            ("fuel_kg", "fuel burned kg", "{:.3f}"),
            ("safe_land", "safe landing", "{}"),
            ("lands_from_launch_m", "lands from launch m", "{:.0f}"),
            ("flight_s", "flight time s", "{:.0f}")]
    w = max(len(l) for _, l, _ in keys) + 2
    print("\n" + " " * w + "".join(f"{r['step']:>22}" for r in rows))
    print("-" * (w + 22 * len(rows)))
    for k, label, fmt in keys:
        line = f"{label:<{w}}"
        for r in rows:
            line += f"{fmt.format(r[k]):>22}"
        print(line)
    for r in rows:
        if r["extra"]:
            print(f"\n{r['step']}: {r['extra']}")
    Path("out_medium_model").mkdir(exist_ok=True)
    Path("out_medium_model/v3a_gate3.json").write_text(
        json.dumps(rows, indent=2, default=str))


if __name__ == "__main__":
    main()
