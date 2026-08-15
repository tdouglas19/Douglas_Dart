"""What does a real tail cost the V4 mission?

Asked by the VSP model (scripts/vsp_model/OPEN_QUESTIONS_FOR_V4_SIM.md §1):
VSPAERO finds the as-sized V4 statically unstable in BOTH pitch and yaw, the
fix is fin area, and the trajectory model has never carried a tail.

Two separate costs, and they are NOT symmetric:

  MASS  -- costs nothing in the trajectory. The vehicle always launches at the
           fixed 50 lb wet mass, so fin mass comes straight out of PAYLOAD
           MARGIN and the flight is bit-identical. Reported analytically.
  DRAG  -- costs real mission. Flown here.

Usage:
  python scripts/medium_model_v4_fin_sensitivity.py            # closed-form sweep
  python scripts/medium_model_v4_fin_sensitivity.py --fp --area 0.21
"""
from __future__ import annotations

import argparse
import json
import os
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.constants import WING_AREAL_MASS_KG_M2  # noqa: E402
from medium_model.design import fly, load_frozen_design  # noqa: E402
from medium_model.mission import MAX_WET_MASS_KG  # noqa: E402

OUT = ROOT / "out_medium_model"
FREEZE = ROOT / "docs" / "v4_medium_frozen" / "design.json"
# The areas the VSP model tabulated, plus zero as the frozen baseline.
AREAS = [0.0, 0.053, 0.21, 0.27, 0.35]
FIN = dict(aspect_ratio=1.20, thickness_ratio=0.04, sweep_deg=35.0)


def run(d, fz, area, use_fp):
    fin = dict(FIN, area_m2=area) if area > 0.0 else None
    propulsion = None
    if use_fp:
        from medium_model.fp_propulsion import FpPropulsion
        from medium_model.fp_spec import spec_from_geometry
        f = fz["fidelity"]
        propulsion = FpPropulsion(
            spec_from_geometry(
                d.geometry,
                chamber_diameter_fraction=f["chamber_diameter_fraction"]),
            fuel="propane", lightoff_mach=d.ramjet_start.gate_mach,
            n_cells=f["n_cells"], mach_step=f["mach_step"],
            altitude_step_m=f["altitude_step_m"])
    r = fly(d, drag_model="buildup", propulsion=propulsion, dt_s=0.02,
            max_fuel_burn_kg=d.burn_limit_kg, max_time_s=600.0, fin=fin)
    st = r.states
    powered = [s for s in st if s.thrust_n > 0.0]
    return {
        "fin_area_m2": area,
        "fin_mass_kg": WING_AREAL_MASS_KG_M2 * area,
        "peak_mach": max(s.mach for s in st),
        "cutoff": bool(r.motor_cutoff_reached),
        "dive_exit_mach": r.dive_exit_mach,
        "ramjet_lightoff_mach": r.ramjet_lightoff_mach,
        "ramjet_lightoff_mode": r.ramjet_lightoff_mode,
        "fuel_kg": max(s.fuel_burned_kg for s in st),
        "burn_cap_kg": d.burn_limit_kg,
        "tank_dry": max(s.fuel_burned_kg for s in st) >= d.burn_limit_kg - 1e-6,
        "traverse_g": r.min_traverse_accel_g,
        "powered_g": r.min_powered_accel_g,
        "peak_load_n_total": r.peak_load_n_total,
        "min_powered_altitude_m": r.min_powered_altitude_m,
        "floor_violated": bool(r.floor_violated),
        "mean_drag_strip_drag_n": (
            sum(s.drag_n for s in st if s.mode == "drag_strip")
            / max(sum(1 for s in st if s.mode == "drag_strip"), 1)),
        "powered_time_s": powered[-1].time_s if powered else 0.0,
        "fp_runs": propulsion.n_transients if propulsion else 0,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fp", action="store_true",
                    help="first-principles engines (~35 min PER area)")
    ap.add_argument("--area", type=float, action="append",
                    help="fin area m^2; repeatable. Default: the VSP table")
    args = ap.parse_args()
    areas = args.area if args.area else AREAS

    fz = json.loads(FREEZE.read_text())
    d = load_frozen_design(ROOT / "docs" / "v4_frozen" / "design.json")
    d = replace(d, ramjet_start=replace(d.ramjet_start,
                                        light_at_pullout=True))

    vm = json.loads((ROOT / "docs/v4_frozen/design.json").read_text()
                    )["verified_mission"]
    base_margin = vm["payload_margin_kg"]

    print(f"V4 + tail sensitivity -- "
          f"{'FIRST-PRINCIPLES' if args.fp else 'closed-form'} engines, "
          f"build-up drag")
    print(f"fin: AR {FIN['aspect_ratio']}, t/c {FIN['thickness_ratio']}, "
          f"sweep {FIN['sweep_deg']} deg, areal mass "
          f"{WING_AREAL_MASS_KG_M2} kg/m^2")
    print(f"wet mass is FIXED at {MAX_WET_MASS_KG:.3f} kg, so fin mass comes "
          f"out of the {base_margin:.3f} kg payload margin and does NOT "
          f"change the flight\n")

    rows = []
    for a in areas:
        r = run(d, fz, a, args.fp)
        r["payload_margin_kg"] = base_margin - r["fin_mass_kg"]
        rows.append(r)
        print(f"  area {a:.3f} m^2 -> peak M {r['peak_mach']:.3f}  "
              f"cutoff {str(r['cutoff']):5}  fuel {r['fuel_kg']:.3f}/"
              f"{r['burn_cap_kg']:.3f}  margin {r['payload_margin_kg']:.3f} kg",
              flush=True)

    tag = "fp" if args.fp else "closedform"
    p = OUT / f"v4_fin_sensitivity_{tag}.json"
    p.write_text(json.dumps(rows, indent=2), encoding="utf-8")

    base = rows[0]
    print(f"\n{'fin m^2':>8} {'fin kg':>7} {'payload kg':>10} {'peak M':>7} "
          f"{'cutoff':>7} {'fuel kg':>8} {'dry?':>5} {'strip D':>8} "
          f"{'trav g':>7} {'dM':>7}")
    for r in rows:
        print(f"{r['fin_area_m2']:8.3f} {r['fin_mass_kg']:7.2f} "
              f"{r['payload_margin_kg']:10.3f} {r['peak_mach']:7.3f} "
              f"{str(r['cutoff']):>7} {r['fuel_kg']:8.3f} "
              f"{str(r['tank_dry']):>5} {r['mean_drag_strip_drag_n']:8.1f} "
              f"{r['traverse_g']:7.3f} "
              f"{r['peak_mach']-base['peak_mach']:+7.3f}")
    print(f"\nwrote {p}")


if __name__ == "__main__":
    main()
