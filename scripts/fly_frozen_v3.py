"""Re-fly the frozen V3 climb-dive design and print its mission numbers as
JSON.

Standalone for the same reason as scripts/fly_frozen_v2.py: CD0 is baked
into simple_model.constants at IMPORT time and re-exported by
drag/flight_sim, so a test cannot reliably override it in-process once any
other test module has imported simple_model. Running the flight in its own
process makes the frozen check independent of test module import order.

Usage: python scripts/fly_frozen_v3.py [path/to/design.json]
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DESIGN_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT / "docs" / "v3_frozen" / "design.json")
DESIGN = json.loads(DESIGN_PATH.read_text())
os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(
    DESIGN["constants_at_freeze"]["CD0_FRONTAL"])

from simple_model.constants import AIRFOILS, FUELS  # noqa: E402
from simple_model.drag import WingConcept  # noqa: E402
from simple_model.flight_sim import VehicleGeometry, run_flight  # noqa: E402
from simple_model.optimize import (  # noqa: E402
    Candidate, FUEL_RESERVE_MARGIN, FUEL_VOLUME_FRACTION_OF_ANNULUS,
    MAX_WET_MASS_KG, MOTOR_CUTOFF_MACH, _annular_volume_m3)


def main() -> None:
    cand = Candidate(**DESIGN["vehicle_candidate"])
    w = DESIGN["wing_concept"]
    geometry = VehicleGeometry(
        cand.diameter_m, cand.throat_diameter_m, cand.chamber_length_m,
        cand.throat_length_m, cand.wingspan_m, FUELS[cand.fuel_key])
    concept = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                          w["sweep_deg"], AIRFOILS[w["airfoil_key"]])
    tank = (FUEL_VOLUME_FRACTION_OF_ANNULUS
            * _annular_volume_m3(cand.diameter_m, cand.throat_diameter_m,
                                 cand.throat_length_m)
            * FUELS[cand.fuel_key].density_kg_per_m3)
    r = run_flight(geometry, MAX_WET_MASS_KG,
                   climb_angle_deg=cand.climb_angle_deg,
                   motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02,
                   max_time_s=900.0, wing_concept=concept,
                   max_fuel_burn_kg=tank / (1.0 + FUEL_RESERVE_MARGIN),
                   return_to_launch=True, climb_dive=cand.to_climb_dive())
    modes = [s.mode for s in r.states]
    dive = [s for s in r.states if s.mode == "v3_dive"]
    print("FLIGHT_JSON:" + json.dumps({
        "motor_cutoff_reached": r.motor_cutoff_reached,
        "safe_landing": r.safe_landing,
        "stalled": r.stalled,
        "hit_mass_floor": r.hit_mass_floor,
        "peak_thrust_to_weight": max(s.thrust_to_weight for s in r.states),
        "min_traverse_accel_g": r.min_traverse_accel_g,
        "min_traverse_accel_mach": r.min_traverse_accel_mach,
        "min_powered_accel_g": r.min_powered_accel_g,
        "min_powered_thrust_margin": r.min_powered_thrust_margin,
        "climb_dive_top_altitude_m": r.climb_dive_top_altitude_m,
        "rule_violated": r.rule_violated,
        "modes_ordered": sorted(set(modes), key=modes.index),
        "dive_min_altitude_m": min((s.altitude_m for s in dive), default=None),
        "final_distance_m": r.states[-1].distance_m,
        "max_distance_m": max(s.distance_m for s in r.states),
    }))


if __name__ == "__main__":
    main()
