"""Re-fly the frozen V2 design and print its mission numbers as JSON.

Exists as a standalone script because CD0 is baked into
simple_model.constants at IMPORT time and re-exported by drag/flight_sim, so
a test cannot reliably override it in-process once any other test module has
already imported simple_model. Running the flight in its own process makes
the frozen check independent of test module import order.

Usage: python scripts/fly_frozen_v2.py [path/to/design.json]
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DESIGN_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT / "docs" / "v2_frozen" / "design.json")
DESIGN = json.loads(DESIGN_PATH.read_text())
os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(
    DESIGN["constants_at_freeze"]["CD0_FRONTAL"])

from simple_model.constants import AIRFOILS, FUELS  # noqa: E402
from simple_model.drag import WingConcept  # noqa: E402
from simple_model.flight_sim import VehicleGeometry, run_flight  # noqa: E402
from simple_model.optimize import (  # noqa: E402
    FUEL_RESERVE_MARGIN, FUEL_VOLUME_FRACTION_OF_ANNULUS, MAX_WET_MASS_KG,
    MOTOR_CUTOFF_MACH, _annular_volume_m3)


def main() -> None:
    c = DESIGN["vehicle_candidate"]
    w = DESIGN["wing_concept"]
    geometry = VehicleGeometry(
        c["diameter_m"], c["throat_diameter_m"], c["chamber_length_m"],
        c["throat_length_m"], c["wingspan_m"], FUELS[c["fuel_key"]])
    concept = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                          w["sweep_deg"], AIRFOILS[w["airfoil_key"]])
    tank = (FUEL_VOLUME_FRACTION_OF_ANNULUS
            * _annular_volume_m3(c["diameter_m"], c["throat_diameter_m"],
                                 c["throat_length_m"])
            * FUELS[c["fuel_key"]].density_kg_per_m3)
    r = run_flight(geometry, MAX_WET_MASS_KG,
                   climb_angle_deg=c["climb_angle_deg"],
                   motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02,
                   max_time_s=900.0, wing_concept=concept,
                   max_fuel_burn_kg=tank / (1.0 + FUEL_RESERVE_MARGIN),
                   return_to_launch=True)
    print("FLIGHT_JSON:" + json.dumps({
        "motor_cutoff_reached": r.motor_cutoff_reached,
        "safe_landing": r.safe_landing,
        "stalled": r.stalled,
        "hit_mass_floor": r.hit_mass_floor,
        "min_powered_accel_g": r.min_powered_accel_g,
        "min_accel_mach": r.min_accel_mach,
        "min_traverse_accel_g": r.min_traverse_accel_g,
        "min_powered_thrust_margin": r.min_powered_thrust_margin,
        "peak_thrust_to_weight": max(s.thrust_to_weight for s in r.states),
        "final_distance_m": r.states[-1].distance_m,
        "max_distance_m": max(s.distance_m for s in r.states),
    }))


if __name__ == "__main__":
    main()
