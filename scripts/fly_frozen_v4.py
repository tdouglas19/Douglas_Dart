"""Re-fly the frozen V4 design and print its mission numbers as JSON.

Standalone for the same reason as scripts/fly_frozen_v2.py and
scripts/fly_frozen_v3.py: CD0 is baked into simple_model.constants at IMPORT
time and re-exported by drag/flight_sim, so a test cannot reliably override it
in-process once any other test module has imported simple_model. Running the
flight in its own process makes the frozen check independent of test module
import order.

Usage: python scripts/fly_frozen_v4.py [path/to/design.json]
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DESIGN_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT / "docs" / "v4_frozen" / "design.json")
DESIGN = json.loads(DESIGN_PATH.read_text())
os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(
    DESIGN["constants_at_freeze"]["CD0_FRONTAL"])

from simple_model.constants import AIRFOILS, FUELS  # noqa: E402
from simple_model.drag import WingConcept  # noqa: E402
from simple_model.flight_sim import (  # noqa: E402
    ClimbDiveProfile, RamjetStart, VehicleGeometry, run_flight)
from simple_model.mass_model import vehicle_dry_mass  # noqa: E402
from simple_model.optimize import (  # noqa: E402
    FUEL_RESERVE_MARGIN, FUEL_VOLUME_FRACTION_OF_ANNULUS, MAX_WET_MASS_KG,
    MOTOR_CUTOFF_MACH, _annular_volume_m3)


def main() -> None:
    v, w, tj = DESIGN["vehicle"], DESIGN["wing_concept"], DESIGN["trajectory"]
    geometry = VehicleGeometry(
        v["diameter_m"], v["throat_diameter_m"], v["chamber_length_m"],
        v["throat_length_m"], v["legacy_wingspan_m"], FUELS[v["fuel_key"]])
    concept = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                          w["sweep_deg"], AIRFOILS[w["airfoil_key"]])
    tank = (FUEL_VOLUME_FRACTION_OF_ANNULUS
            * _annular_volume_m3(geometry.diameter_m, geometry.throat_diameter_m,
                                 geometry.throat_length_m)
            * FUELS[v["fuel_key"]].density_kg_per_m3)
    profile = ClimbDiveProfile(
        initial_climb_angle_deg=tj["initial_climb_angle_deg"],
        dive_angle_deg=tj["dive_angle_deg"],
        floor_altitude_m=tj["floor_altitude_m"],
        dive_end_mach=tj["dive_end_mach"],
        top_altitude_m=tj["top_altitude_m"],
        pullout_load_factor=tj["pullout_load_factor"],
        pushover_load_factor=tj["pushover_load_factor"],
        pullout_max_load_factor=tj["pullout_max_load_factor"],
        spiral_climb=tj["spiral_climb"],
        spiral_bank_deg=tj["spiral_bank_deg"],
    )
    r = run_flight(
        geometry, MAX_WET_MASS_KG,
        climb_angle_deg=v["drag_strip_climb_angle_deg"],
        motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02, max_time_s=600.0,
        wing_concept=concept, max_fuel_burn_kg=tank / (1.0 + FUEL_RESERVE_MARGIN),
        return_to_launch=True, climb_dive=profile,
        ramjet_start=RamjetStart(gate_mach=tj["ramjet_gate_mach"],
                                 require_descending=tj["ramjet_require_descending"]),
    )
    fuel_burned = r.states[-1].fuel_burned_kg
    fuel_loaded = (1.0 + FUEL_RESERVE_MARGIN) * fuel_burned
    mass = vehicle_dry_mass(geometry.diameter_m, geometry.chamber_length_m,
                            geometry.throat_diameter_m, geometry.throat_length_m,
                            concept.reference_area_m2, fuel_loaded)
    modes = {s.mode for s in r.states}
    print("FLIGHT_JSON:" + json.dumps({
        "ramjet_lightoff_mach": r.ramjet_lightoff_mach,
        "ramjet_lightoff_altitude_m": r.ramjet_lightoff_altitude_m,
        "ramjet_lightoff_mode": r.ramjet_lightoff_mode,
        "ramjet_lit_in_dive": r.ramjet_lit_in_dive,
        "dive_exit_mach": r.dive_exit_mach,
        "peak_mach": max(s.mach for s in r.states),
        "motor_cutoff_reached": r.motor_cutoff_reached,
        "safe_landing": r.safe_landing,
        "stalled": r.stalled,
        "hit_mass_floor": r.hit_mass_floor,
        "rule_violated": r.rule_violated,
        "floor_violated": r.floor_violated,
        "min_powered_altitude_m": r.min_powered_altitude_m,
        "peak_thrust_to_weight": max(s.thrust_to_weight for s in r.states),
        "min_traverse_accel_g": r.min_traverse_accel_g,
        "min_powered_accel_g": r.min_powered_accel_g,
        "min_powered_thrust_margin": r.min_powered_thrust_margin,
        "peak_load_n_total": r.peak_load_n_total,
        "peak_load_n_yaw": r.peak_load_n_yaw,
        "peak_load_n_roll": r.peak_load_n_roll,
        "peak_load_mode": r.peak_load_mode,
        "pushover_radius_m": r.pushover_radius_m,
        "pullout_radius_m": r.pullout_radius_m,
        "spiral_radius_m": r.spiral_radius_m,
        "pushover_duration_s": r.pushover_duration_s,
        "pullout_duration_s": r.pullout_duration_s,
        "fuel_burned_kg": fuel_burned,
        "tank_capacity_kg": tank,
        "dry_mass_kg": mass.dry_mass_kg,
        "payload_margin_kg": MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded,
        "stall_speed_m_per_s": r.states[-1].stall_speed_m_per_s,
        "modes_flown": sorted(modes),
        "lands_from_launch_m": abs(r.states[-1].distance_m),
    }))


if __name__ == "__main__":
    main()
