"""V4 baseline definition + a single sanity flight.

The V4 airframe is NOT simple_model's own frozen V3 -- it is the medium
model's V3b optimum (docs/v3b_medium_model/design.json) carried back into
simple_model, with the user's 2026-08-13 change: the chamber diameter
BECOMES the body diameter (225.2 -> 214.0 mm, the 5.6 mm annular gap that
never fit the valve runners anyway is deleted).

Two consequences that are forced, not chosen:

  1. simple_model has ONE diameter -- pulsejet_simple builds chamber volume
     from diameter_m -- so the chamber shrinks 9.7 % along with the frontal
     area. This model therefore does NOT claim the "free drag reduction"
     a split body/chamber would give. If medium_model keeps a true 214 mm
     chamber inside a 214 mm skin it will measure MORE thrust than this,
     which is the safe direction to be wrong in.
  2. V3b's 121.6 mm throat against a 214 mm body is an area fraction of
     0.323, over PULSEJET_MAX_THROAT_AREA_FRACTION = 0.30 -- the gate that
     zeroes the engine outright. The throat is held at V3b's OWN area
     fraction (0.2916) instead, which forces 121.6 -> 115.6 mm.

Mass is re-derived from this geometry, never inherited: the V3a/V3b/V3
design.json files all carry a dry_mass_kg that is stale by +2.93 kg
(docs/v3_learnings_for_v4.md section 5.2 #9), and loaded fuel is derived as
wet - dry - margin, so inheriting it silently corrupts the fuel budget.

Usage:  SIMPLE_MODEL_CD0_FRONTAL=0.1 .venv/Scripts/python scripts/simple_model_v4_baseline.py
"""
from __future__ import annotations

import os
import sys
from math import sqrt
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("SIMPLE_MODEL_CD0_FRONTAL", "0.1")

from simple_model.constants import (  # noqa: E402
    AIRFOILS, FUELS, PULSEJET_MAX_THROAT_AREA_FRACTION)
from simple_model.drag import WingConcept  # noqa: E402
from simple_model.flight_sim import (  # noqa: E402
    ClimbDiveProfile, RamjetStart, V4_PULLOUT_LOAD_FACTOR,
    V4_PUSHOVER_LOAD_FACTOR, VehicleGeometry, run_flight)
from simple_model.mass_model import vehicle_dry_mass  # noqa: E402
from simple_model.optimize import (  # noqa: E402
    FUEL_RESERVE_MARGIN, FUEL_VOLUME_FRACTION_OF_ANNULUS, MAX_WET_MASS_KG,
    MOTOR_CUTOFF_MACH, _annular_volume_m3)
from simple_model.pulsejet_simple import pulsejet_thrust  # noqa: E402

# --- V4 baseline airframe (medium-model V3b, chamber diameter -> body) -----
V4_BODY_DIAMETER_M = 0.214
V3B_THROAT_AREA_FRACTION = (0.12161734042955713 / 0.22521729709177246) ** 2
V4_THROAT_DIAMETER_M = V4_BODY_DIAMETER_M * sqrt(V3B_THROAT_AREA_FRACTION)
V4_CHAMBER_LENGTH_M = 0.38930734448088206
# V3b's duct: cone 215.3 mm + tailpipe 1027.0 mm. simple_model's own V3
# carried 689 mm, which V3a had to extend to sustain the pulsejet at all --
# use the extended value, it is what the vehicle physically is.
V4_THROAT_LENGTH_M = 1.2423
V4_LEGACY_WINGSPAN_M = 0.8033145899324761   # unused once a WingConcept is passed
V4_DRAG_STRIP_ANGLE_DEG = 1.0
V4_FUEL_KEY = "propane"

V4_WING = WingConcept(0.532526287202532, 1.8595845502152375,
                      0.5120481632935137, 13.4220600837535,
                      AIRFOILS["thin_cambered"])

# The FP pulsejet flames out at ~1250 m alive / 1275 m dead at CONFIRM tier,
# and the ceiling DROPPED ~200 m when the grid was refined
# (docs/v3_learnings_for_v4.md section 2.3). simple_model has no flame-out
# model at all -- it only lapses thrust as (rho/rho_SL)^3 -- so a derived
# top-of-climb above this is a number simple_model will happily produce and
# the real engine will not fly. Tracked as a reported constraint.
FP_PULSEJET_CEILING_M = 1200.0


def v4_geometry(diameter_m: float = V4_BODY_DIAMETER_M,
                throat_diameter_m: float = V4_THROAT_DIAMETER_M,
                chamber_length_m: float = V4_CHAMBER_LENGTH_M,
                throat_length_m: float = V4_THROAT_LENGTH_M) -> VehicleGeometry:
    return VehicleGeometry(diameter_m, throat_diameter_m, chamber_length_m,
                           throat_length_m, V4_LEGACY_WINGSPAN_M,
                           FUELS[V4_FUEL_KEY])


def tank_capacity_kg(geometry: VehicleGeometry) -> float:
    return (FUEL_VOLUME_FRACTION_OF_ANNULUS
            * _annular_volume_m3(geometry.diameter_m, geometry.throat_diameter_m,
                                 geometry.throat_length_m)
            * geometry.fuel.density_kg_per_m3)


def fly(climb_deg: float, dive_deg: float, floor_m: float, gate_mach: float,
        pullout_n: float = V4_PULLOUT_LOAD_FACTOR,
        pushover_n: float = V4_PUSHOVER_LOAD_FACTOR,
        geometry: VehicleGeometry | None = None,
        wing: WingConcept | None = None,
        dive_end_mach: float = 0.60,
        top_altitude_m: float | None = None,
        spiral_climb: bool = True,
        dt_s: float = 0.02, max_time_s: float = 600.0):
    # top_altitude_m: V3 DERIVED the top from the dive Mach band, which makes
    # the top an implicit function of dive_end_mach -- fine when the dive
    # really does terminate on Mach, misleading here because V4's dive always
    # terminates on ALTITUDE (the vehicle never reaches dive_end_mach; see
    # the reachability screen). V4 therefore lets the top be commanded
    # directly, which is also what "climb higher to dive longer" needs to be
    # a searchable variable rather than a side effect.
    geometry = geometry or v4_geometry()
    wing = wing or V4_WING
    profile = ClimbDiveProfile(
        initial_climb_angle_deg=climb_deg, dive_angle_deg=dive_deg,
        floor_altitude_m=floor_m, dive_end_mach=dive_end_mach,
        top_altitude_m=top_altitude_m,
        pullout_load_factor=pullout_n, pushover_load_factor=pushover_n,
        spiral_climb=spiral_climb,
    )
    return run_flight(
        geometry, MAX_WET_MASS_KG, climb_angle_deg=V4_DRAG_STRIP_ANGLE_DEG,
        motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=dt_s, max_time_s=max_time_s,
        wing_concept=wing,
        max_fuel_burn_kg=tank_capacity_kg(geometry) / (1.0 + FUEL_RESERVE_MARGIN),
        return_to_launch=True, climb_dive=profile,
        ramjet_start=RamjetStart(gate_mach=gate_mach, require_descending=True),
    )


def mass_block(result, geometry: VehicleGeometry, wing: WingConcept) -> dict:
    """Re-derived from THIS geometry -- never inherited (learnings 5.2 #9)."""
    fuel_burned = result.states[-1].fuel_burned_kg
    fuel_loaded = (1.0 + FUEL_RESERVE_MARGIN) * fuel_burned
    mass = vehicle_dry_mass(geometry.diameter_m, geometry.chamber_length_m,
                            geometry.throat_diameter_m, geometry.throat_length_m,
                            wing.reference_area_m2, fuel_loaded)
    return {
        "fuel_burned_kg": fuel_burned,
        "fuel_loaded_kg": fuel_loaded,
        "tank_capacity_kg": tank_capacity_kg(geometry),
        "dry_mass_kg": mass.dry_mass_kg,
        "payload_margin_kg": MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded,
    }


def main() -> None:
    geometry = v4_geometry()
    frac = (geometry.throat_diameter_m / geometry.diameter_m) ** 2
    print("--- V4 baseline airframe ---")
    print(f"body/chamber diameter  {geometry.diameter_m*1000:.1f} mm "
          f"(V3b body 225.2, chamber 214.0)")
    print(f"throat diameter        {geometry.throat_diameter_m*1000:.1f} mm "
          f"(V3b 121.6 -- forced down by the area-fraction cap)")
    print(f"throat area fraction   {frac:.4f}  (cap {PULSEJET_MAX_THROAT_AREA_FRACTION})")
    print(f"chamber length         {geometry.chamber_length_m*1000:.1f} mm")
    print(f"throat/duct length     {geometry.throat_length_m*1000:.1f} mm")
    print(f"tail/chamber-dia       {(geometry.throat_length_m - 0.2153)/geometry.diameter_m:.2f} "
          f"(FP sustain rule >= ~4.5, measured at M 0.15 only)")

    pj = pulsejet_thrust(geometry.diameter_m, geometry.chamber_length_m,
                         geometry.throat_diameter_m, geometry.throat_length_m,
                         0.15, 60.0, geometry.fuel)
    print(f"pulsejet @ M0.15/60m   {pj.average_thrust_n:.1f} N, "
          f"{pj.frequency_hz:.1f} Hz, operable={pj.operable}")

    print("\n--- sanity flight: V3b trajectory + arcs + gate M 0.50 ---")
    r = fly(climb_deg=10.0, dive_deg=9.890058542648028, floor_m=133.1432273621529,
            gate_mach=0.50)
    m = mass_block(r, geometry, V4_WING)
    print(f"top of climb           {r.climb_dive_top_altitude_m:.1f} m")
    print(f"dive exit              M {r.dive_exit_mach:.3f} at "
          f"{r.pullout_entry_altitude_m:.1f} m")
    print(f"ramjet lightoff        {r.ramjet_lightoff_mach} "
          f"mode={r.ramjet_lightoff_mode!r} in_dive={r.ramjet_lit_in_dive}")
    print(f"cutoff reached         {r.motor_cutoff_reached}")
    print(f"min altitude (powered) {r.min_powered_altitude_m:.1f} m "
          f"(floor 133.1, violated={r.floor_violated})")
    print(f"pushover radius        {r.pushover_radius_m:.0f} m over {r.pushover_duration_s:.2f} s")
    print(f"pullout radius         {r.pullout_radius_m:.0f} m over {r.pullout_duration_s:.2f} s")
    print(f"peak body load         {r.peak_load_n_total:.2f} g total "
          f"(yaw {r.peak_load_n_yaw:.2f}, roll {r.peak_load_n_roll:.2f}) in {r.peak_load_mode}")
    print(f"traverse accel         {r.min_traverse_accel_g:.3f} g @ M{r.min_traverse_accel_mach:.2f}")
    print(f"thrust margin          {r.min_powered_thrust_margin:.3f}")
    print(f"peak T/W               {max(s.thrust_to_weight for s in r.states):.2f}")
    print(f"fuel burned / tank     {m['fuel_burned_kg']:.3f} / {m['tank_capacity_kg']:.3f} kg")
    print(f"dry mass               {m['dry_mass_kg']:.3f} kg")
    print(f"payload margin         {m['payload_margin_kg']:.3f} kg")


if __name__ == "__main__":
    main()
