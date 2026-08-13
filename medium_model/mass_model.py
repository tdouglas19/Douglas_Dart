"""Closed-form parametric mass model: structure + auxiliaries, so the
optimizer cannot buy thrust with diameter for free. Every term is a single
algebraic expression (areas x gauge x density, or fixed allowances).

Constraint consumed by both optimizers: dry_mass + fuel_loaded <= 50 lb wet
budget. The flight sim still LAUNCHES at the full 50 lb (the user's fixed
wet-mass convention); slack between (dry+fuel) and 50 lb is reported as
payload/ballast margin, negative margin = infeasible design.
"""
from __future__ import annotations

from math import pi
from typing import NamedTuple

from .constants import (
    AVIONICS_FIXED_MASS_KG, CFRP_DENSITY_KG_M3, CFRP_MIN_GAUGE_M,
    LANDING_HARDWARE_KG, NOSE_TAIL_LENGTH_DIAMETERS,
    PULSEJET_PEAK_PRESSURE_RATIO, STEEL_ALLOWABLE_STRESS_PA,
    STEEL_DENSITY_KG_M3, STEEL_MIN_GAUGE_M, STRUCTURAL_OVERHEAD_FRACTION,
    TANK_HARDWARE_FIXED_KG, TANK_HARDWARE_FUEL_FRACTION,
    WING_AREAL_MASS_KG_M2,
)

_SEA_LEVEL_PA = 101_325.0


class MassBreakdown(NamedTuple):
    engine_duct_kg: float
    airframe_skin_kg: float
    wing_kg: float
    avionics_kg: float
    tank_hardware_kg: float
    landing_hardware_kg: float
    dry_mass_kg: float
    duct_wall_thickness_m: float


def duct_wall_thickness_m(diameter_m: float) -> float:
    """Hoop-stress requirement vs manufacturable minimum gauge; the design
    pressure is the pulsejet peak gauge pressure at sea level."""
    p_gauge = (PULSEJET_PEAK_PRESSURE_RATIO - 1.0) * _SEA_LEVEL_PA
    t_pressure = p_gauge * (diameter_m / 2.0) / STEEL_ALLOWABLE_STRESS_PA
    return max(t_pressure, STEEL_MIN_GAUGE_M)


def vehicle_dry_mass(
    diameter_m: float,
    chamber_length_m: float,
    throat_diameter_m: float,
    throat_length_m: float,
    wing_area_m2: float,
    fuel_loaded_kg: float,
) -> MassBreakdown:
    t_duct = duct_wall_thickness_m(diameter_m)
    duct_area = pi * (diameter_m * chamber_length_m
                      + throat_diameter_m * throat_length_m)
    engine_duct = STEEL_DENSITY_KG_M3 * duct_area * t_duct

    body_length = (chamber_length_m + throat_length_m
                   + NOSE_TAIL_LENGTH_DIAMETERS * diameter_m)
    skin_area = pi * diameter_m * body_length
    airframe = (CFRP_DENSITY_KG_M3 * skin_area * CFRP_MIN_GAUGE_M
                * (1.0 + STRUCTURAL_OVERHEAD_FRACTION))

    wing = WING_AREAL_MASS_KG_M2 * wing_area_m2
    tank = TANK_HARDWARE_FIXED_KG + TANK_HARDWARE_FUEL_FRACTION * fuel_loaded_kg
    dry = (engine_duct + airframe + wing + AVIONICS_FIXED_MASS_KG
           + tank + LANDING_HARDWARE_KG)
    return MassBreakdown(engine_duct, airframe, wing, AVIONICS_FIXED_MASS_KG,
                         tank, LANDING_HARDWARE_KG, dry, t_duct)
