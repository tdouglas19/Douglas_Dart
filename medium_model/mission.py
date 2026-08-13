"""Mission-level constants and tank sizing.

Lifted from ``simple_model/optimize.py`` (the only pieces of the SEARCH
module a single flight actually needs) so medium_model can re-fly a design
without importing an optimizer it deliberately does not have: this model
refines one narrowed-down design, it does not search.

Values are the frozen-V2 values and must not drift silently --
``tests/test_medium_model_v2_parity.py`` asserts them against
``docs/v2_frozen/design.json``.
"""
from __future__ import annotations

from math import pi

KG_PER_LB = 0.45359237

MAX_WET_MASS_KG = 50.0 * KG_PER_LB
"""50 lb wet-mass cap (the competition/vehicle requirement)."""

FUEL_RESERVE_MARGIN = 0.25
"""25% of usable fuel held in reserve, i.e. burnable = capacity/1.25."""

MOTOR_CUTOFF_MACH = 1.1
"""Powered flight ends here; everything after is unpowered."""

FUEL_VOLUME_FRACTION_OF_ANNULUS = 0.5
"""Half the annulus between the duct and the outer body is usable tank."""


def annular_volume_m3(diameter_m: float, throat_diameter_m: float,
                      throat_length_m: float) -> float:
    """Volume of the annulus between the outer body and the exhaust duct."""
    annular_area_m2 = (pi / 4.0) * (diameter_m ** 2 - throat_diameter_m ** 2)
    return annular_area_m2 * throat_length_m


def usable_fuel_kg(diameter_m: float, throat_diameter_m: float,
                   throat_length_m: float, fuel_density_kg_per_m3: float
                   ) -> float:
    """Burnable fuel: tank capacity less the reserve margin."""
    capacity = (FUEL_VOLUME_FRACTION_OF_ANNULUS
                * annular_volume_m3(diameter_m, throat_diameter_m,
                                    throat_length_m)
                * fuel_density_kg_per_m3)
    return capacity / (1.0 + FUEL_RESERVE_MARGIN)
