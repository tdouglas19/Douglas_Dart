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

FUEL_BURN_FRACTION_OF_LOADED = 0.90
"""Engine shuts down once 90% of the LOADED fuel (inclusive of reserve)
has been consumed; the last 10% is trapped/unusable.

Why this rule exists (a real defect inherited from the ancestor):
``simple_model`` caps a flight's burn at the TANK VOLUME
(capacity/1.25 = 5.84 kg for V2) and then sizes the loaded fuel AFTER the
fact as 1.25x whatever was actually burned. That closes consistently only
when the design reaches motor cutoff quickly -- V2 burned 1.62 kg, so its
loaded fuel came out 2.03 kg and the volumetric cap never bound.

A design that does NOT reach cutoff burns straight through to the
volumetric cap: 5.84 kg on a vehicle carrying 2.03 kg. Worse, the
integrator sheds that phantom mass as it goes, so the vehicle gets
~3.8 kg lighter than it should and accelerates better than it can. The
engine effectively runs on fuel that does not exist.

medium_model therefore caps the burn against the design's ACTUAL loaded
allocation, so an engine can never outrun its own tank.

**This is a deliberate divergence from the ancestor, and the direction of
the arrow is the point.** In simple_model, loaded fuel is an OUTPUT --
fly first, then size the tank to 1.25x what was burned. That is the right
answer for a SEARCH, where the tank is still free to grow. medium_model
refines ONE frozen design, so its fuel load is already decided: V2's
2.011 kg (from the mass budget it closed on: wet - dry - margin) is taken
as the max allocation INPUT, and the engine shuts down at 90% of it.
A refinement model must not be allowed to quietly enlarge the very
vehicle it is supposed to be evaluating."""


def burn_limit_kg(loaded_fuel_kg: float) -> float:
    """Usable fuel before shutdown: 90% of what is actually loaded."""
    return FUEL_BURN_FRACTION_OF_LOADED * loaded_fuel_kg


def loaded_fuel_kg(dry_mass_kg: float, mass_margin_kg: float,
                   wet_mass_kg: float = MAX_WET_MASS_KG) -> float:
    """The design's fuel allocation, from the mass budget it closed on:
    wet = dry + fuel + margin."""
    return max(wet_mass_kg - dry_mass_kg - mass_margin_kg, 0.0)


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
