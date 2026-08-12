"""Mach-indexed total-drag-area model.

The prior flight kernel used one constant zero-lift drag coefficient regardless of
Mach number, while the sizing/robustness modules used a diameter-scaled drag-AREA
budget evaluated only at the single peak-Mach design point
(:func:`douglas_dart.sizing.geometrically_scaled_drag_area_target_m2`). Those two
representations disagreed everywhere except exactly Mach 1.10, which is the
inconsistency flagged in ``docs/design_convergence.md`` (\"the flight kernel's simple
coefficient polar is not consistent enough to replace it at Mach 1.10\").

This module keeps the same diameter-scaled drag-AREA budget as the single physical
anchor (it is still not solver-backed) but distributes it across Mach number with an
explicit transonic-drag-rise shape, so every discipline reads one Mach-indexed total
drag model instead of two disagreeing ones. It is still a budget/proxy, not a
validated aerodynamic prediction; solver-backed VSPAERO tables are the closure path.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass

from .atmosphere import standard_atmosphere
from .config import ReferenceCase, FlightConfig
from .sizing import geometrically_scaled_drag_area_target_m2

# Fraction of the calibrated peak-Mach (1.10) zero-lift drag area, at each breakpoint
# Mach. Shape is a representative slender-body/fin transonic drag rise: roughly flat
# subsonic, rising sharply from Mach 0.8, peaking just past Mach 1.0, and relaxing
# supersonically. This shape is a literature-typical placeholder, not a measurement of
# this vehicle; it is the single documented assumption that should be replaced first
# by solver-backed or wind-tunnel drag-rise data.
_DRAG_RISE_BREAKPOINTS: tuple[tuple[float, float], ...] = (
    (0.00, 0.42),
    (0.40, 0.42),
    (0.60, 0.46),
    (0.75, 0.55),
    (0.85, 0.72),
    (0.90, 0.84),
    (0.95, 0.94),
    (1.00, 1.00),
    (1.05, 1.015),
    (1.10, 1.00),
    (1.20, 0.90),
    (1.35, 0.80),
    (1.50, 0.74),
)

_BREAKPOINT_MACH = tuple(point[0] for point in _DRAG_RISE_BREAKPOINTS)
_BREAKPOINT_RATIO = tuple(point[1] for point in _DRAG_RISE_BREAKPOINTS)
_CALIBRATION_MACH = 1.10
_CALIBRATION_RATIO = 1.00


@dataclass(frozen=True)
class DragBreakdown:
    mach: float
    altitude_m: float
    dynamic_pressure_pa: float
    zero_lift_drag_ratio_to_peak_mach: float
    zero_lift_drag_area_m2: float
    zero_lift_drag_n: float
    lift_coefficient: float
    induced_drag_n: float
    total_drag_n: float
    numerical_reference_only: bool = True


def drag_rise_ratio(mach: float) -> float:
    """Interpolate the transonic drag-rise shape, clamped outside the table."""

    if mach <= _BREAKPOINT_MACH[0]:
        return _BREAKPOINT_RATIO[0]
    if mach >= _BREAKPOINT_MACH[-1]:
        return _BREAKPOINT_RATIO[-1]
    index = bisect_left(_BREAKPOINT_MACH, mach)
    if _BREAKPOINT_MACH[index] == mach:
        return _BREAKPOINT_RATIO[index]
    lower_mach, lower_ratio = _BREAKPOINT_MACH[index - 1], _BREAKPOINT_RATIO[index - 1]
    upper_mach, upper_ratio = _BREAKPOINT_MACH[index], _BREAKPOINT_RATIO[index]
    fraction = (mach - lower_mach) / (upper_mach - lower_mach)
    return lower_ratio + fraction * (upper_ratio - lower_ratio)


def mach_indexed_zero_lift_drag_area_m2(
    case: ReferenceCase,
    mach: float,
    *,
    body_diameter_m: float | None = None,
) -> float:
    """Return the Mach-indexed zero-lift drag area, anchored at the peak-Mach budget.

    ``geometrically_scaled_drag_area_target_m2`` is evaluated once at the configured
    peak Mach (its only physically anchored point today) and scaled by the shape in
    :func:`drag_rise_ratio`. Both the anchor and the shape remain provisional.
    """

    diameter_m = (
        case.vehicle.body_diameter_m if body_diameter_m is None else body_diameter_m
    )
    peak_mach_drag_area_m2 = geometrically_scaled_drag_area_target_m2(case, diameter_m)
    anchor_drag_area_m2 = peak_mach_drag_area_m2 / _CALIBRATION_RATIO
    return anchor_drag_area_m2 * drag_rise_ratio(mach)


def evaluate_total_drag(
    case: ReferenceCase,
    flight: FlightConfig,
    altitude_m: float,
    mach: float,
    lift_coefficient: float,
    *,
    body_diameter_m: float | None = None,
    drag_multiplier: float = 1.0,
) -> DragBreakdown:
    """Return the Mach-indexed zero-lift drag plus induced drag at one flight state."""

    if mach < 0.0:
        raise ValueError("Mach cannot be negative")
    if drag_multiplier <= 0.0:
        raise ValueError("drag multiplier must be positive")
    atmosphere = standard_atmosphere(altitude_m)
    speed_m_per_s = mach * atmosphere.speed_of_sound_m_per_s
    dynamic_pressure_pa = 0.5 * atmosphere.density_kg_per_m3 * speed_m_per_s**2

    ratio = drag_rise_ratio(mach)
    # Reuses `ratio` above instead of calling mach_indexed_zero_lift_drag_area_m2
    # (which would call drag_rise_ratio(mach) a second time with the same
    # argument) -- same formula as that function, same result, called on
    # every trajectory time step.
    diameter_m = case.vehicle.body_diameter_m if body_diameter_m is None else body_diameter_m
    anchor_drag_area_m2 = geometrically_scaled_drag_area_target_m2(case, diameter_m) / _CALIBRATION_RATIO
    zero_lift_drag_area_m2 = anchor_drag_area_m2 * ratio * drag_multiplier
    zero_lift_drag_n = dynamic_pressure_pa * zero_lift_drag_area_m2
    induced_drag_n = (
        dynamic_pressure_pa
        * flight.reference_area_m2
        * flight.induced_drag_factor
        * lift_coefficient**2
        * drag_multiplier
    )
    return DragBreakdown(
        mach=mach,
        altitude_m=altitude_m,
        dynamic_pressure_pa=dynamic_pressure_pa,
        zero_lift_drag_ratio_to_peak_mach=ratio,
        zero_lift_drag_area_m2=zero_lift_drag_area_m2,
        zero_lift_drag_n=zero_lift_drag_n,
        lift_coefficient=lift_coefficient,
        induced_drag_n=induced_drag_n,
        total_drag_n=zero_lift_drag_n + induced_drag_n,
    )


def mach_sweep_drag_area_m2(
    case: ReferenceCase,
    mach_values: tuple[float, ...] | list[float],
    *,
    body_diameter_m: float | None = None,
) -> list[tuple[float, float]]:
    """Return ``[(mach, zero_lift_drag_area_m2), ...]`` for plotting/inspection."""

    if not mach_values:
        raise ValueError("mach_values cannot be empty")
    return [
        (
            mach,
            mach_indexed_zero_lift_drag_area_m2(
                case, mach, body_diameter_m=body_diameter_m
            ),
        )
        for mach in mach_values
    ]
