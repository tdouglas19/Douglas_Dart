"""Parametric mass model tied to actual geometry (docs/design_workflow.md Level 3).

`optimizer.py`'s own prior docstring admitted the gap this module closes:
"Body diameter growth's structural-mass penalty is not modeled... any
candidate the search prefers with a substantially different body diameter
should be re-checked against a real mass budget." Body length was not a
search variable at all for the same reason.

This module does NOT run a structural analysis. It scales the
geometry-linked components already named in `configs/robustness_candidate_b.yaml`
(body skin, lifting surfaces, propulsion hardware, fuel tankage) using areal-
or mass-density coefficients CALIBRATED to reproduce today's configured
baseline mass at today's configured geometry. That calibration is a
consistency anchor, not independent evidence the scaling law itself is
correct -- see `docs/assumptions_registry.md`. Every coefficient is provisional
until real structural data replaces it.

Components NOT scaled here (avionics, actuators, thermal protection, landing/
recovery, wiring/fasteners, engineering growth allowance) are carried as a
fixed lump sum, since nothing in this codebase currently models how they'd
change with vehicle geometry -- scaling them would be inventing a law with no
basis at all, worse than leaving them fixed and visible.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from .config import ReferenceCase

_FIXED_COMPONENT_KEYS = (
    "fuel_system_and_ignition",
    "avionics_and_instrumentation",
    "actuators",
    "thermal_protection",
    "landing_and_recovery",
    "wiring_fasteners_and_miscellaneous",
    "explicit_current_growth_allowance",
)
_BODY_SKIN_COMPONENT_KEY = "primary_structure_and_skin"
_LIFTING_SURFACE_COMPONENT_KEY = "wings_fins_and_control_surfaces"
_PROPULSION_HARDWARE_COMPONENT_KEY = "shared_engine_body_combustor_nozzle"
_SELECTOR_COMPONENT_KEY = "selector_and_inlet_structure"


@dataclass(frozen=True)
class MassModelCalibration:
    """Density-style coefficients back-solved from one baseline mass budget.

    Every field name states exactly what physical quantity it multiplies, so
    a reader does not have to trust a docstring to know what "the number"
    means.
    """

    body_skin_areal_density_kg_per_m2: float
    lifting_surface_areal_density_kg_per_m2: float
    propulsion_hardware_mass_per_throat_area_kg_per_m2: float
    selector_mass_per_intake_area_kg_per_m2: float
    fixed_systems_mass_kg: float
    calibration_source_path: str

    def __post_init__(self) -> None:
        for name in (
            "body_skin_areal_density_kg_per_m2",
            "lifting_surface_areal_density_kg_per_m2",
            "propulsion_hardware_mass_per_throat_area_kg_per_m2",
            "selector_mass_per_intake_area_kg_per_m2",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.fixed_systems_mass_kg < 0.0:
            raise ValueError("fixed systems mass cannot be negative")


def _body_wetted_area_m2(diameter_m: float, length_m: float) -> float:
    """Cylinder side area, pi*D*L -- a low-order wetted-area proxy.

    Ignores nose/tail taper (see openvsp_geometry.py's real station-based
    mold line for the actual shape); adequate for a mass-scaling law, not for
    an aerodynamic wetted-area calculation.
    """

    from math import pi

    return pi * diameter_m * length_m


def _lifting_and_fin_area_m2(case: ReferenceCase) -> float:
    geometry = case.geometry
    return (
        geometry.lifting_surface_count * geometry.lifting_surface.exposed_area_per_surface_m2
        + geometry.fin_count * geometry.fin.exposed_area_per_surface_m2
    )


def _throat_area_m2(throat_diameter_m: float) -> float:
    from math import pi

    return pi * (throat_diameter_m / 2.0) ** 2


def _intake_area_m2(intake_diameter_m: float) -> float:
    from math import pi

    return pi * (intake_diameter_m / 2.0) ** 2


def calibrate_mass_model(
    case: ReferenceCase, mass_budget_path: str | Path
) -> MassModelCalibration:
    """Back-solve density coefficients from one named mass-budget YAML.

    Solves ``coefficient = configured_component_mass_kg / configured_geometry_quantity``
    for each geometry-linked component, at the case's own configured geometry.
    This is calibration, not derivation -- see the module docstring.
    """

    mass_budget_path = Path(mass_budget_path)
    with mass_budget_path.open("r", encoding="utf-8") as stream:
        data: Mapping[str, Any] = yaml.safe_load(stream)
    components = data["mass_budget"]["components"]

    body_area_m2 = _body_wetted_area_m2(case.vehicle.body_diameter_m, case.vehicle.body_length_m)
    lifting_area_m2 = _lifting_and_fin_area_m2(case)
    throat_area_m2 = _throat_area_m2(case.nozzle.throat_diameter_m)
    intake_area_m2 = _intake_area_m2(case.selector.circular_intake_diameter_m)

    fixed_mass_kg = sum(
        float(components[key]["current_mass_kg"]) for key in _FIXED_COMPONENT_KEYS
    )

    return MassModelCalibration(
        body_skin_areal_density_kg_per_m2=(
            float(components[_BODY_SKIN_COMPONENT_KEY]["current_mass_kg"]) / body_area_m2
        ),
        lifting_surface_areal_density_kg_per_m2=(
            float(components[_LIFTING_SURFACE_COMPONENT_KEY]["current_mass_kg"]) / lifting_area_m2
        ),
        propulsion_hardware_mass_per_throat_area_kg_per_m2=(
            float(components[_PROPULSION_HARDWARE_COMPONENT_KEY]["current_mass_kg"]) / throat_area_m2
        ),
        selector_mass_per_intake_area_kg_per_m2=(
            float(components[_SELECTOR_COMPONENT_KEY]["current_mass_kg"]) / intake_area_m2
        ),
        fixed_systems_mass_kg=fixed_mass_kg,
        calibration_source_path=str(mass_budget_path),
    )


@dataclass(frozen=True)
class ParametricMassBreakdown:
    body_skin_mass_kg: float
    lifting_surface_mass_kg: float
    propulsion_hardware_mass_kg: float
    selector_mass_kg: float
    fixed_systems_mass_kg: float
    empty_mass_kg: float
    """Everything except loaded fuel -- matches optimizer.py's prior
    baseline_empty_mass_kg concept, now a function of geometry instead of a
    frozen constant."""
    body_wetted_area_m2: float
    lifting_surface_area_m2: float
    throat_area_m2: float
    intake_area_m2: float


def evaluate_parametric_mass(
    case: ReferenceCase,
    calibration: MassModelCalibration,
    *,
    body_diameter_m: float | None = None,
    body_length_m: float | None = None,
    throat_diameter_m: float | None = None,
) -> ParametricMassBreakdown:
    """Empty mass as a function of geometry, using calibrated density laws.

    Lifting-surface and selector-intake geometry are read from ``case`` as
    configured -- this module does not yet expose them as independent search
    variables (see docs/design_workflow.md Level 3's "lifting area and
    preliminary stabilizer sizing" item, still queued).
    """

    body_diameter_m = case.vehicle.body_diameter_m if body_diameter_m is None else body_diameter_m
    body_length_m = case.vehicle.body_length_m if body_length_m is None else body_length_m
    throat_diameter_m = (
        case.nozzle.throat_diameter_m if throat_diameter_m is None else throat_diameter_m
    )

    body_area_m2 = _body_wetted_area_m2(body_diameter_m, body_length_m)
    lifting_area_m2 = _lifting_and_fin_area_m2(case)
    throat_area_m2 = _throat_area_m2(throat_diameter_m)
    intake_area_m2 = _intake_area_m2(case.selector.circular_intake_diameter_m)

    body_skin_mass_kg = calibration.body_skin_areal_density_kg_per_m2 * body_area_m2
    lifting_surface_mass_kg = (
        calibration.lifting_surface_areal_density_kg_per_m2 * lifting_area_m2
    )
    propulsion_hardware_mass_kg = (
        calibration.propulsion_hardware_mass_per_throat_area_kg_per_m2 * throat_area_m2
    )
    selector_mass_kg = calibration.selector_mass_per_intake_area_kg_per_m2 * intake_area_m2

    empty_mass_kg = (
        body_skin_mass_kg
        + lifting_surface_mass_kg
        + propulsion_hardware_mass_kg
        + selector_mass_kg
        + calibration.fixed_systems_mass_kg
    )

    return ParametricMassBreakdown(
        body_skin_mass_kg=body_skin_mass_kg,
        lifting_surface_mass_kg=lifting_surface_mass_kg,
        propulsion_hardware_mass_kg=propulsion_hardware_mass_kg,
        selector_mass_kg=selector_mass_kg,
        fixed_systems_mass_kg=calibration.fixed_systems_mass_kg,
        empty_mass_kg=empty_mass_kg,
        body_wetted_area_m2=body_area_m2,
        lifting_surface_area_m2=lifting_area_m2,
        throat_area_m2=throat_area_m2,
        intake_area_m2=intake_area_m2,
    )
