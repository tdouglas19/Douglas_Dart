"""Load a frozen design (docs/v2_frozen, docs/v3_frozen) into the pieces
a medium_model flight needs.

simple_model rebuilds these from ``optimize.Candidate``, but medium_model
deliberately does not copy the optimizer -- it refines one narrowed
design, it does not search. This module is the equivalent entry point:
design.json in, flyable objects out.

Mirrors ``Candidate.to_climb_dive`` / ``.to_geometry`` exactly, including
the floor clamp (``max(floor_altitude_m, V3_FLOOR_ALTITUDE_M)``).
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .constants import AIRFOILS, FUELS
from .drag import WingConcept
# V3_FLOOR_ALTITUDE_M lives in flight_sim, not constants (mirrors the
# ancestor's layout -- it is a trajectory constant, not a vehicle one)
from .flight_sim import (V3_FLOOR_ALTITUDE_M, ClimbDiveProfile, RamjetStart,
                         VehicleGeometry)
from .mission import MAX_WET_MASS_KG, burn_limit_kg, loaded_fuel_kg


@dataclass(frozen=True)
class FrozenDesign:
    name: str
    geometry: VehicleGeometry
    wing: WingConcept
    climb_dive: ClimbDiveProfile | None
    climb_angle_deg: float
    cd0_frontal: float
    burn_limit_kg: float
    loaded_fuel_kg: float
    verified: dict
    raw: dict
    ramjet_start: RamjetStart | None = None
    """V4 only: the lightoff POLICY (gate Mach + must-be-descending).
    None for V2/V3, which used the bare module constant."""

    @property
    def uses_climb_dive(self) -> bool:
        return self.climb_dive is not None

    @property
    def uses_arcs(self) -> bool:
        return self.climb_dive is not None and self.climb_dive.uses_arcs


def load_frozen_design(path: str | Path) -> FrozenDesign:
    """Read a frozen design.json into flyable objects.

    NOTE on CD0: the value lives in ``constants_at_freeze`` but
    ``constants.CD0_FRONTAL`` is bound at IMPORT time, so setting the
    environment variable after importing does nothing. Callers that care
    (anything CD0-sensitive) must set MEDIUM_MODEL_CD0_FRONTAL before
    importing medium_model -- normally by running in a subprocess. This
    loader returns the frozen value so a caller can assert it matches.
    """
    data = json.loads(Path(path).read_text())
    if "vehicle_candidate" not in data and "vehicle" in data:
        # V4 froze a different shape. It is not a renamed V2/V3 file: the
        # trajectory moved out of the vehicle block into its own, because V4
        # commands parameters (top of climb, arc load factors, the ramjet
        # gate) that V2/V3 either derived or did not have. Handled in its own
        # loader rather than by aliasing keys, so neither schema has to
        # pretend to be the other.
        return _load_v4_design(path, data)
    c = data["vehicle_candidate"]
    w = data["wing_concept"]
    frozen = data.get("constants_at_freeze", {})

    geometry = VehicleGeometry(
        diameter_m=c["diameter_m"],
        throat_diameter_m=c["throat_diameter_m"],
        chamber_length_m=c["chamber_length_m"],
        throat_length_m=c["throat_length_m"],
        wingspan_m=c["wingspan_m"],
        fuel=FUELS[c["fuel_key"]],
    )
    wing = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                       w["sweep_deg"], AIRFOILS[w["airfoil_key"]])

    climb_dive = None
    if c.get("initial_climb_angle_deg") is not None:
        # mirrors Candidate.to_climb_dive, floor clamp included
        climb_dive = ClimbDiveProfile(
            initial_climb_angle_deg=c["initial_climb_angle_deg"],
            dive_angle_deg=c["dive_angle_deg"],
            floor_altitude_m=max(c["floor_altitude_m"], V3_FLOOR_ALTITUDE_M),
        )

    opt = data.get("optimizer_results", {})
    loaded = loaded_fuel_kg(opt.get("dry_mass_kg", 0.0),
                           opt.get("mass_margin_kg", 0.0))
    return FrozenDesign(
        name=data.get("name", str(path)),
        geometry=geometry, wing=wing, climb_dive=climb_dive,
        climb_angle_deg=c["climb_angle_deg"],
        cd0_frontal=frozen.get("CD0_FRONTAL", float("nan")),
        burn_limit_kg=burn_limit_kg(loaded), loaded_fuel_kg=loaded,
        verified=data.get("verified_mission", {}), raw=data,
    )


def _load_v4_design(path, data: dict) -> FrozenDesign:
    """Loader for the V4 freeze (docs/v4_frozen/design.json).

    Differences from V2/V3 that matter, all of them deliberate:

    * ``vehicle`` instead of ``vehicle_candidate``; the wingspan is
      ``legacy_wingspan_m`` because the WingConcept owns the real span and
      the legacy number only feeds the no-concept fallback path.
    * ``trajectory`` is its own block, and ``top_altitude_m`` is COMMANDED
      (1100 m), not derived -- so ``derive_top_altitude`` never runs on a V4
      re-fly, which also means the V4 arc-drop term in that function is not
      on the critical path for this design.
    * mass comes from ``verified_mission`` (dry + payload margin) rather
      than an ``optimizer_results`` block, because V4 was frozen from a
      single flight, not from a search.
    """
    c = data["vehicle"]
    w = data["wing_concept"]
    tr = data.get("trajectory", {})
    frozen = data.get("constants_at_freeze", {})

    geometry = VehicleGeometry(
        diameter_m=c["diameter_m"],
        throat_diameter_m=c["throat_diameter_m"],
        chamber_length_m=c["chamber_length_m"],
        throat_length_m=c["throat_length_m"],
        wingspan_m=c["legacy_wingspan_m"],
        fuel=FUELS[c["fuel_key"]],
    )
    wing = WingConcept(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                       w["sweep_deg"], AIRFOILS[w["airfoil_key"]])

    climb_dive = ClimbDiveProfile(
        initial_climb_angle_deg=tr["initial_climb_angle_deg"],
        dive_angle_deg=tr["dive_angle_deg"],
        floor_altitude_m=max(tr["floor_altitude_m"], V3_FLOOR_ALTITUDE_M),
        dive_end_mach=tr["dive_end_mach"],
        top_altitude_m=tr["top_altitude_m"],
        pullout_load_factor=tr["pullout_load_factor"],
        pushover_load_factor=tr["pushover_load_factor"],
        pullout_max_load_factor=tr["pullout_max_load_factor"],
        spiral_climb=tr["spiral_climb"],
        spiral_bank_deg=tr["spiral_bank_deg"],
    )
    ramjet_start = RamjetStart(
        gate_mach=tr["ramjet_gate_mach"],
        require_descending=tr["ramjet_require_descending"],
    )

    verified = data.get("verified_mission", {})
    loaded = loaded_fuel_kg(verified.get("dry_mass_kg", 0.0),
                            verified.get("payload_margin_kg", 0.0))
    return FrozenDesign(
        name=data.get("name", str(path)),
        geometry=geometry, wing=wing, climb_dive=climb_dive,
        climb_angle_deg=c["drag_strip_climb_angle_deg"],
        cd0_frontal=frozen.get("CD0_FRONTAL", float("nan")),
        burn_limit_kg=burn_limit_kg(loaded), loaded_fuel_kg=loaded,
        verified=verified, raw=data, ramjet_start=ramjet_start,
    )


def fly(design: FrozenDesign, **overrides):
    """Fly a frozen design with medium_model's integrator.

    Fuel: medium_model caps the burn at 90% of the design's LOADED
    allocation (mission.py) -- the vehicle cannot burn fuel it does not
    carry -- rather than at tank volume the way the search-oriented
    ancestor does.
    """
    from .flight_sim import run_flight
    from .mission import MOTOR_CUTOFF_MACH

    kwargs = dict(
        climb_angle_deg=design.climb_angle_deg,
        motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02, max_time_s=900.0,
        wing_concept=design.wing, max_fuel_burn_kg=design.burn_limit_kg,
        return_to_launch=True, climb_dive=design.climb_dive,
        ramjet_start=design.ramjet_start,
    )
    kwargs.update(overrides)
    return run_flight(design.geometry, MAX_WET_MASS_KG, **kwargs)
