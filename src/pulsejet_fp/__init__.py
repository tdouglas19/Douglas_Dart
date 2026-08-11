"""pulsejet_fp: first-principles valved pulsejet transient cycle model.

Public API: pulsejet_thrust(), reference_* design builders, ThrustResult.
Derivation: docs/derivation.md.
"""
from .gas import GasModel, propane_air
from .geometry import EngineGeometry
from .valve import PetalValveDesign
from .engine import Numerics, PulsejetEngine, StartCondition, TurbulenceParams
from .query import (ThrustResult, pulsejet_thrust, reference_gas,
                    reference_geometry, reference_valve)

__all__ = [
    "GasModel", "propane_air", "EngineGeometry", "PetalValveDesign",
    "Numerics", "PulsejetEngine", "StartCondition", "TurbulenceParams",
    "ThrustResult", "pulsejet_thrust", "reference_gas",
    "reference_geometry", "reference_valve",
]
