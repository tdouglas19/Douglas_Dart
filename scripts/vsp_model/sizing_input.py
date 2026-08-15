"""Load ``structural_dimensions.json`` into a typed record.

This is the ONE authoritative input from the trajectory/sizing chain. Everything
this module exposes traces to a field in that file; anything the file does not
contain lives in :mod:`geometry_inputs` instead, so the boundary between
"sized by the model" and "chosen by us" stays visible in the code.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

DEFAULT_SIZING_JSON = Path("out_medium_model/v4_export/structural_dimensions.json")


@dataclass(frozen=True)
class SizingInput:
    """The subset of the sizing export that the outer mold line depends on."""

    source_path: str
    git_sha: str

    # Body
    body_diameter_m: float
    body_length_m: float
    nose_fairing_length_m: float
    boattail_length_m: float
    boattail_start_x_m: float
    frontal_area_m2: float

    # Internal flowpath -- only the two diameters that pierce the outer mold line
    nozzle_exit_diameter_m: float
    chamber_start_x_m: float
    chamber_end_x_m: float

    # Wing planform (theoretical trapezoid, spanning the centreline)
    wing_span_m: float
    wing_reference_area_m2: float
    wing_root_chord_m: float
    wing_tip_chord_m: float
    wing_taper_ratio: float
    wing_mean_aerodynamic_chord_m: float
    wing_mac_y_m: float
    wing_le_sweep_deg: float
    wing_quarter_chord_sweep_deg: float
    wing_thickness_to_chord: float
    wing_airfoil_key: str

    raw: dict[str, Any] = field(repr=False, default_factory=dict)

    @property
    def inlet_lip_diameter_m(self) -> float:
        """Diameter of the open nose inlet.

        NOT a field of the sizing export. The flown drag build-up takes the lip
        area as ``pi * throat_diameter**2 / 4`` (``medium_model/flight_sim.py``,
        ``geom_cache["lip_area_m2"]``), and the throat/nozzle diameter is the
        same number, so the lip diameter is the nozzle exit diameter.
        """

        return self.nozzle_exit_diameter_m

    @classmethod
    def from_json(cls, path: str | Path = DEFAULT_SIZING_JSON) -> SizingInput:
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"sizing JSON not found: {path}")
        data = json.loads(path.read_text(encoding="utf-8"))

        body = data["body"]
        stations = data["stations_from_nose_tip_m"]
        flow = data["internal_flowpath"]
        wing = data["wing"]

        record = cls(
            source_path=str(path),
            git_sha=str(data.get("provenance", {}).get("git_sha", "unknown")),
            body_diameter_m=float(body["diameter_m"]),
            body_length_m=float(body["overall_length_m"]),
            nose_fairing_length_m=float(body["nose_fairing_length_m"]),
            boattail_length_m=float(body["boattail_length_m"]),
            boattail_start_x_m=float(stations["boattail_start"]),
            frontal_area_m2=float(body["frontal_area_m2"]),
            nozzle_exit_diameter_m=float(flow["nozzle_exit_diameter_m"]),
            chamber_start_x_m=float(stations["combustion_chamber_start"]),
            chamber_end_x_m=float(stations["combustion_chamber_end__tailpipe_start"]),
            wing_span_m=float(wing["span_m"]),
            wing_reference_area_m2=float(wing["reference_area_m2"]),
            wing_root_chord_m=float(wing["root_chord_m"]),
            wing_tip_chord_m=float(wing["tip_chord_m"]),
            wing_taper_ratio=float(wing["taper_ratio_tip_over_root"]),
            wing_mean_aerodynamic_chord_m=float(wing["mean_aerodynamic_chord_m"]),
            wing_mac_y_m=float(wing["mac_spanwise_station_from_centreline_m"]),
            wing_le_sweep_deg=float(wing["leading_edge_sweep_deg"]),
            wing_quarter_chord_sweep_deg=float(wing["quarter_chord_sweep_deg"]),
            wing_thickness_to_chord=float(wing["thickness_to_chord"]),
            wing_airfoil_key=str(wing["airfoil_key"]),
            raw=data,
        )
        record.validate()
        return record

    def validate(self) -> None:
        """Reject a sizing file whose own numbers do not close.

        Cheap, but it catches a hand-edited or half-regenerated export before it
        becomes a silently wrong mold line.
        """

        problems: list[str] = []
        if self.wing_span_m <= self.body_diameter_m:
            problems.append(
                f"wing span {self.wing_span_m:.4f} m does not clear the body "
                f"diameter {self.body_diameter_m:.4f} m -- no exposed panel exists"
            )
        trapezoid_area = 0.5 * (self.wing_root_chord_m + self.wing_tip_chord_m) * self.wing_span_m
        if abs(trapezoid_area - self.wing_reference_area_m2) > 1e-6:
            problems.append(
                f"wing reference area {self.wing_reference_area_m2:.6f} m^2 disagrees "
                f"with its own root/tip/span trapezoid {trapezoid_area:.6f} m^2"
            )
        if not (0.0 < self.boattail_start_x_m < self.body_length_m):
            problems.append("boattail start is not inside the body length")
        expected_boattail = self.body_length_m - self.boattail_start_x_m
        if abs(expected_boattail - self.boattail_length_m) > 1e-6:
            problems.append(
                f"boattail length {self.boattail_length_m:.6f} m disagrees with its "
                f"own stations ({expected_boattail:.6f} m)"
            )
        if self.nozzle_exit_diameter_m >= self.body_diameter_m:
            problems.append("nozzle exit diameter is not smaller than the body diameter")
        if problems:
            raise ValueError(
                f"{self.source_path} is internally inconsistent:\n  - "
                + "\n  - ".join(problems)
            )
