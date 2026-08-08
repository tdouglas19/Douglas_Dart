"""Validated interpolation and persistence for VSPAERO coefficient tables.

VSPAERO supplies inviscid external-aerodynamic coefficients. This module keeps
that provenance visible and does not infer viscous, base, inlet, or spillage drag.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import isclose
from pathlib import Path

from .vspaero import VSPAeroPoint, VSPAeroSweepSummary


@dataclass(frozen=True)
class VSPAeroCoefficientValue:
    mach: float
    alpha_deg: float
    beta_deg: float
    lift_coefficient: float
    drag_coefficient_inviscid: float
    side_force_coefficient: float
    rolling_moment_coefficient: float
    pitching_moment_coefficient: float
    yawing_moment_coefficient: float
    clamped_to_table_boundary: bool
    external_aerodynamics_only: bool = True


def _bracket(values: tuple[float, ...], query: float) -> tuple[int, int, float, bool]:
    if query <= values[0]:
        return 0, 0, 0.0, query < values[0]
    if query >= values[-1]:
        last = len(values) - 1
        return last, last, 0.0, query > values[-1]
    for lower_index, (lower, upper) in enumerate(zip(values, values[1:])):
        if lower <= query <= upper:
            fraction = (query - lower) / (upper - lower)
            return lower_index, lower_index + 1, fraction, False
    raise RuntimeError("failed to bracket a finite table query")


def _bilinear(
    rows: tuple[tuple[float, ...], ...],
    mach_bracket: tuple[int, int, float, bool],
    alpha_bracket: tuple[int, int, float, bool],
) -> float:
    mach_low, mach_high, mach_fraction, _ = mach_bracket
    alpha_low, alpha_high, alpha_fraction, _ = alpha_bracket
    low_mach_value = rows[mach_low][alpha_low] + alpha_fraction * (
        rows[mach_low][alpha_high] - rows[mach_low][alpha_low]
    )
    high_mach_value = rows[mach_high][alpha_low] + alpha_fraction * (
        rows[mach_high][alpha_high] - rows[mach_high][alpha_low]
    )
    return low_mach_value + mach_fraction * (high_mach_value - low_mach_value)


@dataclass(frozen=True)
class VSPAeroCoefficientTable:
    """One complete rectangular longitudinal slice of a VSPAERO sweep."""

    case_name: str
    reference_area_m2: float
    reference_span_m: float
    reference_chord_m: float
    reference_cg_x_m: float
    beta_deg: float
    mach_values: tuple[float, ...]
    alpha_deg_values: tuple[float, ...]
    lift_coefficient_rows: tuple[tuple[float, ...], ...]
    drag_coefficient_inviscid_rows: tuple[tuple[float, ...], ...]
    side_force_coefficient_rows: tuple[tuple[float, ...], ...]
    rolling_moment_coefficient_rows: tuple[tuple[float, ...], ...]
    pitching_moment_coefficient_rows: tuple[tuple[float, ...], ...]
    yawing_moment_coefficient_rows: tuple[tuple[float, ...], ...]
    openvsp_version: str
    analysis_method: str
    live_solver_run: bool

    def __post_init__(self) -> None:
        if not self.case_name.strip():
            raise ValueError("VSPAERO table case name cannot be empty")
        if self.reference_area_m2 <= 0.0:
            raise ValueError("VSPAERO reference area must be positive")
        if len(self.mach_values) < 2 or len(self.alpha_deg_values) < 2:
            raise ValueError("VSPAERO table needs at least two Mach and alpha values")
        if tuple(sorted(set(self.mach_values))) != self.mach_values:
            raise ValueError("VSPAERO Mach values must be strictly increasing")
        if tuple(sorted(set(self.alpha_deg_values))) != self.alpha_deg_values:
            raise ValueError("VSPAERO alpha values must be strictly increasing")
        expected_shape = (len(self.mach_values), len(self.alpha_deg_values))
        for name in (
            "lift_coefficient_rows",
            "drag_coefficient_inviscid_rows",
            "side_force_coefficient_rows",
            "rolling_moment_coefficient_rows",
            "pitching_moment_coefficient_rows",
            "yawing_moment_coefficient_rows",
        ):
            rows = getattr(self, name)
            shape = (len(rows), len(rows[0]) if rows else 0)
            if shape != expected_shape or any(
                len(row) != expected_shape[1] for row in rows
            ):
                raise ValueError(
                    f"{name} shape {shape} does not match {expected_shape}"
                )

    @classmethod
    def from_summary(
        cls,
        summary: VSPAeroSweepSummary,
        *,
        beta_deg: float = 0.0,
        beta_tolerance_deg: float = 1.0e-9,
    ) -> "VSPAeroCoefficientTable":
        selected = tuple(
            point
            for point in summary.points
            if isclose(
                point.beta_deg,
                beta_deg,
                rel_tol=0.0,
                abs_tol=beta_tolerance_deg,
            )
        )
        if not selected:
            raise ValueError(f"VSPAERO summary has no beta={beta_deg:g} deg points")
        mach_values = tuple(sorted({point.mach for point in selected}))
        alpha_values = tuple(sorted({point.alpha_deg for point in selected}))
        by_coordinate: dict[tuple[float, float], VSPAeroPoint] = {}
        for point in selected:
            coordinate = (point.mach, point.alpha_deg)
            if coordinate in by_coordinate:
                raise ValueError(
                    "VSPAERO summary contains duplicate longitudinal coordinate "
                    f"Mach={point.mach:g}, alpha={point.alpha_deg:g} deg"
                )
            by_coordinate[coordinate] = point
        missing = [
            (mach, alpha)
            for mach in mach_values
            for alpha in alpha_values
            if (mach, alpha) not in by_coordinate
        ]
        if missing:
            mach, alpha = missing[0]
            raise ValueError(
                "VSPAERO longitudinal slice is not rectangular; first missing point "
                f"is Mach={mach:g}, alpha={alpha:g} deg"
            )

        def rows(attribute: str) -> tuple[tuple[float, ...], ...]:
            return tuple(
                tuple(
                    float(getattr(by_coordinate[(mach, alpha)], attribute))
                    for alpha in alpha_values
                )
                for mach in mach_values
            )

        return cls(
            case_name=summary.case_name,
            reference_area_m2=summary.reference_area_m2,
            reference_span_m=summary.reference_span_m,
            reference_chord_m=summary.reference_chord_m,
            reference_cg_x_m=summary.reference_cg_x_m,
            beta_deg=beta_deg,
            mach_values=mach_values,
            alpha_deg_values=alpha_values,
            lift_coefficient_rows=rows("lift_coefficient"),
            drag_coefficient_inviscid_rows=rows(
                "drag_coefficient_inviscid"
            ),
            side_force_coefficient_rows=rows("side_force_coefficient"),
            rolling_moment_coefficient_rows=rows(
                "rolling_moment_coefficient"
            ),
            pitching_moment_coefficient_rows=rows(
                "pitching_moment_coefficient"
            ),
            yawing_moment_coefficient_rows=rows(
                "yawing_moment_coefficient"
            ),
            openvsp_version=summary.openvsp_version,
            analysis_method=summary.analysis_method,
            live_solver_run=summary.live_solver_run,
        )

    def evaluate(self, mach: float, alpha_deg: float) -> VSPAeroCoefficientValue:
        if mach < 0.0:
            raise ValueError("Mach cannot be negative")
        mach_bracket = _bracket(self.mach_values, mach)
        alpha_bracket = _bracket(self.alpha_deg_values, alpha_deg)

        def coefficient(rows: tuple[tuple[float, ...], ...]) -> float:
            return _bilinear(rows, mach_bracket, alpha_bracket)

        return VSPAeroCoefficientValue(
            mach=mach,
            alpha_deg=alpha_deg,
            beta_deg=self.beta_deg,
            lift_coefficient=coefficient(self.lift_coefficient_rows),
            drag_coefficient_inviscid=coefficient(
                self.drag_coefficient_inviscid_rows
            ),
            side_force_coefficient=coefficient(
                self.side_force_coefficient_rows
            ),
            rolling_moment_coefficient=coefficient(
                self.rolling_moment_coefficient_rows
            ),
            pitching_moment_coefficient=coefficient(
                self.pitching_moment_coefficient_rows
            ),
            yawing_moment_coefficient=coefficient(
                self.yawing_moment_coefficient_rows
            ),
            clamped_to_table_boundary=(mach_bracket[3] or alpha_bracket[3]),
        )

    def alpha_deg_for_lift_coefficient(
        self,
        mach: float,
        target_lift_coefficient: float,
    ) -> float:
        """Invert a monotonic alpha slice for the longitudinal controller."""

        slice_values = tuple(
            self.evaluate(mach, alpha).lift_coefficient
            for alpha in self.alpha_deg_values
        )
        if any(
            upper <= lower for lower, upper in zip(slice_values, slice_values[1:])
        ):
            raise ValueError(
                "VSPAERO lift coefficient must increase strictly with alpha for "
                "longitudinal control inversion"
            )
        if target_lift_coefficient <= slice_values[0]:
            return self.alpha_deg_values[0]
        if target_lift_coefficient >= slice_values[-1]:
            return self.alpha_deg_values[-1]
        for index, (lower, upper) in enumerate(
            zip(slice_values, slice_values[1:])
        ):
            if lower <= target_lift_coefficient <= upper:
                fraction = (target_lift_coefficient - lower) / (upper - lower)
                return self.alpha_deg_values[index] + fraction * (
                    self.alpha_deg_values[index + 1]
                    - self.alpha_deg_values[index]
                )
        raise RuntimeError("failed to invert a finite VSPAERO lift slice")


def write_vspaero_summary_json(
    summary: VSPAeroSweepSummary,
    path: str | Path,
) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        json.dump(asdict(summary), stream, indent=2)
        stream.write("\n")


def load_vspaero_summary_json(path: str | Path) -> VSPAeroSweepSummary:
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError("VSPAERO summary JSON must contain an object")
    raw_points = payload.pop("points", None)
    if not isinstance(raw_points, list):
        raise ValueError("VSPAERO summary JSON must contain a points array")
    try:
        points = []
        for raw_point in raw_points:
            point = dict(raw_point)
            point["status"] = tuple(point["status"])
            points.append(VSPAeroPoint(**point))
        return VSPAeroSweepSummary(points=tuple(points), **payload)
    except (KeyError, TypeError) as error:
        raise ValueError(f"invalid VSPAERO summary JSON: {error}") from error
