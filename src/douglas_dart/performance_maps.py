"""Cycle-averaged propulsion maps used by the mission time scale."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass

from .config import ReferenceCase
from .pulsejet import PulsejetSimulator, summarize_pulsejet


@dataclass(frozen=True)
class EngineMapValue:
    net_thrust_n: float
    fuel_mass_flow_kg_per_s: float
    clamped_to_map_boundary: bool = False


@dataclass(frozen=True)
class PulsejetMapPoint:
    altitude_m: float
    mach: float
    mean_net_thrust_n: float
    mean_fuel_mass_flow_kg_per_s: float
    peak_chamber_pressure_pa: float
    peak_chamber_temperature_k: float
    completed_cycles: int
    numerical_reference_only: bool = True


@dataclass(frozen=True)
class RectilinearEngineMap:
    """Bilinear map with visible boundary clamping and no extrapolation."""

    altitudes_m: tuple[float, ...]
    mach_values: tuple[float, ...]
    net_thrust_rows_n: tuple[tuple[float, ...], ...]
    fuel_flow_rows_kg_per_s: tuple[tuple[float, ...], ...]
    numerical_reference_only: bool = True

    def __post_init__(self) -> None:
        if len(self.altitudes_m) < 2 or len(self.mach_values) < 2:
            raise ValueError("engine map requires at least a two-by-two grid")
        if tuple(sorted(set(self.altitudes_m))) != self.altitudes_m:
            raise ValueError("engine-map altitudes must be strictly increasing")
        if tuple(sorted(set(self.mach_values))) != self.mach_values:
            raise ValueError("engine-map Mach values must be strictly increasing")
        if len(self.net_thrust_rows_n) != len(self.altitudes_m):
            raise ValueError("net-thrust row count does not match altitude grid")
        if len(self.fuel_flow_rows_kg_per_s) != len(self.altitudes_m):
            raise ValueError("fuel-flow row count does not match altitude grid")
        for rows, name in (
            (self.net_thrust_rows_n, "net thrust"),
            (self.fuel_flow_rows_kg_per_s, "fuel flow"),
        ):
            if any(len(row) != len(self.mach_values) for row in rows):
                raise ValueError(f"{name} column count does not match Mach grid")
        if any(value < 0.0 for row in self.fuel_flow_rows_kg_per_s for value in row):
            raise ValueError("engine-map fuel flow cannot be negative")

    @staticmethod
    def _bracket(values: tuple[float, ...], query: float) -> tuple[int, int, float, bool]:
        if query < values[0]:
            return 0, 0, 0.0, True
        if query > values[-1]:
            index = len(values) - 1
            return index, index, 0.0, True
        if query == values[0]:
            return 0, 0, 0.0, False
        if query == values[-1]:
            index = len(values) - 1
            return index, index, 0.0, False
        upper = bisect_right(values, query)
        lower = upper - 1
        fraction = (query - values[lower]) / (values[upper] - values[lower])
        return lower, upper, fraction, False

    @staticmethod
    def _interpolate_grid(
        rows: tuple[tuple[float, ...], ...],
        altitude_bracket: tuple[int, int, float, bool],
        mach_bracket: tuple[int, int, float, bool],
    ) -> float:
        altitude_low, altitude_high, altitude_fraction, _ = altitude_bracket
        mach_low, mach_high, mach_fraction, _ = mach_bracket
        low_altitude_value = rows[altitude_low][mach_low] + mach_fraction * (
            rows[altitude_low][mach_high] - rows[altitude_low][mach_low]
        )
        high_altitude_value = rows[altitude_high][mach_low] + mach_fraction * (
            rows[altitude_high][mach_high] - rows[altitude_high][mach_low]
        )
        return low_altitude_value + altitude_fraction * (
            high_altitude_value - low_altitude_value
        )

    def evaluate(self, altitude_m: float, mach: float) -> EngineMapValue:
        altitude_bracket = self._bracket(self.altitudes_m, altitude_m)
        mach_bracket = self._bracket(self.mach_values, mach)
        return EngineMapValue(
            net_thrust_n=self._interpolate_grid(
                self.net_thrust_rows_n,
                altitude_bracket,
                mach_bracket,
            ),
            fuel_mass_flow_kg_per_s=self._interpolate_grid(
                self.fuel_flow_rows_kg_per_s,
                altitude_bracket,
                mach_bracket,
            ),
            clamped_to_map_boundary=altitude_bracket[3] or mach_bracket[3],
        )


@dataclass(frozen=True)
class PulsejetMapBuildResult:
    engine_map: RectilinearEngineMap
    points: tuple[PulsejetMapPoint, ...]


def build_pulsejet_performance_map(
    case: ReferenceCase,
    *,
    altitudes_m: tuple[float, ...] | None = None,
    mach_values: tuple[float, ...] | None = None,
    warmup_s: float | None = None,
    measurement_s: float | None = None,
    time_step_s: float | None = None,
) -> PulsejetMapBuildResult:
    """Generate a startup-excluded map from independent chamber simulations."""

    altitudes = (
        case.mission_simulation.pulsejet_map_altitudes_m
        if altitudes_m is None
        else tuple(altitudes_m)
    )
    machs = (
        case.mission_simulation.pulsejet_map_mach_values
        if mach_values is None
        else tuple(mach_values)
    )
    if tuple(sorted(set(altitudes))) != altitudes or len(altitudes) < 2:
        raise ValueError("pulsejet map altitudes must be strictly increasing")
    if tuple(sorted(set(machs))) != machs or len(machs) < 2:
        raise ValueError("pulsejet map Mach values must be strictly increasing")
    warmup = (
        case.simulation.pulsejet_steady_warmup_s
        if warmup_s is None
        else warmup_s
    )
    measurement = (
        case.simulation.pulsejet_steady_measurement_s
        if measurement_s is None
        else measurement_s
    )
    time_step = case.simulation.time_step_s if time_step_s is None else time_step_s
    if warmup <= 0.0 or measurement <= 0.0 or time_step <= 0.0:
        raise ValueError("pulsejet map timing inputs must be positive")

    points: list[PulsejetMapPoint] = []
    thrust_rows: list[tuple[float, ...]] = []
    fuel_rows: list[tuple[float, ...]] = []
    for altitude_m in altitudes:
        thrust_row: list[float] = []
        fuel_row: list[float] = []
        for mach in machs:
            simulator = PulsejetSimulator(
                case.pulsejet,
                case.selector,
                case.nozzle,
                case.fuel,
                altitude_m,
                mach,
            )
            summary = summarize_pulsejet(
                simulator.run(warmup + measurement, time_step),
                minimum_time_s=warmup,
            )
            point = PulsejetMapPoint(
                altitude_m=altitude_m,
                mach=mach,
                mean_net_thrust_n=summary.mean_net_thrust_n,
                mean_fuel_mass_flow_kg_per_s=summary.mean_fuel_mass_flow_kg_per_s,
                peak_chamber_pressure_pa=summary.peak_chamber_pressure_pa,
                peak_chamber_temperature_k=summary.peak_chamber_temperature_k,
                completed_cycles=summary.completed_cycles,
            )
            points.append(point)
            thrust_row.append(point.mean_net_thrust_n)
            fuel_row.append(point.mean_fuel_mass_flow_kg_per_s)
        thrust_rows.append(tuple(thrust_row))
        fuel_rows.append(tuple(fuel_row))

    return PulsejetMapBuildResult(
        engine_map=RectilinearEngineMap(
            altitudes_m=altitudes,
            mach_values=machs,
            net_thrust_rows_n=tuple(thrust_rows),
            fuel_flow_rows_kg_per_s=tuple(fuel_rows),
        ),
        points=tuple(points),
    )
