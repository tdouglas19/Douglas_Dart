"""Local, standalone coupled-design search over trajectory closure.

This is meant to run unattended on your own machine for as long as you want — it
does not call out to any AI model, so it costs nothing but CPU time and wall clock.
`douglas-dart design-optimize` (or `python -m douglas_dart.optimizer`) drives it from
the command line; progress is checkpointed to CSV/JSON after every generation so a
long run can be interrupted, inspected, and resumed.

## Method

Differential evolution (Storn & Price, 1997) over a small population of candidate
design-variable vectors, gradient-free by construction. That matters here because the
objective is not smooth: `trajectory.py`'s phase transitions, fuel-exhaustion events,
and the ``ValueError``s raised by an infeasible `ReferenceCase` combination all
produce discontinuities that would break a gradient-based method. No SciPy
dependency; the algorithm is small enough to keep in-repo and inspectable.

## Mass model

`mass_model.py` now supplies a real, geometry-linked empty-mass calculation
(body skin, lifting surfaces, propulsion hardware, and selector each scale
with their own geometry; everything else is a fixed lump sum) instead of
holding empty mass at the configured value regardless of body diameter,
length, or throat size. Its density coefficients are calibrated to reproduce
today's configured mass budget at today's configured geometry -- a
consistency anchor, not independent structural validation (see
`mass_model.py`'s module docstring and `docs/assumptions_registry.md`). Body
length is now an active search variable for the same reason it is in
`docs/design_workflow.md`'s Level 3: it changes wetted area and therefore
structural mass, not just packaging.

Lifting-surface area is now a search variable too (`wing_area_scale_factor`):
root chord, tip chord, and exposed semispan all scale by
``sqrt(wing_area_scale_factor)``, so planform area scales by exactly
``wing_area_scale_factor`` while aspect ratio and taper ratio stay fixed at
the baseline config's shape (geometric similarity, not independent shape
optimization). `flight.reference_area_m2` is recomputed to exactly match
(`ReferenceCase`'s own validated invariant), so `mass_model.py`'s
lifting-surface mass and every drag/lift calculation that reads
`reference_area_m2` see the same, consistent number. Fin geometry and
lifting-surface x-location/sweep/thickness are still read from the baseline
config, not independently searched.
"""

from __future__ import annotations

import csv
import json
import random
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

from .atmosphere import G0_M_PER_S2
from .config import ReferenceCase
from .mass_model import MassModelCalibration, calibrate_mass_model, evaluate_parametric_mass
from .trajectory import (
    ADVERSE_SCENARIO,
    CONSERVATIVE_SCENARIO,
    NOMINAL_SCENARIO,
    TrajectoryResult,
    simulate_mission,
)


@dataclass(frozen=True)
class DesignVariableBounds:
    """Search bounds for each coupled design variable, in SI units."""

    body_diameter_m: tuple[float, float]
    body_length_m: tuple[float, float]
    throat_diameter_m: tuple[float, float]
    exit_to_throat_area_ratio: tuple[float, float]
    loaded_fuel_mass_kg: tuple[float, float]
    ramjet_fuel_fraction: tuple[float, float]
    climb_angle_deg: tuple[float, float]
    dive_angle_deg: tuple[float, float]
    dive_entry_mach: tuple[float, float]
    sled_release_speed_m_per_s: tuple[float, float]
    wing_area_scale_factor: tuple[float, float]

    def names(self) -> tuple[str, ...]:
        return (
            "body_diameter_m",
            "body_length_m",
            "throat_diameter_m",
            "exit_to_throat_area_ratio",
            "loaded_fuel_mass_kg",
            "ramjet_fuel_fraction",
            "climb_angle_deg",
            "dive_angle_deg",
            "dive_entry_mach",
            "sled_release_speed_m_per_s",
            "wing_area_scale_factor",
        )

    def as_pairs(self) -> tuple[tuple[float, float], ...]:
        return tuple(getattr(self, name) for name in self.names())


# sled_release_speed_m_per_s: reclassified from a fixed mission requirement to
# a Level 2 search variable (see MissionConfig's docstring and
# docs/level0_feasibility_bounds.md -- release speed directly sets the
# Gate 1 stall-speed margin). The lower bound (35 m/s) sits just under the
# previously configured 39-42 m/s range. The upper bound is v_max =
# sqrt(2 * a * L) at MissionConfig's own default rail length (75 m) and
# launch-acceleration placeholder (10 g) -- i.e. it is tied to the one real,
# user-stated requirement ("reach release speed within the rail"), not an
# arbitrary speed literal. `a` is still an UNSOURCED placeholder (see
# MissionConfig's docstring and feasibility.py's `sled_launch_feasibility`,
# which reports the required acceleration for whatever speed a search picks
# so a human can judge it, rather than hard-failing on this placeholder).
# If a case's own sled_rail_length_m/sled_launch_acceleration_g differ from
# these defaults, recompute this bound to match.
_SLED_RAIL_LENGTH_DEFAULT_M = 75.0
_SLED_LAUNCH_ACCELERATION_DEFAULT_G = 10.0
_SLED_RELEASE_SPEED_UPPER_BOUND_M_PER_S = (
    2.0 * _SLED_LAUNCH_ACCELERATION_DEFAULT_G * G0_M_PER_S2 * _SLED_RAIL_LENGTH_DEFAULT_M
) ** 0.5

DEFAULT_BOUNDS = DesignVariableBounds(
    body_diameter_m=(0.195, 0.260),
    # Wide enough to matter for wetted area/structural mass (mass_model.py)
    # and tail moment arm, narrow enough to stay near the configured 2.20-
    # 2.30 m baseline this search's other bounds were tuned against.
    body_length_m=(1.80, 2.80),
    throat_diameter_m=(0.110, 0.190),
    exit_to_throat_area_ratio=(1.02, 1.30),
    loaded_fuel_mass_kg=(2.50, 6.00),
    ramjet_fuel_fraction=(0.20, 0.80),
    climb_angle_deg=(3.0, 15.0),
    dive_angle_deg=(-20.0, -3.0),
    dive_entry_mach=(0.30, 0.78),
    sled_release_speed_m_per_s=(35.0, _SLED_RELEASE_SPEED_UPPER_BOUND_M_PER_S),
    # docs/level0_feasibility_bounds.md found the configured reference area is
    # roughly 3.6x too small for the configured release speed at max mass;
    # the upper bound here is set generously above that so the search can
    # actually reach and cross that threshold rather than stopping just short
    # of it.
    wing_area_scale_factor=(0.5, 6.0),
)


@dataclass(frozen=True)
class DesignVariables:
    body_diameter_m: float
    body_length_m: float
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    loaded_fuel_mass_kg: float
    ramjet_fuel_fraction: float
    climb_angle_deg: float
    dive_angle_deg: float
    dive_entry_mach: float
    sled_release_speed_m_per_s: float
    wing_area_scale_factor: float

    def as_vector(self, bounds: DesignVariableBounds) -> list[float]:
        return [getattr(self, name) for name in bounds.names()]

    @classmethod
    def from_vector(cls, vector: list[float], bounds: DesignVariableBounds) -> "DesignVariables":
        return cls(**dict(zip(bounds.names(), vector)))


def apply_design_variables(
    case: ReferenceCase,
    variables: DesignVariables,
    mass_calibration: MassModelCalibration,
) -> ReferenceCase:
    """Build a candidate `ReferenceCase` from a baseline case and one design vector.

    Non-fuel ("empty") mass now comes from `mass_model.py`'s geometry-linked
    calculation (body diameter/length, throat diameter, lifting-surface area)
    instead of being held fixed at the baseline's configured value regardless
    of geometry -- see this module's docstring and `mass_model.py`'s for what
    is and is not scaled. `mass_calibration` is required, not defaulted, so a
    caller cannot silently fall back to the old fixed-mass behavior by
    omitting it.
    """

    # Geometric-similarity wing scaling: root/tip chord and semispan all scale
    # by sqrt(factor) so planform area scales by exactly `factor` while aspect
    # and taper ratio stay fixed at the baseline shape (see module docstring).
    linear_scale = variables.wing_area_scale_factor**0.5
    lifting_surface = replace(
        case.geometry.lifting_surface,
        root_chord_m=case.geometry.lifting_surface.root_chord_m * linear_scale,
        tip_chord_m=case.geometry.lifting_surface.tip_chord_m * linear_scale,
        exposed_semispan_m=case.geometry.lifting_surface.exposed_semispan_m * linear_scale,
    )
    geometry = replace(case.geometry, lifting_surface=lifting_surface)
    new_reference_area_m2 = (
        geometry.lifting_surface_count * lifting_surface.exposed_area_per_surface_m2
        if geometry.lifting_surface_count
        else case.flight.reference_area_m2
    )

    vehicle = replace(
        case.vehicle,
        body_diameter_m=variables.body_diameter_m,
        body_length_m=variables.body_length_m,
    )
    # A case reflecting the new body + wing geometry (and the reference area
    # that must match it, per ReferenceCase's own validated invariant), built
    # before the mass calculation so evaluate_parametric_mass's own geometry
    # reads (wetted area, lifting/fin area) see the scaled values
    # automatically instead of needing every scaled quantity threaded through
    # as a separate override.
    geometry_case = replace(
        case,
        vehicle=vehicle,
        geometry=geometry,
        flight=replace(case.flight, reference_area_m2=new_reference_area_m2),
    )
    empty_mass_kg = evaluate_parametric_mass(
        geometry_case,
        mass_calibration,
        throat_diameter_m=variables.throat_diameter_m,
    ).empty_mass_kg
    new_initial_mass_kg = empty_mass_kg + variables.loaded_fuel_mass_kg
    ramjet_fuel_kg = variables.ramjet_fuel_fraction * variables.loaded_fuel_mass_kg

    nozzle = replace(
        case.nozzle,
        throat_diameter_m=variables.throat_diameter_m,
        exit_to_throat_area_ratio=variables.exit_to_throat_area_ratio,
    )
    flight = replace(
        case.flight,
        initial_mass_kg=new_initial_mass_kg,
        reference_area_m2=new_reference_area_m2,
    )
    # trajectory.py's mission solver reads sled_release_speed_max_m_per_s as the
    # initial condition; both min and max are set to the searched value here
    # rather than kept as an independent range, since the range previously
    # represented sled-performance uncertainty, not a design choice.
    mission = replace(
        case.mission,
        loaded_fuel_mass_kg=variables.loaded_fuel_mass_kg,
        ramjet_speed_run_fuel_budget_kg=ramjet_fuel_kg,
        sled_release_speed_min_m_per_s=variables.sled_release_speed_m_per_s,
        sled_release_speed_max_m_per_s=variables.sled_release_speed_m_per_s,
    )
    return replace(
        case, vehicle=vehicle, nozzle=nozzle, flight=flight, mission=mission, geometry=geometry
    )


@dataclass(frozen=True)
class CandidateEvaluation:
    variables: DesignVariables
    feasible: bool
    infeasibility_reason: str | None
    score: float
    nominal_peak_mach: float | None
    adverse_peak_mach: float | None
    nominal_meets_duration: bool | None
    adverse_meets_duration: bool | None
    nominal_rule_satisfied: bool | None
    adverse_rule_satisfied: bool | None
    mass_margin_kg: float | None


# Every weight below is visible here, not hidden inside the score, per the same
# discipline `robustness.py` uses for its named scenarios. Adverse is weighted three
# times nominal because the user's goal is a design that closes even off-nominal, not
# a design that only closes in the best case.
_SCENARIO_WEIGHTS = {"nominal": 1.0, "adverse": 3.0}
_REACHED_PEAK_MACH_REWARD = 200.0
_MISSED_PEAK_MACH_PENALTY = 1000.0
_DURATION_MET_REWARD = 300.0
_DURATION_MISSED_PENALTY = 500.0
_RULE_VIOLATION_PENALTY = 2000.0
_TIME_ABOVE_MACH_ONE_REWARD_PER_S = 5.0
_MASS_MARGIN_REWARD_PER_KG = 50.0
_BODY_DIAMETER_TIEBREAK_PENALTY_PER_M = 1000.0


def evaluate_design(
    baseline_case: ReferenceCase,
    variables: DesignVariables,
    mass_calibration: MassModelCalibration,
    *,
    fast_time_step_s: float = 0.08,
    fast_pulsejet_warmup_s: float = 0.10,
    fast_pulsejet_measurement_s: float = 0.10,
    fast_pulsejet_time_step_s: float = 0.0001,
    max_time_s: float = 200.0,
) -> CandidateEvaluation:
    """Score one design-variable vector against the nominal and adverse scenarios.

    Runs at reduced pulsejet-table fidelity by default for search speed; re-run the
    winning candidate through `simulate_mission` at full fidelity before trusting it.
    """

    try:
        candidate_case = apply_design_variables(baseline_case, variables, mass_calibration)
    except Exception as exc:  # noqa: BLE001 - a long unattended search must not die on one bad candidate
        return CandidateEvaluation(
            variables=variables,
            feasible=False,
            infeasibility_reason=str(exc),
            score=-1e9,
            nominal_peak_mach=None,
            adverse_peak_mach=None,
            nominal_meets_duration=None,
            adverse_meets_duration=None,
            nominal_rule_satisfied=None,
            adverse_rule_satisfied=None,
            mass_margin_kg=None,
        )

    results: dict[str, TrajectoryResult] = {}
    for scenario in (NOMINAL_SCENARIO, ADVERSE_SCENARIO):
        try:
            results[scenario.name] = simulate_mission(
                candidate_case,
                scenario,
                time_step_s=fast_time_step_s,
                max_time_s=max_time_s,
                climb_angle_deg=variables.climb_angle_deg,
                dive_angle_deg=variables.dive_angle_deg,
                dive_entry_mach=variables.dive_entry_mach,
                pulsejet_table_warmup_s=fast_pulsejet_warmup_s,
                pulsejet_table_measurement_s=fast_pulsejet_measurement_s,
                pulsejet_table_time_step_s=fast_pulsejet_time_step_s,
            )
        except Exception as exc:  # noqa: BLE001 - a long unattended search must not die on one bad candidate
            return CandidateEvaluation(
                variables=variables,
                feasible=False,
                infeasibility_reason=str(exc),
                score=-1e9,
                nominal_peak_mach=None,
                adverse_peak_mach=None,
                nominal_meets_duration=None,
                adverse_meets_duration=None,
                nominal_rule_satisfied=None,
                adverse_rule_satisfied=None,
                mass_margin_kg=None,
            )

    score = 0.0
    for name, weight in _SCENARIO_WEIGHTS.items():
        result = results[name]
        if result.reached_peak_mach_target:
            score += weight * _REACHED_PEAK_MACH_REWARD
        else:
            score -= weight * _MISSED_PEAK_MACH_PENALTY
        if result.meets_minimum_time_above_mach_one:
            score += weight * _DURATION_MET_REWARD
        else:
            score -= weight * _DURATION_MISSED_PENALTY
        if not result.transonic_no_altitude_loss_rule_satisfied:
            score -= weight * _RULE_VIOLATION_PENALTY
        score += weight * result.time_above_mach_one_s * _TIME_ABOVE_MACH_ONE_REWARD_PER_S

    mass_margin_kg = (
        candidate_case.requirements.maximum_takeoff_mass_kg
        - candidate_case.flight.initial_mass_kg
    )
    score += mass_margin_kg * _MASS_MARGIN_REWARD_PER_KG
    score -= candidate_case.vehicle.body_diameter_m * _BODY_DIAMETER_TIEBREAK_PENALTY_PER_M

    return CandidateEvaluation(
        variables=variables,
        feasible=True,
        infeasibility_reason=None,
        score=score,
        nominal_peak_mach=results["nominal"].peak_mach_reached,
        adverse_peak_mach=results["adverse"].peak_mach_reached,
        nominal_meets_duration=results["nominal"].meets_minimum_time_above_mach_one,
        adverse_meets_duration=results["adverse"].meets_minimum_time_above_mach_one,
        nominal_rule_satisfied=results["nominal"].transonic_no_altitude_loss_rule_satisfied,
        adverse_rule_satisfied=results["adverse"].transonic_no_altitude_loss_rule_satisfied,
        mass_margin_kg=mass_margin_kg,
    )


@dataclass(frozen=True)
class GenerationRecord:
    generation: int
    best_score: float
    best_variables: DesignVariables
    best_evaluation: CandidateEvaluation
    mean_score: float
    feasible_count: int
    population_size: int


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def run_differential_evolution(
    baseline_case: ReferenceCase,
    *,
    mass_budget_path: str | Path = "configs/robustness_candidate_b.yaml",
    bounds: DesignVariableBounds = DEFAULT_BOUNDS,
    population_size: int = 20,
    generations: int = 30,
    mutation_factor: float = 0.6,
    crossover_probability: float = 0.7,
    seed: int | None = 0,
    checkpoint_path: str | Path | None = None,
    progress_callback: Any | None = None,
) -> list[GenerationRecord]:
    """Run differential evolution over `DEFAULT_BOUNDS`-shaped design vectors.

    Checkpoints the best-so-far result to `checkpoint_path` (JSON) after every
    generation if given, so a long unattended run can be inspected or killed and
    resumed from the CSV log without losing progress. This function itself never
    calls out to any AI model; it is meant to be started once and left running.

    `mass_budget_path` calibrates `mass_model.py` once, from `baseline_case`'s
    own configured geometry, before the search starts (see
    `apply_design_variables`'s docstring for what that calibration means and
    does not mean).
    """

    mass_calibration = calibrate_mass_model(baseline_case, mass_budget_path)
    rng = random.Random(seed)
    pairs = bounds.as_pairs()
    dimension = len(pairs)

    def random_vector() -> list[float]:
        return [rng.uniform(lower, upper) for lower, upper in pairs]

    def clip_vector(vector: list[float]) -> list[float]:
        return [_clip(value, lower, upper) for value, (lower, upper) in zip(vector, pairs)]

    population = [random_vector() for _ in range(population_size)]
    evaluations = [
        evaluate_design(baseline_case, DesignVariables.from_vector(vector, bounds), mass_calibration)
        for vector in population
    ]

    records: list[GenerationRecord] = []
    for generation in range(generations):
        for index in range(population_size):
            candidates = [i for i in range(population_size) if i != index]
            a, b, c = rng.sample(candidates, 3)
            mutant = [
                population[a][d]
                + mutation_factor * (population[b][d] - population[c][d])
                for d in range(dimension)
            ]
            mutant = clip_vector(mutant)
            trial = list(population[index])
            forced_dimension = rng.randrange(dimension)
            for d in range(dimension):
                if d == forced_dimension or rng.random() < crossover_probability:
                    trial[d] = mutant[d]
            trial_evaluation = evaluate_design(
                baseline_case, DesignVariables.from_vector(trial, bounds), mass_calibration
            )
            if trial_evaluation.score >= evaluations[index].score:
                population[index] = trial
                evaluations[index] = trial_evaluation

        best_index = max(range(population_size), key=lambda i: evaluations[i].score)
        feasible_count = sum(1 for e in evaluations if e.feasible)
        mean_score = sum(e.score for e in evaluations) / population_size
        record = GenerationRecord(
            generation=generation,
            best_score=evaluations[best_index].score,
            best_variables=DesignVariables.from_vector(population[best_index], bounds),
            best_evaluation=evaluations[best_index],
            mean_score=mean_score,
            feasible_count=feasible_count,
            population_size=population_size,
        )
        records.append(record)
        if progress_callback is not None:
            progress_callback(record)
        if checkpoint_path is not None:
            _write_checkpoint(checkpoint_path, records)

    return records


def _json_ready(value: Any) -> Any:
    from dataclasses import is_dataclass

    if is_dataclass(value):
        return _json_ready(asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    return value


def _write_checkpoint(path: str | Path, records: list[GenerationRecord]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_ready(records), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_generation_log_csv(path: str | Path, records: list[GenerationRecord]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "generation",
        "best_score",
        "mean_score",
        "feasible_count",
        "population_size",
        "nominal_peak_mach",
        "adverse_peak_mach",
        "nominal_meets_duration",
        "adverse_meets_duration",
        "mass_margin_kg",
        *DEFAULT_BOUNDS.names(),
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            row = {
                "generation": record.generation,
                "best_score": record.best_score,
                "mean_score": record.mean_score,
                "feasible_count": record.feasible_count,
                "population_size": record.population_size,
                "nominal_peak_mach": record.best_evaluation.nominal_peak_mach,
                "adverse_peak_mach": record.best_evaluation.adverse_peak_mach,
                "nominal_meets_duration": record.best_evaluation.nominal_meets_duration,
                "adverse_meets_duration": record.best_evaluation.adverse_meets_duration,
                "mass_margin_kg": record.best_evaluation.mass_margin_kg,
            }
            row.update(asdict(record.best_variables))
            writer.writerow(row)
    return path
