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

## What is NOT optimized here

Body diameter growth's structural-mass penalty is not modeled (there is no
parametric mass-vs-geometry relationship yet — see `robustness.py`'s fixed component
list). Growing the body or throat only changes drag area and nozzle flow capacity in
this search; the resulting "empty" (non-fuel) mass is held at the configured value.
Any candidate the search prefers with a substantially different body diameter should
be re-checked against a real mass budget before it is taken seriously.
"""

from __future__ import annotations

import csv
import json
import random
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

from .config import ReferenceCase
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
    throat_diameter_m: tuple[float, float]
    exit_to_throat_area_ratio: tuple[float, float]
    loaded_fuel_mass_kg: tuple[float, float]
    ramjet_fuel_fraction: tuple[float, float]
    climb_angle_deg: tuple[float, float]
    dive_angle_deg: tuple[float, float]
    dive_entry_mach: tuple[float, float]

    def names(self) -> tuple[str, ...]:
        return (
            "body_diameter_m",
            "throat_diameter_m",
            "exit_to_throat_area_ratio",
            "loaded_fuel_mass_kg",
            "ramjet_fuel_fraction",
            "climb_angle_deg",
            "dive_angle_deg",
            "dive_entry_mach",
        )

    def as_pairs(self) -> tuple[tuple[float, float], ...]:
        return tuple(getattr(self, name) for name in self.names())


DEFAULT_BOUNDS = DesignVariableBounds(
    body_diameter_m=(0.195, 0.260),
    throat_diameter_m=(0.110, 0.190),
    exit_to_throat_area_ratio=(1.02, 1.30),
    loaded_fuel_mass_kg=(2.50, 6.00),
    ramjet_fuel_fraction=(0.20, 0.80),
    climb_angle_deg=(3.0, 15.0),
    dive_angle_deg=(-20.0, -3.0),
    dive_entry_mach=(0.30, 0.78),
)


@dataclass(frozen=True)
class DesignVariables:
    body_diameter_m: float
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    loaded_fuel_mass_kg: float
    ramjet_fuel_fraction: float
    climb_angle_deg: float
    dive_angle_deg: float
    dive_entry_mach: float

    def as_vector(self, bounds: DesignVariableBounds) -> list[float]:
        return [getattr(self, name) for name in bounds.names()]

    @classmethod
    def from_vector(cls, vector: list[float], bounds: DesignVariableBounds) -> "DesignVariables":
        return cls(**dict(zip(bounds.names(), vector)))


def apply_design_variables(case: ReferenceCase, variables: DesignVariables) -> ReferenceCase:
    """Build a candidate `ReferenceCase` from a baseline case and one design vector.

    Non-fuel ("empty") mass is held fixed at the baseline's configured value; only
    the fuel-mass contribution changes with `loaded_fuel_mass_kg`. This keeps the
    mass model honest about what it actually represents (see module docstring).
    """

    baseline_empty_mass_kg = case.flight.initial_mass_kg - case.mission.loaded_fuel_mass_kg
    new_initial_mass_kg = baseline_empty_mass_kg + variables.loaded_fuel_mass_kg
    ramjet_fuel_kg = variables.ramjet_fuel_fraction * variables.loaded_fuel_mass_kg

    vehicle = replace(case.vehicle, body_diameter_m=variables.body_diameter_m)
    nozzle = replace(
        case.nozzle,
        throat_diameter_m=variables.throat_diameter_m,
        exit_to_throat_area_ratio=variables.exit_to_throat_area_ratio,
    )
    flight = replace(case.flight, initial_mass_kg=new_initial_mass_kg)
    mission = replace(
        case.mission,
        loaded_fuel_mass_kg=variables.loaded_fuel_mass_kg,
        ramjet_speed_run_fuel_budget_kg=ramjet_fuel_kg,
    )
    return replace(case, vehicle=vehicle, nozzle=nozzle, flight=flight, mission=mission)


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
        candidate_case = apply_design_variables(baseline_case, variables)
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
    """

    rng = random.Random(seed)
    pairs = bounds.as_pairs()
    dimension = len(pairs)

    def random_vector() -> list[float]:
        return [rng.uniform(lower, upper) for lower, upper in pairs]

    def clip_vector(vector: list[float]) -> list[float]:
        return [_clip(value, lower, upper) for value, (lower, upper) in zip(vector, pairs)]

    population = [random_vector() for _ in range(population_size)]
    evaluations = [
        evaluate_design(baseline_case, DesignVariables.from_vector(vector, bounds))
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
                baseline_case, DesignVariables.from_vector(trial, bounds)
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
