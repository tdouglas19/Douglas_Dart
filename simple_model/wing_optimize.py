"""Wing-concept optimizer: same closed-form philosophy as optimize.py, over
the five wing variables (span, aspect ratio, taper, sweep, airfoil family)
with the VEHICLE fixed -- "find the best wing for the flight regime we
settled on."

Each evaluation is a full launch-to-landing flight of the fixed vehicle
wearing the candidate wing (flight_sim.run_flight(..., wing_concept=...)),
judged by the same feasibility gates as the vehicle search (reach cutoff,
safe landing under the stall cap, fuel fits, minimum powered thrust
margin) with the same objective (minimize peak T/W). Wing evaluations cost
the same ~5-40 ms as vehicle evaluations, so a full wing search runs in a
few minutes and chains directly after the vehicle campaign (see
scripts/simple_model_overnight2.py).

Windows multiprocessing note: same rule as optimize.py -- callers must
guard behind `if __name__ == "__main__":`.
"""
from __future__ import annotations

import os
import random
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

from .constants import AIRFOILS, FUELS
from .drag import WingConcept
from .flight_sim import VehicleGeometry, run_flight
from .optimize import (Candidate, FUEL_RESERVE_MARGIN, MAX_WET_MASS_KG,
                       MAX_ACCEPTABLE_STALL_SPEED_M_PER_S, MOTOR_CUTOFF_MACH,
                       _annular_volume_m3, FUEL_VOLUME_FRACTION_OF_ANNULUS,
                       design_score)
from .mass_model import vehicle_dry_mass
from .constants import MIN_POWERED_THRUST_MARGIN_FRACTION

WING_SPAN_BOUNDS_M = (0.50, 3.00)
WING_AR_BOUNDS = (1.5, 7.0)
WING_TAPER_BOUNDS = (0.20, 1.00)
WING_SWEEP_BOUNDS_DEG = (0.0, 55.0)

SEARCH_DT_S = 0.05
FINAL_DT_S = 0.02
MAX_TIME_S = 700.0
DEFAULT_WORKERS = max(1, (os.cpu_count() or 4) - 1)


@dataclass(frozen=True)
class WingCandidate:
    span_m: float
    aspect_ratio: float
    taper_ratio: float
    sweep_deg: float
    airfoil_key: str

    def to_concept(self) -> WingConcept:
        return WingConcept(self.span_m, self.aspect_ratio, self.taper_ratio,
                           self.sweep_deg, AIRFOILS[self.airfoil_key])


@dataclass(frozen=True)
class EvaluatedWing:
    wing: WingCandidate
    feasible: bool
    max_thrust_to_weight: float | None
    min_powered_thrust_margin: float | None
    score: float | None = None
    mass_margin_kg: float | None = None


def _random_wing(rng: random.Random) -> WingCandidate:
    return WingCandidate(
        span_m=rng.uniform(*WING_SPAN_BOUNDS_M),
        aspect_ratio=rng.uniform(*WING_AR_BOUNDS),
        taper_ratio=rng.uniform(*WING_TAPER_BOUNDS),
        sweep_deg=rng.uniform(*WING_SWEEP_BOUNDS_DEG),
        airfoil_key=rng.choice(list(AIRFOILS.keys())),
    )


def wing_seed_grid() -> list[WingCandidate]:
    """Coarse full-factorial seeds -- the wing space is only 4-D continuous
    + 1 categorical and much better conditioned than the vehicle corner,
    but seeds are still cheap insurance (256 evaluations ~ seconds)."""
    import itertools

    seeds = []
    for span, ar, taper, sweep, foil in itertools.product(
        (0.70, 0.80, 1.00, 1.30), (2.0, 3.0, 4.5, 6.0), (0.35, 1.0),
        (0.0, 30.0), AIRFOILS.keys(),
    ):
        seeds.append(WingCandidate(span, ar, taper, sweep, foil))
    return seeds


def evaluate_wing(args) -> EvaluatedWing:
    vehicle, wing, dt_s = args
    geometry = VehicleGeometry(
        diameter_m=vehicle.diameter_m,
        throat_diameter_m=vehicle.throat_diameter_m,
        chamber_length_m=vehicle.chamber_length_m,
        throat_length_m=vehicle.throat_length_m,
        wingspan_m=wing.span_m,  # overridden by the concept anyway
        fuel=FUELS[vehicle.fuel_key],
    )
    tank_capacity_kg = (FUEL_VOLUME_FRACTION_OF_ANNULUS
                        * _annular_volume_m3(vehicle.diameter_m,
                                             vehicle.throat_diameter_m,
                                             vehicle.throat_length_m)
                        * FUELS[vehicle.fuel_key].density_kg_per_m3)
    result = run_flight(
        geometry, MAX_WET_MASS_KG,
        climb_angle_deg=vehicle.climb_angle_deg,
        motor_cutoff_mach=MOTOR_CUTOFF_MACH,
        dt_s=dt_s, max_time_s=MAX_TIME_S,
        wing_concept=wing.to_concept(),
        max_fuel_burn_kg=tank_capacity_kg / (1.0 + FUEL_RESERVE_MARGIN),
    )
    final = result.states[-1]
    feasible = (
        result.safe_landing
        and final.stall_speed_m_per_s <= MAX_ACCEPTABLE_STALL_SPEED_M_PER_S
        and result.min_powered_thrust_margin >= 1.0 + MIN_POWERED_THRUST_MARGIN_FRACTION
    )
    mass_margin = None
    if feasible:
        fuel_loaded = (1.0 + FUEL_RESERVE_MARGIN) * final.fuel_burned_kg
        fuel_volume = fuel_loaded / FUELS[vehicle.fuel_key].density_kg_per_m3
        annulus = _annular_volume_m3(vehicle.diameter_m, vehicle.throat_diameter_m,
                                     vehicle.throat_length_m)
        feasible = fuel_volume <= FUEL_VOLUME_FRACTION_OF_ANNULUS * annulus
    if feasible:
        # mass budget with the concept's REAL wing area (not the legacy
        # backed-out one) -- see mass_model.py
        mass = vehicle_dry_mass(
            vehicle.diameter_m, vehicle.chamber_length_m,
            vehicle.throat_diameter_m, vehicle.throat_length_m,
            wing.to_concept().reference_area_m2, fuel_loaded,
        )
        mass_margin = MAX_WET_MASS_KG - mass.dry_mass_kg - fuel_loaded
        feasible = mass_margin >= 0.0
    if not feasible:
        return EvaluatedWing(wing, False, None, None)
    peak_tw = max(st.thrust_to_weight for st in result.states)
    return EvaluatedWing(
        wing, True, peak_tw, result.min_powered_thrust_margin,
        score=design_score(vehicle, peak_tw, span_m=wing.span_m),
        mass_margin_kg=mass_margin,
    )


def _perturb(wing: WingCandidate, rng: random.Random) -> WingCandidate:
    def step(value, bounds):
        span = bounds[1] - bounds[0]
        return min(max(value + rng.gauss(0.0, 0.06 * span), bounds[0]), bounds[1])

    airfoil = wing.airfoil_key
    if rng.random() < 0.15:
        airfoil = rng.choice(list(AIRFOILS.keys()))
    return WingCandidate(
        span_m=step(wing.span_m, WING_SPAN_BOUNDS_M),
        aspect_ratio=step(wing.aspect_ratio, WING_AR_BOUNDS),
        taper_ratio=step(wing.taper_ratio, WING_TAPER_BOUNDS),
        sweep_deg=step(wing.sweep_deg, WING_SWEEP_BOUNDS_DEG),
        airfoil_key=airfoil,
    )


def optimize_wing(
    vehicle: Candidate,
    n_random: int = 6000,
    n_refine: int = 120,
    seed: int = 0,
    log=print,
    max_workers: int = DEFAULT_WORKERS,
) -> EvaluatedWing:
    """Seeded random search at coarse dt, best re-validated + hill-climbed
    at final dt. Raises RuntimeError if nothing feasible (same contract as
    optimize.optimize)."""
    rng = random.Random(seed)
    pool = wing_seed_grid() + [_random_wing(rng) for _ in range(n_random)]

    best: EvaluatedWing | None = None
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        futures = [ex.submit(evaluate_wing, (vehicle, w, SEARCH_DT_S)) for w in pool]
        done = 0
        for fut in as_completed(futures):
            ev = fut.result()
            done += 1
            if ev.feasible and (best is None or ev.score < best.score):
                best = ev
            if done % 2000 == 0:
                log(f"[wing search] {done}/{len(pool)}, best_score={best.score if best else None}")
    if best is None:
        raise RuntimeError("no feasible wing found for this vehicle")

    # re-validate + refine at final dt
    current = evaluate_wing((vehicle, best.wing, FINAL_DT_S))
    if not current.feasible:
        current = best  # coarse-dt result stands; refinement will re-check
    for i in range(n_refine):
        trial = evaluate_wing((vehicle, _perturb(current.wing, rng), FINAL_DT_S))
        if trial.feasible and (
            current.score is None or trial.score < current.score
        ):
            current = trial
    if not current.feasible:
        raise RuntimeError("wing refinement lost feasibility at final dt")
    return current
