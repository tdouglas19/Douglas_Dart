"""Random-search optimizer over the 6 sweep variables (vehicle diameter,
throat diameter, chamber length, throat length, wingspan, fuel type).

Run with:  python -m simple_model.optimize

IMPORTANT for any other caller: this module uses multiprocessing
(ProcessPoolExecutor) to evaluate candidates in parallel. On Windows that
means worker processes re-import whatever script called optimize() -- any
caller MUST guard the call behind `if __name__ == "__main__":`, exactly
like this module's own bottom block does, or the workers will recursively
re-run the calling script.

Objective: minimize the *peak* thrust-to-weight ratio reached anywhere
during the flight, subject to feasibility:
- reaches the motor-cutoff Mach under power
- completes a *safe* landing (flares and touches down at stall speed,
  rather than flying into the ground still gliding, or stalling out
  mid-flight)
- that stall speed is itself reasonable (<= MAX_ACCEPTABLE_STALL_SPEED_M_PER_S)

Peak T/W, not static/launch T/W: thrust in this vehicle grows a lot over
the flight (ramjet net thrust climbs steeply with Mach -- see
ramjet_simple.py), so the highest T/W moment is typically right at motor
cutoff, not at release. That peak, wherever it falls, is what actually
sets how much thrust-generating capability (throat area, chamber
pressure, structural loads) the hardware has to be built to survive --
"easiest to build" tracks the peak, not the launch value. fuel_loaded_kg
is still computed and reported for context but is no longer what gets
minimized; a lower-T/W design will generally need more fuel (a longer,
gentler climb), not less.

The stall-speed cap is not optional bookkeeping -- an early version of this
search without it found "safe" designs by shrinking wingspan toward its
lower bound, which drives the vehicle's *own* stall speed up rather than
down (confirmed by direct observation: one such "safe" landing touched
down at 161 m/s, because that candidate's own stall speed was also
161 m/s). Landing exactly at stall speed is only meaningful as "soft" when
stall speed itself is something a real landing gear could survive; without
capping it, "safe_landing" is gameable into exactly the "hundreds of miles
an hour" outcome the whole flare mechanism exists to avoid.

This is plain random search plus a short local-refinement polish, not a
gradient method -- the objective isn't differentiable in any convenient
closed form (it's "run a several-thousand-step simulation and see what
happens"), and random search over a 5-continuous + 1-categorical space is
easy to reason about and trivially handles the discrete fuel choice.
Individual candidate flights are independent of each other, so the search
is embarrassingly parallel -- evaluated across a process pool (one process
per CPU core by default) rather than run one at a time in a single
process, which is most of this module's actual "run ultra fast" speedup
(the per-flight closed-form physics itself was also optimized -- see
flight_sim.py/ramjet_simple.py/pulsejet_simple.py/drag.py -- but that is a
much smaller lever than using all the cores that were sitting idle before).
"""

from __future__ import annotations

import os
import random
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from math import pi

from .constants import FUELS, KG_PER_LB
from .flight_sim import VehicleGeometry, run_flight

MAX_WET_MASS_KG = 50.0 * KG_PER_LB
FUEL_RESERVE_MARGIN = 0.25
MOTOR_CUTOFF_MACH = 1.1

# Search bounds. throat_diameter is sampled as a *fraction* of vehicle
# diameter (not its own absolute bound) so throat < vehicle diameter is
# satisfied by construction, per the user's explicit constraint, rather
# than sampled independently and rejected/clipped after the fact.
DIAMETER_BOUNDS_M = (0.08, 0.30)
# Upper bound capped at the pulsejet operability boundary (2026-08-12):
# diameter fraction 0.54 = throat/chamber AREA fraction 0.29, the largest
# ratio the first-principles pulsejet-fp model sustains (0.43 is stone
# dead -- the previous (0.30, 0.85) bounds let the optimizer pick exactly
# such a dead engine: the 81/123 mm "T/W optimum"). Keeps every sampled
# candidate inside the operable resonator regime instead of wasting
# evaluations on designs the operability gate now zeroes anyway. Note the
# honest design tension this exposes: the ramjet WANTS a bigger shared
# throat -- the optimizer now has to trade that against pulsejet viability.
THROAT_FRACTION_BOUNDS = (0.30, 0.54)
CHAMBER_LENGTH_BOUNDS_M = (0.15, 0.70)
THROAT_LENGTH_BOUNDS_M = (0.08, 1.00)  # widened after adding the fuel-volume-fits-in-the-annulus constraint: longer throat_length gives more annular volume *and* a lower pulsejet cycle frequency (so less fuel burned per unit thrust -- see pulsejet_simple.py), so the search kept pushing against the old 0.60 m bound
WINGSPAN_BOUNDS_M = (0.50, 3.00)  # see MAX_ACCEPTABLE_STALL_SPEED_M_PER_S: below ~0.7 m, stall speed can't reach the (45 m/s) cap for this mass class regardless of anything else, so sampling well below that just wastes search budget
# Powered-climb flight path angle, per the user's explicit request to let
# the optimizer choose this rather than holding it at the old fixed 15
# degrees. There's a genuine trade-off it can search over: total_drag_n
# uses required_lift_n = m*g*cos(gamma) even during the climb, so a
# steeper angle lowers the induced-drag penalty but raises the
# weight-along-path term (m*g*sin(gamma)) working directly against thrust.
# Bounded well short of vertical (90 deg) -- the quasi-steady-lift drag
# model this module relies on throughout is not meant to cover a
# near-vertical launch. Lower bound dropped 5 -> 1 deg (2026-08-12): with
# the calibrated pulsejet thrust (T/W ~ 0.2 at release), only near-level
# acceleration can gain speed at all -- the old 5-deg floor excluded the
# entire remaining feasible corner (diagnosed: 250/250 random candidates
# failed to reach cutoff, best max-Mach 0.44, all bleeding energy into
# climb they could not afford).
CLIMB_ANGLE_BOUNDS_DEG = (1.0, 45.0)

# A real safety requirement, not just a search knob -- see this module's
# docstring for why "lands at stall speed" is only actually safe once
# stall speed itself is capped at something reasonable. 45 m/s (~100 mph)
# is NOT a light-aircraft stall speed (that would be more like 15-25 m/s) --
# it was raised here from an initial 25 m/s after direct experimentation
# showed true light-aircraft stall speeds are not reachable at all within
# practical time/distance from a Mach-1.1 cutoff by *this* mechanism (a
# fixed-CD0, no-speedbrake, no-parachute unpowered glide): even the most
# favorable geometry tested (huge 3 m wingspan, minimal-drag 0.15 m body)
# needed a glide angle so shallow (~-2 degrees) that it took 400+ seconds
# and ~30 km of unpowered glide to get down to ~40 m/s, and pushing for
# noticeably slower than that starts running into multi-hundred-second
# flight times that make the search (and arguably the mission) impractical.
# 45 m/s is still a large, real improvement over an uncontrolled ~150 m/s
# impact -- see run_demo.py's summary for exactly how it compares.
MAX_ACCEPTABLE_STALL_SPEED_M_PER_S = 45.0

# Fuel has to physically go somewhere. This vehicle is a flow-through duct
# (pulsejet/ramjet share one axial path with the throat at the aft end),
# so the natural place for a tank is the annular shell between the outer
# body (diameter_m) and that inner duct (throat_diameter_m) -- the same
# "fin can" annulus-houses-the-tank-and-avionics architecture already used
# elsewhere in this project (see the main douglas_dart package's
# ExternalShellConfig). The tank is assumed to wrap the throat/nozzle
# section specifically (length = throat_length_m), per the user's own
# framing, and gets at most half that annular volume -- the other half is
# walls, structure, plumbing, and everything else that has to share the
# space, not a sourced fraction.
FUEL_VOLUME_FRACTION_OF_ANNULUS = 0.5


def _annular_volume_m3(diameter_m: float, throat_diameter_m: float, throat_length_m: float) -> float:
    annular_area_m2 = (pi / 4.0) * (diameter_m**2 - throat_diameter_m**2)
    return annular_area_m2 * throat_length_m


# Search-phase flights run at a coarser dt (cheaper per candidate); the
# final selected design gets one re-run at the finer dt run_demo.py itself
# uses, so the published numbers aren't resolution-limited by the search.
SEARCH_DT_S = 0.05
SEARCH_MAX_TIME_S = 500.0
FINAL_DT_S = 0.02
FINAL_MAX_TIME_S = 600.0

RANDOM_SEARCH_CANDIDATES = 4000
LOCAL_REFINEMENT_ROUNDS = 150
# Fraction of each variable's bound width to perturb by during refinement.
LOCAL_REFINEMENT_STEP_FRACTION = 0.05

# One process per CPU core (minus one, so the orchestrating main process
# and the OS aren't starved) -- this is the single biggest speed lever in
# this module, since candidate evaluations are fully independent and this
# machine has cores sitting idle during a single-process search.
DEFAULT_WORKERS = max(1, (os.cpu_count() or 2) - 1)
# How many perturbations to evaluate per refinement round. Refinement is a
# sequential hill-climb by nature (each round depends on the current best),
# but batching WORKERS_PER_REFINE_ROUND independent perturbations per round
# and taking the best improvement keeps it parallel without changing the
# algorithm's character -- more neighbors explored per unit wall-clock,
# not a different search strategy.
REFINE_BATCH_SIZE = DEFAULT_WORKERS


@dataclass(frozen=True)
class Candidate:
    diameter_m: float
    throat_diameter_m: float
    chamber_length_m: float
    throat_length_m: float
    wingspan_m: float
    climb_angle_deg: float
    fuel_key: str

    def to_geometry(self) -> VehicleGeometry:
        return VehicleGeometry(
            diameter_m=self.diameter_m,
            throat_diameter_m=self.throat_diameter_m,
            chamber_length_m=self.chamber_length_m,
            throat_length_m=self.throat_length_m,
            wingspan_m=self.wingspan_m,
            fuel=FUELS[self.fuel_key],
        )


@dataclass(frozen=True)
class EvaluatedCandidate:
    candidate: Candidate
    feasible: bool
    max_thrust_to_weight: float | None
    fuel_loaded_kg: float | None
    result: object


def _random_candidate(rng: random.Random) -> Candidate:
    diameter_m = rng.uniform(*DIAMETER_BOUNDS_M)
    throat_diameter_m = diameter_m * rng.uniform(*THROAT_FRACTION_BOUNDS)
    return Candidate(
        diameter_m=diameter_m,
        throat_diameter_m=throat_diameter_m,
        chamber_length_m=rng.uniform(*CHAMBER_LENGTH_BOUNDS_M),
        throat_length_m=rng.uniform(*THROAT_LENGTH_BOUNDS_M),
        wingspan_m=rng.uniform(*WINGSPAN_BOUNDS_M),
        climb_angle_deg=rng.uniform(*CLIMB_ANGLE_BOUNDS_DEG),
        fuel_key=rng.choice(list(FUELS.keys())),
    )


def evaluate(
    candidate: Candidate,
    dt_s: float = SEARCH_DT_S,
    max_time_s: float = SEARCH_MAX_TIME_S,
    return_result: bool = True,
) -> EvaluatedCandidate:
    """`return_result=False` drops the (potentially many-thousand-state)
    FlightResult from the returned object once feasibility/objective are
    known. That matters specifically for the parallel search below: every
    candidate's result has to be pickled across a process boundary to get
    back to the main process, and most candidates are infeasible (~1-2% of
    random draws pass), so serializing their full trajectories back is
    pure overhead. The winning candidate gets one final evaluate() call
    with return_result=True (the default, for any standalone/direct use of
    this function) to reconstruct its full trajectory."""

    result = run_flight(
        candidate.to_geometry(),
        MAX_WET_MASS_KG,
        climb_angle_deg=candidate.climb_angle_deg,
        motor_cutoff_mach=MOTOR_CUTOFF_MACH,
        dt_s=dt_s,
        max_time_s=max_time_s,
    )
    final_state = result.states[-1]
    kept_result = result if return_result else None
    if not result.safe_landing or final_state.stall_speed_m_per_s > MAX_ACCEPTABLE_STALL_SPEED_M_PER_S:
        return EvaluatedCandidate(candidate, False, None, None, kept_result)

    fuel_loaded_kg = (1.0 + FUEL_RESERVE_MARGIN) * final_state.fuel_burned_kg
    fuel_volume_m3 = fuel_loaded_kg / FUELS[candidate.fuel_key].density_kg_per_m3
    annular_volume_m3 = _annular_volume_m3(candidate.diameter_m, candidate.throat_diameter_m, candidate.throat_length_m)
    if fuel_volume_m3 > FUEL_VOLUME_FRACTION_OF_ANNULUS * annular_volume_m3:
        # Reaches cutoff and lands safely, but there's nowhere to put the
        # fuel it needs -- infeasible for a real reason, not a numerical one.
        return EvaluatedCandidate(candidate, False, None, None, kept_result)

    max_thrust_to_weight = max(s.thrust_to_weight for s in result.states)
    return EvaluatedCandidate(candidate, True, max_thrust_to_weight, fuel_loaded_kg, kept_result)


def _evaluate_no_result(args: tuple) -> EvaluatedCandidate:
    """Top-level (picklable) wrapper for submitting (candidate, dt_s,
    max_time_s) tuples to a process pool -- ProcessPoolExecutor needs a
    plain function reference, not a lambda/closure, to pickle the task."""

    candidate, dt_s, max_time_s = args
    return evaluate(candidate, dt_s=dt_s, max_time_s=max_time_s, return_result=False)


def _perturb(candidate: Candidate, rng: random.Random) -> Candidate:
    def step(value: float, bounds: tuple[float, float]) -> float:
        span = bounds[1] - bounds[0]
        new_value = value + rng.uniform(-1.0, 1.0) * span * LOCAL_REFINEMENT_STEP_FRACTION
        return min(max(new_value, bounds[0]), bounds[1])

    throat_fraction_bounds_actual = THROAT_FRACTION_BOUNDS
    diameter_m = step(candidate.diameter_m, DIAMETER_BOUNDS_M)
    throat_fraction = step(candidate.throat_diameter_m / candidate.diameter_m, throat_fraction_bounds_actual)
    fuel_key = candidate.fuel_key if rng.random() > 0.1 else rng.choice(list(FUELS.keys()))
    return Candidate(
        diameter_m=diameter_m,
        throat_diameter_m=diameter_m * throat_fraction,
        chamber_length_m=step(candidate.chamber_length_m, CHAMBER_LENGTH_BOUNDS_M),
        throat_length_m=step(candidate.throat_length_m, THROAT_LENGTH_BOUNDS_M),
        wingspan_m=step(candidate.wingspan_m, WINGSPAN_BOUNDS_M),
        climb_angle_deg=step(candidate.climb_angle_deg, CLIMB_ANGLE_BOUNDS_DEG),
        fuel_key=fuel_key,
    )


def optimize(
    n_random: int = RANDOM_SEARCH_CANDIDATES,
    n_refine: int = LOCAL_REFINEMENT_ROUNDS,
    seed: int = 0,
    log=print,
    max_workers: int = DEFAULT_WORKERS,
    refine_batch_size: int = REFINE_BATCH_SIZE,
) -> EvaluatedCandidate:
    """Broad coarse-dt random search (parallel across max_workers processes)
    to find the feasible region cheaply, then re-validate the best
    candidates at the finer final dt before trusting any of them, then
    refine via batched-parallel hill-climbing -- entirely at the final dt,
    so every accepted improvement is already validated at the resolution
    that gets published, not just at the cheaper search resolution.

    That re-validation step matters, not just for tidiness: a coarse-dt
    "safe landing" can be a false positive right at the flare/landing
    boundary (confirmed by direct observation -- one search-resolution
    winner reported touchdown at ~84 m/s, barely at its own stall speed,
    but re-checking at half the dt showed it actually touches down at
    ~102 m/s, well above stall, because the coarser step size skipped
    over the precise altitude/speed crossing). Accepting a coarse-dt
    winner without re-validating would silently publish an unsafe design
    as if it were a safe one.
    """

    rng = random.Random(seed)
    candidates = [_random_candidate(rng) for _ in range(n_random)]
    tasks = [(c, SEARCH_DT_S, SEARCH_MAX_TIME_S) for c in candidates]

    feasible_candidates: list[EvaluatedCandidate] = []
    completed = 0
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_evaluate_no_result, task) for task in tasks]
        for future in as_completed(futures):
            completed += 1
            evaluated = future.result()
            if evaluated.feasible:
                feasible_candidates.append(evaluated)
            if completed % 500 == 0:
                best_so_far = min(feasible_candidates, key=lambda e: e.max_thrust_to_weight) if feasible_candidates else None
                log(
                    f"[random search, {max_workers} workers] {completed}/{n_random} candidates, "
                    f"{len(feasible_candidates)} feasible at search resolution, "
                    f"best max T/W={best_so_far.max_thrust_to_weight:.2f}" if best_so_far else
                    f"[random search, {max_workers} workers] {completed}/{n_random} candidates, 0 feasible yet"
                )

    if not feasible_candidates:
        raise RuntimeError(
            f"No feasible candidate found in {n_random} random draws -- no sampled geometry both reached "
            f"motor cutoff and completed a safe flare-to-stall-speed landing. Widen the search bounds or "
            f"increase n_random."
        )
    feasible_candidates.sort(key=lambda e: e.max_thrust_to_weight)
    log(
        f"Random search done: {len(feasible_candidates)} feasible at search resolution. "
        f"Validating top candidates at final resolution (dt={FINAL_DT_S}s)..."
    )

    verified: EvaluatedCandidate | None = None
    for checked, candidate_evaluation in enumerate(feasible_candidates, start=1):
        validated = evaluate(candidate_evaluation.candidate, dt_s=FINAL_DT_S, max_time_s=FINAL_MAX_TIME_S)
        if validated.feasible:
            verified = validated
            log(f"Verified starting point after checking {checked} candidate(s): "
                f"max T/W={verified.max_thrust_to_weight:.2f}, fuel_loaded={verified.fuel_loaded_kg / KG_PER_LB:.2f} lb")
            break
    if verified is None:
        raise RuntimeError(
            f"None of the {len(feasible_candidates)} search-resolution-feasible candidates held up when "
            f"re-validated at the finer final dt ({FINAL_DT_S}s) -- the search dt ({SEARCH_DT_S}s) was "
            f"apparently too coarse near the flare/landing boundary for every candidate found this run. "
            f"Increase n_random or lower SEARCH_DT_S."
        )

    current = verified
    n_rounds = max(1, n_refine // refine_batch_size)
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        for round_i in range(n_rounds):
            trial_candidates = [_perturb(current.candidate, rng) for _ in range(refine_batch_size)]
            tasks = [(tc, FINAL_DT_S, FINAL_MAX_TIME_S) for tc in trial_candidates]
            trials = list(executor.map(_evaluate_no_result, tasks))
            for trial in trials:
                if trial.feasible and trial.max_thrust_to_weight < current.max_thrust_to_weight:
                    current = trial
            log(
                f"[refine, final dt, batch={refine_batch_size}] round {round_i + 1}/{n_rounds}, "
                f"best max T/W={current.max_thrust_to_weight:.2f}"
            )

    # The refinement loop above ran with return_result=False throughout for
    # speed -- reconstruct the winner's full trajectory once at the end.
    current = evaluate(current.candidate, dt_s=FINAL_DT_S, max_time_s=FINAL_MAX_TIME_S, return_result=True)

    log(
        f"Local refinement done: best max T/W={current.max_thrust_to_weight:.2f} "
        f"(fuel_loaded={current.fuel_loaded_kg / KG_PER_LB:.2f} lb), validated throughout at final dt"
    )
    return current


if __name__ == "__main__":
    result = optimize()
    c = result.candidate
    print()
    print("--- Optimized design ---")
    print(f"diameter_m={c.diameter_m:.4f}  throat_diameter_m={c.throat_diameter_m:.4f}  "
          f"chamber_length_m={c.chamber_length_m:.4f}  throat_length_m={c.throat_length_m:.4f}  "
          f"wingspan_m={c.wingspan_m:.4f}  climb_angle_deg={c.climb_angle_deg:.4f}  fuel={c.fuel_key}")
    print(f"max_thrust_to_weight={result.max_thrust_to_weight:.3f}")
    print(f"fuel_loaded_kg={result.fuel_loaded_kg:.3f} ({result.fuel_loaded_kg / KG_PER_LB:.2f} lb)")
