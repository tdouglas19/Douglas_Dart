# Local design-closure search

`src/douglas_dart/optimizer.py` runs a standalone differential-evolution search over
the coupled design variables against `trajectory.py`'s nominal and adverse
scenarios. It calls no AI model — start it and let it run on your own machine; it
checkpoints after every generation so you can watch it, kill it, or resume from the
CSV log without losing progress or spending any tokens.

```bash
douglas-dart design-optimize \
  --config configs/shared_nozzle_candidate_b.yaml \
  --population 24 --generations 40 --seed 0 \
  --output-dir results/generated/design_optimize
```

Each generation prints a one-line progress summary and rewrites
`results/generated/design_optimize/checkpoint.json` (full generation history) and
`generation_log.csv` (best-per-generation, spreadsheet-friendly). At ~1-2 seconds per
candidate evaluation, population 24 x generations 40 (≈1,900 evaluations including
mutants) takes on the order of an hour on a single core — run it in the background.

## What it searches

Eight coupled variables (`DesignVariableBounds` in `optimizer.py`): body diameter,
throat diameter, exit/throat area ratio, loaded fuel mass, the pulsejet/ramjet fuel
split, and the climb/dive/dive-entry-Mach schedule. Each candidate becomes a real
`ReferenceCase` via `apply_design_variables` and is scored by actually running
`simulate_mission` at nominal and adverse multipliers — not a surrogate model.

## What it does not search yet

- **Selector/pulsejet cycle parameters** (chamber volume, discharge coefficient,
  target equivalence ratio, burn duration) and **sled release speed** are not search
  variables. A smoke run at the Candidate B baseline found the dominant blocker is a
  pulsejet net-thrust trough near Mach 0.1 that none of the current eight variables
  can route around — the fix likely lives in the selector/chamber parameters or the
  release condition, not body/throat/fuel/angle geometry. Adding those as search
  variables is the natural next step once this is confirmed with the user.
- **Structural mass vs. body diameter** is not modeled — growing the body only
  changes drag area and nozzle flow capacity here, not skin/structure mass. Treat any
  winning candidate with a notably different body diameter as a drag/flow-only
  result pending a real parametric mass budget.
- **Boom Supersonic Prize rule compliance** (no altitude loss from Mach 0.8 to past
  Mach 1) is enforced inside `trajectory.py` itself and penalized in the score
  (`_RULE_VIOLATION_PENALTY`), so the search cannot "cheat" by diving during the
  regulated regime.

## Scoring

Every weight is a named module-level constant in `optimizer.py`
(`_REACHED_PEAK_MACH_REWARD`, `_MISSED_PEAK_MACH_PENALTY`, etc.) — same discipline as
the visible weights in `robustness_candidate_b.yaml`. The adverse scenario is
weighted 3x nominal because the goal is a design that closes off-nominal, not one
that only closes in the best case.

## After a run

The search runs at reduced pulsejet-table fidelity for speed (`evaluate_design`'s
`fast_*` defaults). Re-run the winning `best_variables` through
`douglas-dart mission-trajectory` and `douglas-dart jsbsim-build --check` at full
fidelity before treating a result as anything more than a search hint.
