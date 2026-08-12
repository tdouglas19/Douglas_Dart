# Playbook: simple_model optimization + sensitivity/feasibility campaign

How the 2026-08-12 overnight study was produced, written so it can be rerun
verbatim (or handed to a Claude Code session with "run the simple_model
campaign playbook"). Companion background: `docs/simple_model_overview.md`
(the model + calibration itself).

## One-shot: rerun everything

```bash
.venv/Scripts/python scripts/simple_model_overnight.py   # ~25 min
.venv/Scripts/python scripts/simple_model_report.py      # ~1 min
```

Outputs land in `out_simple_model/`:
- `campaign_cd0_*.log` — full optimizer log per CD0 level
- `summary.json` — machine-readable per-campaign verdicts + best candidates
- `thrust_vs_mach.png`, `flight_profile.png`, `altitude_vs_distance.png`,
  `fuel_mass.png` — the winning design's plots
- `cd0_sensitivity.png` — feasibility & best peak T/W vs CD0
- `tw_optimal_design.md` — the dimensional-parameter table

## What the campaign actually does (the method)

1. **The sensitivity variable**: body CD0 is the single constant that
   decides mission feasibility under calibrated physics (the pulsejet's
   level-flight Mach ceiling vs the ramjet-lightoff Mach depends on CD0
   and area fractions only — see `docs/simple_model_overview.md`). The
   family sweeps CD0 ∈ {0.30, 0.25, 0.20, 0.15, 0.10} via the env override
   `SIMPLE_MODEL_CD0_FRONTAL` (read once at import by
   `simple_model/constants.py`, so each level runs in its own subprocess —
   that is why `simple_model_overnight.py` shells out to
   `simple_model_campaign_child.py` rather than looping in-process).

2. **Each campaign** = `simple_model.optimize.optimize()` with:
   - **576 warm-start seeds** (`_corner_seeds()` in the child script): a
     grid around the known feasible corner (D 0.26–0.30 m × throat
     fraction 0.50–0.54 × chamber 0.40–0.55 m × tube 0.8–1.0 m × span
     0.70–0.80 m × climb 1–2° × all four fuels). REQUIRED: the feasible
     region occupies ~4e-6 of the 7-D search volume, so 20k random draws
     expect ~0.1 hits — pure random search returned "infeasible" 5/5
     times on designs that direct simulation proves fly. If the physics
     or constraints change enough to move the corner, re-derive seeds the
     way it was done originally: probe the binding constraints by hand
     (level-flight thrust=drag crossings per CD0, span-vs-stall-cap
     bound, climb-angle-vs-T/W bound) and grid around where they
     intersect.
   - **20,000 random candidates** (env `SIMPLE_MODEL_N_RANDOM`) at coarse
     dt, then the optimizer's own re-validation at fine dt + ~150
     hill-climb refinement rounds. Objective: minimize peak T/W subject
     to feasibility (reach M 1.1 cutoff, safe landing under the 45 m/s
     stall cap, fuel fits in the annulus).

3. **Timing** (12-core desktop): ~4.5–5 min per campaign, ~24 min for the
   family, ~1 min for the report. A single ad-hoc optimization:
   `SIMPLE_MODEL_CD0_FRONTAL=0.15 python scripts/simple_model_campaign_child.py`
   (env `SIMPLE_MODEL_N_RANDOM` / `SIMPLE_MODEL_N_REFINE` shrink it for
   smoke tests).

4. **The report** (`simple_model_report.py`) picks the lowest-peak-T/W
   *feasible* campaign from `summary.json` (or takes a CD0 argument),
   re-flies the winner at fine dt, and reuses `run_demo.py`'s plotting
   with its module globals overridden to the winner geometry.

## Interpreting results / gotchas

- "Infeasible" at CD0 ≥ 0.20 is a *physics verdict* (transition gap), not
  a search failure — but always check `summary.json`'s `error` text to
  distinguish "no feasible candidate" from a crash.
- The optimum always sits pressed against three constraint boundaries
  (throat-fraction operability limit, stall-speed cap via minimum span,
  climb-angle floor). If a rerun lands elsewhere, something changed —
  diff the constants first.
- Calibration provenance for every constant: `simple_model/constants.py`
  comments + `docs/simple_model_overview.md` + pulsejet-fp
  `architecture.md` #9–11. If pulsejet-fp is recalibrated, refit the five
  constants the same way (three-anchor log-least-squares — see the
  calibration section of the overview doc).
