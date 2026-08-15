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
- `propulsion.png` (thrust/Isp/SFC vs Mach) and `flight_profile.png`
  (time histories incl. fuel remaining + altitude-vs-downrange trajectory,
  background shading = flight mode) — the winning design's two plots
- `tw_optimal_design.md` — the dimensional-parameter table
  (the standalone fuel-remaining and CD0-sensitivity figures were retired
  2026-08-12 per user preference: two plots only; sensitivity verdicts
  live in `summary.json` / `overnight2_summary.json`)

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

## v3 pipeline (2026-08-13): climb-dive trajectory

```bash
python scripts/simple_model_v3_campaign.py 0.1     # ~16 min (vehicle + wing)
python scripts/simple_model_report_v3.py           # ~1 min
```

Outputs: `out_simple_model/v3_summary.json`, `v3_optimal_design.md` (with a
head-to-head table against the frozen V2), `propulsion_v3.png`,
`flight_profile_v3.png`.

**The idea.** The pulsejet→ramjet thrust deficit is a narrow *notch* at
lightoff (level acceleration +0.32 g at M 0.40, +0.21 g at M 0.44, +0.96 g
by M 0.50), not a broad valley. V3 banks the pulsejet's surplus low-Mach
thrust as altitude and spends it as gravity through that notch
(a/g = sin(dive)). The payoff is not the acceleration itself but a
**smaller engine**: the acceleration gate is what forced V2's peak T/W from
6.89 to 10.08, and the dive lets the same gate pass with a much smaller
throat. Measured result: peak T/W 6.61, body 225 mm (from 280 mm), dry mass
12.57 kg (from 14.80 kg), +1.6 kg of payload margin, same 0.26 g traverse.

**Three things worth knowing before re-tuning it:**
1. *Being low is good for the ramjet* — at M 1.05 the vehicle makes
   +3.21 g at 500 ft vs +2.65 g at 5000 ft, because ramjet thrust scales
   with density and beats the drag rise. So the post-dive drag strip wants
   to sit near the floor, and the floor is *searched* (400 ft hard minimum,
   user-set) rather than pinned.
2. *Climbing costs thrust, not just fuel* — the same engine makes +0.38 g
   at 300 m and +0.32 g at 600 m (M 0.40), so roughly a third of the dive's
   benefit is paid back as a climb tax. A lower top with a steeper dive
   beats a gentle dive from high up.
3. *The binding constraint moves.* Once the dive covers the notch, the
   worst acceleration is in the commanded climb. That is why the gate is
   measured on `min_traverse_accel_g` (notch + drag strip, excluding the
   climb): a climb is not a regime the vehicle can be stranded in — it can
   shallow out at any instant and recover 0.6–0.8 g. The climb is still
   held to "must be accelerating at all" (`min_powered_accel_g > 0`) and to
   the engine-only thrust margin.

**Split gates (do not merge them).** The acceleration gate counts gravity;
the thrust-margin gate does not (`demand = drag + max(W·sin γ, 0)`). Without
that clamp a dive shrinks "demand" and an underpowered engine reads as
healthy purely because it is pointed downhill. Both gates are what make the
dive a real gain rather than an accounting trick.

**Top-of-climb is derived, not searched** (`flight_sim.derive_top_altitude`,
two fixed passes, no convergence loop): integrate the dive across its Mach
band and take the altitude it spends. That keeps climb, dive and floor
mutually consistent and removes a search dimension.

## v2 pipeline (2026-08-12 late): margin + mass budget + composite objective + wings

`scripts/simple_model_overnight2.py` supersedes the v1 family for full
studies: per CD0 level it (1) optimizes the vehicle under ALL gates
(thrust margin >= 1.15x, minimum powered acceleration >=
MIN_POWERED_ACCELERATION_G -- env `SIMPLE_MODEL_MIN_ACCEL_G`; the default
was set from a direct achievability scan: grid-scan the corner with the
gate disabled, record each otherwise-feasible design's
min_powered_accel_g, and place the gate just below the observed ceiling
-- rerun that scan if the physics or constraint stack changes, parametric
mass budget from simple_model/mass_model.py, fuel-fits, stall cap) with
the COMPOSITE
objective (constants.py OBJECTIVE_* block: T/W + diameter + length +
span), flying a real thin-wing concept (SIMPLE_MODEL_WING_AIRFOIL) so
wing wave drag is priced in; then (2) optimizes the wing concept (span,
AR, taper, sweep, airfoil -- simple_model/wing_optimize.py) for that
winning vehicle. Results: out_simple_model/overnight2_summary.json +
o2_*.log. Report: scripts/simple_model_report.py (point it at the v2
summary). Re-weighting the objective needs only constants.py's
OBJECTIVE_WEIGHT_* + a re-run; the report table carries mass breakdown,
thrust margin, and payload/ballast slack.
