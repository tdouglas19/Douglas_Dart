# Gate 3 v2-refinement plan

Purpose: take the v2 optimized design (produced by the fast closed-form
`simple_model/` campaign) and **verify and refine it at Gate 3 fidelity**
— not re-optimize from scratch. Working mode is human-in-the-loop:
run the design, discuss the results, choose geometry changes, re-run.
No surrogate models, no automated multi-candidate search.

Source design (`out_simple_model/v2_optimal_design.md`, CD0 = 0.10):
body 234 mm, throat 125 mm (area frac 0.285), chamber/tube 362/811 mm,
overall 1874 mm, propane, climb 1.0°; wing 615 mm span, AR 2.69, taper
0.62, sweep 6°, thin cambered plate, S = 0.140 m², e = 0.841,
CLmax_eff = 1.19; dry 13.22 kg + fuel 2.28 kg, payload margin 7.17 kg;
peak T/W 6.89, min powered thrust margin 1.21 @ M 1.10; return-to-launch,
192 s, 18.4 km ground track, lands 3 m from launch, touchdown = stall =
45 m/s.

---

## Why this is not just "run it"

Gate 3 and the simple model currently describe **different vehicles on
different missions**. Five mismatches must close before a Gate 3 number
about v2 means anything:

| # | Mismatch | Task |
|---|---|---|
| 1 | Gate 3 flies straight out; v2's headline result is return-to-launch | #23 |
| 2 | Frontal-referenced CD0 = 0.10 vs Gate 3's unvalidated drag-**area** model | #18 |
| 3 | 13.22 kg dry (70-line model) vs Gate 3's separately-calibrated mass model | #19 |
| 4 | Gate 3 switches engines either/or; simple model runs them **additively** | #21 |
| 5 | v2 is propane; ramjet-fp is hardcoded Jet-A | #20 |

Plus two fidelity defects in how Gate 3 couples to the FP engines,
found while reviewing the integration (see next section).

---

## The propulsion-coupling defects (why #25 and #30 exist)

**Today:** the ramjet is evaluated at corners of a 0.1-Mach × 1500-m grid
and **bilinearly interpolated**; the pulsejet is evaluated on a
*sea-level* Mach table and scaled by **density ratio**. Neither runs at
the timestep's actual flight condition.

1. **Interpolating across the blow-off fold is unphysical.** Blending a
   lit corner (+500 N, fuel flowing) with a blown-off corner (−30 N,
   fuel cut) yields an engine that is neither lit nor dead, burning half
   the fuel. Flame stability is a fold, not a gradient.
2. **The pulsejet density-ratio scaling is known-wrong by ~3×.**
   pulsejet-fp measured −18% thrust for −5.7% density (the
   near-threshold oscillator amplifies ambient changes) and its own log
   states flight-condition queries "need the transient sim, not scaling
   corrections."

**Fix — a direct time-marched co-simulation, no tables anywhere.**
The solver progresses in time, not over a precomputed grid:

- **t = 1**: pulsejet **cold start** (slower — this is the
  initialization).
- **each step**: take the previous converged cycle-mean thrust and fuel
  flow, integrate the trajectory (dt = 0.05 s) to a new velocity and
  altitude, and feed those — plus the geometry and the previous engine
  snapshot — back into the FP engine.
- **at the handoff**: tell ramjet-fp that the previous step was a
  pulsejet and that this step is a **cold light at this velocity and
  altitude**, so the lightoff question is *answered by the model* rather
  than assumed from `minimum_lightoff_test_mach`. Then the ramjet
  continues the same warm progression.
- **to the end**: march through shutdown, glide and landing.

Both repos already expose the interface (`seed_state=` /
`return_state=True`, with `snapshot()`/`restore()` on the engine;
pulsejet-fp even shortens its first convergence check when seeded).

**Re-convergence trigger (the one implementation subtlety).** A
warm-started pulsejet convergence costs ~43 s wall. A literal FP run at
every 0.05 s timestep over a ~60 s powered phase would be 1,200 runs ≈
**14 h per mission**, so the trajectory integration step and the engine
re-convergence step are decoupled: the trajectory integrates at dt =
0.05 s, and the engine is re-converged when the condition has actually
drifted (**ΔM ≥ 0.05 or Δalt ≥ 500 m** — the validated continuation
limits), holding the last converged cycle-mean in between while keeping
the snapshot live to seed the next re-convergence.

This is not a table and not interpolation: every FP run still happens at
a real condition on the real flown path, seeded from the previous state.
It is the physically meaningful sampling rate — in the time the
condition drifts by one validated step (ΔM = 0.05 ≈ 17 m/s ≈ 0.85 s at
typical powered acceleration) the pulsejet completes **~50 full cycles**
and the ramjet's chugging **~640**, so asking for a fresh converged
cycle-mean every 0.05 s (ΔM ≈ 0.003) requests resolution finer than the
engine itself possesses.

*(Gold-standard alternative, not planned: one uninterrupted engine
transient with slowly-varying boundary conditions — no quasi-steady
assumption at all — costs ~7 h for the powered phase. Revisit only if
something suggests the quasi-steady treatment hides an effect.)*

### Why the main chain is not prefetched (#31)

Predicting the next condition from velocity and flight-path angle is
easy and accurate — but there is nothing to overlap it with. Per cycle
the FP re-convergence is ~43 s while the trajectory integration between
triggers (~0.85 s of flight, ~17 Euler steps, closed-form drag) is
microseconds, so the pipeline is ~100% FP-bound and prefetch hides
latency behind nothing. (Revisit if the trajectory side ever stops being
closed-form — e.g. Gate 5 aero tables entering the loop.)

Deeper speculation is blocked by *physics*, not engineering: step C must
be seeded from B's snapshot, and seeding from A instead is a ΔM = 0.10
jump — exactly the case measured to produce artifact blow-offs. The
coupling that makes the warm march correct is what prevents pipelining
it. Chain length is therefore physics-bounded at ~20–35 re-convergences
≈ **15–25 min per mission, irreducibly serial**.

Parallelism is spent where work is genuinely independent instead:

1. **Speculative ramjet cold light** at 2–3 predicted handoff conditions,
   launched during the pulsejet march — hides the mission's single most
   expensive query (~60–120 s, no seed) behind work already running.
2. **Both branches at true decision points** (stays lit vs flames out and
   attempts relight).
3. **Concurrent design variants per iteration** — the real throughput
   win: ~8 variants for the wall time of one.
4. **Concurrent scenarios**, and concurrent engine chains in the
   additive band.

This is a **physics fix, not just a speed trick**. An independent cold
start at each point is the *relight* branch ("can it light from scratch
here?"); marching warm-started is the *operating* branch ("it is already
running and the condition drifts"). A vehicle accelerating with its
engine lit is unambiguously the latter, and the two branches disagree
materially — at 4500 m they disagree about whether the engine runs at
all. The pulsejet density-ratio scaling also disappears, because the
engine is simply run at the real altitude.

Validated step limits (ramjet-fp continuation campaign): **ΔM ≤ 0.05**
is safe; **ΔM = 0.1 produced artifact blow-offs** (flame killed by the
step transient, not by physics); altitude steps of 500 m marched cleanly
900 → 4500 m. So step size is set by physics, not guessed for cost.
`stop_when_converged` ends each seeded run as soon as the limit cycle
returns, so cost self-adjusts to the size of the perturbation.

Measured/estimated cost: pulsejet 76 s cold → **42.8 s warm** (its own
benchmark); ramjet seeded runs converge at `t_end` 0.30 vs 0.40 cold.
Pulsejet leg (M 0.1→0.9 at ΔM 0.05) ≈ 16 points; ramjet leg
(M 0.4→1.1) ≈ 14 points; concurrent in the additive band. **~20–40 min
per mission**, vs 7–16 min today — 2–3× the compute for exact
conditions, the correct branch, and zero interpolation.

Honest costs of this choice:

1. **Path-dependence kills cross-path answer caching.** A warm-start
   result is valid only for the path that produced it (that hysteresis
   *is* the physics). Stored states can still seed a similar path,
   recovering most of the speed, but cannot be reused as answers.
2. **A geometry change re-runs the whole chain** (~20–40 min): a
   snapshot describes one specific engine, and restoring it into
   different areas would be physically inconsistent. Affordable only
   because this plan deliberately runs few candidates.
3. **The march is sequential** — no parallelism along the path.
   Parallelism survives across scenarios (nominal/adverse) and across
   the two engines.

Edge cases the marcher must handle: cold first point (ramjet with
pilot); **blow-off as a real event** — the engine stays out until a
relight-branch attempt succeeds, which the corridor map says is
impossible inside the no-go band, so Gate 3 gains the ability to model
"flame lost in the dive, no relight until M 1.1"; two concurrent chains
in the additive band with separate fuel ledgers; descent and
deceleration marched the same way.

---

## Phases and ordering

### Phase 1 — Make the modeling real (blocks any v2 verdict)
- **#17 Translate v2 → `configs/v2_seed.yaml`** — start here; everything
  depends on it. Document every mapping assumption inline.
- **#18 Drag reconciliation** — highest risk to the verdict (see Open
  questions).
- **#19 Mass reconciliation** — line-by-line against v2's breakdown.
- **#20 Propane through both FP bridges** — small; ramjet-fp's
  `fuel_air` is already generalized.
- **#21 Additive combined-cycle** — sum both engines through transition,
  separate fuel ledgers, report crossover Mach.
- **#22 Full internal inlet flowpath (ramjet-fp)** — largest single item;
  capture/lip area, internal contraction + throat, subsonic diffuser
  schedule with divergence/separation limit, shared-duct selector. Runs
  in parallel with the Douglas Dart-side work.

### Phase 2 — Propulsion-coupling fidelity
- **#25 Co-march the FP engines along the trajectory, warm-started**
  (replaces grid interpolation; subsumes the pulsejet altitude fix,
  since the engine is now run at the real altitude)

### Phase 2b — Speed levers (none of them trade away physics)

Target: a mission from ~15–25 min down to ~2–5 min, making the
run-discuss-tweak-rerun loop genuinely interactive. Measure, don't
assume, but the levers are independent:

- **#32 Carry φ forward (~4×).** Since φ became an output, every query
  is an operating point = 4–9 transients of mixture bisection. Required
  φ varies slowly and smoothly along a trajectory (0.93 at M 0.4 → flat
  1.00 from M 0.6), so carry the previous φ, verify with one run, and
  re-search only on failure or near the lean boundary. Also more
  physical: a fuel controller follows a schedule, it does not re-derive
  one every cycle.
- **#35 March fast, confirm full (~4×, free).** Cost scales ~N²; the
  grid study measured ±1% cycle-mean thrust across N = 162…486. March at
  N=162, re-run the accepted design at full fidelity. No design is
  accepted on fast-fidelity numbers alone.
- **#33 Ramp the freestream condition — BUILT, kept as a safety margin
  rather than a speed lever (user decision, 2026-08-12).** Ramping the
  condition within the run removes the step transient entirely. Measured
  at M 0.6 → 0.9 in one ΔM = 0.3 step: step-jumped gives −40.9 N with
  the flame **blown off**, ramped gives **+805 N** stable and reproduces
  a six-step ΔM=0.05 chain within the grid band.

  **We nonetheless keep ΔM ≤ 0.05 / Δalt ≤ 500 m.** The big step is
  validated at exactly one condition pair (sea level, φ=1.0, well away
  from the pinch); the flame is most fragile precisely where we have not
  tested it — inside the M 0.6–0.9 oscillation pinch and near blow-off
  boundaries. Forfeiting the ~6× buys a march whose every step sits
  inside the validated envelope, with the ramp now removing the
  transient even at small steps. Cost of the choice: ~18 re-convergences
  over M 0.4→1.3 instead of ~3, i.e. **~10–15 min per mission** rather
  than low single digits.
- **#34 Persistent snapshot library.** Seed each step from the nearest
  stored converged state for this engine geometry. Pays from iteration 2
  on, especially when only non-engine geometry changed (wing): the
  engine is identical and the new path passes near old points, so nearly
  every step starts warm despite a different trajectory.

Deliberately deferred: adaptive step sizing (wait for real
thrust-vs-Mach sensitivity data so the adaptation is informed) and
further solver JIT work (ramjet is already 0.83 ms/step after a 3.4×
pass — diminishing next to the above).

### Phase 3 — Mission parity
- **#23 Port return-to-launch** (half-loop, glide home, spiral, flare,
  touchdown at ~stall); keep straight-out for regression.
- **#24 v2-equivalent metrics** — ground track, landing distance from
  launch, touchdown speed vs stall, flight time, peak T/W, min powered
  thrust margin + its Mach.

### Phase 4 — The iterate loop
- **#26 Run the v2 seed, produce the side-by-side report** vs the v2
  table, every disagreement traced to a mechanism → discuss → choose
  geometry changes → re-run. ~20–60 min compute per iteration once the
  cache is warm.

### Phase 5 — Detail deliverables (parallel with Phase 4)
- **#29 2D cross-section** — pull EARLY if packaging is a live worry; it
  is a real packaging check, not just a drawing.
- **#27 Reed valve geometry/count** as config-driven inputs and reported
  outputs (pulsejet-fp already has the beam-theory physics).
- **#28 Fuel consumption plots** — mission-integrated histories,
  per-phase budgets, required-φ schedule.

**Deliberately excluded:** the ramjet shock cone. Normal-shock recovery
at M 1.10 is 0.9989 — a cone buys ~0.1%. Revisit only if peak Mach
rises above ~1.5. (Internal inlet geometry is still in scope, #22 — that
is a different question from shock structure.)

---

## Rough effort

| Phase | Work | Compute |
|---|---|---|
| 1 | ~2–3 sessions (#22 dominates) | small |
| 2 | ~1 session | small |
| 3 | ~1 session | small |
| 4 | per-iteration discussion | 20–60 min per run |
| 5 | ~1.5 sessions | small |

First v2 verdict: roughly 4–5 working sessions plus modest compute.

---

## Open questions (need answers before or during Phase 1)

1. **CD0 reconciliation risk.** v2 closes at CD0 = 0.10 against its own
   stated feasibility boundary of 0.125–0.15. If #18 lands the real
   drag above ~0.125, the design does not close and that becomes the
   first design discussion — surfacing it early beats finding it after
   several refinement iterations.
2. **Touchdown = stall = 45 m/s is a zero-margin condition.** Is that a
   real requirement or an artifact of the simple model's landing logic?
   Every Gate 3 run to date also fails a *separate*,
   propulsion-independent stall-margin check (−37.7%).
3. **Ramjet mixture.** The FP model requires φ ≈ 1.0, not the 0.60 the
   configs assume — ~1.6× the fuel per kg of air. v2's fuel budget was
   sized under the old assumption.
4. **Continuation step size** — ΔM ≤ 0.05 and Δalt ≤ 500 m are the
   validated safe limits; smaller costs more runs but each converges
   faster, so the cost curve is flatter than it looks. Worth a short
   sensitivity check (ΔM 0.05 vs 0.025) on the first real mission.
5. **Relight policy after an in-flight blow-off** — how many relight
   attempts, and on what schedule, before the mission is declared
   failed? This is now a modelable mission-rule decision rather than an
   assumption.

---

## Standing constraints for this work

- Every reported number comes from a first-principles run; no fitted
  correlations in the deliverable path.
- Test invocation (CLAUDE.md): kill-switches on the command line —
  `unittest discover` does not load `tests/conftest.py`.
- Preserve the old candidate as a regression case when a model
  correction changes the selected design (docs/design_workflow.md).
