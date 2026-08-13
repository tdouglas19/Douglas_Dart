# Candidate B static convergence

## Decision

Carry **Candidate B** as the next geometry and model-integration point:

- 210 mm outer body and 2.30 m length;
- fixed 195 mm circular intake with one-half available to the selected mode;
- 170 mm shared throat (revised from 160 mm -- see "Model correction that moved the
  throat from 160 to 170 mm" below) and `Ae/At = 1.05`;
- representative Jet-A fuel; and
- Mach 1.10 at 4,500 m MSL as the current static analysis point.

Candidate B is selected by the visible score in
`configs/robustness_candidate_b.yaml` after passing the named nominal and
conservative static screens. It is not a closed mission or a hardware recommendation.

**Gate 1 (Level 0 hand-calc bounds, new) does not pass on this candidate.**
`python -m douglas_dart level0-bounds` finds the configured 0.0896 m&sup2;
lifting-surface reference area is roughly 3.6x too small to fly at the
configured 39-42 m/s sled-release speed at 25 kg maximum takeoff mass and the
configured CL_max = 0.90 -- a stall-speed bound of ~73.6 m/s. The pulsejet
chamber also fails packaging even at the loosest possible bound (needs
&ge;722 mm length at full body cross-section vs. a 180 mm forebody). Neither
finding was visible from the static propulsion/drag screens below, which
never checked lift-available-vs.-required. See
`docs/level0_feasibility_bounds.md` for the full result and
`docs/design_workflow.md` for how this gate fits the overall convergence
process. This does not retract Candidate B's static propulsion closure below
-- it adds a previously-missing check that the static screens don't cover.

**Gate 3's new stall-margin check confirms the same finding at the full
mission-integration level, not just the static Level 0 bound.**
`python -m douglas_dart mission-trajectory` now reports
`minimum_stall_margin_fraction = -0.377` nominal (`stall_margin_violated =
true`) -- the vehicle spends time below its own 1g stall speed during the
simulated mission. This is the expected, consistent consequence of the same
undersized reference area Gate 1 found, surfaced independently by a second,
higher-fidelity model rather than contradicted by it. `mass_model.py`,
`body_length_m`, and now `wing_area_scale_factor` are real search variables
in `optimizer.py` (`design-optimize`); the latter directly grows
lifting-surface reference area (at a structural-mass cost) and is the
mechanism that can actually close this gap -- rerun `design-optimize` and
check the winning candidate's `wing_area_scale_factor` and resulting
`reference_area_m2` against the Gate 1 bound before declaring it closed. See
`docs/design_workflow.md`'s Level 3 row.

## `design-optimize` runs with `wing_area_scale_factor` (2026-08-07)

Two 150-200 generation, population-50 unattended searches this session,
against `shared_nozzle_candidate_b.yaml`, `configs/robustness_candidate_b.yaml`
mass calibration, all 11 `optimizer.py` design variables:

- **Run 1** found and exposed a scoring-function bug: below Mach 1, every
  score term was either boolean (reached-target/duration, identical whether
  a scenario stalls at Mach 0.1 or Mach 0.9) or zero (time-above-Mach-one),
  so the search had no gradient telling it that giving up on adverse-scenario
  acceleration was bad -- only that doing so freed up mass margin, which
  scores continuously. The adverse-scenario peak Mach collapsed from ~0.80 to
  ~0.13 between generation 22 and 39 while the reported best score kept
  improving. Fixed in `optimizer.py` by adding
  `_PEAK_MACH_PROGRESS_REWARD_PER_MACH`, continuous credit for however far a
  scenario actually got, sized to outweigh a plausible few-kg mass-margin
  trade.
- **Run 2** (corrected objective) converged cleanly and held: best candidate
  reaches nominal peak Mach 1.102 and meets the nominal duration requirement,
  but adverse peak Mach plateaus at **0.802** and does not meet duration --
  **the design does not close under the adverse scenario** at this search's
  current variable bounds. Six of eleven variables were pinned at a bound in
  the winning vector: `body_diameter_m` (195 mm, the fixed-intake floor --
  a real physical limit), `loaded_fuel_mass_kg` (6.00 kg, the old ceiling),
  `ramjet_fuel_fraction` (0.80, the old ceiling), `climb_angle_deg` (15 deg,
  the old ceiling), `sled_release_speed_m_per_s` (121.2 m/s, the placeholder
  rail/acceleration ceiling), and `wing_area_scale_factor` (0.50, the floor).
  A DE run pinned at a bound usually means the true optimum sits outside the
  box, not that the bound is a real constraint -- so the three ceilings with
  no physical basis (`loaded_fuel_mass_kg`, `ramjet_fuel_fraction`,
  `climb_angle_deg`) were widened in `optimizer.py`'s `DEFAULT_BOUNDS` (see
  its inline comments for the evidence and the physical constraint each new
  ceiling actually ties to -- MTOM margin, the fraction's own 1.0 bound, and
  headroom against no documented structural cap, respectively). A third,
  corrected+widened search is running to see whether that closes the gap or
  whether the adverse scenario's thrust/drag/mass derates (`trajectory.py`'s
  `ADVERSE_SCENARIO`: 0.75x thrust, 1.20x drag, 0.82 ramjet recovery, +2.90
  kg mass) are the actual binding constraint regardless of geometry.
- **Independent confirmation the winning candidate is not yet buildable
  regardless of the Mach question**: running `feasibility.py`'s
  `evaluate_level0_feasibility` on Run 2's best candidate shows the selector
  no longer fits within the shrunk 195 mm body diameter (`selector+allowance
  205 mm vs body 195 mm`) and the pulsejet chamber packaging failure from
  Gate 1 above persists (now 837 mm needed vs. 180 mm forebody, worse than
  the original 722 mm because the search shrank body diameter to its floor).
  `optimizer.py`'s `evaluate_design` does not currently call Level 0
  packaging checks at all -- it only rejects a candidate if `apply_design_
  variables`/`simulate_mission` raise. A design that scores well by this
  search's objective can still fail Level 0 packaging silently; treat every
  `design-optimize` result as requiring a `level0-bounds` check before
  trusting it, per `docs/design_workflow.md`'s closing rule for every gate.
  **Fixed 2026-08-07**: `evaluate_design` now calls `feasibility.py`'s
  `packaging_bounds` (pure geometry, no simulation, negligible added cost)
  and applies `_PACKAGING_VIOLATION_PENALTY` (2000, same order as
  `_RULE_VIOLATION_PENALTY`) per failing check, once per candidate. The
  search can no longer score a packaging-impossible candidate as if it were
  free; `CandidateEvaluation.packaging_failures`/`packaging_failure_names`
  make this visible in every checkpoint/CSV row instead of requiring a
  separate manual `level0-bounds` check. Note this does not retroactively
  fix Runs 1-4 above, which all predate this change -- their winning
  candidates should be re-screened before being trusted.
- **Run 3** (widened `loaded_fuel_mass_kg`/`ramjet_fuel_fraction`/
  `climb_angle_deg`) did not close the gap either -- adverse peak Mach
  0.620, slightly *worse* than Run 2's 0.802, while the overall score still
  improved (nominal's `time_above_mach_one_s` term grew enough to outweigh
  it). `climb_angle_deg` was still pinned at its new 20 deg ceiling (19.98).
  **Root cause, isolated by re-running Run 3's winning candidate under each
  adverse derate individually healed back to nominal** (`trajectory.py`'s
  `minimum_lightoff_test_mach = 0.8`): adverse peak Mach never reaches 0.8,
  i.e. **the vehicle never reaches ramjet ignition in the adverse
  scenario at all** -- the entire shortfall happens inside the pulsejet
  climb/accel/dive phases. Healing thrust alone (0.75x -> 1.0x) recovers
  most of the gap (0.620 -> 0.742, still short of 0.8); healing drag or mass
  growth barely move it (+0.014, +0.037); **healing ramjet total-pressure
  recovery changes nothing at all** (0.620 -> 0.620, exactly) -- confirming
  ramjet is never active in this scenario to be affected by it. This means
  `throat_diameter_m`, `exit_to_throat_area_ratio`, and
  `ramjet_fuel_fraction` -- three of `optimizer.py`'s eleven search
  variables -- are structurally irrelevant to closing the adverse gap
  specifically (they still matter for the nominal case, which does reach
  ramjet); widening their bounds, as Run 3 did for `ramjet_fuel_fraction`,
  could not and did not help. The actual lever is pulsejet-phase
  performance/climb profile. `climb_angle_deg`'s ceiling was raised again
  (20 -> 30 deg) on the strength of this and its own bound-saturation
  signal -- see `optimizer.py`'s `DEFAULT_BOUNDS` inline comment for the
  mechanism (a steeper climb reaches `top_of_climb_m` in less time, spending
  less of the climb phase fighting `mass*g*sin(gamma)` before leveling off
  into the more efficient `pulsejet_accel` phase).
- **Speedup (2026-08-07, unrelated to the above search results but changes
  their wall-clock cost):** `propulsion_map.py`'s pulsejet simulation is now
  memoized -- it was being re-run from scratch, byte-identically, once per
  mission scenario per candidate (the scenario thrust multiplier is applied
  only *after* the simulation, never inside it). Measured ~2x reduction in
  per-candidate `evaluate_design` cost at the search's default fast-fidelity
  settings. No calculation or output changed (121/121 tests unchanged); see
  the commit for the full before/after measurement.
- **Run 4** (250 generations, sped-up code, same bounds as Run 3 plus
  `climb_angle_deg` raised to 30) confirms the plateau is real, not a search-
  budget problem: adverse peak Mach 0.596, statistically the same as (if not
  slightly worse than) Runs 2 and 3, and `climb_angle_deg` pinned again at
  its new ceiling (29.96) with no further score improvement from the extra
  room -- i.e. climbing steeper stopped helping well before 30 deg. Four
  searches (200-250 generations, populations 50-70, three different bound
  sets) have now converged to the same ~0.6-0.8 adverse-Mach band. Further
  identical-shape searches over the current 11 variables are not expected to
  close this gap.
- **The one remaining physically-real, untapped lever is not currently a
  usable search variable**: `case.pulsejet.chamber_volume_m3` (pulsejet
  chamber size) directly sets captured-mass-flow/cycle thrust, and the
  entire adverse shortfall is a pulsejet-phase thrust-margin problem per the
  isolation above -- but `mass_model.py` does not reference
  `chamber_volume_m3` anywhere. `configs/robustness_candidate_b.yaml`'s
  `shared_engine_body_combustor_nozzle` component (the only budget line that
  would cover it) is a single `conceptual_allocation` lump already
  calibrated against throat area alone, with no separate chamber-volume
  breakdown to split from. Making chamber volume a search variable today
  would let the optimizer grow it for free thrust with zero mass penalty --
  exactly the "exploit missing physics" failure mode
  `docs/design_workflow.md`'s Optimizer rules warn against. Closing this
  requires a real calibration decision (how chamber-wall mass should scale
  with volume, sourced or explicitly flagged provisional per
  `docs/assumptions_registry.md`), not a bound change -- deliberately left
  undone rather than inventing an unaudited coefficient.

## Root-cause investigation: why adverse actually stalls (2026-08-07)

Run 5 (300 generations, packaging-scored objective) converged with the same
character as Runs 2-4: adverse peak Mach 0.592, `ramjet_fuel_fraction`
pinned near 0.89. This section traces the actual mechanism, replacing the
"pulsejet-phase thrust-margin problem" framing above with a more precise
one -- the previous framing was correct that the shortfall is confined to
the pulsejet phase, but incomplete about *why*.

**Is `minimum_lightoff_test_mach` (0.8) a hard limit, or can ramjet ignite
earlier?** `config.py`'s `RamjetConfig` docstring already answers this: it
is an explicit "open trade variable," not a value sourced from the
switchable-engine patent lineage. It is freely adjustable. But adjusting it
does not help, for two independent reasons found below.

**Reason 1 -- ramjet has negative net-thrust margin under adverse
conditions at every Mach tested, not just below 0.8.** Sweeping
`evaluate_ramjet` for Run 4's winning candidate from Mach 0.5 to 1.1 at
4500 m under the adverse scenario (0.75x thrust, 1.20x drag, 0.82 recovery)
shows net thrust below drag at *every* point -- the deficit *worsens* with
Mach (-48 N at 0.5, -248 N at 1.1). The same sweep under nominal conditions
(1.0x thrust, 1.0x drag) turns net-thrust-positive around Mach 0.65-0.70 and
stays positive through 1.10, matching why nominal closes. **Igniting ramjet
earlier would not help -- it would hand off from a still-accelerating
pulsejet phase to a mode with negative net thrust, making things worse, not
better.** Growing the nozzle throat *does* close much of this margin
(surplus improves from -290 N to -62 N at Mach 1.1 going from throat 0.190
to 0.220 m, paired with a wider body to keep packaging and captured-air
spillage consistent) -- worth revisiting once the pulsejet-phase problem
below is fixed, since throat_diameter_m's current 0.190 m ceiling was never
approached by the search (it optimizes primarily for nominal, since adverse
never reaches ramjet mode to put pressure on this dimension at all).

**Reason 2 -- the actual failure has nothing to do with ramjet margin: the
pulsejet climb phase runs out of fuel first.** Instrumenting Run 4's winner
directly (`simulate_mission` phase log) shows the adverse mission ends with
`pulsejet_fuel_exhausted_during_climb` at t=14.3 s, altitude 2039 m (vs. a
6000 m top-of-climb target), Mach 0.596 -- consistent with the observed
plateau to the digit. The candidate's `ramjet_fuel_fraction=0.89` leaves
only ~0.97 kg of the 9.0 kg loaded fuel for the pulsejet phase; at the
observed ~0.086+ kg/s pulsejet fuel flow this is exhausted in seconds.
Including the climb's gravity term (`mass*g*sin(gamma)`, non-trivial at
this candidate's 30 deg climb angle -- about 130 N against a ~26 kg adverse
mass) alongside thrust and drag shows **net climb-phase acceleration stays
positive all the way to about Mach 0.75-0.8** -- i.e. thrust margin was
never the constraint during the climb; fuel supply was.

**Confirmed by direct test: rebalancing the fuel split closes most of the
gap for free.** Re-running the same candidate with `ramjet_fuel_fraction`
lowered from 0.89 to 0.6 (pulsejet fuel: 0.97 kg -> 3.6 kg) takes adverse
peak Mach from 0.595 to *exactly* 0.802 -- it crosses `minimum_lightoff_test_mach`
-- while nominal's own peak Mach and closure are completely unaffected
(1.1009, still meets duration). **This is a real, available improvement the
search was not finding.**

**Why the search wasn't finding it -- a genuine scoring gap, now fixed.**
Scoring both candidates through the pre-fix `evaluate_design` showed the
*worse* (0.89) candidate scoring higher: -6915 vs. -7134. Instrumenting the
difference: nominal's `time_above_mach_one_s` fell from 154.9 s to 92.5 s
(less ramjet fuel shortens its Mach-1.10 hold) at
`_TIME_ABOVE_MACH_ONE_REWARD_PER_S=5.0` x weight 1.0 = a 312-point loss;
adverse's `_PEAK_MACH_PROGRESS_REWARD_PER_MACH` term gained only ~93 points
(weight already applied) for reaching 0.802 instead of 0.595 -- nowhere
near enough to compensate, because crossing into ramjet range carried no
reward of its own, only the same smooth per-Mach credit as any other
progress. Added `_REACHED_RAMJET_IGNITION_REWARD=300` (weighted to 900 for
adverse), a discrete bonus for a scenario's peak Mach crossing
`minimum_lightoff_test_mach`, sized from this exact measured trade-off with
margin. Re-scored: 0.6 now beats 0.89 (-5934 vs -6615), as it should.
Regression test: `test_evaluate_design_rewards_crossing_into_ramjet_range_in_adverse`.

**What this does and does not close.** This fixes the search's *incentive*
to find a better fuel split; it does not by itself guarantee the search
converges there over the noisy 11-dimensional DE landscape, and it does not
fix Reason 1 above (ramjet's negative adverse margin once ignition is
reached) -- a candidate that now reaches Mach 0.8 in adverse will likely
still stall shortly after entering ramjet mode until nozzle/throat sizing
is revisited too (see `ramjet_net_thrust_nonpositive_during_acceleration`
in the rebalanced candidate's own status). Both fixes are necessary; neither
alone closes adverse.

## Dive-mechanic audit and the ramjet lightoff threshold as a 12th search variable

Following directly from the above: is the climb-to-altitude / dive /
gravity-assisted-acceleration / pull-out / level-flight-ramjet architecture
actually implemented, and is it being used well?

**Yes, it's implemented and gets exercised once fuel-starvation (above) is
fixed.** With `ramjet_fuel_fraction=0.6`, the rebalanced adverse mission
climbs at 30 deg to the 6000 m top-of-climb gate (Mach 0.762), levels off
briefly, dives at the candidate's shallow -3 deg to cross
`minimum_lightoff_test_mach`, and hands off to ramjet -- exactly the
described sequence. But the dive itself did almost no work here: only 20 m
of altitude loss, because `dive_angle_deg=-3` was shallow and
`dive_entry_mach=0.78` left almost no room before the 0.80 gate.

**Extending the dive does not help, and that is correct, not a gap.**
Testing a much higher top-of-climb altitude (6000 -> 12000 m, not currently
a search variable) combined with a much steeper dive (-3 -> -20 deg) barely
moved peak Mach (0.8016 -> 0.8026) -- because `trajectory.py`'s "dive"
phase already forces `gamma_deg = 0` (no further altitude loss) the instant
`mach >= transonic_regime_start_mach` (0.8), which is `docs/assumptions.md`'s
sourced Boom Supersonic Prize rule ("level or climbing only from Mach
0.8+"), not an arbitrary internal choice. The dive can only build speed
*up to* that gate, never past it -- diving further to gain more speed
would violate the competition's own rule. `top_of_climb_altitude_min/max_msl_m`
is correctly left unsearched; there is no available gain there.

**The real lever hiding in this question turned out to be which Mach the
gate itself sits at.** Since the dive can only carry the vehicle up to
`minimum_lightoff_test_mach`, and (per the root-cause section above)
pulsejet's own thrust margin outlasts ramjet's, *raising* that gate lets
the pulsejet-driven portion of the flight (climb + accel + dive combined)
carry the vehicle further before an inferior propulsion mode takes over.
Made it a 12th `optimizer.py` search variable
(`minimum_lightoff_test_mach`, bounds 0.50-1.00, kept below the fixed
`minimum_self_sustaining_mach=1.10` with margin per `RamjetConfig`'s own
validation). Direct measurement: raising it from 0.80 to 0.87 alone (same
candidate, `ramjet_fuel_fraction=0.6`) takes adverse peak Mach from 0.802
to 0.867, plateauing exactly where the pulsejet-margin crossover predicts.
Regression tests: `test_apply_design_variables_wires_minimum_lightoff_test_mach`,
`test_raising_lightoff_threshold_lets_pulsejet_close_more_of_the_adverse_gap`.

## Grounding the adverse-scenario multipliers (2026-08-07)

`trajectory.py`'s `ADVERSE_SCENARIO` (0.75x thrust, 1.20x drag, 0.82 ramjet
recovery, +2.90 kg mass) has always been filed as a "provisional
assumption, explicitly non-probabilistic" (`docs/assumptions.md`: "these
are engineering screens, not probability statements"). This section checks
each number against published aerospace conceptual-design margin
literature where a comparison is possible. **No values were changed as a
result** -- see "What this does and does not settle" below for why.

**Mass growth (+2.90 kg): benchmarked, and the current value looks
*low*, not high.** The AIAA/SAWE mass-growth-allowance convention
(ANSI/AIAA S-120A-2015, summarized in NASA/SAWE mass-properties-control
literature) commonly cites roughly 15% growth allowance at "Design"
maturity -- the conceptual-design stage this vehicle is at -- rising toward
30%+ for the least mature technology categories. +2.90 kg against this
vehicle's ~21-26 kg mass range is roughly 11-14%, *below* even the more
conservative 15% figure. If anything, literature convention suggests the
adverse mass-growth term is a bit optimistic, not pessimistic -- the
opposite of what would make closure easier.

**Drag (1.20x): directionally plausible, not a precise sourced match.**
Published CFD-versus-experiment comparisons cite combined drag error bands
around 7% for well-resolved grids near buffet onset -- but that is for
CFD, a fidelity level well above this codebase's own drag model, which is
explicitly documented (`drag.py`'s module docstring) as "a budget/proxy,
not a validated aerodynamic prediction," anchored at one calibration point
with a literature-typical (not measured) transonic-rise shape. Early-stage,
semi-empirical drag build-up methods are understood in the literature to
carry meaningfully wider uncertainty than resolved CFD, which is
consistent with (though does not precisely derive) a 20% margin. No single
authoritative percentage for *this specific class* of model was found.

**Thrust (0.75x): no comparable literature benchmark found at all.** The
closest results found were in-flight thrust *measurement* accuracy figures
for instrumented, already-built engines (on the order of 1-4%) -- a
fundamentally different question (post-hardware measurement precision) from
pre-hardware design uncertainty for a novel, switchable pulsejet/ramjet
architecture with no comparable production baseline to benchmark against.
This number remains exactly as unsourced as `docs/assumptions.md` already
says it is.

**What this does and does not settle.** This is a literature comparison,
not a validation -- none of these margins are being claimed as now
"sourced" in `docs/assumptions_registry.md`'s taxonomy sense (a specific
citable number for *this* vehicle class). The mass-growth finding is the
one actionable result: it argues for *raising* +2.90 kg toward the ~15%
convention, which would make the adverse scenario measurably harder to
close, not easier -- the opposite direction from what searching for
"grounding" might have been hoped to produce. Per this session's own stated
principle (propose grounded values with citations for review rather than
silently changing them), `ADVERSE_SCENARIO`'s literal values in
`trajectory.py` were left unchanged; this is a documented recommendation
for deliberate review, not an applied fix.

## Pulsejet low-Mach thrust: widened table, side inlet, and a caught bug (2026-08-07)

Three changes, investigated together because each affects how the others
should be read.

**1. Widened `_PULSEJET_TABLE_MACH_VALUES` from 6 points (0.0-0.50) to 11
(0.0-1.00).** The table previously stopped at Mach 0.50 with no documented
technical reason; `_interp_table` clamped to that last entry above it
rather than re-simulating, so every pulsejet-thrust claim above Mach 0.5
in this codebase's history was extrapolation of a value only ever actually
computed at 0.50. `jsbsim_model.py` had an independent, identically-stale
copy of the same 6-point grid; consolidated to import trajectory.py's
single constant instead of drifting out of sync again.

**2. Set `inlet_type: side` in all three active configs** (previously
defaulting to `"straight"`, per user direction on the real vehicle's
intake geometry).

**3. Caught and fixed a real bug this investigation's own first pass
introduced into the documentation, not into the simulator.** Sweeping the
newly-widened table with the side inlet showed net thrust jumping from
near-zero below Mach ~0.3-0.35 to 100+ N above it -- initially documented
(in this file, in `trajectory.py`'s docstring, and in three config
comments) as pulsejet genuinely producing **zero** thrust below that Mach,
a "true physical zero." **This was wrong**, caught directly by a user
challenge ("pulsejets can provide thrust with zero forward velocity") that
prompted re-checking rather than accepting the first measurement. Stepping
`PulsejetSimulator` for 2 full seconds at Mach 0 (not the usual short
warmup/measurement window) shows the engine *does* keep firing --
ignitions at t=0.001, 0.585, 1.174, 1.767 s, a real cycle period of ~0.585 s
(~1.7 Hz) versus the ~0.014 s (~71 Hz) the engine is tuned for at speed.
Mechanism: refill at low Mach is driven only by the small (inlet total
pressure - blown-down chamber pressure) differential instead of ram
pressure, so it takes far longer to accumulate the fresh-air fraction
`_ignite_if_ready` requires before it can re-arm. The fixed 0.10-0.25 s
warmup+measurement window used everywhere in this codebase is *shorter
than one cycle* at that rate, so `summarize_pulsejet` saw zero completed
cycles in its window and reported near-zero thrust -- a measurement-window
artifact, not the engine's real output. Confirmed with the actual fix:
sweeping with the corrected window shows genuine (if far weaker than
high-Mach) positive thrust at every Mach from 0.0 to 0.30, e.g. ~9.3 N at
Mach 0.0 versus the previously-reported ~0 N.

**Fix: `propulsion_map.py`'s `_run_pulsejet_simulation` now extends its
measurement window adaptively** instead of trusting a fixed duration --
it keeps calling `PulsejetSimulator.run()` with a larger target duration
(each call resumes from the simulator's current clock rather than
restarting) until at least `_MINIMUM_COMPLETED_CYCLES_IN_MEASUREMENT_WINDOW`
(2) cycles have been observed post-warmup, capped by
`_MAXIMUM_PULSEJET_SIMULATION_S` (3.0 s) as a safety valve against a
combination that genuinely never ignites. At any Mach where the original
fixed window already contained enough cycles (the normal, fast-cycling
case this fidelity setting was tuned for), the extension loop never
executes -- zero added cost there; the added cost is isolated to the low-
Mach points that actually needed it. All three now-corrected "true
physical zero" claims (`trajectory.py`, `tests/test_feasibility.py`,
`tests/test_propulsion_map.py`, and the three config files' `mach: 0.40`
comments) were rewritten to describe the real mechanism (a measurement-
window artifact affecting *direct* `PulsejetSimulator` construction, which
`sizing.py`'s static trade screens still use per `propulsion_map.py`'s own
documented exception list and therefore does not benefit from this fix --
`environment.mach: 0.40`, raised from 0.20 for those call sites
specifically, is still correct and necessary).

**What this does and does not change about the adverse-plateau finding
above.** The corrected model gives the adverse-scenario pulsejet phase
genuine, if weak, low-Mach thrust it did not have in either the pre-side-
inlet model or the first (buggy) post-side-inlet measurement. This
materially changes the fuel-starvation dynamics documented earlier in this
file (that section's exact numbers predate both the side inlet and this
fix) -- rerun any candidate evaluation before trusting a specific Mach
number from before this section. The qualitative mechanisms found earlier
(fuel-split matters more than raw thrust margin near the release
condition; pulsejet's margin can exceed ramjet's near the lightoff
threshold, argued for `minimum_lightoff_test_mach` as a search variable)
still hold and were re-verified against the corrected model (see
`tests/test_optimizer.py`'s `test_evaluate_design_rewards_crossing_into_ramjet_range_in_adverse`
and `test_pulsejet_holds_better_adverse_margin_than_ramjet_near_lightoff`),
but the specific numeric fixtures backing those tests needed re-deriving --
several combinations that read as comfortably feasible before now land
exactly on a sharp, poorly-interpolated transition in the widened table
(linear interpolation between two 0.1-spaced Mach samples underestimates
thrust badly right where cycling turns on, since the true curve is closer
to a step than a ramp) or hit fuel exhaustion earlier than expected.
Neither is a new bug -- both are the search space genuinely being touchier
under a more physically complete model -- but they are a reminder that
`docs/design_convergence.md`'s numbers throughout this document are
snapshots of a specific model state, not permanent facts.

## Run 7 (13-variable search, corrected model): a more severe finding -- nominal no longer closes either

Run 7 (250 generations, population 70, seed 0, all 13 search variables
including the two added this session) finished with a winning candidate
that is *worse* than every prior run in this document: nominal peak Mach
0.519 (previously always ~1.10, closing easily), adverse peak Mach 0.360.
`chamber_volume_m3` and `throat_diameter_m` both pinned at their search
floors (0.003 m^3, 0.110 m).

**Verified this is not a search failure.** Hand-constructed a materially
"better-equipped" candidate (larger chamber, throat, fuel) and it scored
*worse* than Run 7's winner (-7562 vs -5286) -- but that comparison was
contaminated by a packaging failure (-2000 points) the larger chamber
triggered. Re-run with a packaging-*feasible* larger-hardware candidate:
still scores worse (-5746 vs -5286), by an amount fully explained by the
mass-margin and body-diameter-tiebreak difference alone (no anomaly). Run
7's winner is a faithful, rational optimum of the current objective, not a
DE convergence bug.

**Root cause, traced the same way as the earlier adverse-only finding:**
the packaging-feasible "better-equipped" candidate's *nominal* scenario
now also ends in `pulsejet_fuel_exhausted_during_climb` -- fuel allocated
to the pulsejet phase (`ramjet_fuel_fraction` complement) burns out at
Mach 0.49-0.65 before reaching `top_of_climb_altitude_min_msl_m` (6000 m),
the same failure mode previously found only in the adverse scenario. This
is a direct, expected consequence of this session's other corrections
compounding: a packaging-consistent chamber (0.003-0.012 m^3, versus the
previously-implicit, packaging-*infeasible* 0.025 m^3 baseline) carries
less captured air/fuel per cycle and less mass-model-honest hardware mass
headroom for fuel, at the same time the real low/high-Mach thrust
transition (this file's previous section) makes the climb-out from release
speed less certain. Individually, each correction this session made is
right; together, they reveal the vehicle's fuel/mass budget was
implicitly relying on physics (a free, oversized chamber; a flawed
low-speed-either-zero-or-full thrust curve) that no longer exists in the
model.

**A release-speed lever exists but is capped by an already-flagged
placeholder.** Sweeping `sled_release_speed_m_per_s` from 121 (this
search's own upper bound) to 200 m/s on a fixed candidate takes adverse
peak Mach from 0.359 to 0.594, while nominal stays flat around 0.65-0.66 --
release speed matters more under adverse specifically because it starts
the vehicle further past the pulsejet's slow-cycle/fast-cycle transition
before drag has to be overcome. `_SLED_RELEASE_SPEED_UPPER_BOUND_M_PER_S`
is derived from an explicitly unsourced 10 g launch-acceleration
placeholder and a 75 m rail length (`docs/assumptions_registry.md`,
`optimizer.py`'s own inline comment) -- worth deliberate review now that
it is not just a stall-speed lever but also a pulsejet-thrust-regime
lever, but not changed here for the same reason the adverse-scenario
multipliers were not changed: it is a real-world hardware constraint
assumption, not a free search-bound parameter.

**Net assessment.** This session closes with the model materially more
physically complete than it started (chamber mass, side inlet, corrected
low-Mach thrust, packaging feasibility scored, 13 search variables) and a
more severe, better-understood finding than the one it started with: not
just "adverse doesn't close," but "neither scenario reliably closes within
the current fuel/mass budget and fixed 195 mm intake," traced to a
specific, fixable-in-principle cause (fuel budget) rather than a vague
"the vehicle is underpowered." The next concrete lever, in the order this
investigation surfaced them, is total fuel mass margin -- which is itself
constrained by MTOM and by how much hardware mass (chamber, throat, body)
the mission needs to carry to generate that fuel's worth of thrust
efficiently; this is a genuine sizing trade this document's existing
static screens (`sizing.py`) were built for, not a new gap.

## Model correction that rejected Candidate A

Candidate A interpreted the configured 0.92 total-pressure recovery as recovery of
only the pressure rise above ambient:

\[
P_{t,2}=P+0.92(P_{t,0}-P).
\]

That is not the conventional total-pressure recovery definition. The corrected
implementation uses separate pulsejet and ramjet inputs and applies:

\[
P_{t,2}=\pi_d P_{t,0}.
\]

At Mach 1.10, this correction reduces Candidate A ramjet net thrust from the prior
603 N result to about 549 N. After the existing 15% propulsion derate, the 205 mm
Candidate A misses its 488 N drag budget by about 21 N. An automated governing-
equation test now prevents the earlier interpretation from returning silently.

## Candidate B numerical balance

| Quantity | Static low-order value |
|---|---:|
| Body / intake / throat diameter | 210 / 195 / 170 mm |
| Exit/throat area ratio | 1.05 |
| Diameter-scaled drag area | 0.010478 m² |
| Drag at Mach 1.10 and 4,500 m | 512.1 N |
| Nominal ramjet net thrust | 937.0 N |
| Existing 15%-derated thrust | 796.4 N |
| Existing 15%-derated margin | 284.3 N |
| Nominal potential-capture spillage | 44.2% |
| Nominal fuel flow | 0.08329 kg/s |
| Full-throttle endurance from 1.40 kg | 16.8 s |
| Selector radial packaging margin | 2.5 mm |
| Pulsejet mean net thrust, sea level Mach 0.20 | about 119 N |

The larger throat improves high-speed ramjet flow capacity but reduces the
low-speed pulsejet result. That shared-nozzle compromise is now explicit and must be
tested in the coupled trajectory rather than optimized at one operating point.

## Model correction that moved the throat from 160 to 170 mm

`ramjet.py` previously applied the configured `ramjet_total_pressure_recovery`
(0.92) as the *entire* inlet total-pressure recovery at every Mach number,
including above Mach 1 where a real inlet also carries an unavoidable normal-shock
loss. The corrected implementation decomposes recovery into two factors
(`ideal_inlet_shock_recovery(mach, gamma) * installed_efficiency`):

- an idealized, Mach-dependent term equal to the stationary normal-shock
  total-pressure ratio above Mach 1 (`1.0` below Mach 1, since an idealized duct has
  no shock loss there) -- reusing this repository's own tested
  `normal_shock_total_pressure_ratio`, not a new empirical correlation; and
- the same 0.92 configured value, now interpreted as the installed-efficiency
  factor for everything the idealized shock does not capture (duct friction, bends,
  bleed, the selector mechanism's own losses).

At Mach 1.10 the idealized shock term is `0.9989`, so installed recovery changes
from a flat `0.9200` to `0.9190` -- a ~0.1% reduction. That was enough to flip the
160 mm candidate's conservative-scenario excess thrust from the previously
documented **+1.5 N to -0.24 N**: it was always a razor-thin, non-robust pass, not
a solid one. Re-running the robustness trade's own selection rule (highest score
among candidates that pass every required scenario) over the existing throat grid,
and confirmed by extending that grid to 130-190 mm, now picks **170 mm** instead,
which clears the conservative screen with a real 128.8 N margin. The nominal ramjet
net thrust at 170 mm is higher (937 N vs. 832 N at 160 mm, since more of the
potential capture reaches the nozzle: spillage drops from 50.5% to 44.2%), at the
cost of faster fuel burn (16.8 s vs. 19.0 s full-throttle endurance from the same
1.40 kg) and a slightly weaker pulsejet result (119 N vs. 122 N, the same
shared-nozzle low/high-speed compromise already noted above).

## Named robustness screens

Every scenario is versioned in `configs/robustness_candidate_b.yaml`; none is a user
requirement or a validated uncertainty distribution.

| Scenario | Recovery (installed factor) | Thrust factor | Drag factor | Mass | Excess thrust |
|---|---:|---:|---:|---:|---:|
| Nominal | 0.92 | 1.00 | 1.00 | 21.0 kg | +424.8 N |
| Conservative | 0.87 | 0.85 | 1.10 | 22.5 kg | +128.8 N |
| Adverse | 0.82 | 0.75 | 1.20 | 23.9 kg | -95.9 N |

The conservative screen has a provisional 50 N excess-thrust budget; Candidate B
(170 mm) now passes it with a real margin instead of the prior 1.5 N razor's edge.
The adverse case is intentionally informational and still fails, though by a smaller
shortfall than the 160 mm candidate (-95.9 N vs. -153.7 N). This remains evidence
that inlet recovery and installed thrust are the dominant risks, not evidence that
the mission closes robustly.

## Mass accounting

The component allocations reconcile exactly to the configured 21.0 kg current
estimate. Their explicit high-side additions total 2.9 kg, producing a 23.9 kg
high-side estimate and 1.1 kg remaining against the 25 kg requirement. Most component
entries are conceptual allocations; the numerical reconciliation is not a weighed
vehicle.

## Present limiting factors

1. **Inlet recovery and installed thrust** dominate the Mach 1.10 prediction. The
   adverse scenario cannot hold the drag budget.
2. **Shared-nozzle sizing** is the main geometry compromise: a larger throat helps the
   ramjet and weakens the current pulsejet result.
3. **Body diameter** is no longer at the exact selector packaging boundary, but the
   2.5 mm radial margin is still provisional and must absorb real tolerances.
4. **Mass** remains below 25 kg in the stated high-side budget, but it has not yet been
   coupled to acceleration, climb, stability, or recovery.
5. **Aerodynamic drag** is still a diameter-scaled budget. The flight kernel's simple
   coefficient polar is not consistent enough to replace it at Mach 1.10.

## Mach-indexed drag and phase-based trajectory (new)

`src/douglas_dart/drag.py` replaces the flight kernel's constant-coefficient polar
with a Mach-indexed drag-area model anchored at the existing peak-Mach drag-area
budget and shaped by an explicit transonic drag-rise table (still an unvalidated
proxy, but now one consistent model instead of two disagreeing ones). Both discipline
paths — `sizing.py`'s static screens and the flight kernel — can now read the same
number at any Mach, not only at exactly 1.10.

`src/douglas_dart/trajectory.py` adds the phase-based sled-release-to-landing mission
integrator named in the roadmap. It is an energy-state model with a *prescribed*
flight-path angle per phase, not a trimmed/lift-solved trajectory, and it can be run
at the nominal, conservative, or adverse robustness multipliers via
`douglas-dart mission-trajectory --scenario <name>`.

Running it at the current Candidate B configuration surfaces two new, previously
invisible findings rather than confirming closure:

1. **A low-speed pulsejet net-thrust trough near Mach ~0.1** appears immediately
   after sled release in the underlying unsteady chamber model (thrust rises from a
   small positive value at Mach 0.0-0.1 to roughly 120-145 N by Mach 0.2 at this
   altitude). Under the conservative and adverse thrust derates this trough is enough
   to stall the climb-phase acceleration entirely in the current model. This is either
   a real low-speed operability risk or a modeling/averaging-window artifact; it is
   not yet distinguished, and it was invisible in the prior static single-point
   pulsejet result.
2. **The configured shallow dive does not reach the ramjet light-off Mach** (0.80)
   before hitting the altitude floor at nominal multipliers with the currently
   configured 10-degree dive angle and 8-degree climb angle. Reaching a higher Mach
   before the dive, diving deeper, or diving longer are all now visible trade knobs
   instead of an assumed transition.

Neither finding should be read as proof the architecture fails; both are direct
consequences of provisional numbers (dive/climb angles, pulsejet low-speed table
resolution) that are now open trajectory-level trade variables. They are reported
here, not hidden, because the point of the model is to find where the design does not
yet close.

## Do not freeze yet

Keep body diameter, throat size, inlet recovery, drag area, fuel allocation, and
transition schedule open. The next high-value gate is a coupled phase-based mission
simulation using one documented drag-area model, followed by inlet/back-pressure and
external-aerodynamic closure. Candidate B must not be promoted as mission-feasible
until those gates, stability/control, and recovery are demonstrated.

## Pulsejet cycle-timing sweep

`scripts/pulsejet_sweep.py` grid-searched the existing, tested `PulsejetSimulator`
(no new physics) over `minimum_cycle_period_s`, `burn_duration_s`,
`ignition_pressure_ratio_max`, `target_equivalence_ratio`, and all four fuels in
`configs/fuels.yaml` -- 3,000 points, results in `docs/pulsejet_sweep_results.csv`.

The unconstrained best point (Jet-A, 0.030 s cycle period, 0.002 s burn, ignition
ratio 1.05, stoichiometric) roughly doubled mean net thrust to 266 N, but pushed peak
chamber pressure to 2.33x the inlet total pressure and dropped completed cycles in a
0.5 s window to 10. That low cycle count under-samples `trajectory.py`'s fixed-window
static thrust table (`_pulsejet_static_thrust_table`, 0.25 s warmup + 0.25 s
measurement) enough to make its Mach-indexed averages noisy rather than smooth --
concretely, it broke the `test_adverse_scenario_does_not_out_perform_nominal`
invariant, since the adverse-scenario table sampled the sparse cycle train at
different phase than the nominal table despite otherwise-worse conditions.

Adopted instead: 0.014 s cycle period, 0.002 s burn duration, 1.01 ignition-pressure
ratio, stoichiometric target equivalence ratio, same Jet-A fuel. This keeps 17
completed cycles in the same averaging window (matching the prior baseline's 16),
still roughly doubles mean net thrust (111 N to 237 N) and specific impulse (318 s to
578 s), and holds peak chamber pressure to 1.62x inlet total pressure instead of
2.33x -- a materially smaller reed-valve/casing overpressure margin to design against.
All three configs (`reference_case.yaml`, `shared_nozzle_candidate_a.yaml`,
`shared_nozzle_candidate_b.yaml`) were updated to this point and the full test suite
was re-verified green.

Fuel type was not switched: gasoline and propane scored marginally higher in the raw
sweep, but propane requires pressurized liquid storage hardware not modeled anywhere
in the mass budget (`fuel_trade.py` already flags this), and switching fuel changes
density-driven tank volume and mass-budget assumptions that were not re-validated in
this pass. Jet-A remains the baseline; the fuel-type trade stays open in
`fuel_trade.py`'s comparison, not silently decided here.

## Coupled mission-level design search confirms the robustness gap is structural

`optimizer.py`'s differential-evolution search (`douglas-dart design-optimize`) was
run unattended over body diameter, throat diameter, nozzle area ratio, fuel mass,
ramjet fuel fraction, and climb/dive/entry angles -- 24 candidates x 60 generations,
1,440 full nominal+adverse trajectory evaluations, checkpointed to
`docs/design_search_run/` -- using the pulsejet timing adopted above. This is a wider
and more systematic search than any single hand-picked point checked so far.

It did not find a design that closes under both scenarios. The best candidate found
reaches Mach 1.106 and meets the minimum-time-above-Mach-1 requirement nominally, but
the adverse scenario tops out at Mach 0.595 and never meets the duration requirement,
despite the search being free to shrink body diameter and throat size toward their
lower bounds and reallocate fuel between pulsejet and ramjet however it liked. The
search score is dominated by the adverse-scenario penalty terms (weighted 3x nominal,
per `optimizer.py`'s `_SCENARIO_WEIGHTS`) and stayed negative through all 60
generations, meaning even the least-bad point found is still a rule/duration failure,
not a marginal pass.

This corroborates -- with a systematic search rather than a single guess -- the
low-speed pulsejet trough and shallow-dive-transition findings already reported above:
the adverse scenario's combined 0.75x thrust multiplier, 1.20x drag multiplier, and
2.9 kg mass growth is enough to prevent ramjet light-off and duration closure across
essentially the whole searched design space, not just the current Candidate B point.
The search's best variables were **not** adopted into `configs/shared_nozzle_candidate_b.yaml`:
it is not a design improvement, only evidence about where the model's true feasible
region is (or currently is not). Closing this gap needs either a real modeling
improvement (finer low-speed pulsejet resolution, a real dive-transition trade) or a
relaxation of what "adverse" represents -- not a further search over this same
variable set.

## Ram-air inlet lip: cowl-ratio parameterization

`RamInletConfig` previously took `length_m` and `lip_thickness_m` as independent
absolute dimensions. It now derives both from the actual intake diameter via two
dimensionless cowl-design ratios: `lip_overshoot_fraction` (the lip highlight's
diametral increase over the intake diameter, 5% in all three configs -- typical
low-drag subsonic pitot lips run 3-10%) and `lip_fineness_ratio` (axial length per
unit of that diametral overshoot, 7.0 in all three configs). This makes the duct
scale correctly if the selector intake diameter trade ever moves, instead of needing
a manual absolute-dimension update; the resulting geometry is numerically close to
the prior hand-picked values (was 70-80 mm length / 5-6 mm lip thickness at 195 mm
intake diameter).

## Low-Mach thrust magnitude sanity check: real, but explained by duty cycle, not a bug

The unsteady simulator's low-Mach (M=0.05-0.30) net thrust, with the inlet-inertia
fix in place and evaluated at full fidelity (`warmup_s=measurement_s=0.25`,
`time_step_s=0.00005`), is ~2.3-3.3N across that band -- confirmed by direct
re-run, not just the earlier session's report. A naive capture-area scale of the
externally cited NACA MR E5J02 static result (2224N from a 22-inch/0.559 m intake)
down to this vehicle's 195 mm intake, `(0.195/0.559)^2 = 0.122`, predicts ~271N --
roughly **90-115x** higher than what this model produces. That NACA figure is taken
from the planning document that supplied it, not re-derived from the primary
source in this repo; treat its exact value as unverified within this codebase even
though the qualitative comparison below still holds regardless of moderate error in
it.

This is a real, large gap, but it does not indicate a bug once duty cycle is
accounted for. Re-running the reference case at M=0.10 shows ignition events at
t=0.0013, 0.861, 1.825 s -- a real cycle period of ~0.86-0.96 s, against
`PulsejetConfig.minimum_cycle_period_s = 0.014` s (the tuned/design period). That is
a duty cycle of roughly 1.5-2% of design frequency: at low Mach, refill is starved
(only the small `inlet_total_pressure_pa` minus blown-down chamber pressure
differential drives inflow, not ram pressure -- see `propulsion_map.py`'s adaptive-
window comment), so the engine fires far less often than a resonance-tuned engine
like NACA's reference would. A ~60x lower firing rate, compounded with
`SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION = 0.15` crediting only a fraction of
whatever ram pressure exists at low Mach, plausibly accounts for the full ~90-115x
gap without requiring a modeling defect.

**Conclusion**: the low-Mach thrust magnitude is gated by cycle period, not simply
"how much air the intake can capture." This reinforces (does not newly reveal) why
the fluid-inertance/side-inlet-recovery work targets the refill-phase mass flow --
that is the actual lever on this number -- and it is a genuine open question whether
this vehicle's cycle period should realistically be this long at low Mach, or
whether the refill-phase physics (inertance duct sizing, check-valve idealization)
is itself under-predicting inflow. Not resolved here; flagged for the closed-form
and phase-lag work already in progress to address, not force-fit away.

## Ramjet nozzle near-critical onset smoothing: a real, consequential fix

`compressible.py`'s `fixed_cd_nozzle` had a hard `chamber_total_pressure_pa <=
ambient_pressure_pa` gate returning exactly zero mass flow. Direct re-derivation
this session found the underlying isentropic relation is actually continuous across
that boundary (`_mach_from_static_pressure_ratio`'s limit as pressure ratio -> 1 is
genuinely zero) -- but with an infinite initial slope (mach ~ sqrt(2*(1-ratio)/gamma)
near the crossing), the same square-root-of-differential-pressure behavior any
small-driving-pressure nozzle shows. Combined with the ramjet's momentum drag
(mdot * V_freestream, mdot ~ sqrt(eps)) growing far faster near the crossing than
gross thrust (mdot * V_exit, both ~ sqrt(eps), so gross ~ eps), this produced a real
but numerically pathological narrow trough -- confirmed by direct fine-resolution
evaluation at 0.0005 Mach steps: net thrust plunged to -16.31N within 0.005 Mach of
the M=0.46 crossing (`evaluate_ramjet` on `reference_case.yaml`), a cusp sharp enough
to look like a hard binary cutoff to any coarse Mach sweep, search algorithm, or
interpolation. Fixed with `_nozzle_onset_smoothing_factor` (`compressible.py`): a
smoothstep blend of the isentropic exit Mach over a small (2%, unsourced engineering
regularization, not a physical value)  pressure-ratio margin approaching the
crossing, giving the curve zero slope there instead of infinite slope. The trough is
now shallower per-Mach-step but wider (bottoms near -17.2N around M=0.48 instead of
-16.3N at M=0.47), and the choked-regime plateau above M~0.535 is essentially
unchanged (was 39.49N at M=0.54, still ~39.5N).

**This is not cosmetic.** `shared_nozzle_candidate_b.yaml`'s adverse-scenario mission
trajectory was riding the exact top edge of the old cusp: before this fix, the
pulsejet climb reached `dive_entry_mach=0.45`, dove to `ramjet_lightoff_mach=0.50`
(landing right at the boundary), then immediately stalled
(`nonpositive_ramjet_net_thrust_cannot_accelerate`) -- `adverse_peak_mach` reported as
0.5006, just barely counted as "reaching" ramjet range. After the fix, the same
mission simply never reaches `dive_entry_mach` at all: pulsejet climb runs out of
mission time first, and `adverse_peak_mach` plateaus at 0.386 -- identically,
regardless of `ramjet_fuel_fraction` (checked 0.05-0.892) or `dive_angle_deg`/
`dive_entry_mach` (checked a 4x3 grid). The old "crossing" result was an artifact of
landing exactly on top of the un-smoothed cusp's peak, not a robust margin -- this
fix makes that visible rather than hiding it, consistent with this codebase's
practice of surfacing real feasibility gaps (`test_optimizer.py`'s and
`test_robustness.py`'s expected values updated accordingly to the new real,
recomputed numbers, not guessed).

Two concrete, verified effects, both real re-runs, not estimates:
- `robustness.py`'s minimum-feasible trade shifted back from 210/170 mm to 205/160 mm
  (the pulsejet inertia fix's own earlier 210/170 mm shift, described above, is
  partially undone by this fix reducing ramjet contribution near the transition band).
- The differential-evolution search's own "did not find a design that closes under
  both scenarios" finding (above, "Coupled mission-level design search") is now
  **more severe for this exact design point**: no explored fuel split reaches ramjet
  range at all in the adverse scenario, not merely a duration-closure shortfall past
  a reached ramjet Mach. Whether a materially different geometry (larger throat, more
  aggressive dive) can still close under the corrected model is open -- the coupled
  search documented above was run before this fix and should be considered stale for
  any conclusion involving the M=0.46-0.53 transition band specifically.

  (This finding is itself superseded by the side-inlet ram recovery fix immediately
  below -- see that section for the updated picture.)

## Side-inlet ram recovery: real correlation, and it reopens the ramjet crossing

`pulsejet.py`'s `SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION = 0.15` (an unsourced flat
placeholder) is replaced by `side_inlet_ram_recovery_ratio`, a real correlation from
Hall & Frank, NACA RM A8I29 (1948), "Ram-Recovery Characteristics of NACA Submerged
Inlets at High Subsonic Speeds" -- flush fuselage inlets, matching this vehicle's
confirmed flush/side-mounted reed-valve architecture (unlike the previously-considered
NACA MR E5J02, a forward-facing inlet that doesn't apply here). The correlation is a
function of mass-flow coefficient (captured mass flow / freestream mass flow through
the capture area), not Mach: 0.50 at zero flow, 0.90 at a 0.6 mass-flow coefficient,
0.95 near 1.0 (Mach/angle-of-attack effects reported under 0.03, not modeled). Coupled
to the fluid-inertance inlet model, not independent: `PulsejetSimulator.step()`
recomputes the recovery ratio every step from the *previous* step's own
`inlet_mass_flow_kg_per_s` state (the same quasi-steady, lagged coupling the
inertance ODE's own explicit-Euler integration already uses), rather than assuming a
separate flow rate.

The floor value (0.50) is more than 3x the old flat 0.15 credit, so this raises
pulsejet thrust broadly, not just at high flow -- confirmed by direct re-run of the
robustness trade and the mission-level crossing check used above:

- `robustness.py`'s minimum-feasible trade moved from 205/160 mm (after the nozzle
  smoothing fix alone) back to 210/170 mm.
- The mission-level crossing check above (`shared_nozzle_candidate_b.yaml`,
  `body_diameter_m=0.21`, `throat_diameter_m=0.14`) now crosses into ramjet range in
  the adverse scenario again, reliably: checked `ramjet_fuel_fraction` from 0.05 to
  0.892, and **every value** produces the identical outcome, `adverse_peak_mach =
  0.5080459099127839` and `score = -6692.232342970098` (both exact matches across
  the whole range, not approximate). The trajectory: pulsejet climb reaches
  `dive_entry_mach`, dive reaches `ramjet_lightoff_mach`, then `ramjet_accel`
  immediately terminates with `nonpositive_ramjet_net_thrust_cannot_accelerate` --
  the M~0.50-0.51 crossing lands inside the still-negative part of the smoothed
  trough (see the section above; the trough's zero-crossing is near M=0.4975-0.50 in
  the bare `evaluate_ramjet` function, but this trajectory's exact flight state lands
  it just inside the negative side). Because this failure depends only on the
  instantaneous flight condition at the lightoff boundary, not on how much fuel was
  allocated to the ramjet, **no fuel split changes the outcome** -- `test_optimizer.py`'s
  crossing-reward test, which specifically checked that fuel allocation could tip this
  balance, no longer has anything to differentiate and is skipped with a citation to
  this section rather than force-fit to a new pair of numbers that don't actually
  differ.

**Net picture across all three fixes this session (inertia -> nozzle smoothing ->
side-inlet recovery)**: the vehicle now reliably *reaches* ramjet lightoff Mach in the
adverse scenario (a real improvement -- it previously either barely grazed it by
coincidence or missed it entirely, depending on which intermediate fix state is
compared). But it cannot yet produce positive net thrust there, so it still cannot
cross into self-sustaining ramjet operation under adverse conditions, for any tested
fuel allocation. This narrows the open problem considerably: it is no longer "does
the vehicle ever reach ramjet range" but specifically "why is ramjet net thrust still
negative in the M~0.50-0.53 window right at this design point's crossing," which is a
`ramjet.py`/nozzle-geometry/`minimum_lightoff_test_mach` question, not a pulsejet-side
one -- worth investigating directly (e.g. does a larger throat or different area ratio
move the positive-thrust onset below the actual crossing Mach?) before re-running the
full differential-evolution search, since the search itself would otherwise spend most
of its budget rediscovering this same local structure.

## Calibration DOE sizing: re-measured cost, and a philosophy check before running anything

Re-measured directly against the current code (inertia + nozzle-smoothing + side-inlet
recovery all in place), an 11-Mach-point pulsejet table now costs:

- **Fast fidelity** (`warmup_s=measurement_s=0.10`, `time_step_s=0.0001`, `optimizer.py`
  defaults): 22.5s / 11 points ~= 2.05s/point. Slightly worse than the earlier ~19.3s/11
  points reported before this session's fixes -- the per-step side-inlet recovery
  recalculation and the wider nozzle-onset trough both add real cost, not just the
  inertia model alone.
- **Full fidelity** (`warmup_s=measurement_s=0.25`, `time_step_s=0.00005`,
  `trajectory.py` defaults): 101.9s / 11 points ~= 9.3s/point -- ~4.5x fast fidelity.

A full pop-70/gen-250 differential-evolution search (17,500 candidates x 11-point
pulsejet tables) would now cost ~109 hours at fast fidelity, ~496 hours at full
fidelity -- both worse than the ~94-hour figure reported before this session, not
better. This *strengthens*, not weakens, the case for the closed form as the search's
actual propulsion model.

**But before sizing a large internal-simulator calibration DOE, a real conflict needs
resolving first, not silently worked around:** the governing unified plan's own stated
philosophy is "stop calibrating the pulsejet closed form against our own
`PulsejetSimulator` ... validate against real external data instead" (NASA/TM-2008-215432,
Litke/Schauer/Paxson) -- `PulsejetSimulator` is explicitly repurposed for
design-sensitivity work only, not as ground truth. A large internal-simulator DOE built
to *calibrate* the closed form the way `pulsejet_calibration_fit.py`/
`pulsejet_calibration_generate.py` (this session's untracked scripts, predating the
philosophy pivot) currently do would directly contradict that decision. Recommendation,
not yet executed pending confirmation:

1. **Do not run a large calibration-DOE sweep against `PulsejetSimulator`** for the
   closed form's coefficients -- that whole approach is superseded per the unified
   plan, regardless of how affordable it is now. `configs/pulsejet_closed_form_calibration.yaml`
   and the `pulsejet_calibration_*.py` scripts are stale artifacts of the pre-pivot
   approach and should be treated as such (not deleted without confirming, since they
   may still be useful as a design-sensitivity harness once repurposed, but not as
   calibration inputs).
2. **What a DOE is still legitimately needed for**: `PulsejetSimulator`'s new
   design-sensitivity role (Part 3's own stated purpose -- "how thrust shifts with
   chamber volume, intake diameter, and other geometry changes"). At ~2.05s/point fast
   fidelity, a bounded sensitivity sweep (e.g. 5 geometry variables x 5 levels x 11 Mach
   points = 1,375 points ~= 47 minutes) is comfortably affordable without any special
   sparsity treatment -- the ~100+ hour cost problem is specific to the *unbounded*
   17,500-candidate differential-evolution search, not to a designed sensitivity study.
3. **The closed form itself** should validate against the external NASA/Litke data
   points directly (a handful of spot checks, not a DOE) once Part 3's static-baseline
   derivation (still blocked on this session's other open items) exists to validate.

This is a scope/direction question, not a technical blocker -- flagging for
confirmation before any further calibration-shaped work proceeds, rather than
defaulting back to the pre-pivot approach because its scripts already exist.

**Self-correction (2026-08-09):** picked up the closed-form task after the Gate 3
architectural-blocker finding below and started re-running exactly the superseded
approach this section warns against -- fixed `pulsejet_calibration_generate.py`'s
stale API call (harmless, real bugfix, left in place) and launched it against
`PulsejetSimulator` to rebuild the Mach-sweep calibration dataset. Caught and killed
before it produced output: this section, written earlier in the same session, already
flags that path as superseded pending confirmation, and no confirmation was given.
Leaving both this task and Task 7 pending rather than proceeding further -- the
external-data-validation approach recommendation above still stands, unexecuted.

## Cycle-based convergent averaging: fixed, confirmed by direct re-plot

Per `cycle_based_averaging_fix.md` (2026-08-08, user-supplied): the pulsejet path's
fixed `warmup_s`+`measurement_s` wall-clock averaging window caught a different,
non-integer number of real ignition cycles at each Mach step, producing broad
sawtooth jaggedness in every Mach sweep this session had regenerated -- confirmed
present in all three "regenerated plots" from earlier in this session, absent from
the smooth closed-form-free ramjet curve (the control case, unaffected since
`evaluate_ramjet` is a steady closed form with no cycle concept). Structurally the
same class of bug as the original Gate 2 fast/full-fidelity mismatch: two callers
choosing different window settings could disagree on the same physical question.

Replaced the fixed window with `pulsejet.py`'s
`run_pulsejet_to_converged_cycle_average`: detects real ignition-cycle boundaries
directly from the simulator's own `event == "ignition"` trigger (no separate
detection algorithm), discards the first 2 completed cycles as startup transient,
then accumulates a running per-cycle average and stops once it has held inside
tolerance (max of 0.1% relative or 0.05N absolute, on net thrust) for 3 consecutive
cycle updates -- or reports `converged=False` if a 30-cycle cap is hit first
(matching `ramjet.py`'s own supercritical-recovery iteration-cap convention), never
silently trusting an unconverged value. Tolerance and the cycle cap are fixed
internal constants now, not caller-adjustable parameters -- `warmup_s`/`measurement_s`
no longer exist as a concept anywhere in the propulsion-map-routed path
(`propulsion_map.py`, `trajectory.py`, `optimizer.py`, `robustness.py`,
`sensitivity.py`, `sizing.py`'s `evaluate_shared_nozzle_trade` which is load-bearing
for `robustness.py`'s trade search, not just a report) -- this closes out that half
of the original Gate 2 fidelity bug for the unsteady-sim path. `time_step_s` (dt)
remains caller-adjustable, deliberately: cycle-counting and integration step size
are separate concerns (the fix doc's step 7), and dt's own convergence study is not
done here.

**Not touched, deliberately scoped out**: three genuinely standalone diagnostic call
sites that construct `PulsejetSimulator` directly for raw per-step sample dumps or
conservation audits (`pipeline.py`'s pulsejet diagnostic stage, `cli.py`'s standalone
`pulsejet` command) -- already documented in `propulsion_map.py`'s own docstring as
intentional exceptions that don't feed another design calculation, so migrating them
doesn't change any design conclusion. Flagged here so this remaining gap doesn't go
unnoticed, not because it's been forgotten.

**Validated by direct re-run, not assumed**: regenerated all three thrust-vs-Mach
plots (`scripts/plot_thrust_vs_mach.py`, `scripts/plot_thrust_curves.py`) at dense
Mach sampling (0.01 step, 100+ points) with the new convergence-driven averaging.
The broad structural jaggedness tracking Mach step-to-step is gone in all three --
`reference_case.yaml`'s pulsejet curve is now a single smooth rise/dip/climb shape
end to end, and the fast-vs-full-dt comparison plot shows both dt curves tracking
each other closely with only fine-scale local wiggle remaining (expected, per the
fix doc's own prediction -- that residual is dt sensitivity, a separate still-open
concern, not evidence the windowing fix failed).

**Real cost, measured not assumed**: a 20-point Mach sweep (`reference_case.yaml`,
full-fidelity dt=5e-5) averaged 6.5s/point (range 1.9-17.1s), noticeably uneven --
some points converge in 5-9 cycles, others need the full 30-cycle cap without
settling (observed at M=0.65, 0.70, 1.00 on this exact sweep: `converged=False`,
correctly flagged via `cycle_average_did_not_converge` rather than silently
reported). This is not uniformly faster or slower than the old fixed-window
approach -- exactly the caveat the fix doc itself flagged in advance.

## Sawtooth root cause: a real damped cycle-to-cycle oscillation, not dt noise

The plots above still showed fine-scale zigzag after the fix, concentrated around
M=0.6-0.75. Direct instrumentation of the raw (unaveraged) per-cycle net thrust
inside a single Mach point found the actual cause: the pulsejet has a genuine,
physical damped oscillation from cycle to cycle, not measurement noise. At M=0.65
(`reference_case.yaml`), consecutive post-transient cycles ran:

```
33 -> 338 -> 103 -> 257 -> 128 -> 226 -> 141 -> 218 -> 147 -> 208 -> 154 -> 198 -> 159 ...
```

still visibly alternating strong/weak after 13 cycles. Mechanism: a strong ignition
burns more of the chamber's fuel/air, leaving less behind for the next refill, so
the next cycle ignites weaker -- which under-consumes the reservoir, so the cycle
after that is strong again. This decays toward a true limit cycle, but how fast
varies a lot by Mach (M=0.45 and M=0.8 damp out in ~3-4 cycles in the original,
looser check; M=0.35 and M=0.65 were still swinging after 13).

The original fix's convergence check (running-mean stability only) was not a
reliable signal for this: because each new cycle's influence on a cumulative mean
shrinks as roughly 1/N, the running mean can stop moving by tolerance well before
the underlying oscillation has actually decayed -- so some Mach points reported a
"converged" value that still carried a phase-dependent residual bias, and this
residual varied unpredictably point to point as Mach stepped by 0.01. Confirmed
directly: at M=0.61-0.70 and M=0.74, nearly every point hit the original 30-cycle
cap and reported `converged=False` (a partial average taken mid-oscillation),
while immediately adjacent points (M=0.71-0.73, 0.75) converged cleanly in
8-15 cycles -- neighboring points getting a real converged value next to a flagged
partial one is a direct, mechanical explanation for visible zigzag in exactly that
band, on top of the subtler mean-lag effect.

**Fix (2026-08-08, user decision)**: `run_pulsejet_to_converged_cycle_average` now
requires *two* conditions before declaring convergence, both inside the same
tolerance (max of 0.1% relative or 0.05N absolute), for 3 consecutive updates:

1. The running mean has stopped moving (original check).
2. The raw per-cycle net thrust itself has settled -- max minus min over the
   trailing 4-cycle window (`_CYCLE_AVERAGE_SWING_WINDOW_CYCLES`, spans two full
   periods of the observed period-2-like alternation, so a persistent
   not-yet-decayed oscillation cannot pass by chance landing on a same-phase pair).

The cycle cap was also raised from 30 to 100 (`_CYCLE_AVERAGE_MAXIMUM_CYCLES`) --
explicit user preference: report honest non-convergence over an averaged value from
a model that has not actually reached steady state, rather than tune the cap to
hide slow-decaying points. Re-verified on the previously-affected range: M=0.65 now
correctly hits the 100-cycle cap and reports `converged=False` (was silently
"converged" at 30 cycles before), while M=0.45/0.6/0.8 still converge cleanly (19,
25, and 14 cycles respectively under the stricter check, vs. far fewer under the
old mean-only check -- the stricter check costs more cycles across the board, not
just at the hard points, which is expected and intentional). All 128 tests still
pass. Real per-point cost is correspondingly higher and more uneven (observed up to
~43s for a point that runs the full 100-cycle cap) -- accepted tradeoff per the
above preference, not treated as a defect to optimize away.

## Inflow phase-lag mismatch: root cause found (idealized valve, not the ignition trigger)

NASA/TM-2008-215432 reports peak inflow lagging the chamber pressure minimum by
~1/9 cycle; this codebase's own cycle instead showed peak inflow landing right at the
next ignition event. Direct instrumentation of one full cycle at M=0.45
(`reference_case.yaml`, period 0.014s) confirms this and identifies the mechanism:

```
t (ms)   chamber p (Pa)   inlet mdot (kg/s)   phase
 0.0        105,584            4.114          combustion (tail of prior cycle)
 1-6        180k->108k          0.0            combustion/blowdown (valve closed)
 7.0         91,827             0.122          blowdown (valve reopens)
 8.0         91,872             0.940          refill
 9.0         92,565             2.066          refill
10.0         94,462             3.197          refill
11.0         98,477             4.001          refill (near-peak)
12.0         99,506             0.0            brief reclosure
13.0         91,783             0.310          refill
14.0            --                --           next ignition
```

Mass flow rises essentially continuously from valve-reopen (t=7ms) to t=11ms, i.e.
across nearly the whole refill window, peaking close to (not right after) the next
ignition -- confirming the earlier report, not just re-describing it. The mechanism:
the idealized instantaneous check valve (Ghulam et al. 2024's two-state
simplification, adopted specifically to avoid needing NASA eq. 3-4's unmeasured reed
valve mass/spring-constant -- see this session's handoff) has no dynamics of its own.
While the valve is open, `dmdot/dt = (A_in/L_in)*(P_in - P_chamber)` is sign-definite
(positive as long as `P_in > P_chamber`), so captured mass flow is *mathematically
forced to be non-decreasing* for as long as the valve stays open, regardless of how
small the driving pressure differential shrinks to. It can only fall by the valve
closing outright (chamber pressure catching up to inlet total pressure) or by
ignition truncating the cycle -- there is no mechanism in this model for inflow to
peak and *then decline while the valve is still open*, which is exactly what a real
reed valve's own inertia/stiffness (NASA's dropped eq. 3-4) does: valve motion driven
by its own mass-spring dynamics can begin closing well before the chamber-pressure
differential alone would demand it, producing a true mid-cycle local maximum instead
of a monotonic ramp cut short by the next event.

**Conclusion**: this is a structural, understood, and -- given the decision already
made not to re-add reed-valve dynamics (unmeasured hardware, dead end per this
session's Task 2 investigation) -- an *accepted* limitation of the idealized-valve
architecture, not an open mystery requiring further investigation. It should be
treated as a known qualitative-shape gap when comparing this codebase's cycle timing
against NASA's resonant-valve data, not evidence of a separate bug. It does not by
itself explain the earlier low-Mach thrust-magnitude gap (that is dominated by cycle
period/duty cycle, documented above) -- the two findings are related (both trace back
to refill-phase modeling choices) but are not the same effect.

## Cycle/time caps need the same tier-dependent treatment dt already got

Found while chasing an unexplained test-suite hang (2026-08-08, same session as the dt
tiers above): `test_evaluate_design_scores_packaging_failures_it_does_not_hide`
(`chamber_volume_m3=0.012`, near this search's own upper bound, deliberately
packaging-infeasible) hung indefinitely. Root cause: `run_pulsejet_to_converged_cycle_
average`'s cycle cap (`_CYCLE_AVERAGE_MAXIMUM_CYCLES`) and simulated-time cap were a
single global constant (100 cycles / 400s) applied to *every* caller, sized for
`PULSEJET_FIDELITY_FULL`'s verification-grade "chase real convergence" needs. But
`evaluate_design` builds an 11-point pulsejet table for each of 2 scenarios (nominal,
adverse) -- if a pathological candidate (oversized chamber volume, in this case) hits
the cap at every point, one `evaluate_design` call could cost up to ~2.4 hours
(11 x 2 x the 400s per-point worst case). Confirmed directly: the isolated test alone
ran past a 90s timeout without finishing.

This is the identical problem dt itself already had (one setting can't serve both "the
search needs bounded, predictable per-candidate cost across thousands of candidates"
and "verification needs to actually chase convergence") -- it just hadn't been
noticed yet because dense Mach-sweep plotting (which motivated raising the cap from
30 to 100 cycles per user decision) doesn't hit a pathological *geometry* the way the
search's full variable-bound exploration does.

**Fix**: `propulsion_map.py`'s `pulsejet_cycle_bounds_for_fidelity` resolves the cap
tier-dependently, same pattern as `pulsejet_time_step_s_for_fidelity`:
- `PULSEJET_FIDELITY_FAST` (search): 30 cycles / 30s -- bounds worst-case search cost;
  well above the 5-20 cycles most points actually need at this coarser dt.
- `PULSEJET_FIDELITY_FULL` (verification): 50 cycles / 150s -- tightened from the
  original 100/400, matching the values `find_converged_time_step`'s own solver
  already used successfully (docs above) rather than the more generous but untested
  100/400. Real re-run: the previously-hanging test now completes in 6.9s.

`run_pulsejet_to_converged_cycle_average`'s two override parameters
(`_maximum_cycles_override`/`_maximum_simulated_time_s_override`) now serve both the
dt-solver's internal evaluations *and* the production tier system -- still not raw
caller-adjustable floats; every real call site resolves them from one of the two named
fidelity tiers, never an arbitrary value, so cycle_based_averaging_fix.md's original
"no caller-adjustable knob two callers could disagree on" guarantee still holds.

## design-optimize v9: real search run, and a real scoring bug it exposed

With the cycle/time caps now tier-dependent (above), per-candidate cost at
`PULSEJET_FIDELITY_FAST` dropped to ~1.4s cold / subsecond cached for a representative
candidate -- a real, measured number, not the ~94-110 hour estimate that applied to
the old fixed-window averaging. Ran a real, complete `design-optimize` search
(population 20, generations 40, seed 0, `shared_nozzle_candidate_b.yaml` baseline,
`configs/robustness_candidate_b.yaml` mass budget) -- `results/generated/
design_optimize_v9/checkpoint.json` -- in ~93 minutes wall clock, 100% feasible
population throughout.

**Real result, verified by re-running the winning candidate through `simulate_mission`
directly at full fidelity (`scripts/gate3_check_candidate.py`), not just trusted from
the search's own fast-fidelity score:**

- **Stall margin is solved.** `minimum_stall_margin_fraction` went from -0.3773
  (nominal) / -1.0 (adverse) at baseline to **+0.8034 / +0.6790** -- both comfortably
  positive. The search found this primarily by maxing out `sled_release_speed_m_per_s`
  at its upper bound (121.0 m/s, from 39-42 m/s baseline) -- confirming
  `docs/level0_feasibility_bounds.md`'s prediction that release speed was the cheap
  lever, not by growing `wing_area_scale_factor` (which the search actually *shrank*
  slightly, to 0.908 from 1.0 baseline -- the higher release speed alone more than
  closed the ~3.6x gap, so growing wings further only cost mass for no benefit at this
  point in the trade).
- **Both scenarios now reach `ramjet_accel` phase** (`['pulsejet_climb', 'dive',
  'ramjet_accel']`), a real improvement over baseline (nominal stalled out in `dive`
  before baseline; adverse never left `pulsejet_climb`).
- **New, clear blocker**: both scenarios terminate with
  `ramjet_net_thrust_nonpositive_during_acceleration` at peak Mach ~0.59-0.60. The
  search picked `minimum_lightoff_test_mach = 0.50` -- *exactly its own lower search
  bound*.

**That last number is not a coincidence -- it is a real scoring bug, found and fixed.**
`evaluate_design`'s `_REACHED_RAMJET_IGNITION_REWARD` (a 300-point, `_SCENARIO_WEIGHTS`-
weighted bonus for crossing into ramjet range, added in an earlier session -- see that
constant's own comment) compared `result.peak_mach_reached` against `candidate_case.
ramjet.minimum_lightoff_test_mach`. That threshold was a fixed config value when the
reward was written, but a later session made `minimum_lightoff_test_mach` a 12th search
variable -- turning the comparison self-referential: **a candidate is rewarded for
picking a *low* threshold almost independent of real mission performance**, since a
lower self-chosen bar is trivially easier to clear. This design-optimize v9 run
confirms the failure mode directly: the search converged on the threshold's own lower
bound for a candidate whose real mission run enters ramjet mode at that exact Mach and
immediately fails to accelerate -- collecting the reward for reaching a threshold
picked specifically because it was already there, not for reaching ramjet-useful
flight.

**Fix**: compare against `candidate_case.ramjet.minimum_self_sustaining_mach` instead
-- fixed per `ReferenceCase` (1.10 in this config), not a search variable, so it cannot
be gamed by choosing it. All 128 tests still pass (none exercised this specific
comparison).

## design-optimize v10: scoring fix confirmed working, but a new real blocker

Re-ran the identical search (population 20, generations 40, seed 0) with the scoring
fix in place -- `results/generated/design_optimize_v10/checkpoint.json`. Confirms the
fix changed real search behavior, not just the code: `minimum_lightoff_test_mach` in
the best-so-far candidate moved to 0.784, 0.753, 0.837, 0.987 (right against the
search's *upper* bound) across the first several generations, a real, substantively
different trajectory than v9's flat 0.50 -- the search is no longer collecting the
reward for free. (v10's final best candidate did land back on 0.50, but -- see
below -- for a different, real reason this time, not the same loophole: this
candidate's actual failure mode happens before ramjet ignition timing is even
reached, so the search had no signal left to push it away from 0.50 specifically.)

v10's own final score (-5508.5) is *lower* than v9's (-4360.2) -- expected and correct,
not a regression: v9's higher score was partly the ~300-point (weighted) reward
collected for free by the loophole; closing it makes the honest score reflect the
real, harder problem.

**Real result, verified the same way (full-fidelity `simulate_mission` re-run via
`scripts/gate3_check_candidate.py`, not trusted from the search's own fast-fidelity
score):** stall margin still holds (+0.2507 nominal, +0.1617 adverse -- both positive,
confirming that fix is robust across different candidates, not a fluke of v9's
specific geometry). But this candidate never reaches `dive` or `ramjet_accel` at all --
`phases: ['pulsejet_climb']`, terminating with `pulsejet_fuel_exhausted_before_top_of_
climb`. Its `loaded_fuel_mass_kg = 2.540` sits right at the search's own lower bound
(2.50) -- the search pushed fuel mass down (helping `_MASS_MARGIN_REWARD_PER_KG` and,
indirectly, stall margin via lower vehicle mass) far enough that the vehicle
structurally cannot carry enough fuel to finish climbing, let alone reach dive or
ramjet transition.

**Reading**: a real, third structural tension, distinct from both the stall-margin
gap and the ramjet-transition-thrust trough already documented above. The scoring
function currently has no term that distinguishes "ran out of fuel before finishing
the climb phase" from any other way of falling short of peak Mach -- both just read as
a lower `peak_mach_reached`, so the mass-margin reward's pull toward less fuel is not
being offset by a strong enough signal that *some* minimum fuel load is a hard
prerequisite, not a smooth tradeoff. Worth a genuine scoring-function look (e.g., an
explicit penalty for terminating a phase specifically via fuel exhaustion, distinct
from the smooth peak-Mach credit), not resolved in this session -- flagging clearly
rather than guessing at a fix without evidence a specific change helps.

## Gate 3 baseline (post all this session's propulsion fixes): still fails, real reasons

Direct re-run (`scripts/gate3_check.py`, full fidelity, `shared_nozzle_candidate_b.yaml`,
`simulate_mission`'s own default climb/dive/zoom parameters -- not a design-search
result) after every propulsion fix in this document landed:

**Nominal** (345.5s to compute): peak Mach 0.798 (target not met), never reaches Mach 1
(0.00s time above Mach 1), phases only `['pulsejet_climb', 'dive']` --
`dive_floor_reached_without_ramjet_lightoff_mach`: the dive burns through its altitude
margin before ever reaching the ramjet's configured lightoff Mach, so ramjet_accel never
starts. `minimum_stall_margin_fraction = -0.3773` -- **essentially identical to the
-37.7% figure `docs/level0_feasibility_bounds.md` reported on 2026-08-07, before any of
this session's propulsion work.** Confirms directly, not assumed: the stall-margin
failure is a pure aero/lift-area problem, structurally untouched by everything this
session did to pulsejet/ramjet thrust. `reference_area_m2` in
`shared_nozzle_candidate_b.yaml` is still 0.0896 m^2, the exact value flagged there as
~3.6x too small at the configured sled-release speed and max mass.

**Adverse** (0.2s -- reused the nominal run's cached pulsejet-table points, scaled by
the scenario's own multiplier, not a separately-run 11-point table): peak Mach only
0.125, phase stays `pulsejet_climb` the whole run until
`mission_time_cap_reached_before_landing` (900s simulated flight time with barely any
climb). `minimum_stall_margin_fraction = -1.0` -- the vehicle spends the entire adverse
run below its own 1g stall speed.

**Reading**: two genuinely separate problems, not one. (1) Stall margin is an aero
sizing problem -- `reference_area_m2` (or `wing_area_scale_factor` in a search context)
and/or `sled_release_speed_m_per_s` need to move, not propulsion. (2) Even where thrust
is adequate to climb (nominal reaches M=0.80), the *dive-to-ramjet-lightoff* strategy
with `simulate_mission`'s default climb/dive/zoom angles doesn't thread the needle
before running out of altitude -- a trajectory-shaping problem, distinct from both the
aero sizing problem and from this session's already-documented ramjet-transition-thrust
finding (which used a specific, non-default dive strategy). Closing Gate 3 needs both
addressed, most practically through the existing `wing_area_scale_factor`/
`sled_release_speed_m_per_s`/`climb_angle_deg`/`dive_angle_deg`/`dive_entry_mach` search
variables `optimizer.py` already has -- not a new propulsion fix.

## design-optimize v11: fuel-exhaustion penalty confirmed working, but a new, more severe blocker -- and it turns out to be architectural, not tunable

Re-ran the identical search (population 20, generations 40, seed 0) with
`_PREMATURE_FUEL_EXHAUSTION_PENALTY` in place --
`results/generated/design_optimize_v11/checkpoint.json`. Verified the same way as
v9/v10 (full-fidelity `simulate_mission` re-run via `scripts/gate3_check_candidate.py`,
not trusted from the search's own fast-fidelity score):

- **`loaded_fuel_mass_kg` is still pinned at its 2.50 kg lower bound**, yet this
  candidate's mission does *not* exhaust fuel early -- the new penalty changed *which*
  low-fuel candidate the search settles on, not the fact that it settles on minimal
  fuel. Not a bug: this candidate's specific fuel split/profile apparently avoids the
  v10 failure mode while still minimizing fuel for mass-margin points, which is exactly
  what the penalty was supposed to allow (penalize the *failure*, not the low fuel load
  itself).
- **`minimum_lightoff_test_mach` moved to the *opposite* bound from v9/v10: 1.00, its
  ceiling**, not a self-referential scoring artifact this time (`_REACHED_RAMJET_
  IGNITION_REWARD` compares against the fixed `minimum_self_sustaining_mach`, not this
  variable -- see v9's fix above). Both scenarios now terminate with
  `dive_floor_reached_without_ramjet_lightoff_mach`: nominal peak Mach 0.7232, adverse
  0.6448, neither ever entering `ramjet_accel`. **GATE 3: FAIL, both scenarios.**

**Why does the search prefer never transitioning to ramjet at all, when Gate 3 requires
peak Mach 1.10 and pulsejet alone has never once exceeded ~0.8 in any run this
session?** Because `_PEAK_MACH_PROGRESS_REWARD_PER_MACH` and `_MISSED_PEAK_MACH_
PENALTY`/`_DURATION_MISSED_PENALTY` are all identical whether a candidate stalls out
via `dive_floor_reached_without_ramjet_lightoff_mach` at Mach 0.72 or via `ramjet_net_
thrust_nonpositive_during_acceleration` after actually attempting the transition and
decelerating back down. The only thing that differs between the two strategies is the
*peak* Mach actually reached -- and a direct margin sweep (below) confirms staying
pulsejet-only and never transitioning genuinely reaches a higher peak Mach than
transitioning does, at this candidate's throat sizing. **The search is not confused or
stuck; it is correctly finding that entering ramjet mode is net-harmful given the
geometry it has to work with.**

### Direct ramjet net-thrust-margin sweep confirms it, and finds the real lever: throat size

Built a standalone diagnostic (`evaluate_ramjet` + `evaluate_total_drag`, scenario
thrust/drag multipliers applied the same way `trajectory.py` does, altitude 4500 m) on
v11's exact winning candidate (`throat_diameter_m=0.1113`, near its 0.110 m search
floor):

| Scenario | Positive-margin Mach points (0.55-1.10 sweep) |
|---|---|
| Nominal | only 0.75, 0.80 (+9.9 N, +3.9 N -- a razor-thin window) |
| Adverse | **none** -- every point from -72 N (M=0.55) to -427 N (M=1.10) |

Re-running the same sweep with only `throat_diameter_m` increased to 0.190 (the
search's current ceiling), everything else held fixed:

| Scenario | Result at throat=0.190 |
|---|---|
| Nominal | **positive margin at every tested point, 0.55-1.10** (+9.7 N to +610.7 N) |
| Adverse | still negative everywhere, but the deficit shrinks by roughly an order of magnitude at the high-Mach end (-16.2 N at M=1.10, vs -426.8 N at the actual 0.111 m throat) |

A finer throat sweep (0.110-0.230 m, nominal margin at M=0.80/0.90/1.10, adverse at
M=0.80/1.10, plus a `feasibility.py` packaging check at each point) shows the
transition is smooth, not a cliff -- margin turns positive around throat~0.13 m at
mid-Mach and keeps improving monotonically with throat size in every column checked,
with packaging only starting to fail beyond ~0.195-0.200 m at this candidate's current
0.2145 m body diameter. Pushing throat and body diameter up together (0.220-0.250 m
throat, 0.240-0.260 m body) closes the top of the adverse band (M=0.90-1.10 all turn
positive) but **never closes M=0.65-0.70** -- the deficit there actually *worsens* as
body diameter grows (-158 N at throat/body=0.220/0.240, -188 N at 0.250/0.260),
because the larger body's drag penalty at that specific Mach outpaces the extra
thrust the wider throat buys there. This is a real, separate finding from the
high-Mach-margin one: **there is no single throat/body-diameter point that closes the
entire adverse Mach band at once** -- the trough that opens right at the Mach where a
transition would have to happen is structurally worse than either edge of it.

### The throat lever is real for the static margin, but costs more on the pulsejet side than it buys -- confirmed by direct mission re-run, not assumed

Constructing v11's winning candidate with `throat_diameter_m` forced to 0.190 (the
value that made *nominal*'s static margin fully positive above) and sweeping
`minimum_lightoff_test_mach` from 0.60-0.80 to try to actually use that window: **every
one of these candidates scores *worse* at the full mission level than the original**,
nominal peak Mach falling from 0.7232 to 0.436 and triggering a new
`stall_margin_violated` failure that wasn't present before; adverse now exhausts fuel
before even finishing the climb. The static ramjet-margin win is real, but it never
gets used -- the vehicle can no longer climb far enough to reach the Mach range where
it would apply.

A finer throat sweep (0.111-0.150 m, same candidate otherwise unchanged) shows this is
not a cliff either: nominal peak Mach falls steadily and substantially with even small
throat increases (0.723 at 0.111 m -> 0.708 at 0.120 m -> 0.687 at 0.130 m -> 0.664 at
0.140 m -> 0.616 at 0.150 m) -- there is no free increment of throat growth available;
every millimeter is paid for immediately in pulsejet-phase reach, well before the
ramjet-margin benefit becomes usable.

**Isolated the mechanism directly**: re-ran the throat=0.150 m candidate with its
`flight.initial_mass_kg` forcibly reset back to the throat=0.111 m baseline (18.433 kg)
before calling `simulate_mission`, to separate "bigger throat costs more structural
mass" from "bigger throat changes the pulsejet's own shared-exhaust efficiency."

| Configuration | Mass | Nominal peak Mach |
|---|---:|---:|
| throat=0.111 m (baseline) | 18.433 kg | 0.7238 |
| throat=0.150 m (real mass) | 19.902 kg | 0.6158 |
| throat=0.150 m (mass forced back to baseline) | 18.433 kg | 0.6245 |

Forcing mass back to baseline recovers almost none of the loss (0.6158 -> 0.6245,
~0.009 Mach out of a ~0.10 Mach gap). **The pulsejet-phase degradation from growing the
throat is overwhelmingly a shared-exhaust-geometry effect, not a mass-model
artifact.** `pulsejet.py` and `ramjet.py` both consume the exact same `NozzleConfig`
(`throat_area_m2`, `exit_area_m2`) -- this is a real, single physical nozzle shared
between the two engine modes (matching the vehicle's own switchable-engine concept,
`shared_nozzle_candidate_b.yaml`'s name, and `RamjetConfig`'s patent-lineage
docstring), not an incidental modeling gap that a mass-calibration fix could paper
over.

Checked whether `exit_to_throat_area_ratio` (a free variable, 1.02-1.30, independent
of `throat_diameter_m`) could buy ramjet margin without this pulsejet cost: it cannot
-- sweeping it at fixed throat shows a much weaker effect on ramjet margin than throat
itself (deficits stay in the -120 to -470 N range across the whole adverse sweep at
every ratio tested) while *still* eroding nominal peak Mach as it rises (0.7278 at
1.02 -> 0.6987 at 1.30), and mass is unaffected by this variable at all -- so it is
strictly worse than doing nothing, not a hidden free lever.

### Conclusion: this is the shared-nozzle compromise flagged at the start of this document, now confirmed structural at the full mission level

The very first "Candidate B numerical balance" section of this document flagged "That
shared-nozzle compromise is now explicit and must be tested in the coupled trajectory
rather than optimized at one operating point." It has now been tested, directly and
repeatedly, at the coupled mission level, across v7 through v11 and this section's own
targeted diagnostics: **one shared throat cannot simultaneously give the pulsejet
enough exhaust efficiency to climb/accelerate to a useful handoff Mach *and* give the
ramjet enough capture/expansion area to hold positive net thrust once it gets there.**
Every lever available to `optimizer.py`'s current 13 search variables (throat size,
exit ratio, fuel split, lightoff threshold, release speed, climb/dive angles, wing
area, body diameter) has now been swept, individually or in the combinations most
likely to help, and none closes the gap:

- Staying pulsejet-only (what v11's search rationally converged to) tops out at
  Mach 0.72-0.80 -- far short of the 1.00 minimum / 1.10 target peak Mach, and cannot
  reach it by construction, since pulsejet's own thrust curve (this document's earlier
  sections) has no mechanism to exceed roughly this range.
- Growing the shared throat enough to make ramjet's margin positive at the Mach range
  where transition would need to happen costs more pulsejet-phase reach than it
  recovers, and even at generous throat/body sizes the adverse scenario's
  transition-Mach trough (~0.65-0.70) does not close.

This is not a search-tuning gap, a scoring-function bug, or a bound picked too
conservatively -- it is a genuine architectural conflict between the two engine modes'
optimal nozzle sizing, now demonstrated with a controlled mass-vs-geometry isolation
rather than inferred. Closing it for real would need one of: (a) a nozzle geometry that
is no longer shared/fixed between modes (a real hardware architecture change, out of
scope for a search-variable adjustment), (b) relaxing the Gate 3 peak-Mach/duration
requirement itself, or (c) a substantially different climb/release strategy not yet
tried that gets pulsejet-only performance close enough to the target that only a small,
low-cost throat increment is needed to finish the job -- not yet found, and not
guaranteed to exist given pulsejet's own thrust curve plateaus well below Mach 1
regardless of release speed or climb angle in every case measured so far. Recommend
treating Gate 3 closure under the current single-shared-nozzle architecture as blocked
pending a user decision on which of these three to pursue, rather than continuing to
launch further identical-shape `design-optimize` searches that are expected, on this
evidence, to reconverge to the same avoid-ramjet local optimum.

**Path (c) checked directly and found to have no accessible headroom.** Perturbed
v11's winning candidate (throat held at its floor, i.e. staying in the pulsejet-only
regime this search already found best) one variable at a time away from its converged
value: `sled_release_speed_m_per_s` to its true 121.3 m/s ceiling (worse: nominal
0.7238 -> 0.6173), climb angle to 5 deg and 30 deg (5 deg much worse; 30 deg
essentially ties baseline at 0.7227), `wing_area_scale_factor` to 1.0 and 3.0 (both
worse), `ramjet_fuel_fraction` down to 0.05 to give pulsejet nearly all the fuel (no
better -- fuel was never the constraint here), `loaded_fuel_mass_kg` to its 9.0 kg
ceiling (adverse improves slightly to 0.668, nominal drops to 0.639, a wash, not a
net gain), and `dive_entry_mach` to 0.30 (much worse). None beats v11's own converged
values; several confirm the search had already found a near-local-optimum for the
pulsejet-only strategy. The gap to the 1.00-1.10 Mach target (0.28-0.38 Mach) is an
order of magnitude larger than any single-variable perturbation moves the peak Mach
achieved. This rules out "the search just didn't push some pulsejet-side lever hard
enough" as an explanation -- reinforcing that (a) or (b) above are the only realistic
ways forward, not further tuning within the current variable set.

## minimum_lightoff_test_mach is now derived, not searched (2026-08-10)

Following the NASA EngineSim comparison work (`docs/ramjet_enginesim_comparison.md`)
and the resulting inlet-recovery/real-gas rework, `ramjet.py`'s raw (ungated)
net-thrust-vs-Mach curve was found to have a real, physically-genuine negative
trough right where the fixed nozzle first starts passing flow -- see
`evaluate_ramjet`'s current docstring for the mechanism (momentum drag scales
linearly in the vanishing exit flow, gross thrust scales quadratically). The
first fix (same session) made `evaluate_ramjet` report zero thrust below
`minimum_lightoff_test_mach` instead of that trough. The user's follow-up
request: stop hand-picking `minimum_lightoff_test_mach` (0.80 in every config)
and derive it instead -- specifically, as the Mach at which the engine's own
net thrust first turns positive and stays positive (engine-only, not relative
to vehicle drag -- confirmed by direct clarifying question, since the two
readings produce very different numbers and this codebase's own prior research
below is directly relevant to which one matters).

`ramjet.py`'s new `derive_lightoff_mach` scans the ungated cycle solve
(extracted into `_solve_ramjet_cycle`) from Mach 0 up to
`minimum_self_sustaining_mach` (still hand-configured, unaffected), finds the
*last* Mach with non-positive net thrust (correctly skipping past the trough
rather than stopping at its first, spurious zero-crossing), then bisects to a
precise root. For every config in this repo (propulsion-relevant fields are
currently identical across `reference_case.yaml` and both
`shared_nozzle_candidate_*.yaml`) this computes to **Mach ~0.494** -- well
below the hand-picked 0.80, and below the 0.50 lower bound the search variable
used to have.

**Directly relevant tension, worth recording rather than glossing over:** the
"Ramjet lightoff Mach" investigation above (optimizer.py's former search
variable) found the *opposite* direction helped -- delaying the handoff
(raising the threshold toward 0.87-1.00) improved adverse-scenario performance,
because ramjet thrust stays below vehicle *drag* through most of Mach 0.5-1.1
even where it's engine-net-positive. Deriving purely from engine-positive
thrust (this change) does not reproduce that finding -- it's answering a
different, narrower question (can the engine produce any net thrust at all)
than the one that investigation was actually optimizing (can the vehicle
usefully accelerate on ramjet power). This was surfaced to and confirmed by
the user before implementing; it is an intentional scope choice, not an
oversight, but a future full-mission re-run under this new derivation should
be expected to show a real, different outcome than the search runs recorded
above -- not treated as a regression to chase back toward 0.80-0.87.

Also confirmed directly (probing `derive_lightoff_mach` across parameters):
within `optimizer.py`'s actual search space, this derived value turns out to
be **invariant** for the current bounds -- neither `throat_diameter_m` nor
`exit_to_throat_area_ratio` (in its searched range, 1.02-1.30) shift the
crossing point at all, since the unchoked-regime exit conditions right at that
crossing are pressure-ratio-driven, not area-driven (mass flow and gross
thrust both scale by the same area factor, so the *sign* of net thrust is
area-independent; only its magnitude scales). What does move it --
`selector.ramjet_total_pressure_recovery` (0.377 at 0.98 recovery vs. 1.09 at
0.80 recovery), `target_combustor_exit_temperature_k`, `combustor_efficiency`
-- are none of them current search variables. A sufficiently large
`exit_to_throat_area_ratio` (~3.0+, well outside the current 1.02-1.30 bound)
does make the derivation infeasible (`ValueError`, correctly propagated and
caught by `evaluate_design`'s existing infeasibility handling) by eliminating
the positive-thrust crossing entirely, rather than moving it.

Consequence for the search: `minimum_lightoff_test_mach` dropped from a 13th
search variable to zero -- it's derived per-candidate in
`apply_design_variables` from that candidate's own nozzle/selector/ramjet/fuel,
which also permanently closes the self-referential-gaming bug class documented
above (the search converging on its own lower bound) at the root, since there
is no longer a free choice here at all.

## PULSEJET_MODE's guarded pulsejet-km-first dispatch, and a 4.1x search speedup (2026-08-11)

Two independent pieces of work, same session, done in that order.

**1. `PULSEJET_KM_MODE` (wired earlier today, additive-only) made
architecturally primary for `PULSEJET_MODE` too, guarded.**
`propulsion_map.py`'s new `_pulsejet_mode_point` tries pulsejet-km first
whenever a candidate provides `pulsejet_km_engine_config`, falling back to
the native `pulsejet.py` simulator only when pulsejet-km itself signals it
cannot answer (not converged, outside its own validated Mach envelope, not
a genuine `STABLE_LIMIT_CYCLE`, non-positive net thrust, or a raised
exception -- zero-git-commit research code with no version pinning).
Deliberately **not** a magnitude cross-check: pulsejet-km's own
architecture.md documents its thrust reading ~8x low even for results that
clear every one of those guards, and there is no principled way to correct
for a systematic bias like that from inside a fallback heuristic -- that is
real physics/calibration work (in progress in the sibling `pulsejet-fp`
repo as of this session, not this repo). Every pulsejet-km-sourced point
instead carries an explicit `pulsejet_km_thrust_known_low_bias_...`
validity flag so the bias is visible, not silently absorbed. In practice
this changes nothing about any real candidate today: none of
`shared_nozzle_candidate_a/b.yaml`/`robustness_candidate_b.yaml` carry a
`pulsejet_km:` section, so `_pulsejet_mode_point` resolves straight to the
native path for every actual vehicle evaluation -- confirmed empirically,
not just by inspection (`_run_pulsejet_km_query` made zero calls in the
profiling run below). Full test suite green both before and after (two
separate runs), plus three new regression tests
(`PulsejetModeGuardedPrimaryDispatchTests`) pinning the trust/reject/
no-config paths. See `docs/design_workflow.md`'s Gate 2 entry for the
policy-level writeup.

**2. Profiled `design-optimize` and found nearly all of its cost in one
place.** `population=4, generations=2` (12 `evaluate_design` calls) took
136s wall-clock under `cProfile`; 96% of that was
`trajectory.py:_pulsejet_static_thrust_table` building the pulsejet Mach
table via the unsteady simulator, and within *that*, 64% of
`PulsejetSimulator.step()`'s own cost was `compressible.py`'s
`fixed_cd_nozzle` -- specifically its Mach-from-area-ratio bisection
solvers. Two fixes, both verified behavior-preserving by a full green test
suite re-run (separate from the pulsejet-km re-run above):

- `pulsejet.py`'s `_ignite_if_ready` was re-deriving `pressure_pa`/
  `temperature_k` from state that `step()` had *already* computed
  identically moments earlier (confirmed: nothing between the two reads
  mutates `state.total_mass_kg`/`internal_energy_j`) -- now computed once
  and passed in.
- `compressible.py`'s three `range(50)`-iteration bisections (the two
  named Mach-from-area-ratio solvers, plus `fixed_cd_nozzle`'s internal
  shock-position solve, which nests calls to the first two with an
  ever-different, non-cacheable midpoint each of its own 50 iterations --
  up to 2,500 nested evaluations per `fixed_cd_nozzle()` call in that
  regime) cut to 30. 50 iterations was already well past both this
  model's own precision needs and, on these functions' bracket widths,
  float64's own resolution floor (2^-50 ~ 1e-15) -- 30 still resolves to
  roughly 1e-9, orders of magnitude past the model's own +/-17% reported
  thrust uncertainty and every `assertAlmostEqual` tolerance in this
  suite.

Re-timed the identical `population=4, generations=2` workload cleanly
(no other load on the machine) after both fixes: **33.1s, a 4.1x
speedup**, i.e. ~2.76s/evaluation versus the original ~11.3s/evaluation.
Two lower-value opportunities were identified and deliberately deferred
rather than implemented tonight: hoisting `fixed_cd_nozzle`'s onset-Mach
lookup (already effectively `@lru_cache`-covered for the truly-invariant
outer call, so the residual win is only the cache-lookup overhead itself)
into `PulsejetSimulator.__init__`, and replacing bisection with
Newton-Raphson (a bigger win, but an algorithm change, not just a
speed-tuning one) -- both real, both left for a future pass.

**Consequence for search sizing**: a default `population=20,
generations=30` run that previously needed on the order of 1.5-2 hours now
completes in roughly 25-30 minutes. A much larger, `design_optimize_v12`
search (`population=32, generations=300`, seed 0, on top of every physics
fix landed today -- the MIL-E-5008B inlet correlation, real-gas cp/gamma,
the derived (not searched) `minimum_lightoff_test_mach` above, and this
section's speedup) was launched to use that headroom -- see its checkpoint
under `results/generated/design_optimize_v12/` for the result; treat
`design_optimize_v9`/`v10`/`v11` (2026-08-08/09) as stale for any
conclusion touching lightoff Mach, inlet recovery, or real-gas effects,
the same way this document has repeatedly flagged earlier runs stale
after each physics correction.

## 2026-08-12: ramjet-fp becomes RAMJET_MODE's guarded primary (Gate 2) and the mission solver's ramjet source (Gate 3)

**What changed.** The sibling first-principles ramjet model (`ramjet-fp`,
unsteady quasi-1D HLLC relaxation: Rankine-Hugoniot inlet, WSR flameholder
with emergent Damkohler blow-off, resolved choking/thermal choking,
eq.30/32 dual-thrust verification -- see `ramjet-fp/architecture.md` and
`docs/derivation.md` there) is now:

1. **Gate 2**: `propulsion_map._ramjet_point` dispatches to ramjet-fp as
   its guarded PRIMARY via the new `ramjet_fp_bridge.py` (direct flowpath
   geometry mapping from the case config; flameholder scaled from the
   validated RJ-1 proportions and capped so the gutter never chokes ahead
   of the shared nozzle throat -- both flagged). ``blown_off`` is a USABLE
   answer: the point carries the cold-throughflow drag,
   `self_sustaining_status=False`, and visible flags. Fallback to the
   native 0D `evaluate_ramjet` is flagged
   (`ramjet_fp_primary_rejected_fell_back_to_native`), and
   `DOUGLAS_DART_DISABLE_RAMJET_FP=1` is the documented kill-switch (the
   legacy test suite sets it in conftest, exactly like the pulsejet one).
   The native model's MIL-E-5008B recovery schedule, configured combustor
   efficiency, and discharge coefficients are exactly what the primary
   replaces with derived physics; achieved recovery is now an OUTPUT
   (scenario recovery overrides do not apply to FP points -- flagged).
2. **Gate 3**: `trajectory.simulate_mission`'s ramjet_accel/mach_hold
   phases read a lazy 0.1-Mach x 1500-m bilinear table over the memoized
   FP query (~12-16 transient runs per unique engine geometry, shared
   across scenarios in-process) instead of a per-step call -- the same
   table-not-per-step pattern as the pulsejet static table. Blown-off
   cells surface as `ramjet_fp_flame_unstable_during_ramjet_phase` in the
   run status.

**Model-fidelity change, and a design-relevant one.** ramjet-fp's campaign
(sibling architecture.md #3-#8) found: (a) at the configs' ramjet
`target_equivalence_ratio: 0.60` a fully-premixed flame holds at NO Mach --
every FP-sourced RAMJET_MODE point at phi 0.60 is `blown_off` with
negative (drag-only) thrust, so Gate 3 missions now fail in the ramjet
phase for the honest reason that the engine as configured cannot burn;
(b) near-stoich fueling is required, with a Mach- and altitude-dependent
lean limit (phi_min 0.87 at M 0.4 SL, ~1.00 in the M 0.6-0.9 oscillation
pinch); (c) cold relight has an altitude-dependent no-go hole (M 0.7-0.8
at 1500 m widening to M 0.8-1.1 at 6000 m) while a continuously carried
flame transits the band at <=~1000 m and survives climb at M >= 1.1 --
i.e. the mission profile wants transonic acceleration LOW, then a lit
supersonic climb to the 4500 m speed run; and (d) every lit point is a
bounded chugging limit cycle (cycle-mean reported; amplitude flagged).
The phi finding means the ramjet `target_equivalence_ratio` config value
is now a live design decision with a physics-backed viability boundary
(`ramjet_fp.minimum_stable_phi` is the queryable schedule); 0.60 was
backed out of a fixed-1900K-exit-temperature assumption that the
first-principles model does not support.

**Verification.** 10 new bridge unit tests (spec round-trip against the
live sibling RJ-1 reference, gutter/throat cap on candidate B, usability
guard incl. blown-off-is-usable, dispatch prefer/fallback/crash/kill-switch
paths) + 2 live end-to-end tests (Gate 2 point and Gate 3 table at fast
fidelity); full legacy regression suite green with the kill-switch
defaulted in conftest (native path byte-identical). `gate3_check.py` now
reports its ramjet source and runs the FP table at full fidelity.

## 2026-08-12 (later): phi is now an OUTPUT of the ramjet query (user directive)

"Fly a ramjet of this size and output the fuel consumption and mixture
ratio." The FP primary no longer reads `ramjet.target_equivalence_ratio`
at all: `run_ramjet_fp_operating_query` calls the sibling's
`ramjet_operating_point`, which bisects the leanest viable mixture at the
queried (M, alt), adds a stability margin, caps at stoichiometric, and
VERIFIES the selection with its own transient run (4-9 transients per
point, memoized). `PropulsionMapPoint` gains
`ramjet_required_equivalence_ratio` -- the fuel-control design value; the
Gate 3 lazy table's corners are now operating points, so the mission
solver flies the self-selected mixture schedule. Engine-out (no viable
mixture at any phi in [0.5, 1.0]) reports fuel CUT to zero plus
cold-throughflow drag -- previously a blown-off point still carried the
configured phi's fuel flow, which double-charged a dead engine.
The configured `target_equivalence_ratio` now feeds only the native 0D
fallback path. The throttle-schedule design curves (required phi + fuel
flow vs M at 0/900/1500/3000/4500 m) are generated by
`ramjet-fp/scripts/throttle_schedule.py` (out/throttle_schedule_rj1.*).
Verification: 10 bridge unit tests updated (phi-as-output mapping,
engine-out fuel cut, dispatch paths), 2 live end-to-end tests (the M=1.1
SL point self-selects phi in [0.90, 1.00] and lights -- positive thrust
from a config whose 0.60 setting could never burn), full regression
suite re-run.
