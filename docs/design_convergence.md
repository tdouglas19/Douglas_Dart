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
