# Pulsejet model audit vs. external reference models (Khrulev & Muntyan, NACA)

Audit only, per the request that produced it -- no code changed. Cross-checks
`pulsejet.py` (the unsteady simulator, the model actually driving Gate 2/3
today) against the two uploaded research documents (`reference_model_cross_check_audit.md`,
the compass research summary) comparing this codebase to pulsejet-sim.com
(Khrulev & Muntyan's peer-reviewed model) and pulse-jets.com's primary
documents (NACA TM-1131, GALCIT/JPL). Findings below quote the actual current
code, not a description of it.

## Gap 1 -- No ignition-delay / heat-release phasing

**Present as described. Confirmed by direct quote.**

`_ignite_if_ready()` (`pulsejet.py:457-499`) computes `burnable_fuel_kg` and
adds its full heat content to `state.pending_heat_release_j` in the same
instant the four-condition trigger clears:

```python
state.pending_heat_release_j += (
    burnable_fuel_kg
    * self.fuel.lower_heating_value_j_per_kg
    * config.combustion_efficiency
)
state.burn_time_remaining_s = config.burn_duration_s
```

`step()` (`pulsejet.py:614-623`) then meters that fixed pool out linearly over
`config.burn_duration_s`:

```python
if state.pending_heat_release_j > 0.0 and state.burn_time_remaining_s > 0.0:
    burn_fraction = min(time_step_s / state.burn_time_remaining_s, 1.0)
    heat_release_j = state.pending_heat_release_j * burn_fraction
```

`burn_duration_s` is a fixed `PulsejetConfig` value (`configs/*.yaml`,
currently 0.002 s per the pulsejet cycle-timing sweep in
`docs/design_convergence.md`) -- a constant, not a function of chamber
pressure, temperature, or Mach. There is no delay between the ignition
trigger firing and heat release starting, and no mechanism (Arrhenius or
otherwise) that would shift when combustion begins relative to when fresh
air actually arrived. Khrulev & Muntyan's model imposes exactly this shift
(`m ~= 0.3` cycle-fraction delay, `theta ~= 0.1` temperature-ratio term,
explicitly in relative/normalized form after a direct Arrhenius calculation
destabilized their own solver).

**New inputs needed**: two dimensionless correction constants (`m`, `theta`
in Khrulev's notation, or equivalent), fit or reasoned by analogy -- not a
hardware measurement. This is the same category as `PulsejetCorrectionCoefficients`'
existing fit constants in `pulsejet_closed_form.py`.

**Priority reasoning**: directly plausible for (c) the lightoff-Mach
feasibility gap and (a) flat Mach-dependence, per the audit doc's own
argument -- if heat release stays locked to ignition-trigger time regardless
of how mass-flow timing shifts with Mach, the model has no mechanism for
combustion phasing to respond to flight speed. **High priority.**

## Gap 2 -- No exhaust/tailpipe inertial ("liquid piston") treatment

**Present as described. Confirmed by direct quote, and the asymmetry with the inlet side is stark.**

The inlet side carries real ODE state between timesteps
(`pulsejet.py:539-552`):

```python
mass_flow_rate_of_change_kg_per_s2 = (
    self.inertance_duct_area_m2 / self.inertance_duct_length_m
    * (self.inlet_total_pressure_pa - chamber_pressure_pa)
)
state.inlet_mass_flow_kg_per_s = max(0.0, state.inlet_mass_flow_kg_per_s
    + mass_flow_rate_of_change_kg_per_s2 * time_step_s)
```

The exhaust side has no equivalent. `fixed_cd_nozzle` (`pulsejet.py:560-569`)
is called fresh every step with only the *current instantaneous* chamber
state:

```python
nozzle_result = fixed_cd_nozzle(
    chamber_pressure_pa, chamber_temperature_k, self.atmosphere.pressure_pa,
    self.nozzle.throat_area_m2, self.nozzle.exit_area_m2,
    self.nozzle.discharge_coefficient, config.gamma, config.gas_constant_j_per_kg_k,
)
```

No `state.exhaust_mass_flow_kg_per_s` (or equivalent momentum-carrying
variable) exists anywhere in `PulsejetState`. This is a purely quasi-steady,
algebraic solve -- exactly the audit's description, and structurally the
same category of issue already found and smoothed on the ramjet side
(`compressible.py`'s nozzle-onset-smoothing fix, `docs/design_convergence.md`).

**New inputs needed**: an exhaust/tailpipe length and cross-section, plus
friction/local-resistance coefficients (`xi_T`, `xi_c` in Khrulev's
notation). **None of this exists in `config.py` today** --
`quarter_wave_resonance_frequency_hz`'s only current call site
(`pulsejet_closed_form.py:709`) uses `nozzle.exit_area_m2**0.5` as an
explicitly-labeled `# coarse geometric proxy, diagnostic only`, not a real
tailpipe length. The inlet side already has a working precedent for this
exact kind of parameter: `_inertance_inlet_duct_area_m2`/
`_inertance_inlet_duct_length_m` (`pulsejet.py:138-158`) derive inlet duct
geometry from `selector.circular_intake_diameter_m` via fixed ratio
constants (`_INLET_DUCT_OPEN_FRACTION_ESTIMATE`, etc.) -- design-choice
estimates, not hardware measurements. An exhaust-side equivalent
(`_EXHAUST_DUCT_LENGTH_TO_DIAMETER_ESTIMATE` or similar, tied to
`nozzle.throat_diameter_m`/`body_length_m`) would follow the same pattern.

**Priority reasoning**: the audit and compass doc both flag this as the
single change most likely to make Mach-dependence emerge naturally and to
remove nozzle-choking-type discontinuities on the exhaust side -- the same
category of problem (quasi-steady solve producing a sharp cusp) already
confirmed and fixed once this session on the ramjet nozzle. **High priority,
likely the highest-leverage single change of the four.**

## Gap 3 -- Valve model: idealized instantaneous check valve

**Present as described, and confirmed to be the same idealization NACA
TM-1131 itself uses.** `pulsejet.py:539-558`:

```python
if self.inlet_total_pressure_pa > chamber_pressure_pa:
    # Idealized check valve: open, integrate the inertance ODE
    ...
else:
    # Closed: pressure differential opposes inflow. A check valve
    # prevents backflow and, idealized as instantaneous, also removes
    # the duct's stored momentum ...
    state.inlet_mass_flow_kg_per_s = 0.0
```

Binary open/closed on pressure-differential sign, no petal mass, stiffness,
or motion state anywhere in `PulsejetState`. This matches the audit's own
finding that this is defensible, not a corner cut -- NACA TM-1131
(Schultz-Grunow) explicitly disregards valve spring action too, and
Khrulev's own quasi-stationary cantilever model is validated only to
10-15% over the full dynamic model. `docs/design_convergence.md`'s
"Inflow phase-lag mismatch" section already reached the same conclusion
independently this session, and documents a prior, explicit decision *not*
to add full mass-spring petal dynamics because it needs unmeasured hardware
parameters (petal mass, spring constant).

**Important distinction the audit itself raises**: Khrulev's *quasi-stationary*
cantilever-beam model (deflection proportional to `delta_p / stiffness`, using
elastic modulus `E`, petal thickness/width/length) is not the same rejected
approach -- it needs only a known material property (spring steel/shim
`E ~= 190-200 GPa`) and petal geometry, which is a design choice this project
controls, not a hardware measurement it's missing. **No petal geometry field
exists anywhere in `config.py` today** -- `SelectorConfig` has
`open_fraction` (the pulsejet/ramjet area-split convention) but nothing
describing valve petal count, thickness, width, or length. The audit doc's
"~75% of circumference" reference is `pulsejet.py:124`'s
`_INLET_DUCT_OPEN_FRACTION_ESTIMATE = 0.75` -- found on a second pass
(source, not docs); it is explicitly labeled a "first-pass geometric
estimate" from a decision doc referenced but not present in this repo
(`revised_inertance_plan_no_beam_mechanics.md`, same status as
`gate2_pulsejet_fork_decisions.md`/`gate2_refill_phase_decision.md` cited by
`pulsejet_closed_form.py` -- conversation artifacts from earlier sessions,
not persisted here). It describes the *inlet* duct's open circumference
fraction, not petal count/thickness/width/length -- still not a valve
petal-geometry parameter, so Gap 3's "no petal geometry field exists" finding
stands, but the source of the "~75%" figure itself is now correctly
identified rather than reported missing.

**Priority reasoning**: flagged as likely to help regularize the near-singular
low-Mach refill equilibrium (issue (b)) by replacing the binary valve with a
continuous `delta_p`-dependent effective area. **Medium priority** -- real
and legitimate, but the current idealization is independently defensible
(matches a primary NACA source), so this is a refinement, not a correction of
an error, unlike Gaps 1 and 2.

## Gap 4 -- Side vs. straight intake: verify momentum-drag/mass-flow terms carry Mach-dependence independent of the in-chamber cycle

**Checked directly, and the mechanism the audit hoped for is confirmed present.**
`pulsejet.py:655-659`:

```python
gross_thrust_n = nozzle_result.gross_thrust_n * exhaust_scale
inlet_momentum_drag_n = (
    inlet_air_mass_flow_kg_per_s * self.freestream_velocity_m_per_s
)
net_thrust_n = gross_thrust_n - inlet_momentum_drag_n
```

`freestream_velocity_m_per_s = mach * atmosphere.speed_of_sound_m_per_s`
(`__init__`, `pulsejet.py:368-370`) -- a direct, explicit Mach dependence in
the net-thrust calculation, structurally separate from whatever the
in-chamber pressure/temperature trace does. Additionally,
`inlet_air_mass_flow_kg_per_s` itself is Mach-dependent through the
side-inlet ram-recovery chain: `side_inlet_ram_recovery_ratio(mass_flow_coefficient)`
(`pulsejet.py:51-71`, Hall & Frank NACA RM A8I29 anchors `0.50/0.90/0.95`)
feeds `inlet_total_pressure_pa` (`pulsejet.py:523-529`), which sets the
inertance ODE's driving pressure differential, which sets captured mass flow.
So both terms of `net_thrust_n` respond to Mach through mechanisms
independent of in-chamber combustion timing -- exactly the "Lenoir cycle,
thrust still rises via mass flow" picture Khrulev's side-intake finding
describes.

**What this does and does not resolve, now confirmed directly (not inferred
from wording):** `propulsion_map.py`'s `_run_pulsejet_simulation`
(`propulsion_map.py:224-258`) calls `run_pulsejet_to_converged_cycle_average`
-- the real, full unsteady `PulsejetSimulator` path. There is no
`force_unsteady` parameter, and no call to `evaluate_pulsejet_closed_form`,
anywhere in `propulsion_map.py`. **The closed form is not wired into Gate
2/3 at all today**, despite `evaluate_pulsejet_closed_form`'s own docstring
claiming to be "Gate 2's default pulsejet evaluation path (`propulsion_map.py`'s
`_pulsejet_point`, `force_unsteady=False`)" -- that docstring describes an
integration that was apparently planned but never completed (or was
completed and later reverted during this session's fidelity-tier refactor),
and is stale. This has two consequences:

1. This session's entire Gate 3 diagnostic chain (design-optimize v9-v11,
   the shared-nozzle throat-margin sweeps, the mass-vs-geometry isolation)
   ran against the real simulator, confirmed correct here -- none of that
   work needs to be revisited on account of this finding.
2. Both source documents' own wording ("flat Mach-dependence... in the
   closed-form derivation," "closed-form's *in-chamber cycle*") already
   says this observation was made against the closed form specifically, and
   that is now confirmed structurally dead code from Gate 2/3's perspective
   -- it cannot be producing wrong live results because it produces no live
   results. The flatness is real (the module's own `_solve_inlet_state`
   docstring already flags a static zero-flow ram-recovery approximation
   in place of a per-cycle mass-flow estimate), but it is scoped entirely
   to the paused closed-form re-derivation task (tracker tasks #6/#7), not
   to anything currently driving Gate 2 or Gate 3 output.

**Priority reasoning**: **No implementation gap found in the live simulator.**
The closed form's flatness is real but inert until/unless tasks #6/#7 (paused
this session pending a scope decision on external-vs-internal calibration)
resume.

## Gap 5 -- Validation anchors: NACA MR E5J02 precision

**Confirmed: current usage is a single-point, order-of-magnitude comparison,
not the point-by-point mapping the audit recommends.**
`docs/design_convergence.md`'s "Low-Mach thrust magnitude sanity check"
section scales E5J02's static 2224 N result by capture-area ratio alone
(`(0.195/0.559)^2 = 0.122` -> ~271 N) and compares it qualitatively against
this model's ~2.3-3.3 N low-Mach output (a ~90-115x gap, explained by duty
cycle, not by a point-by-point ram-pressure mapping). The audit's
recommendation -- map the model's predicted thrust ratios onto E5J02's own
0/18/40/58 in. water = 0/190/280/340 mph -> 500/660/740/770 lb curve shape
(normalized, diminishing-returns check) -- has not been done. This is a
real, currently-open validation gap, not a modeling gap.

**Priority reasoning**: cheap to do (a handful of `evaluate_propulsion_map_point`
calls at 0/190/280/340 mph-equivalent Mach plus a ratio comparison, no new
model changes) and directly answers whether the model's Mach-dependence
*shape* (not just direction) is right. **High priority given the low cost.**

## Summary table

| Gap | Present? | New inputs needed | Category | Priority |
|---|---|---|---|---|
| 1. Ignition-delay/heat-release phasing | Yes, confirmed; first-pass fix attempted and reverted (regressed convergence at Mach 0.90) | 2 fit/reasoned constants -- fixed delay confirmed insufficient, needs Mach/cycle-period scaling | design choice | High -- needs a cycle-period-adaptive delay, not a fixed fraction of the configured reference period |
| 2. Exhaust/tailpipe inertia | Yes, confirmed; first-pass fix attempted and reverted (broke convergence at 2 of 3 Mach points tested) | tailpipe length, area, friction/damping coeffs -- damping now confirmed necessary, not optional | design choice (no field exists yet) | High -- still the likely highest leverage, but needs a damped design, not a straight inlet-model mirror |
| 3. Valve model (quasi-stationary upgrade) | Current idealization defensible, upgrade path real | petal E, thickness, width, length | design choice (no field exists yet) | Medium |
| 4. Momentum-drag/mass-flow Mach-dependence | Already present in simulator, confirmed live-wired | none | -- | Closed. Closed form (source of the "flat" report) confirmed not wired into Gate 2/3 at all |
| 5. E5J02 point-by-point validation | Done -- direction matches, shape does not (real peak-and-decline vs. E5J02's smooth diminishing returns) | none (uses existing model + data already cited) | -- | Follow-up: finer sweep through Mach 0.30-0.45 |

## Gap 2 implementation attempt (2026-08-09): tried, found genuinely broken, reverted

A first-pass implementation was attempted and then fully reverted -- worth
recording why, so a future attempt doesn't repeat the same dead end.

**Design tried**: mirror the inlet's own accepted inertance ODE
(`d(mdot)/dt = (A/L) * (P_up - P_down)`, integrated as real state between
timesteps) onto the exhaust side, using `nozzle.throat_area_m2` as area (a
real configured quantity) and a new first-pass length estimate
(`_EXHAUST_DUCT_LENGTH_TO_THROAT_DIAMETER_ESTIMATE = 15.0`, reasoned by
analogy to published valved-pulsejet tailpipe L/D ratios), capped at the
throat's true sonic mass-flow ceiling (computed via the existing, tested
`compressible_orifice_mass_flow`, forced into its choked branch) rather than
at the regime-dependent quasi-steady value, specifically so the lag could
still let flow coast above the quasi-steady prediction during blowdown
decay -- the whole physical point of adding inertance.

**What broke, confirmed by direct measurement, not assumed:**

| Mach | Before (this session's other fixes, no exhaust inertance) | After (with exhaust inertance) |
|---|---|---|
| 0.20 | 81.511 N, converged, 11 cycles | ~198 N, converged, but only after re-instrumenting by hand -- a **+143% magnitude jump** |
| 0.50 | 176.253 N, converged, 24 cycles | 165.637 N, **did not converge**, hit a 100-cycle cap |
| 0.90 | (not separately re-measured before) | 181.672 N, **did not converge**, hit a 100-cycle cap |
| 0.00 | 0 N, `no_completed_cycles_within_simulation_cap` (a **pre-existing** cap limitation, confirmed present *before* this change too -- see the E5J02 mapping note below) | 239.67 N, `cycle_average_did_not_converge` (worse, but building on an already-broken baseline) |

Two separate problems, not one:

1. **The +143% magnitude swing at Mach 0.20 is far too large to trust from an
   unvalidated first-pass parameter.** A sensitivity check (varying the
   invented tailpipe-length estimate) was started but stopped once the
   convergence problem below made the whole design moot -- the honest
   conclusion is that this magnitude cannot be trusted until the design
   itself is fixed and re-validated, not that any specific length value is
   wrong.
2. **Root cause of the non-convergence, found by direct comparison against
   the inlet's own (working) inertance model**: the inlet's ODE state
   (`inlet_mass_flow_kg_per_s`) gets explicit, physically-grounded damping
   for free every cycle -- `step()`'s check-valve branch hard-zeroes it the
   instant the pressure differential reverses ("idealized as instantaneous,
   also removes the duct's stored momentum rather than letting it coast
   through a reversal"). This reset is *why* the inlet model converges
   cleanly in a handful of cycles. The exhaust side has no analogous event --
   there is no physical valve closure to hang a reset on (unlike the inlet,
   gas already in the tailpipe from blowdown has no reason to reset at the
   next ignition) -- so the exhaust ODE as designed is effectively
   undamped, and small cycle-to-cycle differences apparently never fully
   decay within any practical cycle cap. Khrulev's own full liquid-piston
   model includes exactly the friction/local-resistance terms (`xi_T`,
   `xi_c`) that would provide this damping physically -- deliberately not
   reproduced here (same reasoning as the inlet model: no sourced
   coefficients for this vehicle) -- but without *some* dissipation
   mechanism, the simplified inertance-only form does not behave like a
   real pulsejet exhaust; it behaves like an undamped oscillator being
   driven every cycle.

**Disposition**: fully reverted (`git diff` confirms zero exhaust-inertance
code remains in `pulsejet.py`; `tests/test_pulsejet.py` and
`tests/test_propulsion_map.py` pass clean against the reverted state). Gap 2
remains open. A real fix needs either a genuine, reasoned damping term (not
an invented one -- the same standard this session has held every other
constant to) or a different structural mechanism for keeping the exhaust
state well-behaved cycle to cycle, not a straight mirror of the inlet's
approach. Flagging clearly rather than shipping a change that measurably
regresses convergence at two of three previously-clean Mach points, per this
project's standing principle (surface real non-convergence, don't average
over it).

**Side finding, independent of this attempt**: the clean "before" comparison
at Mach 0.0 (`no_completed_cycles_within_simulation_cap`) confirms Mach 0.0
was *already* failing to converge within `PULSEJET_FIDELITY_FULL`'s cap
before any of this Gap-2 work -- this is why task #22's E5J02 mapping (below)
reported exactly 0.000 N at Mach 0.0 rather than the low-but-real thrust this
document's earlier "Low-Mach thrust magnitude sanity check" section found by
other means. Pre-existing, not caused by this attempt; worth its own look
before trusting any Mach-0-anchored comparison against E5J02's static point.

## Gap 5 result: E5J02 point-by-point mapping run, and the shape does not match

Ran `evaluate_propulsion_map_point` (`PULSEJET_FIDELITY_FULL`, `reference_case.yaml`,
sea level) at the Mach equivalents of E5J02's own 0/190/280/340 mph points
(sea-level speed of sound 340.29 m/s -> Mach 0.0000/0.2496/0.3678/0.4467):

| mph | Mach | E5J02 (lb) | E5J02 ratio to 190mph | Ours (N) | Ours ratio to 190mph |
|---:|---:|---:|---:|---:|---:|
| 0 | 0.0000 | 500 | 0.758 | **invalid -- see below** | -- |
| 190 | 0.2496 | 660 | 1.000 | 129.484 | 1.000 |
| 280 | 0.3678 | 740 | 1.121 | 183.557 | 1.417 |
| 340 | 0.4467 | 770 | 1.167 | 167.672 | 1.295 |

**The Mach 0.0 (static) point is invalid, not zero**: this run hit
`no_completed_cycles_within_simulation_cap` -- confirmed (above) to be a
pre-existing `PULSEJET_FIDELITY_FULL` cap limitation at the already-marginal
Mach-0 duty cycle, not real physics (this document's earlier "Low-Mach thrust
magnitude sanity check" section separately found genuine, if small, static
thrust by other means). Ratios are therefore computed relative to the 190 mph
point instead of static, since that is the lowest point with a trustworthy
result.

**The shape does not match, and this is a real finding, not a units or
scaling error.** E5J02 rises monotonically with decreasing marginal gain at
every step (0.758 -> 1.000 -> 1.121 -> 1.167 lb ratios) -- the "increases at
a rapidly decreasing rate" shape the report describes in words. This model's
net thrust **peaks at 280 mph (Mach 0.368) and then *falls* at 340 mph
(Mach 0.447)** -- 1.000 -> 1.417 -> 1.295, a real non-monotonicity E5J02
shows no equivalent of anywhere in its own data. Notably, the two points
where the discrepancy appears straddle `pulsejet_cycle_mode`'s own
Lenoir-to-Humphrey transitional-to-Humphrey boundary (transitional below
Mach 0.30, Humphrey at/above) -- the 190 mph point (Mach 0.2496) is still
transitional, the 280/340 mph points (Mach 0.368/0.447) are both Humphrey.
Whether this cycle-mode transition is the actual cause of the peak-then-fall
shape, or a coincidence, is not established here -- flagged as the next
concrete thing to check (e.g., a finer Mach sweep through 0.30-0.45 to see
whether the fall is a sharp feature right at the transition or a smooth
peak), not guessed at.

**What this does and does not mean for Gaps 1/2.** This confirms Gap 5's own
prediction that validating against E5J02 would be informative beyond a
simple direction check: the model gets the *direction* right (thrust rises
with speed over this whole band, matching E5J02 qualitatively) but not the
*shape* (a real peak-and-decline where E5J02 shows smooth diminishing
returns). This is at least suggestive that a structural gap -- most plausibly
Gap 1 (ignition-delay phasing) or Gap 2 (exhaust inertance, though the first
attempt at that was reverted above, not a working alternative to compare
against yet) -- is shaping this specific transition band, rather than a
already-known effect. Not proven; the cycle-mode-boundary coincidence above
is the strongest lead.

## Gap 1 implementation attempt (2026-08-09): tried, partial regression, reverted

Same pattern as the Gap 2 attempt above: implemented, measured, reverted --
recorded so a future attempt starts from what's already known rather than
repeating it.

**Design tried**: a fixed delay between the ignition trigger (fuel/air
consumed, heat credited to `pending_heat_release_j`) and the start of
`burn_time_remaining_s` metering, sized as
`0.3 * config.minimum_cycle_period_s` (0.3 chosen as a rough analog of
Khrulev's reported "m ~= 0.3" fraction -- stated plainly in the code comment
at the time, and again here, that only Khrulev's qualitative description was
available, not his actual equations, so this was an order-of-magnitude
analog, not a reproduction). No differential equation this time -- a simple
countdown timer, structurally much lower-risk than Gap 2's ODE.

**What happened, confirmed by direct measurement:**

| Mach | Before | After | Result |
|---|---|---|---|
| 0.20 | 81.511 N, converged, 11 cycles | 75.769 N, converged, 10 cycles | clean, -7% |
| 0.50 | 176.253 N, converged, 24 cycles | 109.593 N, converged, 13 cycles | clean, -38% |
| 0.90 | 201.801 N, converged, 24 cycles | 157.299 N, **did not converge**, 100 cycles | **regression** |

Better-behaved than the Gap 2 attempt (2 of 3 points converged cleanly, in
*fewer* cycles than before, not more), but still a real, confirmed
regression at Mach 0.90. **Likely mechanism**: the delay is a *fixed*
0.3 * `minimum_cycle_period_s` (~4.2 ms for this config), independent of
Mach -- but the simulation's own *emergent* cycle period shortens as Mach
rises (faster refill, more frequent ignition). A fixed delay is therefore a
progressively larger fraction of the actual cycle at higher Mach, plausibly
destabilizing the same ignition-timing feedback loop
(`_ignite_if_ready`'s four-condition gate) that keeps cycling well-behaved
at lower Mach where the fixed delay is a small fraction of a longer cycle.
Not confirmed by direct instrumentation (stopped here rather than chase a
second multi-step debugging effort in the same session) -- a plausible,
not proven, mechanism.

**Disposition**: fully reverted (confirmed zero `ignition_delay`/
`IGNITION_DELAY` references remain in `pulsejet.py`; `tests/test_pulsejet.py`
and `tests/test_propulsion_map.py` pass clean). Gap 1 remains open. Both
attempts this session (Gap 1 and Gap 2) failed for a related reason: a fixed,
Mach-independent parameter (delay fraction here, tailpipe length there)
interacting badly with this simulator's Mach-dependent emergent cycle
dynamics. A real fix likely needs the delay (or damping, for Gap 2) to scale
with the *actual* cycle period rather than the configured reference period --
which reintroduces the chicken-and-egg problem noted in the original design
comment (the real cycle period is an output of the dynamics, not known in
advance) and was not solved here.

## Recommended task order

Not started -- tracked in the session task list (`design-optimize`/Gate 3
task tracker), not implemented as part of this audit:

1. Run the E5J02 point-by-point ram-pressure mapping (Gap 5) first -- cheapest,
   and its result (does the model already get the *shape* right?) should
   inform how urgently Gaps 1/2 are actually needed before investing in them.
2. Confirm whether the original "flat Mach-dependence" observation was against
   the full `PulsejetSimulator` or the closed form (Gap 4 follow-up) --
   determines whether Gaps 1/2 belong in the simulator, the closed form, or
   both.
3. Add exhaust-side inertance (Gap 2), mirroring the existing inlet-side
   `_inertance_inlet_duct_area_m2`/`_inertance_inlet_duct_length_m` pattern.
4. Add ignition-delay phasing (Gap 1), in relative/normalized form per
   Khrulev's own explicit stability warning.
5. Valve model upgrade (Gap 3) -- lower priority, defensible as-is; revisit
   if Gap 2's fix doesn't fully regularize the low-Mach refill equilibrium.

Each of 3-5 is a real simulator change (not closed-form-only) and should go
through this project's existing audit-then-implement separation, with real
before/after numbers recomputed and documented per this session's established
practice -- not estimated.
