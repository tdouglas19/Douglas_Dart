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
