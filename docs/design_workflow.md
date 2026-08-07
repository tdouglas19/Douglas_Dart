# Design workflow: nested loops and gates

This document is the durable reference for how Douglas Dart's design converges.
It exists so that "the code ran" and "a design gate passed" are never conflated
-- see the closing rule in every gate below. Sourced from a 2026-08 design
review (`Core objective.md`); this file is the canonical version of that
guidance, kept in the repo instead of a local download.

## Why nested loops, not one optimizer

The project must answer four distinct questions, in order, and no single
solver answers all four at once:

1. Is the concept energetically and physically plausible?
2. Which propulsion, geometry, mass, and trajectory combinations appear
   feasible?
3. Does a realistic external vehicle shape retain sufficient aerodynamic and
   propulsion performance?
4. Can the resulting aircraft trim, remain stable, and follow the required
   trajectory with practical control surfaces?

Each question needs a different fidelity of model and a different amount of
compute per candidate. Coupling them into one search lets the optimizer
exploit whichever question's model is weakest -- see "Optimizer rules" below.

## The three loops

### Fast inner loop (seconds or less per candidate)

Propulsion-map lookup + low-order aero + parametric mass + packaging +
point-mass trajectory + robustness scenarios + requirement scoring. This is
the only loop allowed to run automated optimization over hundreds or
thousands of candidates. Today's `optimizer.py`/`trajectory.py` pair is this
loop, but is missing the feasibility couplings listed in Level 2 below.

### Geometry and aerodynamic loop (minutes to hours per candidate)

OpenVSP geometry + mesh convergence + VSPAERO sweeps + parasite/wave/base/
inlet-drag estimates + stability estimates + updated mass and packaging. Only
the best few fast-loop candidates enter this loop -- never used as an inner
optimizer.

### Flight-controls loop (mature candidates only)

JSBSim trim + dynamic modes + control effectiveness + actuators + guidance +
off-nominal simulation + recovery behavior.

**Feedback must flow both directions.** VSPAERO drag/stability data updates
the point-mass model; JSBSim trim drag or control-surface changes update the
geometry and mass models. A one-way pipeline (fast loop -> geometry -> flight
controls, never back) is not this workflow.

## Design levels

| Level | Question answered | Tooling | Repo status as of 2026-08 |
|---|---|---|---|
| 0 | Is it energetically/physically plausible at all? | Hand calculations: stall-speed bounds, required lift area, T/W and excess power, climb rate, transonic acceleration time, dive energy benefit, fuel/endurance bounds, dynamic-pressure limits, drag-area requirement, packaging bounds | **Missing** -- no dedicated module or document yet |
| 1 | What does one authoritative propulsion map say? | `ramjet.py`, `pulsejet.py` physics, behind one callable map/table interface | Physics exists and was extended this session (governing-equations pass); the *single interface* does not -- 9 separate call sites build their own tables (see gap assessment below) |
| 2 | Which combinations look feasible? | Fast point-mass/energy-state trajectory solver, coupled to propulsion maps, drag-area model, low-order lift, fuel depletion, mass, packaging, mission requirements, plus lift/stall/dynamic-pressure/load-factor/trim feasibility checks | `trajectory.py` + `optimizer.py` exist, are coupled to `propulsion_map.py` and `mass_model.py`, and close missions. **Stall margin** is now checked every step against a 1g level-flight bound (`minimum_stall_margin_fraction`, `stall_margin_violated`) and **peak dynamic pressure** is reported (no configured ceiling exists to compare against yet). Still not implemented: load factor as a general per-step check (only the existing dive pull-out radius calc touches it), trim drag, CG bounds, control-authority reserve |
| 3 | Do several materially different physical candidates survive? | Complete, internally consistent candidate definitions (body diameter/length, intake, chamber, nozzle, fuel, surfaces, fins, CG, mass breakdown, packaging, recovery, schedule) | **Partial.** `mass_model.py` now ties empty mass to body diameter/length, throat diameter, and configured lifting/fin/selector geometry (CLI: `mass-breakdown`), and `body_length_m` and `wing_area_scale_factor` (geometric-similarity lifting-surface scaling) are real `optimizer.py` search variables (`design-optimize`) instead of fixed config values. Still missing: fin geometry and lifting-surface x-location/sweep/thickness are not independently searched, CG/packaging/recovery are not modeled as consequences of the search, and only one candidate (`shared_nozzle_candidate_b.yaml`) is ever retained -- no multi-candidate retention yet |
| 4 | Does external geometry retain performance? | OpenVSP geometry generation + VSPAERO sweeps, documented total-drag accounting (external aero + viscous/parasite + wave + base + inlet/spillage + trim + installation allowances, one drag-area convention) | Geometry generation and VSPAero sweeps work (this session ran both against updated ram-inlet geometry); total-drag accounting that reconciles VSPAero inviscid drag with the trajectory model's drag-area budget **does not exist yet** |
| 5 | Can it trim, stabilize, and be controlled? | JSBSim trim across mission phases, dynamic-mode check (short-period, phugoid, Dutch roll, roll, spiral), control-surface sizing, actuator practicality, guidance-law feasibility, dive-pullout load/q survival | JSBSim model exists but uses provisional stability derivatives and has no true control surfaces -- correctly treated as an implementation/force-balance cross-check, not a stability-and-control gate, until Level 4 aero and explicit controls replace those approximations |

## Gates

Passing a gate means the specific evidence below exists and has been
reviewed -- **never** "the code executed" or "unit tests are green" alone.

- **Gate 0 -- Requirement definition.** Mission requirements, competition
  rules, user decisions, assumptions, and open questions are separated and
  versioned. *(Partially met: `RequirementsConfig` exists and is sourced --
  see `docs/assumptions.md` -- but requirements, assumptions, and geometry
  still live in one YAML file per case, not separated per the target
  structure below.)*
- **Gate 1 -- Analytical feasibility.** Hand calculations show a plausible
  energy, thrust, lift, fuel, drag, mass, and packaging region. *(Tooling now
  exists -- `python -m douglas_dart level0-bounds` -- but the gate itself is
  **not** passed: it found the current candidate's reference area is ~3.6x too
  small to fly at its own configured sled-release speed at max mass, and the
  pulsejet chamber fails packaging even at the loosest possible bound. See
  `docs/level0_feasibility_bounds.md`. This is the gate doing its job, not a
  tooling failure.)*
- **Gate 2 -- Propulsion-map readiness.** Both propulsion modes produce
  stable, traceable Mach/altitude maps with validity flags, uncertainty
  ranges, and **consistent consumers** (one interface, not nine). *(Met.
  `propulsion_map.py` is the one authoritative interface
  (`evaluate_propulsion_map_point`/`build_propulsion_map`, CLI:
  `propulsion-map`), covering thrust, fuel flow, TSFC, Isp, captured/potential/
  spilled mass flow, installed recovery, nozzle regime, combustor temperature,
  light-off/self-sustaining status, and validity flags for both modes. Every
  consumer that feeds a design calculation now goes through it:
  `trajectory.py` (all 3 former call sites), `robustness.py`, `sensitivity.py`,
  `sizing.py` (5 of 7 call sites), `jsbsim_model.py`, and `pipeline.py`'s
  ramjet-peak-Mach stage. The remaining direct `PulsejetSimulator`/
  `evaluate_ramjet` calls (`sizing.py`'s `evaluate_ramjet_handoff_sizing` and
  its one nozzle-matched pulsejet report field, `pipeline.py`'s pulsejet
  diagnostic stage, `cli.py`'s standalone `pulsejet`/`ramjet` commands) are
  each documented in-place as intentional exceptions -- they need raw
  physics-layer internals (fuel_air_ratio, nozzle_capacity_kg_per_s, raw
  per-step samples, conservation audits) that `PropulsionMapPoint`
  deliberately excludes from its common cross-mode schema, and none of them
  feed another design calculation that could silently drift from the
  authoritative map.)*
- **Gate 3 -- Nominal fast-mission feasibility.** A candidate completes the
  mission in the point-mass solver without violating lift, stall, fuel,
  dynamic-pressure, mass, packaging, or rule constraints. *(Still not met --
  the stall-margin check now exists and immediately fails on candidate B:
  `minimum_stall_margin_fraction` is -37.7% nominal (i.e. the vehicle spends
  time below its own 1g stall speed), confirming at the full mission-
  integration level the same Gate 1 finding from
  `docs/level0_feasibility_bounds.md`, not a new, separate problem. Load-
  factor and trim-drag checks are still missing.)*
- **Gate 4 -- Robust fast-mission feasibility.** At least one candidate
  retains acceptable margins under named conservative cases; adverse cases
  may stay informational but their meaning must be explicit. *(Not met --
  this session's own mission-level differential-evolution search found no
  candidate that closes both nominal and adverse; documented in
  `docs/design_convergence.md` as a structural gap, not silently ignored.)*
- **Gate 5 -- Aerodynamic and geometry closure.** OpenVSP geometry is
  physically inspectable and solver-backed aerodynamic data have replaced
  provisional drag/stability proxies over the important flight envelope.
  *(Not met -- geometry and VSPAero sweeps run, but nothing feeds their
  output back into the trajectory model yet.)*
- **Gate 6 -- Updated mission closure.** The mission still closes after
  using solver-backed aerodynamic tables and updated mass properties. *(Not
  reachable until Gate 5.)*
- **Gate 7 -- Stability and control closure.** JSBSim demonstrates trim,
  acceptable dynamics, sufficient control authority, and feasible guidance
  across the mission. *(Not met -- no true control surfaces yet.)*
- **Gate 8 -- Subsystem validation readiness.** Remaining dominant
  uncertainties are detailed inlet operability, pulsejet physics, structures,
  thermal behavior, hardware mass, transition mechanisms, and recovery --
  not unresolved basic vehicle sizing. *(Not reachable yet.)*

## Optimizer rules

Do not expand `optimizer.py`'s variable set until the objective function and
model boundaries are credible. Before adding a variable: run sensitivity
analysis, confirm it materially affects mission success, confirm changing it
also changes all associated mass/geometry/drag/packaging/propulsion
consequences, add explicit bounds, and document why it's included.

The optimizer must not be allowed to exploit missing physics. Body-diameter
and body-length growth now carry a structural-mass penalty via
`mass_model.py` (calibrated to today's mass budget, not independently
validated -- see that module's docstring). Lifting-surface area is now a
search variable too (`wing_area_scale_factor`, geometric-similarity scaling
of root chord, tip chord, and exposed semispan; see `optimizer.py`'s module
docstring), so the optimizer can trade wing area against mass/drag the way
it already could trade body size against mass/drag -- the mechanism the
Gate 1 stall-speed gap in `docs/level0_feasibility_bounds.md` needed to
close. Fin geometry and lifting-surface x-location/sweep/thickness are still
read from the baseline config, not independently searched. Treat every
`optimizer.py` result as a search hint until rerun at full fidelity and
cross-checked independently -- not as a selected design.

## Working rules for every major change

- State the engineering purpose and which gate it supports.
- State assumptions explicitly.
- Add tests; run the relevant regression suite.
- Record whether a numerical change is a bug fix, a model-fidelity change, or
  a design change.
- Update `docs/design_convergence.md`.
- Never present internal consistency (tests pass, numbers reconcile) as
  physical validation.
- When a model correction changes the selected design, preserve the old
  candidate as a regression case and document why it was rejected -- do not
  silently overwrite it.
- Do not freeze the current candidate. It is the current integration point,
  not a finished design.
