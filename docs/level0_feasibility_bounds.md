# Level 0 feasibility bounds (Gate 1)

`src/douglas_dart/feasibility.py`, run via `python -m douglas_dart level0-bounds
--config <case.yaml>`. Per `docs/design_workflow.md`: these hand-calc bounds do
not select a design. They reject impossible concepts and set the optimizer's
search bounds. Every function is a closed-form or single-point calculation,
not a trajectory simulation -- see the module docstring for exactly which
governing relation each one uses.

## What each check computes

| Check | Relation | What a failure means |
|---|---:|---|
| Sled launch feasibility | required a = v&sup2; / (2L), constant-acceleration rail launch, no drag/friction loss | The sled cannot reach the target release speed within the configured rail length under the (unsourced placeholder) acceleration ceiling |
| Stall speed bound | V_stall = sqrt(2W / (&rho; S CL_max)) at max takeoff mass, field elevation | The vehicle cannot fly at the configured sled-release speed at this mass/area/CL_max |
| Required lift area | S_required = 2W / (&rho; V&sup2; CL_max) at minimum release speed | The configured reference area is too small for the release speed |
| Thrust-to-weight sweep | Net thrust (pulsejet below self-sustaining Mach, ramjet at/above it) over weight, at 0.2/0.5/0.8/1.0/1.1 Mach | Any point &le; 1 means the vehicle cannot accelerate at that condition under this bound's simplifying assumptions |
| Climb rate estimate | (T&minus;D)V / W at zero lift/bank -- an energy-height rate, not a trimmed climb | A negative value flags a Mach region where drag exceeds thrust at zero lift |
| Dive energy benefit | V_exit = sqrt(V_entry&sup2; + 2gh), no drag loss -- an upper bound | Sizes how much dive height is even worth considering before running the real trajectory model |
| Fuel/endurance bounds | Loaded/ramjet-budget chemical energy; fuel mass / fuel flow at one representative point per mode | An upper bound on how long each mode could run at that one throttle setting |
| Packaging bounds | Five loosest-possible-bound inequality checks (intake/throat/selector vs. body diameter, chamber volume vs. forebody length at full body cross-section, fuel volume vs. fin-can annulus volume) | A failure means the component cannot fit under even the most generous interpretation of available space -- passing does **not** mean a real internal layout exists (see `scripts/vehicle_cross_section.py`'s own disclaimer) |

## The sled is not the bottleneck

At the currently configured 42 m/s release speed and 75 m rail, the sled
needs only **~1.2 g** and **~9 m of the 75 m rail** -- enormous margin. The
Gate 1 upper search bound (~121 m/s, from the 10 g placeholder ceiling) is
therefore easily achievable from the sled's side; raising release speed is
cheap in this dimension. This matters because it isolates where the real
Gate 1 problem actually is: **not** sled hardware, but lifting-surface area
(below). A faster release speed helps the stall-speed margin, but the
optimizer will need real wing-area sizing to close the ~3.6x gap on its own
-- see the "what has to give" discussion below.

## Current result on `shared_nozzle_candidate_b.yaml`: does **not** pass

Run 2026-08-07. `level0-bounds --strict` currently exits nonzero. This is the
gate working as intended -- surfacing a real problem rather than a
process-completion checkbox. Three failures:

1. **Stall speed bound fails badly.** At the configured 25 kg maximum takeoff
   mass, 0.0896 m&sup2; reference area, and CL_max = 0.90, the stall speed
   bound is **~73.6 m/s** -- nearly double the configured 39-42 m/s sled
   release speed. The vehicle as currently sized cannot generate enough lift
   to fly at release speed at max mass under this bound's assumptions.
2. **Required lift area confirms it**: ~0.319 m&sup2; would be needed at the
   minimum release speed, versus the 0.0896 m&sup2; configured -- roughly
   3.6x too small.
3. **Pulsejet chamber packaging fails even at the loosest possible bound.**
   The configured 0.025 m&sup3; chamber volume would need &ge;722 mm of
   length even if it occupied the *entire* body cross-section with nothing
   else inside it, against a 180 mm forebody transition length.

This does not mean the architecture is dead. It means one of several things
has to give, and this bound doesn't decide which: a larger reference area
(lifting surfaces sized as a real design variable, not the current OpenVSP
starting geometry), a lower operating mass at the speed that matters, a
different CL_max assumption once real airfoil data replaces the current
literature-typical placeholder (`docs/assumptions.md`), or accepting that
"maximum takeoff mass" and "sled release condition" are not the same
condition and modeling them separately instead of using one worst-case
number. **This finding was not previously visible** -- `docs/design_convergence.md`
tracks static propulsion/drag closure at one flight condition, and
`trajectory.py`'s mission solver does not yet check lift-available-vs.-required
at all (see the Level 2 gap in `docs/design_workflow.md`), so this is the
first hand-calc to actually check whether the vehicle can fly at its own
release speed.

## What passes today

- Sled launch feasibility, by a wide margin (~1.2 g / ~9 m of 75 m needed) --
  see above.
- Thrust-to-weight is comfortably above 1 at every swept Mach point (1.06 to
  4.55), and climb-rate estimates are positive except at exactly Mach 1.0
  (a small negative excess-power point, consistent with the already-documented
  low-speed pulsejet thrust-trough / mode-transition irregularity in
  `docs/design_convergence.md` -- not a new finding, just visible here too).
- Intake, nozzle throat, and selector all fit within the configured body
  diameter with their radial allowances.
- Fuel volume fits within the fin-can annulus even at the loosest bound.

## Explicitly not claimed

- Passing a packaging bound is not evidence a real internal layout exists.
- The thrust-to-weight sweep uses zero-lift drag only -- it is an excess-power
  bound, not a trimmed-flight capability claim.
- The dive-energy bound ignores drag entirely; the real number from
  `trajectory.py`'s integrator will be smaller.
- None of this replaces Gate 3 (`mission-trajectory`/`design-optimize`), which
  already runs and is tracked separately in `docs/design_convergence.md`.
