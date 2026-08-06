# Candidate B static convergence

## Decision

Carry **Candidate B** as the next geometry and model-integration point:

- 210 mm outer body and 2.30 m length;
- fixed 195 mm circular intake with one-half available to the selected mode;
- 160 mm shared throat and `Ae/At = 1.05`;
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
| Body / intake / throat diameter | 210 / 195 / 160 mm |
| Exit/throat area ratio | 1.05 |
| Diameter-scaled drag area | 0.010478 m² |
| Drag at Mach 1.10 and 4,500 m | 512.1 N |
| Nominal ramjet net thrust | 832.1 N |
| Existing 15%-derated thrust | 707.3 N |
| Existing 15%-derated margin | 195.2 N |
| Nominal potential-capture spillage | 50.5% |
| Nominal fuel flow | 0.07386 kg/s |
| Full-throttle endurance from 1.40 kg | 19.0 s |
| Selector radial packaging margin | 2.5 mm |
| Pulsejet mean net thrust, sea level Mach 0.20 | about 122 N |

The larger throat improves high-speed ramjet flow capacity but reduces the
low-speed pulsejet result. That shared-nozzle compromise is now explicit and must be
tested in the coupled trajectory rather than optimized at one operating point.

## Named robustness screens

Every scenario is versioned in `configs/robustness_candidate_b.yaml`; none is a user
requirement or a validated uncertainty distribution.

| Scenario | Recovery | Thrust factor | Drag factor | Mass | Excess thrust |
|---|---:|---:|---:|---:|---:|
| Nominal | 0.92 | 1.00 | 1.00 | 21.0 kg | +320.0 N |
| Conservative | 0.87 | 0.85 | 1.10 | 22.5 kg | +51.5 N |
| Adverse | 0.82 | 0.75 | 1.20 | 23.9 kg | -153.7 N |

The conservative screen has a provisional 50 N excess-thrust budget, so Candidate B
passes it by only 1.5 N. The adverse case is intentionally informational and fails.
This is evidence that inlet recovery and installed thrust remain the dominant risks,
not evidence that the mission closes robustly.

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
