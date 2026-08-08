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

## Coupled Candidate B numerical reference

The phase-based mission model uses a separate 170 mm-throat, 24.6 kg Candidate B
reference with 7.4 kg loaded fuel. At the 0.05 s mission step it reaches Mach
1.10068 and spends 9.20 s above Mach 1, with +47.2 N minimum ramjet acceleration
margin and +70.8 N minimum Mach-target run margin. These are numerical outputs of the
stated low-order equations, not validated vehicle performance.

The result is conditional on a 100 m/s simulated release rather than the recovered
39–42 m/s target, forced ramjet operation from Mach 0.80, and a 50% multiplier on
uncaptured-flow momentum as a spillage-drag proxy. No live VSPAERO polar is used.
Longitudinal ground contact does not demonstrate intact landing or reciprocal return.

The 160 mm point above remains the selected static robustness-grid alternative; its
score and mass budget must not be transferred to the 170 mm coupled-mission case.

## Do not freeze yet

Keep body diameter, throat size, inlet recovery, drag area, fuel allocation, launch
speed, and transition schedule open. The next high-value gate is resolving the
surface-area/launch-speed/drag conflict, followed by live external-aerodynamic and
inlet/back-pressure closure. Candidate B must not be promoted as mission-feasible
until those gates, stability/control, landing, and reciprocal recovery are
demonstrated.
