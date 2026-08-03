# Assumption register

Every consequential input is labeled so a convenient demonstration value cannot
quietly become a design requirement.

| Item | Current value | Status | Closure path |
|---|---:|---|---|
| Intake paths | Mutually exclusive | User requirement | Confirm selector leakage and transition behavior later |
| Available area | 50% of circular intake | User requirement | Retain; characterize additional blockage separately |
| Circular selector intake diameter | 0.195 m | Prior project baseline, not closed | Keep separate from the outer body during packaging trades |
| Nominal outer body diameter | 0.195 m; no hard maximum | User trade instruction / prior baseline | Increase only when packaging benefit outweighs drag penalty |
| Reference body length | 2.20 m | Midpoint of prior 2.0–2.4 m range | Close with fineness, packaging, stability, and structural trades |
| Peak drag-area ceiling | 0.0095 m² at 0.200 m reference diameter | Prior project target, not an aero prediction | Replace diameter-squared similarity proxy with VSPAERO tables |
| Reference chamber volume | 0.025 m³ | Numerical placeholder | Sweep against frequency, residence time, and packaging |
| Throat diameter | 0.100 m | Numerical placeholder | Match transient pulse flow and ramjet steady-flow residual |
| Exit/throat area ratio | 2.25 | Numerical placeholder | Optimize across altitude and pressure histories |
| Fuel | Jet-A representative | Candidate, not selected | Compare operability, atomization, safety, storage, and energy density |
| Fuel LHV / stoichiometric AFR | 43 MJ/kg / 14.7 | Provisional | Replace with cited property ranges and sensitivity bounds |
| Pulsejet gamma / R | 1.33 / 287.05 J/(kg·K) | Low-order approximation | Add temperature/composition dependence |
| Combustion efficiency | 0.58 | Numerical placeholder | Calibrate against relevant hardware or literature data |
| Burn duration | 4 ms | Numerical placeholder | Calibrate to pressure trace and characteristic length |
| Wall heat conductance | 20 W/K | Numerical placeholder | Replace with geometry/material thermal network |
| Maximum gas temperature | 2600 K | Numerical limiter | Replace with equilibrium chemistry / variable properties |
| Ramjet target combustor exit | 1900 K | Numerical placeholder | Close with materials, equivalence ratio, and stability limits |
| Earliest ramjet light-off test | Mach 0.80 | User mission concept | Treat as an experiment, not an assumed sustainable handoff |
| Peak Mach | 1.10 | User requirement | Treat as a cap and prove thrust/drag closure at 4,500 m MSL |
| Speed-run termination | Consume allocated ramjet fuel; no prescribed duration | User requirement | Duration is a model output, never a mission input |
| Loaded / ramjet-phase fuel | 3.80 kg / 1.40 kg | Prior project mass allocation | Reclose after trajectory integration and reserve definition |
| Field / sled release | 900 m MSL / 39–42 m/s TAS | Prior project baseline | Confirm actual launch site and sled performance |
| Top of climb / speed-run altitude | 6,000–6,500 m / 4,500 m MSL | Prior project baseline | Optimize dive and dynamic-pressure limits in coupled mission model |
| Loaded mass / aero reference area | 21 kg / 0.18 m² | Prior mass baseline / area placeholder | Close after engine, fuel, structure, recovery, and aero sizing |

## Handoff throat-sizing interpretation

The `ramjet-sweep` command calculates the throat diameter that would pass all
potential flow captured by the configured half-intake. Intake diameter and outer
body diameter are now separate variables. Increasing both together does not cure the
packaging conflict because both captured flow and required hot-gas throat area scale
with diameter squared. The initial outer-body trade therefore keeps the 0.195 m
intake fixed and grows only the outer mold line.

The `diameter-trade` command compares the flow-matched throat against each outer-body
diameter and scales the prior drag-area ceiling with diameter squared under a
geometric-similarity assumption. Packaging uses zero radial clearance, so a passing
point is only a mathematical lower bound. If full-throttle ramjet thrust cannot
counter the scaled drag target at Mach 1.10, the command reports the required
drag-area reduction and does not invent a speed-run duration. If it can, hold time is
derived from the 1.40 kg ramjet fuel allocation using an explicitly labeled linear
thrust/fuel turndown approximation.

## Interpretation rule

A placeholder may be swept to learn sensitivity. It may not be cited as expected
vehicle performance. Results should retain the case name
`numerical_reference_not_a_design` until the minimum validation gates are closed.
