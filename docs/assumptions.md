# Assumption register

Every consequential input is labeled so a convenient demonstration value cannot
quietly become a design requirement.

| Item | Current value | Status | Closure path |
|---|---:|---|---|
| Intake paths | Mutually exclusive | User requirement | Confirm selector leakage and transition behavior later |
| Available area | 50% of circular intake | User requirement | Retain; characterize additional blockage separately |
| Nominal body / intake diameter | 0.195 m | Prior project baseline, not closed | Close with thrust, drag-area, packaging, and mass-flow trade |
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
| Preliminary sustained handoff search | Mach 1.1–1.3 | Prior project baseline, not closed | Replace with inlet/combustor-specific validated boundary |
| Loaded mass / aero reference area | 21 kg / 0.18 m² | Prior mass baseline / area placeholder | Close after engine, fuel, structure, recovery, and aero sizing |

## Handoff throat-sizing interpretation

The `ramjet-sweep` command calculates the throat diameter that would pass all
potential flow captured by the configured half-intake. It also reports that diameter
as a fraction of the 0.195 m nominal body. A value above 1.0 fails even the absolute
body-diameter gate before wall thickness, structure, actuators, cooling, and other
packaging are considered. The matched thrust is an idealized upper-bound output of
the low-order cycle, not a performance prediction.

## Interpretation rule

A placeholder may be swept to learn sensitivity. It may not be cited as expected
vehicle performance. Results should retain the case name
`numerical_reference_not_a_design` until the minimum validation gates are closed.
