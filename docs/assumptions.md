# Assumption register

Every consequential input is labeled so a convenient demonstration value cannot
quietly become a design requirement.

| Item | Current value | Status | Closure path |
|---|---:|---|---|
| Intake paths | Mutually exclusive | User requirement | Confirm selector leakage and transition behavior later |
| Available area | 50% of circular intake | User requirement | Retain; characterize additional blockage separately |
| Reference intake diameter | 0.300 m | Numerical placeholder | Close with thrust, drag-area, packaging, and mass-flow trade |
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
| Minimum ramjet Mach | 1.5 | Conservative operability gate | Replace with inlet/combustor-specific validated boundary |
| Vehicle mass / area | 45 kg / 0.18 m² | Numerical placeholder | Close after engine, fuel, structure, recovery, and aero sizing |

## Interpretation rule

A placeholder may be swept to learn sensitivity. It may not be cited as expected
vehicle performance. Results should retain the case name
`numerical_reference_not_a_design` until the minimum validation gates are closed.
