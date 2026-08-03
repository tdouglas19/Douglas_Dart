# Assumption register

`shared_nozzle_candidate_a.yaml` is the active trade point. A value in that file is
not automatically a user requirement or a validated parameter.

## Requirements and owner direction

| Item | Current value | Status |
|---|---:|---|
| Intake modes | Pulsejet or ramjet, never both open | User requirement |
| Available geometric intake area | 50% of the 195 mm circular intake for either mode | User requirement / prior baseline diameter |
| Peak Mach | 1.10 | User requirement |
| Speed-run termination | Fuel depletion; no prescribed duration | User requirement |
| Body diameter | No hard maximum; trade against drag and Mach capability | User direction |
| Maximum takeoff mass | 25 kg | Competition requirement |
| Minimum supersonic time | More than Mach 1 for at least 5 s | Competition requirement |
| Recovery | Land intact; reciprocal same-day flight required | Competition requirement |
| Pulsejet eligibility | Organizer confirmation required | Unresolved rule interpretation |

## Candidate A geometry and mission inputs

| Item | Value | Status and closure path |
|---|---:|---|
| Loaded reference mass | 21.0 kg | Prior mass baseline; reclose with real hardware and recovery system |
| Loaded / ramjet-phase fuel | 3.80 / 1.40 kg | Prior allocation; replace with integrated trajectory fuel ledger |
| Fuel | Jet-A/JP-8-class representative | Recommended baseline family; see `fuel_trade.md` and close exact grade, atomization, light-off, safety, and property ranges |
| Body diameter / length | 0.205 / 2.30 m | Candidate A; diameter is at the packaging boundary |
| Shared throat diameter | 0.130 m | Candidate A; only 3.17 mm above the local modeled thrust boundary |
| Exit/throat area ratio | 1.05 | Provisional lower-bound C-D architecture; not optimized or validated |
| Selector radial allowance | 0.005 m | Packaging budget, not a detailed mechanism thickness |
| Nozzle radial allowance | 0.012 m | Packaging budget for wall, insulation, and structure |
| Lifting surfaces | Two; 0.16 m exposed semispan, 0.42/0.14 m chords | OpenVSP starting geometry |
| Fins | Four at 45°, 135°, 225°, and 315°; 0.10 m exposed span | OpenVSP starting geometry |
| Aero coefficient reference | 0.0896 m² exposed lifting area, 0.525 m span, 0.3033 m MAC | Explicit convention shared by flight and VSPAERO |
| Field / sled release | 900 m MSL / 39–42 m/s TAS | Prior mission baseline |
| Top of climb | 6,000–6,500 m MSL | Prior mission baseline; optimize with trajectory and loads |
| Speed-run analysis point | 4,500 m MSL, Mach 1.10 | Current trade point, not yet altitude-optimized |
| Drag-area ceiling | 0.0095 m² at 0.200 m body | Prior conservative target, not an aerodynamic prediction |
| Ramjet propulsion reserve | 15% reduction in modeled net thrust | Explicit trade margin, not an uncertainty distribution |

The candidate's diameter-scaled drag-area budget is 0.009981 m² at 205 mm. Until
the external-aerodynamics work is complete, changing diameter scales this budget with
diameter squared while intake diameter remains fixed.

## Propulsion-model assumptions

| Item | Current value | Status and closure path |
|---|---:|---|
| Chamber volume | 0.025 m³ | Placeholder; packaging and acoustic-length closure required |
| Fuel LHV / stoichiometric AFR | 43 MJ/kg / 14.7 | Representative Jet-A values; replace with cited ranges |
| Pulsejet gamma / gas constant | 1.33 / 287.05 J/(kg·K) | Constant-property approximation |
| Pulsejet combustion efficiency | 0.58 | High-sensitivity placeholder; calibration required |
| Target equivalence ratio | 0.90 | High-sensitivity placeholder; operability bounds required |
| Burn duration / minimum period | 4 / 18 ms | Placeholder timing model; calibrate to pressure histories |
| Wall heat conductance | 20 W/K | Placeholder; replace with a thermal network |
| Maximum gas temperature | 2,600 K | Numerical limiter for omitted chemistry/variable properties |
| Ramjet total-pressure recovery | 0.92 | High-sensitivity placeholder; couple to inlet/shock geometry |
| Ramjet combustor pressure loss | 6% | Placeholder |
| Ramjet target combustor exit | 1,900 K | Placeholder; close with fuel schedule and material limits |
| Light-off experiment / self-sustaining gate | Mach 0.80 / 1.10 | Separate mission concepts, not demonstrated operability |

Pulsejet trade statistics discard the first 0.25 s and average the next 0.25 s.
This prevents the initially charged chamber from dominating a short run. The
configured 20 µs time step changes the steady-window mean thrust by about 0.27%
relative to a 10 µs run for Candidate A.

## Interpretation of current outputs

The static Mach 1.10 calculation predicts 603 N ramjet net thrust, 513 N after the
15% derate, and 488 N against the diameter-scaled drag budget. It also predicts 66%
potential-capture spillage. That spillage is a capacity bound, not a solved
inlet/back-pressure flowfield.

Fuel at a linearly scaled hold condition lasts about 29 s, which is longer than the
five-second rule. This is not a trajectory or compliance result: it omits the fuel
and time used accelerating through Mach 1 and assumes thrust and fuel flow scale
linearly at the hold point.

The steady-window pulsejet model predicts about 185 N net thrust at sea level and
Mach 0.20. Its mean thrust-to-weight ratio is about 0.90 for the full 21 kg vehicle;
that ratio alone neither proves nor disproves climb because the flight-path force
balance also includes aerodynamic drag, lift orientation, and gravity.

Every result remains `numerical_reference_only` until the gates in
[validation.md](validation.md) are closed.
