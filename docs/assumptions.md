# Assumption register

`shared_nozzle_candidate_b.yaml` is the active static trade point. Values in it are
not automatically requirements or validated parameters.

## Requirements and owner direction

| Item | Current value | Status |
|---|---:|---|
| Intake modes | Pulsejet or ramjet, never both open | User requirement |
| Available geometric intake area | 50% of the 195 mm circular intake | User requirement / prior intake diameter |
| Peak Mach | 1.10 | User requirement |
| Speed-run termination | Fuel depletion; no prescribed duration | User requirement |
| Body diameter | No hard maximum; trade against drag and flow | User direction |
| Maximum takeoff mass | 25 kg | Competition requirement |
| Minimum supersonic time | More than Mach 1 for at least 5 s | Competition requirement |
| Recovery | Land intact; reciprocal same-day flight required | Competition requirement |
| Pulsejet eligibility | Organizer confirmation required | Unresolved rule interpretation |

## Candidate B geometry and mission inputs

| Item | Value | Status and closure path |
|---|---:|---|
| Current loaded mass | 21.0 kg | Component allocations reconcile; weighing and hardware definition required |
| High-side loaded mass | 23.9 kg | Sum of provisional positive uncertainties, not a statistical bound |
| Loaded / ramjet-phase fuel | 3.80 / 1.40 kg | Replace with integrated trajectory fuel ledger |
| Fuel | Representative Jet-A family | Close exact grade, atomization, light-off, safety, and properties |
| Body diameter / length | 0.210 / 2.30 m | Candidate B; 2.5 mm radial selector margin |
| Circular intake / open fraction | 0.195 m / 0.50 | Fixed current architecture |
| Shared throat / `Ae/At` | 0.160 m / 1.05 | Candidate B shared-nozzle compromise |
| Lifting surfaces | Two; 0.16 m exposed semispan, 0.42/0.14 m chords | OpenVSP starting geometry |
| Fins | Four X-clocked; 0.10 m exposed span | OpenVSP starting geometry |
| Field / sled release | 900 m MSL / 39–42 m/s TAS | Prior mission baseline |
| Top of climb | 6,000–6,500 m MSL | Optimize with trajectory and loads |
| Speed-run static point | 4,500 m MSL, Mach 1.10 | Analysis point, not altitude closure |
| Drag-area ceiling | 0.0095 m² at 0.200 m body | Prior conservative budget, not an aerodynamic prediction |

Candidate B's diameter-scaled peak-Mach drag area is 0.010478 m². Until external
aerodynamics closes, changing body diameter scales this budget with diameter squared
while intake diameter remains fixed.

## Propulsion-model assumptions

| Item | Current value | Status and closure path |
|---|---:|---|
| Pulsejet / ramjet total-pressure recovery | 0.99 / 0.92 | Separate conventional total-pressure ratios; both provisional |
| Selector discharge coefficient | 0.78 | Effective-area placeholder |
| Chamber volume | 0.025 m³ | Packaging and acoustic-length closure required |
| Fuel LHV / stoichiometric AFR | 43 MJ/kg / 14.7 | Representative Jet-A values |
| Pulsejet combustion efficiency | 0.58 | High-sensitivity placeholder; calibration required |
| Target equivalence ratio | 0.90 | Operability bounds required |
| Burn duration / minimum period | 4 / 18 ms | Calibrate to pressure histories |
| Ramjet combustor pressure loss | 6% | Placeholder |
| Ramjet target combustor exit | 1,900 K | Close with fuel schedule and material limits |
| Light-off experiment / self-sustaining gate | Mach 0.80 / 1.10 | Separate concepts, not demonstrated operability |

Total-pressure recovery now means exactly
`recovered_total_pressure / ideal_freestream_total_pressure`. The pulsejet and
ramjet inputs are separate because their inlet paths and operating Mach ranges differ.

## Robustness and objective assumptions

`robustness_candidate_b.yaml` contains the complete mass budget, named scenarios,
sweep grid, gates, weights, and normalization. The conservative case combines 0.87
ramjet recovery, a 0.85 installed-thrust factor, 10% drag growth, and 1.5 kg mass
growth. The adverse case combines 0.82 recovery, a 0.75 thrust factor, 20% drag growth,
and the full 2.9 kg high-side mass addition.

Those scenarios are engineering screens, not probability statements. Candidate B
passes the required conservative 50 N excess-thrust budget by about 1.5 N and fails
the informational adverse case by about 154 N.

## Interpretation of current outputs

The nominal static Mach 1.10 calculation predicts 832 N ramjet net thrust and 50.5%
potential-capture spillage. The existing 15% derate leaves about 195 N over the
512 N drag budget. The separate conservative combined-penalty screen leaves only
51.5 N.

The pulsejet model predicts about 122 N steady-window net thrust at sea level and
Mach 0.20 for Candidate B. Neither static result demonstrates transonic acceleration,
mode transition, stability/control, thermal acceptability, or recovery.

Every result remains `numerical_reference_only` until the gates in
[validation.md](validation.md) are closed.
