# Assumption register

`shared_nozzle_candidate_b.yaml` is the active static trade point. Values in it are
not automatically requirements or validated parameters.

## Requirements and owner direction

The competition rows below are sourced directly from
[boomsupersonic.com/prize](https://boomsupersonic.com/prize) (fetched 2026-08-05) and
encoded in `RequirementsConfig`. This replaces the earlier "organizer confirmation
required" placeholder for pulsejet eligibility, which the prize page explicitly
resolves.

| Item | Current value | Status |
|---|---:|---|
| Intake modes | Pulsejet or ramjet, never both open | User requirement |
| Available geometric intake area (pulsejet) | 50% of the 195 mm circular intake | User requirement / prior intake diameter |
| Available geometric intake area (ramjet) | 100% of the 195 mm circular intake | User direction, 2026-08-07: the switchable selector fully closes the pulsejet path when ramjet is active, so nothing requires halving the ramjet's captured area (previously also capped at 50%, `ramjet.py`'s `potential_air_mass_flow_kg_per_s`) |
| Peak Mach | 1.10 | User requirement (exceeds the sonic minimum below) |
| Speed-run termination | Fuel depletion; no prescribed duration | User requirement |
| Body diameter | No hard maximum; trade against drag and flow | User direction |
| Maximum takeoff mass | 25 kg / 55 lb, including fuel | Competition requirement (boomsupersonic.com/prize) |
| Minimum supersonic time | True airspeed above the local speed of sound, sustained 5+ continuous seconds | Competition requirement (boomsupersonic.com/prize) |
| Transonic acceleration flight path | Mach 0.8 to past Mach 1 must be flown level or climbing, **no altitude loss** | Competition requirement (boomsupersonic.com/prize); now enforced in `trajectory.py`'s dive/ramjet_accel/mach_hold phases and audited on every run via `transonic_no_altitude_loss_rule_satisfied` |
| Allowed propulsion | Turbojet, turbofan, ramjet, or pulsejet, any combination; no rockets, no onboard oxidizer, air-breathing only | Competition requirement (boomsupersonic.com/prize); confirms the pulsejet+ramjet architecture is eligible |
| Configuration | Fixed-wing airplane, lift from aerodynamic surfaces | Competition requirement (boomsupersonic.com/prize) |
| Control | Remote human pilot, continuous command **and abort authority** | Competition requirement (boomsupersonic.com/prize); not yet reflected in any avionics/control-link design — open item |
| Recovery | Controlled landing (wheeled or belly) on the designated area; reusable without replacing major components; reciprocal-heading same-day dual flight on the same airframe | Competition requirement (boomsupersonic.com/prize) |
| Verification | Calibrated pitot-static + total air temperature, sealed data loggers, GPS telemetry, reciprocal runs | Competition requirement (boomsupersonic.com/prize); drives the "Instrumentation and testability" requirement category, not yet built out |
| Eligibility | Amateur team (US citizens/permanent residents), majority hand-built, no venture capital/corporate sponsorship/government grants | Competition requirement (boomsupersonic.com/prize); non-engineering, tracked for awareness only |

## Candidate B geometry and mission inputs

| Item | Value | Status and closure path |
|---|---:|---|
| Current loaded mass | 21.0 kg | Component allocations reconcile; weighing and hardware definition required |
| High-side loaded mass | 23.9 kg | Sum of provisional positive uncertainties, not a statistical bound |
| Loaded / ramjet-phase fuel | 3.80 / 1.40 kg | Replace with integrated trajectory fuel ledger |
| Fuel | Representative Jet-A family | Close exact grade, atomization, light-off, safety, and properties |
| Body diameter / length | 0.210 / 2.30 m | Candidate B; 2.5 mm radial selector margin |
| Circular intake / open fraction | 0.195 m / 0.50 | Fixed current architecture |
| Shared throat / `Ae/At` | 0.170 m / 1.05 | Candidate B shared-nozzle compromise; revised from 0.160 m after the ramjet inlet-recovery correction below |
| Lifting surfaces | Two; 0.16 m exposed semispan, 0.42/0.14 m chords | OpenVSP starting geometry |
| Fins | Four X-clocked; 0.10 m exposed span | OpenVSP starting geometry |
| Field elevation | 900 m MSL | Prior mission baseline |
| Sled release speed | 39–42 m/s TAS in this static config; **reclassified as a Level 2 search variable** (35–90 m/s) in `optimizer.py`, not a fixed requirement -- see `docs/assumptions_registry.md` | Bounded by `sled_rail_length_m` (75 m, representative) and an as-yet-unset launch-acceleration limit; raising release speed directly closes some of the Gate 1 stall-speed margin found in `docs/level0_feasibility_bounds.md` |
| Top of climb | 6,000–6,500 m MSL | Optimize with trajectory and loads |
| Speed-run static point | 4,500 m MSL, Mach 1.10 | Analysis point, not altitude closure |
| Drag-area ceiling | 0.0095 m² at 0.200 m body | Prior conservative budget, not an aerodynamic prediction |

Candidate B's diameter-scaled peak-Mach drag area is 0.010478 m². Until external
aerodynamics closes, changing body diameter scales this budget with diameter squared
while intake diameter remains fixed.

## Trajectory-model derived quantities (no longer bare constants)

Two previously hardcoded literals in `trajectory.py` are now computed from the
vehicle's actual state instead of one fixed number for the whole flight:

| Item | Was | Now | Basis |
|---|---|---|---|
| Dive pull-out altitude floor | fixed `+50 m` | `V^2(1-cos(gamma))/(g(n-1))` at the current speed | Constant-load-factor circular-arc pull-out, `pull_out_load_factor_g` (default 4.0 g, provisional pending a real structural limit) |
| Zoom-climb exit speed | fixed `60 m/s` | `1.3 * V_stall(mass, altitude)` | `V_stall = sqrt(2W/(rho S CLmax))`; 1.3x is the standard approach-speed margin convention |

`maximum_lift_coefficient` (new `FlightConfig` field, 0.90 in all three configs) is
a literature-typical placeholder for a thin unflapped section pending real airfoil
data — same status as `lift_curve_slope_per_rad`.

## Propulsion-model assumptions

| Item | Current value | Status and closure path |
|---|---:|---|
| Pulsejet / ramjet installed-efficiency recovery factor | 0.99 / 0.92 | Multiplies the idealized Mach-dependent MIL-E-5008B recovery (`ideal_inlet_shock_recovery`, 1.0 below Mach 1); both factors are still provisional |
| Selector discharge coefficient | 0.78 | Effective-area placeholder |
| Chamber volume | 0.025 m³ | Packaging and acoustic-length closure required |
| Fuel LHV / stoichiometric AFR | 43 MJ/kg / 14.7 | Representative Jet-A values |
| Pulsejet combustion efficiency | 0.58 | High-sensitivity placeholder; calibration required |
| Target equivalence ratio | 0.90 | Operability bounds required |
| Burn duration / minimum period | 4 / 18 ms | Calibrate to pressure histories |
| Ramjet combustor pressure loss | 6% | Placeholder |
| Ramjet target equivalence ratio | 0.60 | Backed out of this vehicle's previous fixed-1,900 K combustor-exit target so the two models stay comparable; combustor exit temperature (T4) is now a computed output that varies with Mach (~1,884-1,950 K over the operating range) rather than a fixed input -- close with fuel schedule and material limits |
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
(170 mm throat) passes the required conservative 50 N excess-thrust budget by about
128.8 N and fails the informational adverse case by about 96 N. (The prior 160 mm
throat passed the conservative screen by only 1.5 N before the ramjet inlet-recovery
correction below reduced that to -0.24 N, which is why the throat moved to 170 mm.)

## Interpretation of current outputs

The nominal static Mach 1.10 calculation predicts 937 N ramjet net thrust and 44.2%
potential-capture spillage. The existing 15% derate leaves about 284 N over the
512 N drag budget. The separate conservative combined-penalty screen leaves 128.8 N.

The pulsejet model predicts about 119 N steady-window net thrust at sea level and
Mach 0.20 for Candidate B. Neither static result demonstrates transonic acceleration,
mode transition, stability/control, thermal acceptability, or recovery.

Every result remains `numerical_reference_only` until the gates in
[validation.md](validation.md) are closed.
