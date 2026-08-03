# Candidate A design convergence

## Decision

Carry **Candidate A** as the next geometry and coupled-model point:

- 205 mm outer body;
- 2.30 m length;
- fixed 195 mm circular intake with one-half available to the selected mode;
- 130 mm shared throat;
- `Ae/At = 1.05` shared exit;
- representative Jet-A fuel; and
- Mach 1.10 at 4,500 m MSL as the current peak analysis point.

This is a lower-bound research configuration, not a frozen vehicle. It is the
smallest tested architecture that packages the configured selector allowance and
closes the conservative Mach 1.10 drag budget after a 15% propulsion derate.

## Why the earlier full-capture concept was rejected

At Mach 1.10 and 4,500 m, swallowing all potential flow from the half-open 195 mm
intake required a roughly 223 mm hot-gas throat in the low-order model. Enlarging the
body and intake together cannot cure that conflict because captured flow and throat
capacity both scale approximately with area.

The design therefore keeps intake diameter fixed, chooses a packageable shared
throat, and accepts intentional ramjet spillage. This trades mass flow for smaller
body diameter and lower drag.

The original `Ae/At = 2.25` placeholder was also rejected. At the low total-pressure
ratio near Mach 1.10 it was strongly overexpanded and created a severe thrust penalty.
The current model prefers less expansion in both modes, leading to the 1.05 boundary.

## Candidate A numerical balance

| Quantity | Value |
|---|---:|
| Diameter-scaled drag area budget | 0.009981 m² |
| Drag-budget force at Mach 1.10 | 488.0 N |
| Ramjet gross thrust | 1,050.2 N |
| Ramjet inlet momentum drag | 447.0 N |
| Ramjet net thrust | 603.2 N |
| 15%-derated net thrust | 512.7 N |
| Derated thrust margin | 24.7 N |
| Air / fuel flow | 1.260 / 0.05075 kg/s |
| Potential-capture spillage | 66.0% |
| Full-throttle endurance from 1.40 kg | 27.6 s |
| Linear hold-throttle fraction after derate | 95.2% |
| Static fuel-limited hold / distance | 29.0 s / 10.3 km |
| Configured loaded-mass margin to 25 kg | 4.0 kg |

The static hold exceeds the five-second requirement by a factor of about 5.8. It is
not a compliance margin because acceleration fuel, real drag, transition losses, and
inlet operability remain open.

## Speed-run altitude remains open

The repeatable static altitude sweep holds Mach, geometry, drag-area budget, and
propulsion derate fixed:

| Altitude | Drag | Ramjet net | Derated margin | Fuel flow | Static hold |
|---:|---:|---:|---:|---:|---:|
| 3,000 m | 592.7 N | 722.2 N | 21.2 N | 0.06117 kg/s | 23.7 s |
| 4,500 m | 488.0 N | 603.2 N | 24.7 N | 0.05075 kg/s | 29.0 s |
| 6,000 m | 398.9 N | 500.0 N | 26.1 N | 0.04178 kg/s | 35.7 s |
| 6,500 m | 372.3 N | 468.9 N | 26.3 N | 0.03909 kg/s | 38.3 s |

The low-order static model prefers higher altitude for endurance and dynamic pressure.
It excludes climb/diving energy, lower-density light-off/flameholding, control
authority, and actual Mach-dependent drag. Therefore 4,500 m remains a mission
starting point rather than a frozen speed-run altitude.

## Local feasibility bounds

Holding altitude, Mach, `Ae/At`, component assumptions, drag proxy, and the 15% derate
fixed gives:

| Bound | Value | Candidate margin |
|---|---:|---:|
| Minimum body from configured packaging allowances | 205.00 mm | 0.00 mm |
| Maximum body supported by derated thrust/drag proxy | 210.12 mm | 5.12 mm |
| Minimum throat at 205 mm body | 126.84 mm | 3.17 mm diameter |

These bounds show that Candidate A is useful for the next iteration but not robustly
closed. A few percent of real drag growth or inlet recovery loss can consume the
24.7 N modeled reserve.

## Pulsejet repeating-cycle result

The initially charged chamber creates a large startup impulse, so all design values
discard the first 0.25 s and measure the next 0.25 s.

| Quantity at sea level, Mach 0.20 | 20 µs result |
|---|---:|
| Mean gross / net thrust | 225.8 / 185.2 N |
| Mean fuel flow | 0.03656 kg/s |
| Peak chamber pressure in measurement window | 146.3 kPa |
| Peak chamber temperature in measurement window | 2,058 K |
| Peak instantaneous net thrust in measurement window | 1,043 N |
| Cycles in 0.25 s | 14 |
| Mean net thrust / loaded weight | 0.90 |

The peak instantaneous value is a zero-dimensional output and must not be used as a
structural load. The 20-to-10 µs change in mean net thrust is 0.27%. The 2,600 K
temperature limiter rejects no energy in the 0.50 s Candidate A run.

## Key local variables

The normalized slope is the fractional output change divided by fractional input
change over a centered ±10% perturbation. It is local and does not include parameter
uncertainty or interactions.

| Engine | Variable | Net-thrust normalized slope | Interpretation |
|---|---|---:|---|
| Ramjet | Throat diameter | +2.00 | Dominant because the point is throat-limited |
| Pulsejet | Equivalence ratio | +1.21 | Thrust and fuel flow both move strongly |
| Pulsejet | Fuel LHV | +1.21 | Same heat-release product as combustion efficiency in this model |
| Pulsejet | Combustion efficiency | +1.21 | Must be calibrated; cannot be a tuning knob |
| Pulsejet | Selector discharge coefficient | +1.20 | Effective open area is critical |
| Ramjet | Total-pressure recovery | +1.16 | Inlet design/operability is a first-order risk |
| Pulsejet | Total-pressure recovery | +0.63 | Intake loss matters in pulse mode too |
| Pulsejet | Burn duration | -0.62 | Heat-release timing matters to useful pressure work |
| Ramjet | Combustor exit temperature | +0.41 | Raises thrust and fuel flow |
| Ramjet | Exit/throat area ratio | -0.39 | More expansion hurts this local overexpanded point |

Ramjet mass-capture coefficient has zero local thrust sensitivity because the fixed
throat already limits accepted flow; it changes potential spillage rather than
throughflow. Fuel LHV and combustor efficiency strongly change ramjet fuel flow but
barely change thrust at fixed target combustor temperature.

## OpenVSP geometry promoted for the next gate

The generated external geometry uses body stations at 0.00, 0.18, 0.99, 1.80, and
2.30 m with diameters 195, 205, 205, 205, and 133.21 mm. Two lifting surfaces and four
X-clocked fins are parameterized in YAML. Candidate A's VSPAERO reference quantities
are 0.0896 m², 0.525 m, and 0.3033 m.

The API contract is tested, but no live `.vsp3` or VSPAERO polar is claimed from this
workspace because its imported `openvsp` module lacks the compiled API. The next
geometry decision must follow live build, visual QA, and mesh/wake convergence.

## Do not freeze yet

Keep the following variables open:

- 205–210 mm body diameter;
- 127–132 mm throat neighborhood;
- minimal but genuinely converging-diverging exit ratio;
- lifting-surface area and placement;
- fin area, clocking, and longitudinal location;
- 4,500–6,500 m ramjet operating altitude;
- intake recovery, capture/spillage schedule, and unstart margin;
- chamber volume and pulse timing; and
- fuel allocation and final fuel choice.

The first freeze should occur only after external-drag closure and a coupled
inlet/back-pressure model show a nontrivial reserve at Mach 1.10.
