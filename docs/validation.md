# Validation plan and current limits

## What automated checks establish

The current suite covers implementation and limiting behavior for:

- standard-atmosphere reference values and monotonic pressure;
- no-flow behavior against a pressure gradient and choked-flow independence from
  downstream pressure;
- subsonic and supersonic area-Mach inversion;
- C-D-nozzle continuity at choking onset and as an internal normal shock reaches the
  exit;
- exact enforcement of the mutually exclusive half-area selector;
- conventional pulsejet and ramjet total-pressure recovery ratios with separate
  configuration inputs;
- finite pulsejet states, ignition/blowdown/refill sequencing, and instantaneous net
  thrust accounting;
- cumulative mass and energy closure, including separate wall and temperature-limit
  rejection ledgers;
- removal of the initially charged pulsejet startup transient from trade averages;
- less than 2% steady mean-thrust change between 40 µs and 20 µs time steps for
  Candidate B;
- ramjet light-off/self-sustaining flag separation and throat-limited spillage;
- fuel-derived hold duration and explicit shared-nozzle feasibility selection;
- component mass-budget reconciliation, high-side mass margin, and named robustness
  scenario selection without hiding the adverse-case failure;
- OpenVSP geometry station, surface-count, clocking, flow-through, and reference-area
  call contracts; and
- one VSPAERO single-point execution per configured nonuniform Mach/alpha/beta entry.

These checks show that code paths behave consistently with their stated equations.
They do not establish engine, aerodynamic, structural, or mission accuracy.

## Candidate B numerical checks

For a 0.25 s warmup plus 0.25 s measurement window:

| Time step | Mean net thrust | Peak pressure in measurement window | Cycles |
|---:|---:|---:|---:|
| 40 µs | 121.76 N | 124.86 kPa | 14 |
| 20 µs | 122.01 N | 124.66 kPa | 14 |
| 10 µs | 121.65 N | 124.58 kPa | 14 |

The prior Candidate A convergence values no longer apply because the inlet recovery
definition and shared throat changed. Candidate B's 20-to-10 µs mean-thrust change is
about 0.30%. This is numerical convergence of a lumped model, not physical validation.

## OpenVSP/VSPAERO verification state

The repository pins the intended API contract to OpenVSP 3.51.2 and rejects the empty
namespace-only `openvsp` package present in some Python environments. Unit tests use a
recording fake to verify parameters and analysis topology.

A live gate remains open because this execution environment does not contain the
functional Python API and VSPAERO binaries shipped with OpenVSP. Consequently:

- no generated `.vsp3` file is claimed as visually inspected;
- no VSPAERO mesh, convergence history, polar, or stability derivative is claimed;
- the 210 mm body and surface arrangement have not passed interference or mesh QA;
- the OpenVSP flow-through settings must be smoke-tested in the pinned application;
  and
- panel results must be checked for wake convergence and compared with an independent
  method before entering the flight model.

## Minimum gates before design conclusions

1. **Rules:** obtain written organizer confirmation that the pulsejet architecture is
   eligible.
2. **Fuel and properties:** replace representative Jet-A values with cited ranges,
   temperature dependence, ignition constraints, and a scored fuel-system trade.
3. **Pulsejet calibration:** compare frequency, pressure ratio, blowdown shape, net
   impulse, and fuel flow with a geometrically relevant published or measured case.
4. **Pulsejet physics:** add valve/selector dynamics, distributed gas dynamics,
   acoustic closure, species/variable properties, and a real thermal network where
   sensitivity or calibration demands them.
5. **Nozzle:** validate the quasi-one-dimensional shock branch and add separation and
   side-load limits before trusting expansion-ratio selection.
6. **Ramjet inlet coupling:** solve external compression, terminal-shock position,
   distortion, unstart margin, back-pressure compatibility, and the current 50.5%
   spillage stream.
7. **Combustion and transition:** establish light-off, flameholding, pressure loss,
   selector timing, trapped-volume, and transient mode-change behavior.
8. **External aerodynamics:** run and visually inspect the pinned OpenVSP model;
   complete mesh/wake convergence; add parasite, wave, base, and inlet drag; and
   compare against another appropriate method or experiment.
9. **Mission:** integrate sled release, pulsejet climb, dive, Mach-1 crossing, ramjet
   operation, fuel depletion, zoom/glide, and landing while tracking mass, CG,
   dynamic pressure, heat, stability, and control authority.
10. **Hardware:** complete independent structural, combustion, fuel-system, test-cell,
    fire, acoustics, range, and recovery safety reviews before physical testing.

## Known interpretation cautions

- Candidate B passes its conservative static screen by only about 1.5 N over the
  provisional 50 N excess-thrust budget and fails the adverse screen by about 154 N.
- The 210 mm body drag uses a diameter-squared budget, not VSPAERO data.
- The 160 mm throat is a static robustness compromise; its pulsejet/ramjet transition
  performance is not closed.
- The 19 s full-throttle endurance excludes acceleration fuel and transition losses.
- The 23.9 kg high-side mass is an allocation sum, not a weighed or statistically
  validated vehicle mass.
- A self-sustaining flag checks configured thresholds only; it is not a stability
  prediction.
- VSPAERO drag is labeled inviscid and cannot replace total drag by itself.
