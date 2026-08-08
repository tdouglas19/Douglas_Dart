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
  call contracts;
- one VSPAERO single-point execution per configured nonuniform Mach/alpha/beta entry;
- complete VSPAERO longitudinal-table topology, bilinear interpolation, visible
  boundary clamping, JSON round-trip, and case/reference-area rejection;
- exact additive accounting between VSPAERO inviscid drag area and a separately
  identified supplementary drag area, including a zero-drag limiting case;
- fuel-mass trades that preserve dry mass and reject the 25 kg takeoff-mass limit;
- mission phase/fuel events, full-throttle acceleration logic, and midpoint-step
  consistency.

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

## Candidate B mission numerical checks

Candidate B uses a Mach-zero-to-1.10 pulsejet map, so the 25° climb no longer depends
on low-Mach map clamping. The exact 20 µs map gives:

| Mission step | Peak Mach | Time above Mach 1 | Fuel used | Minimum speed |
|---:|---:|---:|---:|---:|
| 0.100 s | 1.10066 | 9.30 s | 6.8065 kg | 94.15 m/s |
| 0.050 s | 1.10068 | 9.20 s | 6.8025 kg | 94.18 m/s |
| 0.025 s | 1.10009 | 9.20 s | 6.8025 kg | 94.18 m/s |

At 0.05 s, the full-throttle margin is +47.2 N during ramjet acceleration and
+70.8 N during the Mach-target run. This separation proves that the reported run
margin is not borrowed from the dive, while the trajectory itself still depends on
the dive to reach the ramjet state.

The scalar low-order mission gates pass, but the result is not physically validated:
the simulated release is 100 m/s rather than the 39–42 m/s recovered sled target,
forced Mach 0.80 operation is assumed, 50% of uncaptured momentum is assigned as
spillage drag without a flowfield solution, and the trajectory reaches an
angle-of-attack bound for about 14.2 s.

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

The solver-result ingestion path is implemented but does not relax this gate. It
requires a complete beta-zero table with matching case name and reference area,
labels VSPAERO drag as inviscid, and adds the configured parasitic budget as a
separate conservative term. It rejects non-live/synthetic summaries unless a
test-only override is explicitly selected. No live table is available in this
workspace.

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
9. **Mission:** replace the current longitudinal reference with validated aero and
   propulsion maps; add CG, heat, stability/control, wind, six-degree-of-freedom
   dynamics, reciprocal routing, and landing loads; and resolve the 100 m/s versus
   39–42 m/s launch conflict.
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
- Candidate B's scalar mission closure is conditional on a 100 m/s release, forced
  Mach 0.80 ramjet operation, and a 50% spillage-momentum proxy.
- Longitudinal ground contact does not establish intact landing or reciprocal flight.
