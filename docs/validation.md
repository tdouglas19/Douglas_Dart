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
- finite pulsejet states, ignition/blowdown/refill sequencing, and instantaneous net
  thrust accounting;
- cumulative mass and energy closure, including separate wall and temperature-limit
  rejection ledgers;
- removal of the initially charged pulsejet startup transient from trade averages;
- less than 2% steady mean-thrust change between 40 µs and 20 µs time steps for
  Candidate A;
- ramjet light-off/self-sustaining flag separation and throat-limited spillage;
- fuel-derived hold duration and explicit shared-nozzle feasibility selection;
- OpenVSP geometry station, surface-count, clocking, flow-through, and reference-area
  call contracts; and
- one VSPAERO single-point execution per configured nonuniform Mach/alpha/beta entry.

These checks show that code paths behave consistently with their stated equations.
They do not establish engine, aerodynamic, structural, or mission accuracy.

## Candidate A numerical checks

For a 0.25 s warmup plus 0.25 s measurement window:

| Time step | Mean net thrust | Peak pressure in measurement window | Cycles |
|---:|---:|---:|---:|
| 40 µs | 186.91 N | 146.68 kPa | 14 |
| 20 µs | 185.20 N | 146.32 kPa | 14 |
| 10 µs | 184.70 N | 146.11 kPa | 14 |

The 20-to-10 µs mean-thrust change is about 0.27%. Over the full 0.50 s, mass and
energy ledgers close to floating-point tolerance. The run rejects about 6.76 kJ
through the provisional wall-loss term and zero energy through the 2,600 K numerical
temperature limiter, so that limiter is not creating Candidate A's steady result.

This is numerical convergence of a lumped model, not physical validation of its
pressure amplitude, frequency, or thrust.

## OpenVSP/VSPAERO verification state

The repository pins the intended API contract to OpenVSP 3.51.2 and rejects the empty
namespace-only `openvsp` package present in some Python environments. Unit tests use a
recording fake to verify parameters and analysis topology.

A live gate remains open because this execution environment does not contain the
functional Python API and VSPAERO binaries shipped with OpenVSP. Consequently:

- no generated `.vsp3` file is claimed as visually inspected;
- no VSPAERO mesh, convergence history, polar, or stability derivative is claimed;
- the 205 mm body and surface arrangement have not passed interference or mesh QA;
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
   distortion, unstart margin, back-pressure compatibility, and the current 66%
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

- Candidate A's 24.7 N derated Mach 1.10 margin is small relative to unresolved drag
  and inlet uncertainty.
- The 205–210.1 mm body interval uses a diameter-squared drag budget, not VSPAERO data.
- The 126.84 mm throat boundary holds area ratio, altitude, Mach, and component
  assumptions fixed.
- The 29 s fuel estimate assumes linear thrust/fuel turndown and excludes acceleration
  fuel.
- A self-sustaining flag checks configured thresholds only; it is not a stability
  prediction.
- VSPAERO drag is labeled inviscid and cannot replace total drag by itself.
