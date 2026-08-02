# Development roadmap

## Phase 1 — trustworthy low-order kernel

- Close baseline requirements and unit conventions.
- Add mass/energy residual reporting and time-step convergence tests.
- Build pressure-history, thrust, mass-flow, and fuel-flow plots.
- Run fuel and intake/nozzle sensitivity studies with uncertainty bounds.
- Use the handoff sweep to map the throat/body packaging conflict before selecting
  an intake capture schedule or nozzle geometry.

## Phase 2 — geometry and external aerodynamics

- Install a pinned OpenVSP version in a reproducible environment.
- Generate the axisymmetric body, intake reference, fins, and minimal lifting surfaces
  from versioned parameters.
- Run VSPAERO sweeps in Mach, angle of attack, sideslip, and control state.
- Export aerodynamic tables with geometry/configuration provenance.

## Phase 3 — coupled mission model

- Couple cycle-averaged pulsejet maps and steady ramjet maps to the trajectory model.
- Implement sled release, climb, gravity-assisted acceleration, mode transition,
  speed run, zoom climb, and glide/recovery phases.
- Track CG, propellant mass, dynamic pressure, heat-load proxies, stability margin,
  and landing reserve.

## Phase 4 — higher-fidelity closure

- Promote the largest sensitivities to quasi-1D gas dynamics or CFD.
- Add thermal soak, structural loads, controls, and test uncertainty.
- Define a safe, instrumented subscale validation sequence with independent review.

## Required owner decisions

The current starting point is a roughly 21 kg loaded vehicle with a 195 mm nominal
body/intake and a sled → climb → dive → Mach 0.80 light-off test → Mach 1.1–1.3
handoff search → speed run → zoom/glide profile. The next design branch still needs
target peak Mach, speed-run duration, maximum altitude, sled release speed, launch
site elevation, recovery method, and hard length constraints. Fuel selection follows
a scored trade rather than a default.
