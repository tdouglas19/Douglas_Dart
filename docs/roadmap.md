# Development roadmap

## Phase 1 — trustworthy low-order kernel

- Close baseline requirements and unit conventions.
- Add mass/energy residual reporting and time-step convergence tests.
- Build pressure-history, thrust, mass-flow, and fuel-flow plots.
- Run fuel and intake/nozzle sensitivity studies with uncertainty bounds.
- Use the handoff sweep to map the throat/body packaging conflict before selecting
  an intake capture schedule or nozzle geometry.
- Use the peak-Mach diameter trade to bound the drag-area reduction or propulsion
  improvement needed before promoting an outer diameter.

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

The current starting point is a roughly 21 kg loaded vehicle with a 195 mm selector
intake and initially 195 mm outer body. The mission is sled release at 39–42 m/s TAS
near 900 m MSL → climb to 6,000–6,500 m MSL → dive → Mach 0.80 light-off experiment
→ Mach 1.10 peak near 4,500 m MSL → fuel-limited speed run → zoom/glide. Loaded fuel
is 3.80 kg, with 1.40 kg allocated to the ramjet phase. There is no hard maximum body
diameter and no prescribed speed-run duration.

The next owner decisions are recovery method and any hard length constraint. Fuel
selection still follows a scored trade rather than a default. The next engineering
closures are actual Mach-dependent drag, inlet recovery/operability, radial hardware
allowance around the nozzle, and the coupled fuel/mass trajectory.
