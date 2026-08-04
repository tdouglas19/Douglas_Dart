# Development roadmap

## Completed foundation

- Requirements, owner direction, and provisional assumptions are separate in config.
- Pulsejet mass/energy ledgers and C-D-nozzle regime continuity are implemented.
- Pulsejet trade averages exclude startup and have an automated time-step check.
- Ramjet full-capture sizing exposed the original throat/body conflict.
- Conventional total-pressure recovery replaced the earlier ram-rise interpretation
  and rejected Candidate A as a static reserve point.
- A fixed shared-nozzle/spillage trade plus named robustness screens produced
  Candidate B at 210 mm body and 160 mm throat.
- A component mass budget reconciles to 21.0 kg and exposes a 23.9 kg high-side case.
- Fuel-limited Mach 1.10 hold replaces prescribed speed-run duration.
- A 3,000–6,500 m static altitude sweep keeps the speed-run altitude open and shows
  the current model's high-altitude endurance preference.
- Local pulsejet and ramjet key-variable sensitivities are repeatable.
- A performance-only fuel trade supports Jet-A/JP-8-class kerosene as the baseline
  while keeping hardware, operability, and safety criteria open.
- Parameter-driven OpenVSP geometry and explicit-point VSPAERO runner are implemented
  against the official 3.51.2 API contract.

## Next: live external-aerodynamics closure

1. Run `openvsp-build` in a functional OpenVSP 3.51.2 Python environment.
2. Inspect open inlet/outlet topology, body/surface intersections, surface clocking,
   trailing edges, and tessellation in the GUI.
3. Run panel mesh and wake-iteration convergence at Mach 0.2, 0.8, 1.0, and 1.1.
4. Add OpenVSP Parasite Drag and Wave Drag workflows with explicit surface properties.
5. Establish a base-drag and inlet/spillage-drag model; convert every contribution to
   one documented drag area.
6. Replace the diameter-squared budget with a conservative solver-backed table only
   after cross-checking it.

## Next: inlet and propulsion closure

1. Promote the now-conventional ramjet total-pressure recovery and spillage inputs
   into an inlet/back-pressure map with unstart margin.
2. Replace the pulsejet's lumped acoustic assumption with the minimum fidelity needed
   to match a relevant pressure trace.
3. Sweep chamber volume, selector effective area, equivalence ratio, heat release,
   throat, and expansion ratio over altitude and Mach—not only the local point.
4. Trade representative Jet-A/JP-8 against other permitted fuels for light-off,
   atomization, safety, storage, energy density, and hardware mass.
5. Model transition timing, leakage, trapped volume, and igniter/flameholder behavior.

## Coupled mission model

1. Build a phase manager for sled release → pulsejet climb → dive/acceleration →
   light-off experiment → ramjet handoff → fuel-limited run → zoom/glide → landing.
2. Interpolate cycle-averaged pulsejet maps, ramjet maps, and aerodynamic tables.
3. Track fuel by phase, CG, dynamic pressure, heat proxies, stability/control margins,
   and recovery reserve.
4. Optimize speed-run altitude and mode-transition schedule rather than treating
   4,500 m as closed.
5. Extend the current named static robustness cases through the full trajectory,
   including off-nominal atmosphere and failed light-off, before judging the
   five-second and reciprocal-flight requirements.

## Decisions still owned by the user

- recovery method;
- any hard length, transport, or launch-rail envelope;
- whether 21 kg and the 3.8/1.4 kg fuel allocations become hard bounds or remain
  optimization variables; and
- final fuel choice after a sourced scored trade.

None blocks the current software/aerodynamics work. Pulsejet rule confirmation is an
external rules gate and should be obtained before committing to hardware.
