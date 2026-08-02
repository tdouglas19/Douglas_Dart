# Validation plan and current limits

## What the automated tests establish

- sea-level atmosphere reference and monotonic pressure behavior;
- correct no-flow behavior against a pressure gradient;
- choked-flow independence from downstream pressure;
- numerical inversion of the supersonic area-Mach relation;
- explicit enforcement of the half-area selector requirement;
- finite positive pulsejet state through a short integration;
- subsonic ramjet operability flagging; and
- flight-model sign conventions for fuel burn and drag.

These are implementation and limiting-case checks. They do not establish engine
performance accuracy.

## Minimum gates before design conclusions

1. **Property provenance:** replace provisional fuel and gas properties with cited
   ranges and uncertainty distributions.
2. **Pulse frequency:** compare predicted period, pressure ratio, and blowdown shape
   against at least one geometrically relevant published or measured pulsejet case.
3. **Mass/energy audit:** bound per-cycle mass residual and energy accounting over a
   converged time-step sweep.
4. **Nozzle regime:** add normal-shock/separation logic or constrain analysis to cases
   where the ideal fixed-exit assumption is defensible.
5. **Ramjet matching:** solve combustor pressure/nozzle flow compatibility instead of
   merely reporting the residual.
6. **Transition:** model selector timing, trapped volumes, leakage, and ignition during
   the pulsejet-to-ramjet handoff.
7. **Aerodynamics:** replace the placeholder drag polar with VSPAERO tables, then
   cross-check wave/skin-friction drag with methods appropriate to the Mach range.
8. **Flight controls and loads:** add static/dynamic stability, control authority,
   structural loads, and recovery constraints.
9. **Hardware safety:** complete independent combustion, fuel-system, test-cell, fire,
   acoustics, and range-safety reviews before physical testing.

## Known numerical cautions

- Use a time step at least 100 times smaller than the configured burn duration for
  reference runs.
- A temperature ceiling currently represents omitted dissociation/variable-property
  effects; rejected energy is tracked but not yet surfaced in the CLI summary.
- Pressure-thrust clipping in severe overexpansion prevents a low-order artifact from
  producing negative gross thrust. The result is flagged and should not be trusted.
- A self-sustaining ramjet flag means only that configured gates passed; it is not a
  combustion-stability prediction.
- The ramjet spillage treatment bounds flow through an undersized nozzle but is not
  a coupled inlet/back-pressure solution.
