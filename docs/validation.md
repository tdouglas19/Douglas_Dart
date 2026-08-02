# Validation plan and current limits

## What the automated tests establish

- sea-level atmosphere reference and monotonic pressure behavior;
- correct no-flow behavior against a pressure gradient;
- choked-flow independence from downstream pressure;
- numerical inversion of the supersonic area-Mach relation;
- numerical inversion of the subsonic area-Mach relation;
- mass-flow and thrust continuity at C-D-nozzle choking onset;
- internal normal-shock location for overexpanded C-D-nozzle operation;
- explicit enforcement of the half-area selector requirement;
- finite positive pulsejet state through a short integration;
- cumulative pulsejet mass and energy ledger closure to floating-point tolerance;
- subsonic ramjet operability flagging; and
- flight-model sign conventions for fuel burn and drag.

These are implementation and limiting-case checks. They do not establish engine
performance accuracy.

## Minimum gates before design conclusions

1. **Property provenance:** replace provisional fuel and gas properties with cited
   ranges and uncertainty distributions.
2. **Pulse frequency:** compare predicted period, pressure ratio, and blowdown shape
   against at least one geometrically relevant published or measured pulsejet case.
3. **Mass/energy audit:** the cumulative implementation ledger now closes; next bound
   per-cycle residuals and physical-model error over a converged time-step sweep.
4. **Nozzle regime:** validate the new quasi-one-dimensional normal-shock branch and
   add separation criteria before trusting strongly overexpanded points.
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
  effects; rejected energy is surfaced in the CLI conservation audit.
- Internal normal shocks are treated as one-dimensional and stationary. Boundary-layer
  separation and transient shock motion remain unresolved and flagged.
- A self-sustaining ramjet flag means only that configured gates passed; it is not a
  combustion-stability prediction.
- The ramjet spillage treatment bounds flow through an undersized nozzle but is not
  a coupled inlet/back-pressure solution.
