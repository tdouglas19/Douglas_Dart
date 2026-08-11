# pulsejet-fp

First-principles transient cycle model of a **mechanically valved pulsejet**
(reed/petal valves), built from fundamental physical laws only — no external
pulsejet source code, textbook pulsejet correlations, empirical discharge
coefficients, or pre-set valve delay times.

The complete derivation (every governing equation, the discretization, the
stability constraints, and a numbered registry of every assumption with its
physical-law justification) is in [docs/derivation.md](docs/derivation.md).
The running technical log is [architecture.md](architecture.md).

## Model in one paragraph

The whole engine interior (valve head → chamber → cone → tailpipe → exit) is
a single unsteady quasi-1D compressible flow domain (mass/momentum/energy/
reactant-species conservation, shock-capturing HLLC finite volume, MUSCL
2nd order), so tailpipe shock/rarefaction waves, the Kadenacy suction phase,
and exit backflow all *emerge* from the equations. Combustion is single-step
Arrhenius kinetics with a jet-strain extinction closure derived from
activation-energy asymptotics; ignition phasing therefore emerges from
wave-driven compression heating rather than a fitted delay — the Rayleigh
criterion is computed each cycle as a diagnostic of the resulting coupling.
The valve is a genuine spring–mass–damper ODE: stiffness `3EI/L^3` and modal
mass `(33/140) m` from Euler–Bernoulli beam theory, forced by the pressure
differential, with seat/stop contact and restitution; its geometric curtain
area feeds a choked-orifice boundary flux derived from energy conservation
(choking condition derived, not imposed).

## Quick start

```bash
.venv/Scripts/python scripts/run_reference.py --mach 0.0
.venv/Scripts/python scripts/thrust_vs_mach.py
.venv/Scripts/python -m pytest tests -q
```

Primary API:

```python
from pulsejet_fp import pulsejet_thrust
res = pulsejet_thrust(mach=0.3)   # FP-1 reference design, sea level
print(res.thrust_n, res.frequency_hz, res.status)
```

`ThrustResult` also carries frequency, air/fuel flows, TSFC, pressure ratio
extremes, the Rayleigh index, an independent surface-pressure-integral
thrust cross-check (eq. 25), and optional full time traces.

## Layout

- `src/pulsejet_fp/` — model (`gas`, `geometry`, `valve`, `orifice`,
  `solver`, `engine`, `query`, `diagnostics`, `atmosphere`)
- `docs/derivation.md` — the first-principles derivation
- `tests/` — exact-Riemann Sod validation, machine-precision conservation,
  acoustic mode frequency, beam/orifice closed-form checks, engine behavior
- `scripts/` — reference run + thrust-vs-Mach sweep
- `out/` — generated plots
