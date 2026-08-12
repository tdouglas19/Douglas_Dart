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

## Headline results (FP-1 reference design, sea level)

Static (M=0), production resolution (N=300, ~30 settled cycles): sustained
limit cycle at **~18–19 N, 160–162 Hz**, chamber pressure swinging
0.77–1.65× ambient, strongly positive Rayleigh index (heat release
phase-locked to the pressure wave — emergent, not imposed). The two
independent thrust formulations (exit momentum flux, eq. 24, vs the exact
interior-surface momentum-closure identity, eq. 25) agree to <1%.

Thrust vs flight Mach (`out/thrust_vs_mach_FINAL.png`):

- **Side-mounted boundary-layer intake** (inlets perpendicular to flight
  velocity, static-pressure feed — derivation #8c): operates across the
  entire sweep **M 0 → 0.9, every point a converged limit cycle**, thrust
  declining gently 19 → 11.6 N (boundary-layer momentum drag + recovery-
  heated, less-dense charge). Cold-start and operating branches coincide:
  the engine is air-startable at any speed in this configuration, because
  no steady ram bias ever loads the reed petals.
- **Forward ram intake** (comparison): ram supercharging peaks at
  ~25 N near M 0.15–0.20 (+30% over static), then the plenum's steady ram
  bias holds the petals off their seats, the valve stops rectifying, and
  the engine quenches by M 0.40 — even when approached with the engine
  already running.

Uncertainty, stated honestly: discretization ~±5% (N=300 vs N=450 grid
study); the near-threshold oscillator amplifies closure constants (a +3.4%
heat-release change moved static thrust +34%), so absolute thrust carries
the assumption registry's stated closure uncertainty, while curve shapes,
frequencies, and mechanisms (valve rectification, quench boundaries,
branch structure) are the robust outputs. Adiabatic walls (A6) make
thrust/TSFC optimistic at this engine scale.

## Layout

- `src/pulsejet_fp/` — model (`gas`, `geometry`, `valve`, `orifice`,
  `solver`, `engine`, `query`, `diagnostics`, `atmosphere`)
- `docs/derivation.md` — the first-principles derivation
- `tests/` — exact-Riemann Sod validation, machine-precision conservation,
  acoustic mode frequency, beam/orifice closed-form checks, engine behavior
- `scripts/` — reference run + thrust-vs-Mach sweep
- `out/` — generated plots
