# Douglas Dart

Douglas Dart is a repository-first engineering model for a research vehicle with
mutually exclusive pulsejet and ramjet intake paths. The near-term goal is a
traceable, low-order simulation that can support sizing trades before geometry is
promoted into OpenVSP/VSPAERO.

This repository is intentionally an engineering notebook in code:

- every calculation uses SI units;
- user requirements, provisional assumptions, and validated inputs are labeled;
- the pulsejet is time-marching and the ramjet is steady-state;
- VSPAERO is reserved for external aerodynamics, not reacting internal flow; and
- generated geometry and results are outputs, while configuration and source code
  remain authoritative.

## Current capability

- 1976-standard-atmosphere-style troposphere/lower-stratosphere model
- compressible reservoir/orifice flow and fixed-area C-D nozzle model
- zero-dimensional pulsejet filling, finite-duration heat release, blowdown, and
  refilling cycle
- cumulative pulsejet control-volume mass and energy conservation audit
- steady ramjet cycle estimate with intake recovery, combustor loss, fixed-nozzle
  mass-flow residual, and minimum-Mach operability flag
- handoff-envelope throat sizing with body-diameter feasibility and spillage reporting
- separate intake and outer-body geometry with a Mach 1.10 packaging/drag sweep
- fuel-limited peak-Mach hold estimates with no prescribed speed-run duration
- mutually exclusive intake-selector interface implementing
  `available area = 0.5 * circular intake area`
- longitudinal point-mass flight-equation kernel
- YAML configuration, CSV output, unit tests, and CI

The supplied `reference_case.yaml` is a numerical demonstration, **not a closed
vehicle design**. Dimensions and performance must not be treated as predictions
until the validation gates in `docs/validation.md` are closed.

## Run locally

```bash
python -m pip install -e .
python -m unittest discover -s tests -v
douglas-dart pulsejet --config configs/reference_case.yaml --duration 0.25
douglas-dart ramjet --config configs/reference_case.yaml --mach 2.0
douglas-dart ramjet-sweep --config configs/reference_case.yaml \
  --minimum-mach 0.8 --maximum-mach 1.1 --mach-step 0.05
douglas-dart diameter-trade --config configs/reference_case.yaml \
  --maximum-body-diameter 0.300 --body-diameter-step 0.005
```

To write the pulsejet trace:

```bash
douglas-dart pulsejet --config configs/reference_case.yaml \
  --duration 0.25 --csv results/pulsejet_reference.csv
```

## Engineering status

The code is suitable for architecture work, sensitivity studies, and identifying
which measurements matter. It is not yet a detailed combustor design tool. It does
not currently model valve dynamics, distributed combustion, detonation, shocks or
separation inside an overexpanded nozzle, heat-soaked material limits, ignition
hardware, or structural loads.

Start with [docs/model_architecture.md](docs/model_architecture.md), then review
[docs/assumptions.md](docs/assumptions.md) and
[docs/validation.md](docs/validation.md) before interpreting results.
