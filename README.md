# Douglas Dart

Douglas Dart is a repository-first trade study for a research vehicle with mutually
exclusive pulsejet and ramjet intake paths. Versioned YAML is authoritative; Python
generates the propulsion calculations, external geometry, and analysis inputs.

The project deliberately separates three levels of evidence:

- user and competition requirements;
- provisional design parameters and engineering budgets; and
- physically validated inputs or results.

Green tests establish implementation consistency. They do not turn the low-order
engine model into validated hardware performance.

## Current convergence point

`configs/shared_nozzle_candidate_a.yaml` is the first coupled candidate, not a frozen
design.

| Parameter or result | Candidate A numerical value |
|---|---:|
| Loaded reference mass / maximum allowed | 21.0 / 25.0 kg |
| Body diameter / length | 205 mm / 2.30 m |
| Circular intake / area available to one mode | 195 mm / 50% |
| Shared throat / exit-to-throat area ratio | 130 mm / 1.05 |
| Mach 1.10 altitude | 4,500 m MSL |
| Ramjet net thrust / 15%-derated net thrust | 603 / 513 N |
| Conservative drag-budget force | 488 N |
| Derated thrust margin | 24.7 N |
| Ramjet potential-capture spillage | 66.0% |
| Fuel-limited static hold estimate | 29.0 s / 10.3 km |
| Steady-window pulsejet net thrust at sea level, Mach 0.20 | about 185 N |

The local fixed-architecture bounds are narrow: approximately 205–210.1 mm body
diameter and a minimum 126.84 mm throat at the 205 mm body point. Candidate A has no
extra body margin beyond its configured 5 mm radial selector allowance and about
3.17 mm of throat-diameter margin above the modeled derated-thrust boundary.

The 29-second result only says the static low-order fuel balance exceeds the
five-second requirement. Acceleration through Mach 1, inlet operability, drag closure,
and the coupled trajectory are not yet demonstrated. See
[docs/design_convergence.md](docs/design_convergence.md) for the decision record.

## Current capability

- standard-atmosphere, compressible-orifice, normal-shock, and fixed C-D-nozzle tools;
- zero-dimensional pulsejet filling, finite heat release, blowdown, refill, and full
  mass/energy ledgers;
- startup-excluded pulsejet statistics and time-step convergence checks;
- steady ramjet heat addition, pressure loss, fixed-nozzle flow capacity, inlet
  momentum drag, and explicit spillage reporting;
- shared-nozzle body/throat/area-ratio trades with packaging allowances, drag budget,
  propulsion derate, fuel depletion, and requirement checks;
- local normalized pulsejet and ramjet key-variable sensitivities;
- performance-only multi-fuel comparison with tank volume and explicit omitted-system
  flags;
- parameter-driven OpenVSP 3.51.2 flow-through body, two lifting surfaces, and four
  canted fins;
- an explicit-point VSPAERO panel/VLM runner for nonuniform Mach, angle-of-attack,
  and sideslip grids; and
- a longitudinal point-mass flight-equation kernel awaiting aerodynamic tables and a
  phase-based mission integrator.

OpenVSP/VSPAERO is used only for external aerodynamics. Internal combustion and nozzle
flow stay in the Python propulsion model.

## Run locally

```bash
python -m pip install -e .
python -m unittest discover -s tests -v

douglas-dart design-convergence \
  --config configs/shared_nozzle_candidate_a.yaml

douglas-dart shared-nozzle-trade \
  --config configs/shared_nozzle_candidate_a.yaml

douglas-dart key-variables \
  --config configs/shared_nozzle_candidate_a.yaml --engine both

douglas-dart fuel-trade \
  --config configs/shared_nozzle_candidate_a.yaml

douglas-dart altitude-trade \
  --config configs/shared_nozzle_candidate_a.yaml \
  --minimum-altitude 3000 --maximum-altitude 6500 --altitude-step 500

douglas-dart pulsejet \
  --config configs/shared_nozzle_candidate_a.yaml \
  --duration 0.50 --summary-start 0.25
```

CSV output is optional and generated files are ignored by Git:

```bash
douglas-dart pulsejet \
  --config configs/shared_nozzle_candidate_a.yaml \
  --duration 0.50 --summary-start 0.25 \
  --csv results/pulsejet_candidate_a.csv
```

## One-command pipeline

The repository-level runner executes every currently implemented propulsion, sizing,
fuel, and sensitivity analysis, generates CSV/JSON results and plots, builds the
OpenVSP model, and runs the configured 75-point VSPAERO sweep:

```bash
python run_all.py
```

The default result package is written to
`results/generated/shared_nozzle_candidate_a/` with this structure:

```text
inputs/       copied YAML inputs used for the run
csv/          tabular samples and trade sweeps
json/         summaries and complete machine-readable results
plots/        propulsion, sizing, fuel, sensitivity, and VSPAERO figures
logs/         tracebacks for any stage that could not complete
environment.json
manifest.json
summary.md
```

The generated geometry is written to
`openvsp/generated/shared_nozzle_candidate_a.vsp3`. The manifest remains available
even if OpenVSP or VSPAERO fails, so completed Python results are not lost and the
blocked external stage is explicit.

For a Python-only run on a machine without OpenVSP:

```bash
python run_all.py --skip-openvsp
```

To generate the `.vsp3` file but omit the live VSPAERO solver sweep:

```bash
python run_all.py --skip-vspaero
```

The default 300 mm upper diameter value is only a numerical sweep bound. It is not a
vehicle requirement and can be changed with `--body-diameter-max`.

The automated test environment validates the Python package, result serialization,
and OpenVSP API call contracts. It does not contain the official OpenVSP application,
so a live `.vsp3` write and VSPAERO solve must still be run on a machine with the
matching OpenVSP 3.51.2 Python bindings and solver installation.

## OpenVSP and VSPAERO

Use the Python API shipped with the OpenVSP version named in the YAML. The ordinary
PyPI namespace package named `openvsp` is not sufficient. The runtime guard rejects
an empty module or mismatched API version rather than producing a false-success file.

```bash
douglas-dart openvsp-build \
  --config configs/shared_nozzle_candidate_a.yaml \
  --output openvsp/generated/shared_nozzle_candidate_a.vsp3

douglas-dart vspaero-sweep \
  --config configs/shared_nozzle_candidate_a.yaml \
  --model openvsp/generated/shared_nozzle_candidate_a.vsp3 \
  --csv results/vspaero_candidate_a.csv
```

This repository tests the OpenVSP API call contract against the current official
interface. A real `.vsp3` build and solver result still require a functional OpenVSP
3.51.2 installation. Consult the [official OpenVSP API documentation](https://openvsp.org/api_docs/latest/)
and [OpenVSP source repository](https://github.com/OpenVSP/OpenVSP).

## Engineering status

The code is useful for architecture trades and for deciding which measurements or
higher-fidelity models matter. It is not a combustor detail design, a structural
analysis, a safety case, or proof of competition compliance. In particular, the
current ramjet result relies on severe intentional spillage, the pulsejet is a lumped
control volume, and the total-drag budget is not yet replaced by solver-backed and
cross-checked aerodynamic data.

Read [docs/model_architecture.md](docs/model_architecture.md),
[docs/assumptions.md](docs/assumptions.md), and
[docs/validation.md](docs/validation.md) before interpreting output. The current fuel
recommendation and its unresolved criteria are in
[docs/fuel_trade.md](docs/fuel_trade.md).
