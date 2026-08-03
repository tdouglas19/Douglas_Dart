# OpenVSP/VSPAERO workflow

The source implementation lives in:

- `src/douglas_dart/openvsp_geometry.py`
- `src/douglas_dart/vspaero.py`

Geometry and analysis values come from the same YAML used by propulsion and flight.
Generated `.vsp3`, mesh, history, polar, and CSV files belong under ignored output
directories and are not authoritative inputs.

## Runtime prerequisite

Use the Python API distributed with the OpenVSP version declared in the case file
(currently 3.51.2). Python and the compiled bindings must be compatible. A namespace
package that imports as `openvsp` but lacks `AddGeom`, analysis functions, constants,
and `GetVSPVersion` is rejected.

Official references:

- [OpenVSP API documentation](https://openvsp.org/api_docs/latest/)
- [OpenVSP source and examples](https://github.com/OpenVSP/OpenVSP)

## Build Candidate A

```bash
python -m pip install -e .

douglas-dart openvsp-build \
  --config configs/shared_nozzle_candidate_a.yaml \
  --output openvsp/generated/shared_nozzle_candidate_a.vsp3
```

The generator creates five circular body stations:

| Axial position | Diameter | Role |
|---:|---:|---|
| 0.00 m | 195.0 mm | Open circular intake lip |
| 0.18 m | 205.0 mm | Forebody-to-constant-body transition |
| 0.99 m | 205.0 mm | Constant-body control station |
| 1.80 m | 205.0 mm | Aft taper start |
| 2.30 m | 133.21 mm | Open shared-nozzle exit |

It also creates two lifting surfaces at 0°/180° and four fins at
45°/135°/225°/315°. Each is an independent unsymmetrized OpenVSP wing so sideslip
and asymmetric loads are not suppressed by a geometry symmetry shortcut.

## Mandatory visual QA

Before running VSPAERO, inspect the generated file in OpenVSP:

1. inlet and nozzle ends are open and the flow-through engine representation is active;
2. circular station order and diameters match the table;
3. the body has no unintended cap, inversion, or self-intersection;
4. lifting-surface and fin roots meet the body without large gaps or overlaps;
5. four fins form the intended 45-degree X arrangement;
6. airfoil thickness and trailing-edge geometry are suitable for panel meshing; and
7. tessellation resolves the 5 mm body step and aft taper without pathological panels.

## Run VSPAERO

```bash
douglas-dart vspaero-sweep \
  --config configs/shared_nozzle_candidate_a.yaml \
  --model openvsp/generated/shared_nozzle_candidate_a.vsp3 \
  --csv results/vspaero_candidate_a.csv
```

Candidate A uses the panel method because the body is thick geometry. Manual
reference values are 0.0896 m² area, 0.525 m span, 0.3033 m mean aerodynamic chord,
and 1.24 m CG station. Mach values are nonuniform, so the runner executes each
Mach/alpha/beta combination as an individual one-point sweep rather than asking
OpenVSP to linearly interpolate between endpoints.

VSPAERO output is labeled inviscid. Do not feed its drag coefficient directly into
the trajectory as total drag. Add and cross-check viscous, wave, base, and
inlet/spillage drag first.

## Current verification state

API topology is unit-tested with a recording fake. This repository's current
execution environment contains no functional OpenVSP API or VSPAERO executable, so a
live geometry build, visual inspection, mesh-convergence study, and polar generation
remain explicit gates. No solver result is claimed until those gates run in the
pinned environment.
