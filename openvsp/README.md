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

## Build Candidate B

```bash
python -m pip install -e .

douglas-dart openvsp-build \
  --config configs/shared_nozzle_candidate_b.yaml \
  --output openvsp/generated/shared_nozzle_candidate_b.vsp3
```

The generator creates five circular body stations:

| Axial position | Diameter | Role |
|---:|---:|---|
| 0.00 m | 195.0 mm | Open circular intake lip |
| 0.18 m | 205.0 mm | Forebody-to-constant-body transition |
| 0.99 m | 205.0 mm | Constant-body control station |
| 1.80 m | 205.0 mm | Aft taper start |
| 2.30 m | 133.21 mm | Open shared-nozzle exit |

It also creates:

- an annular external shell ("fin can") as a second, independent FUSELAGE surface,
  offset radially outward from the inner flow-through body by the configured 0.5-1
  inch (`openvsp.shell.radial_offset_m`) annulus and faired to the inner body
  diameter at both ends. This annulus is where the fuel tank, avionics, and other
  auxiliary systems are packaged; it is included in vehicle sizing and drag;
- a small non-flush ram-air inlet pod on the nose, since the primary body is a
  flow-through representation for external aerodynamics only and the real vehicle's
  pulsejet/ramjet selector is not a true flow-through inlet; and
- two lifting surfaces at 0°/180° and four fins at 45°/135°/225°/315°, mounted to the
  external shell's outer diameter wherever their axial station falls inside the
  shell span (otherwise the inner body). Each is an independent unsymmetrized OpenVSP
  wing so sideslip and asymmetric loads are not suppressed by a geometry symmetry
  shortcut. Clocking angles are normalized into OpenVSP's -180 to 180 degree
  `X_Rel_Rotation` range before being written (225° -> -135°, 315° -> -45°).

Every circular cross-section (inner body, external shell, and inlet pod) has its
top/bottom/left/right skinning angle and strength zeroed and its symmetric-skinning
flag set, so the mold line stays a plain surface of revolution rather than picking up
an unintended skew.

If you build or hand-adjust a `.vsp3` in the OpenVSP GUI and want it kept as
comparison data, put it (and any VSPAERO output you run locally) in
`openvsp/reference/` — see that directory's `README.md` for the naming convention.
That directory is tracked; `openvsp/generated/` is pipeline output and stays
git-ignored.

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
  --config configs/shared_nozzle_candidate_b.yaml \
  --model openvsp/generated/shared_nozzle_candidate_b.vsp3 \
  --csv results/vspaero_candidate_b.csv
```

Candidate B uses the panel method because the body is thick geometry. Manual
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
