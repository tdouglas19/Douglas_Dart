# `scripts/vsp_model` — OpenVSP/VSPAERO model for Douglas Dart V4

Standalone. Takes the V4 sizing export, builds a `.vsp3`, runs VSPAERO, and
writes everything into one output folder.

Read **[VEHICLE_ARCHITECTURE.md](VEHICLE_ARCHITECTURE.md)** first — it is the
shared understanding of what the vehicle is, what the sizing model decides, and
what we decide.

**[OPEN_QUESTIONS_FOR_V4_SIM.md](OPEN_QUESTIONS_FOR_V4_SIM.md)** is the paste-ready
list of dimensions this model currently invents, for the session working on
`medium_model/`. Every one has a working default, so nothing is blocked — but
each is a choice nobody has made.

## Run it

```bash
.venv/Scripts/python scripts/vsp_model/run_vsp_model.py
```

```bash
.venv/Scripts/python scripts/vsp_model/run_vsp_model.py --geometry-only
```

```bash
.venv/Scripts/python scripts/vsp_model/run_vsp_model.py --grids vlm_subsonic --wing-le 1.15
```

| flag | effect |
|---|---|
| `--sizing-json PATH` | input sizing export (default `out_medium_model/v4_export/structural_dimensions.json`) |
| `--output-dir PATH` | output folder (default `out_vsp_model/v4`) |
| `--geometry-only` | build the `.vsp3`, skip the solver |
| `--reanalyse` | redo `stability.md`/`.csv` from an existing `aero_points.csv`, no solving |
| `--grids a,b` | run only the named sweep grids |
| `--wing-le M` | wing root LE station, metres from the nose |
| `--fin-area-ratio R` | total fin area / wing reference area |
| `--ballast-x M` | station for the 4.68 kg ballast — the main CG lever |
| `--stability-altitude M` | altitude for the dynamic-mode estimates |

## Input, and where each number comes from

Two inputs, and the split between them is deliberate:

1. **`out_medium_model/v4_export/structural_dimensions.json`** — everything the
   trajectory/sizing chain actually computed. Loaded by `sizing_input.py`, which
   validates that the file's own numbers close (wing trapezoid vs stated
   reference area, boattail length vs stations, and so on) before anything is
   built on them.
2. **`geometry_inputs.py`** — everything the sizing chain does *not* produce:
   fins (which do not exist in that model at all), wing longitudinal station,
   nose and boattail profiles, airfoil ordinates, tessellation, sweep grids.
   Every field has a default and a docstring saying why.

If a number is not in (1), it is in (2), and (2) says so.

## Output folder

```
out_vsp_model/v4/
  douglas_dart_v4.vsp3      the model — open this in the OpenVSP GUI
  inputs.json               full provenance: sizing source + git SHA + every input
  geometry_stations.csv     body mold-line stations
  geometry_panels.csv       wing and fin planforms as built
  mass_properties.csv       per-item mass, station, and whether it was derived or chosen
  aero_points.csv           every VSPAERO point, all grids, one tidy table
  stability.md              static margin, Cn_beta, dynamic modes
  stability.csv             the same, machine-readable
  lift_curve.png            CL vs alpha per Mach, VLM with panel spot-checks
  pitching_moment.png       Cm vs alpha — the stability picture
  drag_polar.png            inviscid only, labelled as such
  mach_trends.png           CL_alpha and Cm_alpha vs Mach
  solver/<grid>/            raw VSPAERO output: .polar .history .stab .lod .vspgeom
```

Plots are a separate step so they can be redrawn without re-solving:

```bash
.venv/Scripts/python scripts/vsp_model/plots.py out_vsp_model/v4
```

`solver/` is kept per grid rather than left beside the `.vsp3`, because VSPAERO
names its files after the model and the next grid would overwrite them.

## Modules

| file | job |
|---|---|
| `openvsp_env.py` | find and import the OpenVSP bundle; error checking |
| `sizing_input.py` | load and validate `structural_dimensions.json` |
| `geometry_inputs.py` | every choice the sizing model does not make |
| `vehicle_geometry.py` | derive the full mold line — pure math, no OpenVSP |
| `mass_cg.py` | mass, CG, inertia from `mass_budget.csv` + assigned stations |
| `vsp_build.py` | translate the geometry into a `.vsp3` |
| `vsp_aero.py` | run VSPAERO grids, collect results and solver files |
| `stability.py` | static margin, neutral point, dynamic modes |
| `run_vsp_model.py` | CLI entry point |
| `sweep_stability.py` | search wing station x fin area for a stable airframe |
| `plots.py` | PNGs from `aero_points.csv` |

`vehicle_geometry.py` deliberately has no OpenVSP dependency, so the mold line
can be checked and diffed without a running solver:

```bash
PYTHONPATH=scripts/vsp_model .venv/Scripts/python -c "
from sizing_input import SizingInput; from geometry_inputs import GeometryInputs
from vehicle_geometry import build_vehicle_geometry, describe
print(describe(build_vehicle_geometry(SizingInput.from_json(), GeometryInputs())))"
```

## OpenVSP install

No `pip install` needed. `openvsp_env.py` imports straight off an extracted
OpenVSP 3.51.2 bundle (Python 3.11 bindings, matching this repo's venv), which
avoids two packaging defects in that release — its `setup.py` declares sibling
local folders as requirements, and its `MANIFEST.in` omits the Windows `_vsp.pyd`
so a pip-installed package cannot import.

Searched automatically; override with `DOUGLAS_DART_OPENVSP_ROOT` pointing at the
directory containing `vsp.exe`.

## Things this build of OpenVSP does that will waste your day

All confirmed against the installed 3.51.2 Windows build. Sections A, B and C,
and items D1-D3, were found while building this; D4-D7 were already known
(`docs/openvsp_real_api_findings.md`). Every item in A, B, C and D1-D2 **fails
silently while reporting success** — which is the reason this list exists.

### A. The fuselage must be in the THICK set, not the thin set

VSPAERO splits a configuration into *thick* geometry (surface panels) and *thin*
geometry (zero-thickness lifting sheets), selected by the `GeomSet` and
`ThinGeomSet` analysis inputs. Putting everything in one set is wrong, and
nothing tells you so.

With the whole model in the thin set, the fuselage is solved as a lifting sheet:

| configuration | CL | CS |
|---|---:|---:|
| bare body, α=4°, all-thin | **0.0000** | **−0.0567** |
| bare body, α=4°, thick/thin split | 0.0002 | −0.0016 |

The body's entire normal force came out on the wrong axis — a spurious side
force on a vehicle that is geometrically symmetric about XZ, and no lift at all.
`vsp_build` now writes two named user sets (`VSPAERO_Thick_Bodies`,
`VSPAERO_Thin_Surfaces`) into the `.vsp3`, and `vsp_aero` refuses to run a
vortex-lattice grid on a model that lacks them.

The split value of 0.0002 is also the *physically correct* answer: a closed body
in potential flow carries no net force (d'Alembert), only a Munk moment. The
body's real lift is viscous crossflow, which no panel method produces.

### B. Surface-root intersection can make the solve return NaN, non-monotonically

Root interpenetration is needed at all because exact tangency makes a degenerate
sliver face during triangulation. Too shallow also fails — but **the failure is
not a depth threshold**, and that took two rounds of measurement to establish.

First round, baseline fin (145 mm root, t/c 0.04):

| root overlap | result |
|---|---|
| 2.9 mm (half the root airfoil thickness) | **NaN from GMRES iteration 0** |
| 5.8 mm | solves |
| 6.1 mm (the wing) | always solved |

That looks like a threshold, and a 6 mm floor was set on it. Then a larger fin
(`fin_area_ratio` 0.80) failed at 6 mm, and sweeping the depth gave:

| fin area ratio | 6 mm | 10 mm | 14 mm | 20 mm |
|---|---|---|---|---|
| 0.80 | FAIL | **OK** | FAIL | FAIL |
| 2.30 | **OK** | FAIL | FAIL | — |

Non-monotonic, and the *larger* fin succeeds where the smaller one fails. That is
a mesh-intersection degeneracy — the root landing on a body tessellation feature —
not a depth threshold. There is no rule to derive, and no single value safe for
every planform.

**The remedy is to jitter, not to tune.** `sweep_stability.OVERLAP_ATTEMPTS_M`
retries a configuration at successive depths until it converges, and records
which one worked. `vehicle_geometry.MEASURED_MIN_OVERLAP_M` keeps 6 mm as the
default because it works for the baseline geometry, not because it is universally
safe.

Throughout, there is no error, no warning and a normal exit — GMRES prints
`Red: nan` and every coefficient comes back NaN. `vsp_aero._reject_nonfinite`
raises rather than let one reach a CSV. Fin count, fin clocking, fin axial
station and flow-through-vs-closed body were all bisected and are all red
herrings.

### B2. A solve can diverge *partially* and return large finite garbage

Worse than the NaN case, because it survives an `isfinite` check and lands in a
CSV looking like data. Observed on one sweep configuration:

```
cl_alpha = -23261161.83     cm_alpha = 77106345.71
```

VSPAERO exited normally. Those values propagated into the summary table as a
plausible-looking `Cn_beta = 103.7` sitting between neighbours of 0.66 and 0.81,
and were only caught because that outlier was 100x its neighbours.

`vsp_aero.COEFFICIENT_SANITY_BOUND` rejects any coefficient above 100 — two
orders of magnitude above anything physical on this vehicle, five below the
observed failure — and the sweep retries at a different root depth.

**Calibration note:** the surviving neighbours of that bad point were not
monotonic in fin area when they should have been (`Cn_beta` 0.656, 0.814, 0.553
at fin ratios 1.4, 1.6, 1.7). Expect roughly ±0.15 of scatter on `Cn_beta` even
among converged solves, and do not read these derivatives to two decimals.

### B3. A bad root depth can kill the Python process outright

The same geometry (wing LE 1.00 m, fin ratio 1.40, M 0.50) at three root depths:

| root depth | outcome |
|---|---|
| 6.0 mm | converges, CL_alpha 3.264, Cn_beta +0.656 |
| 7.5 mm | NaN — caught and reported |
| 9.0 mm | **Python process dies**, no exception, no traceback |

The third mode is the same hard crash `docs/openvsp_real_api_findings.md`
records for geom names ending in `_Surface`: the native mesher takes the
interpreter down with it. A `try/except` cannot catch it, so
`sweep_stability.evaluate_with_retry` **cannot** survive it -- the whole sweep
dies at that configuration.

**Mitigation, not yet implemented:** run each configuration in a subprocess, the
same pattern `scripts/fly_frozen_v2.py` already uses in this repo for a different
reason. Until then, an unattended sweep can terminate early, and the streaming
CSV write is what preserves the work done up to that point.

This also means the mesh-sensitivity question is **open**. The intent was to
solve one fixed geometry at five depths and attribute any spread to numerics,
but only one of the five converged, so there is no spread to measure and the
fin 1.50 outlier in the crossover sweep is unexplained.

### C. Roll and yaw moments come back with the opposite sign to what you expect

OpenVSP's body axes put **x aft**, so its `CMx` (roll) and `CMz` (yaw) are the
negatives of the standard aircraft-convention `Cl` and `Cn`. Pitch is unaffected.
Measured on this model in a single stability run:

| standard-convention | body-axis | values |
|---|---|---|
| `CMl_Beta` = −0.3254 | `CMx_Beta` = +0.3254 | flipped |
| `CMm_Alpha` = +6.7560 | `CMy_Alpha` = +6.7560 | identical |
| `CMn_Beta` = −3.6116 | `CMz_Beta` = +3.6116 | flipped |

The `.polar` and `.history` files report the **body-axis** `CMxtot`/`CMztot`, so
anything computed by differencing those needs negating before it is called
`Cl_beta` or `Cn_beta`. Get this wrong and a directionally unstable vehicle
reports as stable. `stability._AXIS_SIGN` applies the correction, and the alias
table deliberately has no `CMz_`/`CMx_` fallback — a missing value beats a
confidently inverted verdict.

### D. The rest

1. **`GetNumTotalErrors` / `PopLastError` do not exist at module level.** Only
   `ErrorMgrSingleton` does. Any error check written as
   `if hasattr(vsp, "GetNumTotalErrors")` silently never fires — which is exactly
   how a run full of `Can't Find Parm` errors reports success. This is why
   `openvsp_env.check_errors` refuses to run if it cannot find the error manager.
2. **Wing airfoil parameters are not geom-container parameters.** `ThickChord`,
   `Camber`, `CamberLoc` must be set through `GetXSecParm(xsec_id, name)`.
   Addressing them as `SetParmVal(geom, "Camber", "XSecCurve_0", v)` fails on
   every call — invisibly, because of (1).
3. **A cambered panel must not be rolled past 90°.** The roll maps +Z to −Z and
   inverts the section. Mirror instead. `vsp_build` now raises rather than build
   it.
4. **No geom name may end in `_Surface`** — it kills the native panel mesher and
   takes the Python process down with it, no exception raised.
5. **Surface roots need a commanded radial interpenetration.** Exact tangency
   produces a degenerate sliver face during triangulation.
6. **`ResultsVec` accumulates across the whole session** and one sweep appends
   several entries, including rotor- and CpSlice-style results with no rotors or
   slices configured. Neither "exactly one" nor "the last one" is safe — scan
   backwards for the entry that actually carries the field you want.
7. **No `AnalysisMethod` input, no `PANEL`/`VORTEX_LATTICE` constants.**
   `GeomSet` (thick) vs `ThinGeomSet` (thin) *is* the method selector.

## READ FIRST: the derivatives are mesh-dependent, and badly

The single most important result in this folder is a negative one.

One **fixed** geometry (wing LE 1.00 m, fin ratio 1.40, M 0.50), solved at seven
different surface-root mount depths. The mount depth is a meshing artifact — it
buries the root a little deeper in the body and has **no aerodynamic meaning
whatsoever**. The aerodynamics is identical in all seven cases:

| mount depth | CL_alpha | Cm_alpha | static margin | Cn_beta |
|---|---|---|---|---|
| 6.0 mm | 3.264 | −1.276 | **+0.391** | **+0.656** |
| 7.5 mm | NaN | | | |
| 9.0 mm | process death | | | |
| 10.5 mm | 1.941 | **+3.658** | **−1.885** | **−6.066** |
| 12.0 mm | NaN | | | |
| 13.5 mm | 3.015 | −0.218 | **+0.072** | **−0.080** |
| 15.0 mm | NaN | | | |

Three converged solves of the same vehicle span **2.3 reference chords** of
static margin and **6.7 /rad** of Cn_beta, and `Cm_alpha` **changes sign**. The
vehicle reads as comfortably stable at one mount depth and violently unstable at
another. Four of seven depths do not converge at all.

**This is not scatter to average over. It means a single VLM solve of this
configuration does not determine the answer.** Any quantitative conclusion drawn
from one mesh realisation — including the fin-sizing recommendation in
`out_vsp_model/v4_stability_sweep/summary.md` — is unsupported until this is
fixed.

### The BASELINE configuration, however, IS converged

The same test on the as-sized vehicle (wing LE 1.00 m, **fin ratio 0.35**):

| mount depth | CL_alpha | Cm_alpha | static margin | Cn_beta |
|---|---|---|---|---|
| 5.0 mm | 1.769 | +3.266 | −1.846 | −2.501 |
| 6.0 mm | 1.722 | +3.373 | −1.959 | −2.319 |
| 7.0, 8.0 mm | NaN | | | |
| 10.0 mm | 1.765 | +3.218 | −1.824 | −2.362 |
| 12.0 mm | 1.685 | +3.468 | −2.058 | −2.435 |

Four independent mesh realisations agree to **±4% on Cm_alpha, ±6% on static
margin and ±4% on Cn_beta**, and every one returns the same verdict. So:

* **Baseline result is solid**: static margin **−1.92 ± 0.12 cref**, `Cn_beta`
  **−2.40 ± 0.09 /rad**. Unstable in both axes, confirmed four ways, and
  independently corroborated by the 78-point grid holding −2.0 cref across seven
  Machs.
* **Large-fin results are not.** Convergence degrades as the fin grows — the
  ratio-1.40 fin is roughly twice the baseline's span, and its thin surface makes
  a much larger intersection with the thick body.

The practical reading: the *direction* of the fix (substantially more fin) is
established, the *magnitude* is not. The convergence study is needed before any
fin ratio can be quoted, but not before believing the vehicle has a problem.

### What has to happen before these numbers mean anything

A mesh-convergence study at the fin/body junction: raise
`GeometryInputs.surface_tessellation` and `fuselage_tessellation` until the
derivatives stop moving with mount depth. Only then is a sweep over fin area
measuring fin area rather than measuring the mesh. The junction is the suspect
because the body is thick geometry and the fins are thin, so their intersection
is where the two solver representations meet.

## What the numbers are, and are not

VSPAERO here is an **inviscid potential solution**. Its `CDtot` carries induced
and (panel mode) volume-wave drag. It does **not** carry skin friction, base
drag, or inlet spillage/additive drag — and this vehicle has an 80.9 cm² annular
base at the boattail. `medium_model/drag_buildup.py` supplies all three. Do not
substitute a VSPAERO CD for the build-up's total.

The Mach grid stops at 0.80 even though the mission reaches 1.10, because the
linear methods are singular through the transonic.

Before trusting a new configuration, **open the `.vsp3` in the OpenVSP GUI and
look at it.** The read-back check in `vsp_build.verify_model` proves the file
parses and its stations match the intent; it cannot tell you the fins are
inside-out.
