# Findings from the first real OpenVSP/VSPAERO run

This repository's OpenVSP/VSPAERO code was unit-tested against a recording fake from
the start (see `docs/model_architecture.md` and `openvsp/README.md`), but no session
had a working OpenVSP install until 2026-08-05, when the user supplied a local
extracted OpenVSP 3.51.2 Windows build
(`OpenVSP-3.51.2-win64/python/{openvsp,openvsp_config,degen_geom,utilities}`).
Running the real solver for the first time surfaced three real defects that the fake
harness could not catch, all now fixed in `src/douglas_dart/openvsp_geometry.py` and
`src/douglas_dart/vspaero.py`.

## Install notes for this specific release

- `pip install openvsp` from PyPI does not exist; OpenVSP ships only as a GitHub
  release archive with a bundled Python source distribution.
- That release's `python/openvsp/setup.py` declares `install_requires=['degen_geom',
  'utilities', 'openvsp_config']`, which are sibling local packages, not PyPI
  packages. Install all four local folders together:
  `pip install python/utilities python/degen_geom python/openvsp_config python/openvsp`.
- **The release's `MANIFEST.in` only lists `_vsp.so` / `_vsp_g.so` (Linux/Mac
  extension names)**, so `pip install` silently drops the Windows `_vsp.pyd` /
  `_vsp_g.pyd` compiled extensions from the wheel. `import openvsp` then fails with
  `ImportError: cannot import name '_vsp'`. Fix: manually copy `_vsp.pyd` and
  `_vsp_g.pyd` from the source `python/openvsp/openvsp/` folder into the installed
  `site-packages/openvsp/` folder after `pip install`.
- This installed 3.51.2 Windows Python build does not expose `vsp.PANEL` /
  `vsp.VORTEX_LATTICE`, and `VSPAEROSweep` has no `AnalysisMethod` input at all
  (`GetIntAnalysisInput` returns empty). Panel vs. vortex-lattice is selected
  entirely through `GeomSet`/`ThinGeomSet`. `vspaero.py` now treats
  `PANEL`/`VORTEX_LATTICE` as optional and no-ops the `AnalysisMethod` call when
  they are absent, keeping `GeomSet`/`ThinGeomSet` as the sole (and sufficient)
  selector.

## Geometry defect 1: a geometry name ending in `_Surface` crashes the mesher

`VSPAEROComputeGeometry` reliably crashed the whole Python process (no exception —
the process died, observed as an unexplained exit code from the OS) whenever any
`WING` geom's name ended in the literal substring `"_Surface"`. Reproduced
deterministically by bisection:

| Name | Result |
|---|---|
| `Lifting_Surface_1`, `Lifting_Surface_2`, `Lifting_Surface`, `Wing_Surface`, `X_Surface`, `AB_Surface`, `_Surface` | crash, 3/3 |
| `Surface`, `Surface_X`, `LiftingSurface1`, `Lifting_Wing_1`, `Fin_1`, `Douglas_Dart_Flowthrough_Body` | fine, 3/3 |

Only names ending in `_Surface` are affected; the word "Surface" elsewhere in a name
is harmless. This is almost certainly a bug in this OpenVSP build's own internal
string matching (plausibly a control-surface auto-detection routine scanning geom
names for a suffix pattern), not anything specific to this repository's geometry.
**Fix**: renamed the generated lifting-surface geoms from `Lifting_Surface_{n}` to
`Lifting_Panel_{n}` in `openvsp_geometry.py`. If you rename any other generated geom
in the future, do not let the name end in `_Surface`.

## Geometry defect 2: exact-tangency wing/fin roots produce a degenerate panel face

Independent of defect 1, mounting a wing or fin root exactly tangent to the body/
shell outer mold line (zero radial overlap — the prior behavior) produced
`PGFace Invalid in Triangulate_DBA` / `1 Faces have errors` during triangulation.
It did not crash, but it silently degraded the mesh at that face. Isolated to the
lifting-surface/shell junction specifically (fin/shell and body/shell junctions were
clean). A small commanded radial interpenetration eliminates it:
`_RADIAL_MOUNT_OVERLAP_M = 0.004` (4 mm) is now subtracted from the computed mount
radius in `_configure_radial_surface`. This is standard OpenVSP modeling practice
(wing-body junctions are conventionally modeled with a small overlap, not exact
tangency) and was confirmed to resolve the warning with the real solver.

## API-usage defect: `ResultsVec` accumulates across the whole session, and mixes result types

`GetStringResults(sweep_id, "ResultsVec", 0)` does not return only the result from
the just-completed `ExecAnalysis` call — it returns every result accumulated in the
live session so far, and a *single* `VSPAEROSweep` call itself appends more than one
entry: one `CLtot`/`CDtot`-bearing force/moment history plus one or more
non-aerodynamic entries (observed: rotor-style `CP`/`CQ`/`CT` entries and a
`CpSlice`-summary entry, present even though this model has no propellers/slices
configured). The prior code asserted exactly one entry per call, which only held for
the very first point; every subsequent point failed with `did not return exactly
one history result (got N)`. Taking the naive last entry (`result_ids[-1]`) is also
wrong, since the newest entry is often one of the trailing non-aerodynamic ones
(observed failure: `history result 'TXEFLLG' does not contain 'CLtot'`).
**Fix**: scan `result_ids` backward for the most recent entry whose
`GetAllDataNames()` actually contains `"CLtot"` (`_latest_force_moment_history_id`
in `vspaero.py`), which is robust regardless of how many trailing entries of other
types get appended.

## Performance note

A single Mach/alpha/beta point at the panel method with the current tessellation
took **~186 seconds** of solve time (5 wake iterations, 5,234 surface loops). The
configured 5×5×3 = 75-point grid in `configs/shared_nozzle_candidate_b.yaml` would
take on the order of **4 hours** at this fidelity. Validate with a small subset first
(as this session did — 1-2 points) before committing to the full grid, and consider:

- reducing `wake_iterations` for a faster/coarser first pass,
- switching `analysis.method` to `vortex_lattice` for the bulk of the grid and
  reserving panel-method points for a handful of validation checks, or
- running the full sweep unattended in the background (`douglas-dart vspaero-sweep`
  with the full grid), the same pattern used for `douglas-dart design-optimize`.

## Status

With all three fixes applied, `douglas-dart openvsp-build` and a reduced-grid
`douglas-dart vspaero-sweep` both ran cleanly against the real installed API,
producing real inviscid `CLtot`/`CDtot`/`CMytot` at Mach 0.20. The full configured
grid has not yet been run to completion in this session due to the per-point solve
cost above — that is the next concrete step, ideally run unattended.
