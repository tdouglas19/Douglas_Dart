# User-supplied OpenVSP/VSPAERO reference artifacts

This directory is for artifacts *you* generate locally in the OpenVSP GUI or a live
VSPAERO run and want kept in the repository as comparison data. It is the opposite of
`openvsp/generated/`, which is pipeline output and is git-ignored.

Everything placed here is tracked (see the `openvsp/reference/` exception in
`.gitignore`) even though matching extensions (`.vsp3`, `.polar`, `.history`,
`.vspaero`, `.vspgeom`, `.lod`, `.adb`) are ignored elsewhere in the repository.

## Naming convention

Prefix every file with the candidate name from `configs/*.yaml` (`case.name`, e.g.
`shared_nozzle_candidate_b`) and an ISO date, so the model-generated and
manually-adjusted versions never collide:

```text
openvsp/reference/<candidate_name>_<yyyy-mm-dd>[_<note>].<ext>
```

Examples:

```text
openvsp/reference/shared_nozzle_candidate_b_2026-08-05.vsp3
openvsp/reference/shared_nozzle_candidate_b_2026-08-05_gui_fillet_fix.vsp3
openvsp/reference/shared_nozzle_candidate_b_2026-08-05_mach1p10_a0_b0.polar
openvsp/reference/shared_nozzle_candidate_b_2026-08-05_mach1p10_a0_b0.history
```

## What to upload here

- The `.vsp3` file you build/adjust in the OpenVSP GUI, especially after the
  mandatory visual QA pass described in `openvsp/README.md` (open inlet/nozzle,
  correct station diameters, no self-intersection, fin/lifting-surface root fit).
- Any VSPAERO solver output you run locally (`.polar`, `.history`, `.lod`, `.adb`,
  stability derivative CSVs) that you want compared against the repository's
  explicit-point runner in `src/douglas_dart/vspaero.py`.
- Screenshots or notes about geometry problems found in the GUI QA pass (any file
  type is fine; keep the same naming convention).

## What happens after you upload

The files here are read-only reference data, not authoritative inputs. Nothing in
`src/douglas_dart/` reads from this directory automatically. When you add a file, say
so explicitly so the analysis can cross-check the repository's own drag/lift/moment
output against it, or so a geometry difference (fillets, fairings, panel density)
found in your GUI edit can be folded back into the parametric generator in
`src/douglas_dart/openvsp_geometry.py`.
