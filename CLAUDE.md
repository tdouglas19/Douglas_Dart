# Douglas Dart — project notes for Claude

50 lb (22.68 kg) pulsejet→ramjet supersonic dart. Design studies run through
the calibrated closed-form `simple_model/` (fast), with first-principles
truth models in `pulsejet-fp/` (vendored subtree, ~1 min/point).

## Output format rules (user preference — firm)

- **NO HTML artifacts / published pages.** They take too long.
- Deliver results as: console tables, `.md` files with tables, PNG plots.
  Plots referenced from a `.md` are fine. Speed beats polish.
- Answer one-off "what does the model say about X" questions with a short
  script + printed numbers, not a report pipeline.

## Standing constraints

- Closed-form only in `simple_model/` — no iteration/ODE fanciness. "The
  goal is to run this fast." Higher-fidelity comes later via pulsejet-fp.
- Vehicle wet-mass cap 50 lb; motor cutoff Mach 1.1; 25% fuel reserve;
  min powered thrust margin ≥ 1.15; stall-speed cap 45 m/s.
- Release velocity is 40 m/s (rail/catapult launch assumption in
  `simple_model/flight_sim.py`) — the vehicle leaves the rail below its
  ~47 m/s full-mass stall speed; there is no ground-roll model.
- CD0 is **frontal-area referenced** (subsonic baseline; transonic
  multiplier applied in-model). Override per-run via env
  `SIMPLE_MODEL_CD0_FRONTAL` (read once at import → subprocess per level).
- **V3 climb-dive profile** (2026-08-13, `run_flight(climb_dive=...)`):
  climb steeply on surplus low-Mach thrust → dive through the ramjet
  lightoff notch (gravity supplies the acceleration the engine can't) →
  pull out at a **400 ft hard floor** → level "drag strip" to cutoff.
  Top-of-climb is DERIVED from the dive, not searched
  (`derive_top_altitude`). Rule enforced + tracked: flight path angle ≥ 0
  from M 0.80 through cutoff. Enable in a campaign with
  `SIMPLE_MODEL_V3=1`; V2 designs are untouched (zero dive angle → no
  profile). **Split gates**: the acceleration gate counts gravity and is
  measured on the *traverse* (`min_traverse_accel_g`, excludes the
  commanded climb — a climb is not a regime you can be stranded in); the
  thrust-margin gate is **engine-only** (gravity clamped at zero) so a dive
  can never make a weak engine look healthy.
- V3 result vs frozen V2: peak T/W **10.08 → 6.61**, body 280 → 225 mm,
  dry mass 14.80 → 12.57 kg, payload margin 5.85 → **7.42 kg**, same
  0.26 g traverse acceleration. `out_simple_model/v3_optimal_design.md`.
- **V2 is frozen** in `docs/v2_frozen/` (design.json = re-flyable inputs +
  constants + git SHA) and guarded by `tests/test_v2_frozen.py`, which
  re-flies it **in a subprocess** (`scripts/fly_frozen_v2.py`) because CD0
  is baked at import and an in-process override silently does nothing once
  another test module has imported simple_model.
- Reports/demo fly the **return-to-launch profile**
  (`run_flight(return_to_launch=True)`, 2026-08-12): pitch-up half-loop at
  cutoff (a flat 5g turn measurably wastes ~80% of the energy), glide home
  at the airframe's equilibrium slope, spiral + flare over the launch
  point (constants `RETURN_*` in `simple_model/flight_sim.py`). The
  optimizer gates still fly the legacy straight-out profile.
- Side-mounted valve inlets (boundary-layer air at ~static pressure) —
  that's why pulsejet thrust lapses with Mach instead of ramming up.

## Key model inputs / current best design (v2 campaign, 2026-08-12)

Winner at CD0=0.10 (feasibility boundary 0.125–0.15): body 234 mm,
throat 125 mm (area frac 0.285), chamber/tube 362/811 mm, overall 1874 mm,
propane, 1° climb; wing 615 mm span, AR 2.69, taper 0.62, sweep 6°,
thin-cambered; dry 13.22 kg + fuel 2.28 kg → 7.17 kg payload margin;
peak T/W 6.89, margin 1.21 @ M1.10. Full table + plots:
`out_simple_model/v2_optimal_design.md`.

## How to run things

- Venv: `.venv/Scripts/python`
- Single flight / ad-hoc query: script against `simple_model.flight_sim`
  (~0.7 s including imports).
- Full campaign: `scripts/simple_model_overnight2.py` (~80 min) then
  `scripts/simple_model_report2.py [cd0]` (~1 min; writes `.md` with
  embedded PNGs to `out_simple_model/`).
- V3 campaign: `scripts/simple_model_v3_campaign.py [cd0]` (~16 min) then
  `scripts/simple_model_report_v3.py` (~1 min; writes
  `v3_optimal_design.md` + `*_v3.png`, with a V2 head-to-head table).
- Method/repeatability doc: `docs/simple_model_campaign_playbook.md`;
  calibration provenance: `docs/simple_model_overview.md`.
- Tests: the kill-switches MUST be set on the command line —
  `tests/conftest.py` is a *pytest* convention and `unittest discover`
  never loads it, so a bare `unittest discover` runs the FP primaries live
  and reports 13 bogus failures (verified 2026-08-12):
  ```
  DOUGLAS_DART_DISABLE_PULSEJET_FP=1 DOUGLAS_DART_DISABLE_RAMJET_FP=1 \
    .venv/Scripts/python -m unittest discover tests
  ```
  Green = `Ran 152 tests ... OK (skipped=1)`, ~814 s (~2600 s without the
  switches). Never pipe the run through `tail` — the pipeline exit code is
  `tail`'s, not Python's, and it masks failures.

## medium_model handoff (firm — user preference)

- **Every change to `simple_model/` must be written up in
  `docs/simple_model_changes_for_medium_model.md`** so the user can paste it
  to the parallel session working on `medium_model/`. Say for each change
  whether it alters a **V2 re-fly** (what the parity test can see) and why
  it was made, not just what changed.
- `medium_model/` is a standalone COPY — zero Python imports of
  `simple_model` (verified). Two shared surfaces though: the `SIMPLE_MODEL_*`
  env namespace (medium_model falls back to it), and CD0 being baked at
  import, which `tests/test_medium_model_v2_parity.py` leaks into the test
  process. CD0-sensitive simple_model tests therefore run in pinned
  subprocesses (`scripts/fly_frozen_v2.py`, `scripts/v3_probe_flights.py`).

## Repo layout notes

- `pulsejet-fp/` is a git-subtree vendor of the first-principles pulsejet;
  the sibling checkout `../pulsejet-fp` is a frozen archive. Read its
  `architecture.md` before touching it.
- `../ramjet-fp` (sibling repo, editable install): first-principles
  unsteady ramjet, RAMJET_MODE's guarded primary since 2026-08-12 via
  `src/douglas_dart/ramjet_fp_bridge.py` (kill-switch
  `DOUGLAS_DART_DISABLE_RAMJET_FP=1`); Gate 3 missions read its lazy
  (M x alt) table. Read its `architecture.md` + `docs/findings.md` first.
  **phi is an OUTPUT, not an input** (user directive, same day): the FP
  path ignores `ramjet.target_equivalence_ratio` and self-selects the
  leanest-stable-plus-margin mixture per (M, alt)
  (`ramjet_required_equivalence_ratio` on the map point; engine-out =
  fuel cut + cold drag). Design curves:
  `ramjet-fp/scripts/throttle_schedule.py`. Cold relight still has an
  altitude-dependent no-go hole (cross transonic low, climb lit).
- Current branch: `ramjet-fp-development`.
