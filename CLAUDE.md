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
- **V4 profile** (2026-08-13, user): the ramjet must light **in the dive**, at
  a gate of **M 0.50** or higher. Adds to `run_flight`: real pitch arcs
  (pushover at 0 g, pull-out at 3 g nominal with the load factor rising to
  hold the 400 ft floor, capped at 6 g), **body loading** outputs
  (`load_n_roll` axial / `load_n_yaw` normal / `load_n_total`, accelerometer
  convention — gravity excluded), a `RamjetStart(gate_mach,
  require_descending)` policy, and a **spiral climb** (`spiral_climb=True`,
  30° bank, zero net downrange). Every knob defaults to pre-V4 behaviour, so
  V2/V3 re-flies stay bit-identical. Frozen: `docs/v4_frozen/design.json`,
  guarded by `tests/test_v4_frozen.py` via `scripts/fly_frozen_v4.py`.
  Result: top 1100 m, dive 14°, climb 6° spiral, gate M 0.50 → payload
  **4.68 kg**, peak T/W 6.07, peak body load **3.45 g**, tolerates a **−10 %**
  pulsejet thrust cut and still lights in the dive.
  `out_simple_model/v4_optimal_design.md`.
  - **M 0.533 is the hard reachability ceiling** (pulsejet-only dive exit at
    the FP altitude limit) — M 0.55+ cannot light in the dive at any top.
  - A **steeper dive makes dive-exit Mach WORSE** at a fixed top (drag-limited:
    a shorter path costs more than the extra gravity buys). Top of climb is
    the only lever that moves it.
  - V4 does **not** close the traverse (0.163 vs 0.25 g), powered-accel
    (−0.021 g at the top of climb) or thrust-margin (0.855 vs 1.15) gates. The
    last two are the same engine-limited gates V2/V3 never closed.
  - Biggest risk: top 1100 m sits just under the FP pulsejet flame-out ceiling
    (~1250 m, and that ceiling dropped ~200 m on one grid refinement).
- **V2 and V3 are both frozen** — `docs/v2_frozen/` and `docs/v3_frozen/`
  (design.json = re-flyable inputs + constants + git SHA), guarded by
  `tests/test_v2_frozen.py` / `tests/test_v3_frozen.py`, which re-fly them
  **in subprocesses** (`scripts/fly_frozen_v2.py`, `fly_frozen_v3.py`)
  because CD0 is baked at import and an in-process override silently does
  nothing once another test module has imported simple_model.
- The V3 report table is **standalone** — no comparisons against earlier
  model versions (user preference, 2026-08-13). Earlier baselines are
  frozen separately and stand on their own.
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
  Green = `Ran 278 tests ... OK (skipped=2)`, ~1040 s (verified 2026-08-14;
  was 152/~814 s on 2026-08-12 — the count grows, so treat it as a floor, not
  an equality). Never pipe the run through `tail` — the pipeline exit code is
  `tail`'s, not Python's, and it masks failures. Redirect to a file and
  `echo "EXIT=$?"` instead.
- Don't trust a suite that ran while you were still editing: `unittest
  discover` imports every test module up front, so a long run reflects a
  mixture of code states. If you changed anything mid-run, re-run it clean.

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

## 2D propulsion section (`scripts/propulsion_2d/`, 2026-08-14)

- Interactive longitudinal section of the integrated propulsion system, built
  from `out_medium_model/v4_export/structural_dimensions.json`. **Zero imports
  of `medium_model`** (guarded by a test) — it cannot drift with the flight
  code. Run: `.venv/Scripts/python -m scripts.propulsion_2d [--show|--export|
  --save-vector]`; bare = print tables only.
- Geometry is **analytic typed segments**, not a sampled point cloud: every
  break is at a non-integer mm and the duct wall is 1.0 mm, so a 1 mm grid
  can resolve neither. `contour.csv` (1 mm uniform) is an *export* derived
  from the model. Viewer = live matplotlib with cursor readout, click-to-pin
  datatips that snap to named stations, and layer toggles.
- **Geometry contract: `scripts/PROPULSION_2D_GEOMETRY.md`** — every station
  with its provenance (JSON / DERIVED / ASSUMED), every invented number, and
  the model-vs-as-drawn reconciliation. Assumptions all live in
  `scripts/propulsion_2d/assumptions.json`; ASSUMED geometry draws dashed grey.
- Architecture (user directive): **nose inlet with an external-compression
  centrebody**. At M 1.10 the spike buys ~0.1% recovery — its justification is
  packaging, not compression. **The centrebody is the design variable** (sized
  for avionics) and the lip is *solved* from the capture rule.
- **Round trip with the V4/medium_model workspace completed 2026-08-14** —
  `docs/v4_export_requests_from_propulsion_2d.md` (asks) →
  `docs/v4_export_answers_from_flight_model.md` (answers) → corrected export
  → this tool rebuilt. **Two of my findings were wrong and are corrected:**
  - **The boattail is 8.0°, and V4's drag was never understated.**
    `drag_buildup.base_diameter_m()` *derives* the base at 8° and never reads
    the exported field; the field was the bug. Aft body closes to **153.849 mm
    at 8.0000°** and does NOT close onto the nozzle — an annular base remains
    (80.87 cm² engine-on, 185.90 engine-off, and ~140 s of the mission are
    unpowered). My "unbuildable base" finding dissolved: 18.1 mm of clearance.
    No re-fly needed. Lesson: the containment test was sound, the *inference*
    about which side was authoritative was not.
  - **The old capture rule oversized the inlet 2.67×.** The FP ramjet actually
    consumes **39.4 cm²** at cutoff (from its own φ and fuel flow), not the
    105.0 cm² throat area. Rule is now
    `annulus_equals_required_mass_flow`, reading `inlet.implied_capture_area_m2`.
    The freed area went into the centrebody: 64.2 → **111.7 mm**, 2.24 L for
    avionics, cowl unchanged. (Export calls its area a FLOOR — no spillage, no
    strut blockage.)
- Findings that stand:
  - **Fixing the inlet broke the diffuser**: capture 39.4 → chamber 343.1 cm²
    is an area ratio of **8.72** over 274.5 mm = **14.13°**, hopeless.
    Diffusing all the way at 7° would need +288 mm of nose. So the drawing
    now does what a ramjet dump combustor does: **7° to a 138.2 mm dump plane,
    then a 2.29× sudden expansion**. The area distribution is genuinely
    discontinuous there and the tests assert the step rather than smooth it.
  - Giving the chamber→tailpipe cone a real length (15° → 174.2 mm) costs
    **11.4% of fuel annulus volume**: capacity 7.798 → 6.911 kg. Still 3.25×
    the 2.125 kg actually burned, so a number to know, not to act on.
- **Divergent nozzle** (user directive 2026-08-14): the export has none — its
  `nozzle_exit_diameter_m` equals the tailpipe dia, so what it calls the exit
  is the THROAT. Expansion is added aft of the throat only; throat area,
  `throat_to_body_area_fraction` and the capture rule are untouched. Default
  area ratio **1.10**. Sizing result: the nozzle only chokes above **M 1.061**,
  the ramjet lights at M 0.488 and cutoff is M 1.100, so it is unchoked for
  nearly the whole burn and the optimum area ratio at M 1.10 is **1.0015** —
  thermodynamically pointless. Its only real effect is eating the annular base
  (80.87 → 66.53 cm², −17.7%), which reduces base drag. **The flight model has
  no expansion and V4 was not flown with one** — pricing it needs `fp_spec` +
  a re-fly.
- **Still open** (only the first is cheap and worth chasing):
  1. **Combustor total pressure vs Mach is not exported** — the one number
     that turns the nozzle expansion from assumption into calculation. The
     flight-model session has offered to add it to the `FpPropulsion` trace
     and re-fly (~35 min). Worth taking.
  2. **Strut count / blockage** — the capture area is explicitly a FLOOR
     without it.
  3. Chamber 214 mm is both OML and flow diameter in the model (a real
     simplification, not a slip); honest bore 209 mm, −4.6% volume. The flight
     session deliberately did NOT change the key, since 214 is what was flown.
- **Also from the flight session, not on our list**: the airframe is
  statically unstable in pitch and yaw and needs fins; the fin-area ceiling is
  **0.12–0.16 m²**, and 0.120 m² closes on only 1.8% fuel margin. The
  un-costed **side valve runner fairings** draw on that same thin budget.
  Also: closed-form and FP ramjets disagree fundamentally near lightoff (~0 N
  vs 227 N at M 0.49) — never screen against `ramjet_simple` there.
- ~~Open conflict~~ **RESOLVED**: `drag_buildup.py:156` sets
  `BOATTAIL_HALF_ANGLE_DEG = 8.0` and `base_diameter_m()` *derives* the base
  from it — it never reads the exported field, so there was never a conflict
  inside the flight code. The export field was wrong; it is now corrected.
- Handoff docs: **`docs/v4_export_requests_from_propulsion_2d.md`** (asks) and
  **`docs/v4_export_answers_from_flight_model.md`** (replies, plus what the
  corrected export now carries: new `inlet`, `design_point` and
  `payload_and_avionics` blocks).
- Tests: `tests/test_propulsion_2d.py` (56, ~11 s, no FP dependency). Guards
  contour continuity (with the dump step asserted, not smoothed over), the
  duct-inside-OML containment, the capture rule against the export's own
  mass-flow figure, provenance tagging, and the viewer's interactive paths
  headless.

## Translating-inlet shut-off (config B, 2026-08-14)

- **`scripts/propulsion_2d_full_drawing.py` is the current design drawing.**
  It imports config A's kernel but builds its own contours; a test asserts it
  cannot mutate A, and A's five files stay byte-identical to freeze `8df8b12`.
- Purpose (user directive): shut the ramjet intake while the pulsejet runs.
  **Why it works**: for fixed capture area `h ~ A/2πR`, so moving the annulus
  outboard thins it, and `stroke = h/tan(θ_seal)` — both levers multiply.
  Closing stroke **28.22 → 7.88 mm**.
- Architecture, with the **OML fixed** (user directive): fixed nose fairing on
  a spar carries the avionics (no harness across a moving joint); annular slot
  at its shoulder is the capture plane; a sleeve rides the spar and telescopes
  forward, its 45° cone seating on the cowl lip. Nothing outside the cowl moves.
- Final: slot 300 mm at r 83.46, gap/stroke 7.88 mm, **95.4 cm² faying**
  (12 mm land), nose volume 1.79 L, diffuser 128 mm, **dump 4.18×**. Plus a
  160×10 mm pitot-static boom, an annular V-gutter flameholder (x 537 mm,
  29% blockage) and 2×57 mm reed-valve side runners (51 cm², 15% of chamber,
  drawn OUTBOARD — they do not fit in a 214 mm skin).
- **Two faults the automated checks caught, both now guarded**: (1) the duct
  left the OML by ~2.5 mm over x 300–315 because the sleeve rises 7.88 mm
  where the cowl only grew 1.3 — fixed with a fast lip + hard clamp; (2) the
  faying land **necked the duct to 15.8 cm²**, 40% of capture, because when
  retracted the land sits in the open flow path — fixed by pulling the lip
  0.86 → 0.78 of body radius. Both checks print every run.
- Trade still open: 124 cm² faying at `--lip-frac 0.74 --faying-mm 16`, costing
  8.35 mm stroke and more nose volume.
- **Final report: `docs/propulsion_2d_final_report.md`** — includes a
  which-numbers-you-may-lean-on table. Short version: the model has **no inlet,
  no diffuser and no duct forward of the chamber at all**, so every area
  forward of x=428 is invented to satisfy one flown number (capture 39.4 cm²,
  itself an *implied* floor back-calculated from FP mass flow).
- Tests: `tests/test_translating_inlet.py` (24, ~10 s).

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
