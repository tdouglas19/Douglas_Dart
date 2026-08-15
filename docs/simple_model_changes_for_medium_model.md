# simple_model → medium_model handoff

**Paste this to the medium_model session.** It lists every change made to
`simple_model/` that a faithful copy has to mirror, what each one does to a
**V2 re-fly** (the parity contract), and the two shared surfaces where the
two packages can interfere with each other.

- Covers: through `simple_model` git `c035579` ("Freeze V2; add the V3
  climb-dive trajectory"), 2026-08-13.
- **Sync status at the time of writing: IN SYNC.** `medium_model/flight_sim.py`
  was verified to already contain the equilibrium spiral
  (`asin`, `SPIRAL_BLEED_SPEED_FACTOR`, `drag_level_n`), the engine-only
  margin (`max(weight_along_path_n, 0.0)`), and the V3 block, and
  `tests/test_medium_model_v2_parity.py` passes **state-for-state** against
  `simple_model` in the full suite (175 tests OK). Nothing below is
  outstanding — it is here so the *reasons* travel with the code, and as the
  checklist for the next round.

---

## 1. Is simple_model safe from medium_model?

**Yes, structurally.** Verified, not assumed:

- `medium_model/*.py` contains **zero** Python imports of `simple_model` —
  every mention is a comment or docstring. It is a standalone copy, not a
  subclass or a wrapper.
- Nothing in `medium_model` writes to `simple_model` files.
- `simple_model`'s own behaviour is pinned by `tests/test_simple_model.py`
  and the frozen-design contract in `tests/test_v2_frozen.py`, both of which
  run their flights in **pinned subprocesses** and so cannot be perturbed by
  anything `medium_model` does in-process.
- Everything is in git; `docs/v2_frozen/` is committed.

**Two shared surfaces are real, though**, and both deserve care:

### Hazard 1 — the `SIMPLE_MODEL_*` environment namespace is shared

`medium_model/constants.py` deliberately falls back to `simple_model`'s
variables:

```python
CD0_FRONTAL = float(_os.environ.get("MEDIUM_MODEL_CD0_FRONTAL",
                    _os.environ.get("SIMPLE_MODEL_CD0_FRONTAL", CD0_FRONTAL)))
MIN_POWERED_ACCELERATION_G = float(_os.environ.get("SIMPLE_MODEL_MIN_ACCEL_G", "0.25"))
```

So a campaign that exports `SIMPLE_MODEL_CD0_FRONTAL` reconfigures **both**
packages in that process. That is a convenience, not a bug — but it means
"set the env var for medium_model" is never a medium_model-only action.
Prefer `MEDIUM_MODEL_*` when you mean only medium_model.

### Hazard 2 — CD0 is baked at IMPORT time, and test modules leak it

This one has already caused a real failure. In both packages:

```python
CD0_FRONTAL = float(_os.environ.get("SIMPLE_MODEL_CD0_FRONTAL", CD0_FRONTAL))
```

…runs once at import, and `drag.py` / `flight_sim.py` then bind the value
by `from .constants import CD0_FRONTAL`. Consequences:

- Setting the env var *after* any import silently does nothing.
- `tests/test_medium_model_v2_parity.py` sets
  `os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = 0.1` **at module import**, and
  sorts alphabetically **before** `test_simple_model.py` under
  `unittest discover`. It therefore changes the CD0 that simple_model's
  in-process tests run at. This silently changed what two V3 tests measured
  (they passed standalone at the default CD0 = 0.30 and failed in the suite
  at 0.10).
- **Fix already applied on the simple_model side:** every CD0-sensitive test
  now runs in a subprocess with CD0 pinned —
  `scripts/fly_frozen_v2.py` and `scripts/v3_probe_flights.py`.
- **Recommended on the medium_model side:** do the same for any new test
  whose result depends on CD0, rather than relying on the ambient value.

---

## 2. Changes to mirror

Each is marked with whether it changes a **V2 re-fly** — i.e. whether the
parity test can see it.

### 2.1 Engine-only thrust margin — *no V2 change*

```python
# was: demand_n = drag_result.total_n + weight_along_path_n
demand_n = drag_result.total_n + max(weight_along_path_n, 0.0)
```

**Why:** in a dive `weight_along_path_n` is negative, which shrinks "demand"
and inflates the margin — an underpowered engine would read as healthy purely
because it is pointed downhill. A climb's gravity term is real work the
engine must do, so it still counts.

**V2 impact:** none. V2 climbs at +1°, so `max()` is the identity. Verified
by `test_v2_frozen.py` reproducing the frozen margin of 1.36 exactly.

### 2.2 Traverse acceleration — *no V2 change (new measurement only)*

New `FlightResult` fields `min_traverse_accel_g` / `min_traverse_accel_mach`:
the minimum powered acceleration **excluding steps whose mode is
`v3_climb`**.

**Why:** the acceleration gate exists so the vehicle is never stranded in a
regime it cannot accelerate through. A *commanded* climb is not such a
regime — level-equivalent acceleration there is 0.6–0.8 g and the vehicle can
shallow out at any instant. The lightoff notch and the transonic drag strip
**are** such regimes. Once the V3 dive covers the notch, the worst
acceleration moves into the climb, so gating on the all-powered figure would
punish exactly the manoeuvre that helps.

The gates now read:

```python
result.min_traverse_accel_g >= MIN_POWERED_ACCELERATION_G   # the real gate
result.min_powered_accel_g > 0.0                            # climb must still accelerate
not result.rule_violated                                    # gamma >= 0, M 0.80 -> cutoff
```

**V2 impact:** none — with no `v3_climb` steps the two figures are identical
(asserted by `test_traverse_and_powered_accel_coincide_without_v3`).

### 2.3 Equilibrium overhead spiral — **CHANGES V2**

The overhead spiral used to fly a fixed `RETURN_GLIDE_ANGLE_DEG` (−9°). It
now solves the steady-glide angle closed-form each step:

```python
# steady unpowered flight: 0 = -D - W sin(gamma)  ->  sin(gamma) = -D/W
drag_level_n = <drag evaluated at gamma = 0>
sin_gamma = -min(drag_level_n / (m * G0_M_PER_S2), 0.5)
gamma_rad = asin(sin_gamma)
cos_gamma = cos(gamma_rad)
```

**Why:** a fixed angle cannot hold speed. At −9° the vehicle bled
monotonically all the way down — V2 reached the flare window with only
3 m/s of margin (25.4 m/s against a 22.4 m/s stall floor), and V3's higher
arrival altitude stalled it outright. Drag is evaluated level to pick the
angle; `cos(gamma) ≈ 1` at these slopes, so the one-pass estimate is exact
to well under a percent.

**V2 impact: yes** — the post-cutoff descent states differ. V2's landing is
strictly *more* robust afterwards. Headline numbers (peak T/W, margins,
accelerations) are all powered-phase quantities and are unchanged.

### 2.4 Spiral → flare handoff — **CHANGES V2**

New constant, and a removal:

```python
SPIRAL_BLEED_SPEED_FACTOR = 1.15   # overhead level-bleed target
# REMOVED: flare_speed_margin = max(flare_speed_margin, DECEL_SPEED_FACTOR)
```

and the level-bleed threshold became mode-dependent:

```python
elif v > (SPIRAL_BLEED_SPEED_FACTOR if overhead
          else DECEL_SPEED_FACTOR) * stall_speed and h > flare_altitude_m:
```

**Why:** the speed-holding spiral holds whatever speed it inherits. The level
bleed used to stop at `DECEL_SPEED_FACTOR` (1.25 × stall) while the flare
needs ≤ `FLARE_SPEED_MARGIN` (1.20 × stall), so the spiral held the vehicle
in that 0.05 gap and flew a perfectly good glide into the ground — observed
arriving at h = 0 holding 55.9 m/s against a 55.6 m/s trigger, 0.3 m/s short.
The old `flare_speed_margin` override was papering over the *fixed-angle*
spiral; with a correct equilibrium spiral it becomes a knife-edge and had to
go.

**V2 impact: yes**, same region as 2.3.

### 2.5 V3 climb-dive profile — *no V2 change (inert unless used)*

New in `flight_sim.py`: `ClimbDiveProfile`, `derive_top_altitude()`, the
`climb_dive=` parameter on `run_flight`, powered modes `v3_climb` /
`v3_dive` / `drag_strip`, rule tracking, and constants
`V3_FLOOR_ALTITUDE_M` (400 ft, user-set hard floor), `V3_DIVE_START_MACH`
0.35, `V3_DIVE_END_MACH` 0.60, `V3_RULE_MACH_LO` 0.80, `V3_RULE_MACH_HI`
1.10, `V3_MAX_TOP_ALTITUDE_M` 4000 m.

Design notes that matter if you re-derive any of it:

- **Top-of-climb is DERIVED, not searched.** `derive_top_altitude` marches
  the dive across its Mach band and integrates the altitude it spends;
  `top = floor + drop`. Two fixed passes (not a convergence loop) — the
  first estimates the drop at the floor's density, the second re-runs it at
  the resulting mid-dive altitude.
- **The cap is load-bearing.** A shallow dive behind a weak engine traverses
  so slowly that `drop = distance × sin(dive)` runs away; it fed
  `standard_atmosphere` an out-of-range altitude and crashed the optimizer.
  `V3_MAX_TOP_ALTITUDE_M` bounds it; such a candidate then simply never
  reaches its top and is reported infeasible.
- **The stall guard is suspended during the loop** (`mode != "loop"`), since
  a loop is deliberately ballistic over the top.

**V2 impact:** none — `climb_dive=None` is the default and no V3 branch runs.

### 2.6 `FlightResult` field order — mirror exactly

Positional construction is used at the return site, so order matters:

```
states, motor_cutoff_reached, landed, safe_landing, hit_mass_floor, stalled,
crossover_mach, min_powered_thrust_margin, min_margin_mach,
min_powered_accel_g, min_accel_mach,
min_traverse_accel_g, min_traverse_accel_mach,
climb_dive_top_altitude_m, rule_violated, rule_violation_mach
```

### 2.7 `constants.py` nose/tail split — *no V2 change*

```python
NOSE_LENGTH_DIAMETERS = 2.0
TAIL_LENGTH_DIAMETERS = 1.0
NOSE_TAIL_LENGTH_DIAMETERS = NOSE_LENGTH_DIAMETERS + TAIL_LENGTH_DIAMETERS  # 3.0
```

Reporting only — the sum is unchanged, and only the sum feeds the mass and
length models.

---

## 2.8 A second frozen contract now exists: V3

`docs/v3_frozen/` freezes the V3 climb-dive design the same way
`docs/v2_frozen/` freezes V2 — `design.json` carries the re-flyable inputs
(vehicle candidate **including** `initial_climb_angle_deg`,
`dive_angle_deg`, `floor_altitude_m`), the constants, and the verified
mission numbers. Guarded by `tests/test_v3_frozen.py` via
`scripts/fly_frozen_v3.py` (subprocess, pinned CD0).

Nothing for medium_model to mirror yet — V2 remains the parity contract,
since V3 is a trajectory feature rather than a fidelity step. But when
medium_model is eventually held to V3 as well, `design.json` is the file to
point at, and `Candidate(**vehicle_candidate).to_climb_dive()` is how the
profile is rebuilt.

Note also: the V3 report table is deliberately **standalone** — it carries
no comparison against V2 (user preference). Keep any medium_model report in
the same style.

## 3. The parity contract

`tests/test_medium_model_v2_parity.py` holds medium_model to
`docs/v2_frozen/design.json`, the same contract `tests/test_v2_frozen.py`
holds simple_model to. Two levels:

1. **Headline numbers** — peak T/W 10.08, min powered acceleration 0.26 g at
   M 0.45, engine-only margin 1.36, cutoff reached, safe landing, returns to
   within 50 m of launch.
2. **State-for-state identity** with simple_model on that flight — the strong
   form, which catches drift the headline numbers would miss.

Level 2 is expected to be retired deliberately when the first real fidelity
step (the drag build-up) lands; at that point it should move behind the
"legacy drag" switch and the delta it measures becomes the first row of the
attribution table. **Until then, a level-2 failure means an accidental
divergence, not progress** — that is the whole point of starting from a
proven-identical copy.

Verify both models at once:

```bash
DOUGLAS_DART_DISABLE_PULSEJET_FP=1 DOUGLAS_DART_DISABLE_RAMJET_FP=1 \
  .venv/Scripts/python -m unittest discover tests
```

Green = `Ran 175 tests ... OK (skipped=1)`, ~840 s. The kill-switches **must**
be on the command line: `tests/conftest.py` is a *pytest* convention and
`unittest discover` never loads it, so a bare run executes the FP primaries
live and reports 13 bogus failures. Never pipe the run through `tail` — the
pipeline exit code is `tail`'s, which masks failures.

---

# V4 changes (2026-08-13) — arcs, body loading, ramjet start policy, spiral climb

Five changes to `simple_model/`. **None of them alters a V2 or V3 re-fly** —
every new knob is defaulted to the pre-V4 behaviour, and
`tests/test_v2_frozen.py` / `tests/test_v3_frozen.py` were re-run after each
edit and stayed bit-identical (23 frozen tests green including the new V4 set).

## 1. `ramjet_simple.ramjet_thrust` — two optional arguments

```python
ramjet_thrust(..., lightoff_mach: float | None = None, allow_light: bool = True)
```

`lightoff_mach` overrides `RAMJET_MIN_LIGHTOFF_MACH` per call (so a campaign can
sweep the gate without monkeypatching the module global). `allow_light=False`
vetoes the light regardless of Mach.

**Why:** a bare Mach constant cannot express *where in the trajectory* the
ramjet lights, and section 3.3 of the learnings doc showed the same gate is
worth 0.052 g or 0.331 g depending only on which phase it fires in.

**V2 re-fly impact:** none. Both default to the old behaviour.

## 2. `flight_sim.RamjetStart` — the lightoff decision as a policy

```python
RamjetStart(gate_mach: float, require_descending: bool = True)
```

Passed to `run_flight(..., ramjet_start=...)`. Latched: once lit, stays lit.
`require_descending` vetoes the light while γ > 0, which is what enforces the
V4 requirement that the ramjet comes alive **in the dive**. New outputs:
`ramjet_lightoff_mach / _altitude_m / _time_s / _mode` and the boolean
`ramjet_lit_in_dive`.

**V2 re-fly impact:** none. `ramjet_start=None` (the default) uses the module
constant and the old always-allowed path.

## 3. Pitch arcs in the phase machine — the big one

V3 switched flight path angle from climb to dive, and from dive to drag strip,
**in a single timestep**; load factor never exceeded 1 anywhere in any V3
flight (learnings 3.6). V4 flies both transitions as constant-load-factor
circular arcs:

```
gamma_dot = g0 * (n - cos gamma) / V        R = V^2 / (g0 |n - cos gamma|)
```

New `ClimbDiveProfile` fields: `pullout_load_factor` (**None = V3 behaviour
exactly**, which is what keeps the freeze intact), `pushover_load_factor`
(default 0 g), `pullout_max_load_factor` (default 6 g).

**The floor is now a hard constraint and the load factor is what gives.** The
dive runs until a *nominal*-g arc can only just still make the 400 ft floor —
the latest possible pull-out, hence the longest dive and the most Mach for
lightoff — and if the arc then stops making the floor (it does; the ramjet is
lit and the vehicle accelerates through the arc) the commanded load rises to
hold it, up to the limit. `required_pullout_load_factor()` is the closed-form
inverse of `pullout_arc_drop_m()`.

`derive_top_altitude` now adds the arc drop, because the dive no longer gets to
use that altitude.

**Two traps worth inheriting, both cost me a full campaign run each:**

- A fixed safety factor on a predicted arc drop **does not work** — 632/1350
  flights busted the floor at a 1.15 factor. Solve for the load factor instead.
- `required_pullout_load_factor` is a **0/0 at the bottom of the arc** (γ→0 and
  remaining altitude→0 together). Unguarded it spikes the command to the 6 g
  limiter on the final step of every flight while the load actually carried
  through the arc was ~3.4 g. Guard on `1 - cos(gamma) <= 3.4e-4`.
- The floor check needs a **discretization tolerance** (1.0 m). An exact check
  flagged a 12 mm shortfall — one Euler step — as a floor bust on 561 flights.

**V2 re-fly impact:** none. `pullout_load_factor` defaults to `None`.

## 4. Body loading — new per-step outputs

`FlightState` gains `flight_path_angle_rad`, `load_n_roll`, `load_n_yaw`,
`load_n_total`, `turn_radius_m` (all appended with defaults, so positional
construction of the first 14 fields is unchanged).

- `load_n_roll` = `(T - D) / (m g0)` — axial, **accelerometer convention,
  gravity excluded**. Deliberately *not* `acceleration_m_per_s2 / g0`, which
  includes the weight-along-path term an onboard accelerometer cannot feel.
- `load_n_yaw` = `cos(gamma) + V*gamma_dot/g0` — normal, in the trajectory
  plane. Equals the commanded n during an arc, `cos(gamma)` on a straight leg.
- 2-D trajectory, so roll and yaw carry all of it.

Maneuvering lift now feeds drag: during an arc the wing carries n·W, not
W·cos γ, and induced drag goes as lift², so a hard pull-out is no longer free.
Implemented by passing `m * n_yaw / cos(gamma)` as the drag mass — **on a
straight leg `n_yaw == cos(gamma)` and this collapses to exactly `m`**, which
is precisely why V2/V3 stay bit-identical.

**V2 re-fly impact:** none (reduces to `m`). New fields are additive.

## 5. Spiral climb — `spiral_climb` / `spiral_bank_deg`

Zeroes the downrange rate during the climb: same air path, speed, γ, drag and
fuel; the ground track becomes a circle. Bank is **charged**, not free —
`n = cos(gamma)/cos(bank)`, which feeds the maneuvering-lift path above.

**Why it exists:** V4 needs a high top of climb (the only lever that moves
dive-exit Mach), but a straight climb to 1100 m spends 7.1 km of ground track,
cutoff lands 15–17 km downrange, and the return glide finishes 4–6 km short of
home. **Every single configuration in the 1350-flight campaign failed its
landing for that reason and that reason alone** — the powered mission closed
fine. Measured cost of the spiral: **0.07 kg payload and 0.006 Mach at top of
climb**, in exchange for 10.4 km of downrange and a landing that works.

Not modelled: roll-in/roll-out, heading alignment before the pushover.

**V2 re-fly impact:** none. `spiral_climb` defaults to `False`.

---

# V4.1 (2026-08-13, user): `RamjetStart.light_at_pullout`

One new field on `RamjetStart`, in **both** models (added to `simple_model`
in the same edit, so the two do not drift):

```python
RamjetStart(gate_mach: float, require_descending: bool = True,
            light_at_pullout: bool = False)     # <-- new
```

**What it does.** If the ramjet is still unlit when the pull-out arc begins,
the Mach gate is **waived** and the engine is lit there — "no matter what
speed you are at" (user, 2026-08-13). Latched, so the waiver survives into
the drag strip; without the latch the gate would re-arm and put the engine
straight back out.

**Why.** The gate is a *goal*, not a physical threshold: the dive is supposed
to deliver `gate_mach` and hand a lit engine to the drag strip. When the dive
under-delivers, the pure-gate policy does the worst available thing — it
keeps waiting for a Mach the vehicle will never see unpowered. Measured, at
FP fidelity: peak Mach 0.488 against a 0.50 gate, the ramjet **never asked**,
and the entire tank burnt at M ~0.44 (section below). The pull-out is the
last moment the decision still changes anything.

**One asymmetry between the models, and it is the interesting one.** Against
the closed-form `ramjet_simple`, the waiver *is* the light — that engine has
no opinion beyond the gate. Against `medium_model`'s `FpPropulsion` it only
means the ramjet is **asked** at the pull-out instead of never; the
first-principles model keeps the right to refuse, and
`FlightResult.ramjet_light_refused` records it if it does. This is the same
"the gate decides when to ASK" split as the base V4 policy.

Implementation notes worth inheriting:

- The waiver sets `gate_mach = 0.0` **only** on the override path, never on
  the already-lit path. Waiving it once lit would skip
  `RAMJET_LIGHTOFF_RAMP_MACH` (0.10 wide) and step the ramjet to full thrust
  at lightoff — which *would* change the frozen V4 re-fly.
- `FpPropulsion.thrust_and_fuel` gained `ignore_gate_mach`, since its own
  `mach >= self.lightoff_mach` check decides whether an FP solve is spent at
  all.

**V2 / V3 / V4 re-fly impact: none.** Defaults to `False`, and
`docs/v4_frozen/design.json` does not set it. Verified: the 28 frozen +
parity tests stay green, and rungs A and B re-flown with the flag ON are
digit-for-digit identical to the same rungs with it off (both already light
at M 0.500 in the dive, so the override never fires on a healthy flight).

---

# V4 SYNC STATUS (2026-08-13): mirrored into medium_model

All five V4 changes above are now in `medium_model/`, and the port was
verified before anything was flown:

- **V2 parity**: `tests/test_medium_model_v2_parity.py` still passes
  state-for-state (5 tests).
- **V3 parity**: the V4 reordering rewrites the phase machine that V2 never
  exercises (V2 has no `climb_dive` at all), so the frozen V3 design was flown
  through *both* models and compared step by step — **10,438 states, biggest
  delta 0.000e+00**. The V3 path is bit-identical.
- **V4 port check**: rung A below (legacy drag + closed-form engines)
  reproduces `docs/v4_frozen/design.json` on every headline number; the worst
  relative disagreement is 0.086 % and it is the rounding stored in the frozen
  JSON itself (`lands_from_launch_m` 2.5522 vs the recorded 2.55).

Four things a mirror of this port has to get right that are not obvious from
the simple_model diff:

1. **Maneuvering lift must reach the build-up drag path.** simple_model
   implements it by handing `m * n_yaw / cos γ` to the drag call as a mass.
   medium_model has three drag paths, and the third (`_buildup_drag`) computes
   required lift internally — it was being passed the bare `m`. Left alone,
   the V4 pull-out would have been free in exactly the run that matters.
2. **The phase machine has to move ahead of the engine call** in medium_model
   too, for the same reason as in simple_model (the start policy needs to know
   whether the vehicle is descending). Nothing in either direction reads the
   other, which is why V2/V3 stay bit-identical through the reorder.
3. **`RamjetStart` means something different against FP propulsion.** With the
   closed-form engines the policy *decides* the light, as in simple_model.
   With `FpPropulsion`, lightoff is a first-principles event, so the policy
   decides only when the ramjet is **asked** to cold light and the FP model
   keeps the right to refuse (user decision, 2026-08-13). New
   `FlightResult.ramjet_light_refused` records a refusal inside the policy
   window. `thrust_and_fuel` gained `allow_ramjet_light`; the veto applies to
   a COLD light only, since a lit flameholder does not go out because the
   vehicle pulled level.
4. **The V4 freeze is a different JSON schema** (`vehicle` /
   `legacy_wingspan_m` / `drag_strip_climb_angle_deg` / a separate
   `trajectory` block / mass from `verified_mission`), so
   `medium_model/design.py` dispatches to a second loader rather than
   aliasing keys.

Also changed on the medium side only: `fp_spec.MediumModelFpSpec` gained
`chamber_diameter_fraction` (default **0.95**, so every V3b/V3c/V3d FP result
stays reproducible). It was hard-coded at 0.95 in four places — chamber bore,
valve isometric scale, intake scale, ramjet combustor — and V4's premise is
that the chamber **is** the body, so a V4 run passes **1.0** and all four move
together (user decision, 2026-08-13). Note this contradicts the claim in the
V4 notes below that medium_model "keeps a true 214 mm chamber inside a 214 mm
skin": before this change it modelled a 203.3 mm chamber, i.e. it would have
read *less* thrust than simple_model, not more.

---

# What medium_model should check first

**The top of climb.** V4 needs 1100 m to make a M 0.50 gate reachable in the
dive, and that sits just under the FP flame-out ceiling (~1250 m alive /
1275 m dead at CONFIRM — a ceiling that **dropped ~200 m** on one grid
refinement). simple_model has no flame-out model at all. It independently
shows `min_powered_accel_g = -0.021 g` at the top of climb, i.e. the vehicle is
already at its climb ceiling there, which is the same wall FP found ("at
1100 m the duct makes 83.5 N; an 8° climb needs ~81 N"). If the FP ceiling is
real, **the V4 gate is not reachable and the answer reverts toward M 0.45.**

Two further notes for the port:

- The V4 airframe sets **body diameter = chamber diameter = 214 mm**. In
  simple_model that shrinks the chamber 9.7 % too (one diameter variable), so
  simple_model is *understating* thrust relative to a medium_model that keeps a
  true 214 mm chamber inside a 214 mm skin. Safe direction, but it means the
  two models will not agree on thrust and the difference is expected.
- The throat had to come down **121.6 → 115.6 mm**: V3b's throat against a
  214 mm body is area fraction 0.323, over the 0.30 operability cap that zeroes
  the pulsejet outright. This shrinks the ramjet throat as well.

Frozen at `docs/v4_frozen/design.json`, guarded by `tests/test_v4_frozen.py`
via `scripts/fly_frozen_v4.py` (subprocess — CD0 is still baked at import).

---

# ANSWER (2026-08-13): the top of climb was the right thing to check, and it fails

`out_medium_model/v4_gate3_report.md` — build-up drag + first-principles
engines at CONFIRM (324 cells), chamber = body diameter, design.json
unmodified.

**V4 does not close at FP fidelity, and the predicted mechanism is the one
that fired.** Peak Mach **0.488** against the 0.50 gate. The chain:

- The FP pulsejet **quenches twice on the climb** — M 0.333 at 1038 m and
  M 0.356 at 948 m. The flame-out boundary at CONFIRM with a full-diameter
  chamber sits around **950–1100 m**, i.e. *lower again* than the ~1250 m the
  freeze assumed, and V4's commanded top of climb is 1100 m — inside it.
- Top of climb is therefore reached at **M 0.284**, not M 0.381: the dive
  starts 0.097 Mach down.
- Dive exit is **M 0.488** (A: 0.600, B: 0.565), so the floor triggers the
  pull-out and the gate is never crossed.
- **The ramjet is never asked.** `ramjet_light_refused = false` — this is not
  the FP ramjet declining, the policy window never opened. Whether it would
  light at M 0.50 remains untested.
- The vehicle then sits on the drag strip for 289 s with thrust within a few
  newtons of drag and empties the tank at M ~0.44.

Thrust calibration, measured at the 34 live points of the march: the FP
pulsejet makes **90.6 %** of the closed-form value (worst live point 76.7 %).
The freeze's 10 % haircut was about right *on average* — and it was still the
wrong instrument, because what kills the design is the quench, which no scalar
haircut can express.

One more thing the ladder found, unrelated to propulsion: **the
return-to-launch profile does not survive build-up drag.** Rung B closes the
powered mission and still stalls the glide 2.7 km from home. That is a
trajectory problem in its own right and is untouched here.

## And then: `light_at_pullout` rescues the powered mission

Rung C+ — identical vehicle, identical trajectory, identical FP settings, one
policy flag:

**The first-principles ramjet cold-lit at M 0.488, 163 m, phi 0.863, inside
the pull-out — and the mission closed.** Peak Mach **1.100**, cutoff reached,
2.125 kg burnt of a 2.764 kg allocation (77 %), peak body load 3.65 g.

So the M 0.50 gate was the only thing standing between this airframe and a
closed supersonic run. It was never an engine limit: the flameholder lights
0.012 Mach below the gate perfectly happily. It was a *policy* limit, and the
policy was written to wait for a Mach the vehicle could no longer reach.

Three things this does **not** fix, and they should not be lost in the good
news:

1. **The ramjet no longer lights in the dive** — it lights in the pull-out.
   `ramjet_lit_in_dive` is False. That was the stated V4 requirement, and this
   is a deliberate relaxation of it, not a satisfaction of it.
2. **The pulsejet still quenches twice on the climb**, unchanged: engine-only
   thrust margin 0.000 and min powered acceleration −0.390 g at the top. The
   climb ceiling is a separate, untouched problem.
3. **The landing still fails** — stalled 4.1 km from home, the same build-up
   drag / return-glide problem rung B showed.

## Frozen and exported

- **Freeze**: `docs/v4_medium_frozen/design.json` — all six rungs, their
  inputs, the constants, the git SHA. Guarded by
  `tests/test_v4_medium_frozen.py`, which re-flies the four closed-form rungs
  in subprocesses. The two FP rungs cost ~35 min each and are **recorded-only**
  (`DOUGLAS_DART_REFLY_FP_RUNGS=1` re-flies them) — a real gap, named rather
  than hidden: the cheap rungs verify the trajectory machinery, not the
  propulsion result.
- **Export for downstream vehicle modelling**: `out_medium_model/v4_export/`
  (`scripts/medium_model_v4_export.py`) — structural dimensions + stations,
  mass budget, loads envelope, per-step time series, fuel budgets, the FP
  engine march, and the trajectory command set.
- Every FP flight now writes a gzipped per-step state trace next to its
  summary, so no later question about an FP rung costs another 35 minutes.
