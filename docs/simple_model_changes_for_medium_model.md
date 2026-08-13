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
