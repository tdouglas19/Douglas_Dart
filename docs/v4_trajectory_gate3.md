# Douglas Dart V4 — vehicle and trajectory specification for Gate 3

**Status: FROZEN.** Source of truth is `docs/v4_frozen/design.json`, locked by
`tests/test_v4_frozen.py`, which re-flies it through `scripts/fly_frozen_v4.py`
in a subprocess (CD0 is baked at import time).

Produced by the calibrated closed-form `simple_model` at
`CD0_FRONTAL = 0.1` (frontal-area referenced). Propane.
Wet mass 22.68 kg (50 lb) at release. Motor cutoff at Mach
1.1.

This document states **what the vehicle is and what it flies**. It does not
cover how the configuration was selected.

---

## 1. Vehicle

### 1.1 Body and duct

| item | value |
|---|---|
| body diameter | **214.0 mm** |
| chamber diameter | 214.0 mm (the chamber **is** the body — no annular gap) |
| chamber length | 389.3 mm |
| duct length (cone + tailpipe) | 1242.3 mm |
| throat / nozzle exit diameter | **115.6 mm** |
| throat / body area fraction | 0.2920 (operability cap 0.3) |
| nose fairing | 428 mm (2.0 D) |
| aft body / boattail | 214 mm (1.0 D) |
| **overall body length** | **2274 mm** |
| fineness ratio | 10.62 |
| frontal area | 0.03597 m² |

### 1.2 Wing

| item | value |
|---|---|
| span | 532.5 mm |
| aspect ratio | 1.8596 |
| taper ratio | 0.5120 |
| leading-edge sweep | 13.42° |
| section | thin_cambered |
| reference area | 0.15250 m² |
| Oswald e | 0.8469 |
| CL_max (sweep-effective) | 1.1353 |

### 1.3 Mass and fuel

| item | value |
|---|---|
| engine duct (steel) | 5.595 kg |
| airframe skin (CFRP + overhead) | 4.952 kg |
| wing | 0.915 kg |
| avionics | 2.000 kg |
| tank hardware | 0.961 kg |
| landing hardware | 0.500 kg |
| **dry mass** | **14.926 kg** |
| fuel burned (mission) | 2.457 kg |
| fuel loaded (burn + 25 % reserve) | 3.071 kg |
| tank capacity (annulus) | 7.798 kg |
| **payload / ballast margin** | **4.683 kg** |
| duct wall thickness | 1.00 mm |

Mass is re-derived from this geometry; no block is inherited from a parent
design.

---

## 2. Trajectory

### 2.1 Commanded parameters

| parameter | value |
|---|---|
| release velocity | 40 m/s (rail / catapult) |
| **ramjet lightoff gate** | **Mach 0.50, and only while descending** |
| top of climb | **1100 m** (commanded, not derived) |
| climb flight path angle | 6.0° |
| climb type | **spiral** (helical), 30° bank |
| pushover load factor | 0.0 g |
| dive flight path angle | −14.0° |
| dive terminates at | Mach 0.60 |
| pull-out load factor (nominal) | 3.0 g |
| pull-out load factor (limit) | 6.0 g |
| hard altitude floor | 121.92 m (400 ft) |
| drag-strip flight path angle | 1.0° |
| motor cutoff | Mach 1.1 |

### 2.2 Phase sequence

| phase | t (s) | duration (s) | altitude (m) | Mach | γ (deg) | peak load (g) |
|---|---|---|---|---|---|---|
| 1. Spiral climb | 0.0 – 97.0 | 97.0 | 0 → 1100 | 0.118 → 0.381 | +6.0 → +6.0 | 1.21 |
| 2. Pushover arc (pitch-down) | 97.0 – 101.7 | 4.7 | 1100 → 1056 | 0.381 → 0.405 | +6.0 → -14.0 | 0.12 |
| 3. Dive | 101.7 – 123.0 | 21.3 | 1055 → 219 | 0.405 → 0.599 | -14.0 → -14.0 | 1.65 |
| 4. Pull-out arc (pitch-up) | 123.0 – 126.0 | 3.0 | 218 → 147 | 0.600 → 0.736 | -14.0 → +0.9 | 3.45 |
| 5. Drag strip | 126.0 – 133.7 | 7.7 | 147 → 190 | 0.737 → 1.100 | +1.0 → +1.0 | 2.40 |

**1. Spiral climb.** Climbs at 6° to
1100 m on a helical ground track over the launch point:
identical air path, speed, flight path angle, drag and fuel to a straight
climb, but **zero net downrange**. The 30° bank is
carried as load factor n = cos γ / cos φ = 1.148 g,
which feeds induced drag. Ground-track turn radius
**2,894 m**.

**2. Pushover arc.** Constant 0.0 g pitch-down
from +6° to −14°.
Radius **1,680 m**, duration
4.7 s.

**3. Dive.** Straight at −14°. **The ramjet lights in
this phase** — see §2.3. The dive ends on Mach 0.60.

**4. Pull-out arc.** Pitch-up to the drag-strip angle at
3.0 g nominal. Radius
**2,087 m**, duration
3.0 s. The floor is a hard constraint and the
load factor is the free variable: if the arc stops making the floor, the
commanded load rises, capped at 6.0 g. On this
design the nominal 3.0 g is sufficient and the arc
bottoms out at **149.2 m**, i.e.
27.3 m above the
121.9 m floor.

**5. Drag strip.** 1.0° to motor cutoff at
Mach 1.1, reached at 133.7 s,
190 m, 7.10 km downrange.

### 2.3 Ramjet lightoff

| item | value |
|---|---|
| gate | Mach 0.50, **must be crossed while descending** |
| lights at | **Mach 0.500, 503 m, t 116.6 s** |
| phase at lightoff | **`v3_dive` — in the dive** |
| latching | once lit, stays lit through the pull-out and drag strip |
| pulsejet | continues running additively through and after lightoff |

The gate is a policy, not a bare Mach constant: a Mach threshold alone cannot
express *where* in the trajectory the engine comes alive, and the requirement
is that it comes alive in the dive.

### 2.4 Body loading

Accelerometer convention — specific force, **gravity excluded**. A 2-D
trajectory loads two body axes: roll (axial) and yaw (normal, in-plane).

| item | value |
|---|---|
| **peak total** | **3.45 g**, in `v4_pullout` |
| peak yaw axis (normal) | 3.00 g |
| peak roll axis (axial) | 2.19 g |
| spiral climb | 1.148 g normal (bank), sustained |
| pushover arc | 0.0 g normal |
| pull-out arc | 3.0 g normal commanded |
| drag strip | axial load rises to 2.19 g under ramjet acceleration |

At 3.45 g on 22.68 kg the airframe carries
767 N
(172 lbf). Structure
is not modelled anywhere in this repo; this is a stated design limit, not a
demonstrated capability.

### 2.5 Return and landing

After cutoff the vehicle flies the return-to-launch profile: pitch-up
half-loop to reverse heading, glide home at the airframe's equilibrium slope,
spiral down over the launch point, flare to touchdown at stall speed.

| item | value |
|---|---|
| touchdown speed | 43.3 m/s (= stall speed) |
| stall speed at landing | 43.3 m/s (cap 45 m/s) |
| lands from launch point | 2.6 m |
| total flight time | 279 s |

---

## 3. Verified mission numbers

| metric | value |
|---|---|
| peak Mach | 1.100 |
| motor cutoff reached | True |
| ramjet lit in the dive | **True** |
| dive exit Mach | 0.600 |
| peak thrust / weight | 6.07 |
| min traverse acceleration | 0.163 g |
| min powered acceleration | -0.0213 g |
| min powered thrust margin (engine-only) | 0.856 |
| min altitude under power | 149.2 m |
| floor violated | False |
| γ ≥ 0 rule (M 0.80 → cutoff) | satisfied |
| fuel burned / capacity | 2.457 / 7.798 kg |
| payload margin | 4.683 kg |

## 4. Constraint status

| constraint | target | V4 | status |
|---|---|---|---|
| ramjet lights in the dive | required | Mach 0.500 in `v3_dive` | **PASS** |
| peak Mach | ≥ 1.0 | 1.100 | **PASS** |
| motor cutoff reached | yes | True | **PASS** |
| 400 ft floor | ≥ 121.9 m | 149.2 m | **PASS** |
| peak body load | ≤ 4.0 g | 3.45 g | **PASS** |
| stall speed | ≤ 45 m/s | 43.3 m/s | **PASS** |
| safe landing | yes | True | **PASS** |
| fuel within capacity | ≤ 6.239 kg | 2.457 kg | **PASS** |
| payload margin | > 0 | 4.683 kg | **PASS** |
| γ ≥ 0 from M 0.80 | required | satisfied | **PASS** |
| min traverse acceleration | ≥ 0.25 g | 0.163 g | **FAIL** |
| min powered acceleration | > 0 | -0.0213 g | **FAIL** |
| min powered thrust margin | ≥ 1.15 | 0.856 | **FAIL** |

The three failures are engine-limited, not trajectory-limited, and none has
been met by any V2, V3 or V4 configuration in any campaign to date.

## 5. Items for Gate 3 to verify

1. **Top of climb at 1100 m.** The first-principles
   pulsejet flames out at roughly 1250 m, and that ceiling fell ~200 m when
   the grid was refined from 162 to 324 cells. `simple_model` has no
   flame-out model — it only lapses thrust as (ρ/ρ_SL)³ — and already shows
   powered acceleration going slightly negative at the top of the climb. If
   the flame-out ceiling is confirmed lower, this trajectory does not close.
2. **Pulsejet sustain at four operating points**, not one: release, top of
   climb, the lightoff condition, and the dive exit.
3. **Reaching the Mach 0.50 gate inside the dive**, at
   FP fidelity. This is an acceleration question and closed-form screens of
   it are unreliable near a boundary.
4. **Climb margin at ΔM ≤ 0.02.** Coarser propulsion time steps flatter the
   climb margin by roughly the entire margin.
5. **Verify at CONFIRM tier (324 cells), not MARCH (162).**
6. **Side valve runner packaging.** The chamber is now the full body
   diameter, so the runners need an external fairing whose drag is not in
   this CD0.

## 6. Reproduce

```bash
SIMPLE_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python scripts/fly_frozen_v4.py
```

Regenerate this document:

```bash
.venv/Scripts/python scripts/simple_model_v4_gate3_spec.py
```

Plots and the fuller write-up: `out_simple_model/v4_optimal_design.md`.
`simple_model` changes made for V4, for the parallel `medium_model` session:
`docs/simple_model_changes_for_medium_model.md`.
