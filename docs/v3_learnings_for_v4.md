# V3 learnings — handoff for a V4 design cycle

**Audience:** a fresh workspace that will develop V4 in `simple_model/`
first, then carry it into `medium_model/`. Assumes no context from the V3
campaigns. Every number here is measured unless explicitly marked as an
estimate or inference.

**Vehicle:** Douglas Dart — 50 lb (22.68 kg) wet, pulsejet→ramjet
combined-cycle supersonic dart. Propane. Rail/catapult release at 40 m/s.
Motor cutoff at Mach 1.1.

**Provenance of the numbers below:**

| source | what it is | speed |
|---|---|---|
| `simple_model/` | calibrated closed-form; the search/optimizer model | ~0.7 s/flight |
| `medium_model/` | standalone copy of simple_model + drag build-up, lift module, FP propulsion hookup | ~1–2 s/flight closed-form |
| `pulsejet-fp/` | first-principles unsteady 1D pulsejet (HLLC + valve ODE + Arrhenius) | ~20 s/point |
| `../ramjet-fp` | first-principles unsteady ramjet (RH inlet + WSR flameholder) | ~20 s/point |
| FP flight | medium_model trajectory with FP engines marched in the loop | 6–25 min/flight |

Fidelity tiers for the FP solvers: **MARCH** = 162 cells, **CONFIRM** =
324, **AUDIT** = 486 (never run on a full flight).

---

# PART 1 — WHERE V3 ENDED UP

## 1.1 The design that survives

**V3b** = the V3a airframe with a revised trajectory and lightoff gate.
Frozen at `docs/v3b_medium_model/design.json`.

| item | value |
|---|---|
| body | 225.2 mm dia × 2307.3 mm, fineness 10.24 |
| duct | 1631.6 mm total |
| chamber | 214.0 mm dia × 389.3 mm |
| cone | 215.3 mm |
| tailpipe | 1027.0 mm — **tail/chamber-dia = 4.80** |
| throat / nozzle exit | 121.6 mm (A_th/A_body = 0.292, cap is 0.30) |
| nose fairing | 450.4 mm (2.0 D) |
| aft body | 225.2 mm (1.0 D) |
| wing | span 532.5 mm, AR 1.860, taper 0.512, LE sweep 13.42°, thin cambered, S = 0.1525 m² |
| fuel | propane, 2.693 kg loaded, **2.424 kg burnable** (90 % rule) |
| trajectory | climb 10° → dive 9.89° → floor 133.1 m → drag strip |
| ramjet gate | **M 0.30** |

**Verified at CONFIRM tier (n_cells 324):** peak M 1.100 with motor
cutoff, min traverse acceleration **0.352 g**, climb margin **+0.036 g**,
thrust margin 1.087, 1.848 kg burned of 2.424 kg.

At MARCH tier the same design gives 0.389 g / +0.049 g / 1.078 / 1.845 kg.

## 1.2 The alternative, if ignition reliability forces it

**V3d** — `docs/v3d_medium_model/design.json`. Same airframe, climb 8° /
dive 14° / floor 122 m, **gate M 0.40**. CONFIRM: traverse 0.283 g, climb
margin +0.019 g, thrust margin 1.045, 1.779 kg.

Lights the ramjet on ~163 N instead of ~95 N — materially better ignition
margin in real hardware, which no model here scores. But its climb margin
sits *at* the model's own grid uncertainty (see §4.3), so it probably does
not survive the next fidelity step.

## 1.3 Gate status — read this before setting V4 targets

| gate | target | best ever achieved | status |
|---|---|---|---|
| peak Mach | ≥ 1.0 | 1.100 | **PASS** |
| motor cutoff reached | yes | yes | **PASS** |
| min traverse accel | ≥ 0.26 g (user-set) | 0.352 g | **PASS** |
| min powered accel | ≥ 0.25 g | **0.123 g** | **FAIL — never met by anything** |
| min powered thrust margin | ≥ 1.15 | **1.106** | **FAIL — never met by anything** |
| stall speed | ≤ 45 m/s | 45.8 legacy / **53.0** with `lift.py` | **FAIL** |
| fuel | < 2.424 kg | 1.85 kg | PASS |

Two project gates have **never been met by any V2 or V3 configuration in
any campaign**, and both are climb-phase thrust problems. If V4 is meant
to close them, that is an engine/drag problem, not a trajectory problem.
Do not spend a campaign on trajectory hoping to fix them — V3 did, and the
gap did not close.

---

# PART 2 — PHYSICS LEARNINGS

## 2.1 Pulsejet: the sustain rule, and why the version you may have heard is wrong

A pulsejet duct either holds a limit cycle or it starts, rings down over
~15–20 pulses, and dies into a steady non-oscillating burn. The
discriminator found in V3 is **tail length ÷ chamber diameter**, not duct
length and not any D/T figure (V3 was dead at D/T 20.6 where a V2 variant
sustained at 20.4).

Physically: the tailpipe gas column is the inertia driving the cycle — the
"liquid piston" — so it must be long *relative to the chamber it breathes
from*, not merely long.

Measured at 235 mm chamber, M 0.15 / 60 m:

| tail/chamber-dia | thrust | p/p0 span | verdict |
|---|---|---|---|
| 3.60 | 0.9 N | 0.989–1.012 (0.024) | **DEAD** |
| 3.70 | 134.9 N | 0.829–1.417 | sustains |
| 3.80 | 140.0 N | 0.826–1.427 | sustains |
| 4.00 | 149.2 N | 0.817–1.443 | sustains |
| 4.25 | 159.1 N | 0.809–1.458 | sustains |

**The cliff is a knife-edge: 23.5 mm of tail separates a dead engine from
135 N.**

### ⚠ THE MOST IMPORTANT CAVEAT IN THIS DOCUMENT

**The `t/D ≥ 4.5` rule, and that entire cliff table, were measured at ONE
operating point: M 0.15 / 60 m — a launch condition. The sustain boundary
MOVES WITH MACH AND ALTITUDE.**

The 235 mm duct at t/D 4.00 was recommended on the strength of that table
(+21 % thrust, 39 mm shorter body). It is dead everywhere the vehicle
actually flies:

| duct | M 0.15/60 m | M 0.25/300 | M 0.35/300 | M 0.45/300 | M 0.54/300 | M 0.54/900 |
|---|---|---|---|---|---|---|
| 235 mm, t/D 4.00 | 153.1 N | **0.94 DEAD** | **0.95 DEAD** | **0.96 DEAD** | **1.10 DEAD** | **1.36 DEAD** |
| V3a, t/D 4.80 | 125.5 N | — | 114.9 | 111.6 | 108.3 | 89.4 |
| 235 mm at t/D 4.80 | 176.8 N | — | 157.0 | 152.1 | 147.4 | 118.6 |

**Rule for V4: a pulsejet sustain check at one operating point is not a
sustain check.** Any candidate duct must be verified alive at, at minimum:
release, top-of-climb (highest altitude), the lightoff condition, and the
dive exit. Four points, ~20 s each — cheap insurance against inventing a
design that only exists at launch.

## 2.2 Pulsejet: thrust is flat with Mach — this is the root constraint

The inlets are **side-mounted reed valves** taking boundary-layer air at
approximately static pressure. There is no ram recovery. Consequently
thrust does not rise with flight speed the way a forward-inlet engine's
would; it is nearly flat and slightly falling.

FP pulsejet thrust, V3a duct, n_cells 324:

| M | alt | thrust |
|---|---|---|
| 0.118 | 30 m | 120.5 N |
| 0.200 | 250 m | 115.8 N |
| 0.300 | 580 m | 102.9 N |
| 0.350 | 500 m | 104.0 N |
| 0.440 | 220 m | 109.6 N |
| 0.500 | 150 m | 109.6 N |
| 0.600 | 122 m | 106.8 N |

Meanwhile drag goes as V². Thrust and drag therefore **cross near M 0.47**
and diverge fast. FP, ~300 m:

| M | T_pulsejet | D_vehicle | T−D | as g | dive needed to hold speed |
|---|---|---|---|---|---|
| 0.30 | 116.5 N | 55.9 N | +60.6 | +0.28 | — |
| 0.40 | 113.3 N | 85.9 N | +27.4 | +0.13 | — |
| 0.45 | 111.6 N | 104.9 N | +6.7 | +0.031 | level ok |
| **0.50** | 109.8 N | 126.1 N | **−16.3** | −0.076 | 4.4° |
| 0.55 | 107.9 N | 149.6 N | −41.7 | −0.193 | 11.1° |
| 0.60 | 106.0 N | 175.0 N | **−69.0** | −0.319 | **18.6°** |

**Consequence — the vehicle cannot exceed ~M 0.47 on the pulsejet alone,
and no trajectory fixes it.** Full FP flights, gate set above the
crossover so the ramjet never lights:

| dive angle | peak Mach reached |
|---|---|
| 22° | 0.461 |
| 25° | 0.464 |
| 28° | 0.468 |
| 30° | 0.470 |

All four ran the tank dry (2.424 kg) without cutoff. **Dive angle does not
move this wall.** A steeper dive gives more gravity, but the dive
terminates at the floor before +0.02 g can integrate into meaningful
speed.

**This single fact shapes the entire V3 trajectory.** The climb-dive
profile exists because the engine cannot push through its own thrust
lapse; gravity has to supply the acceleration. If V4 changes one thing,
this is the highest-leverage candidate — see §6.1.

## 2.3 Pulsejet: altitude ceiling

Thrust falls with altitude, and above a limit the duct flames out
entirely. FP, M 0.35:

| n_cells | 900 m | 1100 m | 1150 m | 1200 m | 1250 m | 1275 m | 1500 m |
|---|---|---|---|---|---|---|---|
| 162 | 97.0 | — | — | — | — | 81.5 alive | **1.10 DEAD** |
| **324** | 90.7 | 83.5 | 81.6 | 79.5 | 77.4 alive | **0.58 DEAD** | — |

**The ceiling is ~1250 m alive / 1275 m dead at CONFIRM, and it DROPPED
~200 m when the grid was refined from 162 to 324.** It may be lower still
at AUDIT. Treat ~1200 m as the working ceiling with any margin at all.

Note the practical ceiling is *lower* than the flame-out ceiling: at
1100 m the duct makes 83.5 N, while an 8° climb needs ~31 N of gravity
plus ~50 N of drag = 81 N. The vehicle can barely climb to 1100 m under
its own power.

## 2.4 Pulsejet: sizing

- Thrust scales roughly as **chamber diameter³** at fixed t/D; stretching
  the tail alone saturates (214 mm gains only +23 % from t/D 4.5 → 6.5).
- Buying thrust with diameter costs less *length* than buying it with
  tail — but the diameter also drives **frontal area**, and drag is
  frontal-area referenced. 214 → 235 mm is +9.8 % diameter, **+20.7 %
  frontal area**.
- 235 mm held at V3a's t/D 4.80 is worth **+36 % FP thrust** — but that
  vehicle is **188 mm longer**, not shorter.
- Pulsejet thrust is **not grid-converged**. It falls 3–4 % per refinement
  level at 200→300→400 cells and was still falling at 400.

## 2.5 Ramjet: lights far earlier than assumed, and φ is an output

The FP ramjet lights cold (with pilot) at every condition probed down to
**M 0.25**. φ is **not commanded** — the model self-selects the
leanest-stable-plus-margin mixture per (M, alt) and reports it.

| flight M | alt | φ (output) | thrust | fuel |
|---|---|---|---|---|
| 0.25 | 600 m | 0.831 | 66.8 N | 0.0218 kg/s |
| 0.30 | 550 m | 0.839 | 95.4 N | 0.0271 kg/s |
| 0.35 | 500 m | 0.855 | 124.8 N | 0.0312 kg/s |
| 0.40 | 450 m | 0.847 | 163.3 N | 0.0366 kg/s |
| 0.45 | 400 m | 0.863 | 202.6 N | 0.0408 kg/s |

In flight, φ runs **0.797–0.909** — a dip immediately after light, a long
plateau near 0.862, a step to ~0.909 approaching cutoff. That curve, not a
constant, is what a throttle schedule must reproduce.

**The ramjet is never the binding constraint anywhere in V3.** Every
ceiling found was the pulsejet's.

---

# PART 3 — TRAJECTORY LEARNINGS

## 3.1 The V3 climb–dive profile

Climb steeply on surplus low-Mach thrust → dive through the ramjet
lightoff notch (gravity supplies what the engine can't) → pull out at a
hard floor → level "drag strip" to cutoff. Top-of-climb is **derived from
the dive**, not searched.

## 3.2 Top-of-climb altitude is set by the DIVE, not the climb

Verified across climb 3–18°: every climb angle at a given (dive, floor)
derives the **same** top. A shallower climb does not lower the top, it
only takes longer to reach it.

| dive angle | derived top (floor 122 m) |
|---|---|
| 9.89° | 589 m (at floor 133) |
| 14° | 672 m |
| 16° | 710 m |
| 18° | 743 m |
| 20° | 772 m |
| 25° | 1053 m |
| 30° | 1131 m |

**Top-of-climb MACH, however, falls steeply with climb angle** (closed-form,
dive 9.89):

| climb | TOC time | TOC alt | TOC Mach |
|---|---|---|---|
| 8° | 45.3 s | 589 m | 0.369 |
| 10° | 40.3 s | 589 m | 0.336 |
| 12° | 37.2 s | 589 m | 0.305 |
| 14° | 35.2 s | 589 m | 0.274 |

A steeper climb spends more of the engine on gravity and arrives slower.

## 3.3 The lightoff gate is not a constant — it is a *position*

`RAMJET_MIN_LIGHTOFF_MACH = 0.45` in `medium_model/constants.py` is a
screening placeholder that the FP engine contradicts (it lights at 0.25).
Correcting it was the single largest improvement in the V3 campaign —
worth more than every geometric change combined.

But it is **non-monotone and interacts with climb angle**, because a Mach
gate's *consequence* is positional: it decides where in the trajectory the
ramjet lights.

FP flights at climb 8° / dive 9.89° (MARCH). Top of climb is at 50.7 s:

| gate | lights at | relative to top of climb | traverse | fuel |
|---|---|---|---|---|
| 0.45 | 66.9 s, 215 m | +16.2 s, deep in the dive | 0.183 | 1.764 kg |
| 0.42 | 61.6 s, 350 m | +10.9 s | 0.209 | 1.751 kg |
| 0.40 | 58.5 s, 424 m | +7.8 s | 0.244 | 1.771 kg |
| 0.38 | 55.8 s, 485 m | +5.1 s | 0.277 | 1.797 kg |
| **0.35** | **52.4 s, 557 m** | **+1.7 s, just past the top** | **0.331** | 1.784 kg |
| 0.30 | 34.4 s, 343 m | −16.3 s, mid-climb | **0.052** | 2.161 kg |
| 0.25 | 23.4 s, 198 m | −27.3 s, low climb | 0.073 | 2.244 kg |

**A tent function peaked at top of climb, and asymmetric:** lighting late
costs ≈0.009 g per second of delay; lighting early costs ≈0.017 g per
second — roughly twice as expensive.

Why the asymmetry: a gate *below* the top-of-climb Mach lights the ramjet
with the whole remaining climb still to fly, so it burns 0.022–0.027 kg/s
at its worst thrust for 16–27 s while the vehicle is still fighting
gravity. Fuel rises 21–27 %. A gate *above* it only wastes part of a short
dive.

### Two rules, one at each end of the dive

- **For maximum traverse:** set the gate ≈ the Mach at **top of climb**.
  Confirmed: 0.35 at climb 8° (FP TOC M 0.334), 0.30 at climb 12°.
- **For latest ignition:** set the gate ≈ the Mach at the **bottom of the
  dive**. The gate must be crossed *while still descending*. At climb 12 /
  dive 20, M 0.44 is crossed at 220 m still in the dive → traverse 0.368;
  M 0.45 is not crossed until 179 m in **level flight after pullout** →
  traverse collapses to 0.006.

**Design implication for V4:** make the lightoff gate a scheduled design
variable, not a scalar constant. A scalar default is the wrong *shape* for
this parameter.

## 3.4 Dive angle: what it buys and what it costs

Dive angle is the dominant trajectory lever for traverse acceleration
(closed-form, floor 122):

| climb ↓ / dive → | 5° | 10° | 15° | 20° |
|---|---|---|---|---|
| 9° | 0.132 | 0.208 | 0.284 | 0.362 |
| 12° | 0.137 | 0.215 | 0.295 | 0.374 |
| 15° | 0.138 | 0.222 | 0.303 | **0.383** |
| 18° | 0.072 | 0.121 | 0.166 | 0.210 |

But it costs **climb margin**, through the derived top: a steeper dive
demands a higher top, and thinner air means less pulsejet thrust up there.
Measured at CONFIRM, climb 8°:

| dive | derived top | climb margin |
|---|---|---|
| 14° | 672 m | **+0.019 g** |
| 15° | 692 m | −0.014 g |
| 16° | 710 m | −0.014 g |

**The climb margin at a given climb angle is set by the DIVE angle, not
the gate.** That is counter-intuitive and worth remembering.

Lowest legal floor always wins (weak, monotone): 122 → 260 → 400 m gives
0.383 → 0.367 → 0.351 g.

## 3.5 The climb is always the pinch

`min_powered_accel_g` sits in the climb phase in **all 130** closed-form
sweep flights, without exception, and in every FP flight. It is not
monotone in climb angle — it peaks near 12–15° and degrades either side:

| climb | climb-exit M | T | D | W·sinγ | powered a |
|---|---|---|---|---|---|
| 3° | 0.436 | 102.9 | 94.9 | 10.9 | −0.0141 g |
| 9° | 0.361 | 106.9 | 70.6 | 33.8 | +0.0117 g |
| 12° | 0.316 | 109.4 | 58.6 | 45.1 | +0.0263 g |
| 15° | 0.270 | 111.9 | 49.1 | 56.2 | +0.0304 g |
| 18° | 0.219 | 114.6 | 42.9 | 67.1 | +0.0211 g |

Because the top is fixed, a shallower climb spends longer under power and
**exits at much higher Mach** (0.436 at 3° vs 0.219 at 18°) — where the
side-inlet pulsejet has lapsed and drag has nearly doubled. That swamps
the gravity term it saved.

Caution: shallow climbs (3–6°) *flame out* — fuel pins at the 2.424 kg cap
and cutoff is never reached. Their apparently-decent traverse numbers are
on flights that never finish. **Always check `motor_cutoff_reached`
before believing a traverse number.**

## 3.6 The dive pull-out is not modelled at all

`flight_sim.run_flight` latches `v3_dive_done` and switches flight-path
angle from the dive angle to the drag-strip angle **in one timestep**.
There is no pull-out arc, no load factor, no g limit anywhere in the phase
machine. Load factor never exceeds 1 in the model.

Priced properly at the dive exit (M 0.60, 122 m, q 25 167 Pa, S 0.1525 m²,
CL_max 0.890 section-stall-limited from `medium_model/lift.py`, W 222 N):

| dive | n @ 15 m arc | n @ 30 m arc | n @ 60 m arc | n available |
|---|---|---|---|---|
| 9.89° | 5.2 | 3.1 | 2.0 | **15.4** |
| 14° | 9.4 | **5.2** | 3.1 | 15.4 |
| 18° | 14.8 | 7.9 | 4.5 | 15.4 |
| 20° | 18.0 ✗ | 9.5 | 5.3 | 15.4 |
| 25° | 27.5 ✗ | 14.2 | 7.6 | 15.4 |
| 30° | 38.9 ✗ | 19.9 ✗ | 10.5 | 15.4 |

Structural load is **not modelled anywhere in the repo**. At dive 20° with
a 30 m arc, 9.5 g on 22.68 kg is **2118 N (476 lbf) through the wing
joint** on a 50 lb airframe.

**V4 action:** either add a pull-out arc with a load-factor limit to the
phase machine, or impose a dive-angle cap justified by this table. V3
used 9.89–14°, which is comfortable. Anything ≥ 20° is leaning on an
unmodelled allowance.

---

# PART 4 — MODEL FIDELITY LEARNINGS

**This is the most transferable section. The V3 campaign's biggest
mistakes were all fidelity mistakes, not physics mistakes.**

## 4.1 Closed-form is optimistic about ACCELERATION specifically

Closed-form propulsion runs 6–16 % high on thrust and ~22 % high on
traverse. That much is a calibratable offset. The dangerous part is
**where** the error lands.

A "haircut" of 0.781 (FP ÷ closed-form traverse) was calibrated across
5 paired flights at dive 9.89° and gates 0.30–0.44 and looked solid. It
**collapses near a boundary**: at gate 0.45 the measured ratio is **0.05,
not 0.78**.

Worked example of the failure mode: closed-form said climb 12 / dive 30 /
gate 0.50 would give traverse 0.415, projecting to ~0.30 under the
haircut. The FP flight scored **−0.046 g and never reached cutoff.**

**Rule:** any question whose answer depends on *reaching* a condition — a
Mach gate, a ceiling, a threshold — cannot be screened closed-form,
because reaching is an acceleration question. Closed-form ranks
steady-state trades fine.

## 4.2 Closed-form and FP can ORDER candidates differently

Not just magnitudes. Two documented cases:

**Wing sizing.** Closed-form recommended growing the wing to span 0.78 m /
AR 4.5 (+0.056 g). Under FP the recommendation **inverts**:

| span / AR | closed-form traverse | FP traverse |
|---|---|---|
| 0.5325 / 1.86 | 0.167 | **0.183** |
| 0.65 / 2.50 | — | 0.178 |
| 0.80 / 3.00 | 0.184 | 0.153 |

The weaker FP engine cannot carry the extra profile drag and wing mass.

**Dive extension.** Closed-form said dive ≥ 22° opens gate 0.50. FP says
peak Mach is 0.464/0.468/0.470 at dive 25/28/30 — the gate is never
reached at any dive angle.

## 4.3 Grid resolution changed the answer — the campaign's headline

The pulsejet loses **3.8–5.5 %** thrust from n_cells 162 → 324, roughly
constant across the mission Mach range:

| M | alt | MARCH 162 | CONFIRM 324 | Δ |
|---|---|---|---|---|
| 0.118 | 30 m | 127.6 N | 120.5 N | −5.5 % |
| 0.200 | 250 m | 121.2 N | 115.8 N | −4.4 % |
| 0.300 | 580 m | 107.8 N | 102.9 N | −4.6 % |
| 0.350 | 500 m | 108.8 N | 104.0 N | −4.4 % |
| 0.440 | 220 m | 114.3 N | 109.6 N | −4.1 % |
| 0.500 | 150 m | 114.1 N | 109.6 N | −4.0 % |
| 0.600 | 122 m | 111.1 N | 106.8 N | −3.8 % |

The duct sustains at every condition on both grids — it is not a
flame-out, it is ~5 N.

But 5 N was decisive. Full-flight comparison:

| configuration | MARCH 162 | CONFIRM 324 |
|---|---|---|
| V3b — climb 10 / dive 9.9 / gate 0.30 | 0.389 g, climb +0.049 | **0.352 g, climb +0.036 — PASS** |
| V3c — climb 12 / dive 20 / gate 0.44 | 0.368 g, climb +0.012 | **peak M 0.257, no cutoff — FAIL** |

The V3c climb *decelerated* from M 0.118 to 0.111 over 81 s and never
reached the gate.

> ### **A margin thinner than the discretisation error of the model that produced it is not a margin.**
>
> V3c's climb margin was 0.012 g ≈ 2.7 N. Grid refinement cost ~5 N.
> V3b's 0.049 g ≈ 10.9 N absorbed the same hit.

**Rule for V4: freeze nothing into a design.json on MARCH-tier evidence.**
Any configuration whose binding margin is under ~0.02 g must be
CONFIRM-verified before it is believed. Ideally spot-check AUDIT (486) —
thrust was still falling at 400 cells, so 324 may not be converged either.

## 4.4 Propulsion time-step refinement: bias, not noise

Thrust is held constant between engine re-convergences, which fire on
drift triggers of ΔM 0.05 / Δalt 250 ft — the *validated ceiling* of the
ramjet-fp continuation campaign, not a speed choice. Refining is the safe
direction; it only costs solves.

Same configuration, n_cells 162, four step sizes:

| ΔM | Δalt | solves | traverse | climb g | peak M | fuel | wall time |
|---|---|---|---|---|---|---|---|
| 0.05 | 76.2 m | 45 | 0.3685 | +0.0115 | 1.1001 | 1.7586 | 385 s |
| 0.03 | 45.7 m | 72 | 0.3705 | +0.0078 | 1.1002 | 1.7058 | 545 s |
| 0.02 | 30.5 m | 109 | 0.3739 | +0.0011 | 1.1001 | 1.6816 | 776 s |
| 0.01 | 15.2 m | 213 | **0.3795** | **−0.0007** | 1.1002 | 1.6823 | 1514 s |

- **traverse: monotone +3.0 % — a real bias.** The coarse staircase
  *understates* traverse, so coarse traverse numbers are conservative.
- **climb margin: falls to zero and goes negative.** The coarse steps
  **flatter the climb margin by the entire margin.** This is the
  dangerous one.
- peak Mach and fuel converge by ΔM 0.02.

**Rule for V4: report climb margin only at ΔM ≤ 0.02.** At ΔM 0.05 it is
not a measurement.

## 4.5 Results are only reproducible to ±0.02 g

The FP snapshot store warm-starts each engine solve from previously
converged cycles, so the *same* configuration reached via a different
sequence of flights converges slightly differently. Measured: the same
config returned 0.376 and 0.389 from two different scripts.

**Do not rank two candidates on a 0.01 g difference.** If a tight
comparison matters, clear the snapshot store between candidates.

---

# PART 5 — MODEL DEFECTS AND GAPS FOUND (checklist for V4)

Each of these was a real bug or gap that produced a wrong answer at least
once. Check whether V4's model has the same issue.

## 5.1 Fixed during V2/V3 — confirm your model has these

| # | defect | symptom | fix |
|---|---|---|---|
| 1 | One-shot hot IC used as igniter | pulsejet never lit | sustained pilot until `t_ignite` |
| 2 | 1-D cell-averaging destroyed ignition | flame died on coarse grids | flame-brush closure (sub-grid flame-sheet consumption) |
| 3 | Base Cp discontinuous | drag jumped 2× at M 0.8 | made continuous |
| 4 | Flat annulus base, no boattail | base drag dominated | 8° attached boattail; base 71 % → 23 % of frontal |
| 5 | Spillage double-counted | nose inlet treated as podded nacelle | geometry-based recovery 0.85–0.92 |
| 6 | `inlet_area_m2 = π·D_body²/4` | 3.4× the physical duct; engine and drag models disagreed | use the actual duct area |
| 7 | Cone added as *extra* duct length | 1318 mm modelled vs 1022 mm actual | cone comes OUT of throat_length |
| 8 | Fuel burned against tank *volume* | vehicle burned fuel it never carried | burn against 90 % of **loaded** mass |

## 5.2 Open — still present, will bite V4

| # | issue | detail |
|---|---|---|
| 9 | **Stale dry mass** | `docs/v3_frozen` / `v3a` state `dry_mass_kg = 12.5696`; recomputing `vehicle_dry_mass` from that file's own geometry gives **15.4965 kg (+2.927 kg)**. Independently confirmed twice. Loaded fuel is derived as `wet − dry − margin`, so **payload margin is really 4.49 kg, not 7.42 kg** — a 40 % cut. **Verify this before sizing V4's payload.** |
| 10 | Pull-out unmodelled | §3.6. No arc, no load factor, no g limit. |
| 11 | `RAMJET_MIN_LIGHTOFF_MACH` hard-coded 0.45 | Contradicted by FP (lights at 0.25). Should be a scheduled design variable. |
| 12 | `lift.py` not wired into the flight model | New module gives e 0.702 (vs 0.847), CL_max 0.849 (vs 1.135), **V_stall 53.0 m/s (vs 45.8)** — past the 45 m/s cap. Wiring it in makes results *worse*, not better. |
| 13 | Release below stall speed | Release is 40 m/s; CL required at 40 m/s is 1.488 against CL_max 0.849, needing **34.8° α vs α_max 25°**. There is no ground-roll model. This is a real design problem, not a modelling artifact. |
| 14 | Flameholder station lands in the pulsejet cone | `x_fh = 0.28 × duct = 907 mm`, where the duct is 185 mm across, but the gutter is sized against `combustor_diameter_m` = 214 mm. Design-dependent. |
| 15 | ramjet-fp and pulsejet station maps disagree | ramjet-fp places its combustor at 0.18–0.80 × duct and throat at 0.95 × duct; the physical duct is constant-diameter tailpipe from 1055 mm to exit at 2082 mm. |
| 16 | Side valve inlets don't fit inside the skin | Scaled runner is 137 mm dia × 165 mm; chamber at 0.95 D leaves a **5.6 mm** radial gap. Needs an external fairing or a smaller chamber fraction. |
| 17 | Fuel annulus formula ~7 % optimistic | `annular_volume_m3` treats the whole tail section as throat-sized: 35.1 L vs a true 32.7 L. Doesn't bind at 2.69 kg loaded (~5.5 L) but would on a volume-limited design. |
| 18 | Return-to-launch glide stalls | Every flight, all configurations. Uniform, so it doesn't discriminate, but the return profile is not healthy. |
| 19 | CD0 bound at **import** time | `SIMPLE_MODEL_CD0_FRONTAL` / `MEDIUM_MODEL_CD0_FRONTAL` are read once at import. Setting them after import silently does nothing. CD0-sensitive comparisons must run in **subprocesses**. |

---

# PART 6 — WHAT V4 SHOULD ATTACK

Ranked by expected payoff, with the evidence behind each.

## 6.1 The pulsejet inlet — highest leverage by far

**Problem:** side-mounted valve inlets take boundary-layer air at static
pressure, so thrust is flat with Mach (§2.2). Thrust crosses drag at
M 0.47, which is why the whole V3 trajectory is contorted around using
gravity to do what the engine cannot.

**Everything downstream of this is a symptom:** the climb-dive profile
exists because of it; the M 0.47 ceiling is it; the failed powered-accel
and thrust-margin gates are it; the inability to light at M 0.6 is it.

**V4 direction:** forward-facing inlets with ram recovery. This is
explicitly noted in the project as a deliberate V2/V3 choice — revisit
whether the reason still holds. Expected payoff: moves the thrust/drag
crossover up, which relaxes the trajectory, the gate scheduling, and both
failed gates simultaneously.

## 6.2 Release velocity

Release is 40 m/s, below the ~47 m/s full-mass stall speed (53 m/s with
`lift.py`). The vehicle leaves the rail unable to fly level; there is no
ground-roll model, so the sim does not charge for it.

**V4 direction:** a faster release, a booster, or a wing that can actually
carry the vehicle at 40 m/s. Note §4.2 — a bigger wing is *not*
automatically the answer; verify under FP.

## 6.3 Drag

CD0 is 0.10 frontal-referenced as a placeholder; the build-up gives
0.14–0.40 depending on regime. Halving drag would move the thrust/drag
crossover from M 0.47 to roughly M 0.6 — which is what the "why can't we
light at M 0.6" question really needs.

Biggest contributors seen: induced drag dominates at release (61 N of
70 N total at 40 m/s), transonic wave drag dominates in the drag strip.

## 6.4 Chamber diameter — worth it only with the tail to match

+36 % FP thrust at 235 mm held to t/D 4.80, but **+188 mm length** and
**+20.7 % frontal area**. The short-duct version does not exist (§2.1).
Model it honestly as a three-way trade: thrust ↑ D³, frontal drag ↑ D²,
length ↑ with t/D.

## 6.5 What NOT to spend a campaign on

- **Trajectory tuning to fix the powered-accel or thrust-margin gates.**
  V3 ran ~450 closed-form flights and ~40 FP flights across the whole
  climb × dive × floor × gate space. Neither gate came close. They are
  engine problems.
- **Growing the wing** without FP verification (§4.2).
- **Shortening the duct.** Every attempt died (§2.1).

---

# PART 7 — HOW TO RUN A V4 CAMPAIGN IN `simple_model` WITHOUT REPEATING V3

`simple_model` is the right place to search — it is ~0.7 s/flight against
6–25 min for an FP flight. But V3 showed exactly which of its answers to
distrust. Guardrails:

1. **Screen with closed-form, decide with FP.** Never freeze a design on
   closed-form alone.
2. **Never screen a "can it reach X" question closed-form** (§4.1). Those
   need FP from the start.
3. **Any candidate that survives screening must be FP-verified at
   CONFIRM (324), not MARCH (162)** — especially if its binding margin is
   under 0.02 g (§4.3).
4. **Report climb margin only at ΔM ≤ 0.02** (§4.4).
5. **Verify pulsejet sustain at ≥ 4 operating points**, not one (§2.1).
6. **Always check `motor_cutoff_reached` and fuel-vs-cap** before
   believing any traverse number (§3.5).
7. **Don't rank on < 0.02 g differences** (§4.5).
8. **Set CD0 before import, in a subprocess** (§5.2 #19).
9. **Re-derive the mass block for any new geometry** — do not inherit
   `optimizer_results` from a parent design (§5.2 #9).
10. **Cap dive angle** at whatever the pull-out table (§3.6) supports for
    your wing, or add the arc to the phase machine.

## Useful scripts from the V3 campaign

All under `scripts/`, all take `MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=.`:

| script | purpose |
|---|---|
| `medium_model_v3b_final.py` | fly one config, dump trace, emit trajectory/fuel/engine plots. Flags: `--climb --dive --floor --lightoff --n-cells --mach-step --alt-step-m --dt --tag` |
| `medium_model_v3c_step_refinement.py` | the ΔM ladder study of §4.4 |
| `medium_model_v3c_gridconv.py` | pulsejet thrust vs n_cells across the mission profile (§4.3) |
| `medium_model_v3c_pullout.py` | pull-out load factor demanded vs available (§3.6) |
| `medium_model_v3c_latest_screen.py` | closed-form climb × dive × gate screen, with the monkeypatch trick for the gate |
| `medium_model_v3c_late_bisect.py` | FP bisection on the gate |
| `medium_model_cross_section.py` | to-scale longitudinal section PNG, assumptions marked |
| `medium_model_v3b_report.py`, `medium_model_v3c_report.py` | report generators that read the result JSONs |

**Tip that saved a lot of time:** the closed-form lightoff gate can be
swept by monkeypatching the module global — it is read at call time:

```python
import medium_model.ramjet_simple as rs
rs.RAMJET_MIN_LIGHTOFF_MACH = 0.42   # verified to work
```

## Running the test suite

The kill-switches **must** be on the command line — `tests/conftest.py` is
a *pytest* convention and `unittest discover` never loads it:

```bash
DOUGLAS_DART_DISABLE_PULSEJET_FP=1 DOUGLAS_DART_DISABLE_RAMJET_FP=1 .venv/Scripts/python -m unittest discover tests
```

Green = `Ran 152 tests ... OK (skipped=1)`, ~814 s. **Never pipe through
`tail`** — the pipeline exit code is `tail`'s and it masks failures.

---

# PART 8 — DATA FILES

Under `out_medium_model/`:

| file | contents |
|---|---|
| `v3b_gate3_report.md` | full V3b report incl. the 235 mm retraction |
| `v3c_latest_ignition_report.md` | full V3c/V3d report incl. the fidelity finding |
| `v3c_gridconv.json` | pulsejet thrust vs n_cells, 7 mission conditions |
| `v3c_step_refinement.json` / `.png` | the ΔM ladder |
| `v3c_pullout.json` | load factor demanded vs available |
| `v3c_late_bisect_round{1,2,3}.json` | ~22 FP flights, gates 0.30–0.50, dives 9.89–30 |
| `v3b_lightoff.json`, `v3b_lightoff_fine.json` | the gate tent function |
| `v3b_joint.json` | gate × climb interaction |
| `v3c_latest_screen.json` | 288-flight closed-form screen |
| `v3b_hifi_final.json`, `v3d_g*_final.json` | CONFIRM-tier flights |
| `cross_section_v3b.png` | to-scale vehicle section |

Design files: `docs/v3a_medium_model/`, `docs/v3b_medium_model/`,
`docs/v3c_medium_model/` (superseded), `docs/v3d_medium_model/`.

---

# APPENDIX — THE FIVE THINGS MOST LIKELY TO MISLEAD A V4 CAMPAIGN

1. **A sustain check at one operating point.** The 235 mm duct made
   153 N at launch and 0.94 N everywhere else. The `t/D ≥ 4.5` rule has
   the same weakness — it was measured at M 0.15 only.
2. **A margin smaller than the model's own discretisation error.** V3c
   passed at MARCH and failed outright at CONFIRM on a 0.012 g margin.
3. **A closed-form screen of a "can it reach X" question.** The haircut
   ratio collapses from 0.78 to 0.05 near a boundary.
4. **An inherited `optimizer_results` block.** The V3a dry mass is stale
   by 2.93 kg and silently changes the fuel budget and payload margin of
   every descendant.
5. **A traverse number from a flight that never reached cutoff.** Shallow
   climbs produce good-looking traverse on flights that pinned the fuel
   cap at M 0.77.
