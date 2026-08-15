# AGENT 1

## Answer: dive 20 deg is NOT the bound, and the dive IS flyable — but only if you stop flying the dive at 122 m

Two new files, no existing file touched:
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3d_dive_extension.py` → `out_medium_model/v3d_dive_extension.json`
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3d_pullout_closed.py` → `out_medium_model/v3d_pullout_closed.json`
- (plus `scripts\medium_model_v3d_pullout_flyability.py` → `out_medium_model/v3d_pullout_flyability.json`, the fine gate scan)

---

## 1. Dive extension — the 0.45 wall was an artifact of stopping at dive 20

**min_traverse_accel_g, floor 121.92 m, closed-form, drag build-up.** `PASS` = traverse ≥ 0.26 AND peak M ≥ 1.0 with cutoff AND fuel < 2.4239 kg.

| gate | climb | d20 | d22 | d25 | d28 | d30 |
|---|---|---|---|---|---|---|
| **0.45** | 12 | 0.374 P | 0.405 P | 0.451 P | 0.497 P | **0.527 P** |
| 0.45 | 14 | 0.380 P | 0.413 P | 0.459 P | 0.505 P | **0.535 P** |
| 0.45 | 16 | 0.348 P | 0.377 P | 0.412 P | 0.439 P | 0.463 P |
| **0.50** | 12 | 0.259 **F** | 0.292 P | 0.338 P | 0.386 P | **0.415 P** |
| 0.50 | 14 | 0.184 F | 0.200 F | 0.224 F | 0.250 F | 0.262 P |
| 0.50 | 16 | 0.063 F | 0.073 F | 0.085 F | 0.101 F | 0.105 F |
| 0.55 | all | F | F | F | F | F |

**Gate 0.50 opens at dive ≥ 22 deg.** The v3c screen called 0.50 a hard wall because it never went past dive 20 — at dive 20 it misses by 0.001 g (0.259 vs 0.260).

Supporting columns, climb 12 (peak M and cutoff identical across the row):

| gate | dive | traverse | pow-g | thrust margin | peak M | cutoff | fuel kg | top m |
|---|---|---|---|---|---|---|---|---|
| 0.45 | 20 | 0.374 | +0.026 | 1.055 | 1.101 | yes | 1.833 | 772 |
| 0.45 | 25 | 0.451 | +0.016 | 1.033 | 1.100 | yes | 1.832 | 834 |
| 0.45 | 30 | 0.527 | +0.008 | 1.017 | 1.101 | yes | 1.833 | 881 |
| 0.50 | 22 | 0.292 | −0.015 | 0.867 | 1.101 | yes | 1.994 | 1072 |
| 0.50 | 30 | 0.415 | −0.020 | 0.863 | 1.101 | yes | 1.966 | 1131 |
| 0.55 | 30 | −0.220 | −0.220 | 0.719 | **0.558** | **no** | **2.424 (capped)** | 1510 |

**Fine gate scan, climb 12, floor 122:** dive 25 passes to **0.51** (0.316), fails 0.52 (0.197). Dive 30 passes to **0.51** (0.362), fails 0.52 (0.238). So the uncharged ceiling is **M 0.51, up from 0.45.**

**0.55 is a genuine wall, not a dive-angle problem.** Peak Mach moves only 0.553 → 0.558 across dive 20→30 while the tank empties into the 2.424 kg cap. The pulsejet alone cannot push past ~M 0.55; more dive angle does not touch it.

**Caveat I am obliged to flag:** the engine-only thrust margin is **below the 1.15 house rule for every row here** — 1.06 at best (gate 0.45, dive 20) and 0.83–0.87 at gate 0.50. Margin < 1.0 means the vehicle is climbing on kinetic energy, not on thrust. That gate was not in the three you specified, so nothing above is failed on it, but gate 0.50 buys its lateness partly by letting the climb decelerate.

---

## 2. Flight-path-angle rule (γ ≥ 0 from M 0.80 to cutoff): **PASS, every candidate, unanimously**

`flight_sim.py` tracks it at lines 783–786 (`V3_RULE_MACH_LO=0.80`, `HI=1.10` at lines 225–226) → `FlightResult.rule_violated` / `.rule_violation_mach`. I did not trust the flag: I re-derived γ from the integrated altitude trace (`asin((dh/dt)/V)` per powered step in the Mach window). Both agree.

| candidate | `rule_violated` | traced min γ in M 0.80–1.10 |
|---|---|---|
| all 45 rows of part 1 | `False` | **+1.00 deg** |

**It cannot be violated by construction, and that is worth knowing.** `dive_end_mach = 0.60` < `V3_RULE_MACH_LO = 0.80`, so the dive is always latched shut before the rule window opens. Measured dive exits: M 0.542 (gate 0.45, dive 30), M 0.539 (gate 0.50, dive 30), M 0.538 (gate 0.50, dive 25) — the floor binds before M 0.60 does. The drag strip then flies at `climb_angle_deg = +1.0`, and altitude at M 0.80 is *above* the dive exit (220 m vs 200 m floor; 326 m vs 300 m). **No candidate is disqualified on the rule. The rule is not the discriminator here — the pull-out is.**

---

## 3. The pull-out: the wing is not the problem, the induced drag is

**First, what the model does.** The phase machine (lines 765–780) latches `v3_dive_done` and γ jumps from −30 deg to +1 deg **in one 0.02 s timestep**. Zero radius, zero altitude, zero time — **infinite g demanded, and nothing charged**. Lift in every drag call is `m·g·cos(γ)`, so the load factor never exceeds 1 either. Task #38 is real: the pull-out is free in this model.

**Condition at the bottom of the dive** (climb 12, floor 122): M 0.528–0.546, V 179.5–185.5 m/s, alt 122–123 m, mass 21.70–21.88 kg, q 19.5–20.8 kPa.

**g available** — `lift.wing_clmax()` on the 1525 cm² / AR 1.86 wing:

| | value |
|---|---|
| CL_max at M 0.54 | **0.886** |
| mechanism | **alpha-limited @ 25 deg** (not section stall), `trustworthy=True` |
| L_max | ~2690 N vs W ~213 N |
| **n_available** | **12.3 – 13.2 g** |

**g demanded**, constant-n arc from −γd to level, `R = dh/(1−cos γd)`, `n = 1 + V²/(gR)` (worst case at the bottom):

| dive | dh = 28 m | 78 m | 128 m | 178 m | 278 m |
|---|---|---|---|---|---|
| 20 deg | 8.09 g | 3.56 | 2.56 | 2.12 | 1.71 |
| 25 deg | 12.35 g | 5.07 | 3.48 | 2.77 | 2.12 |
| 28 deg | 15.35 g | 6.15 | 4.13 | 3.24 | 2.41 |
| 30 deg | **17.58 g** | 6.94 | 4.61 | **3.58** | 2.62 |

**The wing wins that comparison easily** — at dive 30 / dh 178 m it needs 3.58 g of 12.7 available, alpha 5.6 deg of 25. **What kills the tight pull-outs is the induced drag of holding the g**, which scales n²:

| dive 30, dh | n | ΔD_induced | as decel | arc duration | along-path accel in the arc |
|---|---|---|---|---|---|
| 28 m | 17.58 | 1078 N | **−5.08 g** | 0.59 s | **−3.63 g** |
| 78 m | 6.94 | 167 N | −0.785 g | 1.66 s | +0.644 g |
| 128 m | 4.61 | 78 N | −0.342 g | 2.72 s | +1.068 g |
| 178 m | 3.58 | 47 N | −0.203 g | 3.80 s | +1.171 g |

The arc also **gives back gravity**: mean assist over the arc is `g(1−cos γd)/γd` = 0.256 g at 30 deg, versus 0.500 g in the steady dive. Both charges are absent from the model.

### The self-consistent test — floor raised so the ARC bottoms at 121.92 m

Re-flown, not post-processed: `floor_altitude_m = 121.92 + dh`, effective gate = `min(sim traverse, arc accel)`.

| gate | dive | dh | dive ends at | sim traverse | n req / avail | α req | arc accel | **effective traverse** | verdict |
|---|---|---|---|---|---|---|---|---|---|
| 0.45 | 30 | 28 m | 150 m | 0.523 | 17.6 / 12.95 | 27.0° | −3.634 | −3.634 | **g-SHORT** |
| 0.45 | 30 | 78 m | 200 m | 0.517 | 6.94 / 12.87 | 10.7° | +0.644 | **0.517** | PASS |
| 0.45 | 30 | 128 m | 250 m | 0.511 | 4.61 / 12.79 | 7.2° | +1.068 | **0.511** | PASS |
| 0.45 | 30 | 178 m | 300 m | 0.508 | 3.58 / 12.65 | 5.6° | +1.171 | **0.508** | PASS |
| 0.50 | 30 | 28 m | 150 m | 0.412 | 17.7 / 13.14 | 26.8° | −4.346 | −4.346 | **g-SHORT** |
| 0.50 | 30 | 78 m | 200 m | 0.408 | 6.96 / 13.00 | 10.7° | **−0.071** | −0.071 | **a-SHORT** |
| 0.50 | 30 | 128 m | 250 m | 0.404 | 4.61 / 12.87 | 7.1° | +0.344 | **0.344** | PASS |
| 0.50 | 30 | 178 m | 300 m | 0.374 | 3.57 / 12.69 | 5.6° | +0.442 | **0.374** | PASS |
| 0.51 | 30 | 128 m | 250 m | 0.288 | 4.63 / 12.97 | 7.1° | +0.218 | 0.218 | fail |
| 0.51 | 28 | 128 m | 250 m | 0.271 | 4.16 / 12.91 | 6.9° | +0.252 | 0.252 | fail |
| 0.52 | 30 | 128 m | 250 m | 0.163 | 4.65 / 13.08 | 7.1° | +0.094 | 0.094 | fail |

32 of 50 survive. **Charging the pull-out costs exactly one notch of gate: 0.51 → 0.50.**

**Best flyable candidate: climb 12 / dive 30 / gate 0.50, straight dive terminating at 300 m, arc bottoming at 122 m.** Effective traverse 0.374 g, pull-out 3.6 g of 12.7 available (3.5× margin), α 5.6 deg of 25, arc lasts 3.8 s, fuel 2.096 kg of 2.424, peak M 1.100 with cutoff, rule PASS.

**Plainly: the steep dives ARE flyable — but not down to 122 m.** Dive 28–30 deg needs 128–178 m of arc, so the straight dive must stop at 250–300 m. Ending the straight dive at 150 m (dh 28 m) is aerodynamically impossible at dive 28–30 (needs 15–18 g, wing gives 13) and dynamically useless at dive 20–25 (the induced drag decelerates the vehicle harder than the engine accelerates it).

---

## 4. Floor sensitivity (climb 12, dive 30, gate 0.50) — cheap, then a knee at 300 m

| floor (where the straight dive ends) | traverse | pow-g | margin | peak M | fuel kg | top m |
|---|---|---|---|---|---|---|
| **121.9** | 0.415 | −0.020 | 0.863 | 1.101 | 1.966 | 1131 |
| **150.0** | 0.412 | −0.023 | 0.857 | 1.101 | 1.986 | 1164 |
| **200.0** | 0.408 | −0.028 | 0.850 | 1.100 | 2.021 | 1222 |
| 250 | 0.404 | −0.032 | 0.843 | 1.100 | 2.057 | 1281 |
| 300 | 0.374 | −0.036 | 0.833 | 1.100 | 2.096 | 1339 |
| 400 | 0.305 | — | — | 1.100 | 2.173 | ~1450 |

122 → 200 m costs 0.007 g and 55 g of fuel — nothing. The knee is at 300 m (traverse −0.030) and it bites hard at 400 m. **Buying the 128–178 m the pull-out needs is affordable; buying 278 m is not.**

---

## What I did NOT test

- **No FP flights at all.** Everything is closed-form. At the stated ~22% FP derate the winner (0.374 eff) lands near **0.29 g** — above 0.26 but thin. Gate 0.45 / dive 30 / dh 178 (0.508) derates to ~0.40 and is the safe choice. FP verification of the 0.50 candidate is the obvious next run.
- **No structural check on 3.6–4.6 g.** There is no structural model in this repo. I reported only what the *aerodynamics* can deliver.
- **The arc is post-processed, not integrated.** Constant n, evaluated at the dive-exit state; the velocity loss during the 2.7–3.8 s arc is not fed back into the trajectory. Only the wing induced-drag increment is charged — **body alpha drag and trim drag are not**, so the arc numbers are optimistic in that direction.
- **Only climb 12 carried through the closed pull-out test.** Climbs 14/16 appear in part 1 only (climb 12 dominated at gate 0.50; climb 14 dominated marginally at gate 0.45). Climbs below 12 were not run.
- **No dive beyond 30 deg**, and no dive/floor combination outside `dh ∈ {28, 78, 128, 178, 278}`.
- **Vortex lift not credited** in CL_max (`vortex_lift_kv=0.0`, lift.py's conservative default) — with Kv≈3.0 the wing would gain ~0.45 CL and n_available would rise ~50%. It does not matter here; the wing was never the binding constraint.

# AGENT 2

All data in. Here are the findings.

## VERDICT: the stated hypothesis is REFUTED

The vehicle **does** reach M 0.50 and the ramjet **does** light. Every one of the 36 gate-0.50 flights reaches peak Mach 1.100 with `motor_cutoff_reached=True`. Nothing is stranded. What fails is the **0.26 g traverse gate**, and it fails through two different mechanisms in different corners of the climb x dive box.

---

## 1. Thrust / drag / gravity balance, V3a duct, pulsejet ALONE

Specific excess `e = (T_pulsejet - D_vehicle)/W`, in g. Dive acceleration is exactly `a = e + sin(gamma_dive)`. W = 216.1 N (mid-dive mass 22.04 kg, *measured* from the passing flight, not the 222 N wet figure). Drag is the build-up at dive attitude.

**Altitude 300 m** (where the dive actually crosses M 0.45-0.50):

| M | T_cf (N) | T_fp (N) | FP/CF | D (N) | e_cf | e_fp | a_fp @dive10 | a_fp @dive20 |
|---|---|---|---|---|---|---|---|---|
| 0.30 | 127.9 | 116.5 | 0.911 | 55.9 | +0.333 | +0.280 | +0.454 | +0.622 |
| 0.35 | 124.7 | 114.9 | 0.921 | 69.4 | +0.256 | +0.210 | +0.384 | +0.552 |
| 0.40 | 121.6 | 113.3 | 0.932 | 85.9 | +0.165 | +0.127 | +0.300 | +0.469 |
| 0.45 | 118.4 | 111.6 | 0.942 | 104.9 | +0.063 | +0.031 | +0.205 | +0.373 |
| **0.50** | 115.3 | 109.8 | 0.952 | 126.1 | **-0.050** | **-0.076** | +0.098 | +0.266 |
| 0.55 | 112.1 | 107.9 | 0.963 | 149.6 | -0.173 | -0.193 | -0.019 | +0.149 |
| 0.60 | 108.9 | 106.0 | 0.973 | 175.0 | -0.305 | -0.319 | -0.145 | +0.023 |

**Altitude 900 m** (where gate 0.50/0.55 push the profile):

| M | T_cf (N) | T_fp (N) | D (N) | e_cf | e_fp | a_fp @dive10 | a_fp @dive20 |
|---|---|---|---|---|---|---|---|
| 0.30 | 105.9 | 97.0 | 53.6 | +0.242 | +0.201 | +0.374 | +0.543 |
| 0.40 | 100.6 | 94.1 | 81.2 | +0.090 | +0.060 | +0.233 | +0.402 |
| 0.45 | 98.0 | 92.5 | 98.8 | -0.004 | -0.029 | +0.144 | +0.313 |
| 0.50 | 95.4 | 90.8 | 118.6 | -0.107 | -0.129 | +0.045 | +0.213 |
| 0.60 | 90.2 | 87.2 | 164.2 | -0.342 | -0.356 | -0.183 | -0.014 |

The engine is nearly Mach-flat and the drag is not: FP pulsejet loses only **10.5 N over M 0.30 → 0.60** while drag gains **119 N**. `d(T-D)/dM = -486 N per unit Mach` at 300 m. Altitude costs **-19.5 N per 600 m** (FP), i.e. `dM(T=D)/dh = -0.015 Mach per 300 m`.

## 2. Predicted ceilings

| altitude | model | M where a goes negative (dive 10) | (dive 20) | **M where a = 0.26 g (dive 10)** | **(dive 14)** | **(dive 20)** | M(T=D) |
|---|---|---|---|---|---|---|---|
| 130 m | cf | — | — | 0.449 | 0.479 | 0.521 | 0.487 |
| 130 m | fp | — | — | 0.431 | 0.464 | 0.509 | 0.472 |
| 300 m | cf | 0.550 | 0.614 | 0.439 | 0.470 | 0.513 | 0.478 |
| 300 m | fp | 0.542 | 0.609 | 0.422 | 0.456 | 0.503 | 0.465 |
| 600 m | fp | — | — | 0.404 | 0.441 | 0.491 | 0.451 |
| 900 m | cf | 0.529 | 0.600 | 0.402 | 0.439 | 0.488 | 0.448 |
| 900 m | fp | 0.520 | 0.594 | 0.384 | 0.424 | 0.477 | 0.434 |

**The "goes negative" ceiling is not the constraint** — it sits at M 0.54 (dive 10) / M 0.61 (dive 20), far above any gate that fails. The **0.26 g gate** is what caps things, at **M 0.42-0.45 for dive 10** and **M 0.50-0.52 for dive 20**.

## 3. Cross-check against the closed-form screen — it matches

| dive | screen's latest passing gate | my predicted 0.26 g ceiling | agreement |
|---|---|---|---|
| 9.89 deg | 0.40 | 0.42-0.44 (at 130-300 m, cf) | yes — true boundary in (0.40, 0.45] |
| 14-20 deg | 0.45 | 0.47 (dive 14) / 0.51 (dive 20), cf | yes for dive 14; **dive 20 over-predicts** |

Dive 20 over-predicts because a **second ceiling** binds there. Measured min-traverse locations, dive 20:

| climb | gate 0.45 | gate 0.50 | where the gate-0.50 min sits |
|---|---|---|---|
| 6 | 0.348 | 0.239 | M 0.500, v3_dive — thrust balance |
| 10 | 0.367 | 0.253 | M 0.500, v3_dive — thrust balance |
| 12 | 0.374 | **0.259** | M 0.500, v3_dive — thrust balance |
| 14 | 0.380 | 0.184 | M 0.524, **drag_strip** — ran out of altitude |
| 16 | 0.348 | 0.063 | M 0.510, **drag_strip** — ran out of altitude |

The second ceiling is the **altitude budget**. `derive_top_altitude` required top-of-climb diverges with the gate, while the vehicle's own pulsejet service ceiling in the commanded climb (`e(M,h) = sin(gamma_climb)`, M 0.28) is fixed:

| gate | required top, dive 20 (m) | | attainable top (m) | cf | fp |
|---|---|---|---|---|---|
| 0.35 | 509 | | climb 6 | 2153 | 1654 |
| 0.40 | 610 | | climb 10 | 1520 | 1159 |
| 0.45 | 772 | | climb 14 | **998** | 667 |
| 0.50 | **1053** | | climb 16 | **769** | 424 |
| 0.55 | 1603 | | | | |
| 0.60 | 3720 (capped) | | | | |

Required crosses attainable at gate ~0.449 for climb 16 and ~0.49 for climb 14 — exactly where those two columns collapse. Gate 0.55 needs 1603 m against a ~1000 m attainable top: the profile does not exist, which is why the vehicle never leaves M 0.55 (peak Mach 0.551, traverse -0.22 g). That is the genuinely hard part of the wall.

## 4. The ramjet is NOT the limit — confirmed by FP

| M | alt | viable | net thrust (N) | phi (output) | mdot_air (kg/s) |
|---|---|---|---|---|---|
| 0.25 | 300 | True | 68.7 | 0.831 | 0.4289 |
| 0.30 | 300 | True | 97.2 | 0.839 | 0.5058 |
| 0.45 | 300 | True | 205.4 | 0.863 | 0.7529 |
| 0.50 | 900 | True | 232.7 | 0.878 | 0.7666 |

Cold light at M 0.25 (matches the brief's 66.8 N / phi 0.831). The ramjet would light 0.20 Mach earlier than the wall. **Refuted that the ramjet limits anything.**

## 5. A real modelling artifact sharpens the wall — worth acting on

The min traverse is pinned to **one timestep**: the instant M crosses the gate. `flight_sim._duct_swallowed_kg_per_s` switches the duct from cold-throughflow to throat-limited the moment `ramjet_result.net_thrust_n > 0`, but `ramjet_simple` ramps thrust linearly over `RAMJET_LIGHTOFF_RAMP_MACH = 0.10`. So the vehicle pays **full lit-duct spillage drag for ~zero thrust**:

| gate | drag step at crossing | ramjet thrust delivered there | step in g | measured min traverse | traverse without the step |
|---|---|---|---|---|---|
| 0.40 | +4.80 N | 0.10 N | 0.0206 | 0.4766 | 0.4973 |
| 0.45 | +5.97 N | 0.25 N | 0.0263 | 0.3742 | 0.4005 |
| 0.50 | **+7.27 N** | 0.19 N | **0.0328** | **0.2588** | **0.2916** |

Gate 0.50 at climb 12 / dive 20 misses by **0.0012 g** — 0.5%. The artifact is worth 27x that. Remove it and gate 0.50 passes at climb 6/10/12 with dive 20 (0.273 / 0.287 / 0.292 g). **The 0.45 wall is, at its best corner, decided by this step; the 0.55 wall is not (deficit 0.48 g).** The FP path has no such artifact — `FpPropulsion` only restricts the duct once the flame actually holds with real thrust — so an FP screen may well place the wall at 0.50, not 0.45.

## Mechanism, transferable to a different duct

The pulsejet is nearly Mach-flat (side inlets, no ram recovery) while vehicle drag rises as roughly M-squared, so the specific excess `e(M,h) = (T-D)/W` falls almost linearly, at about **-486 N per unit Mach** and **-19.5 N per 600 m of altitude**, crossing zero at **M(T=D) ~ 0.47 (FP, 300 m)**. Everything above that Mach must be bought with altitude, at `sin(gamma_dive)` g per unit height, out of a bank capped below by the 400 ft floor and above by the vehicle's own service ceiling `e(M~0.28, h) = sin(gamma_climb)`. The gate ceiling is therefore `min` of two crossings: the **thrust-balance ceiling**, where `e = 0.26 - sin(gamma_dive)` (about `M(T=D) - 0.04` for a 10 deg dive, `M(T=D) + 0.04` for a 20 deg dive), and the **altitude-budget ceiling**, where the dive height needed to cross M 0.35 to the gate exceeds `h_service(gamma_climb) - 122 m`. To predict a new duct's ceiling you need only its thrust curve: compute `M(T=D)` at 300 m, add `sin(gamma_dive) - 0.26` divided by the local slope `|d e/dM| ~ 2.25 per Mach`, then check that `h_service` clears the dive requirement. To *buy* ceiling, the exchange rate is **+24 N of pulsejet thrust per +0.05 Mach of gate** (equivalently +0.112 g of excess) — the established chamber-235 mm / t-D 4.00 option at +25.6 N is almost exactly one such step, and it raises `h_service` at the same time, which is the arm that kills gate 0.55.

## What I did NOT test

- **No full FP flights** (per instruction). FP thrust came from 11 engine-level `pulsejet_thrust` queries at 300 m and 900 m only, fitted bilinearly; the data is near-linear in both variables so this is a description, not smoothing. Whether a full FP flight (~22% lower traverse) moves the wall is untested.
- I did **not** re-fly with the spillage step removed — that would mean editing `flight_sim.py`, which the brief forbids. The 0.2916 g figure is measured min-traverse plus the analytically-sized step, not a re-flown result.
- I did **not** sweep gates between 0.45 and 0.50 (0.46/0.47/0.48) to locate the true boundary, nor dives above 20 deg, nor floors below 400 ft, nor the chamber-235 mm duct in a flight.
- **Unquantified inconsistency found in passing:** `derive_top_altitude` sizes the climb using `_concept_drag` (the flat CD0), not the build-up the flight actually integrates. At M 0.50 / 304 m the build-up's body terms are 99.5 N against the flat CD0's 68.1 N — 46% higher. The derived top is therefore systematically under-bought, which makes the altitude-budget arm worse than the sizing pass believes. I did not re-derive the top-of-climb ladder with consistent drag.

Files created (all new, nothing existing modified):
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_wall_diag.py`
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_wall_fp.py`
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_wall_balance.py`
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_wall_ceiling.py`
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_wall_rjcheck.py`
- `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_wall_step.py`

Data written to `C:\Users\dougl\Documents\GitHub\Douglas_Dart\out_medium_model\v3c_wall_{diag,fp,balance,ceiling,rjcheck}.json` plus `v3c_wall_fp.log` / `v3c_wall_rjcheck.log`.

# AGENT 3

# Does a bigger pulsejet raise the M 0.45 ceiling?

**Yes, but only by +0.02 Mach — and the "0.45 hard wall" was partly a grid artifact.** Detail below.

## 1. Design file

`C:\Users\dougl\Documents\GitHub\Douglas_Dart\docs\v3c_235mm\design.json`
Built by `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_235mm_build.py` (asserts every derived quantity).

| quantity | V3a | 235 mm | note |
|---|---|---|---|
| chamber dia | 214.0 mm | 235.0 mm | +9.8%, inside the +10% cap |
| body dia | 225.217 mm | **247.368 mm** | = 235/0.95, verified |
| throat dia | 121.617 mm | **133.579 mm** | body x 0.5400 |
| throat area frac | 0.29169 | **0.29160** | under `PULSEJET_MAX_THROAT_AREA_FRACTION` = 0.30 |
| chamber_length_m | 0.389307 | 0.389307 | held |
| throat_length_m | 1.24230 | **1.1370968** | = 0.940/(1−0.173333) |
| cone / tailpipe | — | 197.10 / 940.00 mm | cone fraction 0.130/0.750, **verified against V3a** (its cone/tail = 0.17331) |
| duct | 1631.6 mm | **1526.4 mm** | |
| body | 2307.3 mm | **2268.5 mm** | −38.8 mm |
| fineness | 10.245 | **9.171** | |
| frontal area | 398.38 cm² | **480.59 cm²** | **+20.64%** |

Every number in the task brief reproduced exactly.

## 2. The mass block — worked, not copied

The mass model **is** standalone-evaluable. Reconstructed chain from `simple_model/optimize.py` L361–388: `fuel_loaded = 1.25 x burned` → `dry = vehicle_dry_mass(geom, S_wing, fuel_loaded)` → `margin = 22.6796 − dry − fuel_loaded`. The wing reference area it feeds in is the **wing concept's own `span²/aspect_ratio` = 0.152499 m²**, *not* `wingspan_m²/WING_ASPECT_RATIO`. Proof: that choice reproduces `docs/v3_frozen` `dry_mass_kg = 12.569615400692696` to **0.00e+00 kg** (asserted in the build script).

**Finding you should know about: V3a's own `optimizer_results` block is stale.** It was copied verbatim from `docs/v3_frozen` when the tail was extended 553 mm (`throat_length_m` 0.6892 → 1.2423). V3a's real dry mass at its own geometry is **15.4965 kg, not the 12.5696 kg its design.json claims** (+2.93 kg). Its margin is 4.4899 kg, not 7.4167.

New block, at loaded fuel 2.693257 kg:

| term | V3a-geometry-consistent | 235 mm | Δ |
|---|---|---|---|
| engine_duct_kg | 5.8883 | 6.1209 | +0.2326 |
| airframe_skin_kg | 5.2892 | 5.7119 | +0.4227 |
| wing / avionics / tank / landing | 0.9150 / 2.0 / 0.9040 / 0.5 | identical | 0 |
| **dry_mass_kg** | 15.4965 | **16.1517** | +0.6552 |
| **mass_margin_kg** | 4.4899 | **3.8346** | −0.6553 |

Duct wall is at the 1 mm min-gauge floor on both, so it does not scale with diameter here.

**Stated assumption:** loaded fuel is *held* at V3a's 2.693257 kg rather than re-derived from a flight. Reasons: (a) it makes the burn cap **identical (2.423931 kg)** on both vehicles, so the screen compares engines and drag, not tanks; (b) it is not volume-limited on either (annulus holds 6.91 / 7.63 kg usable). The block is internally closed: `loaded_fuel_kg(dry, margin)` returns 2.693257 exactly, and `vehicle_dry_mass(this geometry, 2.693257)` returns 16.1517 exactly. Margin is positive, so the design does not bust the 50 lb budget.

## 3. Screen result — and a confound I had to remove

`scripts/medium_model_v3c_235mm_screen.py` (your specified grid, 0.05 gate steps, dive ≤ 20), 120 flights per vehicle, both flown in the same process:

| climb \ dive | 14 | 16 | 18 | 20 |
|---|---|---|---|---|
| **V3a** 8–16° | 0.45 | 0.45 | 0.45 | 0.45 |
| **235 mm** 8, 10° | 0.45 | 0.45 | 0.45 | **0.50** |
| **235 mm** 12, 14, 16° | 0.45 | 0.45 | **0.50** | **0.50** |

That reproduces the established V3a 0.45 wall exactly, and looks like +0.05. **It isn't.** Refining to 0.01 gate steps and extending dive to 26° (`..._refine.py`, `..._control.py`, both vehicles on the identical grid) shows:

| dive | V3a ceiling | 235 mm ceiling | Δ |
|---|---|---|---|
| 18° | 0.48 | 0.50 | +0.02 |
| 20° | 0.49 | 0.51 | +0.02 |
| 22° | 0.51 | 0.52 | +0.01 |
| 24° | 0.52 | 0.53 | +0.01 |
| 26° | 0.52 | **0.54** | +0.02 |

Two corrections to the established facts: **V3a's true ceiling at dive ≤ 20 is 0.49, not 0.45** — the 0.05 grid rounded it down — and **it is not a hard wall**; dive angle keeps buying lateness out to 26°, for both vehicles. The pulsejet is worth **+0.01 to +0.02**; an extra 8° of dive is worth **+0.04**.

## 4. Peak Mach, fuel, traverse (climb 12, dive 20)

| gate | V3a traverse | V3a peak M | V3a fuel | 235 traverse | 235 peak M | 235 fuel |
|---|---|---|---|---|---|---|
| 0.45 | 0.374 PASS | 1.101 | 1.833 | **0.426** PASS | 1.100 | **1.769** |
| 0.48 | 0.306 PASS | 1.101 | 1.917 | **0.350** PASS | 1.101 | **1.847** |
| 0.49 | 0.283 PASS | 1.100 | 1.956 | **0.322** PASS | 1.100 | **1.880** |
| 0.50 | 0.259 *fail* | 1.100 | 2.003 | **0.294 PASS** | 1.101 | **1.923** |
| 0.51 | 0.235 *fail* | 1.100 | 2.065 | **0.269 PASS** | 1.101 | **1.973** |
| 0.52 | 0.136 *fail* | 1.101 | 2.147 | 0.241 *fail* | 1.100 | 2.031 |

Peak Mach hits 1.100 with cutoff in every passing cell — **the traverse gate is what binds, never peak Mach and never fuel** (worst case 2.199 kg vs the 2.424 cap). The 235 mm burns 3–4% less fuel at every matched gate. Best cell at each ceiling: V3a M 0.52, climb 10/dive 26, traverse 0.300, fuel 2.110; 235 mm M 0.54, climb 14/dive 26, traverse 0.285, fuel 2.097.

## 5. Why the +20.6% thrust only buys +0.02

The closed-form pulsejet ratio is **1.2064 at every Mach** — and it matches the FP cliff measurement (149.2/123.6 = 1.207) to 0.03%, so the screen is driven by the same physics that motivated the design. But thrust is choked-flow ∝ throat area, and throat/body is held at 0.5400, so **thrust ∝ D² — exactly the same scaling as the frontal drag reference**. Both ratios are 1.2064.

Drag build-up at M 0.50, 300 m, 22.68 kg:

| term | V3a | 235 mm | ratio |
|---|---|---|---|
| friction | 70.61 | 76.44 | 1.083 |
| form | 5.75 | 7.70 | 1.339 |
| base | 23.03 | 27.78 | 1.206 |
| spillage | 13.32 | 15.93 | 1.196 |
| wing profile + induced | 27.24 | 27.24 | 1.000 |
| **total drag** | **139.94** | **155.10** | **1.108** |
| pulsejet thrust | 115.26 | 139.05 | 1.206 |
| **net** | **−24.68** | **−16.05** | 35% less deficit |

The design wins only because wing profile + induced drag (27.2 N, 19% of total) don't scale with diameter, so total drag grows 10.8% against 20.6% more thrust.

**Isolation experiment** (`..._thrust_isolation.py` — V3a airframe with `flight_sim.pulsejet_thrust` wrapped to scale thrust *and* fuel flow together, so Isp is preserved; no drag, mass, or length penalty):

| case | dive 18 | dive 20 | dive 26 | best |
|---|---|---|---|---|
| V3a, stock pulsejet | 0.48 | 0.49 | 0.52 | 0.52 |
| V3a, pulsejet **x1.2064 free** | 0.52 | 0.54 | 0.57 | **0.57** |
| V3a, pulsejet x1.5 free | 0.58 | 0.57 | 0.57 | 0.58 |
| V3a, pulsejet x2.0 free | 0.59 | 0.59 | 0.59 | 0.59 |
| **235 mm (thrust bought with diameter)** | 0.50 | 0.51 | 0.54 | **0.54** |

The *same* 20.64% thrust delivered for free is worth **+0.05**; bought with diameter it delivers **+0.02**. Frontal-area drag eats ~60% of it. And thrust saturates hard — doubling it only reaches 0.59, because past that the ramjet's own low-Mach thrust, not the pulsejet, is what fails.

## Verdict

More pulsejet thrust does buy later ignition, but **buying it with diameter is a poor exchange rate** and it is not what is holding the gate back.

- +21% pulsejet, bought the only way available (bigger chamber → bigger body): **+0.02 Mach**, 0.49 → 0.51 at dive 20, 0.52 → 0.54 at dive 26.
- Costs 0.66 kg of dry mass (margin 4.49 → 3.83 kg) and 21% more frontal area.
- Pays back 3–4% less fuel and +0.03–0.05 g traverse at matched gate, and the body is 39 mm shorter.
- **Dive angle is 2–4x the better lever** and is free. Going from dive 20° to 26° on the *unchanged* V3a buys +0.03; the pulsejet buys +0.02 for a kilogram.
- Thrust saturates: even a free x2.0 pulsejet only reaches 0.59, so no pulsejet growth path gets past ~0.6.

Recommendation: keep the 235 mm duct if you want it for its length and fuel benefit, but **spend the search on dive angle (and the floor), not on engine size**, and re-examine whether 26° is really the ceiling — nothing in the data says it is.

## What I did NOT test

- **No FP flight at all.** Everything above is closed-form, which runs ~22% high on traverse. The +0.02 is well inside that offset, so FP could erase it. The FP thrust *ratio* is confirmed (1.207 vs 1.2064) but the FP *level* is not.
- **No FP sustain check on this exact duct at flight Mach.** The 149.2 N figure is from the stored `v3b_cliff.json` at M 0.15 / 60 m only. The cliff at t/D 3.60 dead / 3.70 alive was measured at that one condition; whether t/D 4.00 still sustains at M 0.5+ and altitude is untested.
- **The thrust margin gate is violated throughout.** `min_powered_thrust_margin` is 0.79–0.95 at every late-gate cell, against CLAUDE.md's ≥ 1.15, and `min_powered_accel_g` goes negative (−0.03). Adding that gate kills **every** cell at dive ≥ 20 on both vehicles; the 235 mm survives only at dive 18 / gate 0.44. This is pre-existing in the V3c campaign (the established 0.45 cell has margin 1.06), not introduced here — but the late-ignition regime as a whole does not meet the full constraint stack.
- **Changes are bundled.** The 235 mm vehicle changes chamber diameter, throat diameter, tail/chamber ratio (4.80 → 4.00) and body length at once. The isolation run separates thrust from airframe, but not t/D from diameter.
- **No dives past 26°, no climbs past 18°, no floor-altitude sweep, no wing resize.** The wing is V3a's unchanged, which is why wing drag stays constant and flatters the bigger body slightly.
- **Loaded fuel was held, not re-derived from a flight of this vehicle.** A self-consistent `1.25 x burned` fixed point would give the 235 mm a *smaller* allocation (it burns less), i.e. a tighter cap — but fuel never binds, so it would not change the answer.

Scripts (all new): `scripts/medium_model_v3c_235mm_build.py`, `..._screen.py`, `..._refine.py`, `..._control.py`, `medium_model_v3c_thrust_isolation.py`. Data: `out_medium_model/v3c_235mm_screen.json`, `v3c_235mm_refine.json`, `v3c_235mm_control.json`, `v3c_thrust_isolation.json`. No existing file was modified.

# AGENT 4

# VERDICT: **CONFIRMED**

I tried to break it on all four axes and could not. Every headline number reproduces from scratch. Two of my own attack lines (fuel budget silently inherited; the closed-form *ramjet* also grows 20.6% so the pulsejet gets undeserved credit) turned out to be real observations that do **not** change the conclusion — I ran the experiment that tests the second one and it vindicated the report.

## 1. Mass block — self-consistent, but the fuel budget is inherited, not re-derived

`scripts/medium_model_v3c_235mm_audit.py`, recomputing `vehicle_dry_mass` from the design.json geometry:

| | V3a | 235 mm | delta |
|---|---|---|---|
| design.json `dry_mass_kg` | 12.569615 | 16.151719 | |
| **recomputed from its own geometry** | **15.496496** | **16.151719** | |
| \|recomputed − claimed\| | **2.927 kg (STALE)** | **0.00e+00 kg (SELF-CONSISTENT)** | |
| engine_duct_kg | 5.888278 | 6.120850 | +0.232572 |
| airframe_skin_kg | 5.289237 | 5.711888 | +0.422652 |
| wing / tank | 0.914992 / 0.903988 | identical | 0 |
| loaded fuel (via `load_frozen_design`) | 2.693257 | 2.693257 | **+0.000000** |
| **burn cap actually flown** | **2.423931** | **2.423931** | **+0.000000** |
| annulus usable | 6.9136 kg | 7.6342 kg | |

- The 235 mm block **is** recomputed for the larger body — machine-zero agreement, V3a's was not reused. Duct wall is at the 1.000 mm min gauge on both, as claimed.
- **But the fuel budget was not re-derived** — loaded fuel was *held*, so the burn cap is bit-identical. The agent discloses this. My counterfactual shows there was no alternative: holding V3a's stated 7.4167 kg margin gives the 235 mm **−0.8888 kg** of loaded fuel.
- **The +2.9269 kg V3a staleness finding is independently confirmed** and matters beyond this study.
- **Caveat the report does not state:** `design.fly()` calls `run_flight(geometry, MAX_WET_MASS_KG, ...)`, so the +0.655 kg of dry mass **never enters the flight** — it is charged entirely to payload margin. The bigger engine is flight-mass-free in this model; only drag pushes back.

## 2. Drag — the +20.64% frontal area is properly reflected; "~20% higher total" is the wrong expectation

M 0.50, 300 m, 22.68 kg, engine on (I used the flight-sim-consistent inlet capture; the report used the pulsejet's air flow, which is why only spillage differs):

| term | V3a | 235 mm | ratio |
|---|---|---|---|
| friction | 70.61 | 76.44 | 1.0827 |
| form | 5.75 | 7.70 | 1.3394 |
| **base** | 23.03 | 27.78 | **1.2064** |
| **spillage** | 7.16 | 8.64 | **1.2064** |
| wing profile / induced | 23.41 / 3.83 | identical | 1.0000 |
| TOTAL | 133.79 | 147.81 | 1.1048 |
| **frontal area m²** | 0.039838 | 0.048059 | **1.2064** |
| CD0_equivalent (frontal-ref) | 0.1907 | 0.1751 | 0.9184 |

Every frontal-area-referenced term scales by **exactly 1.2064**; at M 1.05 wave scales 1.5055 and total reaches 1.2184. Total is only ~1.10 subsonically because `drag_model="buildup"` is a component build-up, not a flat frontal CD0 — friction is wetted-area (and the body is 39 mm *shorter*) and the wing terms don't scale at all. **So "should be ~20% higher" is not the right test for this drag model, and the report's 1.108 is correct.** Its total (139.94/155.10) differs from mine only via the spillage capture assumption; the ratio (1.1083 vs 1.1048) and every conclusion are unchanged.

## 3. Geometry, cap, cliff — all clean

| check | result |
|---|---|
| throat area fraction | **0.29160** both vehicles, cap 0.30 — **OK** (report's "V3a 0.29169" is a typo; both are 0.5400²) |
| tail/chamber-dia | **4.0000** exactly (tailpipe 940.00 mm / chamber 235.000 mm) |
| sustain cliff | 3.60 → 0.88 N dead, 3.70 → 134.9 N alive; **4.00 → 149.19 N, `sustains: True`** |
| `spec_from_geometry(d.geometry)` | `EngineGeometry(chamber_diameter=0.235, chamber_length=0.389307, cone_length=0.1970968, tailpipe_diameter=0.1335789, tailpipe_length=0.940)` — **bit-matches the measured cliff row** |
| pulsejet `operable` / frequency | True / 75.1 Hz vs 220 Hz cap |

## 4. Re-flights — 12/12 reproduce

| vehicle | climb/dive | gate | traverse | margin | peak M | fuel | pass | report |
|---|---|---|---|---|---|---|---|---|
| V3a | 12/20 | 0.45 | 0.374 | 1.055 | 1.101 | 1.833 | PASS | 0.374 / 1.101 / 1.833 ✓ |
| V3a | 12/20 | 0.49 | 0.283 | 0.901 | 1.100 | 1.956 | PASS | 0.283 / 1.100 / 1.956 ✓ |
| V3a | 12/20 | 0.50 | 0.259 | 0.867 | 1.100 | 2.003 | fail | 0.259 fail ✓ |
| V3a | 12/20 | 0.51 | 0.235 | 0.834 | 1.100 | 2.065 | fail | 0.235 / 2.065 ✓ |
| V3a | 10/26 | 0.52 | 0.300 | 0.793 | 1.100 | 2.110 | PASS | 0.300 / 2.110 ✓ |
| 235 | 12/20 | 0.45 | 0.426 | 1.091 | 1.100 | 1.769 | PASS | 0.426 / 1.769 ✓ |
| 235 | 12/20 | 0.50 | 0.294 | 0.930 | 1.101 | 1.923 | PASS | 0.294 PASS ✓ |
| 235 | 12/20 | 0.51 | 0.269 | 0.897 | 1.101 | 1.973 | PASS | 0.269 PASS ✓ |
| 235 | 14/26 | 0.54 | 0.285 | 0.808 | 1.100 | 2.097 | PASS | 0.285 / 2.097 ✓ |

Its three stored JSONs (`v3c_235mm_screen/refine/control.json`) also regenerate its published tables exactly — no fabrication.

## 5. The experiment the report should have run — and it survives it

The closed-form **ramjet** captures on frontal area, so it *also* grows exactly 1.2064× (M 0.60: 387.0 → 466.9 N, +80 N, vs the pulsejet's +24 N). The report's isolation experiment scaled only the pulsejet. `scripts/medium_model_v3c_235mm_attribution.py`, 816 flights on the identical V3a airframe, Isp preserved:

| case | ceiling dive 20 | dive 26 | gain |
|---|---|---|---|
| A V3a stock | 0.49 | 0.52 | — |
| B V3a **pulsejet** ×1.2064 free | 0.54 | 0.57 | **+0.05** |
| C V3a **ramjet** ×1.2064 free | 0.50 | 0.53 | **+0.01** |
| D V3a **both** ×1.2064 free | 0.54 | 0.57 | **+0.05** (ramjet adds 0 on top) |
| E 235 mm as designed | 0.51 | 0.54 | **+0.02** |
| F 235 mm, ramjet de-scaled to V3a capture | — | — | **nothing passes** |

A and E reproduce the report's control table exactly (0.49/0.52 and 0.51/0.54), and B reproduces its isolation row (0.54/0.57). **The binding traverse minimum sits at the gate Mach itself** (`notch_mach` = 0.450 / 0.500 / 0.540 in every stock and scaled case) — i.e. pre-lightoff, pulsejet-only. That is why the free ramjet buys +0.01 alone and nothing marginal. The report's "free +0.05, bought-with-diameter +0.02, drag eats ~60%" is correct **even after** crediting the ramjet growth.

## Corrections to the report (none change the verdict)

1. **"never fuel" is too broad.** At V3a climb 14 / dive 26 / gate 0.54 the flight flames out **at** the 2.423931 kg cap with peak M 0.546 and `motor_cutoff_reached=False`. Fuel is the binding constraint in far cells; the claim holds only inside its climb-12/dive-20 table.
2. V3a throat area fraction is 0.29160, not 0.29169 (identical to the new design).
3. Its M 0.50 drag table feeds the *pulsejet's* air flow as inlet capture, giving spillage 13.32/15.93 N against the flight-consistent 7.16/8.64 N (total 139.94/155.10 vs 133.79/147.81). Ratios and conclusions unaffected.
4. The "V3a hard wall at 0.45" is not contradicted — a true ceiling of 0.49 is exactly what a 0.05 grid reports as "0.45 passes, 0.50 fails."

## What I did NOT test

- **No FP flight and no FP engine call.** Everything is closed-form. The report's own caveat stands: FP runs ~22% low on traverse, so a +0.02 is inside the offset and FP could erase it.
- **No FP sustain check above M 0.15 / 60 m.** I verified the design maps bit-exactly onto the duct that was measured there, not that it sustains at M 0.5+ and altitude.
- Attribution covers dives 20 and 26 and climbs 10–16 only; I did not re-run the 288-flight 0.05-step screen or the 0.48 row.
- I did not evaluate the thrust-margin gate as a gate — I only confirmed it is violated (0.79–1.09 against ≥1.15) at every late-gate cell, as the report states.

New files (nothing existing modified): `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_235mm_audit.py`, `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3c_235mm_attribution.py`, `C:\Users\dougl\Documents\GitHub\Douglas_Dart\out_medium_model\v3c_235mm_audit.json`, `C:\Users\dougl\Documents\GitHub\Douglas_Dart\out_medium_model\v3c_235mm_attribution.json`.

# AGENT 5

All checks complete. Findings below.

---

# ADVERSARIAL AUDIT — V3a lightoff ceiling

Everything below was re-derived from scratch against `docs/v3a_medium_model/design.json`, closed-form propulsion, `drag_model="buildup"`, `MEDIUM_MODEL_CD0_FRONTAL=0.1`. New files only:

| script | output |
|---|---|
| `C:\Users\dougl\Documents\GitHub\Douglas_Dart\scripts\medium_model_v3e_audit_trace.py` | `out_medium_model/v3e_audit_trace.json` |
| `...\scripts\medium_model_v3e_audit_balance.py` | `v3e_audit_balance.json` |
| `...\scripts\medium_model_v3e_audit_confound.py` | `v3e_audit_confound.json` |
| `...\scripts\medium_model_v3e_audit_pullout.py` | `v3e_audit_pullout.json` |
| `...\scripts\medium_model_v3e_audit_split.py` | `v3e_audit_split.json` (part 1 **superseded**, see note) |
| `...\scripts\medium_model_v3e_audit_split2.py` | `v3e_audit_split2.json` |
| `...\scripts\medium_model_v3e_audit_fpspot.py` | `v3e_audit_fpspot.json` |
| `...\scripts\medium_model_v3e_audit_final.py` | `v3e_audit_final.json` |

**Both reports' raw measurements reproduce exactly.** Their closed-form thrust/drag table, their per-cell traverse values, their FP thrust column — all bit-reproducible. The disputes are about *attribution* and about *search box*, not about arithmetic.

---

## 0. The measurement everything is scored against

Neither prior report bisected the ceiling; both worked on a 0.05 grid. Fine bisection (0.01 gate steps, climbs 6–16, floor 122 m, all three gates enforced):

| dive deg | **true ceiling** | best climb | traverse | margin | fuel kg | derived top m |
|---|---|---|---|---|---|---|
| 9.89 | **0.43** | 14 | 0.262 | 1.190 | 1.811 | 507 |
| 12 | **0.44** | 10 | 0.263 | 1.113 | 1.811 | 591 |
| 14 | **0.46** | 14 | 0.261 | 1.036 | 1.874 | 717 |
| 16 | **0.47** | 12 | 0.266 | 0.983 | 1.890 | 805 |
| 20 | **0.49** | 6 | 0.261 | 0.864 | 2.183 | 983 |
| 25 | **0.52** | 6 | 0.270 | 0.773 | 2.381 | 1247 |
| 30 | **0.54** | 8 | 0.307 | 0.732 | 2.314 | 1422 |
| 35 / 40 / 45 | **0.54** | 8 | 0.382 / 0.434 / 0.464 | 0.733 | 2.28 / 2.25 / 2.23 | ~1420 |

Timestep-converged: min traverse moves < 0.0006 g across dt 0.02 → 0.0025 s. Not a dt artifact.

---

## 1. Mechanism report

### M1 — closed-form thrust/drag/gravity table · **CONFIRMED**
Reproduced bit-for-bit at 300 m (T_pj 127.9/124.7/121.6/118.4/115.3/112.1/108.9 N and D 55.9/69.4/85.9/104.9/126.1/149.6/175.0 N at M 0.30–0.60). Slope `d(T−D)/dM = −460 N` per unit Mach closed-form (they quoted −486 on FP).

### M2 — FP pulsejet column · **CONFIRMED**, with a new caveat
Cold-seeded FP at 300 m: **111.55 / 109.75 / 106.03 N** at M 0.45 / 0.50 / 0.60 vs their 111.6 / 109.8 / 106.0. Ratios 0.942 / 0.952 / 0.973 — exact.

**New, not in either report:** the same engine *warm-marched* from M 0.30 reports **105.70 N at M 0.50 (−3.7 %)** instead of 109.75. Their table is cold-seeded; an FP *flight* marches warm. One sample — flagged, not established — but it points the FP flight below the FP table.

### M3 — predicted 0.26 g ceilings match the observed ones · **CONFIRMED**
My own balance (`a/g = (T−D)/W + sin(dive)`, gate set equal to the Mach evaluated so the ramjet contributes ~nothing) versus my own bisection, evaluated at the **measured** crossing altitude for each dive angle:

| dive | crossing alt (measured) | predicted | measured | error |
|---|---|---|---|---|
| 9.89 | 238 m | 0.440 | 0.43 | **+0.010** |
| 14 | 244 m | 0.470 | 0.46 | **+0.010** |
| 20 | 304–344 m | 0.510 | 0.49 | **+0.020** |
| 25 | 327 m | 0.540 | 0.52 | **+0.020** |
| 30 | 345 m | 0.570 | 0.54 | **+0.030** |

Every error ≤ 0.030 — **inside one 0.05 gate step at all seven dive angles.** The mechanism predicts the ceiling. It also correctly identifies that the *sign* ceiling is not the constraint: at 300 m / dive 20, `a = 0` sits at M 0.61 while `a = 0.26 g` sits at M 0.51.

### M4 — "second ceiling is the altitude budget: required top diverges past attainable top" · **REFUTED**
The vehicle **reaches the derived top in 11 of 11 flights traced**, including the ones they said could not:

| gate | climb | derived top | **top actually flown** |
|---|---|---|---|
| 0.50 | 14 | 1053.1 m | 1053.0 m |
| 0.50 | 16 | 1053.1 m | 1052.9 m |
| 0.55 | 12 | 1509.7 m | 1509.4 m |
| 0.54 | 8 | 1421.9 m | 1421.9 m |

Their "attainable top" table (climb 14 → 998 m, climb 16 → 769 m closed-form) is contradicted by measurement — climb 16 flew to 1053 m, climb 12 to 1509 m. The quasi-steady service-ceiling argument fails because the climb is *not* quasi-steady: it climbs on kinetic energy with thrust margin 0.73–0.88.

**The real second mechanism (measured):** the top-of-climb *Mach* falls with climb angle, and the dive band is a roughly fixed width, so the dive exits lower in Mach:

| climb | top-of-climb M | dive exit M | min traverse | where |
|---|---|---|---|---|
| 8 | 0.373 | 0.592 | 0.245 | v3_dive |
| 10 | 0.347 | 0.559 | 0.253 | v3_dive |
| 12 | 0.319 | 0.538 | 0.259 | v3_dive |
| 14 | 0.290 | 0.523 | **0.184** | drag_strip |
| 16 | 0.257 | 0.513 | **0.063** | drag_strip |

(gate 0.50, dive 20.) The gravity assist is withdrawn at M 0.513 while the ramjet ramp (`RAMJET_LIGHTOFF_RAMP_MACH = 0.10`) is only 13 % complete. Not an altitude shortfall — a **Mach-phasing** shortfall.

**Root cause, and it is a live bug:** `ClimbDiveProfile.dive_start_mach` (0.35) is read **only** by `derive_top_altitude`. The phase machine in `flight_sim.run_flight` never references it — it starts the dive the instant `h >= v3_top_altitude_m`, at whatever Mach that is (measured 0.257–0.373). So the altitude budget is sized for a band the vehicle does not fly.

### M5 — the lightoff drag step artifact · **CONFIRMED in magnitude, leverage OVERSTATED**
Duct cold vs lit at M = gate + 1e-3, 300 m, dive 20, 22.04 kg:

| gate | T_ramjet delivered | D_cold | D_lit | step N | step g | net g |
|---|---|---|---|---|---|---|
| 0.40 | 1.71 N | 86.24 | 90.77 | 4.53 | 0.0210 | −0.0130 |
| 0.45 | 2.16 N | 105.27 | 110.99 | 5.71 | 0.0264 | −0.0164 |
| 0.50 | 2.66 N | 126.60 | 133.63 | 7.03 | **0.0325** | −0.0202 |
| 0.55 | 3.21 N | 150.04 | 158.53 | 8.48 | 0.0392 | −0.0244 |

vs their 4.80 / 5.97 / 7.27 N and 0.0206 / 0.0263 / 0.0328 g. **Confirmed.**

But "the 0.45 wall is decided by this step" is an artifact of the 0.05 grid. The continuous dive-20 ceiling is **already 0.49 with the artifact present**; removing 0.033 g moves it to ≈0.51. That is **+0.02 Mach, not a gate step.**

And **"an FP screen may well place the wall at 0.50, not 0.45" is NOT SUPPORTED.** FP thrust sits 4.2–6.9 N below closed-form over M 0.45–0.60, worth −0.019 to −0.032 g — the *same size and opposite sign* as the artifact. The two roughly cancel.

### M6 — "the ramjet is not the limit" · **NOT RE-TESTED**
I ran no FP ramjet queries. Their §4 table stands unverified by me. Circumstantially consistent (every flight in the box reaches cutoff), but treat as not independently established.

---

## 2. Dive-limits report

### D1 — "gate 0.50 opens at dive ≥ 22 deg" · **CONFIRMED in substance, off by one degree**
They tested even angles at climb 12 only. Scanning 1-deg steps across climbs 6–16, gate 0.50, floor 122 m:

| dive | c6 | c8 | c10 | c12 | c14 | c16 |
|---|---|---|---|---|---|---|
| 20 | 0.239 F | 0.245 F | 0.253 F | 0.259 F | 0.184 F | 0.063 F |
| **21** | 0.255 F | **0.263 P** | **0.269 P** | **0.276 P** | 0.191 F | 0.066 F |
| 22 | 0.270 P | 0.278 P | 0.285 P | 0.292 P | 0.200 F | 0.073 F |

**Opens at dive 21**, not 22.

### D2 — "the uncharged ceiling is M 0.51, up from 0.45" · **REFUTED as a maximum**
They pinned climb = 12. With climb free the ceiling is **0.52 at dive 25 and 0.54 at dive 30**, saturating at 0.54 for dive 35/40/45 (table §0). Their 0.51 is a climb-12 artifact.

### D3 — "0.55 is a genuine wall, not a dive-angle problem" · **CONFIRMED**
0.54 passes; 0.55 fails at every dive angle out to 45 deg. At gate 0.55 / dive 30 / climb 12 the vehicle *does* reach the derived 1509.7 m top, hits the floor at M 0.558, strands on the drag strip at **−0.220 g**, and flames out at the 2.4239 kg cap with peak M 0.558.

### D4 — "γ ≥ 0 rule passes unanimously, cannot be violated by construction" · **CONFIRMED, reason corrected**
Traced min γ from integrated altitude in M 0.80–1.10 = **+1.001 deg in every flight** that reaches M 0.80; `rule_violated` False everywhere. Their stated reason ("dive_end_mach 0.60 < V3_RULE_MACH_LO 0.80") is true but **not operative** — the dive never reaches 0.60. The **floor** ends it, at M 0.513–0.558 across all 20 flights I traced; `dive_end_reason = floor` in 11 of 11 trace cases. Either way the rule window opens long after the dive latches and the drag strip flies at `climb_angle_deg = +1.0`. Violation would require a negative drag-strip angle or `dive_end_mach > 0.80`.

### D5 — pull-out CL_max provenance · **CORRECT, and the choice does NOT flip the answer**
They used `lift.py`'s alpha-limited value. Verified: `wing_clmax(1.20, AR 1.860, taper 0.512, sweep 13.42, M 0.54) = 0.8859`, mechanism `alpha-limited @ 25 deg`, `trustworthy=True`. Their 0.886 is right.

The task's premise that the choice flips the verdict is **refuted**. The only arc where it could matter (dive 30, dh 28 m):

| CL_max source | CL_max | n_available | n_required | verdict |
|---|---|---|---|---|
| lift.py alpha-limited | 0.8863 | 12.95 | 17.58 | SHORT |
| legacy `cl_max·cos²(sweep)` | 1.1353 | 16.59 | 17.58 | **still SHORT** |
| lift.py + vortex Kv = 3.0 | 0.9349 | 13.66 | 17.58 | **still SHORT** |

(Kv=3.0 gains little because the section-stall branch takes over at 0.9349.) At dh ≥ 78 m, n_required is 2.8–7.0 against 12.4–13.1 available — every option passes with room. **The verdict is CL_max-independent everywhere.**

### D6 — "the wing wins; induced drag kills the tight pull-outs" · **CONFIRMED in direction, two real arithmetic errors**

**(a) Wrong Oswald factor.** Their Δinduced used `lift.py oswald_efficiency = 0.7105` (M 0.54). The flight integrator charges induced drag with `WingConcept.oswald_e = 0.8469` — 19 % higher. Their arc is over-charged.

**(b) The closed-form arc under-eats altitude.** `R = dh/(1−cos γ_d)` assumes constant V. Marching a constant-n arc with real thrust and real drag (net thrust is positive, so V rises and R grows):

| gate | dive | dh commanded | **dh actually eaten** | error | dt closed-form | dt marched |
|---|---|---|---|---|---|---|
| 0.45 | 25 | 78 | 84.4 | +8 % | 1.99 s | 2.12 s |
| 0.45 | 25 | 128 | 151.3 | +18 % | 3.27 s | 3.71 s |
| 0.45 | 25 | 178 | **231.0** | **+30 %** | 4.56 s | 5.55 s |
| 0.45 | 30 | 178 | **218.6** | +23 % | 3.80 s | 4.43 s |
| 0.50 | 30 | 178 | 200.2 | +12 % | 3.80 s | 4.20 s |

So their "self-consistent" runs **do not bottom out at 121.92 m** — at dive 25 / dh 178 / gate 0.45 the arc bottoms ~53 m **below** the hard floor. Their small-dh rows (78 m) are within 0–5 % and stand.

**Their verdicts survive both corrections.** Marched min along-path accel in the arc, gate 0.45 / dive 30: **+0.913 / +1.367 / +1.448 g** for dh 78 / 128 / 178 (they reported +0.644 / +1.068 / +1.171 — all ≫ 0.26 either way). And at the one place the sign matters, gate 0.50 / dive 30 / dh 78, my marched min is **−0.064 g** against their −0.071 g: both fail, agreeing.

### D7 — "the pull-out is free in the model" · **CONFIRMED by code**
`flight_sim.py` `_concept_drag` and `_buildup_drag` both set `lift = m * G0_M_PER_S2 * cos(gamma_rad)` — load factor is 1 by construction. The only load-factor path in the file is `drag_mass_kg = m * RETURN_LOOP_LOAD_FACTOR` in the post-cutoff half-loop. The `v3_dive → drag_strip` transition is a latch: γ goes −30° → +1° in one 0.02 s step, zero radius, zero altitude, nothing charged.

---

## 3. "Is anything credited to the dive really the floor clamp or the derived top?"

**Answer: no — but the derived top is a hard threshold, and the floor is what ends every dive.**

> *Correction to my own work:* part 1 of `medium_model_v3e_audit_split.py` computed pins with the module-default gate 0.45 and put every pinned top *below* every gate-0.50 derived top, truncating all four runs. It appeared to show 147 % of the credit coming from the top. `split2.py` redoes it correctly and **reverses that**. Use `split2` only.

**Fair test — common top of 1131 m, nothing truncated, gate 0.50, climb 12:**

| dive | own derived top | traverse @ own top | traverse @ 1131 m | @ 1300 m | @ 1500 m |
|---|---|---|---|---|---|
| 14 | 976 m | 0.161 | 0.149 | 0.137 | 0.124 |
| 20 | 1053 m | 0.259 | 0.252 | 0.240 | 0.226 |
| 25 | 1098 m | 0.338 | 0.337 | 0.321 | 0.308 |
| 30 | 1131 m | 0.415 | 0.415 | 0.401 | 0.388 |

At a **fixed** top, dive 14 → 30 buys **+0.266 g** — versus +0.254 g at own derived tops. The credit is the gravity term `sin(dive)`. The derived top contributes **nothing positive**; extra altitude is a mild penalty (thinner air).

**But the top is a cliff-edged enabler.** Dive 20 fixed, top raised alone:

| top m | 772.5 | 900 | 1053 | 1200 | 1350 | 1500 |
|---|---|---|---|---|---|---|
| traverse | −0.030 | 0.008 | **0.259** | 0.247 | 0.235 | 0.226 |
| dive exit M | 0.492 | 0.509 | 0.538 | 0.587 | 0.599 | 0.599 |
| min sits in | drag_strip | drag_strip | v3_dive | v3_dive | v3_dive | v3_dive |

Below ~1000 m the dive is truncated *below* M = gate and the min jumps to the drag strip. The derived top is not credit — it is the binary condition "still descending when Mach crosses the gate."

**The floor clamp is not being credited either — it is monotonically harmful** (gate 0.50, dive 20, climb 12: traverse 0.259 / 0.251 / 0.242 at floor 122 / 200 / 300 m). But it is what **terminates the dive in 100 % of flights** (`dive_end_mach = 0.60` was never reached in any of the ~20 flights traced), so the (top, floor) pair sets the Mach at which gravity is withdrawn.

**At a fixed altitude budget there is an optimum dive angle** (top pinned 1053.1 m, gate 0.50): traverse 0.084 / 0.155 / 0.259 / **0.296** / 0.257 / 0.205 at dive 10 / 14 / 20 / **25** / 30 / 40 — steeper than 25 deg truncates the dive and loses.

---

## 4. What I did NOT test

- **No full FP flights.** Five engine-level FP *pulsejet* queries only. **Zero FP ramjet queries** — mechanism §4 (viability, phi, net thrust, "ramjet lights 0.20 Mach earlier") is entirely unverified by me.
- The warm-vs-cold FP hysteresis is **one sample**. Not established.
- Only V3a, closed-form propulsion, `buildup` drag, CD0 0.1. No sweep of `cowl_suction_recovery`, wing geometry, `DEFAULT_ALPHA_MAX_DEG`, or `RAMJET_LIGHTOFF_RAMP_MACH`.
- **No structural check.** 12–17 g on a 50 lb airframe is unmodelled anywhere in the repo.
- **The 1.15 thrust-margin house rule is violated at every ceiling I found** (0.73–1.19, below 1.0 for every dive ≥ 20 deg). I report it; I did not gate on it, per the three gates specified.
- The dive-20 ceiling of 0.49 is a hairline: traverse 0.2615 against 0.2600. So is dive 30 at 0.54.