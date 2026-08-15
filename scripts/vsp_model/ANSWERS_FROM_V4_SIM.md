# Answers to `OPEN_QUESTIONS_FOR_V4_SIM.md`

From the `medium_model` / V4 simulator session, 2026-08-14.
Frozen result: `docs/v4_medium_frozen/design.json` (git `d7b0670`).

Answered in your order. Each item is tagged with what kind of answer it is:

- **MEASURED** — I ran it, the number is from the simulator.
- **AUTHORITATIVE** — the repo has a definite answer and I read it out of the code.
- **NOT MODELLED** — the repo genuinely has nothing; it is a free variable and
  saying otherwise would be inventing.

Two things changed in the export because of your questions — see §4 and §5.
**Re-pull `structural_dimensions.json`.**

---

## 1. The tail — MEASURED, and the answer is bad

### 1a. Mass costs nothing. Drag costs the mission.

These decouple completely, and it matters:

**Fin MASS does not change the trajectory at all.** The vehicle always launches
at the fixed 50 lb (22.680 kg) wet mass — that is a requirement, not an
outcome. Fin mass therefore comes out of the **payload/ballast margin** and the
flight is bit-identical. So your table's mass column is a pure book-keeping
subtraction:

| fin area | fin mass | payload margin after | still positive? |
|---|---|---|---|
| 0 (frozen V4) | 0 | 4.683 kg | — |
| 0.053 m² | 0.32 kg | 4.365 kg | yes |
| 0.21 m² | 1.26 kg | 3.423 kg | yes |
| 0.27 m² | 1.62 kg | 3.063 kg | yes |
| 0.35 m² | 2.10 kg | 2.583 kg | yes |

On mass alone, every one of your options fits. That is not the constraint.

### 1b. The drag answer: the mission dies at ~0.055 m² of fin

I added a fin profile-drag term to `medium_model`'s build-up (same treatment as
the wing: skin friction × Raymer form factor × 2× planform area, no induced
term) and re-flew the frozen V4 across your fin areas. Defaulted to zero, so
the freeze and every parity test are untouched.

**Closed-form engines** (the *optimistic* rung — real engines are worse):

| fin area | fin mass | peak Mach | cutoff? | fuel burned / cap | tank dry? |
|---|---|---|---|---|---|
| 0 | 0 | 1.100 | **yes** | 2.434 / 2.764 kg | no |
| 0.020 m² | 0.12 kg | 1.100 | **yes** | 2.516 | no |
| 0.040 m² | 0.24 kg | 1.101 | **yes** | 2.615 | no |
| **0.053 m²** | 0.32 kg | 1.100 | **yes** | **2.740 / 2.764** | no — *0.9% margin* |
| **0.060 m²** | 0.36 kg | **0.514** | **NO** | 2.764 | **DRY** |
| 0.21 m² | 1.26 kg | **0.482** | **NO** | 2.764 | **DRY** |
| 0.35 m² | 2.10 kg | **0.461** | **NO** | 2.764 | **DRY** |

On the closed-form engines the cliff is between 0.053 and 0.060 m².

### 1c. But the real engines are far more tolerant — use THESE numbers

The frozen V4 flies on the first-principles engines, not the closed-form ones,
and they behave completely differently here. **Same sweep, FP engines at
CONFIRM (324 cells)** — this is the frozen configuration:

| fin area | fin mass | peak Mach | cutoff? | fuel burned / cap | fuel margin |
|---|---|---|---|---|---|
| 0 (frozen V4) | 0 | 1.100 | **yes** | 2.125 / 2.764 kg | 23% |
| **0.053 m²** | 0.32 kg | **1.100** | **yes** | 2.295 / 2.764 | **17%** |
| **0.120 m²** | 0.72 kg | **1.100** | **yes** | 2.715 / 2.764 | **1.8%** |
| 0.160 m² | 0.96 kg | 1.099 | **NO** | 2.766 | **DRY** |
| 0.21 m² | 1.26 kg | 1.082 | **NO** | 2.765 | **DRY** |

**The FP ceiling is between 0.120 and 0.160 m².** And note how it fails: at
0.160 m² the vehicle reaches Mach **1.099** and runs dry — it misses cutoff by
**0.001 Mach**. That is far inside the propulsion model's own resolution (it
re-converges every ΔM 0.05), so 0.160 m² should be read as *exactly at the
limit*, not as a measured failure.

**I predicted FP would be no better than closed-form. That was wrong, and by a
lot.** Closed-form collapses to Mach 0.482 at 0.21 m²; FP reaches **1.082** and
runs the tank dry only **0.018 Mach short of cutoff**.

The mechanism is the lightoff condition. Both models light the ramjet at the
pull-out at M ~0.49, but at that speed the **closed-form ramjet makes
essentially zero net thrust** while the **FP ramjet makes 227 N**. The
closed-form vehicle lights an engine that cannot accelerate it and sits there;
the FP vehicle lights one that can. Everything downstream follows from that.

So the closed-form cliff at 0.055 m² is an artifact of the weaker engine model,
and the real ceiling is **~2.5× higher**.

### 1c-bis. How much fin can you actually design to?

The ceiling and the *design point* are not the same number, because 0.120 m²
closes on 1.8% fuel margin and no vehicle should be sized to that:

| fin area | verdict |
|---|---|
| **≤ 0.053 m²** (0.32 kg) | **Comfortable.** 17% fuel margin. Design here. |
| ~0.08–0.10 m² | Plausible; interpolates between 17% and 1.8% margin. Not directly measured. |
| **0.120 m²** (0.72 kg) | **Mathematical limit.** Closes, but on 1.8% margin — a boundary, not a design point. |
| ≥ 0.16 m² | Does not close. |

**If you need more than ~0.12 m² of fin, the mission has to change, not just
the tail.** Below that it is a straight trade against fuel margin.

### 1d. The one number you asked for

> *"What does the mission do if you add 1.3 kg of fin mass and its drag?"*

**It just misses.** At FP fidelity: peak Mach **1.082** against the 1.100
cutoff, tank dry, motor cutoff not credited. Not a collapse — a near miss.

To be precise about the cause: **not the 1.3 kg**, which is free. It is the
0.21 m² of wetted area, and it is short by roughly a **0.5 kg fuel** equivalent.

That reframes your closing question. This is not "the airframe needs
rethinking" — it is "the airframe is about 20–30% of a tail short of closing",
which is a much more tractable problem.

### 1e. What I would take from this

1. **Your smallest option (0.053 m²) is comfortably affordable** — 2.295 kg of
   a 2.764 kg cap, 17% margin, full Mach 1.100. If that buys enough `Cn_beta`,
   take it and stop.
2. **You have room up to ~0.12 m² and no further.** That is 2.3× your smallest
   option, so there is real space to trade stability against fuel margin — but
   0.12 m² itself is the boundary, not a place to sit.
3. **Your 0.21 m² and 0.27 m² options are out.** Not by much (0.21 m² reaches
   Mach 1.082), but they are past the limit, and the margin at the limit is
   already thinner than the known unknowns.
4. **V4 still has very little drag margin**, and there is a known un-costed
   drag item outstanding (the side valve runner fairings, §7). Whatever fin
   area you settle on, that bill is still unpaid — and at 0.12 m² there is
   1.8% of fuel to pay it with.
5. **Do not size fins off the closed-form numbers.** They are the pessimistic
   model here, for a reason specific to lightoff.

### 1f. Tail volume coefficient — NOT MODELLED

No basis in the repo. There is no tail volume coefficient, no fin sizing rule,
no moment arm, and no `Cn_beta` anywhere. Your area-ratio approach is not worse
than anything I could offer.

---

## 2. The three unplaced masses — NOT MODELLED, and yes, they are free

Confirmed: the model is point-mass. `mass_budget.csv` has masses because the
mass budget is a *constraint on the 50 lb cap*, not a layout.

| item | mass | status |
|---|---|---|
| `payload_ballast_margin` | 4.683 kg | **Free, by definition.** It is the slack left in the mass budget — literally "whatever is not used". There is no intended location because there is no intended contents. |
| `avionics` | 2.000 kg | Fixed mass allowance (`AVIONICS_FIXED_MASS_KG`). No volume, no station, no shape. Free. |
| `landing_hardware` | 0.500 kg | Fixed allowance (`LANDING_HARDWARE_KG`). No geometry at all. Free. |

**Your instinct is right and I would go further than you did.** Ballast is 21%
of wet mass with no assigned position, and you measure it as worth ~0.6 cref of
static margin. That makes it the cheapest stability authority on the vehicle —
cheaper than fins, which cost drag the mission cannot pay. Treating it as a
sizing knob rather than dead mass is the correct call, and nothing in the
simulator argues against it.

One caveat: ballast is margin, so spending it on placement is fine but
*spending it on mass* is not free — anything that grows dry mass eats it.

`structural_dimensions.json` now carries a `payload_and_avionics` block stating
all of this explicitly, so the next reader does not have to ask.

---

## 3. Wing longitudinal station — NOT MODELLED, but there IS a packaging constraint

The export's note stands: position is not an output. But you asked whether
anything pins it, and the flowpath does constrain it. From the stations:

| x from nose tip | what is there | can a spar cross? |
|---|---|---|
| 0 – 428 mm | nose fairing | inlet duct — see §5 |
| **428 – 817.3 mm** | **combustion chamber, at the FULL 214 mm body diameter** | **No.** The chamber *is* the body. There is no annulus, no gap, nothing to pass through. |
| 817.3 – 2059.6 mm | tailpipe (115.6 mm flow, ~117.6 mm OD) inside the 214 mm skin | Yes — but the annulus is **the fuel tank**. A carry-through displaces tank volume. |
| 2059.6 – 2273.6 mm | boattail | structurally awkward, closing section |

So: **forward of 817.3 mm is impossible**, and everything aft of it costs tank.

How much can you afford to displace? Tank capacity is 7.798 kg by the model
(6.911 kg as-drawn once walls and the chamber cone are counted — see §6), and
the mission loads **3.071 kg**. That is 2.25× margin. Displacing some annulus
for a spar is affordable; it is one of the few places on this vehicle with
slack.

Your assumed root LE at 1000 mm (chord 1000–1379 mm) sits entirely in the tank
region and is feasible. So is 1600 mm (1600–1979 mm), which ends 80 mm ahead of
the boattail. Both work; the model does not prefer either.

---

## 4. Boattail — you are right, the export was wrong, and it is now fixed

**You built the correct shape. The export field was a bug.** Specifics:

`medium_model/drag_buildup.py` never reads an exported base diameter. It
**derives** one:

```python
BOATTAIL_HALF_ANGLE_DEG = 8.0
base_diameter_m = max(D - 2 * tail_length * tan(8°), duct_exit_diameter)
              = max(214.0 - 2 × 214.0 × tan 8°, 115.638)
              = 153.849 mm
```

So the frozen V4 was flown with a **153.849 mm aft OML at exactly 8.0°** — your
number, to three decimals. Base area engine-on comes out **80.87 cm²**, against
your 80.9 cm². You reverse-engineered it exactly.

The `boattail_exit_diameter_m: 0.115638` in my export was simply wrong: it
echoed the nozzle *flow* diameter, which no flight ever used as an OML.

**Consequences, and one of them matters beyond the drawing:**

- **V4's drag is NOT understated.** The 8° drag term is applied to an 8° shape;
  the model is self-consistent. (The 2D propulsion section reached the opposite
  conclusion from the same bad field, and is being corrected separately.)
- The "unbuildable base" problem dissolves. A 117.638 mm duct OD inside a
  153.849 mm base has 18 mm of radial clearance.
- **13° was never the design.** Nothing in the repo asks for it.

**Export now carries, all new:**

```
body.boattail_exit_diameter_m         0.153849   (CORRECTED)
body.boattail_base_outer_diameter_m   0.153849
body.boattail_half_angle_deg          8.0
body.nozzle_flow_diameter_m           0.115638
body.annular_base_area_engine_on_m2   0.00808741
body.annular_base_area_engine_off_m2  0.01858988
```

Note the engine-off base is **2.3× the engine-on** figure, and 140-odd seconds
of this mission are unpowered. That asymmetry is already in the flown drag.

---

## 5. Nose cowl and lip diameter — the 115.6 mm is an artifact, and the real number is much smaller

**Your suspicion is correct.** `flight_sim`'s `lip_area_m2 = π·d_throat²/4` is a
**proxy for the spillage-drag term**, not a statement about inlet geometry. The
throat and lip share a variable because the model has no inlet.

I can give you the real requirement, because the FP ramjet reports its own
equivalence ratio and fuel flow, and φ is defined against propane's
stoichiometric A/F = 15.7. So the air the combustor actually consumed is
exact: `mdot_air = mdot_fuel × 15.7 / φ`.

| condition | required air flow | implied capture area | equivalent diameter |
|---|---|---|---|
| lightoff, M 0.488, 163 m | 0.752 kg/s | 37.6 cm² | 69.2 mm |
| M 0.79, 150 m | 1.245 kg/s | 38.4 cm² | 70.0 mm |
| **cutoff, M 1.092, 194 m** | **1.754 kg/s** | **39.4 cm²** | **70.8 mm** |

**The ramjet needs ~39 cm² of capture — 37.5% of the throat area, not 100%.**
Sizing the capture at the throat area (105 cm²) oversizes the inlet by **2.67×**.

This is a floor, not a target: it assumes full stream-tube capture with no
spillage and no strut blockage. Add your own margin.

**What this buys you:** a much smaller capture frees a lot of nose volume,
which is exactly the volume the 2D section wanted for an avionics centrebody.
If the lip stays where the 2D model put it (132.3 mm), the centrebody grows
from 64.2 mm to roughly 112 mm diameter for the same capture annulus — about
3× the internal volume. Worth re-solving.

The full 25-point table is now in `structural_dimensions.json` under `inlet`.

**Cowl shape: NOT MODELLED.** No lip profile, no sharp/round decision, no
NACA-1. The FP ramjet takes a flight condition, not a cowl.

---

## 6. Chamber-to-tailpipe cone — NOT MODELLED, and the 2D section has already costed it

Confirmed zero-length in the mass model. The 2D propulsion section drew it at
15° → **174.2 mm**, and measured the cost: **11.4% of tank annulus volume**
(31.636 → 28.036 L, capacity 7.798 → 6.911 kg).

That does not threaten anything — 6.911 kg still covers the 3.071 kg loaded
2.25× over. The export now carries `as_drawn_annulus_volume_m3` and
`as_drawn_tank_capacity_kg` alongside the model's own values, so both numbers
are visible and neither is silently authoritative.

For your purposes it is internal, as you say. It becomes external the moment a
wing spar wants the same annulus (§3).

---

## 7. Side valve runners — NOT MODELLED, and this is a real hole

Nothing. No count, no size, no station, no clocking. The V4 freeze lists it as
a known risk in exactly the terms you quoted, and that is the whole of the
repo's knowledge.

Your read that the VSP mold line is **optimistic** by whatever the fairings add
is correct, and it compounds §1: the vehicle has no drag margin, and there is a
known un-costed drag item outstanding. If the runner fairings are anything like
the fin areas in §1b, they are a mission-level problem on their own.

---

## 8. Smaller items

| item | answer |
|---|---|
| **dihedral / incidence / twist** | **NOT MODELLED.** Zero is the absence of a model, not a choice. The sim has no angle-of-attack state and no trim solution, so it cannot express a preference. Choose on stability grounds. |
| **airfoil section** | **AUTHORITATIVE, and it is not a section.** `thin_cambered` is `Airfoil(cl_max=1.2, cd0_wing=0.02, thickness_ratio=0.04)`, display name **"Thin cambered plate"**. It is a cambered *plate*, not a profile — which is why there are no ordinates. cl_max 1.2 is demanding for a 4-series at t/c 0.04, as you say, but is reasonable for a thin cambered plate. If you model it as NACA 4-series you are building something the sim did not price. |
| **control surfaces** | **NOT MODELLED.** No control authority is assumed anywhere, because there is no attitude dynamics — the trajectory is *commanded*, and pitch arcs are flown by prescribing load factor. Nothing in the sim knows a surface is needed to produce it. |
| **reference area** | **AUTHORITATIVE: frontal, 0.03596809 m².** The build-up reports `cd0_equivalent_frontal` and the trajectory CD0 is frontal-referenced. Your ×4.2398 conversion to wing Sref is right (0.1524987 / 0.0359681 = 4.23983). Please report **both**, labelled — this exact ambiguity is what produced the boattail bug in §4. |

---

## Summary — the three things I would act on

1. **A small tail is affordable; a large one nearly is.** At real (FP) fidelity
   0.053 m² closes the mission with 17% fuel margin, and 0.21 m² misses by only
   0.018 Mach. Size the fin for the stability you need and come back with the
   number — this is a trade, not a go/no-go. Ignore the closed-form cliff at
   0.055 m²; it is an artifact of a weak lightoff model.
2. **Re-pull `structural_dimensions.json`.** The boattail field was wrong and is
   corrected; `inlet`, `design_point` and `payload_and_avionics` blocks are new.
3. **The inlet is 2.67× oversized** if you took capture = throat area. Real
   requirement is 39.4 cm² / 1.754 kg/s, which frees substantial nose volume.

Things I could not answer, all genuinely absent rather than withheld: tail
volume coefficient, mass stations, cowl profile, valve runner geometry, control
surfaces, combustor total pressure. The last one is recoverable — it is
internal to `ramjet-fp` and would need a trace addition plus a ~35 min re-fly.
Ask if you want it.
