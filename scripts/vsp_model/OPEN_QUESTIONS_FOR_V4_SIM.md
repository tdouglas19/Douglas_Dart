# Open dimensions the VSP model needs from the V4 simulator

Paste-ready for the session working on `medium_model/`.

Every item below is a dimension the VSP model **currently invents** because
`out_medium_model/v4_export/structural_dimensions.json` does not contain it. The
value in brackets is what `scripts/vsp_model/geometry_inputs.py` assumes today,
so nothing is blocked — but each one is a choice nobody has actually made.

Ordered by how much the answer changes the vehicle, not by how easy it is.

---

> ## CORRECTION, 2026-08-14 — the tail is CHEAPER than §1 says
>
> §1 below asked you to price a tail at **1.3–1.7 kg**. That number came from a
> VSPAERO fin-area sweep which I have since had to retract: the vortex-lattice
> solve does not converge at the fin/body junction, and refining the mesh made it
> worse rather than better.
>
> Re-derived analytically (Barrowman, cross-checked against an independent
> slender-body integral agreeing to 5.2%), the recommended tail is:
>
> | | |
> |---|---|
> | fin area, 4 panels | **0.1220 m²** |
> | **fin mass @ 6.0 kg/m²** | **0.73 kg** — 16% of payload margin, not 27–35% |
> | geometry | 155.5 mm root, 70.0 mm tip, 270.6 mm tall, AR 2.4, 35° sweep, X at 45° |
> | station | root chord 1904–2060 mm, hard against the barrel end |
> | wing | moved aft to root LE **1600 mm** (free, clears the chamber) |
>
> That sits inside your 0.12–0.16 m² ceiling, at the point where you measured
> cutoff still achievable on 1.8% fuel margin. **The useful question is now
> whether 0.73 kg plus that fin's drag still closes the mission** — a smaller ask
> than the one in §1.
>
> Also flagged: our two methods disagree on whether the airframe is unstable at
> all. Barrowman says the as-sized vehicle is *marginally stable* (+0.54 pitch /
> +0.04 yaw calibres); VSPAERO said badly unstable. I cannot resolve it with the
> tools here, and the tail above is sized for the pessimistic case because it is
> affordable either way.

## 1. Can the vehicle carry a real tail, and what does it cost? ⭐ BIGGEST

**Context, and this is the finding that motivates the question.** VSPAERO says
the as-sized V4 is statically unstable in **both** pitch and yaw:

| | value | confidence |
|---|---|---|
| static margin | **−1.92 ± 0.12 cref** (−570 mm) | mesh-converged, 4 realisations |
| `Cn_beta` | **−2.40 ± 0.09 /rad** (needs > 0) | mesh-converged |

The slender body's Munk moment dominates and the 533 mm wing cannot counter it.
The repo has **no tail surfaces at all** — the export says so outright: *"NOT
MODELLED... no area, no volume coefficient, no drag term."*

Yaw is the binding axis: `Cn_beta` barely responds to wing station, only to fin
area.

**What I need:**

1. **Is a tail in scope at all**, or is the intent an actively-controlled
   unstable airframe? That changes everything downstream.
2. **Mass and drag budget for it.** At the sizing model's own
   `WING_AREAL_MASS_KG_M2 = 6.0`:

   | fin area | fin mass | as % of the 4.68 kg payload margin |
   |---|---|---|
   | 0.053 m² (today's VSP default) | 0.32 kg | 7% |
   | 0.21 m² | 1.28 kg | 27% |
   | 0.27 m² | 1.65 kg | 35% |

   Even the *smallest* of these is mass the V4 freeze does not carry. Can the
   mission absorb 1–1.7 kg out of payload margin, plus the fin's parasitic drag,
   and still make Mach 1.1 with the fuel budget?
3. **A tail volume coefficient**, if the sim has any basis for one. Sizing from
   an area ratio (what I do now) is a guess.

**CAVEAT — do not size fins off my numbers yet.** The *direction* (substantially
more fin) is established. The *magnitude* is not: large-fin configurations are
not mesh-converged, and a fin-area sweep is partly measuring the mesh. A
convergence study at the fin/body junction is a prerequisite. See `README.md`.

Currently assumed: 4 fins, X at 45°/135°/225°/315°, total area 0.35 × wing Sref,
AR 1.20, taper 0.45, LE sweep 35°, t/c 0.04, root TE at the boattail start
(2059.6 mm) → root chord 145.4 mm, exposed height 126.5 mm, tip-to-tip 455 mm.

---

## 2. Where do the three unplaced masses go?

The repo is point-mass, so `mass_budget.csv` has masses and **no stations**. I
assigned every one. Three are pure guesses and they set the CG:

| item | mass | assumed station | why it matters |
|---|---|---|---|
| `payload_ballast_margin` | **4.68 kg** | 214 mm | **21% of wet mass, entirely unplaced** — the single largest CG lever on the vehicle |
| `avionics` | 2.00 kg | 214 mm (nose-cowl annulus) | is there a real avionics bay, and where? |
| `landing_hardware` | 0.50 kg | 1137 mm (mid-body) | skids/attach points — where do they mount? |

The rest (duct, skin, tank, fuel, wing) I derived from flowpath geometry and
they are defensible.

**Result today:** CG 922 mm at release → 841 mm at burnout (the tank is an
annulus around the tailpipe, well aft, so the CG runs *forward* 81 mm = 0.27
cref as it burns). Iyy = Izz = 9.13, Ixx = 0.183 kg·m².

**What I need:** real stations, or confirmation that ballast placement is a free
design variable. If it is free, it is worth 0.6 cref of static margin on its own
and should be treated as a sizing knob rather than dead mass.

---

## 3. Wing longitudinal station

Currently assumed: **root LE at 1000 mm** (root chord spans 1000–1379 mm).

The export states this is not an output: *"the wing's LONGITUDINAL POSITION on
the body is not an output of this repo — it is yours to choose."*

It matters more than I expected — moving it from 1000 mm to 1600 mm is worth
about **+1.1 cref** of static margin. Is there any packaging constraint (spar
carry-through vs the chamber, the tank annulus, the valve runners) that pins it?

---

## 4. Boattail exit diameter — the export contradicts the flown drag model

| source | aft diameter | implied half-angle |
|---|---|---|
| `structural_dimensions.json` `boattail_exit_diameter_m` | 115.6 mm | **13°** |
| `medium_model/drag_buildup.BOATTAIL_HALF_ANGLE_DEG` | **153.9 mm** | 8° |

The JSON field just repeats the nozzle diameter. 13° is past the ~8° attached-flow
limit the drag model itself calls the upper bound, and past what V4 was actually
*flown* with. I build the 8° version, so the VSP mold line and the flown drag
describe one aft body — leaving an **80.9 cm² annular base** around the nozzle.

**What I need:** confirmation that 8° / 153.9 mm is intended, or a real aft
profile. This changes base drag, which is one of the larger terms in the build-up.

---

## 5. Nose cowl — length only, no profile, and no stated lip diameter

The export gives a 428 mm nose fairing length and nothing else. The lip diameter
is not a field anywhere — I derive it as the throat diameter (115.6 mm) because
`flight_sim.py` computes lip area as `pi * throat_diameter**2 / 4`.

Currently assumed: cubic profile leaving the lip at a **12° half-angle**, tangent
to the barrel (mean slope over the fairing is only 6.6°, so this is a mild flare).

**What I need:**
- Is 115.6 mm the real ramjet capture diameter, or an artifact of the throat and
  lip sharing a variable?
- Does the FP ramjet inlet imply a cowl shape (sharp lip? rounded? NACA-1?)

---

## 6. Chamber-to-tailpipe transition length

The export is explicit that this is missing: *"the mass model treats the tailpipe
as a CONSTANT-diameter pipe at the nozzle diameter, i.e. the chamber-to-tailpipe
cone has zero length in the model. Give it a real length in CAD; the model does
not tell you what it should be."*

Internal only, so it does not change the external mold line **today** — but it
will the moment anyone wants internal duct aero or a real structural layout.

---

## 7. The side valve runners

The V4 export flags these as an unresolved problem: the pulsejet's side-mounted
valve runners *"still do not fit inside a 214 mm skin — an external fairing is
needed and its drag is in neither the flat CD0 nor the build-up."*

They are also absent from the VSP model, so its external mold line is
**optimistic** by whatever those fairings add.

**What I need:** fairing geometry — count, size, axial station, clocking. Four
runners clocked into the wing/fin gaps would interact with both.

---

## 8. Smaller items

| item | assumed | question |
|---|---|---|
| wing dihedral / incidence / twist | all 0° | zero because *unmodelled*, per the export's own note — is zero actually wanted? |
| airfoil section | NACA 4-series, 2% camber at 40% chord | `thin_cambered` is a label with t/c 0.04 and `cl_max_2d` 1.2, not ordinates. cl_max 1.2 at t/c 0.04 is demanding — what section is intended? |
| control surfaces | none | if the airframe stays unstable, what control authority is assumed? |
| reference area | wing trapezoid 0.15250 m² | I use wing Sref for VSPAERO. The trajectory CD0 is **frontal**-referenced (0.03597 m²); conversion factor **×4.2398**. Confirm the convention you want reported. |

---

## The one number I would most like back

**What does the mission do if you add 1.3 kg of fin mass and its drag to the V4
freeze?** That is the smallest tail consistent with fixing a stability problem
the trajectory model cannot currently see, and it comes straight out of the
payload margin. If the mission does not close with it, the airframe needs
rethinking before the aero is refined any further.
