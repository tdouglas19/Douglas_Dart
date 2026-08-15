# Answers to `v4_export_requests_from_propulsion_2d.md`

From the `medium_model` / V4 simulator session, 2026-08-14.
Frozen result: `docs/v4_medium_frozen/design.json` (git `d7b0670`).

**Headline: your §2.1 is right that there is a conflict, and wrong about which
side is authoritative. The 8° is correct and was flown; the export field was the
bug. Your aft body is built to the wrong shape and needs redrawing.**

The export has been corrected. **Re-pull
`out_medium_model/v4_export/structural_dimensions.json`.**

Answered in your priority order, then the rest.

---

## Your priority 1 — §2.1 boattail 8° vs 12.94° — RESOLVED: 8° is authoritative

`drag_buildup` **never reads an exported base diameter**. It derives one:

```python
# medium_model/drag_buildup.py
BOATTAIL_HALF_ANGLE_DEG = 8.0

def base_diameter_m(body_diameter_m, tail_length_m, duct_exit_diameter_m,
                    boattail_half_angle_deg=BOATTAIL_HALF_ANGLE_DEG):
    shrink = 2.0 * tail_length_m * tan(radians(boattail_half_angle_deg))
    return max(body_diameter_m - shrink, duct_exit_diameter_m)
```

For V4: `max(214.0 − 2 × 214.0 × tan 8°, 115.638) = **153.849 mm**`, at exactly
8.0°. Verified by running the function.

So the chain you inferred — *"drag_buildup is applying an 8° boattail-drag term
to a 12.94° shape, therefore V4's drag is understated"* — **does not hold.** The
model applies an 8° term to an 8° shape it computed itself. It is internally
consistent, and **V4's drag is not understated. No re-fly is needed.**

What went wrong was on my side: `boattail_exit_diameter_m` in the export echoed
the nozzle **flow** diameter, which no flight ever used as an outer mould line.
The 12.94° you derived is an artifact of that bad field.

**Consequences for your drawing:**

- The as-flown aft body closes 214.0 → **153.849 mm** over 214 mm. It does
  **not** close onto the nozzle.
- An annular base of **80.87 cm²** (engine on) / **185.90 cm²** (engine off)
  remains between the boattail base and the nozzle. Base drag on it is a real,
  already-flown term.
- Your §1.1 containment defect **dissolves**: a 117.638 mm duct OD inside a
  153.849 mm base has ~18 mm radial clearance. Nothing is unbuildable.
- Your fix — opening the base OD to 117.638 mm at 12.69° — solves a problem
  that does not exist and produces a shape the flight model did not fly.

The VSP model reached the correct answer independently (it built the 8° version
and measured the same 80.9 cm² annulus), which is a useful cross-check.

**Export now carries:**

```
body.boattail_exit_diameter_m         0.153849   (CORRECTED, was 0.115638)
body.boattail_base_outer_diameter_m   0.153849
body.boattail_half_angle_deg          8.0
body.nozzle_flow_diameter_m           0.115638
body.annular_base_area_engine_on_m2   0.00808741
body.annular_base_area_engine_off_m2  0.01858988
```

Note the 2.3× engine-off/engine-on base ratio, and that ~140 s of this mission
are unpowered. That asymmetry is already priced in the flown build-up.

---

## Your priority 2 — §4.5 / §3.1 the avionics volume and the inlet

### §4.5 avionics volume — NOT MODELLED, genuinely

`AVIONICS_FIXED_MASS_KG = 2.0` is the entirety of the repo's knowledge. There is
no volume, no station, no packing density, no shape. I am not going to invent
one and hand it back as a simulator output.

It is a **free design variable**, as is the 4.683 kg payload/ballast margin.
The export now says so explicitly in a new `payload_and_avionics` block rather
than being silent.

### §3.1 the inlet — I can answer the one you actually wanted

> *`"required_mass_flow_kg_s": null,  // <-- the one I actually want`*

**It is recoverable, and here it is.** The FP ramjet records its own equivalence
ratio and fuel flow at every re-convergence, and φ is defined against propane's
stoichiometric A/F = 15.7, so the combustor's air flow is exact:

```
mdot_air = mdot_fuel × (A/F)_stoich / φ
```

| condition | required air flow | implied capture area | equivalent dia |
|---|---|---|---|
| lightoff, M 0.488, 163 m | 0.752 kg/s | 37.6 cm² | 69.2 mm |
| M 0.790, 150 m | 1.245 kg/s | 38.4 cm² | 70.0 mm |
| **cutoff, M 1.092, 194 m** | **1.754 kg/s** | **39.4 cm²** | **70.8 mm** |

**This overturns your capture rule.** You solved the lip diameter so that
annular capture = throat area = 105.02 cm². The engine only ever needs
**39.4 cm² — 37.5% of that.** The inlet as drawn is oversized **2.67×**.

The capture area is remarkably flat across the burn (37.6 → 39.4 cm²), so a
single design point is defensible here in a way it usually is not.

Caveats, both making this a **floor**: full stream-tube capture assumed (no
spillage), and no strut blockage — which is your own §3.1 open item, and it
still is.

**What this buys you, and it lands on your priority 2:** a 2.67× smaller capture
frees most of the nose. If the lip stays at 132.264 mm, the same capture annulus
allows a centrebody of roughly **112 mm diameter instead of 64.2 mm** — about
3× the internal volume for avionics. The centrebody stops being volume-starved.
Worth re-solving before drawing anything else.

Full 25-point table is now exported under `inlet` in
`structural_dimensions.json`.

---

## Your priority 3 — §1.1 unbuildable base diameter — DISSOLVED

See priority 1. The field was wrong; the geometry was never unbuildable. Your
automated containment test did its job — it found a real inconsistency, it just
pointed at the geometry when the fault was in the export.

---

## §1.2 OML vs flow conventions — CONFIRMED, and partly fixed

You are right that the export mixed conventions, and right about which is which:

| key | it is | now |
|---|---|---|
| `body.diameter_m` = 0.214 | **OML** | unchanged |
| `internal_flowpath.tailpipe_diameter_m` = 0.115638 | **flow** | unchanged, plus `body.nozzle_flow_diameter_m` added for the aft end |
| `internal_flowpath.chamber_diameter_m` = 0.214 | **ambiguous** | see below |

**On the chamber: it is genuinely both, and that is a real modelling
simplification, not a labelling slip.** V4's premise (user directive) is that
the chamber *is* the body — the annular gap was deleted. The mass model builds
the duct from `π·D·L_chamber` at D = 214.0 mm and the drag model uses the same
214.0 mm as the OML. Nothing anywhere subtracts a wall.

So your observation stands exactly: `chamber_volume_m3` assumes zero wall
thickness, and the honest bore with the export's own gauges is ~209 mm, −4.6%
on volume. I have **not** changed the key, because 214.0 mm is what was flown
and silently changing it would make the export disagree with the freeze. It is
now flagged in the flowpath note instead.

If you want the −4.6% chamber volume reflected in the *engine* model rather
than just the drawing, that is a real change to `fp_spec` and needs a re-fly —
say so and I will run it.

---

## §1.3 fuel annulus 11.4% optimistic — CONFIRMED, exported, not a problem

Your correction is right and I have taken it at your numbers:

| | model | as drawn | Δ |
|---|---|---|---|
| annulus volume | 31.636 L | 28.036 L | −11.4% |
| tank capacity @ 50% | 7.798 kg | 6.911 kg | −11.4% |

Both are now in the export (`annulus_volume_m3` unchanged as flown, plus
`as_drawn_annulus_volume_m3` and `as_drawn_tank_capacity_kg`), with a note
saying which is which and why the model's is left alone.

**It does not threaten the mission**, and your 2.25× framing is the right one:
loaded fuel is 3.071 kg and the *flown* burn is only **2.125 kg**. Even the
as-drawn 6.911 kg capacity is 3.25× the actual burn.

The 2.33 L aft of `tailpipe_end` — noted, not claimed. The model's tank does
not extend there and I would rather not credit volume the mass model has not
priced.

---

## §4 Questions only the flight model can answer

**§4.1 — Does the V4 thrust model assume a choked nozzle? NO.**

Two separate answers, and the second is the one that counts for V4:

- The closed-form `ramjet_simple` explicitly branches choked/unchoked on the
  pressure ratio (`compressible_orifice_mass_flow(...) -> (mdot, choked)`) and
  reports `choked` as an output flag. Its docstring is explicit that thrust is
  "the general expansion from (P0, T0) to Pe", not a choked-flow assumption.
- **But the frozen V4 was not flown on that model.** It was flown on `ramjet-fp`
  via `FpPropulsion`. So your M 1.061 concern does not apply to either path —
  neither assumes choking.

**§4.2 — Combustor total pressure vs Mach and altitude? NOT EXPORTED.**

`FpPropulsion`'s trace records thrust, fuel flow and equivalence ratio per
re-convergence. Combustor `p0` is internal to `ramjet-fp` and is not surfaced.

This is **recoverable**: add it to the trace and re-fly (~35 min). The export
now carries `design_point.combustor_total_pressure_ratio: null` with that note
attached, so the hole is explicit rather than absent. Say the word and I will
run it — it is the one thing on your list that would convert your nozzle trade
from assumption to calculation, and it is cheap.

**§4.3 — Is the 8° boattail applied to the V4 shape? YES, and correctly.**
See priority 1. No re-fly needed.

**§4.4 — Does the mass model account for a real chamber→tailpipe cone? NO.**

Confirmed zero-length, exactly as you inferred, and that is where your §1.3
11.4% comes from. `vehicle_dry_mass` computes duct area as
`π(D·L_chamber + d_throat·L_throat)` — a full-diameter chamber butted straight
onto a constant-diameter pipe, with no transition.

**§4.5 — avionics volume. NOT MODELLED.** See priority 2.

**§4.6 — ramjet capture mass flow. YES — 1.754 kg/s max.** See priority 2.

---

## §3.2–§3.6 — the rest

**§3.2 diffuser length-limited at 7.82°.** No comment available from the flight
model: it has no diffuser, no internal duct aero, and no pressure-recovery term
that responds to diffuser geometry. Your 7.82° finding is not something the
simulator can confirm or contradict. Worth noting the inlet is now 2.67×
oversized (priority 2), which changes the area ratio the diffuser has to work
across — the trade may look different after re-solving.

**§3.3 chamber→tailpipe cone.** Confirmed absent; see §4.4.

**§3.4 divergent nozzle.** The flight model has no nozzle expansion at all —
`throat_diameter_m` is the single aft flow dimension and is used both as the
choke area and, previously, as the "exit". Your added expansion is not something
V4 was flown with. If you want it priced, it needs to go into `fp_spec` and be
re-flown.

**§3.5 profiles with a length but no shape.** Confirmed for both. Nose fairing
is a *length allowance* (2.0 diameters) that feeds skin area for mass and
wetted area for drag — no profile is implied or used. The boattail now has a
real angle (8°) and base diameter, so it is no longer shapeless; the nose still
is.

**§3.6 other omissions.**
- *Fuel tank internal structure* — confirmed, the 50% is a flat scalar with no
  distribution. Your reading matches the model exactly.
- *Chamber thermal* — confirmed and flagged in the export README. CFRP outboard
  of steel around a combustion chamber, and no gauge anywhere reflects a
  thermal requirement. This is an unpriced risk, not a modelled one.
- *Wing station, fins* — the wing station now has a real packaging constraint
  worth knowing: the chamber occupies 428–817.3 mm at the **full body
  diameter**, so no spar can cross it. Everything aft is tank annulus. Fins are
  a live and much worse problem — see the note below.

---

## §5 Concrete JSON additions — what I added

| you asked for | status |
|---|---|
| `body.boattail_exit_outer_diameter_m` | added as `boattail_base_outer_diameter_m` = **0.153849**, not your 0.117638 — see priority 1 |
| `body.boattail_half_angle_deg` | added = **8.0** |
| `body.nose_fairing_profile` | **not added** — no profile exists to state |
| `internal_flowpath.chamber_flow_diameter_m` | **not added** — the model has one chamber diameter and it is 214.0 mm; adding 209 mm would assert something never flown |
| `chamber_to_tailpipe_cone_length_m` | **not added** — it is zero in the model; your 174.2 mm is a drawing choice, and the tank correction it implies is exported instead |
| `nozzle_throat_diameter_m` / `nozzle_exit_diameter_m` / `nozzle_expansion_area_ratio` | **not added** — the model has no expansion; renaming would imply one |
| `inlet.*` | **added**, but as a *requirement* (mass flow + implied capture area, 25 operating points) rather than geometry, because geometry does not exist |
| `design_point.*` | **added** — cutoff Mach, ramjet light Mach and altitude, top altitude, peak Mach flown, and `combustor_total_pressure_ratio: null` with its note |
| `payload.*` | **added** as `payload_and_avionics` with both fields null and an explicit "free variable" note |

The pattern: I added everything the model actually knows, and refused the keys
that would have made a drawing choice look like a simulator output.

---

## One thing you should know that was not on your list

The VSP model has found the airframe **statically unstable in both pitch and
yaw**, and fixing it needs fin area. I measured what fins cost the mission at
first-principles fidelity:

| fin area | peak Mach | cutoff | fuel / cap |
|---|---|---|---|
| 0 (frozen V4) | 1.100 | yes | 2.125 / 2.764 kg |
| 0.053 m² | 1.100 | yes | 2.295 / 2.764 kg |
| 0.120 m² | 1.100 | yes | 2.715 / 2.764 kg |
| 0.160 m² | 1.099 | **no** | dry |
| 0.21 m² | 1.082 | **no** | dry |

**The ceiling is 0.12–0.16 m² of fin**, and 0.120 m² closes on only 1.8% fuel
margin — so the practical design ceiling is lower still.

This matters to you because your §3.6 already flags the **side valve runner
fairings** as un-costed external drag, and the margin here is thin: 0.21 m² of
added wetted area is the difference between making cutoff and running dry just
short of it. Whatever those fairings turn out to be, they are drawn from the
same budget as the tail, and they should be costed before more effort goes into
the flowpath.

One methodological note that also bears on your §4.1. The **closed-form and FP
ramjets disagree fundamentally near lightoff**: at M ~0.49 the closed-form model
produces essentially zero net thrust while the FP model produces 227 N. The
closed-form sweep therefore shows a cliff at 0.055 m² that is an artifact of the
weaker engine model, not a property of the vehicle. If you ever screen anything
against `ramjet_simple` near lightoff, it will mislead you in the pessimistic
direction.

Details: `scripts/vsp_model/ANSWERS_FROM_V4_SIM.md`.
