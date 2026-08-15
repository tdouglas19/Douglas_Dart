# Douglas Dart V4 — 2D propulsion geometry contract

What the 2D integrated-propulsion section actually is, station by station,
and for every number: **where it came from**. Written 2026-08-14.

> **Revision 2, 2026-08-14** — rebuilt against the **corrected** export after
> the flight model answered
> [`docs/v4_export_requests_from_propulsion_2d.md`](../docs/v4_export_requests_from_propulsion_2d.md).
> Three things changed, and two of them were my errors:
>
> 1. **The boattail was mine to get wrong, and I got it wrong.** I inferred
>    that `drag_buildup`'s 8° was being applied to a 12.94° shape and that V4's
>    drag was therefore understated. It is not: `base_diameter_m()` *derives*
>    the base at 8° and never reads the export field. The field was the bug.
>    The aft body closes to **153.849 mm at exactly 8.00°** and does **not**
>    close onto the nozzle — an annular base remains. **V4's drag stands; no
>    re-fly.** My "unbuildable base" finding dissolved with it (18.1 mm of
>    clearance, not −1.0 mm).
> 2. **The capture rule was overturned by data.** The engine consumes
>    **39.365 cm²** at cutoff, against the 105.02 cm² my throat-area rule
>    demanded — the inlet was oversized **2.67×**. The rule now reads the
>    engine's real requirement, and the freed area went into the centrebody:
>    64.2 → **111.7 mm**, ~3× the avionics volume.
> 3. **Fixing the inlet broke the diffuser**, which is a genuine finding
>    rather than a regression. See §4.3.



Tool: `scripts/propulsion_2d/`. Input:
`out_medium_model/v4_export/structural_dimensions.json` (git `d7b0670`) plus
`scripts/propulsion_2d/assumptions.json`. **No imports of `medium_model`** —
this tool cannot silently drift with the flight code, and it cannot be
"fixed" by changing the flight code either.

---

## 1. How the geometry is represented

**Analytic segments, not a point cloud.** The vehicle is an ordered list of
typed `Segment` objects on the (x, r) half-plane. Each carries exact
endpoints, a closed-form `r_at(x)`, and a provenance tag.

Sampling every 1 mm was considered and rejected as the *model*:

- every station break is at a non-integer mm (817.307, 991.523, 2059.607,
  2273.607) — a 1 mm grid aliases all of them;
- the duct wall is 1.0 mm and the skin 1.5 mm — a 1 mm grid cannot resolve
  the wall it is meant to draw;
- the 1068 mm tailpipe needs two points, not 1068.

A uniform 1 mm sample is still produced, as an **export** (`contour.csv`),
derived from the analytic model rather than standing in for it.

### Curve laws

All but the last blend the endpoints as `r = r₀ + (r₁−r₀)·f(t)`:

| law | `f(t)` | used for |
|---|---|---|
| `line` / `cone` | `t` | cylinders, cones, boattail |
| `smoothstep` | `t²(3−2t)` | centrebody closure (zero slope both ends) |
| `parabolic_tangent` | `1−(1−t)²` | cowl outer (meets the barrel tangentially) |
| `constant_area_angle` | — closed-form in `x`, see §4.3 | the annular intake diffuser wall |

`constant_area_angle` is the only law that is not a pure function of `t`:
the duct is annular, so the wall follows the centrebody. It is still exact,
just closed-form in `x` rather than `t`.

### Chains

| chain | what | x range (mm) |
|---|---|---|
| `centrebody` | spike → barrel → closure | 0 → 428.0 |
| `cowl_inner` | annular diffuser outer wall, to the dump plane | 153.5 → 428.0 |
| `flow` | chamber → cone → tailpipe → divergent nozzle | 428.0 → 2273.6 |
| `duct_od` | `flow` + 1.0 mm steel | 428.0 → 2273.6 |
| `skin_id` | OML − 1.5 mm CFRP | 428.0 → 2273.6 |
| `oml` | cowl outer → barrel → boattail | 153.5 → 2273.6 |

`cowl_inner` deliberately does **not** meet `flow` at 428.0 — the gap between
them is the dump (§4.3).

---

## 2. Station table

`JSON` = straight from `structural_dimensions.json`. `DERIVED` = closed-form
consequence of JSON values only. `ASSUMED` = invented, lives in
`assumptions.json`, drawn dashed grey.

| station | x (mm) | x (in) | source | rule |
|---|---:|---:|---|---|
| `nose_tip` | 0.000 | 0.000 | JSON | `stations_from_nose_tip_m.nose_tip` |
| `cowl_lip` | 153.458 | 6.042 | **ASSUMED** | spike shoulder plane = capture plane |
| `centrebody_max_dia_end` | 273.458 | 10.766 | **ASSUMED** | lip + 120 mm barrel |
| `nose_fairing_end` | 428.000 | 16.850 | JSON | `nose_fairing_end__barrel_start` |
| `chamber_start` | 428.000 | 16.850 | JSON | `combustion_chamber_start` — also the **dump plane** |
| `chamber_end` | 817.307 | 32.177 | JSON | `combustion_chamber_end__tailpipe_start` |
| `cone_end` | 991.523 | 39.036 | **ASSUMED** | `chamber_end` + 174.216 mm at 15° |
| `tailpipe_end` | 2059.607 | 81.087 | JSON | `tailpipe_end` / `boattail_start` |
| `nozzle_throat` | 2263.075 | 89.097 | **ASSUMED** | `nozzle_exit` − 10.532 mm of divergence |
| `nozzle_exit` | 2273.607 | 89.512 | JSON | `nozzle_exit__body_end` |

The nose fairing ends exactly where the chamber starts — one station, two
names. The drawing merges the label.

---

## 3. Architecture

Nose inlet with an external-compression **centrebody** (user directive,
2026-08-14), feeding an annular diffuser that **dumps** into a
full-body-diameter combustion chamber, contracts through a cone, runs a
constant-diameter tailpipe, and expands through a short divergent nozzle.
Fuel lives in the annulus around the tailpipe. The aft body closes at 8° to a
base **wider than the nozzle**, leaving an annular base.

```
   spike         annular diffuser  ||  chamber      cone      tailpipe     div  boattail
 |----------|--------------------|DUMP|---------|--------|--------------|--|
 0        153.5                 428.0        817.3    991.5        2059.6  2263 2273.6
```

**On the centrebody:** at the vehicle's M 1.10 peak, a normal shock already
recovers 99.9% of total pressure, so a spike buys ~0.1%. Its justification on
this vehicle is **packaging** — annular capture frees a 111.7 mm diameter ×
120 mm nose volume (2.24 L) for guidance/avionics — not compression.
Shock-on-lip is therefore *not* used to place the lip: the vehicle never
reaches a Mach where a 20° cone throws an attached shock. The lip sits in the
spike shoulder plane, which is a clean, dimensionable geometric rule instead.

That packaging argument got much stronger once the inlet was correctly sized:
the centrebody nearly doubled in diameter and roughly tripled in volume
without changing the cowl at all (§4.1–4.2).

---

## 4. Every assumption, and why

All of these live in `scripts/propulsion_2d/assumptions.json`. Edit that
file to re-shape the vehicle; nothing else reads them.

### 4.1 Centrebody — sized for avionics

| knob | value | why |
|---|---|---|
| cone half-angle | 20° | conventional low-supersonic spike angle |
| max diameter | 0.522 × body = **111.708 mm** | see below |
| barrel length | **120 mm** | avionics can |
| closure | smoothstep to r=0 at the chamber head | full-area entry to the dump plane |

**The centrebody is the design variable and the lip is solved from it** (user
directive: size it for avionics). 0.522 is the value that keeps the lip at the
132.26 mm it had under the old throat-area rule — i.e. it takes the entire
2.67× inlet oversizing the flight model found and spends it on avionics volume
rather than shrinking the cowl.

Internal volume available: **2.24 L**. Spike length falls out: 55.854 / tan 20°
= **153.458 mm**.

The flight model states avionics volume is a **free variable**
(`payload_and_avionics.avionics_volume_m3` is `null`, against a 2.0 kg fixed
mass allowance), so nothing here overrides a requirement — there isn't one. If
one appears, set this knob from it and the lip follows.

### 4.2 Capture area rule — `annulus_equals_required_mass_flow`

The lip diameter is **solved**, not chosen:

```
π (r_lip² − r_cb²) = A_capture   →   r_lip = sqrt(r_cb² + A_capture/π)
```

`A_capture` now comes from the export's `inlet.implied_capture_area_m2` —
what the FP ramjet **actually consumed**, derived from its own equivalence
ratio and fuel flow via `mdot_air = mdot_fuel × 15.7 / φ`:

| condition | air flow | capture area |
|---|---:|---:|
| lightoff, M 0.488 | 0.752 kg/s | 37.6 cm² |
| **cutoff, M 1.092** | **1.754 kg/s** | **39.4 cm²** |

Remarkably flat across the burn, so a single design point is defensible here
in a way it usually is not.

**The superseded rule** (`annulus_equals_throat_area`, still selectable) set
capture = throat area = 105.02 cm². That **oversized the inlet 2.67×**. Kept
only so the earlier drawing can be reproduced, and guarded by a test so the
oversizing stays visible.

The export calls its area a **floor**: full stream-tube capture, no spillage,
and **no strut blockage** — which remains my own open item (§6).

### 4.3 Intake diffuser — `constant_angle_then_dump`

**Correctly sizing the inlet made the diffuser harder, not easier.** Capture
39.4 cm² into a 343.1 cm² chamber head is an area ratio of **8.72** (it was
3.27 when the inlet was 2.67× too big), and over the available 274.5 mm that
is a **14.13°** equivalent cone — far past separation. No wall shape fixes it:
both areas and the length are fixed.

Diffusing all the way at 7° would need **562.8 mm** of diffuser, i.e. **+288 mm
of nose fairing** (2.00 → 3.35 D). That is not a trade worth making.

So the drawing does what a ramjet with a flameholder does anyway: **diffuse at
7° as far as the nose allows, then dump.**

| | value |
|---|---:|
| diffuser wall area ratio | 3.811 |
| equivalent half-angle (uniform) | **7.00°** |
| dump plane diameter | 138.215 mm |
| **dump sudden-expansion ratio** | **2.29×** |

The area distribution is therefore **genuinely discontinuous** at
`chamber_start`. That step is a real feature, not a drawing artifact, and the
tests assert it is there and has the right size rather than smoothing it over.

Set `diffuser_policy: "full"` to diffuse the whole way instead and see the
14.13° for yourself.

**The wall law** is `constant_area_angle`. The duct is annular, so a plain
shape law on the wall alone does not control the thing that matters (area).
This law grows the equivalent-cone radius `sqrt(A/π)` linearly:

```
r_cowl(x) = sqrt( R_eq(x)² + r_cb(x)² )
```

It matters: `smoothstep` bunches all the area growth into the middle and
peaked at 13.1° even under the *old*, easier area ratio.

### 4.4 Chamber → tailpipe cone — 15° half-angle

`structural_dimensions.json` states outright that the mass model gives this
transition **zero length**. A 214 → 115.6 mm step over 0 mm is not a shape.
At 15° the cone is **174.216 mm** (817.307 → 991.523).

Consequence, reported and not hidden: the cone is fatter than the pipe, so
it eats fuel annulus. See §5.

### 4.5 Wall stack-up

The source data is internally inconsistent about whether its diameters are
flow IDs or outer surfaces, and both readings are provably in use:

- `frontal_area_m2 = π/4 × 0.214²` → **214 mm is the OML.**
- `throat_to_body_area_fraction = (115.638/214)²` → **115.638 mm is a flow
  area.**

So they are read differently, deliberately:

| region | reading | flow ID | outer |
|---|---|---:|---:|
| chamber | 214 = OML | 209.0 mm (−1.5 CFRP −1.0 steel) | 214.0 mm |
| tailpipe | 115.638 = flow ID | 115.638 mm | 117.638 mm (+1.0 steel) |

In the chamber the liner **is** the body skin (V4 deleted the annular gap),
modelled as CFRP outboard of steel — which is exactly what makes the thermal
problem the export README flags visible in the drawing.

Neither reading is a free choice, so neither is really an assumption:
`_check_wall_readings()` re-derives both from the JSON on every build and
**raises** if either stops holding. A future export that changes convention
fails loudly instead of quietly drawing the wall on the wrong side.

### 4.6 Profiles the model does not specify

| feature | model gives | assumed |
|---|---|---|
| nose fairing | length only (428 mm) | `parabolic_tangent`, tangent to the barrel |
| cowl lip | nothing | 3.0 mm thickness |
| boattail | length, base dia **and** angle | nothing — fully specified by the corrected export |

### 4.7 The boattail — corrected, and my error

**Revision 1 of this document was wrong here, and the error is worth keeping
on the record.** I read `boattail_exit_diameter_m = 115.638` (the tailpipe
*flow* diameter), concluded the aft body closed onto the nozzle at 12.94°,
found the 1.0 mm duct wall poking outside the mould line over the last
4.35 mm, and inferred that `drag_buildup`'s 8° term was being applied to a
12.94° shape — so V4's drag was understated and needed a re-fly.

**The inference was wrong.** `medium_model.drag_buildup.base_diameter_m()`
*derives* the base diameter at 8° and never reads the exported field:

```python
shrink = 2.0 * tail_length_m * tan(radians(BOATTAIL_HALF_ANGLE_DEG))
return max(body_diameter_m - shrink, duct_exit_diameter_m)   # -> 153.849 mm
```

The model applied an 8° term to an 8° shape it computed itself. It was
internally consistent all along; the **export field** was the bug, and it has
since been corrected. **V4's drag stands and no re-fly is needed.**

| | rev 1 (wrong) | corrected export |
|---|---:|---:|
| base outer diameter | 117.638 mm | **153.849 mm** |
| boattail half-angle | 12.688° | **8.0000°** |
| duct clearance at the base | −1.0 mm (called unbuildable) | **+18.1 mm** |

**The aft body does not close onto the nozzle.** An annular base remains, and
base drag on it is a real term in the flown build-up:

| | area |
|---|---:|
| export, engine on (measured to the nozzle flow dia) | 80.87 cm² |
| export, engine off | 185.90 cm² |
| as drawn (to the duct OD, with AR 1.10 expansion) | **66.53 cm²** |

Note the 2.3× engine-off/engine-on ratio, and that ~140 s of this mission are
unpowered — already priced in the flown build-up.

**What the containment test got right:** it did find a real inconsistency, and
it is unchanged and still guarding. It pointed at the geometry when the fault
was in the export; the test was sound, my interpretation was not.

---

## 5. Model vs as-drawn

Where giving the geometry a real shape disagrees with the model's own
bookkeeping. None of this is an error in either — it is the cost of turning
a mass model into a drawing.

| quantity | model | as drawn | Δ | why |
|---|---:|---:|---:|---|
| chamber flow diameter | 214.000 mm | 209.000 mm | −2.3% | model quotes the OML; as-drawn subtracts the wall stack |
| chamber volume | 14.003 L | 13.356 L | −4.6% | same cause |
| tailpipe length | 1242.300 mm | 1282.084 mm | +3.2% | cone gets a real length; pipe runs to the exit plane |
| **fuel annulus volume** | 31.636 L | 28.036 L | **−11.4%** | the cone is fatter than the pipe, and true walls shrink the gap |
| **fuel capacity** | 7.798 kg | 6.911 kg | **−11.4%** | 50% of the annulus, same rule as the model |
| nozzle exit flow diameter | 115.638 mm | 121.282 mm | +4.9% | the model has no divergence — what it calls the exit is the **throat**, unchanged; this is the added expansion |
| annular base area | 80.874 cm² | 66.530 cm² | −17.7% | the export's annulus assumes no expansion; the divergent exit eats into it, which **reduces** base drag |
| fuel loaded | 3.071 kg | 3.071 kg | 0 | an input, not a geometry output |
| spare annulus aft of the barrel | — | 3.221 L | — | exists but the model's tank stops at the boattail start |

**The fuel result is safe.** Capacity falls to 6.911 kg against 3.071 kg
loaded — 2.25× margin, and the *flown burn* is only 2.125 kg, so 3.25×. The
11.4% is worth knowing, not worth acting on. The flight model has taken this
correction into its export.

**The boattail is settled** — 8.0000°, matching `drag_buildup` exactly. See
§4.7 for the error that used to live here.

---

## 6. Not modelled, at all

Drawn as nothing. Listed so the omissions are explicit.

- **Centrebody support struts.** A nose centrebody must be carried across the
  annular diffuser by struts or a strut ring. No strut geometry here, and no
  strut blockage in any area number in this tool.
- **Side valve runners.** The pulsejet's reed-valve runners still do not fit
  inside a 214 mm skin (export README). Out of scope for this drawing.
- **Flameholder / reed-valve stations.** `medium_model.fp_spec` carries both,
  but `structural_dimensions.json` does not, and this tool stays pure to its
  JSON input.
- **Wing and fins.** Out of scope (propulsion + OML only). The model has no
  fin geometry at all and no longitudinal station for the wing.
- **Fuel tank internal walls.** The model says 50% of the annulus is usable
  but not how the other 50% is distributed. The annulus is drawn full and the
  50% applied as a scalar, exactly as the model does.
- **Everything structural, thermal and CG-related** — see the export README.

---

## 7. Running it

```bash
.venv/Scripts/python -m scripts.propulsion_2d --show
```

| flag | effect |
|---|---|
| *(none)* | print the tables and exit — the fast "what does the geometry say about X" path |
| `--show` | open the interactive viewer |
| `--export` | write `stations.csv`, `contour.csv`, `flow_area.csv`, `geometry.json` |
| `--save-vector` | write SVG + PDF + PNG without opening a window |
| `--dims PATH` | build from a different `structural_dimensions.json` |
| `--assumptions PATH` | swap the assumptions file |
| `--contour-step-mm` | uniform sample step for the CSV export (default 1.0) |

### In the viewer

| input | does |
|---|---|
| left-click | pin a datatip; snaps to the nearest contour and to a named station within 6 mm |
| right-click | remove the nearest datatip |
| `c` | clear all datatips |
| `s` | save SVG + PDF + PNG |
| `h` | help |
| `1`–`8` | toggle layers: oml, centrebody, diffuser, flow, walls, fuel, stations, dims |

The toolbar readout gives station, radius, diameter, local flow area,
A/A_body, A/A_throat and which region of the engine the cursor is in. The
geometry underneath is analytic, so zooming reveals the 1.0 mm duct wall
rather than blurring it.

### Outputs

Written to `out_medium_model/propulsion_2d/`:

| file | what |
|---|---|
| `stations.csv` | the exact named breakpoints, full float precision |
| `contour.csv` | every chain sampled uniformly (default 1 mm) — the point cloud |
| `flow_area.csv` | area distribution: x, region, radius, area, A/A_body, A/A_throat |
| `geometry.json` | the segment model itself, for other tools — self-describing (a law-shaped segment stores the law name and its constants, not a serialised function), so rebuild by re-applying the formula, or just re-run the build (well under a second) |
| `propulsion_2d_section.{svg,pdf,png}` | vector + raster renders |

---

## 8. Open questions

**Closed by the flight model, 2026-08-14** (see
[`docs/v4_export_answers_from_flight_model.md`](../docs/v4_export_answers_from_flight_model.md)):

- ~~Boattail angle~~ — **8.0°, settled.** The export field was the bug, not the
  drag model. §4.7.
- ~~Centrebody size~~ — avionics volume is a **free variable** (2.0 kg fixed
  mass, no volume requirement), so the centrebody was sized to absorb the
  freed inlet area: 64.2 → 111.7 mm, 2.24 L. §4.1.
- ~~Capture area~~ — the engine needs **39.4 cm²**, not the 105.0 cm² the
  throat rule assumed. §4.2.
- ~~Does the thrust model assume a choked nozzle?~~ — **No**, on either path.

**Still open:**

1. **Combustor total pressure vs Mach** — the one number that would turn the
   nozzle expansion from an assumption into a calculation (§4.4). Not
   currently exported; the flight model has offered to add it to the trace and
   re-fly (~35 min). **Worth taking** — it is cheap and it is the last thing
   standing between the nozzle sizing and a real answer.
2. **Strut count and blockage.** The capture area is explicitly a *floor*
   because it assumes full stream-tube capture and no strut blockage. A nose
   centrebody must be carried by struts. Until this exists, the 39.4 cm² is a
   lower bound, and so is the lip diameter derived from it.
3. **Diffuser 7° + 2.29× dump, or a longer nose?** Diffusing all the way at 7°
   needs +288 mm of nose (2.00 → 3.35 D), which is not worth it — but the dump
   loss is unmodelled here and unmodelled in the flight code, so neither side
   currently prices this.
4. **Nozzle expansion ratio.** AR 1.10 as drawn. It is thermodynamically
   near-pointless (optimum 1.0015) and its only real effect is shrinking the
   annular base by 17.7%. Question 1 would settle it.
5. **Side valve runner fairings.** Still un-costed external drag — and the
   flight model has since warned that the fin budget is thin enough (0.12–0.16
   m² ceiling) that added wetted area is the difference between making cutoff
   and running dry. These fairings draw on the same budget.
4. **Strut count and blockage** — needed before any capture-area number here
   is trustworthy.
