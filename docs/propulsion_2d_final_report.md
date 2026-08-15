# Douglas Dart V4 — 2D propulsion section: final report

**2026-08-14.** What was built, what it found, and exactly which numbers you
may lean on.

Two configurations exist and both are kept:

| | tool | status |
|---|---|---|
| **A — baseline** | `python -m scripts.propulsion_2d` | frozen at commit `8df8b12`; interactive viewer |
| **B — translating inlet** | `python scripts/propulsion_2d_full_drawing.py` | the design; adds a ramjet shut-off valve |

B imports A's geometry kernel but builds its own contours. A test asserts B
cannot mutate A, and A's five source files are byte-identical to the freeze.

---

## 1. How the geometry is represented

**Analytic typed segments, not a sampled point cloud.** Each piece carries
exact endpoints, a closed-form `r_at(x)`, and a provenance tag
(`JSON` / `DERIVED` / `ASSUMED`).

Point-per-mm was considered and rejected as the *model*: every station break
is at a non-integer mm (817.307, 991.523, 2059.607, 2273.607), and the duct
wall is 1.0 mm — a 1 mm grid can resolve neither. A uniform 1 mm sample is
still produced as an **export** (`contour.csv`), derived from the model, with
the exact stations merged in as rows.

Neither configuration imports `medium_model`. Both read
`out_medium_model/v4_export/structural_dimensions.json` and nothing else, so
they cannot silently drift with the flight code.

---

## 2. Configuration A — the baseline (frozen)

Nose inlet with an external-compression centrebody, annular diffuser dumping
into a full-body-diameter combustor, contraction cone, tailpipe, short
divergent nozzle, 8° boattail onto an annular base.

| | value | source |
|---|---:|---|
| body diameter | 214.000 mm | JSON |
| overall length | 2273.607 mm | JSON |
| capture area | 39.365 cm² | JSON (`inlet.implied_capture_area_m2`) |
| centrebody | 111.708 mm, 2.24 L | ASSUMED |
| chamber flow dia | 209.000 mm | DERIVED (214 OML − skin − liner) |
| diffuser | 7.00° to a 138.2 mm dump plane, then **2.29× dump** | ASSUMED |
| contraction cone | 15°, 174.216 mm | ASSUMED |
| throat | 115.638 mm (105.02 cm²) | JSON |
| nozzle expansion | 1.10 area ratio → 121.282 mm exit | ASSUMED |
| boattail | **8.0000°** to a 153.849 mm base | DERIVED |
| annular base | 66.53 cm² as drawn | DERIVED |

Interactive viewer: cursor readout (station, diameter, flow area, A/A_throat,
region), click-to-pin datatips that snap to named stations, layer toggles,
linked flow-area panel. Contract: `scripts/PROPULSION_2D_GEOMETRY.md`.

---

## 3. Configuration B — the translating inlet shut-off

**Purpose:** shut the ramjet intake while the pulsejet runs.

### 3.1 Why it works

For a fixed capture area the annulus thins as it moves outboard:

```
A = π(r_lip² − r_fore²) = π(r_lip + r_fore)·h        →   h ≈ A / 2πR
stroke = h / tan(θ_seal)
```

Both levers multiply. Moving the annulus out and using a 45° seat instead of
the 20° spike angle takes the closing stroke from **28.22 mm to 7.88 mm**.

### 3.2 Architecture (outer mould line fixed — user directive)

```
  boom      fixed nose fairing        translating sleeve
   ---o=====[ avionics, on a spar ]==|SLOT|===================
   -160       0                    300                    428
```

- **Fixed nose fairing** carries the avionics — no harness crosses a moving
  joint. It is part of the OML and never moves.
- **Annular slot** at its shoulder is the capture plane.
- **Translating sleeve** rides the spar inside the fairing's aft end. Its 45°
  cone seats against a matching cone on the cowl lip.
- Nothing outside the cowl line moves. A test asserts the fairing and cowl
  contours are bit-identical in both states.

### 3.3 Final numbers

| | value |
|---|---:|
| slot station / lip radius | 300.0 mm / 83.46 mm |
| annulus radial gap | 7.88 mm |
| **stroke, open to sealed** | **7.88 mm** |
| seal half-angle | 45° |
| **faying contact** | **95.4 cm²** (12 mm land → 17.0 mm slant) |
| headroom, seat top to body | 11.54 mm |
| fixed nose volume (avionics) | 1.79 L |
| diffuser length | 128.0 mm |
| dump | 82.08 → 343.07 cm², **4.18×** |
| 90% / 99% shut at | 7.12 / 7.80 mm of the 7.88 mm stroke |

The closing is near-linear in stroke — 90% of the area is gone only in the
last 10% of travel. There is no soft landing: the final 0.08 mm does the last
1%. If the actuator overshoots, the seat takes the impact. A shallower angle
near the crest would cushion it, at the cost of a longer stroke.

Added features, all ASSUMED (the export has geometry for none of them):

| | value | sizing rule |
|---|---|---|
| pitot-static boom | 160 × 10 mm | statics at 9 probe diameters aft of the tip |
| ramjet flameholder | annular V-gutter, x 537 mm, r 62.7 mm, 25 mm | 28% into the chamber, **29% blockage** |
| pulsejet reed valves | 2 × 57 mm side runners, 51.0 cm² | 15% of chamber area, at the chamber head |

Body length is unchanged at 2273.607 mm; with the boom, 2433.607 mm.

---

## 4. What this work found

### 4.1 Two of my own findings were wrong, and were corrected

**The boattail.** I read `boattail_exit_diameter_m = 115.638` (the tailpipe
*flow* diameter), concluded the aft body closed at 12.94°, found the duct
poking outside the mould line, and inferred `drag_buildup`'s 8° term was being
applied to a 12.94° shape — so V4's drag was understated.

The inference was wrong. `base_diameter_m()` *derives* the base at 8° and
never reads the exported field. The model applied an 8° term to an 8° shape it
computed itself. **V4's drag stands; no re-fly.** The export field was the bug
and has been corrected to 153.849 mm. My "unbuildable base" finding dissolved
with it — 18.1 mm of clearance, not −1.0 mm.

The containment test was sound and is unchanged. What was wrong was treating
the export as ground truth about the flight model, when it was a lossy report
of it.

**The capture rule.** My rule set capture = throat area = 105.02 cm². The
flight model then supplied what the engine actually consumes — 39.4 cm² at
cutoff, from the FP ramjet's own φ and fuel flow. **The inlet was oversized
2.67×.**

### 4.2 Findings that stand

**Correctly sizing the inlet made the diffuser harder, not easier.** Capture
39.4 cm² into a 343.1 cm² chamber is an area ratio of 8.72, which over the
available length is a 14.13° equivalent cone — far past separation, and no
wall shape fixes it because both areas and the length are fixed. Diffusing all
the way at 7° would need +288 mm of nose. So both configurations do what a
ramjet dump combustor does: diffuse at 7°, then dump. **The area distribution
is genuinely discontinuous** at the chamber head, and the tests assert the
step rather than smooth it.

**A real chamber→tailpipe cone costs 11.4% of fuel annulus volume** —
capacity 7.798 → 6.911 kg. Still 3.25× the 2.125 kg actually burned, so a
number to know, not to act on.

**The nozzle expansion is thermodynamically near-pointless.** The nozzle only
chokes above M 1.061; the ramjet lights at M 0.488 and cutoff is M 1.100, so
it is unchoked for nearly the whole burn, and the optimum area ratio at M 1.10
is 1.0015. Its only real effect is shrinking the annular base 17.7%, which
reduces base drag. **The flight model has no expansion and V4 was not flown
with one.**

**The pulsejet may be the real reason to want the shut-off.** It shares this
duct with its reed valves at the chamber head. With the nose inlet open, its
blowdown can vent forward out of the nose. Closing it gives a proper closed
head end — plausibly worth more than the spillage drag it also saves. This is
a hypothesis from the geometry, **not a modelled result**.

### 4.3 Two faults the automated checks caught

Both were found by tests, not by eye, and both are now guarded:

1. **The duct left the outer mould line** by ~2.5 mm over x 300–315 mm. The
   sleeve rises 7.88 mm there (that rise *is* the seal stroke), so the outer
   wall needed 6.6 mm of growth in 7 mm while the cowl only grew 1.3 mm. Fixed
   with a fast-rising cowl lip plus a hard clamp to `cowl_outer − wall`.
2. **The faying land necked the duct to 15.8 cm²** — 40% of capture, which
   would have choked the inlet. When retracted, the land sits out in the open
   flow path as a 12 mm radial protrusion the cowl must clear. Fixed by
   pulling the lip inboard from 0.86 → 0.78 of body radius.

Both checks run on every invocation and print their result.

---

## 5. Which numbers you may lean on

| number | provenance |
|---|---|
| body 214.0, length 2273.607, all five JSON stations | **flown** |
| throat 115.638 mm / 105.02 cm², area fraction 0.29199 | **flown** |
| boattail 8.0°, base 153.849 mm, base annulus 80.87 cm² | **flown** |
| fuel loaded 3.071 kg, burned 2.125 kg | **flown** |
| capture 39.365 cm² | **flight-model output**, but *implied* — back-calculated from mass flow, and explicitly a **floor** (no spillage, no strut blockage) |
| chamber flow 209.0 mm / 343.07 cm² | **derived** — the export quotes 214 mm, which is the OML |
| everything about the inlet, diffuser, dump, nozzle expansion, boom, flameholder, reed valves, valve stroke, faying area | **assumed** — no geometry for any of it exists in the model |

The single most important line: **the model has no inlet, no diffuser and no
duct forward of the chamber at all.** Every area in configuration B forward of
x = 428 mm is geometry I invented to satisfy one flown number (39.4 cm²).

---

## 6. Open items

1. **Combustor total pressure vs Mach** is not exported. It is the one number
   that would turn the nozzle expansion from an assumption into a calculation.
   The flight-model session has offered to add it to the `FpPropulsion` trace
   and re-fly (~35 min). **Worth taking** — it is the cheapest item here.
2. **Strut count and blockage.** The fixed fairing must be carried on struts
   crossing the annular diffuser. Until they exist, 39.4 cm² is a lower bound
   and so is every area derived from it.
3. **Dump loss is unmodelled** — here and in the flight code. The 4.18× dump
   in configuration B is larger than A's 2.29×, and neither side prices it.
4. **The seal is geometry only** — no seat land width tolerance, no thermal
   growth allowance, no actuator, no contact stress.
5. **Fin budget.** The flight session reports the airframe statically unstable
   in pitch and yaw, with a fin ceiling of 0.12–0.16 m²; 0.120 m² closes on
   only 1.8% fuel margin. The un-costed side valve runner fairings — now drawn
   — draw on that same budget.
6. **Trade still on the table.** 124 cm² of faying is achievable at lip 0.74 /
   16 mm land, at the cost of 8.35 mm stroke and further nose volume. Run
   `--lip-frac 0.74 --faying-mm 16`.

---

## 7. Files

**Tools**

| path | what |
|---|---|
| `scripts/propulsion_2d/` | geometry kernel, assumptions, interactive viewer, CLI |
| `scripts/propulsion_2d/translating_inlet.py` | valve kinematics and the stroke solve |
| `scripts/propulsion_2d_translating_study.py` | parametric stroke study |
| `scripts/propulsion_2d_full_drawing.py` | the full section, configuration B |

**Documents**

| path | what |
|---|---|
| `scripts/PROPULSION_2D_GEOMETRY.md` | geometry contract for configuration A |
| `docs/v4_export_requests_from_propulsion_2d.md` | asks sent to the flight-model workspace |
| `docs/v4_export_answers_from_flight_model.md` | its replies |
| this file | final report |

**Outputs** (`out_medium_model/propulsion_2d/`)

`propulsion_2d_section.{png,svg,pdf}` (A),
`propulsion_2d_full_translating.{png,svg,pdf}` (B),
`translating_inlet_study.{png,svg}`,
`stations.csv`, `contour.csv`, `flow_area.csv`, `geometry.json`.

**Tests** — `tests/test_propulsion_2d.py` (56) and
`tests/test_translating_inlet.py` (24), ~17 s combined, no FP dependency.

---

## 8. Running it

```bash
.venv/Scripts/python -m scripts.propulsion_2d --show          # A, interactive
.venv/Scripts/python scripts/propulsion_2d_full_drawing.py    # B, the design
```

Bare invocation prints tables and exits. `--export` writes the CSVs,
`--save-vector` writes SVG/PDF/PNG. In the viewer: left-click pins a datatip,
right-click removes, `c` clears, `1`–`8` toggle layers, `h` for help.
