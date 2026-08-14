# Requests to the V4 export, from the 2D propulsion section

> **ANSWERED 2026-08-14** → [`v4_export_answers_from_flight_model.md`](v4_export_answers_from_flight_model.md).
> Kept as sent, unedited, so the exchange reads in order. **Two of the findings
> below turned out to be wrong**, and the answers doc is authoritative where
> they differ:
>
> - **§2.1 / §1.1 (boattail, and the "unbuildable" base) — WRONG.**
>   `drag_buildup.base_diameter_m()` *derives* the base at 8° and never reads
>   the exported field, so the 8° term was applied to an 8° shape all along.
>   V4's drag is **not** understated and no re-fly is needed. The export field
>   was the bug; corrected to 153.849 mm. With a 153.849 mm base the duct has
>   18.1 mm of clearance, so nothing was ever unbuildable.
> - **§3.1 capture rule — SUPERSEDED.** The engine consumes 39.4 cm², not the
>   105.0 cm² my throat-area rule assumed: the inlet was oversized **2.67×**.
> - §1.2, §1.3, §3.3, §3.5, §3.6, §4.1, §4.4 — confirmed as written.
> - §3.2 (diffuser) — the flight model cannot comment, and the correctly-sized
>   inlet makes it *harder*, not easier. See the geometry contract §4.3.

**For the `medium_model` / V4-export workspace.** Paste-ready.

Source of these findings: `scripts/propulsion_2d/`, which builds a to-scale
longitudinal section of the integrated propulsion system from
`out_medium_model/v4_export/structural_dimensions.json` (git `d7b0670`) and
nothing else. It imports no `medium_model` code, so everything below is a
statement about the **exported numbers**, not about the flight code.

Three categories: **defects** (§1–2, the export is wrong or self-contradictory),
**missing parameters** (§3, I had to invent them), and **questions only the
flight model can answer** (§4). §5 is a concrete proposed JSON addition.

---

## 1. Defects in the export

### 1.1 The boattail exit diameter cannot be built — HIGH

```
body.boattail_exit_diameter_m        = 0.115638   <-- equals the tailpipe
internal_flowpath.tailpipe_diameter_m = 0.115638       FLOW diameter
materials_and_gauges.duct_wall_thickness_m = 0.001
```

`throat_to_body_area_fraction = (115.638/214)²` proves 115.638 mm is a **flow**
diameter. So the duct's *outer* diameter is 117.638 mm. Closing the skin onto
115.638 mm therefore puts the steel duct **outside the outer mould line over
the last 4.35 mm** before the base plane. The geometry as exported is not
manufacturable.

Found by an automated containment test, not by eye.

**What I did:** opened the base OD to 117.638 mm and left the flow area
untouched (it sets thrust). Half-angle 12.94° → 12.69°.

**Ask:** decide whether `boattail_exit_diameter_m` is meant to be an OML or a
flow diameter, and export both. Suggested keys in §5.

### 1.2 The export mixes OML and flow conventions — MEDIUM

Both readings are provable, and they disagree with each other:

| key | value | proven to be | proven by |
|---|---|---|---|
| `body.diameter_m` | 0.214 | an **OML** | `frontal_area_m2 = π/4 × 0.214²` |
| `internal_flowpath.tailpipe_diameter_m` | 0.115638 | a **flow** dia | `throat_to_body_area_fraction = (115.638/214)²` |
| `internal_flowpath.chamber_diameter_m` | 0.214 | quoted as the body | — ambiguous |

Because the chamber is quoted at the body diameter, `chamber_volume_m3`
implicitly assumes **zero wall thickness**. Subtracting the export's own
gauges (1.5 mm CFRP + 1.0 mm steel) gives a 209.0 mm bore:

| | export | with the export's own walls | Δ |
|---|---:|---:|---:|
| chamber flow diameter | 214.000 mm | 209.000 mm | −2.3% |
| chamber volume | 14.003 L | 13.356 L | **−4.6%** |

**Ask:** state the convention per key, or export flow and outer diameters
separately everywhere.

### 1.3 The fuel annulus volume is optimistic by 11.4% — MEDIUM

`fuel_system.annulus_volume_m3 = 0.0316359` treats the whole tail section as
throat-sized and ignores wall thickness. Two corrections:

- the chamber→tailpipe cone is **fatter than the pipe**, so it displaces tank
  volume the formula credits;
- the real gap is duct **OD** to skin **ID**, not flow dia to body dia.

| | export | as drawn | Δ |
|---|---:|---:|---:|
| annulus volume | 31.636 L | 28.036 L | **−11.4%** |
| tank capacity @ 50% | 7.798 kg | 6.911 kg | **−11.4%** |

**This does not threaten the mission** — 6.911 kg still covers the 3.071 kg
loaded with 2.25× margin. It is a number to correct, not a problem to solve.

There is also **2.33 L of annulus aft of `tailpipe_end`** that the model's
tank does not count, if you ever want it.

---

## 2. Conflict with `medium_model` itself

### 2.1 Boattail angle: the drag model and the geometry disagree — HIGH

```
medium_model/drag_buildup.py:156   BOATTAIL_HALF_ANGLE_DEG = 8.0
V4 export geometry                 12.94 deg  (214 mm closing to 115.638 mm)
```

The export's own length and exit diameter imply **12.94°**, not 8°. Over this
48.2 mm of radial drop, an 8° boattail needs **343 mm** of length — 129 mm more
than the 214 mm budgeted.

12.94° is past the ~10–12° where boattail separation normally begins, so if
`drag_buildup` is applying an 8° boattail-drag term to a 12.94° shape, **the V4
drag is understated** and the whole V4 result inherits it.

**Ask (this is the one I would prioritise):** which is authoritative? Either
- lengthen the boattail toward 343 mm (costs overall length), or
- keep 214 mm and make `drag_buildup` use the real angle, then re-fly V4.

---

## 3. Parameters that do not exist in the export

Everything here I had to invent. All of it is tagged `ASSUMED` and drawn dashed
grey; all of it lives in `scripts/propulsion_2d/assumptions.json`. Where the
export *could* supply a real number, I would rather use yours.

### 3.1 The inlet — the biggest hole

`structural_dimensions.json` contains **no inlet geometry whatsoever**: no
capture area, no lip station, no lip diameter, no diffuser, no centrebody.
The vehicle has a ramjet; the export does not say where its air comes in.

| what | what I assumed | what I would rather have |
|---|---|---|
| architecture | nose inlet, external-compression centrebody (your directive) | confirmed |
| capture area | **solved** so annular capture = throat area exactly | the ramjet's actual **required mass flow** at the design point, or a capture area |
| lip station | spike shoulder plane, x = 88.194 mm | — |
| lip diameter | 132.264 mm (falls out of the capture rule) | — |
| lip thickness | 3.0 mm | — |
| diffuser wall | `constant_area_angle` (see §3.2) | — |
| centrebody | 64.2 mm dia × 120 mm barrel, 20° cone | **the real avionics/payload volume** — you said size it for avionics, so this is now the top ask |
| struts | **none modelled** | strut count and blockage fraction; no capture-area number is trustworthy without it |

### 3.2 The intake diffuser is length-limited — worth knowing

With capture area and chamber-head area both fixed, and the diffuser length
fixed at 339.8 mm by the 428 mm nose fairing, the **best achievable**
equivalent-cone half-angle is **7.82°** — marginally above the ~7° rule of
thumb. No wall shape does better; I verified a naive shape law peaks at 13.1°
and separates, and the `constant_area_angle` law achieves the 7.82° floor
uniformly.

**Length is the only lever: +40.4 mm of nose fairing** (428.0 → 468.4 mm, 2.00 →
2.19 D) clears 7°. That is 1.8% of overall length.

**Ask:** is 40 mm of extra nose affordable in the mass/drag budget? If yes it
is cheap insurance on inlet recovery, which nothing in this repo currently
models.

### 3.3 The chamber→tailpipe cone

The export says outright: *"the chamber-to-tailpipe cone has zero length in the
model. Give it a real length in CAD; the model does not tell you what it should
be."*

I assumed **15° half-angle → 174.2 mm**. Consequence is §1.3's fuel volume.

**Ask:** is there a length or pressure-loss budget for it? If the mass model
grows a real cone, §1.3's numbers change again.

### 3.4 The divergent nozzle — added on your directive, 2026-08-14

The export has **no divergence**: `nozzle_exit_diameter_m` equals
`tailpipe_diameter_m`, so what it calls the nozzle exit is really the **throat**.

I added expansion aft of the throat. The throat, `throat_to_body_area_fraction`
and the inlet capture rule are all untouched.

**The sizing result is worth reading carefully** (quasi-1D isentropic,
γ = 1.3, 0.90 total-pressure recovery):

| | value |
|---|---|
| nozzle chokes only above | **M 1.061** |
| V4 ramjet lights at | **M 0.488** |
| V4 cutoff at | **M 1.100** |
| optimum area ratio at M 1.10 | **1.0015** — essentially none |

So the nozzle is **unchoked for nearly the entire ramjet burn**, and below
choking a divergent section acts as a subsonic diffuser and *reduces* thrust.
The thermodynamic case for expanding is close to zero.

The real case is **external** — a bigger exit lets the boattail close less
steeply, which bears directly on §2.1:

| area ratio | exit dia | divergent length | boattail half-angle |
|---:|---:|---:|---:|
| 1.00 | 115.64 mm | 0.00 mm | 12.69° |
| **1.10** | **121.28 mm** | **10.53 mm** | **11.97°** ← as drawn |
| 1.20 | 126.68 mm | 20.60 mm | 11.27° |
| 1.40 | 136.82 mm | 39.53 mm | 9.96° |
| 1.72 | 151.66 mm | 67.21 mm | 8.03° |
| **1.724** | **151.85 mm** | **67.57 mm** | **8.00°** ← exactly `drag_buildup` |

**Ask:** AR **1.724** would make the geometry agree with
`BOATTAIL_HALF_ANGLE_DEG = 8.0` **without lengthening the vehicle at all** — it
buys the 8° boattail out of the 214 mm already budgeted, instead of the 343 mm
§2.1 would otherwise need. It is badly over-expanded internally, but the nozzle
is unchoked for most of the mission anyway, so the internal penalty may well be
smaller than the external gain. **This needs the flight model to arbitrate** —
see §4.2.

That is the most interesting single result in this document: the divergent
nozzle you asked for may be the cheapest available fix for the boattail
conflict in §2.1.

### 3.5 Profiles the export gives a length but no shape

| feature | export gives | I assumed |
|---|---|---|
| nose fairing | length only (428 mm) | parabolic, tangent to the barrel |
| boattail | length **and** exit dia | nothing — the angle is derived |

### 3.6 Other omissions, listed so they are not silent

- **Fuel tank internal structure.** "50% usable" does not say how the other 50%
  is distributed (radial standoff? plumbing? axial?). I drew the annulus full
  and applied 50% as a scalar, exactly as the model does.
- **Chamber thermal.** The liner *is* the body skin, so the export stacks CFRP
  outboard of steel around a combustion chamber. The export README flags this;
  no gauge here reflects a thermal requirement.
- **Wing longitudinal station, fins.** Out of scope for this drawing, and the
  export states both are unavailable.

---

## 4. Questions only the flight model can answer

**4.1 — Does the V4 thrust model assume a choked nozzle?**
If it does, it is wrong below **M 1.061**, which is nearly the whole ramjet
burn (lights at M 0.488). If it does not, what exit condition does it use?

**4.2 — What total pressure does the combustor actually reach, vs Mach and
altitude?**
This is the single number that decides the expansion ratio in §3.4. I assumed
0.90 recovery from freestream stagnation; if the FP ramjet reports real
combustor `p0`, the whole nozzle trade becomes a calculation instead of an
assumption.

**4.3 — Is `drag_buildup`'s 8° boattail applied to the V4 shape?** (§2.1)
If yes, V4's drag is understated and the result needs a re-fly.

**4.4 — Does the mass model account for a real chamber→tailpipe cone?**
It currently assumes zero length, which is where §1.3's 11.4% comes from.

**4.5 — What is the real avionics/payload volume, and where must it sit?**
Now the top ask, since the centrebody is to be sized for avionics. Volume,
plus any longitudinal constraint.

**4.6 — Is the ramjet's required capture mass flow available?**
It would replace my capture-area *rule* with a capture-area *requirement*.

---

## 5. Concrete JSON additions requested

Everything below is additive — no existing key changes meaning.

```jsonc
{
  "body": {
    // disambiguate 1.1 / 1.2: export BOTH, everywhere
    "boattail_exit_outer_diameter_m": 0.117638,   // OML at the base plane
    "boattail_half_angle_deg": 12.94,             // so it can be compared to
                                                  // drag_buildup's 8.0
    "nose_fairing_profile": "parabolic_tangent"   // or whatever is intended
  },

  "internal_flowpath": {
    "chamber_flow_diameter_m": 0.209,             // distinct from the OML
    "chamber_to_tailpipe_cone_length_m": 0.1742,  // currently zero
    "nozzle_throat_diameter_m": 0.115638,         // rename of what is now
                                                  // called nozzle_exit
    "nozzle_expansion_area_ratio": 1.10,
    "nozzle_exit_diameter_m": 0.121282            // the REAL exit
  },

  "inlet": {                                       // whole block absent today
    "architecture": "nose_centrebody",
    "capture_area_m2": 0.01050247,
    "required_mass_flow_kg_s": null,               // <-- the one I actually want
    "lip_station_m": 0.088194,
    "lip_diameter_m": 0.132264,
    "centrebody_max_diameter_m": 0.0642,
    "centrebody_length_m": 0.120,
    "strut_count": null,
    "strut_blockage_fraction": null
  },

  "design_point": {                                // absent; needed to size
    "cutoff_mach": 1.100,                          // the nozzle at all
    "ramjet_light_mach": 0.488,
    "top_altitude_m": 1100.0,
    "combustor_total_pressure_ratio": null         // <-- §4.2
  },

  "payload": {
    "avionics_volume_m3": null,                    // <-- §4.5
    "avionics_longitudinal_constraint": null
  }
}
```

---

## Priority, if you only action three things

1. **§2.1** — boattail 8° vs 12.94°. It is the only finding that can move a V4
   *flight result*.
2. **§4.5 / §3.1** — the avionics volume, so the centrebody stops being a guess.
3. **§1.1** — the unbuildable base diameter. Cheap to fix, and it is a hard
   geometric contradiction rather than a modelling choice.
