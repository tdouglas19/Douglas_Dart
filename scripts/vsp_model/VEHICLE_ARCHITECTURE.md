# Douglas Dart V4 — vehicle architecture for the VSP model

The shared understanding of what this vehicle *is*, geometrically. The code in
this folder implements exactly what is written here; if the two disagree, that is
a bug in one of them.

**Source of truth for dimensions:** `out_medium_model/v4_export/structural_dimensions.json`
(git `d7b0670`), produced by `scripts/medium_model_v4_export.py` from the frozen
V4 design. Nothing dimensional is hand-typed into this folder.

---

## 1. What the vehicle is

A 50 lb (22.68 kg) supersonic dart. A single **flow-through duct** runs the whole
length: air enters an open nose lip, passes through a combustion chamber that
*is* the body barrel, exits through a tailpipe and nozzle. Two engines share that
duct — a valved pulsejet for the low-speed climb, then a ramjet once there is
enough ram pressure. The mission is a spiral climb to 1100 m, a 14° dive to light
the ramjet, a 3 g pull-out at a 400 ft floor, and a level run to Mach 1.1.

Aerodynamically it is a **body-dominated** configuration: 2274 mm long on a
214 mm diameter (fineness 10.6), with a 533 mm span wing that barely clears the
body. Body frontal area is 0.0360 m²; the wing reference area is 0.1525 m². Most
of the drag and a large share of the lift come from the body, which is why the
trajectory model references its CD0 to **frontal area**, not wing area.

---

## 2. Outer mold line

All stations measured from the nose tip, along the body axis (+X aft).

| station | x (mm) | dia (mm) | what |
|---|---:|---:|---|
| inlet lip | 0.0 | 115.6 | **open** — the inlet is the nose itself |
| nose cowl end | 428.0 | 214.0 | cowl fairs out to the barrel; chamber starts here |
| chamber end / tailpipe start | 817.3 | 214.0 | internal only, no OML change |
| boattail start | 2059.6 | 214.0 | end of constant barrel |
| aft face | 2273.6 | 153.8 | **open** — annular base around the 115.6 mm nozzle |

### 2.1 The nose is the inlet

There is no inlet pod and no separate nacelle. The drag build-up states this
outright: *"this vehicle's inlet is the NOSE of the body, not a pod"*
(`medium_model/drag_buildup.py`). The lip diameter is **not** a field in the
sizing export — it is derived, because `flight_sim.py` computes the inlet lip
area as `pi * throat_diameter**2 / 4`, and the throat and nozzle diameters are
the same 115.6 mm.

The cowl profile is **ours**. The sizing model gives a 428 mm length (2.0 body
diameters) and nothing else. We build a cubic that leaves the lip at a commanded
12° half-angle and arrives tangent to the barrel — the conventional subsonic cowl
shape, and the one the build-up's cowl-suction term assumes. Mean slope over the
fairing is only 6.6°, so the lip flare is mild.

### 2.2 The boattail closes at 8°, not onto the nozzle

`structural_dimensions.json` reports `boattail_exit_diameter_m = 0.1156`, which
is just the nozzle diameter copied across. Taken literally that closes 214 mm to
115.6 mm over 214 mm — a **13° half-angle**, past the attached-flow limit.

The model the vehicle was actually flown with does something else.
`drag_buildup.base_diameter_m` closes at `BOATTAIL_HALF_ANGLE_DEG = 8.0`, floored
at the nozzle diameter, giving **153.8 mm** and an **80.9 cm² annular base**
around the exhaust. That annulus is where the flown model charges base drag.

We build the 8° version, so the VSP mold line and the flown drag describe one
aft body. `GeometryInputs.boattail_half_angle_deg = None` closes onto the nozzle
instead, if you want the literal reading of the JSON.

### 2.3 The internal flowpath is NOT modelled

Internally the duct necks from the 214 mm chamber to a 115.6 mm tailpipe. In the
VSP model the flow-through interior is simply the inside of the outer mold line,
which stays at 214 mm through the barrel. Internal duct losses and true capture
are therefore absent. This is acceptable here only because VSPAERO is being used
for **inviscid external aerodynamics**, and the trajectory model accounts for
spillage and additive drag separately.

---

## 3. Wing

Planform comes straight from the sizing export, and is a **theoretical trapezoid
spanning the centreline** — the 532.5 mm span is tip-to-tip *through* the body.

| quantity | value |
|---|---|
| span (tip to tip, theoretical) | 532.5 mm |
| reference area | 0.15250 m² |
| root chord (at centreline) | 378.8 mm |
| tip chord | 194.0 mm |
| aspect ratio / taper | 1.860 / 0.512 |
| LE sweep / c/4 sweep | 13.42° / 3.72° |
| thickness | t/c 0.04, `thin_cambered` |

What actually gets built is the **exposed** panel, from the body surface outward:
root chord 308.7 mm at the mount radius, tip 194.0 mm, exposed semispan 165.4 mm.
The chord at the mount is read off the same straight taper line, so the built
surface lies exactly on the theoretical planform rather than approximating it.

**Reference area stays the theoretical 0.15250 m²**, not the exposed area,
because that is what the trajectory model used to produce every CL and CD it
flew with.

### 3.1 Choices the sizing model does not make

Its own note: *"the wing's LONGITUDINAL POSITION on the body is not an output of
this repo — it is yours to choose. Likewise dihedral, incidence and twist, all of
which are zero here because they are unmodelled, not because they were chosen."*

- **Longitudinal station** — default root LE at 1000 mm, clear of the cowl
  (ends 428 mm) and the boattail (starts 2060 mm).
- **Dihedral, incidence, twist** — held at zero so the VSP model reproduces the
  flown planform exactly. All are live inputs.
- **Airfoil ordinates** — `thin_cambered` is a label with t/c 0.04 and
  `cl_max_2d` 1.2, not a section. Built as a NACA 4-series, 2% camber at 40%
  chord, which reproduces that cl_max at that thickness.

### 3.2 Two panels, built as one mirrored geom

The port panel is built by **reflection about XZ**, not as a second geom rolled
180°. A 180° roll maps +Z to −Z, so it mounts the panel correctly but with its
camber upside down — the two halves then fight each other, and the model reports
a rolling moment and a lift deficit at zero sideslip with no other symptom. This
was observed here before it was fixed: CL rose from 0.1045 to 0.1545 at α=4°,
M 0.30 once the mirror replaced the roll. `vsp_build` now refuses to roll a
cambered panel past 90°.

---

## 4. Fins — entirely ours

> `structural_dimensions.json`: *"NOT MODELLED. There are no tail/fin surfaces
> anywhere in this repo — no area, no volume coefficient, no drag term. A real
> vehicle needs them and their mass and drag are absent from every number here."*

Every fin number is a choice, not a model output. Configuration (user decision):
**cruciform, four panels clocked 45°/135°/225°/315°**, so they sit between the
wing panels rather than in their wake.

Sized from an area ratio against the wing reference area rather than a volume
coefficient, because a volume coefficient needs a CG the project does not have:

| input | default | meaning |
|---|---:|---|
| `fin_area_ratio` | 0.35 | total exposed fin area / wing reference area |
| `fin_aspect_ratio` | 1.20 | one panel's exposed semispan² / its area |
| `fin_taper_ratio` | 0.45 | |
| `fin_le_sweep_deg` | 35.0 | |
| `fin_root_te_x_m` | boattail start | furthest aft a root fits on the barrel |

Defaults give a 145.4 mm root, 65.5 mm tip, 126.5 mm exposed height per panel.
Symmetric section (a cambered fin would trim the vehicle in roll and yaw).

**Constraint enforced in code:** the entire fin root chord must land on the
constant-diameter barrel. A root overhanging a taper touches the surface at one
end and opens a gap at the other — a real defect found by GUI inspection on the
previous vehicle.

---

## 5. Mass, CG and inertia — built here, not read

The sizing chain is a **point-mass** model. `mass_budget.csv` lists items and
masses with **no stations at all**, and the export states that CG, moments of
inertia and static margin are not outputs. Pitching moment out of VSPAERO is
meaningless without a moment reference, so `mass_cg.py` assigns one.

Masses are read. Stations are assigned, and each is labelled:

| item | kg | station basis |
|---|---:|---|
| engine duct (chamber + tailpipe) | 5.60 | **derived** — split by wetted area, the mass model's own basis |
| airframe skin CFRP | 4.95 | **derived** — centroid integrated over the built OML |
| wing | 0.91 | **derived** — MAC quarter-chord; moves with wing station |
| tank hardware | 0.96 | **derived** — tailpipe annulus |
| fuel (loaded) | 3.07 | **derived** — tailpipe annulus; burns off aft, CG moves forward |
| avionics | 2.00 | **chosen** — nose-cowl annulus |
| landing hardware | 0.50 | **chosen** — mid-body |
| **payload ballast margin** | **4.68** | **chosen** — see below |

`payload_ballast_margin` is 4.68 kg — **21% of the 22.68 kg wet mass** — with no
assigned location anywhere in the project. It is the single largest CG lever on
the vehicle and is treated as a solvable variable, not a fixed lump.

At the defaults the vehicle sits at **CG 922 mm, 40.6% of body length**, with
Iyy = Izz = 9.13 and Ixx = 0.183 kg·m².

Any dynamic-stability number carries the uncertainty of those three chosen
stations. Static margins carry only the CG's.

---

## 6. What this model does NOT represent

Beyond the internal flowpath (§2.3):

- **Side valve inlets.** The pulsejet breathes through side-mounted valve runners
  ingesting boundary-layer air — that is why its thrust lapses with Mach instead
  of ramming up. The V4 export notes the runners *still do not fit inside a
  214 mm skin* and need an external fairing whose drag is in neither the flat
  CD0 nor the build-up. No fairing is in this VSP model either.
- **Base drag.** VSPAERO's inviscid solution does not produce it, and this
  vehicle carries an 80.9 cm² annular base.
- **Skin friction.** Not in the VSPAERO CD.
- **Control surfaces.** None defined.
- **Thermal.** The combustion chamber is the body skin. Not a geometry problem,
  but it is a real one the whole chain is silent about.

---

## 7. How the model is handed to the solver

Not a geometry choice exactly, but it belongs with the architecture because
getting it wrong silently corrupts every number:

- The **body is thick geometry**; the **wing and fins are thin**. VSPAERO solves
  thick surfaces with panels and thin ones as zero-thickness lifting sheets, and
  the assignment is made by the `GeomSet`/`ThinGeomSet` analysis inputs.
  `vsp_build` writes two named user sets (`VSPAERO_Thick_Bodies`,
  `VSPAERO_Thin_Surfaces`) into the `.vsp3` to carry the split. Putting the
  fuselage in the thin set makes the bare body return CL = 0.0000 with a
  spurious CS = −0.0567 — its whole normal force on the wrong axis.
- Every surface root is buried at least **6 mm** into the body
  (`MEASURED_MIN_OVERLAP_M`). Below about 5 mm the mixed thick/thin solve
  returns NaN from GMRES with no error at all.
- The panel-method grid puts everything in the thick set, which is correct for a
  method that meshes real surfaces.

See `README.md` sections A–C for the measurements behind each of these.

### 7.1 A body in potential flow carries no lift

The bare body returns CL ≈ 0.0002 at α = 4° with a large pitching moment. That
is correct, not a defect: d'Alembert's paradox says a closed body in inviscid
potential flow has no net force, but a slender body still has a **Munk moment**.

This matters more than it sounds. The Munk moment is strongly *destabilizing*,
and on this vehicle it dominates. Measured over the full 78-point baseline grid
(`out_vsp_model/v4/stability.md`):

| | vortex lattice | panel |
|---|---|---|
| `Cm_alpha` | +3.20 to +3.62 /rad | +4.92 to +5.21 /rad |
| `CL_alpha` | +1.60 to +1.79 /rad | +1.43 to +1.89 /rad |
| neutral point | ~330 mm from nose | −100 to +104 mm |
| **static margin** (CG 922 mm) | **−2.0 cref** | −2.8 to −3.5 cref |
| `Cn_beta` | −2.1 to −2.5 /rad | — |

The static margin is essentially **constant at −2.0 chords from M 0.20 to
M 0.80** — the neutral point is a geometric property and barely moves subsonically.
The airframe as sized is badly unstable in **both** pitch and yaw; `Cl_beta` is
the only stable term, and only marginally (−0.017 /rad).

**This one is mesh-converged.** Re-solving the same geometry at four different
surface-root mount depths (a meshing parameter with no physical meaning) agrees
to ±4% on `Cm_alpha` and ±6% on static margin: **SM = −1.92 ± 0.12 cref,
`Cn_beta` = −2.40 ± 0.09 /rad**. Larger-fin configurations are NOT converged and
their numbers must not be used — see `README.md`, "the derivatives are
mesh-dependent".

The real body lift — viscous crossflow separation, substantial on a fineness-10.6
body at incidence — is absent from any panel method, and it acts well forward.
Treat these as a **lower bound** on the instability, not an upper one.

## 8. Reference quantities

| quantity | value | why |
|---|---:|---|
| `Sref` | 0.15250 m² | theoretical wing trapezoid — what the trajectory model used |
| `bref` | 0.5325 m | theoretical span |
| `cref` | 0.29631 m | wing MAC |
| `Xcg` | from `mass_cg` | 0.9221 m at defaults |

The trajectory model's CD0 is **frontal-area referenced** (0.0360 m²). Multiply a
wing-referenced coefficient by **4.2398** to convert. `vehicle_geometry.describe`
prints this factor with every run so the two conventions never get mixed silently.
