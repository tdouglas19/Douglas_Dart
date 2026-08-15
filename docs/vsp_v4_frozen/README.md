# Douglas Dart V4 — VSP aerodynamics & stability freeze

**Frozen 2026-08-14.** Machine-readable record: `design.json` (inputs, results,
git SHA). Built by `scripts/vsp_model/`; nothing here is hand-typed.

The V4 sizing chain is a **point-mass** model. It has no CG, no moment arms, no
stability terms and **no tail surfaces at all**. This freeze is the layer that
adds them, and the headline result is that the vehicle as sized could not fly the
V4 trajectory without a tail that the trajectory model does not carry.

---

## The configuration

| | |
|---|---|
| **tail** | **V-tail, 2 panels at 45°/135°** (45° half-angle from vertical) |
| total fin area | 0.1220 m² (0.0610 m² each) |
| fin geometry | 235.1 mm root, 105.8 mm tip, 357.9 mm exposed height |
| aspect ratio / taper / sweep | 2.1 / 0.45 / 35° LE |
| thickness | **t/c 0.08** — set by flutter, not by aero |
| material / attachment | CFRP, bonded and filleted to the skin, **no through spar** |
| control | **40% chord ruddervator (flap)**, not all-moving |
| fin mass | **0.73 kg** @ 6.0 kg/m² |
| **wing** | **V4 planform unchanged**, root LE moved to **1600 mm** |
| CG (release → burnout) | 946.3 → 869.2 mm |

## Results

| | release | burnout |
|---|---|---|
| pitch static margin | **+3.01 cal** | +3.37 cal |
| yaw static margin | **+2.61 cal** | +2.97 cal |
| fin flutter (CFRP, bonded) | **M 2.92**, margin 2.66 | — |
| roll damping `Cl_p` | −1.60, τ = 0.052 s | — |

Control deflection required, all-moving equivalent: 3 g pull-out **6.9°**,
spiral climb **8.7°**, 6 g structural cap **13.8°**, glide trim **12.6°**.
A 40% chord flap multiplies these by 1.56 and still covers every case inside a
25° limit.

## Why each choice was forced

**V-tail, not the cruciform.** The cruciform's lower fins hung **143 mm below the
body**, so on a belly landing they took the whole vehicle. Both V-tail panels sit
in the upper half; the lowest surface is 322 mm *above* the axis. Same total
area gives the same pitch and yaw stiffness (a cruciform splits cos²45° into each
axis; a V at 45° does the same), and roll damping *improves* 34% because two
larger panels reach further out and damping goes as y².

**45° half-angle, not steeper.** A steeper V buys yaw stability (+3.31 at 30°)
but halves pitch *control* authority — the 6 g case then needs 21.5°, past any
limit. 45° is the only angle where every commanded manoeuvre fits.

**t/c 0.08, AR 2.1.** Flutter, not aerodynamics, sets these. Flutter speed goes
as `√((t/c)³/A³)`, and an aero-only search drives AR up and t/c down — exactly
the wrong way. At AR 3.0 / t/c 0.04 in CFRP the fin flutters at **Mach 0.67**,
below the Mach 1.10 requirement: it departs during the dive.

**Flap, not all-moving.** All-moving needs less deflection (13.8° vs 21.7°) but
requires a pivot spindle through the skin — the through-structure the no-spar
constraint rules out. A flap keeps the fin bonded.

## Known problems — read before using any number here

1. **Barrowman and VSPAERO disagree ~2× on the body destabilising term.**
   Barrowman is primary because the VSPAERO vortex-lattice solve does not
   converge at the fin/body junction (0 of 4 mount depths at the finest mesh) and
   its panel mode returns intermittent outliers. **Absolute margins are
   uncertain; the wing/fin split is robust.**
2. **Barrowman is inviscid.** No viscous body crossflow, which is destabilising
   on a fineness-10.6 body. These margins are **optimistic**.
3. **This is a dart, not a glider.** Trim alpha −0.33°, L/D −0.44, glide angle
   **90°**, terminal **295 m/s** nose-down. Recovery needs a parachute, a moving
   CG, or sustained control deflection. ~3 calibres of margin is what makes it
   fly straight under misaligned thrust, and the same property stops it gliding.
4. **32% of the wet mass sits at stations chosen here**, not modelled anywhere.
   The 4.68 kg ballast at 214 mm is what makes the CG forward enough for these
   margins. At a mid-body CG, VSPAERO's neutral point puts the vehicle
   **unstable**.
5. **0.25° of fin misalignment gives 119 deg/s of steady roll.** Damping limits
   it but cannot null it. With bonded fins and no jig feature, build tolerance
   drives roll behaviour, not aerodynamics.
6. **Fin root bending passes entirely through the bond and fillet.** Nothing in
   this project sizes that joint.
7. **The fin drag sits exactly at the mission ceiling.** The V4 sim measured
   0.120 m² as achieving cutoff on 1.8% fuel margin, and 0.160 m² as running dry.
   The un-costed side valve runner fairings draw from the same budget.
8. **A V-tail couples roll and yaw** at sideslip in a way the cruciform did not.

## Provenance

- Sizing input: `out_medium_model/v4_export/structural_dimensions.json`
  (git `d7b0670`, corrected boattail)
- Built by: `scripts/vsp_model/` — see its `README.md` for the eight OpenVSP
  defects found along the way, and `VEHICLE_ARCHITECTURE.md` for the geometry
- Model: `out_vsp_model/v4_final/douglas_dart_v4_final.vsp3`
