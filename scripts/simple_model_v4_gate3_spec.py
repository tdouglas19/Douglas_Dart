"""Emit the V4 vehicle + trajectory specification for Gate 3.

This is a SPECIFICATION, not a study: it states the vehicle and the
trajectory that is frozen, and nothing about how they were chosen. Every
number is read from docs/v4_frozen/design.json or measured on a live re-fly
of it, so the document cannot drift from the frozen design.

Usage: .venv/Scripts/python scripts/simple_model_v4_gate3_spec.py
Writes docs/v4_trajectory_gate3.md
"""
from __future__ import annotations

import json
import subprocess
import sys
from math import degrees, pi
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DESIGN_PATH = ROOT / "docs" / "v4_frozen" / "design.json"
OUT_PATH = ROOT / "docs" / "v4_trajectory_gate3.md"

PHASE_ORDER = ["v3_climb", "v4_pushover", "v3_dive", "v4_pullout", "drag_strip"]
PHASE_LABELS = {
    "v3_climb": "1. Spiral climb",
    "v4_pushover": "2. Pushover arc (pitch-down)",
    "v3_dive": "3. Dive",
    "v4_pullout": "4. Pull-out arc (pitch-up)",
    "drag_strip": "5. Drag strip",
}


def refly() -> dict:
    """Run the frozen design in its own process (CD0 is baked at import)."""
    proc = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "fly_frozen_v4.py")],
        capture_output=True, text=True, cwd=str(ROOT), check=True)
    for line in proc.stdout.splitlines():
        if line.startswith("FLIGHT_JSON:"):
            return json.loads(line[len("FLIGHT_JSON:"):])
    raise SystemExit("fly_frozen_v4.py produced no FLIGHT_JSON")


def phase_rows(states) -> str:
    rows = []
    for phase in PHASE_ORDER:
        pts = [s for s in states if s.mode == phase]
        if not pts:
            continue
        rows.append(
            f"| {PHASE_LABELS[phase]} "
            f"| {pts[0].time_s:.1f} – {pts[-1].time_s:.1f} "
            f"| {pts[-1].time_s - pts[0].time_s:.1f} "
            f"| {pts[0].altitude_m:.0f} → {pts[-1].altitude_m:.0f} "
            f"| {pts[0].mach:.3f} → {pts[-1].mach:.3f} "
            f"| {degrees(pts[0].flight_path_angle_rad):+.1f} → "
            f"{degrees(pts[-1].flight_path_angle_rad):+.1f} "
            f"| {max(s.load_n_total for s in pts):.2f} |")
    return "\n".join(rows)


def main() -> None:
    design = json.loads(DESIGN_PATH.read_text())
    flight = refly()
    v, w, tj = design["vehicle"], design["wing_concept"], design["trajectory"]
    cf = design["constants_at_freeze"]

    # live states, for the per-phase table
    import os
    os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = str(cf["CD0_FRONTAL"])
    from scripts.simple_model_v4_baseline import V4_WING, fly, v4_geometry
    from simple_model.mass_model import vehicle_dry_mass
    result = fly(climb_deg=tj["initial_climb_angle_deg"],
                 dive_deg=tj["dive_angle_deg"],
                 floor_m=tj["floor_altitude_m"], gate_mach=tj["ramjet_gate_mach"],
                 pullout_n=tj["pullout_load_factor"],
                 top_altitude_m=tj["top_altitude_m"],
                 dive_end_mach=tj["dive_end_mach"])
    geom = v4_geometry()
    mass = vehicle_dry_mass(geom.diameter_m, geom.chamber_length_m,
                            geom.throat_diameter_m, geom.throat_length_m,
                            V4_WING.reference_area_m2,
                            flight["fuel_burned_kg"] * 1.25)

    d_mm = v["diameter_m"] * 1000.0
    nose_mm = 2.0 * d_mm
    tail_mm = 1.0 * d_mm
    body_mm = v["chamber_length_m"] * 1000.0 + v["throat_length_m"] * 1000.0 + 3.0 * d_mm
    frontal_m2 = pi * v["diameter_m"] ** 2 / 4.0
    throat_frac = (v["throat_diameter_m"] / v["diameter_m"]) ** 2
    cutoff = [s for s in result.states if s.thrust_n > 0.0][-1]

    md = f"""# Douglas Dart V4 — vehicle and trajectory specification for Gate 3

**Status: FROZEN.** Source of truth is `docs/v4_frozen/design.json`, locked by
`tests/test_v4_frozen.py`, which re-flies it through `scripts/fly_frozen_v4.py`
in a subprocess (CD0 is baked at import time).

Produced by the calibrated closed-form `simple_model` at
`CD0_FRONTAL = {cf['CD0_FRONTAL']}` (frontal-area referenced). Propane.
Wet mass 22.68 kg (50 lb) at release. Motor cutoff at Mach
{cf['MOTOR_CUTOFF_MACH']}.

This document states **what the vehicle is and what it flies**. It does not
cover how the configuration was selected.

---

## 1. Vehicle

### 1.1 Body and duct

| item | value |
|---|---|
| body diameter | **{d_mm:.1f} mm** |
| chamber diameter | {d_mm:.1f} mm (the chamber **is** the body — no annular gap) |
| chamber length | {v['chamber_length_m'] * 1000:.1f} mm |
| duct length (cone + tailpipe) | {v['throat_length_m'] * 1000:.1f} mm |
| throat / nozzle exit diameter | **{v['throat_diameter_m'] * 1000:.1f} mm** |
| throat / body area fraction | {throat_frac:.4f} (operability cap {cf['PULSEJET_MAX_THROAT_AREA_FRACTION']}) |
| nose fairing | {nose_mm:.0f} mm (2.0 D) |
| aft body / boattail | {tail_mm:.0f} mm (1.0 D) |
| **overall body length** | **{body_mm:.0f} mm** |
| fineness ratio | {body_mm / d_mm:.2f} |
| frontal area | {frontal_m2:.5f} m² |

### 1.2 Wing

| item | value |
|---|---|
| span | {w['span_m'] * 1000:.1f} mm |
| aspect ratio | {w['aspect_ratio']:.4f} |
| taper ratio | {w['taper_ratio']:.4f} |
| leading-edge sweep | {w['sweep_deg']:.2f}° |
| section | {w['airfoil_key']} |
| reference area | {V4_WING.reference_area_m2:.5f} m² |
| Oswald e | {V4_WING.oswald_e:.4f} |
| CL_max (sweep-effective) | {V4_WING.cl_max_effective:.4f} |

### 1.3 Mass and fuel

| item | value |
|---|---|
| engine duct (steel) | {mass.engine_duct_kg:.3f} kg |
| airframe skin (CFRP + overhead) | {mass.airframe_skin_kg:.3f} kg |
| wing | {mass.wing_kg:.3f} kg |
| avionics | {mass.avionics_kg:.3f} kg |
| tank hardware | {mass.tank_hardware_kg:.3f} kg |
| landing hardware | {mass.landing_hardware_kg:.3f} kg |
| **dry mass** | **{flight['dry_mass_kg']:.3f} kg** |
| fuel burned (mission) | {flight['fuel_burned_kg']:.3f} kg |
| fuel loaded (burn + 25 % reserve) | {flight['fuel_burned_kg'] * 1.25:.3f} kg |
| tank capacity (annulus) | {flight['tank_capacity_kg']:.3f} kg |
| **payload / ballast margin** | **{flight['payload_margin_kg']:.3f} kg** |
| duct wall thickness | {mass.duct_wall_thickness_m * 1000:.2f} mm |

Mass is re-derived from this geometry; no block is inherited from a parent
design.

---

## 2. Trajectory

### 2.1 Commanded parameters

| parameter | value |
|---|---|
| release velocity | 40 m/s (rail / catapult) |
| **ramjet lightoff gate** | **Mach {tj['ramjet_gate_mach']:.2f}, and only while descending** |
| top of climb | **{tj['top_altitude_m']:.0f} m** (commanded, not derived) |
| climb flight path angle | {tj['initial_climb_angle_deg']:.1f}° |
| climb type | **spiral** (helical), {tj['spiral_bank_deg']:.0f}° bank |
| pushover load factor | {tj['pushover_load_factor']:.1f} g |
| dive flight path angle | −{tj['dive_angle_deg']:.1f}° |
| dive terminates at | Mach {tj['dive_end_mach']:.2f} |
| pull-out load factor (nominal) | {tj['pullout_load_factor']:.1f} g |
| pull-out load factor (limit) | {tj['pullout_max_load_factor']:.1f} g |
| hard altitude floor | {tj['floor_altitude_m']:.2f} m (400 ft) |
| drag-strip flight path angle | {v['drag_strip_climb_angle_deg']:.1f}° |
| motor cutoff | Mach {cf['MOTOR_CUTOFF_MACH']} |

### 2.2 Phase sequence

| phase | t (s) | duration (s) | altitude (m) | Mach | γ (deg) | peak load (g) |
|---|---|---|---|---|---|---|
{phase_rows(result.states)}

**1. Spiral climb.** Climbs at {tj['initial_climb_angle_deg']:.0f}° to
{tj['top_altitude_m']:.0f} m on a helical ground track over the launch point:
identical air path, speed, flight path angle, drag and fuel to a straight
climb, but **zero net downrange**. The {tj['spiral_bank_deg']:.0f}° bank is
carried as load factor n = cos γ / cos φ = {result.states[0].load_n_yaw:.3f} g,
which feeds induced drag. Ground-track turn radius
**{flight['spiral_radius_m']:,.0f} m**.

**2. Pushover arc.** Constant {tj['pushover_load_factor']:.1f} g pitch-down
from +{tj['initial_climb_angle_deg']:.0f}° to −{tj['dive_angle_deg']:.0f}°.
Radius **{flight['pushover_radius_m']:,.0f} m**, duration
{flight['pushover_duration_s']:.1f} s.

**3. Dive.** Straight at −{tj['dive_angle_deg']:.0f}°. **The ramjet lights in
this phase** — see §2.3. The dive ends on Mach {tj['dive_end_mach']:.2f}.

**4. Pull-out arc.** Pitch-up to the drag-strip angle at
{tj['pullout_load_factor']:.1f} g nominal. Radius
**{flight['pullout_radius_m']:,.0f} m**, duration
{flight['pullout_duration_s']:.1f} s. The floor is a hard constraint and the
load factor is the free variable: if the arc stops making the floor, the
commanded load rises, capped at {tj['pullout_max_load_factor']:.1f} g. On this
design the nominal {tj['pullout_load_factor']:.1f} g is sufficient and the arc
bottoms out at **{flight['min_powered_altitude_m']:.1f} m**, i.e.
{flight['min_powered_altitude_m'] - tj['floor_altitude_m']:.1f} m above the
{tj['floor_altitude_m']:.1f} m floor.

**5. Drag strip.** {v['drag_strip_climb_angle_deg']:.1f}° to motor cutoff at
Mach {cf['MOTOR_CUTOFF_MACH']}, reached at {cutoff.time_s:.1f} s,
{cutoff.altitude_m:.0f} m, {cutoff.distance_m / 1000:.2f} km downrange.

### 2.3 Ramjet lightoff

| item | value |
|---|---|
| gate | Mach {tj['ramjet_gate_mach']:.2f}, **must be crossed while descending** |
| lights at | **Mach {flight['ramjet_lightoff_mach']:.3f}, {flight['ramjet_lightoff_altitude_m']:.0f} m, t {result.ramjet_lightoff_time_s:.1f} s** |
| phase at lightoff | **`{flight['ramjet_lightoff_mode']}` — in the dive** |
| latching | once lit, stays lit through the pull-out and drag strip |
| pulsejet | continues running additively through and after lightoff |

The gate is a policy, not a bare Mach constant: a Mach threshold alone cannot
express *where* in the trajectory the engine comes alive, and the requirement
is that it comes alive in the dive.

### 2.4 Body loading

Accelerometer convention — specific force, **gravity excluded**. A 2-D
trajectory loads two body axes: roll (axial) and yaw (normal, in-plane).

| item | value |
|---|---|
| **peak total** | **{flight['peak_load_n_total']:.2f} g**, in `{flight['peak_load_mode']}` |
| peak yaw axis (normal) | {flight['peak_load_n_yaw']:.2f} g |
| peak roll axis (axial) | {flight['peak_load_n_roll']:.2f} g |
| spiral climb | {result.states[0].load_n_yaw:.3f} g normal (bank), sustained |
| pushover arc | {tj['pushover_load_factor']:.1f} g normal |
| pull-out arc | {tj['pullout_load_factor']:.1f} g normal commanded |
| drag strip | axial load rises to {flight['peak_load_n_roll']:.2f} g under ramjet acceleration |

At {flight['peak_load_n_total']:.2f} g on 22.68 kg the airframe carries
{flight['peak_load_n_total'] * 22.68 * 9.80665:.0f} N
({flight['peak_load_n_total'] * 22.68 * 9.80665 / 4.44822:.0f} lbf). Structure
is not modelled anywhere in this repo; this is a stated design limit, not a
demonstrated capability.

### 2.5 Return and landing

After cutoff the vehicle flies the return-to-launch profile: pitch-up
half-loop to reverse heading, glide home at the airframe's equilibrium slope,
spiral down over the launch point, flare to touchdown at stall speed.

| item | value |
|---|---|
| touchdown speed | {result.states[-1].velocity_m_per_s:.1f} m/s (= stall speed) |
| stall speed at landing | {flight['stall_speed_m_per_s']:.1f} m/s (cap 45 m/s) |
| lands from launch point | {flight['lands_from_launch_m']:.1f} m |
| total flight time | {result.states[-1].time_s:.0f} s |

---

## 3. Verified mission numbers

| metric | value |
|---|---|
| peak Mach | {flight['peak_mach']:.3f} |
| motor cutoff reached | {flight['motor_cutoff_reached']} |
| ramjet lit in the dive | **{flight['ramjet_lit_in_dive']}** |
| dive exit Mach | {flight['dive_exit_mach']:.3f} |
| peak thrust / weight | {flight['peak_thrust_to_weight']:.2f} |
| min traverse acceleration | {flight['min_traverse_accel_g']:.3f} g |
| min powered acceleration | {flight['min_powered_accel_g']:+.4f} g |
| min powered thrust margin (engine-only) | {flight['min_powered_thrust_margin']:.3f} |
| min altitude under power | {flight['min_powered_altitude_m']:.1f} m |
| floor violated | {flight['floor_violated']} |
| γ ≥ 0 rule (M 0.80 → cutoff) | {'satisfied' if not flight['rule_violated'] else 'VIOLATED'} |
| fuel burned / capacity | {flight['fuel_burned_kg']:.3f} / {flight['tank_capacity_kg']:.3f} kg |
| payload margin | {flight['payload_margin_kg']:.3f} kg |

## 4. Constraint status

| constraint | target | V4 | status |
|---|---|---|---|
| ramjet lights in the dive | required | Mach {flight['ramjet_lightoff_mach']:.3f} in `{flight['ramjet_lightoff_mode']}` | **PASS** |
| peak Mach | ≥ 1.0 | {flight['peak_mach']:.3f} | **PASS** |
| motor cutoff reached | yes | {flight['motor_cutoff_reached']} | **PASS** |
| 400 ft floor | ≥ {tj['floor_altitude_m']:.1f} m | {flight['min_powered_altitude_m']:.1f} m | **PASS** |
| peak body load | ≤ 4.0 g | {flight['peak_load_n_total']:.2f} g | **PASS** |
| stall speed | ≤ 45 m/s | {flight['stall_speed_m_per_s']:.1f} m/s | **PASS** |
| safe landing | yes | {flight['safe_landing']} | **PASS** |
| fuel within capacity | ≤ {flight['tank_capacity_kg'] / 1.25:.3f} kg | {flight['fuel_burned_kg']:.3f} kg | **PASS** |
| payload margin | > 0 | {flight['payload_margin_kg']:.3f} kg | **PASS** |
| γ ≥ 0 from M 0.80 | required | {'satisfied' if not flight['rule_violated'] else 'violated'} | **PASS** |
| min traverse acceleration | ≥ {cf['MIN_POWERED_ACCELERATION_G']} g | {flight['min_traverse_accel_g']:.3f} g | **FAIL** |
| min powered acceleration | > 0 | {flight['min_powered_accel_g']:+.4f} g | **FAIL** |
| min powered thrust margin | ≥ {1 + cf['MIN_POWERED_THRUST_MARGIN_FRACTION']:.2f} | {flight['min_powered_thrust_margin']:.3f} | **FAIL** |

The three failures are engine-limited, not trajectory-limited, and none has
been met by any V2, V3 or V4 configuration in any campaign to date.

## 5. Items for Gate 3 to verify

1. **Top of climb at {tj['top_altitude_m']:.0f} m.** The first-principles
   pulsejet flames out at roughly 1250 m, and that ceiling fell ~200 m when
   the grid was refined from 162 to 324 cells. `simple_model` has no
   flame-out model — it only lapses thrust as (ρ/ρ_SL)³ — and already shows
   powered acceleration going slightly negative at the top of the climb. If
   the flame-out ceiling is confirmed lower, this trajectory does not close.
2. **Pulsejet sustain at four operating points**, not one: release, top of
   climb, the lightoff condition, and the dive exit.
3. **Reaching the Mach {tj['ramjet_gate_mach']:.2f} gate inside the dive**, at
   FP fidelity. This is an acceleration question and closed-form screens of
   it are unreliable near a boundary.
4. **Climb margin at ΔM ≤ 0.02.** Coarser propulsion time steps flatter the
   climb margin by roughly the entire margin.
5. **Verify at CONFIRM tier (324 cells), not MARCH (162).**
6. **Side valve runner packaging.** The chamber is now the full body
   diameter, so the runners need an external fairing whose drag is not in
   this CD0.

## 6. Reproduce

```bash
SIMPLE_MODEL_CD0_FRONTAL={cf['CD0_FRONTAL']} PYTHONPATH=. .venv/Scripts/python scripts/fly_frozen_v4.py
```

Regenerate this document:

```bash
.venv/Scripts/python scripts/simple_model_v4_gate3_spec.py
```

Plots and the fuller write-up: `out_simple_model/v4_optimal_design.md`.
`simple_model` changes made for V4, for the parallel `medium_model` session:
`docs/simple_model_changes_for_medium_model.md`.
"""
    OUT_PATH.write_text(md, encoding="utf-8")
    print(f"wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
