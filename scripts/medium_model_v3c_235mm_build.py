"""Build docs/v3c_235mm/design.json -- the 235 mm-chamber pulsejet vehicle.

WHY: the V3c latest-gate screen walled at M 0.45 on the V3a body at EVERY
climb x dive combination.  A wall that ignores the trajectory points at the
engine: the vehicle has to reach the gate Mach on pulsejet thrust alone.
out_medium_model/v3b_cliff.json measured a 235 mm chamber at tail/chamber-
diameter 4.00 producing 149.2 N at M 0.15 vs V3a's 123.6 N (+21%) in a body
39 mm SHORTER.  This file turns that duct into a flyable design.

THE MASS BLOCK IS THE HARD PART and is done here rather than copied.
``medium_model/design.py`` derives the fuel allocation as
    loaded = MAX_WET_MASS_KG - dry_mass_kg - mass_margin_kg
    burn cap = 0.90 * loaded
so those two numbers ARE the fuel budget; a copied block is a wrong budget.

Reconstruction of the ancestor's chain (simple_model/optimize.py ~L361-388):
    fuel_loaded = 1.25 * fuel_burned            (from the optimizer flight)
    dry         = vehicle_dry_mass(geometry..., wing_area, fuel_loaded)
    margin      = MAX_WET_MASS_KG - dry - fuel_loaded
The wing reference area it feeds in is the WING CONCEPT's own
span^2 / aspect_ratio (0.152499 m^2), NOT wingspan_m^2 / WING_ASPECT_RATIO.
Verified: that choice reproduces docs/v3_frozen dry_mass_kg
12.569615400692696 to machine precision (see the assertion below).

FUEL ALLOCATION CHOICE (stated, not hidden): loaded fuel is HELD at V3a's
2.693257 kg.  Two reasons.  (1) It makes the burn cap identical
(2.423931 kg) on both vehicles, so the screen compares engines and drag,
not tanks.  (2) It is not volume-limited on either vehicle -- the annulus
holds 6.91 kg (V3a) / 7.63 kg (235 mm) usable.  dry_mass_kg and
mass_margin_kg are then both recomputed for THIS geometry, so the block is
internally consistent: loaded_fuel_kg(dry, margin) returns 2.693257 exactly
and vehicle_dry_mass(this geometry, 2.693257) returns dry exactly.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model import constants as C                       # noqa: E402
from medium_model.mass_model import vehicle_dry_mass          # noqa: E402
from medium_model.mission import (MAX_WET_MASS_KG,            # noqa: E402
                                  burn_limit_kg, loaded_fuel_kg,
                                  usable_fuel_kg)

V3A = Path("docs/v3a_medium_model/design.json")
V3 = Path("docs/v3_frozen/design.json")
OUT = Path("docs/v3c_235mm/design.json")

CHAMBER_DIA_M = 0.235          # +9.8% on V3a's 214.0 mm; the user's +10% cap
BODY_OVER_CHAMBER = 1.0 / 0.95
THROAT_OVER_BODY = 0.5400      # V3a's ratio -> area frac 0.2916 < 0.30 cap
TAIL_OVER_CHAMBER_DIA = 4.00   # above the measured cliff (3.60 dead/3.70 alive)
CONE_FRACTION_OF_TAIL = 0.130 / (0.130 + 0.620)   # FP-1 proportions


def main() -> None:
    v3a = json.loads(V3A.read_text())
    v3 = json.loads(V3.read_text())
    c3, w3 = v3a["vehicle_candidate"], v3a["wing_concept"]
    wing_area = w3["span_m"] ** 2 / w3["aspect_ratio"]

    # --- prove the mass model reproduces the optimizer -------------------
    v3c, v3o = v3["vehicle_candidate"], v3["optimizer_results"]
    loaded_v3 = loaded_fuel_kg(v3o["dry_mass_kg"], v3o["mass_margin_kg"])
    check = vehicle_dry_mass(v3c["diameter_m"], v3c["chamber_length_m"],
                             v3c["throat_diameter_m"], v3c["throat_length_m"],
                             wing_area, loaded_v3)
    err = abs(check.dry_mass_kg - v3o["dry_mass_kg"])
    assert err < 1e-9, f"mass model does not reproduce V3: {err}"
    print(f"mass model reproduces docs/v3_frozen dry mass to {err:.2e} kg "
          f"(wing ref area {wing_area:.6f} m^2, loaded {loaded_v3:.6f} kg)")

    # --- V3a's OWN block is stale (copied from V3 at a shorter tail) -----
    dry_v3a_true = vehicle_dry_mass(
        c3["diameter_m"], c3["chamber_length_m"], c3["throat_diameter_m"],
        c3["throat_length_m"], wing_area, loaded_v3).dry_mass_kg
    print(f"V3a design.json claims dry {v3a['optimizer_results']['dry_mass_kg']:.4f} kg, "
          f"but its OWN geometry gives {dry_v3a_true:.4f} kg "
          f"({dry_v3a_true - v3a['optimizer_results']['dry_mass_kg']:+.4f}); "
          f"the tail grew 553 mm and the block was not rerun.")

    # --- geometry --------------------------------------------------------
    d_body = CHAMBER_DIA_M * BODY_OVER_CHAMBER
    d_throat = d_body * THROAT_OVER_BODY
    area_frac = (d_throat / d_body) ** 2
    assert area_frac <= C.PULSEJET_MAX_THROAT_AREA_FRACTION, area_frac
    L_chamber = c3["chamber_length_m"]                    # held from V3a
    tail = TAIL_OVER_CHAMBER_DIA * CHAMBER_DIA_M
    L_throat = tail / (1.0 - CONE_FRACTION_OF_TAIL)       # cone comes OUT of it
    cone = CONE_FRACTION_OF_TAIL * L_throat
    duct = L_chamber + L_throat
    body = duct + C.NOSE_TAIL_LENGTH_DIAMETERS * d_body

    assert abs((L_throat - cone) - tail) < 1e-12
    print(f"\nbody dia {d_body*1e3:.3f} mm, throat {d_throat*1e3:.3f} mm, "
          f"area frac {area_frac:.4f} (cap {C.PULSEJET_MAX_THROAT_AREA_FRACTION})")
    print(f"chamber {L_chamber*1e3:.2f} mm, throat_length_m {L_throat*1e3:.3f} mm "
          f"(cone {cone*1e3:.2f} + tailpipe {tail*1e3:.2f})")
    print(f"duct {duct*1e3:.1f} mm, body {body*1e3:.1f} mm, fineness {body/d_body:.3f}")

    frontal_v3a = 0.25 * math.pi * c3["diameter_m"] ** 2
    frontal_new = 0.25 * math.pi * d_body ** 2
    print(f"frontal area {frontal_v3a*1e4:.2f} -> {frontal_new*1e4:.2f} cm^2 "
          f"({100*(frontal_new/frontal_v3a - 1):+.2f}%)")

    # --- mass ------------------------------------------------------------
    loaded = loaded_v3          # HELD, see module docstring
    vol_cap = usable_fuel_kg(d_body, d_throat, L_throat,
                             C.FUELS[c3["fuel_key"]].density_kg_per_m3)
    assert loaded <= vol_cap, (loaded, vol_cap)
    m = vehicle_dry_mass(d_body, L_chamber, d_throat, L_throat, wing_area, loaded)
    margin = MAX_WET_MASS_KG - m.dry_mass_kg - loaded
    assert margin > 0.0, f"design busts the 50 lb budget: margin {margin}"
    assert abs(loaded_fuel_kg(m.dry_mass_kg, margin) - loaded) < 1e-12

    print(f"\nmass at loaded fuel {loaded:.6f} kg (annulus holds {vol_cap:.2f} kg usable)")
    for f in ("engine_duct_kg", "airframe_skin_kg", "wing_kg", "avionics_kg",
              "tank_hardware_kg", "landing_hardware_kg", "dry_mass_kg"):
        print(f"  {f:22s} {getattr(m, f):8.4f} kg")
    print(f"  {'mass_margin_kg':22s} {margin:8.4f} kg  "
          f"(V3a-geometry-consistent {MAX_WET_MASS_KG - dry_v3a_true - loaded:.4f})")
    print(f"  burn cap 0.90 x loaded = {burn_limit_kg(loaded):.6f} kg "
          f"(identical to V3a's, by construction)")

    design = {
        "name": "V3c-235mm: V3a body enlarged to a 235 mm pulsejet chamber at t/D 4.00",
        "frozen_utc_date": "2026-08-13",
        "derived_from": "docs/v3a_medium_model/design.json",
        "how_to_refly": ("MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. then "
                         "medium_model.design.load_frozen_design(this file) -> "
                         "fly(d, drag_model='buildup', climb_dive=...)."),
        "vehicle_candidate": {
            "diameter_m": d_body,
            "throat_diameter_m": d_throat,
            "chamber_length_m": L_chamber,
            "throat_length_m": L_throat,
            "wingspan_m": c3["wingspan_m"],
            "climb_angle_deg": c3["climb_angle_deg"],
            "fuel_key": c3["fuel_key"],
            "initial_climb_angle_deg": c3["initial_climb_angle_deg"],
            "dive_angle_deg": c3["dive_angle_deg"],
            "floor_altitude_m": c3["floor_altitude_m"],
        },
        "wing_concept": dict(w3),
        "constants_at_freeze": dict(v3a["constants_at_freeze"]),
        "optimizer_results": {
            "dry_mass_kg": m.dry_mass_kg,
            "mass_margin_kg": margin,
            "loaded_fuel_kg_derived": loaded,
            "burn_limit_kg_derived": burn_limit_kg(loaded),
            "provenance": ("dry_mass_kg = medium_model.mass_model."
                           "vehicle_dry_mass(d=0.247368, L_ch=0.389307, "
                           "d_th=0.133579, L_th=1.137097, S_wing=0.152499, "
                           "fuel_loaded=2.693257) -- recomputed for THIS "
                           "geometry, not copied. mass_margin_kg = 22.6796 - "
                           "dry - loaded. Loaded fuel HELD at V3a's 2.693257 kg "
                           "so the 2.423931 kg burn cap is identical on both "
                           "vehicles and the screen compares engines, not tanks; "
                           "it is not volume-limited (annulus holds "
                           f"{vol_cap:.2f} kg usable). No vehicle_score/"
                           "wing_score: this design was not produced by the "
                           "optimizer's objective."),
        },
        "geometry_derivation": {
            "chamber_diameter_m": CHAMBER_DIA_M,
            "body_over_chamber": BODY_OVER_CHAMBER,
            "throat_over_body": THROAT_OVER_BODY,
            "throat_area_fraction": area_frac,
            "tail_over_chamber_diameter": TAIL_OVER_CHAMBER_DIA,
            "cone_fraction_of_tail_section": CONE_FRACTION_OF_TAIL,
            "cone_length_m": cone,
            "tailpipe_length_m": tail,
            "duct_length_m": duct,
            "body_length_m": body,
            "fineness_ratio": body / d_body,
            "frontal_area_m2": frontal_new,
            "frontal_area_change_vs_v3a": frontal_new / frontal_v3a - 1.0,
        },
        "verified_mission": {
            "note": ("Nothing is verified yet. Engine-level evidence only: "
                     "out_medium_model/v3b_cliff.json measured 149.2 N at "
                     "M 0.15 / 60 m for this duct (FP pulsejet, n=200) vs "
                     "V3a's 123.6 N, with the sustain cliff bracketed at "
                     "t/D 3.60 dead / 3.70 alive. No FP flight has been run.")
        },
        "change": ("Chamber 214.0 -> 235.0 mm (body 225.217 -> 247.368 mm, "
                   "+20.6% frontal area), throat 121.617 -> 133.579 mm at the "
                   "same 0.2916 area fraction, tail/chamber-dia 4.80 -> 4.00 "
                   "(throat_length_m 1.2423 -> 1.1371 m). Body 2307.3 -> "
                   "2268.5 mm, fineness 10.25 -> 9.17. Chamber length, wing, "
                   "and the frozen climb-dive trajectory are unchanged. Dry "
                   "mass recomputed from geometry (see optimizer_results."
                   "provenance); V3a's own block is stale -- it was copied "
                   "from docs/v3_frozen without rerunning after the 553 mm "
                   "tail extension.")
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(design, indent=2))
    print(f"\nwrote {OUT}")

    # round-trip through the loader the screen will use
    from medium_model.design import load_frozen_design
    d = load_frozen_design(OUT)
    print(f"loader round-trip: loaded_fuel {d.loaded_fuel_kg:.6f} kg, "
          f"burn_limit {d.burn_limit_kg:.6f} kg, "
          f"climb_dive={d.climb_dive is not None}")


if __name__ == "__main__":
    main()
