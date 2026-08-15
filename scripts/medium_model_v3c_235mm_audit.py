"""INDEPENDENT AUDIT of the 235 mm-chamber claim (adversarial re-check).

Written to REFUTE, not confirm.  Checks, in order:
  1. mass block  -- recompute vehicle_dry_mass for BOTH geometries from
     scratch; confirm the design.json block is self-consistent and that the
     fuel budget actually flown (burn cap) is what the file claims.
  2. drag        -- component build-up at a FIXED (M, alt, mass) for both,
     term by term, with the frontal-area ratio printed next to it.
  3. geometry    -- throat area fraction vs the cap, operability flags,
     tail/chamber-dia vs the measured sustain cliff, and (the thing the
     report does NOT audit) whether the RAMJET also grew.
  4. re-fly      -- 3 key closed-form points reproduced from scratch.
Nothing here modifies an existing file.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from douglas_dart.atmosphere import standard_atmosphere          # noqa: E402
from medium_model import constants as C                          # noqa: E402
import medium_model.ramjet_simple as rs                          # noqa: E402
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.drag_buildup import total_drag_buildup         # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile             # noqa: E402
from medium_model.mass_model import vehicle_dry_mass             # noqa: E402
from medium_model.mission import (MAX_WET_MASS_KG, burn_limit_kg,  # noqa: E402
                                  loaded_fuel_kg, usable_fuel_kg)
from medium_model.pulsejet_simple import pulsejet_thrust         # noqa: E402
from medium_model.ramjet_simple import ramjet_thrust             # noqa: E402

V3A = "docs/v3a_medium_model/design.json"
NEW = "docs/v3c_235mm/design.json"
FLOOR_M = 122.0
TARGET_G = 0.26


def hdr(s):
    print("\n" + "=" * 78)
    print(s)
    print("=" * 78)


# ---------------------------------------------------------------- 1. mass
def audit_mass():
    hdr("1. MASS BLOCK -- recomputed from geometry, both vehicles")
    out = {}
    for tag, path in (("V3a", V3A), ("235mm", NEW)):
        raw = json.loads(Path(path).read_text())
        c, w = raw["vehicle_candidate"], raw["wing_concept"]
        opt = raw["optimizer_results"]
        S = w["span_m"] ** 2 / w["aspect_ratio"]
        loaded_from_file = loaded_fuel_kg(opt["dry_mass_kg"],
                                          opt["mass_margin_kg"])
        m = vehicle_dry_mass(c["diameter_m"], c["chamber_length_m"],
                             c["throat_diameter_m"], c["throat_length_m"],
                             S, loaded_from_file)
        d = load_frozen_design(path)
        vol = usable_fuel_kg(c["diameter_m"], c["throat_diameter_m"],
                             c["throat_length_m"],
                             C.FUELS[c["fuel_key"]].density_kg_per_m3)
        out[tag] = dict(
            claimed_dry=opt["dry_mass_kg"], claimed_margin=opt["mass_margin_kg"],
            recomputed_dry=m.dry_mass_kg, loaded=loaded_from_file,
            burn_cap_loader=d.burn_limit_kg, loaded_loader=d.loaded_fuel_kg,
            duct=m.engine_duct_kg, skin=m.airframe_skin_kg, wing=m.wing_kg,
            tank=m.tank_hardware_kg, t_wall=m.duct_wall_thickness_m,
            wing_area=S, vol_cap=vol)
    print(f"{'':22} {'V3a':>14} {'235mm':>14} {'delta':>10}")
    for key, lab in (("claimed_dry", "design.json dry"),
                     ("recomputed_dry", "RECOMPUTED dry"),
                     ("claimed_margin", "design.json margin"),
                     ("loaded", "loaded fuel (derived)"),
                     ("loaded_loader", "loaded via loader"),
                     ("burn_cap_loader", "BURN CAP flown"),
                     ("duct", "  engine_duct_kg"),
                     ("skin", "  airframe_skin_kg"),
                     ("wing", "  wing_kg"),
                     ("tank", "  tank_hardware_kg"),
                     ("vol_cap", "annulus usable kg")):
        a, b = out["V3a"][key], out["235mm"][key]
        print(f"{lab:22} {a:14.6f} {b:14.6f} {b-a:+10.6f}")
    for tag in out:
        o = out[tag]
        err = abs(o["recomputed_dry"] - o["claimed_dry"])
        print(f"  {tag}: |recomputed - claimed| dry = {err:.3e} kg "
              f"({'SELF-CONSISTENT' if err < 1e-9 else 'STALE / COPIED'})")
        print(f"       duct wall gauge {o['t_wall']*1e3:.4f} mm "
              f"(min gauge {C.STEEL_MIN_GAUGE_M*1e3:.3f} mm)")
        print(f"       wing ref area {o['wing_area']:.6f} m^2")
    # what would the budget be if margin, not loaded fuel, were held?
    o = out["235mm"]
    alt_loaded = MAX_WET_MASS_KG - o["recomputed_dry"] - out["V3a"]["claimed_margin"]
    print(f"\n  COUNTERFACTUAL: if payload margin were held at V3a's claimed "
          f"{out['V3a']['claimed_margin']:.4f} kg instead of fuel,")
    print(f"  the 235 mm loaded fuel would be {alt_loaded:+.4f} kg "
          f"(burn cap {burn_limit_kg(max(alt_loaded,0)):.4f} kg).")
    a_true = vehicle_dry_mass(
        json.loads(Path(V3A).read_text())["vehicle_candidate"]["diameter_m"],
        0.38930734448088206, 0.12161734042955713, 1.2423,
        out["V3a"]["wing_area"], out["V3a"]["loaded"]).dry_mass_kg
    print(f"  V3a dry at ITS OWN geometry = {a_true:.4f} kg vs the "
          f"{out['V3a']['claimed_dry']:.4f} kg its file claims "
          f"({a_true-out['V3a']['claimed_dry']:+.4f} kg).")
    return out


# ---------------------------------------------------------------- 2. drag
def audit_drag():
    hdr("2. DRAG -- component build-up at FIXED (M, alt, mass), both vehicles")
    conds = [(0.30, 300.0), (0.50, 300.0), (0.80, 300.0), (1.05, 300.0),
             (0.50, 1500.0)]
    designs = {t: load_frozen_design(p) for t, p in (("V3a", V3A), ("235mm", NEW))}
    for mach, alt in conds:
        atm = standard_atmosphere(alt)
        v = mach * atm.speed_of_sound_m_per_s
        print(f"\n  M {mach:.2f}, {alt:.0f} m, mass {MAX_WET_MASS_KG:.4f} kg, "
              f"engine ON, level flight")
        print(f"    {'term':>14} {'V3a':>10} {'235mm':>10} {'ratio':>8}")
        cache = {}
        for tag, d in designs.items():
            g, w = d.geometry, d.wing
            body_len = (g.chamber_length_m + g.throat_length_m
                        + C.NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m)
            S = w.span_m ** 2 / w.aspect_ratio
            lip = math.pi * g.throat_diameter_m ** 2 / 4.0
            # ramjet capture at this condition, so the spillage term is real
            rj = ramjet_thrust(g.diameter_m, g.throat_diameter_m, mach, alt,
                               g.fuel)
            mdot = rj.air_mass_flow_kg_per_s if rj.net_thrust_n > 0 else \
                0.35 * atm.density_kg_per_m3 * v * lip
            b = total_drag_buildup(
                diameter_m=g.diameter_m, body_length_m=body_len,
                duct_exit_diameter_m=g.throat_diameter_m,
                tail_length_m=C.TAIL_LENGTH_DIAMETERS * g.diameter_m,
                wing_reference_area_m2=S,
                wing_thickness_ratio=w.airfoil.thickness_ratio,
                wing_sweep_deg=w.sweep_deg, velocity_m_per_s=v,
                density_kg_per_m3=atm.density_kg_per_m3,
                temperature_k=atm.temperature_k, mach=mach,
                required_lift_n=MAX_WET_MASS_KG * 9.80665,
                oswald_efficiency=w.oswald_e,
                wing_aspect_ratio=w.aspect_ratio, engine_on=True,
                captured_mass_flow_kg_per_s=mdot, lip_area_m2=lip)
            cache[tag] = (b, math.pi * g.diameter_m ** 2 / 4.0)
        for i, name in enumerate(("friction", "form", "base", "wave",
                                  "spillage", "wing_prof", "induced")):
            a, bb = cache["V3a"][0][i], cache["235mm"][0][i]
            r = bb / a if a else float("nan")
            print(f"    {name:>14} {a:10.2f} {bb:10.2f} {r:8.4f}")
        a, bb = cache["V3a"][0].total_n, cache["235mm"][0].total_n
        print(f"    {'TOTAL':>14} {a:10.2f} {bb:10.2f} {bb/a:8.4f}")
        fa, fb = cache["V3a"][1], cache["235mm"][1]
        print(f"    {'frontal m^2':>14} {fa:10.6f} {fb:10.6f} {fb/fa:8.4f}"
              f"   <-- the +20.6% claim")
        ca = cache["V3a"][0].cd0_equivalent_frontal
        cb = cache["235mm"][0].cd0_equivalent_frontal
        print(f"    {'CD0_equiv':>14} {ca:10.4f} {cb:10.4f} {cb/ca:8.4f}")
        print(f"    frontal-referenced parasite force check: "
              f"CD0equiv*q*A ratio = {(cb*fb)/(ca*fa):.4f}")


# ------------------------------------------------------------ 3. geometry
def audit_geometry():
    hdr("3. GEOMETRY / OPERABILITY / ENGINE SCALING")
    designs = {t: load_frozen_design(p) for t, p in (("V3a", V3A), ("235mm", NEW))}
    for tag, d in designs.items():
        g = d.geometry
        af = (g.throat_diameter_m / g.diameter_m) ** 2
        chamber_dia = g.diameter_m * 0.95
        cone_frac = 0.130 / 0.750
        tail = g.throat_length_m * (1.0 - cone_frac)
        print(f"  {tag}: body {g.diameter_m*1e3:.3f} mm, throat "
              f"{g.throat_diameter_m*1e3:.3f} mm, area frac {af:.5f} "
              f"(cap {C.PULSEJET_MAX_THROAT_AREA_FRACTION}) "
              f"{'OK' if af <= C.PULSEJET_MAX_THROAT_AREA_FRACTION else 'VIOLATION'}")
        print(f"        implied chamber dia {chamber_dia*1e3:.3f} mm, "
              f"tailpipe {tail*1e3:.2f} mm, t/D = {tail/chamber_dia:.4f}")
    print("\n  cliff evidence (out_medium_model/v3b_cliff.json, chamber 235 mm):")
    for row in json.loads(Path("out_medium_model/v3b_cliff.json").read_text()):
        print(f"    t/D {row['t_over_d']:.2f}  thrust {row['thrust_n']:8.2f} N  "
              f"sustains={row['sustains']}  body {row['body_mm']:.1f} mm")

    print("\n  closed-form engine thrust at 122 m -- BOTH engines, not just "
          "the pulsejet")
    print(f"    {'M':>5} {'PJ V3a':>9} {'PJ 235':>9} {'ratio':>7} "
          f"{'RJ V3a':>9} {'RJ 235':>9} {'ratio':>7} {'opA/opB':>8}")
    rs.RAMJET_MIN_LIGHTOFF_MACH = 0.45
    for mach in (0.15, 0.30, 0.45, 0.50, 0.60, 0.80, 1.00, 1.10):
        vals = []
        for tag, d in designs.items():
            g = d.geometry
            pj = pulsejet_thrust(g.diameter_m, g.chamber_length_m,
                                 g.throat_diameter_m, g.throat_length_m,
                                 mach, 122.0, g.fuel)
            rj = ramjet_thrust(g.diameter_m, g.throat_diameter_m, mach,
                               122.0, g.fuel)
            vals.append((pj, rj))
        (pa, ra), (pb, rb) = vals
        pr = pb.average_thrust_n / pa.average_thrust_n if pa.average_thrust_n else float("nan")
        rr = rb.net_thrust_n / ra.net_thrust_n if ra.net_thrust_n else float("nan")
        print(f"    {mach:5.2f} {pa.average_thrust_n:9.2f} "
              f"{pb.average_thrust_n:9.2f} {pr:7.4f} "
              f"{ra.net_thrust_n:9.2f} {rb.net_thrust_n:9.2f} {rr:7.4f} "
              f"{str(pa.operable)[0]+'/'+str(pb.operable)[0]:>8}")
    print(f"    pulsejet freq: V3a {pulsejet_thrust(*_g(designs['V3a']), 0.5, 122.0, designs['V3a'].geometry.fuel).frequency_hz:.1f} Hz, "
          f"235 {pulsejet_thrust(*_g(designs['235mm']), 0.5, 122.0, designs['235mm'].geometry.fuel).frequency_hz:.1f} Hz "
          f"(cap {C.PULSEJET_MAX_FREQUENCY_HZ} Hz)")


def _g(d):
    g = d.geometry
    return (g.diameter_m, g.chamber_length_m, g.throat_diameter_m,
            g.throat_length_m)


# --------------------------------------------------------------- 4. re-fly
def audit_reflights():
    hdr("4. RE-FLY -- key closed-form points reproduced from scratch")
    designs = {t: load_frozen_design(p) for t, p in (("V3a", V3A), ("235mm", NEW))}
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    print(f"  {'vehicle':>8} {'climb':>6} {'dive':>5} {'gate':>5} {'trav':>7} "
          f"{'pow':>7} {'margin':>7} {'peakM':>6} {'fuel':>6} {'cut':>5} "
          f"{'PASS':>5}")
    rows = []
    try:
        for tag, d in designs.items():
            for climb, dive, gate in ((12.0, 20.0, 0.45), (12.0, 20.0, 0.49),
                                      (12.0, 20.0, 0.50), (12.0, 20.0, 0.51),
                                      (14.0, 26.0, 0.54), (10.0, 26.0, 0.52)):
                cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                      dive_angle_deg=dive,
                                      floor_altitude_m=FLOOR_M)
                rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                r = fly(d, drag_model="buildup", climb_dive=cd)
                peak = max(s.mach for s in r.states)
                fuel = max(s.fuel_burned_kg for s in r.states)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                print(f"  {tag:>8} {climb:6.1f} {dive:5.1f} {gate:5.2f} "
                      f"{r.min_traverse_accel_g:7.3f} "
                      f"{r.min_powered_accel_g:7.3f} "
                      f"{r.min_powered_thrust_margin:7.3f} {peak:6.3f} "
                      f"{fuel:6.3f} {str(bool(r.motor_cutoff_reached)):>5} "
                      f"{str(bool(ok)):>5}")
                rows.append(dict(vehicle=tag, climb=climb, dive=dive,
                                 gate=gate, traverse=r.min_traverse_accel_g,
                                 peak=peak, fuel=fuel, passes=bool(ok)))
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original
    Path("out_medium_model/v3c_235mm_audit.json").write_text(
        json.dumps(rows, indent=2))


if __name__ == "__main__":
    audit_mass()
    audit_drag()
    audit_geometry()
    audit_reflights()
