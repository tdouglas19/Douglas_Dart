"""ADVERSARIAL AUDIT part 2 -- does the quasi-static balance PREDICT the
observed ceiling?

Two halves, deliberately independent of each other:

A. QUASI-STATIC PREDICTION.  Rebuild the mechanism agent's balance from
   scratch with medium_model's own functions:

       a/g = (T_pulsejet + T_ramjet_ramped - D)/W + sin(dive)

   and solve for the Mach where a/g = 0.26, at a range of altitudes, for
   each dive angle.  Reported for BOTH duct states (cold throughflow and
   lit/throat-limited spillage) so the lightoff step is visible rather
   than averaged away.

B. MEASURED CEILING.  Fine gate bisection at 0.01 Mach resolution on real
   flights, best over a set of climb angles, for each dive angle.

Then A is scored against B.  "Within one 0.05 gate step" is the bar.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_balance.py
"""
from __future__ import annotations

import json
import os
from math import pi, radians, sin
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                          # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere          # noqa: E402
from medium_model.constants import (G0_M_PER_S2,                 # noqa: E402
                                    NOSE_TAIL_LENGTH_DIAMETERS,
                                    TAIL_LENGTH_DIAMETERS)
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.flight_sim import (ClimbDiveProfile,           # noqa: E402
                                     _buildup_drag,
                                     _duct_swallowed_kg_per_s)
from medium_model.pulsejet_simple import pulsejet_thrust         # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0
MID_DIVE_MASS_KG = 22.04      # measured mid-dive mass, matches the reports

DIVES = (9.890058542648028, 12.0, 14.0, 16.0, 20.0, 25.0, 30.0)
ALTS = (130.0, 240.0, 300.0, 345.0, 600.0, 900.0)
CLIMBS = (8.0, 10.0, 12.0, 14.0)
GATE_LO, GATE_HI, GATE_STEP = 0.30, 0.60, 0.01


def make_cache(d):
    g = d.geometry
    w = d.wing
    return {
        "diameter_m": g.diameter_m,
        "duct_exit_diameter_m": g.throat_diameter_m,
        "tail_length_m": TAIL_LENGTH_DIAMETERS * g.diameter_m,
        "body_length_m": (g.chamber_length_m + g.throat_length_m
                          + NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m),
        "wing_area_m2": w.span_m ** 2 / w.aspect_ratio,
        "aspect_ratio": w.aspect_ratio,
        "wing_sweep_deg": w.sweep_deg,
        "wing_thickness_ratio": w.airfoil.thickness_ratio,
        "oswald_e": w.oswald_e,
        "lip_area_m2": pi * g.throat_diameter_m ** 2 / 4.0,
        "cowl_suction_recovery": None,
    }


def balance(d, cache, mach, alt, dive_deg, mass_kg, gate):
    """(T - D)/W + sin(dive), evaluated exactly as the integrator would."""
    atm = standard_atmosphere(alt)
    v = mach * atm.speed_of_sound_m_per_s
    g = d.geometry
    old = rs.RAMJET_MIN_LIGHTOFF_MACH
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    try:
        rj = rs.ramjet_thrust(g.diameter_m, g.throat_diameter_m, mach, alt,
                              g.fuel, atmosphere=atm)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = old
    pj = pulsejet_thrust(g.diameter_m, g.chamber_length_m, g.throat_diameter_m,
                         g.throat_length_m, mach, alt, g.fuel, atmosphere=atm)
    thrust = pj.average_thrust_n + rj.net_thrust_n
    mdot = _duct_swallowed_kg_per_s(rj, atm.density_kg_per_m3, v,
                                    cache["lip_area_m2"])
    dr, _bd = _buildup_drag(cache, d.wing, v, atm.density_kg_per_m3,
                            atm.temperature_k, mass_kg, -radians(dive_deg),
                            mach, engine_on=True, captured_mdot_kg_per_s=mdot)
    w_n = mass_kg * G0_M_PER_S2
    return ((thrust - dr.total_n) / w_n + sin(radians(dive_deg)),
            pj.average_thrust_n, rj.net_thrust_n, dr.total_n, w_n)


def ceiling_mach(d, cache, alt, dive_deg, mass_kg, target=TARGET_G):
    """Highest Mach at which a/g >= target, gate set = that Mach (so the
    ramjet contributes ~nothing: the worst instant, which is where the
    integrator's min traverse actually lands)."""
    lo, hi, best = 0.20, 0.90, None
    m = lo
    while m <= hi + 1e-9:
        a = balance(d, cache, m, alt, dive_deg, mass_kg, gate=m)[0]
        if a >= target:
            best = m
        m += 0.005
    return best


def measured_ceiling(d, dive_deg, climbs=CLIMBS):
    """Fine bisection of the true gate ceiling: highest gate (0.01 steps)
    at which SOME climb angle closes all three gates."""
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    best_gate, best_row = None, None
    try:
        gate = GATE_HI
        while gate >= GATE_LO - 1e-9:
            rs.RAMJET_MIN_LIGHTOFF_MACH = round(gate, 3)
            for climb in climbs:
                cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                      dive_angle_deg=dive_deg,
                                      floor_altitude_m=FLOOR_M)
                r = fly(d, drag_model="buildup", climb_dive=cd)
                peak = max(s.mach for s in r.states)
                fuel = max(s.fuel_burned_kg for s in r.states)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                if ok:
                    best_gate = round(gate, 3)
                    best_row = dict(climb=climb, gate=best_gate,
                                    traverse=r.min_traverse_accel_g,
                                    traverse_mach=r.min_traverse_accel_mach,
                                    margin=r.min_powered_thrust_margin,
                                    fuel=fuel, peak=peak,
                                    top_m=r.climb_dive_top_altitude_m)
                    break
            if best_gate is not None:
                break
            gate -= GATE_STEP
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    return best_gate, best_row


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    cache = make_cache(d)

    print("=== A. thrust/drag/gravity balance, medium_model closed-form ===")
    print(f"mass {MID_DIVE_MASS_KG} kg, drag build-up at dive attitude, "
          f"gate == the Mach evaluated (ramjet ~0 there)\n")
    print(f"{'alt':>5} " + " ".join(f"{'d%.0f' % dv:>7}" for dv in DIVES))
    pred = {}
    for alt in ALTS:
        cells = []
        for dv in DIVES:
            c = ceiling_mach(d, cache, alt, dv, MID_DIVE_MASS_KG)
            pred[(alt, dv)] = c
            cells.append(f"{c:7.3f}" if c else "   none")
        print(f"{alt:5.0f} " + " ".join(cells), flush=True)

    print("\n--- the raw balance at 300 m, dive 20, gate == M ---")
    print(f"{'M':>5} {'T_pj':>7} {'T_rj':>7} {'D':>7} {'e_g':>7} {'a_g':>7}")
    for m in (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60):
        a, tpj, trj, dn, wn = balance(d, cache, m, 300.0, 20.0,
                                      MID_DIVE_MASS_KG, gate=m)
        print(f"{m:5.2f} {tpj:7.1f} {trj:7.1f} {dn:7.1f} "
              f"{(tpj + trj - dn) / wn:7.3f} {a:7.3f}")

    print("\n=== B. MEASURED ceiling, 0.01 gate resolution, real flights ===")
    print(f"climbs {CLIMBS}, floor {FLOOR_M} m\n")
    print(f"{'dive':>6} {'measured':>9} {'climb':>6} {'trav':>6} {'@M':>6} "
          f"{'margin':>7} {'fuel':>6} {'top_m':>7}")
    meas = {}
    for dv in DIVES:
        gate, row = measured_ceiling(d, dv)
        meas[dv] = gate
        if row:
            print(f"{dv:6.2f} {gate:9.2f} {row['climb']:6.1f} "
                  f"{row['traverse']:6.3f} {row['traverse_mach']:6.3f} "
                  f"{row['margin']:7.3f} {row['fuel']:6.3f} "
                  f"{row['top_m']:7.1f}", flush=True)
        else:
            print(f"{dv:6.2f} {'NONE':>9}", flush=True)

    print("\n=== SCORE: prediction (at each altitude) minus measurement ===")
    print(f"{'dive':>6} {'meas':>6} " + " ".join(f"{'@%.0fm' % a:>8}"
                                                 for a in ALTS))
    scores = []
    for dv in DIVES:
        mv = meas[dv]
        cells = []
        for alt in ALTS:
            p = pred[(alt, dv)]
            if p is None or mv is None:
                cells.append("      --")
            else:
                cells.append(f"{p - mv:+8.3f}")
                scores.append(dict(dive=dv, alt=alt, pred=p, meas=mv,
                                   err=p - mv))
        print(f"{dv:6.2f} {(mv if mv else float('nan')):6.2f} "
              + " ".join(cells))

    (OUT / "v3e_audit_balance.json").write_text(json.dumps(
        dict(predicted={f"{a}|{dv}": pred[(a, dv)] for a in ALTS
                        for dv in DIVES},
             measured={str(dv): meas[dv] for dv in DIVES},
             scores=scores), indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_balance.json'}")


if __name__ == "__main__":
    main()
