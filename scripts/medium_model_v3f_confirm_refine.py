"""Refine the FP CONFIRM pick around the steep-dive knee.

Scoring rule derived in medium_model_v3f_confirm_pick.py from SIX paired
closed-form/FP flights of the V3a airframe:

    FP traverse  = 0.781 .. 0.798 x closed-form   (worst 0.781)
    FP fuel burn = 1.004 .. 1.069 x closed-form   (worst 1.069)

...but ONLY while the closed-form traverse is comfortably clear of the
gate.  The sixth pair (cf 0.212) collapsed to fp 0.011 -- a cliff, not a
scaling.  So the pick must carry headroom, not sit on 0.26/0.781.

A candidate is FP-DEFENSIBLE here iff
    cf_traverse * 0.781 >= 0.26        and
    cf_fuel     * 1.069 <  2.423931    and
    peak Mach >= 1.0 with cutoff.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3f_confirm_refine.py
"""
from __future__ import annotations

import json
import os
import time
from math import cos, degrees, radians
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                      # noqa: E402
from medium_model.constants import G0_M_PER_S2               # noqa: E402
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import (V3_FLOOR_ALTITUDE_M,    # noqa: E402
                                     ClimbDiveProfile)
from medium_model.lift import wing_clmax                     # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere      # noqa: E402

OUT = Path("out_medium_model")
DESIGNS = {"V3a": "docs/v3a_medium_model/design.json",
           "235mm": "docs/v3c_235mm/design.json"}

TRAV_HAIRCUT = 0.781
FUEL_PENALTY = 1.069
CAP = 2.423931


def probe(d, climb, dive, floor, gate, dt=0.02):
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    prof = ClimbDiveProfile(initial_climb_angle_deg=climb,
                            dive_angle_deg=dive,
                            floor_altitude_m=max(floor, V3_FLOOR_ALTITUDE_M))
    r = fly(d, drag_model="buildup", climb_dive=prof, dt_s=dt)
    fuel = max(s.fuel_burned_kg for s in r.states)
    peak = max(s.mach for s in r.states)
    trav = r.min_traverse_accel_g
    exit_ = None
    for s in r.states:
        if s.mode == "v3_dive":
            exit_ = s
    return dict(design=d.name.split(":")[0], climb=climb, dive=dive,
                floor=floor, gate=round(gate, 3), dt=dt,
                traverse=trav, fp_traverse=trav * TRAV_HAIRCUT,
                fuel=fuel, fp_fuel=fuel * FUEL_PENALTY,
                peak_mach=peak, cutoff=bool(r.motor_cutoff_reached),
                margin=r.min_powered_thrust_margin,
                trav_mach=r.min_traverse_accel_mach,
                top_m=r.climb_dive_top_altitude_m,
                rule_violated=bool(r.rule_violated),
                exit_mach=exit_.mach if exit_ else None,
                exit_alt=exit_.altitude_m if exit_ else None,
                exit_mass=exit_.mass_kg if exit_ else None,
                cf_pass=bool(trav >= 0.26 and peak >= 1.0
                             and r.motor_cutoff_reached and fuel < CAP),
                fp_pass=bool(trav * TRAV_HAIRCUT >= 0.26 and peak >= 1.0
                             and r.motor_cutoff_reached
                             and fuel * FUEL_PENALTY < CAP))


t0 = time.time()
rows = []
GRID = [(n, c, dv, g)
        for n in ("V3a", "235mm")
        for dv in (29.0, 30.0, 31.0, 32.0, 33.0)
        for c in (9.0, 10.0, 11.0)
        for g in (0.51, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57)]
cache = {n: load_frozen_design(p) for n, p in DESIGNS.items()}
for n, c, dv, g in GRID:
    rows.append(probe(cache[n], c, dv, 121.92, g))
print(f"grid done {time.time()-t0:.0f} s")

hdr = (f"{'design':7s}{'dive':>5}{'climb':>6}{'gate':>6}{'cf_trav':>8}"
       f"{'fp_trav':>8}{'cf_fuel':>8}{'fp_fuel':>8}{'marg':>7}"
       f"{'exitM':>7}{'FP?':>5}")
print(hdr)
for r in rows:
    if r["gate"] >= 0.53:
        print(f"{r['design']:7s}{r['dive']:5.0f}{r['climb']:6.0f}"
              f"{r['gate']:6.2f}{r['traverse']:8.3f}{r['fp_traverse']:8.3f}"
              f"{r['fuel']:8.3f}{r['fp_fuel']:8.3f}{r['margin']:7.3f}"
              f"{r['exit_mach']:7.3f}{'YES' if r['fp_pass'] else '-':>5}")

# ---- timestep convergence + pull-out demand on the leading FP-pass cells
best = {}
for r in rows:
    if r["fp_pass"]:
        k = r["design"]
        if k not in best or (r["gate"], r["fp_traverse"]) > (best[k]["gate"],
                                                            best[k]["fp_traverse"]):
            best[k] = r
conv, pullout = [], []
for k, b in best.items():
    for dt in (0.01, 0.005):
        conv.append(probe(cache[k], b["climb"], b["dive"], b["floor"],
                          b["gate"], dt=dt))
    atm = standard_atmosphere(b["exit_alt"])
    v = b["exit_mach"] * atm.speed_of_sound_m_per_s
    q = 0.5 * atm.density_kg_per_m3 * v * v
    w = cache[k].wing
    S = w.span_m ** 2 / w.aspect_ratio
    clmax = wing_clmax(w.airfoil.cl_max, w.aspect_ratio, w.taper_ratio,
                       w.sweep_deg, b["exit_mach"])
    cl = getattr(clmax, "cl_max", None) or getattr(clmax, "clmax", None) \
        or float(clmax)
    n_avail = q * S * cl / (b["exit_mass"] * G0_M_PER_S2)
    gd = radians(b["dive"])
    for dh in (78.0, 128.0, 178.0):
        R = dh / (1.0 - cos(gd))
        n_req = 1.0 + v * v / (G0_M_PER_S2 * R)
        pullout.append(dict(design=k, dive=b["dive"], dh_m=dh,
                            n_required=n_req, n_available=n_avail,
                            radius_m=R, v_m_s=v, cl_max=cl, S_m2=S))
    print(f"\n{k} pick: climb {b['climb']:.0f} dive {b['dive']:.0f} "
          f"gate {b['gate']:.2f} | cf_trav {b['traverse']:.3f} -> fp "
          f"{b['fp_traverse']:.3f} | cf_fuel {b['fuel']:.3f} -> fp "
          f"{b['fp_fuel']:.3f} | exit M {b['exit_mach']:.3f} @ "
          f"{b['exit_alt']:.0f} m | n_avail {n_avail:.1f}")
for c in conv:
    print(f"  dt {c['dt']:.4f}: traverse {c['traverse']:.4f} "
          f"fuel {c['fuel']:.4f} peak {c['peak_mach']:.4f} ({c['design']})")
for p in pullout:
    print(f"  pullout {p['design']} dh {p['dh_m']:.0f} m -> n_req "
          f"{p['n_required']:.2f} of {p['n_available']:.1f} available")

json.dump(dict(grid=rows, convergence=conv, pullout=pullout,
               haircut=dict(traverse=TRAV_HAIRCUT, fuel=FUEL_PENALTY,
                            cap=CAP)),
          open(OUT / "v3f_confirm_refine.json", "w"), indent=1)
print("wrote", OUT / "v3f_confirm_refine.json", f"{time.time()-t0:.0f} s")
