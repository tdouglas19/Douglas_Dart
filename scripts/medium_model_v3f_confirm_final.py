"""Final head-to-head for the ONE first-principles CONFIRM run.

Scores a short list against the FP haircut measured in
medium_model_v3f_confirm_pick.py (six paired CF/FP flights of V3a):
    traverse x 0.781 (worst observed),  fuel x 1.069 (worst observed).

Also reports, for each finalist:
  * where the dive ENDS (Mach-limited at dive_end_mach 0.60, or
    floor-limited) and how much altitude is left over the floor -- the
    room a real, non-instantaneous pull-out would need;
  * the load factor a constant-n pull-out would demand vs what
    lift.wing_clmax says the wing can deliver;
  * timestep convergence at dt 0.02 / 0.01 / 0.005 s.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3f_confirm_final.py
"""
from __future__ import annotations

import json
import os
import time
from math import cos, radians
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
PATHS = {"V3a": "docs/v3a_medium_model/design.json",
         "235mm": "docs/v3c_235mm/design.json"}
TRAV_HAIRCUT, FUEL_PENALTY, CAP = 0.781, 1.069, 2.423931

# design, climb, dive, floor, gate
SHORTLIST = [
    ("235mm", 11.0, 33.0, 121.92, 0.54),
    ("235mm", 10.0, 33.0, 121.92, 0.54),
    ("235mm", 11.0, 32.0, 121.92, 0.54),
    ("235mm", 11.0, 30.0, 121.92, 0.54),
    ("235mm", 11.0, 33.0, 121.92, 0.55),
    ("235mm", 10.0, 33.0, 121.92, 0.55),
    ("235mm", 11.0, 30.0, 121.92, 0.53),
    ("235mm", 10.0, 30.0, 121.92, 0.53),
    ("V3a",    9.0, 33.0, 121.92, 0.53),
    ("V3a",    9.0, 32.0, 121.92, 0.53),
    ("V3a",   10.0, 33.0, 121.92, 0.52),
    ("V3a",   10.0, 30.0, 121.92, 0.52),
]

cache = {k: load_frozen_design(p) for k, p in PATHS.items()}


def probe(key, climb, dive, floor, gate, dt=0.02):
    d = cache[key]
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    prof = ClimbDiveProfile(initial_climb_angle_deg=climb,
                            dive_angle_deg=dive,
                            floor_altitude_m=max(floor, V3_FLOOR_ALTITUDE_M))
    r = fly(d, drag_model="buildup", climb_dive=prof, dt_s=dt)
    fuel = max(s.fuel_burned_kg for s in r.states)
    peak = max(s.mach for s in r.states)
    trav = r.min_traverse_accel_g
    dive_states = [s for s in r.states if s.mode == "v3_dive"]
    ex = dive_states[-1] if dive_states else None
    en = dive_states[0] if dive_states else None
    return dict(design=key, climb=climb, dive=dive, floor=floor,
                gate=gate, dt=dt, traverse=trav,
                fp_traverse=trav * TRAV_HAIRCUT, fuel=fuel,
                fp_fuel=fuel * FUEL_PENALTY, peak_mach=peak,
                cutoff=bool(r.motor_cutoff_reached),
                margin=r.min_powered_thrust_margin,
                trav_mach=r.min_traverse_accel_mach,
                top_m=r.climb_dive_top_altitude_m,
                rule_violated=bool(r.rule_violated),
                dive_start_mach=en.mach if en else None,
                dive_start_alt=en.altitude_m if en else None,
                exit_mach=ex.mach if ex else None,
                exit_alt=ex.altitude_m if ex else None,
                exit_mass=ex.mass_kg if ex else None,
                dive_drop_m=(en.altitude_m - ex.altitude_m) if ex else None,
                trav_headroom=trav * TRAV_HAIRCUT / 0.26 - 1.0,
                fuel_headroom=1.0 - fuel * FUEL_PENALTY / CAP,
                fp_pass=bool(trav * TRAV_HAIRCUT >= 0.26 and peak >= 1.0
                             and r.motor_cutoff_reached
                             and fuel * FUEL_PENALTY < CAP))


def pullout(key, row):
    w = cache[key].wing
    S = w.span_m ** 2 / w.aspect_ratio
    atm = standard_atmosphere(row["exit_alt"])
    v = row["exit_mach"] * atm.speed_of_sound_m_per_s
    q = 0.5 * atm.density_kg_per_m3 * v * v
    cm = wing_clmax(w.airfoil.cl_max, w.aspect_ratio, w.taper_ratio,
                    w.sweep_deg, row["exit_mach"])
    cl = getattr(cm, "cl_max", None)
    if cl is None:
        cl = getattr(cm, "clmax", None) or float(cm)
    n_av = q * S * cl / (row["exit_mass"] * G0_M_PER_S2)
    gd = radians(row["dive"])
    out = []
    for dh in (78.0, 128.0, 178.0):
        R = dh / (1.0 - cos(gd))
        out.append(dict(dh_m=dh, radius_m=R,
                        n_required=1.0 + v * v / (G0_M_PER_S2 * R),
                        n_available=n_av, cl_max=cl, S_m2=S, v_m_s=v))
    return out


t0 = time.time()
rows = [probe(*s) for s in SHORTLIST]
print(f"{'design':7s}{'clm':>4}{'dive':>5}{'gate':>6}{'cfTrv':>7}{'fpTrv':>7}"
      f"{'hd%':>6}{'cfFuel':>7}{'fpFuel':>7}{'hd%':>6}{'marg':>6}"
      f"{'topM':>6}{'divM0':>6}{'exitM':>6}{'exitH':>6}{'FP?':>5}")
for r in rows:
    print(f"{r['design']:7s}{r['climb']:4.0f}{r['dive']:5.0f}{r['gate']:6.2f}"
          f"{r['traverse']:7.3f}{r['fp_traverse']:7.3f}"
          f"{100*r['trav_headroom']:6.1f}{r['fuel']:7.3f}{r['fp_fuel']:7.3f}"
          f"{100*r['fuel_headroom']:6.1f}{r['margin']:6.3f}"
          f"{r['top_m']:6.0f}{r['dive_start_mach']:6.3f}"
          f"{r['exit_mach']:6.3f}{r['exit_alt']:6.0f}"
          f"{'YES' if r['fp_pass'] else '-':>5}")

FINAL = [("235mm", 11.0, 33.0, 121.92, 0.54),
         ("235mm", 10.0, 33.0, 121.92, 0.54),
         ("V3a", 9.0, 33.0, 121.92, 0.53)]
conv, po = [], []
for s in FINAL:
    for dt in (0.01, 0.005):
        conv.append(probe(*s, dt=dt))
    base = [r for r in rows
            if (r["design"], r["climb"], r["dive"], r["floor"], r["gate"]) == s][0]
    po.append(dict(cfg=s, rows=pullout(s[0], base)))

print("\ntimestep convergence")
for c in conv:
    print(f"  {c['design']:7s} clm{c['climb']:.0f} dv{c['dive']:.0f} "
          f"g{c['gate']:.2f} dt {c['dt']:.4f}: traverse {c['traverse']:.4f} "
          f"fuel {c['fuel']:.4f} peak {c['peak_mach']:.4f}")
print("\npull-out demand (constant-n arc from -dive to level)")
for p in po:
    for r in p["rows"]:
        print(f"  {p['cfg'][0]:7s} dive {p['cfg'][2]:.0f} dh {r['dh_m']:5.0f} m"
              f" -> n_req {r['n_required']:5.2f} of n_avail "
              f"{r['n_available']:5.1f}  (R {r['radius_m']:.0f} m, "
              f"V {r['v_m_s']:.0f} m/s, CLmax {r['cl_max']:.3f})")

json.dump(dict(shortlist=rows, convergence=conv, pullout=po),
          open(OUT / "v3f_confirm_final.json", "w"), indent=1)
print("wrote", OUT / "v3f_confirm_final.json", f"{time.time()-t0:.0f} s")
