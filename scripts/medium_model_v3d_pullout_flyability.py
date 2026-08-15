"""Part 3 in earnest: is the pull-out at the floor actually flyable, and
what does it cost the traverse gate?  Plus a fine gate scan between 0.45
and 0.55 to bracket the real ceiling.

flight_sim performs the pull-out in ONE timestep: the phase machine
latches v3_dive_done and gamma jumps from -dive to +climb_angle_deg.
Zero radius, zero altitude, zero time, infinite g, and -- because lift is
always m*g*cos(gamma) in the drag calls -- zero induced-drag penalty.
Everything below is what the model does not charge.

Constant-n circular arc from -gamma_d to level:
    n(gamma) = cos(gamma) + V^2/(g R)   -> worst at the bottom, gamma = 0
    dh       = R (1 - cos gamma_d)
    dt       = R gamma_d / V
    mean gravity assist over the arc = g (1 - cos gamma_d)/gamma_d
      (vs g sin(gamma_d) in the steady dive -- the pull-out GIVES BACK
       part of the gravity term the dive was there to collect)

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3d_pullout_flyability.py
"""
from __future__ import annotations

import json
import os
from math import cos, pi, radians
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                      # noqa: E402
from medium_model.constants import G0_M_PER_S2               # noqa: E402
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.lift import oswald_efficiency, wing_clmax  # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere      # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
SRC = OUT / "v3d_dive_extension.json"
TARGET_G = 0.26
FLOOR_M = 121.92

# altitude the pull-out is allowed to consume.  28 m == start the arc at a
# 150 m floor and bottom out at 122 m; 78 m == start at 200 m; 122 m ==
# start AT the legal floor and arrive at the ground with zero to spare.
BUDGETS = (28.08, 78.08, 121.92)


def pullout_for_budget(w, dive_deg, exit_mach, exit_alt_m, mass_kg,
                       thrust_n, drag_n, dh_budget_m):
    atm = standard_atmosphere(exit_alt_m)
    v = exit_mach * atm.speed_of_sound_m_per_s
    q = 0.5 * atm.density_kg_per_m3 * v * v
    g_r = radians(dive_deg)
    one_minus_cos = 1.0 - cos(g_r)
    radius_m = dh_budget_m / one_minus_cos
    n = 1.0 + v * v / (G0_M_PER_S2 * radius_m)
    dt = radius_m * g_r / v
    weight_n = mass_kg * G0_M_PER_S2
    e = oswald_efficiency(w.aspect_ratio, w.taper_ratio, w.sweep_deg,
                          exit_mach)

    def di(lift_n):
        return lift_n ** 2 / (q * pi * w.span_m ** 2 * e)

    d_induced_n = di(n * weight_n) - di(weight_n * cos(g_r))
    # gravity: the steady dive was collecting g sin(gamma_d); the arc only
    # collects its mean value
    grav_dive_g = (G0_M_PER_S2 * __import__("math").sin(g_r)) / G0_M_PER_S2
    grav_arc_g = one_minus_cos / g_r
    accel_model_g = (thrust_n - drag_n) / (mass_kg * G0_M_PER_S2) + grav_dive_g
    accel_arc_g = ((thrust_n - drag_n - d_induced_n)
                   / (mass_kg * G0_M_PER_S2) + grav_arc_g)

    cl = wing_clmax(w.airfoil.cl_max, w.aspect_ratio, w.taper_ratio,
                    w.sweep_deg, exit_mach)
    n_avail = q * (w.span_m ** 2 / w.aspect_ratio) * cl.clmax / weight_n
    return dict(dive_deg=dive_deg, dh_budget_m=dh_budget_m,
                radius_m=radius_m, n_required=n, n_available=n_avail,
                n_margin=n_avail - n, duration_s=dt,
                d_induced_n=d_induced_n,
                d_induced_g=d_induced_n / weight_n,
                grav_dive_g=grav_dive_g, grav_arc_g=grav_arc_g,
                accel_model_g=accel_model_g, accel_arc_g=accel_arc_g,
                clmax=cl.clmax, mechanism=cl.mechanism,
                clmax_trustworthy=cl.trustworthy,
                v_m_per_s=v, q_pa=q, mass_kg=mass_kg, weight_n=weight_n)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    w = d.wing
    src = json.loads(SRC.read_text())
    original = rs.RAMJET_MIN_LIGHTOFF_MACH

    # ---------- fine gate scan: where is the real ceiling? ----------
    print("=== fine gate scan, climb 12, floor 121.92 m ===")
    print(f"{'dive':>5} {'gate':>5} {'trav':>6} {'pow':>7} {'margin':>7} "
          f"{'peakM':>6} {'cut':>5} {'fuel':>6} {'PASS':>5}")
    fine = []
    try:
        for dive in (25.0, 30.0):
            for gate in (0.46, 0.48, 0.50, 0.51, 0.52, 0.53, 0.54):
                cd = ClimbDiveProfile(initial_climb_angle_deg=12.0,
                                      dive_angle_deg=dive,
                                      floor_altitude_m=FLOOR_M)
                rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                r = fly(d, drag_model="buildup", climb_dive=cd)
                peak = max(s.mach for s in r.states)
                fuel = max(s.fuel_burned_kg for s in r.states)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                fine.append(dict(dive=dive, gate=gate,
                                 traverse=r.min_traverse_accel_g,
                                 powered=r.min_powered_accel_g,
                                 margin=r.min_powered_thrust_margin,
                                 peak=peak, fuel=fuel, passes=bool(ok),
                                 cutoff=bool(r.motor_cutoff_reached),
                                 rule_violated=bool(r.rule_violated)))
                print(f"{dive:5.1f} {gate:5.2f} {r.min_traverse_accel_g:6.3f} "
                      f"{r.min_powered_accel_g:7.3f} "
                      f"{r.min_powered_thrust_margin:7.3f} {peak:6.3f} "
                      f"{str(r.motor_cutoff_reached)[:5]:>5} {fuel:6.3f} "
                      f"{'PASS' if ok else 'fail':>5}", flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original

    # ---------- pull-out flyability ----------
    print("\n=== pull-out: g REQUIRED to fit an altitude budget vs g "
          "AVAILABLE ===")
    print(f"wing area {w.span_m**2/w.aspect_ratio*1e4:.0f} cm^2, "
          f"span {w.span_m*1000:.1f} mm, AR {w.aspect_ratio:.3f}, "
          f"alpha limit 25 deg, vortex lift NOT credited")
    rows = []
    print(f"\n{'gate':>5} {'dive':>5} {'dh_m':>7} {'R_m':>7} {'n_req':>6} "
          f"{'n_avail':>8} {'n_marg':>7} {'dt_s':>5} {'dDi_N':>7} "
          f"{'dDi_g':>6} {'a_mod':>6} {'a_arc':>6} {'verdict':>9}")
    for gate in (0.45, 0.50):
        for r in src["sweep"]:
            if r["climb"] != 12.0 or r["gate"] != gate:
                continue
            for dh in BUDGETS:
                p = pullout_for_budget(
                    w, r["dive"], r["exit_mach"], r["exit_alt_m"],
                    r["exit_mass"], r["exit_thrust_n"], r["exit_drag_n"], dh)
                p["gate"] = gate
                p["climb"] = 12.0
                p["exit_mach"] = r["exit_mach"]
                p["exit_alt_m"] = r["exit_alt_m"]
                p["traverse_before_pullout_g"] = r["traverse"]
                verdict = ("OK" if p["n_margin"] > 0
                           and p["accel_arc_g"] >= TARGET_G else
                           "g-SHORT" if p["n_margin"] <= 0 else "a-SHORT")
                p["verdict"] = verdict
                rows.append(p)
                print(f"{gate:5.2f} {r['dive']:5.1f} {dh:7.1f} "
                      f"{p['radius_m']:7.1f} {p['n_required']:6.2f} "
                      f"{p['n_available']:8.2f} {p['n_margin']:7.2f} "
                      f"{p['duration_s']:5.2f} {p['d_induced_n']:7.1f} "
                      f"{p['d_induced_g']:6.3f} {p['accel_model_g']:6.3f} "
                      f"{p['accel_arc_g']:6.3f} {verdict:>9}")
            print()

    (OUT / "v3d_pullout_flyability.json").write_text(
        json.dumps(dict(fine_gate_scan=fine, pullout=rows), indent=2))
    print(f"wrote {OUT / 'v3d_pullout_flyability.json'}")


if __name__ == "__main__":
    main()
