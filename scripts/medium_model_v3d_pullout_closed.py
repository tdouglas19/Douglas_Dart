"""Self-consistent pull-out: the dive must END high enough that the ARC
bottoms out at the 121.92 m hard floor, not start there.

flight_sim's floor is where the straight dive stops and gamma jumps to
level in one timestep.  A real vehicle needs an arc of altitude dh below
that point.  So the honest experiment is:

    floor_altitude_m = 121.92 + dh          <- re-fly, don't post-process
    R  = dh / (1 - cos gamma_d)
    n  = 1 + V_exit^2 / (g R)               <- worst case, bottom of arc
    effective traverse gate = min(sim traverse, along-path accel in the arc)

The arc costs twice: it gives back part of the gravity term (mean assist
is g(1-cos gd)/gd instead of g sin gd) and it pays induced drag at n g
instead of 1 g.  Both are charges the model never makes.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3d_pullout_closed.py
"""
from __future__ import annotations

import json
import os
from math import cos, degrees, pi, radians, sin
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                        # noqa: E402
from medium_model.constants import G0_M_PER_S2                 # noqa: E402
from medium_model.design import fly, load_frozen_design        # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile           # noqa: E402
from medium_model.lift import (finite_wing_lift_curve_slope,   # noqa: E402
                               oswald_efficiency, wing_clmax)
from douglas_dart.atmosphere import standard_atmosphere        # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
HARD_FLOOR_M = 121.92
TARGET_G = 0.26
CLIMB = 12.0
DIVES = (20.0, 22.0, 25.0, 28.0, 30.0)
GATES = (0.45, 0.50)
DHS = (28.0, 78.0, 128.0, 178.0, 278.0)


def dive_exit(result):
    last = None
    for s in result.states:
        if s.mode == "v3_dive":
            last = s
        elif last is not None:
            break
    return last


def arc(w, ex, dive_deg, dh_m):
    atm = standard_atmosphere(ex.altitude_m)
    v = ex.velocity_m_per_s
    q = 0.5 * atm.density_kg_per_m3 * v * v
    gd = radians(dive_deg)
    R = dh_m / (1.0 - cos(gd))
    n = 1.0 + v * v / (G0_M_PER_S2 * R)
    dt = R * gd / v
    weight_n = ex.mass_kg * G0_M_PER_S2
    e = oswald_efficiency(w.aspect_ratio, w.taper_ratio, w.sweep_deg, ex.mach)
    area = w.span_m ** 2 / w.aspect_ratio

    def di(lift_n):
        return lift_n ** 2 / (q * pi * w.span_m ** 2 * e)

    d_ind = di(n * weight_n) - di(weight_n * cos(gd))
    cl_req = n * weight_n / (q * area)
    cl_alpha = finite_wing_lift_curve_slope(w.aspect_ratio, w.sweep_deg,
                                            ex.mach,
                                            taper_ratio=w.taper_ratio)
    alpha_req_deg = degrees(cl_req / cl_alpha) if cl_alpha > 0 else float("inf")
    cm = wing_clmax(w.airfoil.cl_max, w.aspect_ratio, w.taper_ratio,
                    w.sweep_deg, ex.mach)
    n_avail = q * area * cm.clmax / weight_n
    grav_arc_g = (1.0 - cos(gd)) / gd
    accel_arc_g = ((ex.thrust_n - ex.drag_n - d_ind) / weight_n) + grav_arc_g
    return dict(dh_m=dh_m, radius_m=R, n_req=n, n_avail=n_avail,
                n_margin=n_avail - n, duration_s=dt, d_induced_n=d_ind,
                d_induced_g=d_ind / weight_n, cl_req=cl_req,
                clmax=cm.clmax, clmax_mechanism=cm.mechanism,
                clmax_trustworthy=cm.trustworthy,
                alpha_req_deg=alpha_req_deg,
                grav_arc_g=grav_arc_g, grav_dive_g=sin(gd),
                accel_arc_g=accel_arc_g, q_pa=q, oswald_e=e,
                v_m_per_s=v, mass_kg=ex.mass_kg)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    w = d.wing
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    rows = []
    print(f"V3a wing: span {w.span_m*1000:.1f} mm, AR {w.aspect_ratio:.3f}, "
          f"area {w.span_m**2/w.aspect_ratio*1e4:.0f} cm^2, "
          f"airfoil cl_max 2D {w.airfoil.cl_max}")
    print(f"hard floor {HARD_FLOOR_M} m; the dive now ends at "
          f"{HARD_FLOOR_M}+dh so the ARC bottoms at the floor")
    print(f"climb {CLIMB} deg, closed-form propulsion, drag build-up\n")
    print(f"{'gate':>5} {'dive':>5} {'dh_m':>5} {'floor':>6} {'sim_tv':>7} "
          f"{'peakM':>6} {'fuel':>6} {'exitM':>6} {'exitV':>6} {'R_m':>7} "
          f"{'n_req':>6} {'n_av':>6} {'a_req':>6} {'dt_s':>5} {'dDi_g':>6} "
          f"{'a_arc':>6} {'EFFtv':>6} {'VERDICT':>8}")
    try:
        for gate in GATES:
            rs.RAMJET_MIN_LIGHTOFF_MACH = gate
            for dive in DIVES:
                for dh in DHS:
                    floor = HARD_FLOOR_M + dh
                    cd = ClimbDiveProfile(initial_climb_angle_deg=CLIMB,
                                          dive_angle_deg=dive,
                                          floor_altitude_m=floor)
                    r = fly(d, drag_model="buildup", climb_dive=cd)
                    peak = max(s.mach for s in r.states)
                    fuel = max(s.fuel_burned_kg for s in r.states)
                    ex = dive_exit(r)
                    if ex is None:
                        continue
                    a = arc(w, ex, dive, dh)
                    eff_tv = min(r.min_traverse_accel_g, a["accel_arc_g"])
                    base_ok = (r.motor_cutoff_reached and peak >= 1.0
                               and fuel < d.burn_limit_kg - 1e-6
                               and not r.rule_violated)
                    if not base_ok:
                        verdict = "no-cutoff" if not r.motor_cutoff_reached \
                            else "base-fail"
                    elif a["n_margin"] <= 0:
                        verdict = "g-SHORT"
                    elif eff_tv < TARGET_G:
                        verdict = "a-SHORT"
                    else:
                        verdict = "PASS"
                    row = dict(gate=gate, dive=dive, climb=CLIMB, floor_m=floor,
                               sim_traverse=r.min_traverse_accel_g,
                               sim_traverse_mach=r.min_traverse_accel_mach,
                               powered=r.min_powered_accel_g,
                               margin=r.min_powered_thrust_margin,
                               peak=peak, fuel=fuel,
                               cutoff=bool(r.motor_cutoff_reached),
                               rule_violated=bool(r.rule_violated),
                               top_m=r.climb_dive_top_altitude_m,
                               exit_mach=ex.mach, exit_alt_m=ex.altitude_m,
                               exit_thrust_n=ex.thrust_n,
                               exit_drag_n=ex.drag_n,
                               effective_traverse_g=eff_tv,
                               verdict=verdict, **a)
                    rows.append(row)
                    print(f"{gate:5.2f} {dive:5.1f} {dh:5.0f} {floor:6.1f} "
                          f"{r.min_traverse_accel_g:7.3f} {peak:6.3f} "
                          f"{fuel:6.3f} {ex.mach:6.3f} "
                          f"{ex.velocity_m_per_s:6.1f} {a['radius_m']:7.1f} "
                          f"{a['n_req']:6.2f} {a['n_avail']:6.2f} "
                          f"{a['alpha_req_deg']:6.2f} {a['duration_s']:5.2f} "
                          f"{a['d_induced_g']:6.3f} {a['accel_arc_g']:6.3f} "
                          f"{eff_tv:6.3f} {verdict:>8}", flush=True)
                print()
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original

    (OUT / "v3d_pullout_closed.json").write_text(json.dumps(rows, indent=2))
    ok = [q for q in rows if q["verdict"] == "PASS"]
    print(f"{len(ok)} of {len(rows)} survive the CLOSED (pull-out-charged) "
          f"test")
    if ok:
        top = max(q["gate"] for q in ok)
        print(f"latest gate that survives with a flyable pull-out: M {top:.2f}")
        for q in sorted((z for z in ok if z["gate"] == top),
                        key=lambda z: -z["effective_traverse_g"])[:8]:
            print(f"  dive {q['dive']:4.1f}  dh {q['dh_m']:5.0f} m  "
                  f"n_req {q['n_req']:5.2f}/{q['n_avail']:5.2f}  "
                  f"eff traverse {q['effective_traverse_g']:.3f} g  "
                  f"fuel {q['fuel']:.3f} kg")
    print(f"\nwrote {OUT / 'v3d_pullout_closed.json'}")


if __name__ == "__main__":
    main()
