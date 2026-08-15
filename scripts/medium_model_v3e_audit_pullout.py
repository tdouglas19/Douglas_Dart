"""ADVERSARIAL AUDIT part 4 -- the pull-out, re-derived independently.

Three things the dive report asserts, checked a different way each:

A. CL_max / n_available.  Recomputed BOTH ways -- lift.py's alpha-limited
   value and drag.py's legacy cl_max * cos^2(sweep) -- to see whether the
   choice flips any verdict.  Also both Oswald factors (lift.py's vs the
   one the flight integrator actually uses).

B. The arc, re-derived by NUMERICAL INTEGRATION rather than by the
   closed-form R = dh/(1 - cos gd), n = 1 + V^2/(gR).  A constant-load-
   factor pull-up is marched with real thrust and real drag (including
   induced drag at n*W), and the altitude it eats, the time it takes and
   the speed change are measured.  If the closed form is right the two
   agree.

C. The gamma >= 0 rule: is it violable at all, or shut by construction?

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_pullout.py
"""
from __future__ import annotations

import json
import os
from math import asin, cos, degrees, pi, radians, sin
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                          # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere          # noqa: E402
from medium_model.constants import (G0_M_PER_S2,                 # noqa: E402
                                    NOSE_TAIL_LENGTH_DIAMETERS,
                                    TAIL_LENGTH_DIAMETERS)
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.drag_buildup import total_drag_buildup         # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile             # noqa: E402
from medium_model.lift import (finite_wing_lift_curve_slope,     # noqa: E402
                               oswald_efficiency, wing_clmax)
from medium_model.pulsejet_simple import pulsejet_thrust         # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
HARD_FLOOR_M = 121.92
CLIMB = 12.0


def make_cache(d, oswald_e):
    g, w = d.geometry, d.wing
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
        "oswald_e": oswald_e,
        "lip_area_m2": pi * g.throat_diameter_m ** 2 / 4.0,
    }


def drag_at(d, cache, v, rho, temp, mach, lift_n, mdot):
    b = total_drag_buildup(
        diameter_m=cache["diameter_m"], body_length_m=cache["body_length_m"],
        duct_exit_diameter_m=cache["duct_exit_diameter_m"],
        tail_length_m=cache["tail_length_m"],
        wing_reference_area_m2=cache["wing_area_m2"],
        wing_thickness_ratio=cache["wing_thickness_ratio"],
        wing_sweep_deg=cache["wing_sweep_deg"],
        velocity_m_per_s=v, density_kg_per_m3=rho, temperature_k=temp,
        mach=mach, required_lift_n=lift_n,
        oswald_efficiency=cache["oswald_e"],
        wing_aspect_ratio=cache["aspect_ratio"], engine_on=True,
        captured_mass_flow_kg_per_s=mdot, lip_area_m2=cache["lip_area_m2"],
        cowl_suction_recovery=None)
    return b.total_n


def march_arc(d, cache, gate, exit_state, dive_deg, n_load, dt=0.005,
              max_t=20.0):
    """Constant-load-factor pull-up marched forward from the dive exit.

    gamma_dot = g (n - cos gamma) / V   (pull-UP: n is the load factor,
    the cos term is the weight component normal to the path).  Thrust and
    drag are the real closed forms; lift is n*m*g so induced drag is
    charged at the g being held.  Returns altitude eaten, duration, exit
    speed and the minimum along-path acceleration during the arc.
    """
    g = d.geometry
    gamma = -radians(dive_deg)
    v = exit_state.velocity_m_per_s
    h = exit_state.altitude_m
    m = exit_state.mass_kg
    t = 0.0
    h0 = h
    min_a_g = float("inf")
    h_min = h
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
    try:
        while gamma < 0.0 and t < max_t and h > 0.0:
            atm = standard_atmosphere(max(h, 0.0))
            mach = v / atm.speed_of_sound_m_per_s
            rj = rs.ramjet_thrust(g.diameter_m, g.throat_diameter_m, mach,
                                  h, g.fuel, atmosphere=atm)
            pj = pulsejet_thrust(g.diameter_m, g.chamber_length_m,
                                 g.throat_diameter_m, g.throat_length_m,
                                 mach, h, g.fuel, atmosphere=atm)
            thrust = pj.average_thrust_n + rj.net_thrust_n
            mdot = (rj.air_mass_flow_kg_per_s if rj.net_thrust_n > 0.0
                    else 0.90 * atm.density_kg_per_m3 * v
                    * cache["lip_area_m2"])
            lift_n = n_load * m * G0_M_PER_S2
            drag = drag_at(d, cache, v, atm.density_kg_per_m3,
                           atm.temperature_k, mach, lift_n, mdot)
            a = (thrust - drag) / m - G0_M_PER_S2 * sin(gamma)
            min_a_g = min(min_a_g, a / G0_M_PER_S2)
            fuel_rate = (pj.fuel_mass_flow_kg_per_s
                         + rj.fuel_mass_flow_kg_per_s)
            gamma += (G0_M_PER_S2 * (n_load - cos(gamma)) / max(v, 1.0)) * dt
            v = max(v + a * dt, 1.0)
            h = h + v * sin(gamma) * dt
            h_min = min(h_min, h)
            m = max(m - fuel_rate * dt, 1.0)
            t += dt
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    return dict(n=n_load, dh_m=h0 - h_min, duration_s=t, v_exit=v,
                mach_exit=v / standard_atmosphere(max(h, 0.0)
                                                  ).speed_of_sound_m_per_s,
                min_arc_accel_g=min_a_g, h_end=h)


def dive_exit(result):
    last = None
    for s in result.states:
        if s.mode == "v3_dive":
            last = s
        elif last is not None:
            break
    return last


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    w = d.wing
    out = {}

    # ---- A. CL_max both ways --------------------------------------------
    print("=== A. CL_max and n_available: lift.py alpha-limit vs legacy ===")
    legacy_clmax = w.cl_max_effective
    print(f"wing: span {w.span_m*1000:.1f} mm, AR {w.aspect_ratio:.3f}, "
          f"taper {w.taper_ratio:.3f}, LE sweep {w.sweep_deg:.2f} deg, "
          f"S {w.span_m**2/w.aspect_ratio*1e4:.0f} cm^2")
    print(f"airfoil 2-D cl_max {w.airfoil.cl_max}; "
          f"legacy cl_max*cos^2(sweep) = {legacy_clmax:.4f}")
    print(f"\n{'M':>5} {'lift.py':>8} {'mech':>26} {'trust':>6} "
          f"{'legacy':>7} {'ratio':>6}")
    clrows = []
    for m in (0.0, 0.30, 0.45, 0.50, 0.54, 0.60, 0.80):
        cm = wing_clmax(w.airfoil.cl_max, w.aspect_ratio, w.taper_ratio,
                        w.sweep_deg, m)
        clrows.append(dict(mach=m, lift_clmax=cm.clmax,
                           mechanism=cm.mechanism,
                           trustworthy=cm.trustworthy,
                           legacy_clmax=legacy_clmax))
        print(f"{m:5.2f} {cm.clmax:8.4f} {cm.mechanism:>26} "
              f"{str(cm.trustworthy):>6} {legacy_clmax:7.4f} "
              f"{legacy_clmax/cm.clmax:6.3f}")
    out["clmax"] = clrows

    e_lift = oswald_efficiency(w.aspect_ratio, w.taper_ratio, w.sweep_deg,
                               0.54)
    print(f"\nOswald e at M 0.54: lift.py {e_lift:.4f}  vs  "
          f"WingConcept.oswald_e {w.oswald_e:.4f} "
          f"(the one the integrator uses)  ratio {w.oswald_e/e_lift:.3f}")
    out["oswald"] = dict(lift_py=e_lift, integrator=w.oswald_e)

    # ---- B. arc, closed form vs marched ---------------------------------
    print("\n=== B. pull-out arc: closed form vs marched integration ===")
    cache_int = make_cache(d, w.oswald_e)      # integrator's own e
    rows = []
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    print(f"{'gate':>5} {'dive':>5} {'dh_cmd':>7} {'exitM':>6} {'exitV':>6} "
          f"{'exitH':>7} {'n_cf':>6} {'n_lift':>7} {'n_lgcy':>7} "
          f"{'dh_marched':>10} {'dt_cf':>6} {'dt_mar':>7} {'aArc_cf':>8} "
          f"{'aArc_mar':>9} {'verdict':>10}")
    try:
        for gate in (0.45, 0.50):
            for dive in (25.0, 30.0):
                for dh in (78.0, 128.0, 178.0):
                    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                    cd = ClimbDiveProfile(initial_climb_angle_deg=CLIMB,
                                          dive_angle_deg=dive,
                                          floor_altitude_m=HARD_FLOOR_M + dh)
                    r = fly(d, drag_model="buildup", climb_dive=cd)
                    rs.RAMJET_MIN_LIGHTOFF_MACH = orig
                    ex = dive_exit(r)
                    if ex is None:
                        continue
                    gd = radians(dive)
                    R = dh / (1.0 - cos(gd))
                    n_cf = 1.0 + ex.velocity_m_per_s ** 2 / (G0_M_PER_S2 * R)
                    dt_cf = R * gd / ex.velocity_m_per_s
                    atm = standard_atmosphere(ex.altitude_m)
                    q = 0.5 * atm.density_kg_per_m3 * ex.velocity_m_per_s ** 2
                    area = w.span_m ** 2 / w.aspect_ratio
                    wn = ex.mass_kg * G0_M_PER_S2
                    cm = wing_clmax(w.airfoil.cl_max, w.aspect_ratio,
                                    w.taper_ratio, w.sweep_deg, ex.mach)
                    n_lift = q * area * cm.clmax / wn
                    n_lgcy = q * area * legacy_clmax / wn
                    mar = march_arc(d, cache_int, gate, ex, dive, n_cf)
                    # closed-form arc accel, integrator's own e, for parity
                    grav_arc = (1.0 - cos(gd)) / gd
                    d_ind = ((n_cf * wn) ** 2 - (wn * cos(gd)) ** 2) / (
                        q * pi * w.span_m ** 2 * w.oswald_e)
                    a_cf = (ex.thrust_n - ex.drag_n - d_ind) / wn + grav_arc
                    verdict = ("g-SHORT-both" if n_cf > n_lgcy else
                               "g-SHORT-lift" if n_cf > n_lift else "g-OK")
                    row = dict(gate=gate, dive=dive, dh_cmd=dh,
                               exit_mach=ex.mach, exit_v=ex.velocity_m_per_s,
                               exit_alt=ex.altitude_m, n_cf=n_cf,
                               n_avail_lift=n_lift, n_avail_legacy=n_lgcy,
                               dt_cf=dt_cf, a_arc_cf=a_cf,
                               clmax_lift=cm.clmax, verdict=verdict,
                               sim_traverse=r.min_traverse_accel_g, **{
                                   f"march_{k}": v for k, v in mar.items()})
                    rows.append(row)
                    print(f"{gate:5.2f} {dive:5.1f} {dh:7.0f} {ex.mach:6.3f} "
                          f"{ex.velocity_m_per_s:6.1f} {ex.altitude_m:7.1f} "
                          f"{n_cf:6.2f} {n_lift:7.2f} {n_lgcy:7.2f} "
                          f"{mar['dh_m']:10.1f} {dt_cf:6.2f} "
                          f"{mar['duration_s']:7.2f} {a_cf:8.3f} "
                          f"{mar['min_arc_accel_g']:9.3f} {verdict:>12}",
                          flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    out["arc"] = rows

    # ---- C. the gamma >= 0 rule ------------------------------------------
    print("\n=== C. can the gamma>=0 rule (M 0.80-1.10) ever be violated? ===")
    print("dive exit Mach across the whole box (rule window opens at 0.80):")
    rows_c = []
    try:
        for gate in (0.45, 0.50, 0.55):
            for dive in (14.0, 20.0, 30.0):
                rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                cd = ClimbDiveProfile(initial_climb_angle_deg=CLIMB,
                                      dive_angle_deg=dive,
                                      floor_altitude_m=122.0)
                r = fly(d, drag_model="buildup", climb_dive=cd)
                ex = dive_exit(r)
                # trace gamma from integrated altitude in the rule window
                worst = None
                for a, b in zip(r.states, r.states[1:]):
                    if b.mode not in ("pulsejet", "ramjet", "v3_climb",
                                      "v3_dive", "drag_strip"):
                        continue
                    if not (0.80 <= a.mach <= 1.10):
                        continue
                    dh_ = b.altitude_m - a.altitude_m
                    ds = max(a.velocity_m_per_s * (b.time_s - a.time_s), 1e-9)
                    gg = degrees(asin(max(min(dh_ / ds, 1.0), -1.0)))
                    worst = gg if worst is None else min(worst, gg)
                rows_c.append(dict(gate=gate, dive=dive,
                                   dive_exit_mach=(ex.mach if ex else None),
                                   rule_violated=bool(r.rule_violated),
                                   traced_min_gamma=worst))
                print(f"  gate {gate:.2f} dive {dive:4.1f}: dive exits at "
                      f"M {(ex.mach if ex else float('nan')):.3f}, flag "
                      f"{str(r.rule_violated):>5}, traced min gamma "
                      f"{(worst if worst is not None else float('nan')):.3f} "
                      f"deg", flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    out["rule"] = rows_c

    (OUT / "v3e_audit_pullout.json").write_text(json.dumps(out, indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_pullout.json'}")


if __name__ == "__main__":
    main()
