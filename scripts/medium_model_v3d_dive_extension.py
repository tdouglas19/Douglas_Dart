"""Does a STEEPER dive buy a later ramjet lightoff, and is that dive flyable?

The v3c screen stopped at dive 20 deg and found a hard wall at gate M 0.45.
This extends the dive to 30 deg and asks three separate questions:

  1. does the gate ceiling move past 0.45 with more dive?
  2. does the flight-path-angle rule (gamma >= 0 from M 0.80 to cutoff)
     survive?  flight_sim tracks it as FlightResult.rule_violated /
     .rule_violation_mach (set in the powered loop, V3_RULE_MACH_LO..HI).
  3. what does the PULL-OUT at the floor cost?  flight_sim's phase machine
     switches gamma from -dive to +climb_angle in ONE timestep -- zero
     radius, zero altitude, infinite g.  Nothing in the model charges for
     it.  This script computes what a real pull-out would demand and what
     medium_model/lift.py says the wing can deliver.

Closed-form propulsion throughout (FP runs ~22% lower on traverse); this
ranks, it does not certify.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3d_dive_extension.py
"""
from __future__ import annotations

import json
import os
from math import asin, cos, degrees, pi, radians
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                      # noqa: E402
from medium_model.constants import AIRFOILS, G0_M_PER_S2     # noqa: E402
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.lift import (oswald_efficiency,            # noqa: E402
                               wing_clmax)
from douglas_dart.atmosphere import standard_atmosphere      # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)

TARGET_G = 0.26
FLOOR_M = 121.92                 # V3_FLOOR_ALTITUDE_M -- the legal minimum
CLIMBS = (12.0, 14.0, 16.0)
DIVES = (20.0, 22.0, 25.0, 28.0, 30.0)
GATES = (0.45, 0.50, 0.55)
FLOORS = (121.92, 150.0, 200.0)


# ---------------------------------------------------------------------------
# trajectory probes
# ---------------------------------------------------------------------------

def dive_exit_state(result):
    """Last state flown in mode 'v3_dive' -- the bottom of the dive, i.e.
    the instant the model performs its free pull-out."""
    last = None
    for s in result.states:
        if s.mode == "v3_dive":
            last = s
        elif last is not None and s.mode != "v3_dive":
            break
    return last


def gamma_trace_check(result, lo=0.80, hi=1.10):
    """Independent re-derivation of the flight-path-angle rule straight
    off the state trace, so we are not merely trusting the flag: h is the
    integrated altitude, so dh/dt < 0 anywhere in the Mach window while
    the engine is running is a violation regardless of what the phase
    machine intended."""
    POWERED = ("v3_climb", "v3_dive", "drag_strip", "pulsejet", "ramjet")
    worst_gamma_deg = 90.0
    worst_mach = None
    prev = None
    for s in result.states:
        if s.mode not in POWERED:
            break                      # engine off -> rule no longer applies
        if prev is not None and lo <= s.mach <= hi:
            dh = s.altitude_m - prev.altitude_m
            dt = s.time_s - prev.time_s
            if dt > 0 and s.velocity_m_per_s > 1.0:
                sin_g = max(min((dh / dt) / s.velocity_m_per_s, 1.0), -1.0)
                g_deg = degrees(asin(sin_g))
                if g_deg < worst_gamma_deg:
                    worst_gamma_deg = g_deg
                    worst_mach = s.mach
        prev = s
    return worst_gamma_deg, worst_mach


def min_powered_altitude(result):
    lo = 1e9
    for s in result.states:
        if s.mode in ("v3_climb", "v3_dive", "drag_strip", "pulsejet",
                      "ramjet", "climb"):
            lo = min(lo, s.altitude_m)
        else:
            break
    return lo


# ---------------------------------------------------------------------------
# pull-out physics
# ---------------------------------------------------------------------------

WING = None          # filled in main()


def wing_area_m2(w):
    return w.span_m ** 2 / w.aspect_ratio


def available_load_factor(w, mach, altitude_m, mass_kg):
    """n_max = q S CL_max / W, with CL_max from lift.wing_clmax (which for
    this AR 1.86 planform is ALPHA-limited at 25 deg, not section stall)."""
    atm = standard_atmosphere(altitude_m)
    v = mach * atm.speed_of_sound_m_per_s
    q = 0.5 * atm.density_kg_per_m3 * v * v
    cl = wing_clmax(w.airfoil.cl_max, w.aspect_ratio, w.taper_ratio,
                    w.sweep_deg, mach)
    lift_max_n = q * wing_area_m2(w) * cl.clmax
    weight_n = mass_kg * G0_M_PER_S2
    return dict(n_max=lift_max_n / weight_n, clmax=cl.clmax,
                mechanism=cl.mechanism, trustworthy=cl.trustworthy,
                q_pa=q, v_m_per_s=v, lift_max_n=lift_max_n,
                weight_n=weight_n, area_m2=wing_area_m2(w))


def pullout(w, mach, altitude_m, mass_kg, dive_deg, n):
    """Constant-n circular pull-out from -dive to level.

        R  = V^2 / (g (n - cos gamma))          (worst case at gamma = 0)
        dh = R (1 - cos gamma)
        dt = R * gamma_rad / V

    Also charges the induced drag of holding n g, which the model never
    sees (it flies the dive at n = cos gamma)."""
    atm = standard_atmosphere(altitude_m)
    v = mach * atm.speed_of_sound_m_per_s
    q = 0.5 * atm.density_kg_per_m3 * v * v
    g_r = radians(dive_deg)
    denom = n - 1.0                       # arrest at the bottom, gamma -> 0
    if denom <= 1e-6:
        return None
    R = v * v / (G0_M_PER_S2 * denom)
    dh = R * (1.0 - cos(g_r))
    dt = R * g_r / v
    weight_n = mass_kg * G0_M_PER_S2
    e = oswald_efficiency(w.aspect_ratio, w.taper_ratio, w.sweep_deg, mach)
    def di(lift):
        return lift ** 2 / (q * pi * w.span_m ** 2 * e)
    extra_n = di(n * weight_n) - di(weight_n * cos(g_r))
    return dict(n=n, radius_m=R, altitude_lost_m=dh, duration_s=dt,
                induced_extra_n=extra_n,
                induced_extra_g=extra_n / weight_n, oswald_e=e)


# ---------------------------------------------------------------------------

def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    w = d.wing
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    print(f"V3a  body {d.geometry.diameter_m*1000:.1f} mm, wing span "
          f"{w.span_m*1000:.1f} mm AR {w.aspect_ratio:.3f} taper "
          f"{w.taper_ratio:.3f} sweep {w.sweep_deg:.2f} deg, "
          f"area {wing_area_m2(w)*1e4:.0f} cm^2")
    print(f"burn limit {d.burn_limit_kg:.4f} kg, loaded "
          f"{d.loaded_fuel_kg:.4f} kg, target traverse {TARGET_G} g\n")

    rows = []
    try:
        # ---- part 1+2: dive extension, gate ceiling, rule check ----
        print("=== PART 1/2: dive extension at gates 0.45/0.50/0.55 ===")
        hdr = (f"{'climb':>5} {'dive':>5} {'gate':>5} {'trav':>6} "
               f"{'@M':>5} {'pow':>7} {'margin':>7} {'peakM':>6} "
               f"{'cut':>4} {'fuel':>6} {'top_m':>7} {'PASS':>5} "
               f"{'rule':>5} {'minGam':>7}")
        print(hdr)
        for climb in CLIMBS:
            for dive in DIVES:
                cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                      dive_angle_deg=dive,
                                      floor_altitude_m=FLOOR_M)
                for gate in GATES:
                    rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                    r = fly(d, drag_model="buildup", climb_dive=cd)
                    peak = max(s.mach for s in r.states)
                    fuel = max(s.fuel_burned_kg for s in r.states)
                    ok = (r.motor_cutoff_reached and peak >= 1.0
                          and r.min_traverse_accel_g >= TARGET_G
                          and fuel < d.burn_limit_kg - 1e-6)
                    gmin, gmin_mach = gamma_trace_check(r)
                    ex = dive_exit_state(r)
                    row = dict(
                        climb=climb, dive=dive, gate=gate,
                        traverse=r.min_traverse_accel_g,
                        traverse_mach=r.min_traverse_accel_mach,
                        powered=r.min_powered_accel_g,
                        margin=r.min_powered_thrust_margin,
                        margin_mach=r.min_margin_mach,
                        peak=peak, cutoff=bool(r.motor_cutoff_reached),
                        fuel=fuel, passes=bool(ok),
                        top_m=r.climb_dive_top_altitude_m,
                        rule_violated=bool(r.rule_violated),
                        rule_violation_mach=r.rule_violation_mach,
                        traced_min_gamma_deg=gmin,
                        traced_min_gamma_mach=gmin_mach,
                        min_alt_m=min_powered_altitude(r),
                        exit_mach=ex.mach if ex else None,
                        exit_alt_m=ex.altitude_m if ex else None,
                        exit_v=ex.velocity_m_per_s if ex else None,
                        exit_mass=ex.mass_kg if ex else None,
                        exit_thrust_n=ex.thrust_n if ex else None,
                        exit_drag_n=ex.drag_n if ex else None,
                        floor_m=FLOOR_M)
                    rows.append(row)
                    print(f"{climb:5.1f} {dive:5.1f} {gate:5.2f} "
                          f"{row['traverse']:6.3f} {row['traverse_mach']:5.2f} "
                          f"{row['powered']:7.3f} {row['margin']:7.3f} "
                          f"{peak:6.3f} {str(row['cutoff'])[:4]:>4} "
                          f"{fuel:6.3f} {row['top_m']:7.1f} "
                          f"{'PASS' if ok else 'fail':>5} "
                          f"{'VIOL' if row['rule_violated'] else 'ok':>5} "
                          f"{gmin:7.2f}", flush=True)

        # ---- part 4: floor sensitivity at the best passing dive ----
        passing = [q for q in rows if q["passes"]]
        best = max(passing, key=lambda q: (q["gate"], q["traverse"])) \
            if passing else None
        floor_rows = []
        if best:
            print(f"\n=== PART 4: floor sensitivity at climb "
                  f"{best['climb']:.0f} / dive {best['dive']:.0f} / gate "
                  f"{best['gate']:.2f} ===")
            print(f"{'floor':>6} {'trav':>6} {'@M':>5} {'pow':>7} "
                  f"{'margin':>7} {'peakM':>6} {'fuel':>6} {'top_m':>7} "
                  f"{'exitM':>6} {'PASS':>5}")
            for floor in FLOORS:
                cd = ClimbDiveProfile(
                    initial_climb_angle_deg=best["climb"],
                    dive_angle_deg=best["dive"], floor_altitude_m=floor)
                rs.RAMJET_MIN_LIGHTOFF_MACH = best["gate"]
                r = fly(d, drag_model="buildup", climb_dive=cd)
                peak = max(s.mach for s in r.states)
                fuel = max(s.fuel_burned_kg for s in r.states)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                ex = dive_exit_state(r)
                fr = dict(floor=floor, traverse=r.min_traverse_accel_g,
                          traverse_mach=r.min_traverse_accel_mach,
                          powered=r.min_powered_accel_g,
                          margin=r.min_powered_thrust_margin,
                          peak=peak, fuel=fuel, passes=bool(ok),
                          top_m=r.climb_dive_top_altitude_m,
                          rule_violated=bool(r.rule_violated),
                          exit_mach=ex.mach if ex else None,
                          exit_alt_m=ex.altitude_m if ex else None,
                          exit_mass=ex.mass_kg if ex else None,
                          climb=best["climb"], dive=best["dive"],
                          gate=best["gate"])
                floor_rows.append(fr)
                print(f"{floor:6.1f} {fr['traverse']:6.3f} "
                      f"{fr['traverse_mach']:5.2f} {fr['powered']:7.3f} "
                      f"{fr['margin']:7.3f} {peak:6.3f} {fuel:6.3f} "
                      f"{fr['top_m']:7.1f} "
                      f"{(fr['exit_mach'] or 0):6.3f} "
                      f"{'PASS' if ok else 'fail':>5}", flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original

    # ---- part 3: pull-out g demanded vs available ----
    print("\n=== PART 3: PULL-OUT at the floor ===")
    pull_rows = []
    seen = set()
    for q in rows:
        if q["exit_mach"] is None:
            continue
        key = (q["climb"], q["dive"], q["gate"])
        if key in seen:
            continue
        seen.add(key)
        av = available_load_factor(w, q["exit_mach"], q["exit_alt_m"],
                                   q["exit_mass"])
        entry = dict(key=key, dive=q["dive"], climb=q["climb"],
                     gate=q["gate"], exit_mach=q["exit_mach"],
                     exit_alt_m=q["exit_alt_m"], exit_mass=q["exit_mass"],
                     **av)
        entry["pullouts"] = [
            p for p in (pullout(w, q["exit_mach"], q["exit_alt_m"],
                                q["exit_mass"], q["dive"], n)
                        for n in (2.0, 3.0, 4.0, 6.0, av["n_max"] * 0.8,
                                  av["n_max"]))
            if p]
        pull_rows.append(entry)

    print(f"{'climb':>5} {'dive':>5} {'gate':>5} {'exitM':>6} {'alt':>6} "
          f"{'q kPa':>7} {'CLmax':>6} {'n_avail':>8} {'mech':>28}")
    for e in pull_rows:
        print(f"{e['climb']:5.1f} {e['dive']:5.1f} {e['gate']:5.2f} "
              f"{e['exit_mach']:6.3f} {e['exit_alt_m']:6.1f} "
              f"{e['q_pa']/1000:7.2f} {e['clmax']:6.3f} "
              f"{e['n_max']:8.2f} {e['mechanism']:>28}")

    print(f"\npull-out geometry (constant-n arc, -dive -> level):")
    print(f"{'dive':>5} {'exitM':>6} {'n':>7} {'R m':>8} {'dh m':>7} "
          f"{'dt s':>6} {'dD_i N':>8} {'dD_i g':>7} {'fits 122m?':>11}")
    for e in pull_rows:
        for p in e["pullouts"]:
            print(f"{e['dive']:5.1f} {e['exit_mach']:6.3f} {p['n']:7.2f} "
                  f"{p['radius_m']:8.1f} {p['altitude_lost_m']:7.1f} "
                  f"{p['duration_s']:6.2f} {p['induced_extra_n']:8.1f} "
                  f"{p['induced_extra_g']:7.3f} "
                  f"{'yes' if p['altitude_lost_m'] <= e['exit_alt_m'] else 'NO':>11}")

    (OUT / "v3d_dive_extension.json").write_text(json.dumps(
        dict(sweep=rows, floors=floor_rows,
             pullout=[{k: v for k, v in e.items() if k != "key"}
                      for e in pull_rows]), indent=2))
    print(f"\nwrote {OUT / 'v3d_dive_extension.json'}")


if __name__ == "__main__":
    main()
