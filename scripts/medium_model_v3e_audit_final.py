"""ADVERSARIAL AUDIT part 7 -- the two loose ends.

1. Does gate 0.50 really "open at dive >= 22 deg"?  Scan dive 20-24 at
   1 deg, all climbs.

2. The one arc where the CL_max choice could matter: dive 30, dh 28 m
   (n_req ~17.6).  n_available with lift.py's alpha-limited CL_max, with
   the legacy cl_max*cos^2(sweep), and with lift.py + the vortex-lift
   term the module documents but leaves off.

Run:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
        scripts/medium_model_v3e_audit_final.py
"""
from __future__ import annotations

import json
import os
from math import cos, radians
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                          # noqa: E402
from douglas_dart.atmosphere import standard_atmosphere          # noqa: E402
from medium_model.constants import G0_M_PER_S2                   # noqa: E402
from medium_model.design import fly, load_frozen_design          # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile             # noqa: E402
from medium_model.lift import wing_clmax                         # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
HARD_FLOOR_M = 121.92


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    w = d.wing
    out = {}
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH

    print("=== 1. gate 0.50: which dive angle opens it? (floor 122 m) ===")
    print(f"{'dive':>5} " + " ".join(f"{'c%d' % c:>7}" for c in
                                     (6, 8, 10, 12, 14, 16)) + "   best")
    rows = []
    try:
        rs.RAMJET_MIN_LIGHTOFF_MACH = 0.50
        for dv in (18.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0):
            cells, best = [], -9.0
            for c in (6.0, 8.0, 10.0, 12.0, 14.0, 16.0):
                cd = ClimbDiveProfile(initial_climb_angle_deg=c,
                                      dive_angle_deg=dv, floor_altitude_m=122.0)
                r = fly(d, drag_model="buildup", climb_dive=cd)
                peak = max(s.mach for s in r.states)
                fuel = max(s.fuel_burned_kg for s in r.states)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                tv = r.min_traverse_accel_g
                best = max(best, tv if ok else -9.0)
                cells.append(f"{tv:6.3f}{'P' if ok else 'F'}")
                rows.append(dict(dive=dv, climb=c, traverse=tv, passes=bool(ok),
                                 peak=peak, fuel=fuel))
            print(f"{dv:5.1f} " + " ".join(cells)
                  + f"   {'OPEN' if best > -1 else 'shut'}", flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    out["gate050_dive_scan"] = rows

    print("\n=== 2. the tight arc (dive 30, dh 28 m): does CL_max flip it? ===")
    tight = []
    try:
        for gate in (0.45, 0.50):
            rs.RAMJET_MIN_LIGHTOFF_MACH = gate
            cd = ClimbDiveProfile(initial_climb_angle_deg=12.0,
                                  dive_angle_deg=30.0,
                                  floor_altitude_m=HARD_FLOOR_M + 28.0)
            r = fly(d, drag_model="buildup", climb_dive=cd)
            ex = None
            for s in r.states:
                if s.mode == "v3_dive":
                    ex = s
                elif ex is not None:
                    break
            gd = radians(30.0)
            R = 28.0 / (1.0 - cos(gd))
            n_req = 1.0 + ex.velocity_m_per_s ** 2 / (G0_M_PER_S2 * R)
            atm = standard_atmosphere(ex.altitude_m)
            q = 0.5 * atm.density_kg_per_m3 * ex.velocity_m_per_s ** 2
            area = w.span_m ** 2 / w.aspect_ratio
            wn = ex.mass_kg * G0_M_PER_S2
            cl_alpha_lim = wing_clmax(w.airfoil.cl_max, w.aspect_ratio,
                                      w.taper_ratio, w.sweep_deg, ex.mach)
            cl_vortex = wing_clmax(w.airfoil.cl_max, w.aspect_ratio,
                                   w.taper_ratio, w.sweep_deg, ex.mach,
                                   vortex_lift_kv=3.0)
            legacy = w.cl_max_effective
            row = dict(gate=gate, exit_mach=ex.mach, exit_v=ex.velocity_m_per_s,
                       radius_m=R, n_req=n_req,
                       clmax_alpha=cl_alpha_lim.clmax,
                       clmax_legacy=legacy, clmax_vortex=cl_vortex.clmax,
                       n_alpha=q * area * cl_alpha_lim.clmax / wn,
                       n_legacy=q * area * legacy / wn,
                       n_vortex=q * area * cl_vortex.clmax / wn)
            tight.append(row)
            print(f"  gate {gate:.2f}: exit M {ex.mach:.3f}, V "
                  f"{ex.velocity_m_per_s:.1f} m/s, R {R:.1f} m -> "
                  f"n_req {n_req:.2f}")
            print(f"      lift.py alpha-limited CLmax {row['clmax_alpha']:.4f}"
                  f" -> n_avail {row['n_alpha']:6.2f}  "
                  f"{'SHORT' if n_req > row['n_alpha'] else 'ok'}")
            print(f"      legacy cl_max*cos^2      {row['clmax_legacy']:.4f}"
                  f" -> n_avail {row['n_legacy']:6.2f}  "
                  f"{'SHORT' if n_req > row['n_legacy'] else 'ok'}")
            print(f"      lift.py + vortex Kv=3.0  {row['clmax_vortex']:.4f}"
                  f" -> n_avail {row['n_vortex']:6.2f}  "
                  f"{'SHORT' if n_req > row['n_vortex'] else 'ok'}",
                  flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    out["tight_arc"] = tight

    (OUT / "v3e_audit_final.json").write_text(json.dumps(out, indent=2))
    print(f"\nwrote {OUT / 'v3e_audit_final.json'}")


if __name__ == "__main__":
    main()
