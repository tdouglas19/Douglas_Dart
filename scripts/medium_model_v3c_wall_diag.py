"""WHERE does the M 0.45 -> 0.50 wall actually bite?

The screen JSON already says something the "vehicle never reaches the gate"
hypothesis does not predict: every gate-0.50 flight still reaches peak Mach
1.10 with motor_cutoff_reached True.  So the ramjet DOES light and the
vehicle DOES finish.  What fails is min_traverse_accel_g.

This dumps the trajectory around the min-traverse point for a ladder of
gates so the mechanism is visible: which flight MODE it happens in, at what
Mach and altitude, and what thrust / drag / gravity are doing there.
Closed-form propulsion, ~1-2 s per flight.
"""
from __future__ import annotations
import json, os
from math import degrees, sin, radians
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.ramjet_simple as rs                      # noqa: E402
from medium_model.design import fly, load_frozen_design       # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile          # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
FLOOR_M = 122.0
TARGET_G = 0.26
G0 = 9.80665

CASES = [(14.0, 20.0), (6.0, 20.0), (14.0, 9.890058542648028),
         (10.0, 20.0), (12.0, 20.0), (16.0, 20.0)]
GATES = (0.35, 0.40, 0.45, 0.50, 0.55)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    orig = rs.RAMJET_MIN_LIGHTOFF_MACH
    out = []
    try:
        for climb, dive in CASES:
            cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                  dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
            print(f"\n=== climb {climb:.1f} / dive {dive:.2f} deg "
                  f"(g*sin(dive) = {sin(radians(dive)):.3f} g) ===")
            print(f"{'gate':>5} {'top_m':>7} {'trav_g':>7} {'@M':>6} "
                  f"{'mode@min':>10} {'alt@min':>8} {'T@min':>7} {'D@min':>7} "
                  f"{'M@floor':>8} {'M@dive_end':>10} {'peakM':>6} {'pass':>5}")
            for gate in GATES:
                rs.RAMJET_MIN_LIGHTOFF_MACH = gate
                r = fly(d, drag_model="buildup", climb_dive=cd)
                st = r.states
                powered = [s for s in st if s.mode in
                           ("v3_climb", "v3_dive", "drag_strip",
                            "pulsejet", "ramjet")]
                trav = [s for s in powered if s.mode != "v3_climb"]
                # recover the min-traverse step exactly the way flight_sim does
                smin = min(trav, key=lambda s: s.acceleration_m_per_s2 / G0)
                dive_states = [s for s in st if s.mode == "v3_dive"]
                strip = [s for s in st if s.mode == "drag_strip"]
                m_dive_end = dive_states[-1].mach if dive_states else float("nan")
                h_dive_end = dive_states[-1].altitude_m if dive_states else float("nan")
                m_strip0 = strip[0].mach if strip else float("nan")
                peak = max(s.mach for s in st)
                fuel = max(s.fuel_burned_kg for s in st)
                ok = (r.motor_cutoff_reached and peak >= 1.0
                      and r.min_traverse_accel_g >= TARGET_G
                      and fuel < d.burn_limit_kg - 1e-6)
                print(f"{gate:5.2f} {r.climb_dive_top_altitude_m:7.0f} "
                      f"{r.min_traverse_accel_g:7.3f} "
                      f"{r.min_traverse_accel_mach:6.3f} {smin.mode:>10} "
                      f"{smin.altitude_m:8.0f} {smin.thrust_n:7.1f} "
                      f"{smin.drag_n:7.1f} {h_dive_end:8.0f} "
                      f"{m_dive_end:10.3f} {peak:6.3f} "
                      f"{'YES' if ok else 'no':>5}", flush=True)
                out.append(dict(
                    climb=climb, dive=dive, gate=gate,
                    top_m=r.climb_dive_top_altitude_m,
                    traverse=r.min_traverse_accel_g,
                    traverse_mach=r.min_traverse_accel_mach,
                    min_mode=smin.mode, min_alt=smin.altitude_m,
                    min_thrust=smin.thrust_n, min_drag=smin.drag_n,
                    dive_end_mach=m_dive_end, dive_end_alt=h_dive_end,
                    strip_start_mach=m_strip0, peak=peak, fuel=fuel,
                    passes=bool(ok)))

        # detailed step dump for the pivotal pair
        for climb, dive, gate in ((14.0, 20.0, 0.45), (14.0, 20.0, 0.50)):
            rs.RAMJET_MIN_LIGHTOFF_MACH = gate
            cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                                  dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
            r = fly(d, drag_model="buildup", climb_dive=cd)
            print(f"\n--- trajectory, climb {climb} dive {dive} gate {gate} "
                  f"(every 0.02 Mach, powered only) ---")
            print(f"{'t':>6} {'mode':>10} {'M':>6} {'alt':>7} {'T':>7} "
                  f"{'D':>7} {'T-D':>8} {'a/g':>7}")
            last = -1.0
            for s in r.states:
                if s.mode in ("glide", "loop", "flare", "decel", "level"):
                    break
                if s.mach - last < 0.02:
                    continue
                last = s.mach
                print(f"{s.time_s:6.1f} {s.mode:>10} {s.mach:6.3f} "
                      f"{s.altitude_m:7.0f} {s.thrust_n:7.1f} {s.drag_n:7.1f} "
                      f"{s.thrust_n - s.drag_n:8.1f} "
                      f"{s.acceleration_m_per_s2 / G0:7.3f}")
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = orig
    (OUT / "v3c_wall_diag.json").write_text(json.dumps(out, indent=2))
    print(f"\nwrote {OUT / 'v3c_wall_diag.json'}")


if __name__ == "__main__":
    main()
