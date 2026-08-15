"""Phase-boundary diagnostic for the V3a climb-dive sweep (agent-A).

For each (climb, dive, floor) prints where each phase starts/ends in Mach
and altitude, and the thrust/drag/weight breakdown at the worst powered
step -- which is what tells us WHY the minimum moves.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from medium_model.constants import CD0_FRONTAL
from medium_model.design import fly, load_frozen_design
from medium_model.flight_sim import V3_FLOOR_ALTITUDE_M, ClimbDiveProfile
from math import radians, sin

G = 9.80665
POWERED = {"v3_climb", "v3_dive", "drag_strip", "pulsejet", "ramjet"}


def main() -> None:
    design = load_frozen_design("docs/v3a_medium_model/design.json")
    assert abs(CD0_FRONTAL - 0.1) < 1e-12
    pts = json.loads(Path(sys.argv[1]).read_text())
    print("climb,dive,floor | climb_exit_M climb_exit_h | dive_exit_M dive_exit_h"
          " | strip_entry_M | worstP: M mode T D Wpath a_g | worstT: M mode T D a_g")
    for climb, dive, floor in pts:
        prof = ClimbDiveProfile(float(climb), float(dive),
                                max(float(floor), V3_FLOOR_ALTITUDE_M))
        r = fly(design, climb_dive=prof, drag_model="buildup")
        st = r.states
        def last(mode):
            xs = [s for s in st if s.mode == mode]
            return xs[-1] if xs else None
        def first(mode):
            for s in st:
                if s.mode == mode:
                    return s
            return None
        ce, de, se = last("v3_climb"), last("v3_dive"), first("drag_strip")
        pw = min((s for s in st if s.mode in POWERED and s.thrust_n > 0),
                 key=lambda s: s.acceleration_m_per_s2)
        tv = min((s for s in st if s.mode in POWERED and s.thrust_n > 0
                  and s.mode != "v3_climb"),
                 key=lambda s: s.acceleration_m_per_s2)
        gam = {"v3_climb": radians(climb), "v3_dive": -radians(dive)}
        wp = lambda s: s.mass_kg * G * sin(gam.get(s.mode, radians(1.0)))
        print(f"{climb:g},{dive:g},{floor:g} | "
              f"{ce.mach:.3f} {ce.altitude_m:6.1f} | "
              f"{(de.mach if de else float('nan')):.3f} "
              f"{(de.altitude_m if de else float('nan')):6.1f} | "
              f"{(se.mach if se else float('nan')):.3f} | "
              f"{pw.mach:.3f} {pw.mode:10s} T={pw.thrust_n:6.1f} D={pw.drag_n:6.1f} "
              f"W={wp(pw):6.1f} a={pw.acceleration_m_per_s2/G:+.4f} | "
              f"{tv.mach:.3f} {tv.mode:10s} T={tv.thrust_n:6.1f} D={tv.drag_n:6.1f} "
              f"a={tv.acceleration_m_per_s2/G:+.4f}")


if __name__ == "__main__":
    main()
