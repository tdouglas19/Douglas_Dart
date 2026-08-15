"""Trajectory sweep for V3a (agent-A, unique file). Closed-form propulsion,
buildup drag. Reads a JSON list of [climb_deg, dive_deg, floor_m] triples
from argv[1] (or a default coarse grid) and prints one CSV row per flight.

Run as:  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
             scripts/traj_sweep_v3a_agentA.py points.json out.csv
"""
from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

from medium_model.constants import CD0_FRONTAL
from medium_model.design import fly, load_frozen_design
from medium_model.flight_sim import (V3_FLOOR_ALTITUDE_M, ClimbDiveProfile,
                                     derive_top_altitude)
from medium_model.mission import MAX_WET_MASS_KG

DESIGN_PATH = "docs/v3a_medium_model/design.json"
G = 9.80665

POWERED_MODES = {"v3_climb", "v3_dive", "drag_strip", "pulsejet", "ramjet"}


def phase_of_min(states, exclude_climb: bool):
    """Recover which flight phase holds the worst powered acceleration."""
    best = None
    for s in states:
        if s.mode not in POWERED_MODES:
            continue
        if s.thrust_n <= 0.0:
            continue
        if exclude_climb and s.mode == "v3_climb":
            continue
        g = s.acceleration_m_per_s2 / G
        if best is None or g < best[0]:
            best = (g, s.mode, s.mach, s.altitude_m)
    return best if best else (float("nan"), "none", float("nan"), float("nan"))


def main() -> None:
    design = load_frozen_design(DESIGN_PATH)
    assert abs(CD0_FRONTAL - 0.1) < 1e-12, f"CD0 not 0.1: {CD0_FRONTAL}"

    if len(sys.argv) > 1 and sys.argv[1] not in ("-", "default"):
        pts = json.loads(Path(sys.argv[1]).read_text())
    else:
        pts = [list(p) for p in itertools.product(
            [3, 6, 9, 12, 15, 18], [5, 10, 15, 20], [122, 200, 300, 400])]

    rows = []
    hdr = ("climb_deg,dive_deg,floor_m,top_m,traverse_g,traverse_mach,"
           "traverse_phase,powered_g,powered_mach,powered_phase,peak_mach,"
           "cutoff,margin,margin_mach,rule,fuel_kg,cutoff_time_s,stalled,"
           "safe_landing")
    print(hdr)
    for climb, dive, floor in pts:
        prof = ClimbDiveProfile(
            initial_climb_angle_deg=float(climb),
            dive_angle_deg=float(dive),
            floor_altitude_m=max(float(floor), V3_FLOOR_ALTITUDE_M),
        )
        top = derive_top_altitude(design.geometry, design.wing,
                                  MAX_WET_MASS_KG, prof)
        r = fly(design, climb_dive=prof, drag_model="buildup")
        st = r.states
        peak_mach = max(s.mach for s in st)
        fuel = max(s.fuel_burned_kg for s in st)
        # time at which cutoff mach was first reached (powered phase length)
        t_cut = next((s.time_s for s in st if s.mach >= 1.1), float("nan"))
        tg, tmode, tmach, _ = phase_of_min(st, exclude_climb=True)
        pg, pmode, pmach, _ = phase_of_min(st, exclude_climb=False)
        row = (f"{climb:g},{dive:g},{floor:g},"
               f"{(r.climb_dive_top_altitude_m or top):.1f},"
               f"{r.min_traverse_accel_g:.4f},{r.min_traverse_accel_mach:.3f},"
               f"{tmode},"
               f"{r.min_powered_accel_g:.4f},{r.min_accel_mach:.3f},{pmode},"
               f"{peak_mach:.4f},{int(r.motor_cutoff_reached)},"
               f"{r.min_powered_thrust_margin:.3f},{r.min_margin_mach:.3f},"
               f"{int(r.rule_violated)},{fuel:.4f},{t_cut:.1f},"
               f"{int(r.stalled)},{int(r.safe_landing)}")
        print(row)
        rows.append(row)

    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(hdr + "\n" + "\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
