"""Is it the THRUST, or is it the AIRFRAME? Isolate the two.

The 235 mm vehicle bundles three changes at once: +20.6% pulsejet thrust,
+20.6% frontal area (and CD0 is FRONTAL-AREA referenced), and a 39 mm
shorter but fatter body.  The control run showed the net gain over V3a is
only +0.01..+0.02 in gate Mach at matched dive angle.  This asks why.

Experiment: give the UNCHANGED V3a airframe a pulsejet scaled by an
arbitrary factor -- thrust and fuel flow together, so specific impulse is
preserved -- by wrapping flight_sim's module-level ``pulsejet_thrust``.
No drag penalty, no mass penalty, no length change: pure thrust.

x1.2064 is the exact factor the 235 mm duct delivers in the closed-form
model, so "V3a at x1.2064" vs "the real 235 mm vehicle" is precisely the
drag/airframe cost of buying that thrust with diameter.  x1.5 and x2.0
say what thrust is worth at all, i.e. whether the gate is thrust-limited.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

import medium_model.flight_sim as fs                           # noqa: E402
import medium_model.ramjet_simple as rs                        # noqa: E402
from medium_model.design import fly, load_frozen_design        # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile           # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0
CLIMBS = (10.0, 12.0, 14.0)
DIVES = (18.0, 20.0, 26.0)
GATES = tuple(round(0.44 + 0.01 * i, 2) for i in range(17))     # 0.44..0.60

_REAL = fs.pulsejet_thrust


def scaled_pulsejet(factor: float):
    def wrapper(*a, **k):
        r = _REAL(*a, **k)
        return r._replace(
            average_thrust_n=factor * r.average_thrust_n,
            peak_thrust_n=factor * r.peak_thrust_n,
            fuel_mass_flow_kg_per_s=factor * r.fuel_mass_flow_kg_per_s,
            air_mass_flow_kg_per_s=factor * r.air_mass_flow_kg_per_s)
    return wrapper


def ceiling(d, tag: str, factor: float) -> list[dict]:
    fs.pulsejet_thrust = scaled_pulsejet(factor) if factor != 1.0 else _REAL
    rows = []
    try:
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
                    rows.append(dict(case=tag, factor=factor, climb=climb,
                                     dive=dive, gate=gate,
                                     traverse=r.min_traverse_accel_g,
                                     peak=peak, fuel=fuel, passes=bool(ok)))
    finally:
        fs.pulsejet_thrust = _REAL
    return rows


def drag_decomposition() -> None:
    """Where does the 235 mm body actually lose, and where does it win?"""
    from douglas_dart.atmosphere import standard_atmosphere
    from medium_model.drag_buildup import total_drag_buildup
    from medium_model import constants as C

    a = load_frozen_design("docs/v3a_medium_model/design.json")
    b = load_frozen_design("docs/v3c_235mm/design.json")
    print("\ndrag at a matched condition (M 0.50, 300 m, 22.68 kg, level)")
    atm = standard_atmosphere(300.0)
    v = 0.50 * atm.speed_of_sound_m_per_s
    from medium_model.pulsejet_simple import pulsejet_thrust as _pj
    res = {}
    for tag, d in (("V3a", a), ("235", b)):
        g, w = d.geometry, d.wing
        body_len = (g.chamber_length_m + g.throat_length_m
                    + C.NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m)
        pj = _pj(g.diameter_m, g.chamber_length_m, g.throat_diameter_m,
                 g.throat_length_m, 0.50, 300.0, g.fuel, atmosphere=atm)
        bd = total_drag_buildup(
            diameter_m=g.diameter_m, body_length_m=body_len,
            duct_exit_diameter_m=g.throat_diameter_m,
            tail_length_m=C.TAIL_LENGTH_DIAMETERS * g.diameter_m,
            wing_reference_area_m2=w.span_m ** 2 / w.aspect_ratio,
            wing_thickness_ratio=w.airfoil.thickness_ratio,
            wing_sweep_deg=w.sweep_deg, velocity_m_per_s=v,
            density_kg_per_m3=atm.density_kg_per_m3,
            temperature_k=atm.temperature_k, mach=0.50,
            required_lift_n=22.68 * C.G0_M_PER_S2,
            oswald_efficiency=w.oswald_e, wing_aspect_ratio=w.aspect_ratio,
            engine_on=True,
            captured_mass_flow_kg_per_s=pj.air_mass_flow_kg_per_s,
            lip_area_m2=0.25 * math.pi * g.throat_diameter_m ** 2)
        res[tag] = dict(
            frontal_cm2=0.25 * math.pi * g.diameter_m ** 2 * 1e4,
            wetted_m2=math.pi * g.diameter_m * body_len,
            friction=bd.friction_n, form=bd.form_n, base=bd.base_n,
            wave=bd.wave_n, spillage=bd.spillage_n,
            wing_profile=bd.wing_profile_n, induced=bd.induced_n,
            total_drag=bd.total_n, pj_thrust=pj.average_thrust_n,
            net=pj.average_thrust_n - bd.total_n)
    print(f"  {'':>13} {'V3a':>10} {'235mm':>10} {'ratio':>7}")
    for k in res["V3a"]:
        x, y = res["V3a"][k], res["235"][k]
        r = f"{y/x:7.4f}" if abs(x) > 1e-12 else f"{'--':>7}"
        print(f"  {k:>13} {x:10.4f} {y:10.4f} {r}")


def main() -> None:
    original = rs.RAMJET_MIN_LIGHTOFF_MACH
    v3a = load_frozen_design("docs/v3a_medium_model/design.json")
    new = load_frozen_design("docs/v3c_235mm/design.json")
    cases = [("V3a airframe, stock pulsejet", v3a, 1.0),
             ("V3a airframe, pulsejet x1.2064 (free)", v3a, 1.2064),
             ("V3a airframe, pulsejet x1.5 (free)", v3a, 1.5),
             ("V3a airframe, pulsejet x2.0 (free)", v3a, 2.0),
             ("235 mm vehicle (thrust BOUGHT with diameter)", new, 1.0)]
    rows: list[dict] = []
    try:
        print(f"{'case':>44} | {'d18':>5} {'d20':>5} {'d26':>5} | best")
        for tag, d, f in cases:
            r = ceiling(d, tag, f)
            rows += r
            cells = []
            for dive in DIVES:
                g = [q["gate"] for q in r if q["dive"] == dive and q["passes"]]
                cells.append(f"{max(g):5.2f}" if g else f"{'--':>5}")
            allg = [q["gate"] for q in r if q["passes"]]
            print(f"{tag:>44} | " + " ".join(cells)
                  + f" | {max(allg) if allg else 'NONE'}", flush=True)
    finally:
        rs.RAMJET_MIN_LIGHTOFF_MACH = original
    (OUT / "v3c_thrust_isolation.json").write_text(json.dumps(rows, indent=2))
    drag_decomposition()
    print(f"\nwrote {OUT/'v3c_thrust_isolation.json'}")


if __name__ == "__main__":
    main()
