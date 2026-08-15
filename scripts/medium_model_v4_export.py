"""Export the frozen V4 design as data, for work downstream of this repo.

Written for someone about to MODEL THE VEHICLE, so the structural dimensions
are the primary product and everything else supports them.

ONE flight: the frozen V4 (docs/v4_medium_frozen/design.json) -- build-up drag,
first-principles engines, ramjet lit at the pull-out.

Writes out_medium_model/v4_export/:

  README.md                      what each file is, units, and what is NOT here
  structural_dimensions.json     PRIMARY -- geometry, stations, wing, materials
  structural_dimensions.csv      the same, flat (parameter,value,unit,...)
  mass_budget.csv                every mass term the model computes
  loads_envelope.json / .csv     load per phase, peak q, peak axial force
  timeseries.csv                 per-step state: trajectory, loads, forces, fuel
  fuel_budget.csv                per-phase fuel and mean forces
  engine_operating_points.csv    the first-principles engine march
  trajectory_command.json        the inputs that regenerate the trajectory

Everything is derived from the freeze and the state trace written by
scripts/medium_model_v4_final.py. Nothing is re-flown and nothing is re-typed.

Usage: python scripts/medium_model_v4_export.py
"""
from __future__ import annotations

import csv
import gzip
import json
import os
from math import atan, cos, degrees, pi, radians, tan
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from douglas_dart.atmosphere import standard_atmosphere  # noqa: E402
from medium_model import constants as C  # noqa: E402
from medium_model import drag_buildup as db  # noqa: E402
from medium_model import mission  # noqa: E402
from medium_model.mass_model import vehicle_dry_mass  # noqa: E402

OUT = ROOT / "out_medium_model"
EXPORT = OUT / "v4_export"
FREEZE = ROOT / "docs" / "v4_medium_frozen" / "design.json"
TAG = "v4_rungc_lap"

G0 = 9.80665
KG_PER_LB = 0.45359237
N_PER_LBF = 4.4482216152605
M_PER_IN = 0.0254
POWERED_MODES = ("v3_climb", "v4_pushover", "v3_dive", "v4_pullout",
                 "drag_strip")


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------
def structural(freeze) -> dict:
    v, w = freeze["vehicle"], freeze["wing_concept"]
    D = v["diameter_m"]
    d_th = v["throat_diameter_m"]
    L_ch = v["chamber_length_m"]
    L_tp = v["throat_length_m"]

    nose_L = C.NOSE_LENGTH_DIAMETERS * D
    tail_L = C.TAIL_LENGTH_DIAMETERS * D
    # The aft body AS FLOWN. drag_buildup never reads an exported base
    # diameter -- it derives one from its own boattail half-angle, so this
    # call reproduces the exact geometry the frozen V4 was flown with.
    base_d = db.base_diameter_m(D, tail_L, d_th)
    base_on = db.base_area_m2(base_d, d_th, engine_on=True)
    base_off = db.base_area_m2(base_d, d_th, engine_on=False)
    body_L = L_ch + L_tp + C.NOSE_TAIL_LENGTH_DIAMETERS * D
    # External mold line: nose fairing, then a constant-diameter barrel, then
    # a boattail closing to the nozzle exit. The barrel is what is left.
    barrel_L = body_L - nose_L - tail_L

    airfoil = C.AIRFOILS[w["airfoil_key"]]
    b, AR, lam = w["span_m"], w["aspect_ratio"], w["taper_ratio"]
    S = b * b / AR
    c_root = 2.0 * S / (b * (1.0 + lam))
    c_tip = lam * c_root
    mac = (2.0 / 3.0) * c_root * (1.0 + lam + lam * lam) / (1.0 + lam)
    y_mac = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    # Quarter-chord sweep from the leading-edge sweep, standard planform
    # relation: tan(L_c/4) = tan(L_LE) - (4/AR)*0.25*(1-lam)/(1+lam)
    sweep_le = w["sweep_deg"]
    tan_c4 = tan(radians(sweep_le)) - (4.0 / AR) * 0.25 * (1 - lam) / (1 + lam)

    fuel = C.FUELS[v["fuel_key"]]
    tank_capacity = (mission.FUEL_VOLUME_FRACTION_OF_ANNULUS
                     * mission.annular_volume_m3(D, d_th, L_tp)
                     * fuel.density_kg_per_m3)
    # Fuel loaded comes from the mass budget the design closed on
    # (wet = dry + fuel + payload margin), exactly as medium_model.design
    # derives it -- not back-computed from a burn cap.
    vm = json.loads((ROOT / "docs" / "v4_frozen" / "design.json").read_text()
                    )["verified_mission"]
    loaded = mission.loaded_fuel_kg(vm["dry_mass_kg"], vm["payload_margin_kg"])
    mass = vehicle_dry_mass(D, L_ch, d_th, L_tp, S, loaded)

    return {
        "provenance": {
            "source": "docs/v4_medium_frozen/design.json",
            "git_sha": freeze["git_sha"],
            "units": "SI throughout (m, kg, N, s, Pa, deg). Imperial "
                     "companions are given where a shop drawing wants them.",
        },
        "body": {
            "diameter_m": D,
            "radius_m": D / 2.0,
            "diameter_in": D / M_PER_IN,
            "overall_length_m": body_L,
            "overall_length_in": body_L / M_PER_IN,
            "fineness_ratio": body_L / D,
            "frontal_area_m2": pi * D * D / 4.0,
            "nose_fairing_length_m": nose_L,
            "nose_fairing_length_diameters": C.NOSE_LENGTH_DIAMETERS,
            "constant_diameter_barrel_length_m": barrel_L,
            "boattail_length_m": tail_L,
            "boattail_length_diameters": C.TAIL_LENGTH_DIAMETERS,
            # CORRECTED 2026-08-14. This key previously repeated the NOZZLE
            # FLOW diameter (115.638 mm), which was never the outer mould
            # line and was never what any flight used. The drag model does
            # not read a base diameter at all -- it DERIVES one from
            # BOATTAIL_HALF_ANGLE_DEG, and the frozen V4 was flown with the
            # 8 deg / 153.849 mm aft body below. Both the VSP model and the
            # 2D propulsion section found the old value independently.
            "boattail_exit_diameter_m": base_d,
            "boattail_base_outer_diameter_m": base_d,
            "boattail_half_angle_deg": db.BOATTAIL_HALF_ANGLE_DEG,
            "nozzle_flow_diameter_m": d_th,
            "annular_base_area_engine_on_m2": base_on,
            "annular_base_area_engine_off_m2": base_off,
            "aft_body_note":
                "The aft end closes from the full body diameter to "
                f"{base_d*1000:.2f} mm over the {tail_L*1000:.0f} mm boattail "
                f"-- exactly {db.BOATTAIL_HALF_ANGLE_DEG:.1f} deg, which is "
                "the attached-flow limit the drag model assumes and applies. "
                "It does NOT close onto the nozzle: an annulus of "
                f"{base_on*1e4:.2f} cm2 (engine on) remains as base area "
                "between the boattail base and the nozzle flow diameter, and "
                "base drag on it is a real term in the flown build-up.",
            "wetted_area_m2_cylinder_approx": pi * D * body_L,
        },
        "stations_from_nose_tip_m": {
            "nose_tip": 0.0,
            "nose_fairing_end__barrel_start": nose_L,
            "combustion_chamber_start": nose_L,
            "combustion_chamber_end__tailpipe_start": nose_L + L_ch,
            "tailpipe_end": nose_L + L_ch + L_tp,
            "boattail_start": body_L - tail_L,
            "nozzle_exit__body_end": body_L,
        },
        "internal_flowpath": {
            "chamber_diameter_m": D,
            "chamber_length_m": L_ch,
            "chamber_volume_m3": pi * D * D / 4.0 * L_ch,
            "tailpipe_diameter_m": d_th,
            "tailpipe_length_m": L_tp,
            "nozzle_exit_diameter_m": d_th,
            "throat_to_body_area_fraction": (d_th / D) ** 2,
            "operability_cap_on_that_fraction":
                C.PULSEJET_MAX_THROAT_AREA_FRACTION,
            "note": "The chamber IS the body diameter -- V4 deleted the "
                    "annular gap (user directive). The mass model treats the "
                    "tailpipe as a CONSTANT-diameter pipe at the nozzle "
                    "diameter, i.e. the chamber-to-tailpipe cone has zero "
                    "length in the model. Give it a real length in CAD; the "
                    "model does not tell you what it should be.",
        },
        "wing": {
            "span_m": b,
            "aspect_ratio": AR,
            "taper_ratio_tip_over_root": lam,
            "reference_area_m2": S,
            "root_chord_m": c_root,
            "tip_chord_m": c_tip,
            "mean_aerodynamic_chord_m": mac,
            "mac_spanwise_station_from_centreline_m": y_mac,
            "leading_edge_sweep_deg": sweep_le,
            "quarter_chord_sweep_deg": degrees(atan(tan_c4)),
            "airfoil_key": w["airfoil_key"],
            "thickness_to_chord": airfoil.thickness_ratio,
            "root_thickness_m": airfoil.thickness_ratio * c_root,
            "cl_max_2d": airfoil.cl_max,
            "cl_max_sweep_effective": airfoil.cl_max
            * cos(radians(sweep_le)) ** 2,
            "oswald_efficiency": C.OSWALD_BASE_E - C.OSWALD_TAPER_PENALTY
            * (lam - C.OSWALD_TAPER_MIN_DELTA_AT) ** 2,
            "note": "Planform only. The model has NO stability, CG or "
                    "moment-arm terms, so the wing's LONGITUDINAL POSITION on "
                    "the body is not an output of this repo -- it is yours to "
                    "choose. Likewise dihedral, incidence and twist, all of "
                    "which are zero here because they are unmodelled, not "
                    "because they were chosen.",
        },
        "tail_surfaces": {
            "note": "NOT MODELLED. There are no tail/fin surfaces anywhere in "
                    "this repo -- no area, no volume coefficient, no drag "
                    "term. A real vehicle needs them and their mass and drag "
                    "are absent from every number here.",
        },
        "materials_and_gauges": {
            "duct_material": "steel",
            "duct_wall_thickness_m": mass.duct_wall_thickness_m,
            "duct_wall_thickness_mm": mass.duct_wall_thickness_m * 1000.0,
            "duct_wall_driver": "max(hoop stress at pulsejet peak pressure, "
                                "minimum manufacturable gauge)",
            "steel_density_kg_m3": C.STEEL_DENSITY_KG_M3,
            "steel_allowable_stress_pa": C.STEEL_ALLOWABLE_STRESS_PA,
            "steel_min_gauge_m": C.STEEL_MIN_GAUGE_M,
            "pulsejet_peak_pressure_ratio": C.PULSEJET_PEAK_PRESSURE_RATIO,
            "pulsejet_peak_gauge_pressure_pa":
                (C.PULSEJET_PEAK_PRESSURE_RATIO - 1.0) * 101325.0,
            "skin_material": "CFRP",
            "skin_gauge_m": C.CFRP_MIN_GAUGE_M,
            "cfrp_density_kg_m3": C.CFRP_DENSITY_KG_M3,
            "structural_overhead_fraction": C.STRUCTURAL_OVERHEAD_FRACTION,
            "wing_areal_mass_kg_m2": C.WING_AREAL_MASS_KG_M2,
        },
        "fuel_system": {
            "fuel": fuel.display_name,
            "fuel_density_kg_m3": fuel.density_kg_per_m3,
            "tank_geometry": "annulus between the tailpipe OD and the body ID "
                             "over the tailpipe length",
            "annulus_volume_m3": mission.annular_volume_m3(D, d_th, L_tp),
            "usable_volume_fraction_of_annulus":
                mission.FUEL_VOLUME_FRACTION_OF_ANNULUS,
            "tank_capacity_kg": tank_capacity,
            "fuel_loaded_kg": loaded,
            "reserve_margin": mission.FUEL_RESERVE_MARGIN,
            "burn_limit_kg": mission.burn_limit_kg(loaded),
            "burn_limit_rule": "90% of LOADED fuel; the last 10% is "
                               "trapped/unusable",
            "fuel_actually_burned_kg":
                freeze["verified_mission"]["fuel_kg"],
            "annulus_volume_note":
                "This is the MODEL's tank volume: flow diameter to body "
                "diameter over the tailpipe length, ignoring wall thickness "
                "and treating the chamber-to-tailpipe cone as zero-length. "
                "The 2D propulsion section measured the as-drawn annulus at "
                "28.036 L / 6.911 kg capacity, 11.4% lower. That correction "
                "does not threaten the mission -- 6.911 kg still covers the "
                f"{loaded:.3f} kg loaded 2.25x over -- so the model's number "
                "is left as flown and the corrected one is stated here.",
            "as_drawn_annulus_volume_m3": 0.028036,
            "as_drawn_tank_capacity_kg": 6.911,
        },
        "inlet": inlet_requirement(freeze),
        "design_point": {
            "release_velocity_m_s": 40.0,
            "ramjet_light_mach": freeze["verified_mission"]
            ["ramjet_lightoff_mach"],
            "ramjet_light_altitude_m": freeze["verified_mission"]
            ["ramjet_lightoff_altitude_m"],
            "top_altitude_m": freeze["trajectory"]["top_altitude_m"],
            "cutoff_mach": freeze["constants_at_freeze"]["MOTOR_CUTOFF_MACH"],
            "peak_mach_flown": freeze["verified_mission"]["peak_mach"],
            "combustor_total_pressure_ratio": None,
            "combustor_total_pressure_note":
                "NOT EXPORTED BY THE PROPULSION MODEL. FpPropulsion's trace "
                "records thrust, fuel flow and equivalence ratio per "
                "re-convergence; combustor p0 is internal to ramjet-fp and "
                "is not surfaced. Getting it means adding it to the trace "
                "and re-flying (~35 min), which is doable on request.",
        },
        "payload_and_avionics": {
            "avionics_mass_kg": C.AVIONICS_FIXED_MASS_KG,
            "payload_ballast_margin_kg":
                mission.MAX_WET_MASS_KG - vehicle_dry_mass(
                    D, L_ch, d_th, L_tp, S, loaded).dry_mass_kg - loaded,
            "avionics_volume_m3": None,
            "avionics_longitudinal_constraint": None,
            "note": "The model carries avionics as a FIXED MASS ALLOWANCE and "
                    "nothing else -- no volume, no station, no shape. There "
                    "is no repo answer to give; both are free design "
                    "variables. Same for the payload/ballast margin, which is "
                    "by definition the slack left in the mass budget and can "
                    "be placed anywhere.",
        },
    }


def inlet_requirement(freeze) -> dict:
    """What the ramjet actually needs to swallow, derived from the FP march.

    The engine trace records equivalence ratio and fuel flow at every
    re-convergence, and phi is defined against the fuel's stoichiometric
    air/fuel ratio, so the air flow the combustor consumed is exactly
    mdot_air = mdot_fuel * (A/F)_stoich / phi. This is a REQUIREMENT the
    inlet has to meet, not an assumption about inlet geometry -- the repo has
    no inlet geometry at all.
    """
    tr = json.loads((OUT / f"{TAG}_final.json").read_text()).get(
        "engine_trace")
    fuel = C.FUELS[freeze["vehicle"]["fuel_key"]]
    af = fuel.stoichiometric_air_fuel_ratio
    pts = []
    for i in range(len(tr["time_s"])):
        phi = tr["ramjet_phi"][i]
        ff = tr["ramjet_fuel_kg_s"][i]
        if not tr["ramjet_lit"][i] or not phi or ff <= 0.0:
            continue
        atm = standard_atmosphere(tr["altitude_m"][i])
        v = tr["mach"][i] * atm.speed_of_sound_m_per_s
        mdot = ff * af / phi
        pts.append({"mach": round(tr["mach"][i], 4),
                    "altitude_m": round(tr["altitude_m"][i], 1),
                    "required_air_mass_flow_kg_s": round(mdot, 5),
                    "implied_capture_area_m2": round(
                        mdot / (atm.density_kg_per_m3 * v), 7)})
    worst = max(pts, key=lambda p: p["required_air_mass_flow_kg_s"])
    throat_a = pi * freeze["vehicle"]["throat_diameter_m"] ** 2 / 4.0
    return {
        "architecture": None,
        "architecture_note": "NOT MODELLED. The repo has no inlet geometry: "
                             "no capture area, no lip station, no diffuser, "
                             "no centrebody. What it does have is the flow "
                             "the engine consumed, below.",
        "required_air_mass_flow_kg_s_max": worst[
            "required_air_mass_flow_kg_s"],
        "at_mach": worst["mach"],
        "at_altitude_m": worst["altitude_m"],
        "implied_capture_area_m2": worst["implied_capture_area_m2"],
        "implied_capture_diameter_m": (4.0 * worst["implied_capture_area_m2"]
                                       / pi) ** 0.5,
        "capture_area_as_fraction_of_throat_area":
            worst["implied_capture_area_m2"] / throat_a,
        "derivation": "mdot_air = mdot_fuel * (A/F)_stoich / phi, using the "
                      f"FP ramjet's own phi and fuel flow and propane's "
                      f"(A/F)_stoich = {af}. Full stream-tube capture "
                      "assumed for the area (no spillage, no strut "
                      "blockage), so the area is a FLOOR.",
        "operating_points": pts,
    }


def mass_rows(freeze, dims):
    v, w = freeze["vehicle"], freeze["wing_concept"]
    S = w["span_m"] ** 2 / w["aspect_ratio"]
    loaded = dims["fuel_system"]["fuel_loaded_kg"]
    m = vehicle_dry_mass(v["diameter_m"], v["chamber_length_m"],
                         v["throat_diameter_m"], v["throat_length_m"],
                         S, loaded)
    wet = mission.MAX_WET_MASS_KG
    payload = wet - m.dry_mass_kg - loaded
    return [
        ("engine_duct_steel", m.engine_duct_kg, "steel duct: chamber + "
         "tailpipe wetted area x wall gauge x density"),
        ("airframe_skin_cfrp", m.airframe_skin_kg, "body skin area x gauge x "
         f"density x (1 + {C.STRUCTURAL_OVERHEAD_FRACTION} overhead)"),
        ("wing", m.wing_kg, f"{C.WING_AREAL_MASS_KG_M2} kg/m^2 x "
         f"{S:.5f} m^2 reference area"),
        ("avionics", m.avionics_kg, "fixed allowance"),
        ("tank_hardware", m.tank_hardware_kg, "fixed + fraction of fuel mass"),
        ("landing_hardware", m.landing_hardware_kg, "fixed allowance"),
        ("DRY_MASS", m.dry_mass_kg, "sum of the above"),
        ("fuel_loaded", loaded, "wet - dry - payload margin"),
        ("payload_ballast_margin", payload,
         "what is left inside the 50 lb cap"),
        ("WET_MASS_AT_RELEASE", wet, "50 lb, fixed by requirement"),
    ]


# --------------------------------------------------------------------------
# trace
# --------------------------------------------------------------------------
def load_states():
    p = OUT / f"{TAG}_states.json.gz"
    if not p.exists():
        raise SystemExit(
            f"{p} not found -- re-run scripts/medium_model_v4_final.py "
            f"--rung c --n-cells 324 --chamber-fraction 1.0 "
            f"--light-at-pullout (~35 min).")
    with gzip.open(p, "rt", encoding="utf-8") as fh:
        return json.load(fh)["states"]


def timeseries_rows(s, dt_s):
    n = len(s["time_s"])
    prev_fuel = 0.0
    rows = []
    for i in range(n):
        h = s["altitude_m"][i]
        v = s["velocity_m_per_s"][i]
        atm = standard_atmosphere(h)
        q = 0.5 * atm.density_kg_per_m3 * v * v
        fuel = s["fuel_burned_kg"][i]
        mdot = (fuel - prev_fuel) / dt_s if i else 0.0
        prev_fuel = fuel
        mass = s["mass_kg"][i]
        rows.append({
            "time_s": round(s["time_s"][i], 4),
            "mode": s["mode"][i],
            "downrange_m": round(s["distance_m"][i], 4),
            "altitude_m": round(h, 4),
            "velocity_m_s": round(v, 5),
            "mach": round(s["mach"][i], 6),
            "flight_path_angle_deg": round(
                degrees(s["flight_path_angle_rad"][i]), 5),
            "dynamic_pressure_pa": round(q, 3),
            "air_density_kg_m3": round(atm.density_kg_per_m3, 6),
            "static_temperature_k": round(atm.temperature_k, 4),
            "mass_kg": round(mass, 6),
            "thrust_n": round(s["thrust_n"][i], 4),
            "drag_n": round(s["drag_n"][i], 4),
            "thrust_to_weight": round(s["thrust_to_weight"][i], 6),
            "along_path_accel_m_s2": round(s["acceleration_m_per_s2"][i], 5),
            "along_path_accel_g": round(s["acceleration_m_per_s2"][i] / G0, 6),
            "load_n_roll_axial_g": round(s["load_n_roll"][i], 6),
            "load_n_yaw_normal_g": round(s["load_n_yaw"][i], 6),
            "load_n_total_g": round(s["load_n_total"][i], 6),
            "axial_force_n": round(s["load_n_roll"][i] * mass * G0, 3),
            "normal_force_n": round(s["load_n_yaw"][i] * mass * G0, 3),
            "turn_radius_m": round(s["turn_radius_m"][i], 3),
            "fuel_burned_kg": round(fuel, 7),
            "fuel_flow_kg_s": round(mdot, 8),
            "specific_impulse_s": round(s["specific_impulse_s"][i], 3),
            "stall_speed_m_s": round(s["stall_speed_m_per_s"][i], 4),
        })
    return rows


def write_csv(path, rows, fieldnames=None):
    if not rows:
        return None
    with open(path, "w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=fieldnames or list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    return path


def flatten(prefix, obj, out, category):
    for k, val in obj.items():
        if isinstance(val, dict):
            flatten(f"{prefix}{k}." if prefix else f"{k}.", val, out, category)
        elif k in ("note", "units"):
            out.append({"parameter": f"{prefix}{k}", "value": "", "unit": "",
                        "category": category, "notes": val})
        else:
            unit = ""
            for suffix, u in (("_m2", "m^2"), ("_m3", "m^3"),
                              ("_kg_m3", "kg/m^3"), ("_kg_m2", "kg/m^2"),
                              ("_m", "m"), ("_mm", "mm"), ("_in", "in"),
                              ("_kg", "kg"), ("_deg", "deg"), ("_pa", "Pa"),
                              ("_s", "s")):
                if k.endswith(suffix):
                    unit = u
                    break
            out.append({"parameter": f"{prefix}{k}", "value": val,
                        "unit": unit, "category": category, "notes": ""})


def main() -> None:
    freeze = json.loads(FREEZE.read_text())
    states = load_states()
    EXPORT.mkdir(parents=True, exist_ok=True)
    written = []

    # ---- structural dimensions (PRIMARY) ------------------------------
    dims = structural(freeze)
    p = EXPORT / "structural_dimensions.json"
    p.write_text(json.dumps(dims, indent=2), encoding="utf-8")
    written.append(p)

    flat = []
    for section, cat in (("body", "body"),
                         ("stations_from_nose_tip_m", "stations"),
                         ("internal_flowpath", "flowpath"),
                         ("wing", "wing"), ("tail_surfaces", "tail"),
                         ("materials_and_gauges", "materials"),
                         ("fuel_system", "fuel"),
                         ("design_point", "design_point"),
                         ("payload_and_avionics", "payload")):
        flatten("", {section: dims[section]}, flat, cat)
    written.append(write_csv(EXPORT / "structural_dimensions.csv", flat,
                             ["category", "parameter", "value", "unit",
                              "notes"]))

    # ---- mass budget --------------------------------------------------
    written.append(write_csv(EXPORT / "mass_budget.csv", [
        {"item": n, "mass_kg": round(v, 6), "mass_lb": round(v / KG_PER_LB, 5),
         "basis": why} for n, v, why in mass_rows(freeze, dims)]))

    # ---- time series --------------------------------------------------
    rows = timeseries_rows(states, freeze["fidelity"]["dt_s"])
    written.append(write_csv(EXPORT / "timeseries.csv", rows))

    # ---- per-phase budget ---------------------------------------------
    written.append(write_csv(EXPORT / "fuel_budget.csv", [
        {"phase": q["mode"], "powered": q["mode"] in POWERED_MODES,
         "t_start_s": q["t0"], "t_end_s": q["t1"], "duration_s": q["dt"],
         "mach_start": q["mach0"], "mach_end": q["mach1"],
         "altitude_start_m": q["alt0"], "altitude_end_m": q["alt1"],
         "gamma_start_deg": q["gamma0_deg"], "gamma_end_deg": q["gamma1_deg"],
         "peak_load_n_total_g": q["peak_load_n"], "fuel_kg": q["fuel_kg"],
         "mean_thrust_n": q["mean_thrust_n"], "mean_drag_n": q["mean_drag_n"],
         "min_along_path_accel_g": q["min_accel_g"]}
        for q in freeze["phases"]]))

    # ---- engine operating points --------------------------------------
    tr = json.loads((OUT / f"{TAG}_final.json").read_text()).get(
        "engine_trace")
    n_solves = freeze["fidelity"]["fp_solves"]
    if tr:
        erows = []
        for i in range(len(tr["time_s"])):
            pj, rj = tr["pulsejet_thrust_n"][i], tr["ramjet_thrust_n"][i]
            fpj, frj = tr["pulsejet_fuel_kg_s"][i], tr["ramjet_fuel_kg_s"][i]
            erows.append({
                "time_s": round(tr["time_s"][i], 3),
                "mach": round(tr["mach"][i], 5),
                "altitude_m": round(tr["altitude_m"][i], 3),
                "pulsejet_thrust_n": round(pj, 4),
                "ramjet_thrust_n": round(rj, 4),
                "total_thrust_n": round(pj + rj, 4),
                "pulsejet_fuel_kg_s": round(fpj, 8),
                "ramjet_fuel_kg_s": round(frj, 8),
                "total_fuel_kg_s": round(fpj + frj, 8),
                "ramjet_equivalence_ratio_phi": tr["ramjet_phi"][i],
                "ramjet_lit": tr["ramjet_lit"][i],
                "pulsejet_quenched": pj <= 0.0,
            })
        written.append(write_csv(EXPORT / "engine_operating_points.csv",
                                 erows))
        n_rows = len(erows)
    else:
        n_rows = 0

    # ---- loads envelope ------------------------------------------------
    env_rows = sorted(
        [{"phase": q["mode"], "powered": q["mode"] in POWERED_MODES,
          "peak_load_n_total_g": q["peak_load_n"],
          "mach_start": q["mach0"], "mach_end": q["mach1"],
          "altitude_start_m": q["alt0"], "altitude_end_m": q["alt1"],
          "duration_s": q["dt"]} for q in freeze["phases"]],
        key=lambda z: -z["peak_load_n_total_g"])
    written.append(write_csv(EXPORT / "loads_envelope.csv", env_rows))

    peak_q = peak_axial = peak_normal = 0.0
    peak_all = 0.0
    n = len(states["time_s"])
    for i in range(n):
        atm = standard_atmosphere(states["altitude_m"][i])
        v = states["velocity_m_per_s"][i]
        peak_q = max(peak_q, 0.5 * atm.density_kg_per_m3 * v * v)
        w_n = states["mass_kg"][i] * G0
        peak_axial = max(peak_axial, abs(states["load_n_roll"][i]) * w_n)
        peak_normal = max(peak_normal, abs(states["load_n_yaw"][i]) * w_n)
        peak_all = max(peak_all, states["load_n_total"][i])

    vm = freeze["verified_mission"]
    wet = mission.MAX_WET_MASS_KG
    env = {
        "design_limits_stated_in_the_trajectory": {
            "pullout_nominal_g": freeze["trajectory"]["pullout_load_factor"],
            "pullout_limiter_g":
                freeze["trajectory"]["pullout_max_load_factor"],
            "return_loop_g": 6.0,
            "gate_used_in_the_reports_g": 4.0,
        },
        "WARNING_worst_load_is_not_the_pullout": (
            f"The unpowered return half-loop pulls a fixed 6 g by "
            f"construction and total |n| there reaches "
            f"{max(q['peak_load_n'] for q in freeze['phases'] if q['mode'] == 'loop'):.2f} g "
            f"-- higher than any powered phase. peak_load_n_total in the "
            f"mission summary tracks POWERED steps ONLY (deliberately: the "
            f"loop is a fixed constant of the landing profile, not something "
            f"the trajectory search controls). For STRUCTURE, the loop is the "
            f"sizing case. Read loads_envelope.csv, not peak_load_n_total."),
        "peak_load_n_total_powered_g": vm["peak_load_n_total"],
        "peak_load_n_yaw_powered_g": vm["peak_load_n_yaw"],
        "peak_load_n_roll_powered_g": vm["peak_load_n_roll"],
        "peak_load_phase_powered": vm["peak_load_mode"],
        "peak_load_n_total_whole_flight_g": round(peak_all, 4),
        "peak_dynamic_pressure_pa": round(peak_q, 1),
        "peak_axial_force_n": round(peak_axial, 1),
        "peak_axial_force_lbf": round(peak_axial / N_PER_LBF, 2),
        "peak_normal_force_n": round(peak_normal, 1),
        "peak_normal_force_lbf": round(peak_normal / N_PER_LBF, 2),
        "per_phase": env_rows,
        "reference_weight": {
            "wet_mass_kg": wet, "wet_weight_n": wet * G0,
            "wet_weight_lbf": wet * G0 / N_PER_LBF,
            "note": "load factor x wet weight = force, for a mass that only "
                    "drops ~2 kg over the powered flight",
        },
        "not_modelled": [
            "distributed aerodynamic loads (this is a point-mass model -- "
            "there is no spanwise or longitudinal load distribution)",
            "centre of gravity, moments of inertia, static margin",
            "any structural response: no stress, no buckling, no flutter, "
            "no margins of safety",
            "landing/ground loads beyond a touchdown speed",
            "thermal loads from the duct (the chamber IS the body skin here)",
        ],
    }
    p = EXPORT / "loads_envelope.json"
    p.write_text(json.dumps(env, indent=2), encoding="utf-8")
    written.append(p)

    # ---- trajectory command -------------------------------------------
    p = EXPORT / "trajectory_command.json"
    p.write_text(json.dumps({
        "how_to_regenerate": freeze["how_to_refly"],
        "commanded": freeze["trajectory"],
        "fidelity": freeze["fidelity"],
        "release": {"velocity_m_s": 40.0,
                    "note": "rail/catapult launch; the vehicle leaves the "
                            "rail BELOW its full-mass stall speed. There is "
                            "no ground-roll model."},
        "motor_cutoff_mach":
            freeze["constants_at_freeze"]["MOTOR_CUTOFF_MACH"],
        "phase_order": [q["mode"] for q in freeze["phases"]],
        "outcome": {k: vm[k] for k in (
            "peak_mach", "cutoff", "dive_exit_mach", "ramjet_lightoff_mach",
            "ramjet_lightoff_altitude_m", "ramjet_lightoff_time_s",
            "ramjet_lightoff_mode", "ramjet_lit_in_dive", "fuel_kg",
            "flight_s", "safe_landing", "stalled", "lands_from_launch_m")},
        "engine_events": freeze["engine_events"],
    }, indent=2), encoding="utf-8")
    written.append(p)

    written.append(write_readme(freeze, dims, len(rows), n_rows, n_solves))

    for pth in written:
        if pth:
            print(f"wrote {pth.relative_to(ROOT)}")


def write_readme(freeze, dims, n_steps, n_engine_rows, n_solves) -> Path:
    b = dims["body"]
    st = dims["stations_from_nose_tip_m"]
    fp = dims["internal_flowpath"]
    vm = freeze["verified_mission"]
    loop_g = max(q["peak_load_n"] for q in freeze["phases"]
                 if q["mode"] == "loop")
    lines = [
        "# Douglas Dart V4 — design data export",
        "",
        f"Source: `docs/v4_medium_frozen/design.json` (git "
        f"`{freeze['git_sha'][:7]}`). Generated by "
        f"`scripts/medium_model_v4_export.py`; nothing here is hand-typed.",
        "",
        "One flight: build-up drag, first-principles engines at CONFIRM "
        "resolution (324 cells), chamber = body diameter, ramjet lit at the "
        "pull-out. It reaches **Mach "
        f"{vm['peak_mach']:.3f}** and makes motor cutoff.",
        "",
        "**Units are SI throughout** — m, kg, N, s, Pa, degrees. A few "
        "imperial companions are included where a shop drawing wants them, "
        "and are always suffixed (`_in`, `_lb`, `_lbf`).",
        "",
        "## Start here",
        "",
        "`structural_dimensions.json` / `.csv` — the vehicle. Same content, "
        "one nested and one flat.",
        "",
        f"The body is **{b['diameter_m']*1000:.1f} mm diameter × "
        f"{b['overall_length_m']*1000:.0f} mm long** "
        f"(fineness {b['fineness_ratio']:.2f}), laid out from the nose tip as:",
        "",
        "| station | x from nose tip (mm) |",
        "|---|---|",
        f"| nose tip | {st['nose_tip']*1000:.0f} |",
        f"| nose fairing ends, barrel + chamber start | "
        f"{st['nose_fairing_end__barrel_start']*1000:.1f} |",
        f"| chamber ends, tailpipe starts | "
        f"{st['combustion_chamber_end__tailpipe_start']*1000:.1f} |",
        f"| tailpipe ends, boattail starts | "
        f"{st['boattail_start']*1000:.1f} |",
        f"| nozzle exit / body end | {st['nozzle_exit__body_end']*1000:.1f} |",
        "",
        "## Files",
        "",
        "| file | what |",
        "|---|---|",
        "| `structural_dimensions.json` / `.csv` | geometry, stations, wing "
        "planform, materials and gauges, fuel system |",
        "| `mass_budget.csv` | every mass term the model computes, with the "
        "basis for each |",
        "| `loads_envelope.json` / `.csv` | load per phase, peak dynamic "
        "pressure, peak axial and normal force |",
        f"| `timeseries.csv` | per-timestep state — trajectory, forces, body "
        f"loads, fuel flow. {n_steps:,} rows at "
        f"{freeze['fidelity']['dt_s']} s |",
        "| `fuel_budget.csv` | per-phase fuel, mean thrust/drag, worst "
        "acceleration |",
        f"| `engine_operating_points.csv` | the first-principles engine march "
        f"— {n_engine_rows} rows, one per re-convergence event |",
        "| `trajectory_command.json` | the inputs that regenerate the "
        "trajectory, plus the outcome |",
        "",
        "## Read this before sizing anything",
        "",
        f"**The worst body load in the flight is not the pull-out.** The "
        f"powered peak is {vm['peak_load_n_total']:.2f} g in "
        f"`{vm['peak_load_mode']}`. But the unpowered return half-loop pulls "
        f"a fixed 6 g by construction, and total |n| there reaches "
        f"**{loop_g:.2f} g**. The mission summary's `peak_load_n_total` "
        f"tracks *powered* steps only, deliberately, so that load does not "
        f"appear in it. `loads_envelope.csv` does include it. If you size to "
        f"the {freeze['trajectory']['pullout_load_factor']:.1f} g nominal "
        f"pull-out you will be under by a factor of ~"
        f"{loop_g/freeze['trajectory']['pullout_load_factor']:.0f}.",
        "",
        "**What this repo does not model, at all:**",
        "",
        "- Structure. No stress, buckling, flutter, or margins of safety "
        "anywhere. The 4 g figure quoted in the reports is a *stated design "
        "limit*, not a demonstrated capability.",
        "- Centre of gravity, moments of inertia, static margin. This is a "
        "point-mass model, so the **wing's longitudinal position is not an "
        "output** — it is yours to choose.",
        "- Tail or fin surfaces. There are none: no area, no mass, no drag.",
        "- Distributed aerodynamic loads. Only the resultant force at the CG.",
        "- Thermal loads. On this design the combustion chamber *is* the body "
        "skin, which is a real thermal problem the model is silent about.",
        "- The side valve runners, which still do not fit inside a "
        f"{b['diameter_m']*1000:.0f} mm skin. An external fairing is needed "
        "and its drag is in neither the flat CD0 nor the build-up.",
        "",
        "**Modelling simplifications that will surprise you in CAD:**",
        "",
        "- The mass model treats the tailpipe as a **constant-diameter pipe "
        f"at the nozzle diameter** ({fp['tailpipe_diameter_m']*1000:.1f} mm), "
        "so the chamber-to-tailpipe cone has *zero length*. Give it a real "
        "length; the model does not tell you what it should be.",
        "- The external boattail closes from full body diameter to the nozzle "
        f"diameter over the last {b['boattail_length_m']*1000:.0f} mm "
        "(1.0 diameter). The nose fairing is "
        f"{b['nose_fairing_length_m']*1000:.0f} mm (2.0 diameters). Neither "
        "has a specified profile — only a length.",
        "- Wing dihedral, incidence and twist are all zero because they are "
        "*unmodelled*, not because they were chosen.",
        "",
        "**On `engine_operating_points.csv`:** thrust is a *step-hold*, not a "
        "continuous function. Each row is a converged transient at that "
        "(Mach, altitude), held until the next re-convergence — that is the "
        "propulsion model's real resolution, and it is why the thrust trace "
        "in the time series is a staircase. The pulsejet and ramjet "
        f"re-converge on independent drift triggers, so the row count "
        f"({n_engine_rows}) is not the total solve count ({n_solves}). "
        "Interpolating between rows is your call; the model does not claim "
        "the intermediate values.",
        "",
        "## Known problems with this flight",
        "",
        "These are in the freeze too, and none of them is fixed:",
        "",
    ]
    for k in ("findings", "known_risks"):
        for item in freeze[k]:
            lines.append(f"- {item}")
    lines += [
        "",
        "## Provenance",
        "",
        f"- Frozen: `docs/v4_medium_frozen/design.json`, git "
        f"`{freeze['git_sha'][:7]}`",
        "- Guarded by `tests/test_v4_medium_frozen.py`, which by default "
        "checks the freeze against the stored per-step trace rather than "
        "re-flying (the re-fly costs ~35 min; "
        "`DOUGLAS_DART_REFLY_FP_RUNGS=1` forces it).",
        "",
    ]
    p = EXPORT / "README.md"
    p.write_text("\n".join(lines), encoding="utf-8")
    return p


if __name__ == "__main__":
    main()
