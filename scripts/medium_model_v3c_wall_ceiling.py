"""Closes the loop on the M 0.45 wall: FP balance + the CLIMB ceiling.

Two ceilings decide the gate, and the lower one binds:

  1. THRUST-BALANCE ceiling -- the Mach where  e + sin(gamma_dive) = 0.26,
     with  e = (T_pulsejet - D)/W.  Computed here on FP thrust (from
     out_medium_model/v3c_wall_fp.json) as well as closed form.

  2. ALTITUDE-BUDGET ceiling -- a later gate makes derive_top_altitude buy
     a higher top of climb, but the pulsejet's own service ceiling in the
     commanded climb is  e(M_climb, h) = sin(gamma_climb).  When the
     REQUIRED top crosses the ATTAINABLE top, the profile stops existing.

Both are printed against the gate ladder so the crossing is visible.
"""
from __future__ import annotations
import json, os
from math import pi, radians, sin, cos
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from douglas_dart.atmosphere import standard_atmosphere        # noqa: E402
from medium_model.constants import (NOSE_TAIL_LENGTH_DIAMETERS,  # noqa: E402
                                    TAIL_LENGTH_DIAMETERS)
from medium_model.design import load_frozen_design             # noqa: E402
from medium_model.drag_buildup import total_drag_buildup       # noqa: E402
from medium_model.pulsejet_simple import pulsejet_thrust       # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
G0 = 9.80665
TARGET_G = 0.26
W_MID_KG = 22.04              # measured mid-dive mass (wall_balance.py)
W_CLIMB_KG = 22.4             # measured mid-climb mass


def gc_of(d):
    g, w = d.geometry, d.wing
    return dict(
        diameter_m=g.diameter_m, duct_exit_diameter_m=g.throat_diameter_m,
        tail_length_m=TAIL_LENGTH_DIAMETERS * g.diameter_m,
        body_length_m=(g.chamber_length_m + g.throat_length_m
                       + NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m),
        wing_area_m2=w.span_m ** 2 / w.aspect_ratio,
        aspect_ratio=w.aspect_ratio, wing_sweep_deg=w.sweep_deg,
        wing_thickness_ratio=getattr(w.airfoil, "thickness_ratio", 0.03),
        oswald_e=w.oswald_e,
        lip_area_m2=pi * g.throat_diameter_m ** 2 / 4.0)


def drag_n(d, gc, mach, alt, mass_kg, gamma_rad):
    atm = standard_atmosphere(alt)
    v = mach * atm.speed_of_sound_m_per_s
    b = total_drag_buildup(
        diameter_m=gc["diameter_m"], body_length_m=gc["body_length_m"],
        duct_exit_diameter_m=gc["duct_exit_diameter_m"],
        tail_length_m=gc["tail_length_m"],
        wing_reference_area_m2=gc["wing_area_m2"],
        wing_thickness_ratio=gc["wing_thickness_ratio"],
        wing_sweep_deg=gc["wing_sweep_deg"], velocity_m_per_s=v,
        density_kg_per_m3=atm.density_kg_per_m3,
        temperature_k=atm.temperature_k, mach=mach,
        required_lift_n=mass_kg * G0 * cos(gamma_rad),
        oswald_efficiency=gc["oswald_e"], wing_aspect_ratio=gc["aspect_ratio"],
        engine_on=True,
        captured_mass_flow_kg_per_s=0.90 * atm.density_kg_per_m3 * v
        * gc["lip_area_m2"], lip_area_m2=gc["lip_area_m2"],
        cowl_suction_recovery=None)
    return b.total_n


def pj_cf(d, mach, alt):
    atm = standard_atmosphere(alt)
    return pulsejet_thrust(d.geometry.diameter_m, d.geometry.chamber_length_m,
                           d.geometry.throat_diameter_m,
                           d.geometry.throat_length_m, mach, alt,
                           d.geometry.fuel,
                           atmosphere=atm).average_thrust_n


def fp_model(rows):
    """Linear-in-Mach, linear-in-altitude fit to the FP queries. The FP
    data is very nearly linear in both (thrust falls ~1.8 N per 0.05 Mach
    and ~19.5 N per 600 m), so a bilinear fit is not a smoothing choice,
    it is what the data says."""
    low = {r["mach"]: r["thrust_n"] for r in rows if r["row"] == "low"}
    high = {r["mach"]: r["thrust_n"] for r in rows if r["row"] == "high"}

    def at(mach, alt):
        def interp(tbl):
            ks = sorted(tbl)
            m = min(max(mach, ks[0]), ks[-1])
            for a, b in zip(ks, ks[1:]):
                if a <= m <= b:
                    f = (m - a) / (b - a)
                    return tbl[a] + f * (tbl[b] - tbl[a])
            return tbl[ks[-1]]
        t300, t900 = interp(low), interp(high)
        return t300 + (alt - 300.0) / 600.0 * (t900 - t300)
    return at


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    gc = gc_of(d)
    fp_rows = json.loads((OUT / "v3c_wall_fp.json").read_text())
    fp_at = fp_model([r for r in fp_rows if "thrust_n" in r])
    w_mid = W_MID_KG * G0
    w_climb = W_CLIMB_KG * G0

    print("=== FP vs closed-form pulsejet, and e = (T-D)/W ===")
    print(f"W_mid-dive = {w_mid:.1f} N; drag at dive attitude (-20 deg)")
    print(f"{'M':>5} {'alt':>6} {'T_cf':>7} {'T_fp':>7} {'FP/CF':>6} "
          f"{'D':>7} {'e_cf':>7} {'e_fp':>7} {'a_fp@10':>8} {'a_fp@20':>8}")
    tbl = []
    for alt in (300.0, 900.0):
        for mach in (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60):
            tcf = pj_cf(d, mach, alt)
            tfp = fp_at(mach, alt)
            dd = drag_n(d, gc, mach, alt, W_MID_KG, radians(-20.0))
            ecf, efp = (tcf - dd) / w_mid, (tfp - dd) / w_mid
            a10 = efp + sin(radians(10.0))
            a20 = efp + sin(radians(20.0))
            print(f"{mach:5.2f} {alt:6.0f} {tcf:7.1f} {tfp:7.1f} "
                  f"{tfp/tcf:6.3f} {dd:7.1f} {ecf:+7.3f} {efp:+7.3f} "
                  f"{a10:+8.3f} {a20:+8.3f}")
            tbl.append(dict(mach=mach, alt=alt, t_cf=tcf, t_fp=tfp, drag=dd,
                            e_cf=ecf, e_fp=efp))
        print()

    # ---- ceiling Mach from the FP balance -----------------------------
    def ceiling(alt, want, thrust_fn, weight):
        prev_m = prev_e = None
        m = 0.28
        while m <= 0.85001:
            e = (thrust_fn(m, alt) - drag_n(d, gc, m, alt, W_MID_KG,
                                            radians(-20.0))) / weight
            if prev_e is not None and prev_e >= want > e:
                f = (prev_e - want) / (prev_e - e)
                return prev_m + f * (m - prev_m)
            prev_m, prev_e = m, e
            m += 0.005
        return float("nan")

    print("=== THRUST-BALANCE ceiling Mach (gate needs a >= 0.26 g) ===")
    print(f"{'alt':>6} {'model':>6} {'M(T=D)':>8} {'dive 10':>9} "
          f"{'dive 14':>9} {'dive 20':>9}")
    ceilings = {}
    for alt in (130.0, 300.0, 600.0, 900.0, 1200.0):
        for tag, fn in (("cf", lambda m, a: pj_cf(d, m, a)), ("fp", fp_at)):
            row = [f"{alt:6.0f} {tag:>6} {ceiling(alt, 0.0, fn, w_mid):8.3f}"]
            for dv in (10.0, 14.0, 20.0):
                mc = ceiling(alt, TARGET_G - sin(radians(dv)), fn, w_mid)
                ceilings[f"{tag}|{alt}|{dv}"] = mc
                row.append(f"{mc:9.3f}")
            print("".join(row))

    # ---- the CLIMB ceiling: how high can this vehicle even GET? -------
    # In a steady climb at gamma_c the vehicle needs e >= sin(gamma_c) just
    # to hold Mach. The altitude where e(M_climb, h) = sin(gamma_c) is the
    # pulsejet's service ceiling on that profile.
    print("\n=== ATTAINABLE top of climb: e(M,h) = sin(gamma_climb) ===")
    print("(pulsejet-only service ceiling in the commanded climb, m)")
    print(f"{'M_climb':>8} {'model':>6} " + "".join(
        f"{'climb ' + f'{c:.0f}':>11}" for c in (6.0, 10.0, 14.0, 16.0)))
    svc = {}
    for m_climb in (0.25, 0.28, 0.30):
        for tag, fn in (("cf", lambda m, a: pj_cf(d, m, a)), ("fp", fp_at)):
            cells = []
            for climb in (6.0, 10.0, 14.0, 16.0):
                want = sin(radians(climb))
                prev_h = prev_e = None
                h = 100.0
                hit = float("nan")
                while h <= 4000.0:
                    e = (fn(m_climb, h)
                         - drag_n(d, gc, m_climb, h, W_CLIMB_KG,
                                  radians(climb))) / w_climb
                    if prev_e is not None and prev_e >= want > e:
                        f = (prev_e - want) / (prev_e - e)
                        hit = prev_h + f * (h - prev_h)
                        break
                    prev_h, prev_e = h, e
                    h += 25.0
                svc[f"{tag}|{m_climb}|{climb}"] = hit
                cells.append(f"{hit:11.0f}" if hit == hit else f"{'>4000':>11}")
            print(f"{m_climb:8.2f} {tag:>6} " + "".join(cells))

    (OUT / "v3c_wall_ceiling.json").write_text(json.dumps(
        dict(balance=tbl, ceilings=ceilings, service_ceiling=svc), indent=2))
    print(f"\nwrote {OUT / 'v3c_wall_ceiling.json'}")


if __name__ == "__main__":
    main()
