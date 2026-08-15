"""V3a wing sizing sweep: span x aspect ratio, closed-form propulsion,
component build-up drag.

Motivation: at the 40 m/s sled release (M ~0.117) the V3a vehicle's drag is
~71 N of which ~69 N is INDUCED -- the wing is far too small for the release
speed. In this drag model

    D_i = L^2 / (q * pi * e * AR * S) = L^2 / (q * pi * e * b^2)

so induced drag depends on SPAN ONLY (AR*S == b^2); aspect ratio moves wing
wetted area S = b^2/AR (profile + wave drag), wing mass, and stall speed.

Everything else (body geometry, trajectory, fuel allocation) is held at the
frozen V3a values. Wing mass is re-costed against the mass budget so the
bigger wings are charged for themselves.

Run:
  MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python \
      scripts/medium_model_v3a_wing_span_ar_sweep.py [--baseline]
"""
from __future__ import annotations

import os
import sys
from math import cos, pi, radians

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.constants import (AIRFOILS, G0_M_PER_S2,  # noqa: E402
                                    NOSE_TAIL_LENGTH_DIAMETERS,
                                    TAIL_LENGTH_DIAMETERS)
from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.drag import WingConcept                    # noqa: E402
from medium_model.drag_buildup import total_drag_buildup     # noqa: E402
from medium_model.flight_sim import (DEFAULT_RELEASE_VELOCITY_M_PER_S,  # noqa: E402
                                     standard_atmosphere)
from medium_model.mass_model import vehicle_dry_mass         # noqa: E402
from medium_model.mission import MAX_WET_MASS_KG             # noqa: E402

DESIGN = "docs/v3a_medium_model/design.json"


def launch_drag(design, wing: WingConcept, gamma_deg: float,
                v: float = DEFAULT_RELEASE_VELOCITY_M_PER_S):
    """Component drag build-up at the release condition (sea level, engine
    on, cold duct swallowing ~0.90*rho*V*A_lip -- same convention the sim
    uses for an unlit duct)."""
    g = design.geometry
    atm = standard_atmosphere(0.0)
    mach = v / atm.speed_of_sound_m_per_s
    lip_area = pi * g.throat_diameter_m ** 2 / 4.0
    mdot = 0.90 * atm.density_kg_per_m3 * v * lip_area
    lift = MAX_WET_MASS_KG * G0_M_PER_S2 * cos(radians(gamma_deg))
    return mach, total_drag_buildup(
        diameter_m=g.diameter_m,
        body_length_m=(g.chamber_length_m + g.throat_length_m
                       + NOSE_TAIL_LENGTH_DIAMETERS * g.diameter_m),
        duct_exit_diameter_m=g.throat_diameter_m,
        tail_length_m=TAIL_LENGTH_DIAMETERS * g.diameter_m,
        wing_reference_area_m2=wing.reference_area_m2,
        wing_thickness_ratio=wing.airfoil.thickness_ratio,
        wing_sweep_deg=wing.sweep_deg,
        velocity_m_per_s=v, density_kg_per_m3=atm.density_kg_per_m3,
        temperature_k=atm.temperature_k, mach=mach, required_lift_n=lift,
        oswald_efficiency=wing.oswald_e, wing_aspect_ratio=wing.aspect_ratio,
        engine_on=True, captured_mass_flow_kg_per_s=mdot,
        lip_area_m2=lip_area, cowl_suction_recovery=None)


def dry_mass_with(design, wing: WingConcept) -> float:
    g = design.geometry
    return vehicle_dry_mass(
        diameter_m=g.diameter_m, chamber_length_m=g.chamber_length_m,
        throat_diameter_m=g.throat_diameter_m,
        throat_length_m=g.throat_length_m,
        wing_area_m2=wing.reference_area_m2,
        fuel_loaded_kg=design.loaded_fuel_kg).dry_mass_kg


def run_one(design, span: float, ar: float, base: WingConcept):
    wing = WingConcept(span, ar, base.taper_ratio, base.sweep_deg,
                       base.airfoil)
    r = fly(design, drag_model="buildup", wing_concept=wing)
    peak_m = max(s.mach for s in r.states)
    fuel = max(s.fuel_burned_kg for s in r.states)
    _, bd = launch_drag(design, wing,
                        design.climb_dive.initial_climb_angle_deg)
    dry = dry_mass_with(design, wing)
    margin = MAX_WET_MASS_KG - dry - design.loaded_fuel_kg
    return dict(
        span=span, ar=ar, area=wing.reference_area_m2,
        trav=r.min_traverse_accel_g, powg=r.min_powered_accel_g,
        trav_m=r.min_traverse_accel_mach, pow_m=r.min_accel_mach,
        top=r.climb_dive_top_altitude_m or 0.0,
        peak=peak_m, cutoff=r.motor_cutoff_reached,
        margin_tw=r.min_powered_thrust_margin, fuel=fuel,
        safe=r.safe_landing, stalled=r.stalled,
        rule=r.rule_violated,
        d_ind=bd.induced_n, d_wing=bd.wing_profile_n, d_tot=bd.total_n,
        dry=dry, mass_margin=margin, t_end=r.states[-1].time_s,
    )


HDR = (f"{'span':>5} {'AR':>5} {'S':>6} {'trav_g':>7} {'pow_g':>7} "
       f"{'peakM':>6} {'cut':>5} {'T/W':>6} {'fuel':>6} {'safe':>5} "
       f"{'Dind0':>7} {'Dwing0':>7} {'Dtot0':>7} {'dry':>6} {'margin':>7}")


def fmt(r):
    return (f"{r['span']:5.2f} {r['ar']:5.2f} {r['area']:6.3f} "
            f"{r['trav']:7.3f} {r['powg']:7.3f} {r['peak']:6.3f} "
            f"{str(r['cutoff']):>5} {r['margin_tw']:6.2f} {r['fuel']:6.3f} "
            f"{str(r['safe']):>5} {r['d_ind']:7.1f} {r['d_wing']:7.1f} "
            f"{r['d_tot']:7.1f} {r['dry']:6.2f} {r['mass_margin']:7.2f}")


def main():
    d = load_frozen_design(DESIGN)
    base = d.wing
    print(f"# {d.name}")
    print(f"# CD0_frontal(frozen)={d.cd0_frontal}  loaded_fuel="
          f"{d.loaded_fuel_kg:.3f} kg  burn_limit={d.burn_limit_kg:.3f} kg")
    print(f"# baseline wing: span={base.span_m:.4f} AR={base.aspect_ratio:.4f}"
          f" taper={base.taper_ratio:.3f} sweep={base.sweep_deg:.2f} "
          f"airfoil={base.airfoil.key} e={base.oswald_e:.4f}")
    print(f"# trajectory: climb={d.climb_dive.initial_climb_angle_deg:.3f} "
          f"dive={d.climb_dive.dive_angle_deg:.3f} "
          f"floor={d.climb_dive.floor_altitude_m:.1f}")

    if "--baseline" in sys.argv:
        mach, bd = launch_drag(d, base, d.climb_dive.initial_climb_angle_deg)
        print(f"\n# launch drag build-up @ V=40 m/s, M={mach:.4f}, sea level")
        for k in ("friction_n", "form_n", "base_n", "wave_n", "spillage_n",
                  "wing_profile_n", "induced_n", "total_n"):
            print(f"#   {k:>16}: {getattr(bd, k):8.2f} N")
        print(HDR)
        print(fmt(run_one(d, base.span_m, base.aspect_ratio, base)))
        return

    if "--dt" in sys.argv:
        # the traverse minimum lands within a few states of the ramjet
        # lightoff mode switch, so check it is not a timestep artifact
        print(f"{'span':>5} {'AR':>5} {'dt':>6} {'trav_g':>7} {'travM':>6} "
              f"{'pow_g':>7} {'peakM':>6} {'fuel':>6}", flush=True)
        combos = ((base.span_m, base.aspect_ratio), (0.74, 4.0),
                  (0.82, 5.0), (0.86, 5.5), (0.90, 6.0))
        if "--confirm" in sys.argv:   # the recommended point + neighbours
            combos = ((0.76, 4.5), (0.78, 4.0), (0.78, 4.5), (0.78, 5.0),
                      (0.80, 4.5), (0.78, 4.25), (0.78, 4.75))
        for span, ar in combos:
            wing = WingConcept(span, ar, base.taper_ratio, base.sweep_deg,
                               base.airfoil)
            for dt in (0.02, 0.01, 0.005):
                r = fly(d, drag_model="buildup", wing_concept=wing, dt_s=dt)
                print(f"{span:5.2f} {ar:5.2f} {dt:6.3f} "
                      f"{r.min_traverse_accel_g:7.3f} "
                      f"{r.min_traverse_accel_mach:6.3f} "
                      f"{r.min_powered_accel_g:7.3f} "
                      f"{max(s.mach for s in r.states):6.3f} "
                      f"{max(s.fuel_burned_kg for s in r.states):6.3f}",
                      flush=True)
        return

    if "--peak" in sys.argv:
        # fine grid around the observed optimum, plus stall flag and the
        # induced drag AT THE PINCH (level flight, the state where
        # min_traverse_accel_g is recorded) -- the number that explains why
        # the launch-induced-drag lever saturates.
        print(f"{'span':>5} {'AR':>5} {'S':>6} {'trav_g':>7} {'travM':>6} "
              f"{'pow_g':>7} {'peakM':>6} {'cut':>5} {'T/W':>6} {'fuel':>6} "
              f"{'safe':>5} {'stall':>5} {'Dind0':>7} {'Dtot0':>7} "
              f"{'Dind*':>6} {'Dtot*':>7} {'dry':>6} {'margin':>6}",
              flush=True)
        combos = [(base.span_m, base.aspect_ratio)]
        if "--finalists" in sys.argv:
            combos += [(0.70, 3.5), (0.74, 4.0), (0.78, 4.0), (0.78, 4.25),
                       (0.78, 4.5), (0.80, 4.5), (0.82, 4.5), (0.86, 5.5),
                       (0.90, 6.0)]
        else:
            for span in (0.74, 0.78, 0.82, 0.86, 0.90, 0.94):
                for ar in (4.5, 5.0, 5.5, 6.0, 6.5):
                    combos.append((span, ar))
        for span, ar in combos:
            wing = WingConcept(span, ar, base.taper_ratio, base.sweep_deg,
                               base.airfoil)
            r = fly(d, drag_model="buildup", wing_concept=wing)
            st = min(r.states, key=lambda s: abs(s.mach
                                                 - r.min_traverse_accel_mach))
            atm = standard_atmosphere(st.altitude_m)
            _, bdp = launch_drag(d, wing, 0.0,
                                 v=st.mach * atm.speed_of_sound_m_per_s)
            _, bd0 = launch_drag(d, wing,
                                 d.climb_dive.initial_climb_angle_deg)
            dry = dry_mass_with(d, wing)
            print(f"{span:5.2f} {ar:5.2f} {wing.reference_area_m2:6.3f} "
                  f"{r.min_traverse_accel_g:7.3f} "
                  f"{r.min_traverse_accel_mach:6.3f} "
                  f"{r.min_powered_accel_g:7.3f} "
                  f"{max(s.mach for s in r.states):6.3f} "
                  f"{str(r.motor_cutoff_reached):>5} "
                  f"{r.min_powered_thrust_margin:6.2f} "
                  f"{max(s.fuel_burned_kg for s in r.states):6.3f} "
                  f"{str(r.safe_landing):>5} {str(r.stalled):>5} "
                  f"{bd0.induced_n:7.1f} {bd0.total_n:7.1f} "
                  f"{bdp.induced_n:6.1f} {st.drag_n:7.1f} {dry:6.2f} "
                  f"{MAX_WET_MASS_KG - dry - d.loaded_fuel_kg:6.2f}",
                  flush=True)
        return

    if "--pinch" in sys.argv:
        # thrust/drag vs Mach through the pinch, baseline vs a big wing
        from medium_model.drag import stall_speed_concept_m_per_s
        for wing in (base, WingConcept(0.74, 4.0, base.taper_ratio,
                                       base.sweep_deg, base.airfoil)):
            r = fly(d, drag_model="buildup", wing_concept=wing)
            print(f"\n## wing span={wing.span_m:.3f} AR={wing.aspect_ratio:.2f}"
                  f" S={wing.reference_area_m2:.3f} "
                  f"Vstall(sl,wet)={stall_speed_concept_m_per_s(wing, MAX_WET_MASS_KG, 1.225, G0_M_PER_S2):.1f} m/s"
                  f"  stalled={r.stalled} trav={r.min_traverse_accel_g:.3f}"
                  f" @M{r.min_traverse_accel_mach:.3f}")
            print(f"{'t':>6} {'M':>6} {'alt':>6} {'mode':>10} {'thrust':>8} "
                  f"{'drag':>8} {'mass':>6} {'a_g':>7}")
            seen = set()
            for i, s in enumerate(r.states):
                key = round(s.mach, 2)
                if s.mach > 1.15 or key in seen:
                    continue
                seen.add(key)
                w = s.mass_kg * G0_M_PER_S2
                print(f"{s.time_s:6.1f} {s.mach:6.3f} {s.altitude_m:6.0f} "
                      f"{s.mode:>10} {s.thrust_n:8.1f} {s.drag_n:8.1f} "
                      f"{s.mass_kg:6.2f} "
                      f"{(s.thrust_n - s.drag_n) / w:7.3f}")
        return

    if "--wide" in sys.argv:
        spans = [0.70, 0.80, 0.90, 1.00]
        ars = [5.0, 6.0, 7.0, 8.0]
    elif "--refine" in sys.argv:
        spans = [0.62, 0.66, 0.70, 0.74, 0.78, 0.82]
        ars = [3.0, 3.5, 4.0, 4.5, 5.0]
    if "--wide" in sys.argv or "--refine" in sys.argv:
        print(f"{'span':>5} {'AR':>5} {'S':>6} {'trav_g':>7} {'travM':>6} "
              f"{'pow_g':>7} {'powM':>6} {'peakM':>6} {'cut':>5} {'T/W':>6} "
              f"{'fuel':>6} {'safe':>5} {'top_m':>6} {'Dind0':>7} "
              f"{'margin':>7}", flush=True)
        for span in spans:
            for ar in ars:
                r = run_one(d, span, ar, base)
                print(f"{r['span']:5.2f} {r['ar']:5.2f} {r['area']:6.3f} "
                      f"{r['trav']:7.3f} {r['trav_m']:6.3f} "
                      f"{r['powg']:7.3f} {r['pow_m']:6.3f} "
                      f"{r['peak']:6.3f} {str(r['cutoff']):>5} "
                      f"{r['margin_tw']:6.2f} {r['fuel']:6.3f} "
                      f"{str(r['safe']):>5} {r['top']:6.0f} "
                      f"{r['d_ind']:7.1f} {r['mass_margin']:7.2f}",
                      flush=True)
        return

    spans = [0.5325, 0.60, 0.70, 0.80, 0.90, 1.00]
    ars = [1.86, 2.5, 3.0, 3.5, 4.0]
    print(HDR, flush=True)
    rows = [run_one(d, base.span_m, base.aspect_ratio, base)]
    print(fmt(rows[0]) + "   <- V3a baseline", flush=True)
    for span in spans:
        for ar in ars:
            r = run_one(d, span, ar, base)
            rows.append(r)
            print(fmt(r), flush=True)


if __name__ == "__main__":
    main()
