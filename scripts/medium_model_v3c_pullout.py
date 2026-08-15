"""What does the dive pull-out actually COST? (the model charges nothing)

`flight_sim.run_flight` latches `v3_dive_done` and switches gamma from the
dive angle to the drag-strip angle in ONE TIMESTEP.  There is no pull-out
arc, no load factor, and no g limit anywhere in the phase machine.  The
trajectory is therefore free to command a -20 deg to +1 deg flight-path
change instantaneously, and every traverse number in this campaign is
computed against a profile that does that.

That matters here more than it did for V3b, because the latest-ignition
result LEANS on the steep dive: gate 0.44 is reachable only because a
20 deg dive gets the vehicle to M 0.44 while it is still descending.  If
the steep dive is not flyable, the late gate is not real.

This prices it two ways and compares:

  DEMANDED   a pull-out from gamma to level, executed within an altitude
             allowance dh, needs radius R = dh / (1 - cos gamma) and load
             factor n = 1 + V^2 / (g R).  Since the floor is hard at
             121.9 m, dh is whatever altitude is conceded to the arc.

  AVAILABLE  n_max = q S CL_max / W, with CL_max from medium_model.lift --
             which is ALPHA-limited (0.849) for this low-AR wing, not the
             legacy 1.135.  That choice roughly flips the verdict, so both
             are reported.

Structural load is NOT modelled anywhere in the repo; the wet mass is
22.68 kg, so n g means 22.68*n*9.81 newtons through the wing joint.
"""
from __future__ import annotations
import json, math, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from douglas_dart.atmosphere import standard_atmosphere      # noqa: E402
from medium_model.design import load_frozen_design           # noqa: E402
from medium_model.mission import MAX_WET_MASS_KG             # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
G = 9.80665
FLOOR_M = 121.92
DIVES = (9.89, 14.0, 16.0, 18.0, 20.0, 25.0, 30.0)
# Altitude conceded to the pull-out arc. The floor is hard, so this is
# altitude the dive does NOT get to use -- it comes out of the dive, not
# out of the floor.
ALLOWANCES_M = (15.0, 30.0, 60.0)


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    S = d.wing.reference_area_m2
    W = MAX_WET_MASS_KG * G

    try:
        from medium_model.lift import wing_clmax
        res = wing_clmax(1.20, d.wing.aspect_ratio, d.wing.taper_ratio,
                         d.wing.sweep_deg, 0.60)
        cl_alpha_limited = float(res.clmax)
        src = ("medium_model.lift.wing_clmax, "
               + str(getattr(res, "mechanism", "alpha-limited")))
    except Exception as exc:                       # pragma: no cover
        cl_alpha_limited = 0.849
        src = f"fallback 0.849 ({type(exc).__name__}: {exc})"
    cl_legacy = d.wing.cl_max_effective

    print(f"V3a wing: S = {S:.4f} m2, AR {d.wing.aspect_ratio:.3f}, "
          f"wet mass {MAX_WET_MASS_KG:.2f} kg -> W = {W:.1f} N")
    print(f"CL_max: {cl_alpha_limited:.3f} from {src}")
    print(f"        {cl_legacy:.3f} from the legacy drag.py model")
    print(f"floor {FLOOR_M:.1f} m; pull-out happens at the bottom of the "
          f"dive\n")

    # Mach at the bottom of the dive, from the FP flights that reached it.
    # (dive exit is where the floor terminates the dive in 100% of flights)
    exit_mach = {9.89: 0.60, 14.0: 0.60, 16.0: 0.60, 18.0: 0.60,
                 20.0: 0.60, 25.0: 0.60, 30.0: 0.60}

    atm = standard_atmosphere(FLOOR_M)
    rho = atm.density_kg_per_m3
    a = atm.speed_of_sound_m_per_s
    rows = []
    print(f"{'dive':>6} {'V m/s':>7} {'q Pa':>8} "
          + "".join(f"{'n@'+str(int(x))+'m':>9}" for x in ALLOWANCES_M)
          + f" {'n_avail':>8} {'n_legacy':>9}  verdict")
    for dive in DIVES:
        M = exit_mach[dive]
        V = M * a
        q = 0.5 * rho * V * V
        n_avail = q * S * cl_alpha_limited / W
        n_leg = q * S * cl_legacy / W
        demanded = []
        for dh in ALLOWANCES_M:
            g_rad = math.radians(dive)
            R = dh / (1.0 - math.cos(g_rad))
            demanded.append(1.0 + V * V / (G * R))
        worst = demanded[0]
        ok = n_avail >= min(demanded)
        rows.append(dict(dive_deg=dive, exit_mach=M, V_m_s=V, q_pa=q,
                         n_demanded={f"{x:.0f}m": v for x, v
                                     in zip(ALLOWANCES_M, demanded)},
                         n_available=n_avail, n_available_legacy=n_leg,
                         flyable_at_some_allowance=bool(ok)))
        print(f"{dive:6.1f} {V:7.1f} {q:8.0f} "
              + "".join(f"{v:9.1f}" for v in demanded)
              + f" {n_avail:8.1f} {n_leg:9.1f}  "
              + ("ok at >=60 m" if n_avail >= demanded[-1]
                 else "ok only with a generous arc" if ok
                 else "NOT FLYABLE"))

    (OUT / "v3c_pullout.json").write_text(json.dumps(
        dict(wing_area_m2=S, weight_n=W, clmax_alpha_limited=cl_alpha_limited,
             clmax_legacy=cl_legacy, clmax_source=src,
             floor_m=FLOOR_M, rows=rows), indent=2))

    print(f"\nStructural load through the wing joint at the demanded g "
          f"(22.68 kg wet):")
    for dh, lbl in zip(ALLOWANCES_M, ALLOWANCES_M):
        g_rad = math.radians(20.0)
        R = dh / (1.0 - math.cos(g_rad))
        V = 0.60 * a
        n = 1.0 + V * V / (G * R)
        print(f"  dive 20 deg, {dh:.0f} m arc -> {n:.1f} g = "
              f"{MAX_WET_MASS_KG*n*G:.0f} N "
              f"({MAX_WET_MASS_KG*n*G/4.448:.0f} lbf)")
    print("\nNOTE: the flight model charges NONE of this. It switches gamma "
          "in one timestep.\nStructural capability is not modelled anywhere "
          "in the repo.")
    print(f"\nwrote {OUT / 'v3c_pullout.json'}")


if __name__ == "__main__":
    main()
