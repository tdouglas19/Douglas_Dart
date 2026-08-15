"""Why can't we climb higher and light the ramjet at M 0.60?

The question is really an ALTITUDE BUDGET question, because the pulsejet
cannot accelerate the vehicle to M 0.60 in level flight -- above about
M 0.47 its thrust is below the vehicle's drag, so the only way to gain
that speed is to trade height for it in the dive.

So: how much height does M 0.60 cost, and how much height do we have?

  NEEDED    the kinetic energy to go from top-of-climb Mach to M 0.60,
            expressed as a height, PLUS the height burned paying the
            net drag deficit (D - T) along the dive path.

  AVAILABLE the flame-out ceiling minus the hard floor. The pulsejet has
            an altitude limit -- measured alive at 1250 m and dead at
            1275 m on the 324-cell grid -- and above it there is no
            thrust at all, so climbing higher does not buy a longer dive,
            it buys a glider.

Both numbers come out of measured data, not correlations.
"""
from __future__ import annotations
import json, math, os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from douglas_dart.atmosphere import standard_atmosphere      # noqa: E402
from medium_model.design import load_frozen_design           # noqa: E402
from medium_model.drag_buildup import total_drag_buildup     # noqa: E402
from medium_model.mission import MAX_WET_MASS_KG             # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
G = 9.80665
FLOOR_M = 121.92
# Measured on the 324-cell grid at M 0.35: alive at 1250 m, dead at 1275.
FLAMEOUT_CEILING_M = 1250.0
# FP pulsejet thrust (n_cells 324) along the dive, from v3c_gridconv.json
# and the ceiling study. Linear in Mach over this range to within ~2 N.
PJ_THRUST_N = {0.30: 102.9, 0.35: 104.0, 0.40: 107.0, 0.44: 109.6,
               0.50: 109.6, 0.55: 108.2, 0.60: 106.8}
MASS_KG = 22.0                     # mid-dive, after ~0.4 kg of climb fuel


def thrust_at(mach):
    ks = sorted(PJ_THRUST_N)
    if mach <= ks[0]:
        return PJ_THRUST_N[ks[0]]
    if mach >= ks[-1]:
        return PJ_THRUST_N[ks[-1]]
    for a, b in zip(ks, ks[1:]):
        if a <= mach <= b:
            f = (mach - a) / (b - a)
            return PJ_THRUST_N[a] + f * (PJ_THRUST_N[b] - PJ_THRUST_N[a])
    return PJ_THRUST_N[ks[-1]]


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    print("Why the pulsejet cannot deliver M 0.60\n")

    # ---- 1. where does thrust cross drag? ----------------------------
    print("1. Pulsejet thrust vs vehicle drag at 300 m (FP, n_cells 324)\n")
    print(f"{'M':>6} {'T_pj N':>8} {'D N':>8} {'T-D N':>8} "
          f"{'(T-D)/W g':>10} {'dive to break even':>19}")
    rows = []
    atm = standard_atmosphere(300.0)
    W = MASS_KG * G
    for mach in (0.35, 0.40, 0.44, 0.50, 0.55, 0.60):
        v = mach * atm.speed_of_sound_m_per_s
        try:
            dr = total_drag_buildup(d.geometry, d.wing, mach, 300.0,
                                    MASS_KG)
            drag = float(getattr(dr, "total_n", dr))
        except Exception:
            drag = float("nan")
        T = thrust_at(mach)
        excess = T - drag
        g_excess = excess / W
        need = (math.degrees(math.asin(min(1.0, -g_excess)))
                if g_excess < 0 else 0.0)
        rows.append(dict(mach=mach, thrust_n=T, drag_n=drag,
                         excess_n=excess, excess_g=g_excess,
                         dive_to_break_even_deg=need))
        print(f"{mach:6.2f} {T:8.1f} {drag:8.1f} {excess:8.1f} "
              f"{g_excess:10.3f} "
              f"{(f'{need:.1f} deg' if need else 'level flight ok'):>19}")

    # ---- 2. the height M 0.60 costs ----------------------------------
    print("\n2. Height needed to accelerate to M 0.60 on the pulsejet\n")
    a_sl = standard_atmosphere(400.0).speed_of_sound_m_per_s
    print(f"{'from M':>7} {'dv m/s':>8} {'KE height m':>12} "
          f"{'drag height m':>14} {'TOTAL m':>9}")
    budget = []
    for m0 in (0.27, 0.30, 0.32, 0.35, 0.40):
        v0, v1 = m0 * a_sl, 0.60 * a_sl
        ke_h = (v1 * v1 - v0 * v0) / (2.0 * G)
        # net drag deficit averaged over the speed range, converted to
        # the extra height that must be spent paying it. Path length at
        # dive angle gamma is h/sin(gamma); the sin cancels for the work
        # per unit height only if (D-T) is constant, so integrate crudely
        # over 5 sub-intervals.
        n = 5
        drag_h = 0.0
        for i in range(n):
            mm = m0 + (0.60 - m0) * (i + 0.5) / n
            atm_i = standard_atmosphere(600.0)
            try:
                dr = total_drag_buildup(d.geometry, d.wing, mm, 600.0,
                                        MASS_KG)
                dd = float(getattr(dr, "total_n", dr))
            except Exception:
                dd = float("nan")
            deficit = max(0.0, dd - thrust_at(mm))
            drag_h += (deficit / W) * (ke_h / n)
        total = ke_h + drag_h
        budget.append(dict(from_mach=m0, dv=v1 - v0, ke_height_m=ke_h,
                           drag_height_m=drag_h, total_height_m=total))
        print(f"{m0:7.2f} {v1-v0:8.1f} {ke_h:12.0f} {drag_h:14.0f} "
              f"{total:9.0f}")

    # ---- 3. the height we actually have ------------------------------
    have = FLAMEOUT_CEILING_M - FLOOR_M
    print(f"\n3. Height available\n")
    print(f"   pulsejet altitude flame-out ceiling   {FLAMEOUT_CEILING_M:7.0f} m"
          f"   (alive 1250, DEAD 1275, n_cells 324)")
    print(f"   hard floor                            {FLOOR_M:7.0f} m")
    print(f"   maximum usable drop                   {have:7.0f} m")

    print(f"\n4. Verdict\n")
    for b in budget:
        short = b["total_height_m"] - have
        print(f"   from top-of-climb M {b['from_mach']:.2f}: need "
              f"{b['total_height_m']:5.0f} m, have {have:.0f} m -> "
              + (f"**SHORT BY {short:.0f} m ({100*short/have:.0f}%)**"
                 if short > 0 else f"feasible with {-short:.0f} m spare"))

    (OUT / "v3_why_not_m06.json").write_text(json.dumps(
        dict(thrust_vs_drag=rows, height_budget=budget,
             ceiling_m=FLAMEOUT_CEILING_M, floor_m=FLOOR_M,
             usable_drop_m=have), indent=2))
    print(f"\nwrote {OUT / 'v3_why_not_m06.json'}")


if __name__ == "__main__":
    main()
