"""How long must the duct be for V2's chamber diameter to sustain?

Chamber length was falsified as the fix (every ratio stays dead). The
discriminator across all cases run so far is chamber diameter against
CYCLE PERIOD -- sustaining engines sit near 12-13 mm/ms, every dead one
near 33 -- because the Damkohler mixing cap ties burn time to chamber
diameter while the acoustic period is set by DUCT LENGTH.

So hold the diameter (the airframe sets it) and lengthen the duct until
the cycle is slow enough for the charge to finish burning in phase.
Reports the vehicle slenderness each option implies, since that is the
constraint the answer has to live inside.
"""
from __future__ import annotations

import json, math, time
from pathlib import Path

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
NOSE_TAIL_DIAMETERS = 3.0        # medium_model's own nose+tail allowance


def main():
    from dataclasses import replace
    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from pulsejet_fp.diagnostics import standard_cycle_plot
    from pulsejet_fp.geometry import EngineGeometry

    D = json.loads(Path("docs/v2_frozen/design.json").read_text())
    c = D["vehicle_candidate"]
    d_body = c["diameter_m"]
    d_ch = 0.95 * d_body
    d_tail = c["throat_diameter_m"]
    rv = reference_valve(); p_a, T_a = ambient(60.0)
    v2_duct = c["chamber_length_m"] + c["throat_length_m"]
    v2_body_len = v2_duct + NOSE_TAIL_DIAMETERS * d_body

    print(f"chamber dia held at {d_ch*1e3:.0f} mm (0.95 x {d_body*1e3:.0f} mm "
          f"body); tail dia {d_tail*1e3:.0f} mm")
    print(f"V2 baseline: duct {v2_duct*1e3:.0f} mm, body "
          f"{v2_body_len*1e3:.0f} mm, slenderness "
          f"{v2_body_len/d_body:.2f}\n")
    print(f"{'duct':>6} {'L_ch':>6} {'tail':>6} {'body':>6} {'fine':>5} "
          f"{'vol_L':>6} {'thrust':>8} {'Hz':>6} {'D/T':>5} {'p/p0':>12} "
          f"{'status':>12}")

    f_ch, f_cone = 0.150 / 0.900, 0.130 / 0.900
    for duct in (1.022, 1.400, 1.800, 2.200, 2.700):
        L_ch, L_cone = f_ch * duct, f_cone * duct
        L_tail = duct - L_ch - L_cone
        geom = EngineGeometry(chamber_diameter=d_ch, chamber_length=L_ch,
                              cone_length=L_cone, tailpipe_diameter=d_tail,
                              tailpipe_length=L_tail)
        s = d_ch / 0.078
        valve = replace(rv, petal_length=rv.petal_length*s,
                        petal_width=rv.petal_width*s,
                        petal_thickness=rv.petal_thickness*s,
                        port_area=rv.port_area*s*s, max_lift=rv.max_lift*s,
                        seat_preload=rv.seat_preload*s)
        intake = IntakeDesign(duct_length=0.060*s, duct_diameter=0.050*s,
                              plenum_volume=5e-5*s**3, orientation="side",
                              bl_momentum_fraction=0.6)
        t0 = time.perf_counter()
        r = pulsejet_thrust(mach=0.15, altitude_m=60.0,
                            gas=reference_gas(phi=1.0), geom=geom,
                            valve=valve, intake=intake,
                            numerics=Numerics(n_cells=200), t_end=1.5,
                            stop_when_converged=True, keep_traces=True)
        rc, rt = 0.5*d_ch, 0.5*d_tail
        vol = (math.pi*rc*rc*L_ch
               + (math.pi*L_cone/3.0)*(rc*rc + rc*rt + rt*rt))
        body = duct + NOSE_TAIL_DIAMETERS * d_body
        dof = (d_ch*1e3)*(r.frequency_hz/1e3) if r.frequency_hz > 0 else 0
        print(f"{duct*1e3:6.0f} {L_ch*1e3:6.0f} {L_tail*1e3:6.0f} "
              f"{body*1e3:6.0f} {body/d_body:5.2f} {vol*1e3:6.1f} "
              f"{r.thrust_n:8.1f} {r.frequency_hz:6.1f} {dof:5.1f} "
              f"{r.p_min_ratio:5.2f}-{r.p_max_ratio:<5.2f} {r.status:>12}"
              f"  ({time.perf_counter()-t0:.0f}s)", flush=True)
        try:
            standard_cycle_plot(r.traces, p_a, T_a,
                str(OUT / f"pulsejet_duct{duct*1e3:.0f}mm.png"),
                title=f"duct {duct*1e3:.0f} mm, chamber dia {d_ch*1e3:.0f} mm"
                      f" -> {r.thrust_n:.1f} N ({r.status})")
        except Exception as exc:
            print(f"   (plot skipped: {exc})")


if __name__ == "__main__":
    main()
