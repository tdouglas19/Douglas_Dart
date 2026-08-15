"""Is CHAMBER DIAMETER the binding constraint, not chamber length?

The length sweep falsified the length hypothesis: even at FP-1's exact
tail/chamber ratio the engine stays dead. But across every case run so
far, the sustaining engines share a number and the dead ones miss it:

    chamber diameter / cycle period      FP-1  12 mm/ms   (sustains)
                                    isometric  13 mm/ms   (sustains)
                                    V2 sweeps  33 mm/ms   (all dead)

Mechanism: the Damkohler mixing cap makes burn time scale with chamber
diameter (l_m = 0.35 D), while the acoustic period is set by duct LENGTH.
A fat chamber on a short duct cannot finish burning inside its own cycle,
so heat release lands out of phase and damps the oscillation.

Prediction: at the ~110 Hz these ducts ring at, a sustaining chamber
wants D ~ 112 mm. The duct does NOT have to fill the 280 mm body -- it
can be slim, with the annulus carrying fuel (which is where V2 already
puts it, around the tailpipe).

Sweep chamber diameter at fixed duct budget and FP-1's length ratio.
"""
from __future__ import annotations

import json, math, time
from pathlib import Path

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)


def main():
    from dataclasses import replace
    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from pulsejet_fp.diagnostics import standard_cycle_plot
    from pulsejet_fp.geometry import EngineGeometry

    D = json.loads(Path("docs/v2_frozen/design.json").read_text())
    c = D["vehicle_candidate"]
    duct = c["chamber_length_m"] + c["throat_length_m"]
    rv = reference_valve(); p_a, T_a = ambient(60.0)

    # FP-1 length proportions inside the vehicle's duct budget
    f_ch, f_cone = 0.150 / 0.900, 0.130 / 0.900
    L_ch, L_cone = f_ch * duct, f_cone * duct
    L_tail = duct - L_ch - L_cone
    print(f"duct {duct*1e3:.0f} mm -> chamber {L_ch*1e3:.0f}, cone "
          f"{L_cone*1e3:.0f}, tail {L_tail*1e3:.0f} mm (FP-1 proportions)")
    print(f"body dia {c['diameter_m']*1e3:.0f} mm is the CEILING, not a "
          f"requirement -- the duct may be slimmer\n")
    print(f"{'D_ch':>6} {'D_tail':>7} {'vol_L':>6} {'thrust':>8} {'Hz':>6} "
          f"{'D/T':>6} {'p/p0':>12} {'status':>12}")

    for d_ch in (0.090, 0.112, 0.140, 0.180, 0.230, 0.266):
        d_tail = (0.042 / 0.078) * d_ch          # FP-1 tail:chamber dia
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
                            numerics=Numerics(n_cells=162), t_end=1.2,
                            stop_when_converged=True, keep_traces=True)
        rc, rt = 0.5*d_ch, 0.5*d_tail
        vol = (math.pi*rc*rc*L_ch
               + (math.pi*L_cone/3.0)*(rc*rc + rc*rt + rt*rt))
        dof = (d_ch*1e3)*(r.frequency_hz/1e3) if r.frequency_hz > 0 else 0
        print(f"{d_ch*1e3:6.0f} {d_tail*1e3:7.0f} {vol*1e3:6.1f} "
              f"{r.thrust_n:8.1f} {r.frequency_hz:6.1f} {dof:6.1f} "
              f"{r.p_min_ratio:5.2f}-{r.p_max_ratio:<5.2f} {r.status:>12}"
              f"  ({time.perf_counter()-t0:.0f}s)", flush=True)
        try:
            standard_cycle_plot(r.traces, p_a, T_a,
                str(OUT / f"pulsejet_dia{d_ch*1e3:.0f}mm.png"),
                title=f"chamber dia {d_ch*1e3:.0f} mm in {duct*1e3:.0f} mm "
                      f"duct -> {r.thrust_n:.1f} N")
        except Exception as exc:
            print(f"   (plot skipped: {exc})")


if __name__ == "__main__":
    main()
