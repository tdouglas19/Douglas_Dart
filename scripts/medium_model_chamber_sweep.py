"""Can shortening the chamber fix V2's pulsejet acoustics, within the
airframe's existing duct length?

The failure (docs/design_convergence.md, 2026-08-13) is that V2's duct
starts and then decays: a 680 mm tail on a 266 mm chamber does not return
the compression wave in phase with heat release. FP-1's working shape has
tail/chamber_length = 4.13; V2's is 1.99.

The vehicle cannot grow, so the DUCT LENGTH BUDGET is held fixed at V2's
own chamber + throat (1022 mm) and chamber length is traded against tail
length inside it. Chamber DIAMETER stays at V2's 266 mm -- that is what
carries the volume and hence the thrust, and it is set by the airframe.

Sweeping chamber length therefore sweeps the acoustic ratio while keeping
the engine inside the vehicle it has to fit.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)

CONE_FRACTION_OF_CHAMBER = 0.130 / 0.150     # FP-1's cone : chamber


def chamber_zone_volume(geom) -> float:
    r_c, r_t = 0.5 * geom.chamber_diameter, 0.5 * geom.tailpipe_diameter
    return (math.pi * r_c * r_c * geom.chamber_length
            + (math.pi * geom.cone_length / 3.0)
            * (r_c * r_c + r_c * r_t + r_t * r_t))


def main():
    from dataclasses import replace

    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from pulsejet_fp.diagnostics import standard_cycle_plot
    from pulsejet_fp.geometry import EngineGeometry

    D = json.loads(Path("docs/v2_frozen/design.json").read_text())
    c = D["vehicle_candidate"]
    d_chamber = 0.95 * c["diameter_m"]
    d_tail = c["throat_diameter_m"]
    duct_budget = c["chamber_length_m"] + c["throat_length_m"]
    rv = reference_valve()
    p_a, T_a = ambient(60.0)

    print(f"duct budget {duct_budget*1e3:.0f} mm (V2 chamber+throat), "
          f"chamber dia {d_chamber*1e3:.0f} mm, tail dia {d_tail*1e3:.0f} mm")
    print(f"FP-1 reference ratio tail/chamber_length = "
          f"{0.620/0.150:.2f}; V2 as-built = "
          f"{c['throat_length_m']/c['chamber_length_m']:.2f}\n")
    print(f"{'L_ch':>6} {'cone':>6} {'tail':>6} {'t/c':>5} {'vol_L':>6} "
          f"{'thrust':>8} {'Hz':>6} {'p/p0':>12} {'status':>12}")

    for L in (0.342, 0.280, 0.220, 0.170, 0.130, 0.100):
        cone = CONE_FRACTION_OF_CHAMBER * L
        tail = duct_budget - L - cone
        if tail <= 0.05:
            continue
        geom = EngineGeometry(chamber_diameter=d_chamber, chamber_length=L,
                              cone_length=cone, tailpipe_diameter=d_tail,
                              tailpipe_length=tail)
        s = d_chamber / 0.078
        valve = replace(rv, petal_length=rv.petal_length * s,
                        petal_width=rv.petal_width * s,
                        petal_thickness=rv.petal_thickness * s,
                        port_area=rv.port_area * s * s,
                        max_lift=rv.max_lift * s,
                        seat_preload=rv.seat_preload * s)
        intake = IntakeDesign(duct_length=0.060 * s, duct_diameter=0.050 * s,
                              plenum_volume=5e-5 * s ** 3,
                              orientation="side", bl_momentum_fraction=0.6)
        t0 = time.perf_counter()
        res = pulsejet_thrust(mach=0.15, altitude_m=60.0,
                              gas=reference_gas(phi=1.0), geom=geom,
                              valve=valve, intake=intake,
                              numerics=Numerics(n_cells=162), t_end=1.2,
                              stop_when_converged=True, keep_traces=True)
        dt = time.perf_counter() - t0
        vol = chamber_zone_volume(geom)
        print(f"{L*1e3:6.0f} {cone*1e3:6.0f} {tail*1e3:6.0f} "
              f"{tail/L:5.2f} {vol*1e3:6.1f} {res.thrust_n:8.1f} "
              f"{res.frequency_hz:6.1f} "
              f"{res.p_min_ratio:5.2f}-{res.p_max_ratio:<5.2f} "
              f"{res.status:>12}  ({dt:.0f}s)", flush=True)
        try:
            standard_cycle_plot(
                res.traces, p_a, T_a,
                str(OUT / f"pulsejet_chamber{L*1e3:.0f}mm.png"),
                title=f"chamber {L*1e3:.0f} mm, tail {tail*1e3:.0f} mm "
                      f"(t/c {tail/L:.2f}) -> {res.thrust_n:.1f} N")
        except Exception as exc:
            print(f"   (plot skipped: {exc})")


if __name__ == "__main__":
    main()
