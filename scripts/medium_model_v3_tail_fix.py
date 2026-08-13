"""What tail length does V3 need, and does the tail/diameter rule hold?

Across 11 runs the predictor that separates sustaining from dead is
TAIL LENGTH / CHAMBER DIAMETER, not the duct length or the D/T figure
tried first (V3 is dead at D/T 20.6 where a V2 variant sustained at
20.4). Threshold sits between 3.62 (dead) and 4.66 (alive).

Physically: the tailpipe gas column is the inertia that drives the
cycle -- the "liquid piston" -- so it has to be long relative to the
chamber it breathes from, not merely long.

This extends V3's tail while holding its chamber (diameter AND length,
both frozen design values) and reports the vehicle fineness each option
implies, since that is the constraint the answer has to live inside.
"""
from __future__ import annotations

import math, time
from pathlib import Path

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
NOSE_TAIL_DIAMETERS = 3.0


def main():
    from dataclasses import replace
    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from pulsejet_fp.diagnostics import standard_cycle_plot
    from pulsejet_fp.geometry import EngineGeometry
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    d = load_frozen_design("docs/v3_frozen/design.json")
    spec = spec_from_geometry(d.geometry)
    base = spec.pulsejet_geometry()
    d_body = d.geometry.diameter_m
    d_ch, d_tail = base.chamber_diameter, base.tailpipe_diameter
    L_ch, L_cone = base.chamber_length, base.cone_length
    rv = reference_valve(); p_a, T_a = ambient(60.0)

    print(f"V3: body {d_body*1e3:.0f}, chamber {d_ch*1e3:.0f} x "
          f"{L_ch*1e3:.0f}, cone {L_cone*1e3:.0f}, tail dia {d_tail*1e3:.0f} mm")
    print(f"as-designed tail {base.tailpipe_length*1e3:.0f} mm -> "
          f"tail/D_ch = {base.tailpipe_length/d_ch:.2f} (DEAD)\n")
    print(f"{'tail':>6} {'t/D':>5} {'duct':>6} {'body':>6} {'fine':>5} "
          f"{'thrust':>8} {'Hz':>6} {'p/p0':>12} {'verdict':>9}")

    for ratio in (2.66, 3.5, 4.5, 5.5, 6.5):
        L_tail = ratio * d_ch
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
        duct = L_ch + L_cone + L_tail
        body = duct + NOSE_TAIL_DIAMETERS * d_body
        alive = (r.p_max_ratio - r.p_min_ratio) > 0.25 and r.thrust_n > 50
        print(f"{L_tail*1e3:6.0f} {ratio:5.2f} {duct*1e3:6.0f} {body*1e3:6.0f} "
              f"{body/d_body:5.2f} {r.thrust_n:8.1f} {r.frequency_hz:6.1f} "
              f"{r.p_min_ratio:5.2f}-{r.p_max_ratio:<5.2f} "
              f"{'SUSTAINS' if alive else 'DEAD':>9}"
              f"  ({time.perf_counter()-t0:.0f}s)", flush=True)
        if alive:
            standard_cycle_plot(r.traces, p_a, T_a,
                str(OUT / f"pulsejet_v3_tail{L_tail*1e3:.0f}mm.png"),
                title=f"V3 tail {L_tail*1e3:.0f} mm (t/D {ratio:.1f}) "
                      f"-> {r.thrust_n:.1f} N, fineness {body/d_body:.1f}")


if __name__ == "__main__":
    main()
