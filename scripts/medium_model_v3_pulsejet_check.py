"""Can the V3 duct sustain a pulsejet cycle at all?

Asked BEFORE anything else is modelled on V3 (user directive), because
V2's duct could not: it started, pulsed ~12 times and decayed to a dead
flat line, and the closed-form model that selected it has no acoustic
tuning in it and so cannot tell.

Discriminator established on V2: chamber diameter against cycle period.
Sustaining engines ran ~12-20 mm/ms, every dead one ~25-31. The
mechanism is the Damkohler mixing cap -- burn time scales with chamber
diameter while the acoustic period is set by duct length, so a fat
chamber on a short duct cannot finish burning inside its own cycle.

V3 is slimmer than V2 (225 vs 280 mm body) on a slightly longer duct
(1079 vs 1022 mm), so it should sit closer to the threshold. This
measures where.
"""
from __future__ import annotations

import math, time
from pathlib import Path

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)


def main():
    from dataclasses import replace
    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from pulsejet_fp.diagnostics import standard_cycle_plot
    from pulsejet_fp.geometry import EngineGeometry
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    rv = reference_valve(); p_a, T_a = ambient(60.0)
    print(f"{'design':>6} {'D_ch':>6} {'duct':>6} {'L_ch':>6} {'tail':>6} "
          f"{'vol_L':>6} {'thrust':>8} {'Hz':>6} {'D/T':>5} {'p/p0':>12} "
          f"{'verdict':>10}")

    for tag, path in (("V2", "docs/v2_frozen/design.json"),
                      ("V3", "docs/v3_frozen/design.json")):
        d = load_frozen_design(path)
        spec = spec_from_geometry(d.geometry)
        geom = spec.pulsejet_geometry()
        s = geom.chamber_diameter / 0.078
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
        rc, rt = 0.5*geom.chamber_diameter, 0.5*geom.tailpipe_diameter
        vol = (math.pi*rc*rc*geom.chamber_length
               + (math.pi*geom.cone_length/3.0)*(rc*rc + rc*rt + rt*rt))
        duct = geom.chamber_length + geom.cone_length + geom.tailpipe_length
        dof = (geom.chamber_diameter*1e3)*(r.frequency_hz/1e3)
        alive = (r.p_max_ratio - r.p_min_ratio) > 0.25 and r.thrust_n > 50
        print(f"{tag:>6} {geom.chamber_diameter*1e3:6.0f} {duct*1e3:6.0f} "
              f"{geom.chamber_length*1e3:6.0f} {geom.tailpipe_length*1e3:6.0f} "
              f"{vol*1e3:6.1f} {r.thrust_n:8.1f} {r.frequency_hz:6.1f} "
              f"{dof:5.1f} {r.p_min_ratio:5.2f}-{r.p_max_ratio:<5.2f} "
              f"{'SUSTAINS' if alive else 'DEAD':>10}"
              f"  ({time.perf_counter()-t0:.0f}s)", flush=True)
        standard_cycle_plot(r.traces, p_a, T_a,
            str(OUT / f"pulsejet_{tag.lower()}_asdesigned.png"),
            title=f"{tag} as designed: chamber {geom.chamber_diameter*1e3:.0f} mm"
                  f", duct {duct*1e3:.0f} mm -> {r.thrust_n:.1f} N")


if __name__ == "__main__":
    main()
