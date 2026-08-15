"""Why is the V2-proportioned pulsejet dead?

Two candidates, and they need separating before anything is concluded:
  (a) V2's DUCT PROPORTIONS are not a workable pulsejet (fat and short:
      3.41x FP-1's chamber diameter but only 1.10x its tailpipe length,
      so the chamber/tailpipe acoustic tuning that drives the cycle is
      gone), or
  (b) my geometry/valve MAPPING is wrong.

Test: fly the same chamber VOLUME two ways -- once with V2's actual
proportions, once isometrically scaled from the validated FP-1 shape
(the approach the Douglas Dart bridge uses and that ran at 2.84x). If
isometric fires and V2's proportions do not, the geometry is the finding.

Emits the standing two-panel diagnostic (p/p0 with valve/exit velocities,
and T/T0) for every attempt.
"""
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.fp_spec import MediumModelFpSpec  # noqa: E402

OUT = Path("out_medium_model")
OUT.mkdir(exist_ok=True)


def fp1_chamber_zone_volume(geom) -> float:
    """Chamber cylinder + cone frustum, the pulsejet-fp 'chamber zone'."""
    r_c = 0.5 * geom.chamber_diameter
    r_t = 0.5 * geom.tailpipe_diameter
    cyl = math.pi * r_c * r_c * geom.chamber_length
    frustum = (math.pi * geom.cone_length / 3.0) * (r_c * r_c + r_c * r_t
                                                    + r_t * r_t)
    return cyl + frustum


def run(tag, geom, valve, intake, mach, n_cells, t_end=1.2):
    from pulsejet_fp import (Numerics, pulsejet_thrust, reference_gas)

    t0 = time.perf_counter()
    res = pulsejet_thrust(
        mach=mach, altitude_m=60.0, gas=reference_gas(phi=1.0),
        geom=geom, valve=valve, intake=intake,
        numerics=Numerics(n_cells=n_cells), t_end=t_end,
        stop_when_converged=True, keep_traces=True)
    dt = time.perf_counter() - t0
    print(f"{tag:26s} M{mach}: {res.thrust_n:8.1f} N  {res.status:12s} "
          f"{res.frequency_hz:6.1f} Hz  fuel {res.mdot_fuel_kg_s:.4f} kg/s "
          f" p/p0 {res.p_min_ratio:.2f}-{res.p_max_ratio:.2f}  ({dt:.0f}s)",
          flush=True)
    try:
        from pulsejet_fp.atmosphere import ambient
        from pulsejet_fp.diagnostics import standard_cycle_plot
        p_a, T_a = ambient(60.0)
        standard_cycle_plot(res.traces, p_a, T_a,
                            str(OUT / f"pulsejet_{tag}.png"),
                            title=f"{tag}  M={mach}  {res.status}")
    except Exception as exc:
        print(f"   (plot skipped: {type(exc).__name__}: {exc})")
    return res


def main():
    D = json.loads(Path("docs/v2_frozen/design.json").read_text())
    c = D["vehicle_candidate"]
    spec = MediumModelFpSpec(c["diameter_m"], c["throat_diameter_m"],
                             c["chamber_length_m"], c["throat_length_m"])
    from pulsejet_fp import IntakeDesign, reference_valve
    from pulsejet_fp.geometry import EngineGeometry

    v2_geom = spec.pulsejet_geometry()
    v2_vol = fp1_chamber_zone_volume(v2_geom)

    fp1 = EngineGeometry(chamber_diameter=0.078, chamber_length=0.150,
                         cone_length=0.130, tailpipe_diameter=0.042,
                         tailpipe_length=0.620)
    s = (v2_vol / fp1_chamber_zone_volume(fp1)) ** (1.0 / 3.0)
    iso_geom = EngineGeometry(
        chamber_diameter=0.078 * s, chamber_length=0.150 * s,
        cone_length=0.130 * s, tailpipe_diameter=0.042 * s,
        tailpipe_length=0.620 * s)

    print(f"V2 chamber-zone volume {v2_vol * 1e3:.2f} L  -> isometric "
          f"scale s = {s:.2f} vs FP-1")
    print(f"  V2 shape : chamber {v2_geom.chamber_diameter*1e3:.0f} x "
          f"{v2_geom.chamber_length*1e3:.0f}, tail "
          f"{v2_geom.tailpipe_diameter*1e3:.0f} x "
          f"{v2_geom.tailpipe_length*1e3:.0f} mm")
    print(f"  isometric: chamber {iso_geom.chamber_diameter*1e3:.0f} x "
          f"{iso_geom.chamber_length*1e3:.0f}, tail "
          f"{iso_geom.tailpipe_diameter*1e3:.0f} x "
          f"{iso_geom.tailpipe_length*1e3:.0f} mm\n")

    def valve_for(scale):
        from dataclasses import replace
        rv = reference_valve()
        return replace(rv, petal_length=rv.petal_length * scale,
                       petal_width=rv.petal_width * scale,
                       petal_thickness=rv.petal_thickness * scale,
                       port_area=rv.port_area * scale * scale,
                       max_lift=rv.max_lift * scale,
                       seat_preload=rv.seat_preload * scale)

    def intake_for(scale):
        return IntakeDesign(duct_length=0.060 * scale,
                            duct_diameter=0.050 * scale,
                            plenum_volume=5e-5 * scale ** 3,
                            orientation="side", bl_momentum_fraction=0.6)

    mach = 0.15
    # (a) V2's own proportions, valve scaled on chamber diameter
    run("v2_proportions_N162", v2_geom, valve_for(v2_geom.chamber_diameter / 0.078),
        intake_for(v2_geom.chamber_diameter / 0.078), mach, 162)
    # (b) same volume, FP-1 shape, isometric everything
    run("isometric_N162", iso_geom, valve_for(s), intake_for(s), mach, 162)
    # (c) grid check on the isometric case -- is 162 cells enough for a
    #     duct this long? (flame front ~5-10 mm vs dx)
    run("isometric_N300", iso_geom, valve_for(s), intake_for(s), mach, 300)


if __name__ == "__main__":
    main()


def discriminate():
    """Is the V2 shape's failure ACOUSTIC (cannot resonate) or a VALVE
    self-start failure (cannot crack the reeds to admit first charge)?

    Signature to explain: zero fuel admitted and only a +-2% pressure
    ripple. If easing the valve (lower preload / more port area) lights
    it, the shape is fine and the valve pack was mis-sized. If it stays
    dead however easy the valve is, the duct genuinely will not pulse.
    """
    import json
    from dataclasses import replace
    from pathlib import Path

    from pulsejet_fp import IntakeDesign, reference_valve
    from medium_model.fp_spec import MediumModelFpSpec

    D = json.loads(Path("docs/v2_frozen/design.json").read_text())
    c = D["vehicle_candidate"]
    spec = MediumModelFpSpec(c["diameter_m"], c["throat_diameter_m"],
                             c["chamber_length_m"], c["throat_length_m"])
    geom = spec.pulsejet_geometry()
    s = geom.chamber_diameter / 0.078
    rv = reference_valve()

    def valve(scale, preload_scale=1.0, port_scale=1.0):
        return replace(rv, petal_length=rv.petal_length * scale,
                       petal_width=rv.petal_width * scale,
                       petal_thickness=rv.petal_thickness * scale,
                       port_area=rv.port_area * scale * scale * port_scale,
                       max_lift=rv.max_lift * scale,
                       seat_preload=rv.seat_preload * scale * preload_scale)

    intake = IntakeDesign(duct_length=0.060 * s, duct_diameter=0.050 * s,
                          plenum_volume=5e-5 * s ** 3, orientation="side",
                          bl_momentum_fraction=0.6)
    print("\n=== V2 shape: valve or acoustics? ===")
    run("v2_preload_half", geom, valve(s, preload_scale=0.5), intake, 0.15, 162)
    run("v2_preload_zero", geom, valve(s, preload_scale=0.0), intake, 0.15, 162)
    run("v2_port_double", geom, valve(s, preload_scale=0.5, port_scale=2.0),
        intake, 0.15, 162)


if __name__ == "__main__":
    import sys
    if "--discriminate" in sys.argv:
        discriminate()
