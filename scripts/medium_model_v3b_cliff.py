"""Where exactly is the pulsejet sustain cliff at the 235 mm chamber?

The geometry sweep found that chamber 235 mm (the +10% the user allowed)
at tail/chamber-dia 4.00 STRICTLY DOMINATES V3a: body 2269 mm (39 mm
SHORTER than V3a's 2308) at 149.2 N (+21% thrust).  That is the only
point in the whole sweep that beats the current design on both length and
thrust at once, so it is worth taking seriously.

But it sits close to a cliff: the same duct at t/D 3.50 is DEAD (0.9 N,
p/p0 span 0.024 -- it does not oscillate at all), and nothing between
3.50 and 4.00 was sampled.  If the cliff is at 3.9 the design has no
margin; if it is at 3.6 it has plenty.

The earlier "t/D >= 4.5" rule was derived at 214 mm and is evidently
diameter-dependent, so this also tests whether that rule is a rule or
just where that diameter's cliff happened to fall.
"""
from __future__ import annotations

import json, math, time
from dataclasses import replace
from pathlib import Path

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
NOSE_TAIL_DIAMETERS = 3.0
CHAMBER_DIA_M = 0.235          # +9.8% on V3a's 214.0 mm, inside the +10% cap
THROAT_OVER_BODY = 0.5400      # V3a ratio -> area frac 0.2916, under the 0.30 cap
BODY_OVER_CHAMBER = 1.0 / 0.95


def main():
    from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                             reference_gas, reference_valve)
    from pulsejet_fp.atmosphere import ambient
    from pulsejet_fp.diagnostics import standard_cycle_plot
    from pulsejet_fp.geometry import EngineGeometry
    from medium_model.design import load_frozen_design
    from medium_model.fp_spec import spec_from_geometry

    d = load_frozen_design("docs/v3a_medium_model/design.json")
    base = spec_from_geometry(d.geometry).pulsejet_geometry()
    L_ch, L_cone_v3a = base.chamber_length, base.cone_length
    cone_frac = L_cone_v3a / (L_cone_v3a + base.tailpipe_length)

    d_ch = CHAMBER_DIA_M
    d_body = d_ch * BODY_OVER_CHAMBER
    d_tail = d_body * THROAT_OVER_BODY
    try:
        gas = reference_gas(phi=1.0, fuel="propane")
    except TypeError:
        gas = reference_gas(phi=1.0)
    rv = reference_valve(); p_a, T_a = ambient(60.0)

    print(f"chamber {d_ch*1e3:.0f} mm (V3a {base.chamber_diameter*1e3:.0f}, "
          f"{100*(d_ch/base.chamber_diameter-1):+.1f}%) -> body "
          f"{d_body*1e3:.1f} mm, tailpipe dia {d_tail*1e3:.1f} mm")
    print(f"chamber length {L_ch*1e3:.1f} mm held; cone = "
          f"{100*cone_frac:.1f}% of tail section")
    print(f"V3a reference: body {d.geometry.diameter_m*1e3:.1f} mm dia, "
          f"2307.3 mm overall, 123.6 N at M 0.15\n")
    print(f"{'t/D':>5} {'tail':>6} {'cone':>5} {'duct':>6} {'body':>6} "
          f"{'fine':>5} {'thrust':>8} {'Hz':>6} {'p/p0':>12} {'verdict':>9}")

    rows = []
    for ratio in (3.60, 3.70, 3.80, 3.90, 4.00, 4.25):
        L_tail = ratio * d_ch
        L_cone = L_tail * cone_frac / (1.0 - cone_frac)
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
        r = pulsejet_thrust(mach=0.15, altitude_m=60.0, gas=gas, geom=geom,
                            valve=valve, intake=intake,
                            numerics=Numerics(n_cells=200), t_end=1.5,
                            stop_when_converged=True, keep_traces=True)
        duct = L_ch + L_cone + L_tail
        body = duct + NOSE_TAIL_DIAMETERS * d_body
        alive = (r.p_max_ratio - r.p_min_ratio) > 0.25 and r.thrust_n > 50
        rows.append(dict(t_over_d=ratio, tail_mm=L_tail*1e3,
                         cone_mm=L_cone*1e3, duct_mm=duct*1e3,
                         body_mm=body*1e3, fineness=body/d_body,
                         thrust_n=r.thrust_n, freq_hz=r.frequency_hz,
                         p_min=r.p_min_ratio, p_max=r.p_max_ratio,
                         sustains=bool(alive)))
        print(f"{ratio:5.2f} {L_tail*1e3:6.0f} {L_cone*1e3:5.0f} "
              f"{duct*1e3:6.0f} {body*1e3:6.0f} {body/d_body:5.2f} "
              f"{r.thrust_n:8.1f} {r.frequency_hz:6.1f} "
              f"{r.p_min_ratio:5.2f}-{r.p_max_ratio:<5.2f} "
              f"{'SUSTAINS' if alive else 'DEAD':>9}"
              f"  ({time.perf_counter()-t0:.0f}s)", flush=True)
        standard_cycle_plot(
            r.traces, p_a, T_a,
            str(OUT / f"pulsejet_ch235_tD{ratio:.2f}.png"),
            title=f"chamber 235 mm, t/D {ratio:.2f}, "
                  f"{r.thrust_n:.0f} N @ {r.frequency_hz:.0f} Hz")
        (OUT / "v3b_cliff.json").write_text(json.dumps(rows, indent=2))

    alive = [q for q in rows if q["sustains"]]
    dead = [q for q in rows if not q["sustains"]]
    print()
    if alive:
        best = min(alive, key=lambda q: q["body_mm"])
        print(f"shortest SUSTAINING: t/D {best['t_over_d']:.2f} -> body "
              f"{best['body_mm']:.0f} mm, {best['thrust_n']:.1f} N")
        print(f"  vs V3a (2307 mm, 123.6 N): "
              f"{best['body_mm']-2307.3:+.0f} mm length, "
              f"{100*(best['thrust_n']/123.6-1):+.0f}% thrust")
        if dead:
            print(f"cliff bracketed between t/D "
                  f"{max(q['t_over_d'] for q in dead):.2f} (dead) and "
                  f"{best['t_over_d']:.2f} (alive)")
        else:
            print(f"no dead point in 3.60-4.25: the cliff is below t/D 3.60, "
                  f"so t/D 4.00 has >= 10% margin")
    else:
        print("nothing in 3.60-4.25 sustains at 235 mm -- the 4.00 point "
              "from the earlier sweep does not reproduce; investigate")
    print(f"\nwrote {OUT / 'v3b_cliff.json'}")


if __name__ == "__main__":
    main()
