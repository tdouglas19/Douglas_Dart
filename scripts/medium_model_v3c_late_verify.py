"""FP-verify the latest-ignition candidates the closed-form screen found.

Closed-form runs ~25% optimistic on traverse against FP (V3b: 0.481 vs
0.376 at climb 10 / gate 0.30), and it disagreed with FP on ORDERING once
already -- the wing sweep.  So the screen ranks, FP decides.

Because of that 25% gap, candidates whose closed-form traverse is barely
over 0.26 will not survive; this takes the ones with real headroom and
walks the gate DOWN from the latest until FP passes, which finds the true
latest-ignition point rather than assuming the screen's answer.

Reads out_medium_model/v3c_latest_screen.json; writes v3c_late_verify.json.
"""
from __future__ import annotations
import json, os, sys
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402
from medium_model.fp_propulsion import FpPropulsion          # noqa: E402
from medium_model.fp_spec import spec_from_geometry          # noqa: E402

OUT = Path("out_medium_model"); OUT.mkdir(exist_ok=True)
TARGET_G = 0.26
FLOOR_M = 122.0
# Explicit candidates: (climb, dive, gate). Chosen to walk the gate down
# from the screen's latest, at the dive angles the screen liked.
# Ordered latest-first, and within a gate by closed-form traverse, so the
# answer to "how late can it light" arrives before the refinements do.
# The 0.50 probe is expected to fail -- it failed at every one of the 36
# climb/dive combinations in closed form, and FP is harsher -- but the
# ceiling is worth one flight to document rather than infer.
CANDIDATES = [
    (14.0, 20.0, 0.50),     # ceiling probe
    (14.0, 20.0, 0.45),     # best closed-form at the latest passing gate
    (12.0, 20.0, 0.45),
    (10.0, 20.0, 0.45),
    (14.0, 18.0, 0.45),
    (16.0, 20.0, 0.45),
    (14.0, 16.0, 0.45),
    (14.0, 20.0, 0.42),     # fallback if 0.45 does not survive FP
    (14.0, 20.0, 0.40),
    (12.0, 18.0, 0.42),
]


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    spec = spec_from_geometry(d.geometry)
    cands = CANDIDATES
    if "--from-screen" in sys.argv:
        scr = json.loads((OUT / "v3c_latest_screen.json").read_text())
        ok = [q for q in scr if q["passes"]]
        # Take the latest gate, then the best few by closed-form traverse,
        # then step the gate down one notch and repeat -- FP will lose
        # ~25%, so the screen's marginal passes are not worth an FP hour.
        cands = []
        for gate in sorted({q["gate"] for q in ok}, reverse=True)[:3]:
            at = sorted((q for q in ok if q["gate"] == gate),
                        key=lambda q: -q["traverse"])[:3]
            cands += [(q["climb"], q["dive"], q["gate"]) for q in at]

    print(f"V3a body + frozen wing, FP propulsion, floor {FLOOR_M} m")
    print(f"{len(cands)} candidates, latest ignition first\n")
    print(f"{'climb':>6} {'dive':>6} {'gate':>5} {'trav_g':>7} {'pow_g':>7} "
          f"{'margin':>7} {'fuel':>6} {'peakM':>6} {'cut':>5} {'runs':>5} "
          f" gate  lights at")
    rows = []
    for climb, dive, gate in cands:
        cd = ClimbDiveProfile(initial_climb_angle_deg=climb,
                              dive_angle_deg=dive, floor_altitude_m=FLOOR_M)
        fp = FpPropulsion(spec, fuel="propane", lightoff_mach=gate,
                          n_cells=162)
        r = fly(d, drag_model="buildup", climb_dive=cd, propulsion=fp)
        peak = max(s.mach for s in r.states)
        fuel = max(s.fuel_burned_kg for s in r.states)
        ev = [f"{t:.1f}s {w}" for t, w in fp.trace.events]
        lit = next((e for e in ev if "ramjet_lit" in e), "-")
        ok = (r.motor_cutoff_reached and peak >= 1.0
              and r.min_traverse_accel_g >= TARGET_G
              and fuel < d.burn_limit_kg - 1e-6)
        rows.append(dict(climb=climb, dive=dive, gate=gate,
                         traverse=r.min_traverse_accel_g,
                         powered=r.min_powered_accel_g,
                         margin=r.min_powered_thrust_margin,
                         fuel_kg=fuel, peak_mach=peak,
                         cutoff=bool(r.motor_cutoff_reached),
                         fp_runs=fp.n_transients, passes=bool(ok),
                         events=ev))
        print(f"{climb:6.1f} {dive:6.1f} {gate:5.2f} "
              f"{r.min_traverse_accel_g:7.3f} {r.min_powered_accel_g:7.3f} "
              f"{r.min_powered_thrust_margin:7.3f} {fuel:6.3f} {peak:6.3f} "
              f"{str(r.motor_cutoff_reached):>5} {fp.n_transients:5d} "
              f" {'PASS' if ok else 'fail'}  {lit}", flush=True)
        (OUT / "v3c_late_verify.json").write_text(json.dumps(rows, indent=2))

    good = [q for q in rows if q["passes"]]
    print(f"\n{len(good)}/{len(rows)} pass under FP")
    if good:
        latest = max(q["gate"] for q in good)
        print(f"LATEST FP-VERIFIED IGNITION: M {latest:.2f}")
        for q in sorted((z for z in good if z["gate"] == latest),
                        key=lambda z: -z["traverse"]):
            print(f"  climb {q['climb']:.0f} / dive {q['dive']:.0f} / "
                  f"gate {q['gate']:.2f}: traverse {q['traverse']:.3f} g, "
                  f"powered {q['powered']:.3f}, margin {q['margin']:.3f}, "
                  f"fuel {q['fuel_kg']:.3f} kg")
    print(f"\nwrote {OUT / 'v3c_late_verify.json'}")


if __name__ == "__main__":
    main()
