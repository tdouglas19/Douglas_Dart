"""Do the V3b levers compose?

Three independent agents each found a lever, each measured against the
*baseline* of the other two:

  trajectory : climb 16.657 -> 15.5 deg, dive 9.890 -> 20 deg,
               floor 133.14 -> 122 m           (traverse 0.167 -> 0.385)
  wing       : span 0.5325 -> 0.78 m, AR 1.86 -> 4.5
                                                (traverse 0.167 -> 0.223)
  lightoff   : ramjet lights at M 0.25, not the configured M 0.45
               (FP only -- the closed-form gate is bound at import)

They cannot simply add: BOTH the wing lever and the trajectory lever were
credited to the same pinch, the M 0.45 dive trough just before ramjet
lightoff.  If they are the same lever wearing two hats, the combination
buys nothing over the better of the two.

Closed-form propulsion, drag build-up -- seconds per flight, so this
screens the composition before spending FP time on it.
"""
from __future__ import annotations
import os

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

from medium_model.design import fly, load_frozen_design      # noqa: E402
from medium_model.drag import WingConcept                    # noqa: E402
from medium_model.flight_sim import ClimbDiveProfile         # noqa: E402

BASE_TRAJ = None            # filled from the frozen design
BEST_TRAJ = (15.5, 20.0, 122.0)
BASE_WING = None
BEST_WING = (0.78, 4.5)


def wing_of(design, span_m, aspect_ratio):
    w = design.wing
    return WingConcept(span_m=span_m, aspect_ratio=aspect_ratio,
                       taper_ratio=w.taper_ratio, sweep_deg=w.sweep_deg,
                       airfoil=w.airfoil)


def run(design, traj, wing, label, rows):
    cd = ClimbDiveProfile(initial_climb_angle_deg=traj[0],
                          dive_angle_deg=traj[1], floor_altitude_m=traj[2])
    r = fly(design, drag_model="buildup", climb_dive=cd,
            wing_concept=wing)
    peak = max(s.mach for s in r.states)
    fuel = max(s.fuel_burned_kg for s in r.states)
    rows.append(dict(
        label=label, trav=r.min_traverse_accel_g,
        pow=r.min_powered_accel_g, peak=peak,
        cutoff=r.motor_cutoff_reached,
        margin=r.min_powered_thrust_margin, fuel=fuel,
        stalled=getattr(r, "stalled", None),
        span=wing.span_m, ar=wing.aspect_ratio,
        climb=traj[0], dive=traj[1], floor=traj[2]))
    q = rows[-1]
    print(f"{label:<34} {q['trav']:7.3f} {q['pow']:8.3f} {peak:6.3f} "
          f"{str(q['cutoff']):>6} {q['margin']:7.3f} {fuel:6.3f}",
          flush=True)
    return rows[-1]


def main():
    d = load_frozen_design("docs/v3a_medium_model/design.json")
    base_traj = (d.climb_dive.initial_climb_angle_deg,
                 d.climb_dive.dive_angle_deg,
                 d.climb_dive.floor_altitude_m)
    base_wing = wing_of(d, d.wing.span_m, d.wing.aspect_ratio)
    best_wing = wing_of(d, *BEST_WING)

    print(f"V3a body ({d.geometry.diameter_m*1000:.0f} mm dia), "
          f"closed-form propulsion, drag build-up")
    print(f"baseline trajectory {base_traj[0]:.3f}/{base_traj[1]:.3f}/"
          f"{base_traj[2]:.2f}, wing {base_wing.span_m:.4f}/"
          f"{base_wing.aspect_ratio:.2f}\n")
    print(f"{'config':<34} {'trav_g':>7} {'pow_g':>8} {'peakM':>6} "
          f"{'cutoff':>6} {'margin':>7} {'fuel':>6}")
    print("-" * 80)

    rows = []
    run(d, base_traj, base_wing, "baseline", rows)
    run(d, BEST_TRAJ, base_wing, "+trajectory", rows)
    run(d, base_traj, best_wing, "+wing", rows)
    both = run(d, BEST_TRAJ, best_wing, "+trajectory +wing", rows)

    # If they overlap, back the dive off and see how much of the 20 deg
    # is actually needed once the wing is carrying its share.
    print()
    for dive in (18.0, 16.0, 14.0, 12.0, 10.0):
        run(d, (BEST_TRAJ[0], dive, BEST_TRAJ[2]), best_wing,
            f"  wing + dive {dive:.0f} deg", rows)

    # And the reverse: keep dive 20, shrink the wing back down.
    print()
    for span, ar in ((0.70, 3.5), (0.65, 2.5), (0.5325, 1.86)):
        run(d, BEST_TRAJ, wing_of(d, span, ar),
            f"  traj + wing {span:.3f}/{ar:.2f}", rows)

    import json
    from pathlib import Path
    out = Path("out_medium_model"); out.mkdir(exist_ok=True)
    (out / "v3b_compose.json").write_text(json.dumps(rows, indent=2))

    print(f"\ncombined traverse {both['trav']:.3f} g vs 0.26 target "
          f"-> {'PASS' if both['trav'] >= 0.26 else 'FAIL'}")
    print(f"wrote {out / 'v3b_compose.json'}")


if __name__ == "__main__":
    main()
