# V3 climb-dive design -- FROZEN 2026-08-13

**Do not edit these files**; regenerate only by deliberately re-freezing.

- `design.json` -- the exact inputs (vehicle candidate including the
  climb-dive variables, wing concept), the constants that produced them, and
  the verified mission numbers. This is what makes the design re-flyable.
- `v3_optimal_design.md` -- the dimensional/mass/mission table
- `propulsion_v3.png`, `flight_profile_v3.png` -- the two plots
- `v3_summary.json` -- the optimizer campaign output it came from

Frozen at git `df6b5d9e3d97`. Campaign: CD0 = 0.1, acceleration
gate 0.25 g on the traverse, engine-only thrust
margin >= 1.15x, 400 ft hard floor.

Trajectory: climb 16.7 deg to a DERIVED
1933 ft, dive
9.9 deg, pull out at
437 ft, 1.0 deg drag
strip to M 1.1 cutoff, then the return-to-launch profile.

Headline: 225 mm body /
122 mm throat /
389+689 mm duct /
533 mm wing, propane; dry
12.57 kg + 2.69 kg fuel,
7.42 kg payload margin;
peak T/W 6.61, traverse acceleration
+0.261 g at M 0.45,
lands 2 m from launch.

**Protected by `tests/test_v3_frozen.py`**, which re-flies this design from
`design.json` in a subprocess (`scripts/fly_frozen_v3.py`) and asserts the
headline numbers. CD0 is baked at import, so the subprocess is what makes
the check independent of test module import order.
