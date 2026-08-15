# V2 optimal design -- FROZEN 2026-08-12

Baseline for the V3 climb-dive trajectory work. **Do not edit these files**;
regenerate only by deliberately re-freezing.

- `design.json` -- the exact inputs (vehicle candidate + wing concept), the
  constants that produced them, and the verified mission numbers. This is
  the file that makes the design re-flyable.
- `v2_optimal_design.md` -- the dimensional/mass/mission table
- `propulsion.png`, `flight_profile.png` -- the two plots
- `overnight2_summary.json` -- the optimizer campaign output it came from

Frozen at git `0f3759c4812b`. Campaign: CD0 = 0.1, minimum powered
acceleration gate 0.25 g, thrust margin
>= 1.15x, composite objective.

Headline: 280 mm body / 151 mm throat / 342+680 mm duct / 654 mm flat-plate
wing, propane; dry 14.80 kg + 2.03 kg fuel, 5.85 kg payload margin; peak
T/W 10.08, min powered acceleration 0.26 g at M 0.45, lands 2 m from launch.

**Protected by `tests/test_v2_frozen.py`**, which re-flies this design from
`design.json` and asserts the headline numbers -- that test, not the copied
files, is what actually catches drift once `flight_sim.py` grows the V3
phases.
