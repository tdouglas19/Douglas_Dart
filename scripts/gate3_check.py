"""Run a Gate 3 (nominal fast-mission feasibility) check and report status.

docs/design_workflow.md: "A candidate completes the mission in the point-mass
solver without violating lift, stall, fuel, dynamic-pressure, mass,
packaging, or rule constraints." Full fidelity (verification-grade dt,
docs/design_convergence.md's dt-convergence solver) -- this is a trust-the-
number check, not a search query.

Usage: python scripts/gate3_check.py [config_path]
Defaults to configs/shared_nozzle_candidate_b.yaml, the config
docs/design_workflow.md's own Gate 3 status line tracks.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import PULSEJET_FIDELITY_FULL
from douglas_dart.trajectory import ADVERSE_SCENARIO, NOMINAL_SCENARIO, simulate_mission

config_path = sys.argv[1] if len(sys.argv) > 1 else "configs/shared_nozzle_candidate_b.yaml"
case = load_reference_case(config_path)

print(f"Gate 3 check: {config_path}")
print(f"  body_diameter_m={case.vehicle.body_diameter_m}  body_length_m={case.vehicle.body_length_m}")
print(f"  throat_diameter_m={case.nozzle.throat_diameter_m}  exit_to_throat_area_ratio={case.nozzle.exit_to_throat_area_ratio}")
print(f"  reference_area_m2={case.flight.reference_area_m2}  sled_release_speed range="
      f"[{case.mission.sled_release_speed_min_m_per_s}, {case.mission.sled_release_speed_max_m_per_s}]")
print()

for scenario in (NOMINAL_SCENARIO, ADVERSE_SCENARIO):
    t0 = time.time()
    result = simulate_mission(
        case, scenario, pulsejet_table_fidelity=PULSEJET_FIDELITY_FULL
    )
    dt = time.time() - t0
    print(f"=== {scenario.name} ({dt:.1f}s) ===")
    print(f"  peak_mach_reached: {result.peak_mach_reached:.4f}  (target met: {result.reached_peak_mach_target})")
    print(f"  time_above_mach_one_s: {result.time_above_mach_one_s:.2f}  "
          f"(required: {result.minimum_time_above_mach_one_requirement_s:.2f}, "
          f"met: {result.meets_minimum_time_above_mach_one})")
    print(f"  minimum_stall_margin_fraction: {result.minimum_stall_margin_fraction:.4f}  "
          f"(violated: {result.stall_margin_violated})")
    print(f"  landed_at_or_below_field_elevation: {result.landed_at_or_below_field_elevation}")
    print(f"  transonic_no_altitude_loss_rule_satisfied: {result.transonic_no_altitude_loss_rule_satisfied}")
    print(f"  peak_dynamic_pressure_pa: {result.peak_dynamic_pressure_pa:.1f}")
    print(f"  final mass_kg: {result.points[-1].mass_kg:.3f}" if result.points else "  no points recorded")
    print(f"  phases: {[p.name for p in result.phases]}")
    print(f"  final_status: {result.final_status}")
    gate3_pass = (
        result.reached_peak_mach_target
        and result.meets_minimum_time_above_mach_one
        and not result.stall_margin_violated
        and result.landed_at_or_below_field_elevation
        and result.transonic_no_altitude_loss_rule_satisfied
    )
    print(f"  GATE 3 (this scenario): {'PASS' if gate3_pass else 'FAIL'}")
    print()
