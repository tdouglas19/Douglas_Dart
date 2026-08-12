"""Run a Gate 3 check on a specific design-optimize checkpoint's best candidate.

Usage: python scripts/gate3_check_candidate.py <checkpoint.json> [mass_budget_path]
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.mass_model import calibrate_mass_model
from douglas_dart.optimizer import DesignVariables, apply_design_variables
from douglas_dart.propulsion_map import PULSEJET_FIDELITY_FULL
from douglas_dart.trajectory import ADVERSE_SCENARIO, NOMINAL_SCENARIO, simulate_mission

checkpoint_path = Path(sys.argv[1])
mass_budget_path = sys.argv[2] if len(sys.argv) > 2 else "configs/robustness_candidate_b.yaml"
base_config_path = sys.argv[3] if len(sys.argv) > 3 else "configs/shared_nozzle_candidate_b.yaml"

with checkpoint_path.open() as f:
    checkpoint = json.load(f)
best_variables = checkpoint[-1]["best_variables"]
print(f"Best candidate from {checkpoint_path} (generation {checkpoint[-1]['generation']}):")
print(json.dumps(best_variables, indent=2))
print()

case = load_reference_case(base_config_path)
mass_calibration = calibrate_mass_model(case, mass_budget_path)
variables = DesignVariables(**best_variables)
candidate = apply_design_variables(case, variables, mass_calibration)

print(f"Resulting candidate geometry: body_diameter_m={candidate.vehicle.body_diameter_m:.4f} "
      f"reference_area_m2={candidate.flight.reference_area_m2:.4f} "
      f"initial_mass_kg={candidate.flight.initial_mass_kg:.3f}")
print()

for scenario in (NOMINAL_SCENARIO, ADVERSE_SCENARIO):
    t0 = time.time()
    result = simulate_mission(
        candidate, scenario,
        climb_angle_deg=variables.climb_angle_deg,
        dive_angle_deg=variables.dive_angle_deg,
        dive_entry_mach=variables.dive_entry_mach,
        pulsejet_table_fidelity=PULSEJET_FIDELITY_FULL,
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
