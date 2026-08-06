"""Standalone pulsejet design-space search: pulse timing, equivalence ratio, fuel.

Runs the existing, tested `PulsejetSimulator` (no new physics) across a grid of
cycle-timing parameters, target equivalence ratio, and every fuel in
`configs/fuels.yaml`, at the static test-stand condition (altitude, Mach) from a
baseline `ReferenceCase`. Each point runs to a warmed-up steady window and is
scored on mean net thrust and fuel-specific impulse, subject to two hard limits
that make a candidate physically buildable rather than just numerically best:

- peak chamber pressure must stay under `_MAX_CHAMBER_PRESSURE_RATIO` times the
  inlet total pressure (valve/casing overpressure margin)
- the model's own mass/energy conservation residuals must stay small (a large
  residual means the timestep or cycle timing is numerically breaking the
  lumped model, not that the physical design is actually good)

Writes every evaluated point to a CSV so the full trade space is inspectable,
not just the winner. Meant to run unattended for as long as you want; it does
not call out to any AI model.
"""

from __future__ import annotations

import csv
import itertools
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from douglas_dart.config import load_fuels, load_reference_case  # noqa: E402
from douglas_dart.pulsejet import PulsejetSimulator, summarize_pulsejet  # noqa: E402

_STANDARD_GRAVITY_M_PER_S2 = 9.80665
_MAX_CHAMBER_PRESSURE_RATIO = 4.0
_MAX_RELATIVE_CONSERVATION_RESIDUAL = 0.02
_WARMUP_S = 0.30
_MEASUREMENT_S = 0.30
_TIME_STEP_S = 0.00002

_MINIMUM_CYCLE_PERIOD_S_VALUES = (0.010, 0.014, 0.018, 0.022, 0.026, 0.030)
_BURN_DURATION_S_VALUES = (0.002, 0.003, 0.004, 0.005, 0.006)
_IGNITION_PRESSURE_RATIO_MAX_VALUES = (1.01, 1.02, 1.03, 1.05, 1.08)
_TARGET_EQUIVALENCE_RATIO_VALUES = (0.70, 0.80, 0.90, 1.00, 1.10)


def _evaluate(case, fuel_key: str, fuel) -> dict:
    trial_case = replace(
        case,
        fuel=fuel,
        pulsejet=replace(
            case.pulsejet,
            minimum_cycle_period_s=trial_minimum_cycle_period_s,
            burn_duration_s=trial_burn_duration_s,
            ignition_pressure_ratio_max=trial_ignition_pressure_ratio_max,
            target_equivalence_ratio=trial_target_equivalence_ratio,
            initial_equivalence_ratio=trial_target_equivalence_ratio,
        ),
    )
    simulator = PulsejetSimulator(
        trial_case.pulsejet,
        trial_case.selector,
        trial_case.nozzle,
        trial_case.fuel,
        trial_case.altitude_m,
        trial_case.mach,
    )
    samples = simulator.run(_WARMUP_S + _MEASUREMENT_S, _TIME_STEP_S)
    summary = summarize_pulsejet(samples, minimum_time_s=_WARMUP_S)
    audit = simulator.conservation_audit()

    pressure_ratio = summary.peak_chamber_pressure_pa / simulator.inlet_total_pressure_pa
    feasible = (
        pressure_ratio <= _MAX_CHAMBER_PRESSURE_RATIO
        and abs(audit.relative_mass_balance_residual) <= _MAX_RELATIVE_CONSERVATION_RESIDUAL
        and abs(audit.relative_energy_balance_residual) <= _MAX_RELATIVE_CONSERVATION_RESIDUAL
        and summary.completed_cycles > 0
        and summary.mean_fuel_mass_flow_kg_per_s > 0.0
    )
    specific_impulse_s = (
        summary.mean_net_thrust_n
        / (summary.mean_fuel_mass_flow_kg_per_s * _STANDARD_GRAVITY_M_PER_S2)
        if summary.mean_fuel_mass_flow_kg_per_s > 0.0
        else 0.0
    )
    return {
        "fuel_key": fuel_key,
        "minimum_cycle_period_s": trial_minimum_cycle_period_s,
        "burn_duration_s": trial_burn_duration_s,
        "ignition_pressure_ratio_max": trial_ignition_pressure_ratio_max,
        "target_equivalence_ratio": trial_target_equivalence_ratio,
        "feasible": feasible,
        "completed_cycles": summary.completed_cycles,
        "mean_net_thrust_n": summary.mean_net_thrust_n,
        "peak_net_thrust_n": summary.peak_net_thrust_n,
        "mean_fuel_mass_flow_kg_per_s": summary.mean_fuel_mass_flow_kg_per_s,
        "specific_impulse_s": specific_impulse_s,
        "peak_chamber_pressure_pa": summary.peak_chamber_pressure_pa,
        "peak_chamber_pressure_ratio": pressure_ratio,
        "peak_chamber_temperature_k": summary.peak_chamber_temperature_k,
        "relative_mass_balance_residual": audit.relative_mass_balance_residual,
        "relative_energy_balance_residual": audit.relative_energy_balance_residual,
    }


def main() -> None:
    case_path = ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
    fuels_path = ROOT / "configs" / "fuels.yaml"
    case = load_reference_case(case_path, fuels_path)
    fuels = load_fuels(fuels_path)

    output_path = ROOT / "docs" / "pulsejet_sweep_results.csv"
    fieldnames = [
        "fuel_key",
        "minimum_cycle_period_s",
        "burn_duration_s",
        "ignition_pressure_ratio_max",
        "target_equivalence_ratio",
        "feasible",
        "completed_cycles",
        "mean_net_thrust_n",
        "peak_net_thrust_n",
        "mean_fuel_mass_flow_kg_per_s",
        "specific_impulse_s",
        "peak_chamber_pressure_pa",
        "peak_chamber_pressure_ratio",
        "peak_chamber_temperature_k",
        "relative_mass_balance_residual",
        "relative_energy_balance_residual",
    ]

    combinations = list(
        itertools.product(
            fuels.items(),
            _MINIMUM_CYCLE_PERIOD_S_VALUES,
            _BURN_DURATION_S_VALUES,
            _IGNITION_PRESSURE_RATIO_MAX_VALUES,
            _TARGET_EQUIVALENCE_RATIO_VALUES,
        )
    )
    total = len(combinations)
    print(f"evaluating {total} pulsejet design points across {len(fuels)} fuels", flush=True)

    global trial_minimum_cycle_period_s, trial_burn_duration_s
    global trial_ignition_pressure_ratio_max, trial_target_equivalence_ratio

    best_by_fuel: dict[str, dict] = {}
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, (
            (fuel_key, fuel),
            trial_minimum_cycle_period_s,
            trial_burn_duration_s,
            trial_ignition_pressure_ratio_max,
            trial_target_equivalence_ratio,
        ) in enumerate(combinations):
            try:
                row = _evaluate(case, fuel_key, fuel)
            except Exception as exc:  # noqa: BLE001 - an unattended sweep must not die on one bad point
                row = {
                    "fuel_key": fuel_key,
                    "minimum_cycle_period_s": trial_minimum_cycle_period_s,
                    "burn_duration_s": trial_burn_duration_s,
                    "ignition_pressure_ratio_max": trial_ignition_pressure_ratio_max,
                    "target_equivalence_ratio": trial_target_equivalence_ratio,
                    "feasible": False,
                    "completed_cycles": 0,
                    "mean_net_thrust_n": None,
                    "peak_net_thrust_n": None,
                    "mean_fuel_mass_flow_kg_per_s": None,
                    "specific_impulse_s": None,
                    "peak_chamber_pressure_pa": None,
                    "peak_chamber_pressure_ratio": None,
                    "peak_chamber_temperature_k": None,
                    "relative_mass_balance_residual": None,
                    "relative_energy_balance_residual": None,
                }
                print(f"  point {index+1}/{total} failed: {exc}", flush=True)
            writer.writerow(row)
            if row["feasible"]:
                current_best = best_by_fuel.get(fuel_key)
                if (
                    current_best is None
                    or row["mean_net_thrust_n"] > current_best["mean_net_thrust_n"]
                ):
                    best_by_fuel[fuel_key] = row
            if (index + 1) % 200 == 0:
                print(f"  {index+1}/{total} evaluated", flush=True)

    print(f"\nwrote {total} rows to {output_path}")
    print("\nbest feasible design point per fuel (by mean net thrust):")
    for fuel_key, row in sorted(
        best_by_fuel.items(), key=lambda item: item[1]["mean_net_thrust_n"], reverse=True
    ):
        print(
            f"  {fuel_key}: thrust={row['mean_net_thrust_n']:.2f} N, "
            f"Isp={row['specific_impulse_s']:.1f} s, "
            f"cycle_period={row['minimum_cycle_period_s']:.3f} s, "
            f"burn={row['burn_duration_s']:.3f} s, "
            f"ignition_ratio={row['ignition_pressure_ratio_max']:.2f}, "
            f"phi={row['target_equivalence_ratio']:.2f}, "
            f"peak_p_ratio={row['peak_chamber_pressure_ratio']:.2f}"
        )
    if not best_by_fuel:
        print("  no feasible design points found")


if __name__ == "__main__":
    main()
