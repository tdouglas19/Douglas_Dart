"""Command-line entry points for transparent reference calculations."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

from .config import load_fuels, load_reference_case
from .feasibility import evaluate_level0_feasibility
from .fuel_trade import fuel_performance_trade
from .mass_model import calibrate_mass_model, evaluate_parametric_mass
from .propulsion_map import (
    NOMINAL,
    PULSEJET_MODE,
    RAMJET_MODE,
    build_propulsion_map,
    write_propulsion_map_csv,
)
from .openvsp_geometry import build_openvsp_geometry
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet
from .robustness import run_robustness_trade
from .sensitivity import (
    pulsejet_local_sensitivities,
    ramjet_local_sensitivities,
    rank_by_net_thrust_sensitivity,
)
from .sizing import (
    peak_mach_altitude_trade_sweep,
    peak_mach_diameter_trade_sweep,
    ramjet_handoff_sweep,
    select_minimum_feasible_shared_nozzle,
    shared_nozzle_feasibility_bounds,
    shared_nozzle_trade_sweep,
)
from .jsbsim_model import validate_with_jsbsim, write_jsbsim_aircraft
from .optimizer import DEFAULT_BOUNDS, run_differential_evolution, write_generation_log_csv
from .trajectory import (
    ADVERSE_SCENARIO,
    CONSERVATIVE_SCENARIO,
    NOMINAL_SCENARIO,
    simulate_mission,
)
from .vspaero import run_vspaero_sweep

_MISSION_SCENARIOS = {
    "nominal": NOMINAL_SCENARIO,
    "conservative": CONSERVATIVE_SCENARIO,
    "adverse": ADVERSE_SCENARIO,
}


def _write_samples(path: Path, samples: list) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(sample) for sample in samples]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def _pulsejet(args: argparse.Namespace) -> int:
    # Intentionally NOT migrated to propulsion_map.py: this standalone
    # diagnostic command's entire purpose is exposing the raw simulator
    # (per-step samples, conservation audit) for direct human inspection of
    # one chamber run -- not participating in the cross-consumer consistency
    # guarantee propulsion_map.py exists for. See docs/design_workflow.md
    # Gate 2 and evaluate_ramjet_handoff_sizing's docstring in sizing.py for
    # the same reasoning applied elsewhere.
    case = load_reference_case(args.config, args.fuels)
    simulator = PulsejetSimulator(
        case.pulsejet,
        case.selector,
        case.nozzle,
        case.fuel,
        case.altitude_m if args.altitude is None else args.altitude,
        case.mach if args.mach is None else args.mach,
    )
    duration_s = case.simulation.pulsejet_duration_s if args.duration is None else args.duration
    time_step_s = case.simulation.time_step_s if args.dt is None else args.dt
    samples = simulator.run(duration_s, time_step_s)
    if args.csv:
        _write_samples(Path(args.csv), samples)
    output = {
        "summary": asdict(
            summarize_pulsejet(samples, minimum_time_s=args.summary_start)
        ),
        "conservation_audit": asdict(simulator.conservation_audit()),
    }
    print(json.dumps(output, indent=2))
    return 0


def _ramjet(args: argparse.Namespace) -> int:
    # Intentionally NOT migrated to propulsion_map.py: same reasoning as
    # _pulsejet above -- this is a standalone single-point diagnostic dump of
    # the full RamjetResult (including solver internals like fuel_air_ratio
    # and nozzle_capacity_kg_per_s that PropulsionMapPoint deliberately
    # omits), not a value consumed by another design calculation.
    case = load_reference_case(args.config, args.fuels)
    result = evaluate_ramjet(
        case.ramjet,
        case.selector,
        case.nozzle,
        case.fuel,
        case.altitude_m if args.altitude is None else args.altitude,
        args.mach,
    )
    print(json.dumps(asdict(result), indent=2))
    return 0


def _ramjet_sweep(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    points = ramjet_handoff_sweep(
        case,
        case.mission.speed_run_altitude_msl_m if args.altitude is None else args.altitude,
        args.minimum_mach,
        args.maximum_mach,
        args.mach_step,
    )
    if args.csv:
        _write_samples(Path(args.csv), points)
    print(json.dumps([asdict(point) for point in points], indent=2))
    return 0


def _diameter_trade(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    minimum_body_diameter_m = (
        case.vehicle.body_diameter_m
        if args.minimum_body_diameter is None
        else args.minimum_body_diameter
    )
    points = peak_mach_diameter_trade_sweep(
        case,
        minimum_body_diameter_m,
        args.maximum_body_diameter,
        args.body_diameter_step,
    )
    if args.csv:
        _write_samples(Path(args.csv), points)
    print(json.dumps([asdict(point) for point in points], indent=2))
    return 0


def _float_list(value: str) -> tuple[float, ...]:
    values = tuple(float(item.strip()) for item in value.split(",") if item.strip())
    if not values:
        raise argparse.ArgumentTypeError("at least one comma-separated value is required")
    return values


def _shared_nozzle_trade(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    points = shared_nozzle_trade_sweep(
        case,
        body_diameters_m=args.body_diameters,
        throat_diameters_m=args.throat_diameters,
        exit_to_throat_area_ratios=args.area_ratios,
        propulsion_derate_fraction=args.propulsion_derate,
        pulsejet_warmup_s=args.pulsejet_warmup,
        pulsejet_measurement_s=args.pulsejet_measurement,
        pulsejet_time_step_s=args.pulsejet_dt,
    )
    selected = select_minimum_feasible_shared_nozzle(points)
    if args.csv:
        _write_samples(Path(args.csv), points)
    output = {
        "selection_rule": (
            "smallest feasible body, then throat, then exit-to-throat area ratio"
        ),
        "selected": asdict(selected) if selected is not None else None,
        "points": [asdict(point) for point in points],
    }
    print(json.dumps(output, indent=2))
    return 0


def _design_convergence(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    bounds = shared_nozzle_feasibility_bounds(
        case,
        propulsion_derate_fraction=args.propulsion_derate,
    )
    print(json.dumps(asdict(bounds), indent=2))
    return 0


def _propulsion_map(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    modes = (PULSEJET_MODE, RAMJET_MODE) if args.mode == "both" else (args.mode,)
    points = build_propulsion_map(
        case,
        mach_values=tuple(args.mach_values),
        altitude_values=tuple(args.altitude_values),
        modes=modes,
        scenario=NOMINAL,
    )
    if args.csv:
        write_propulsion_map_csv(Path(args.csv), points)
    print(json.dumps([asdict(point) for point in points], indent=2))
    return 0


def _mass_breakdown(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    calibration = calibrate_mass_model(case, args.mass_budget)
    breakdown = evaluate_parametric_mass(
        case,
        calibration,
        body_diameter_m=args.body_diameter,
        body_length_m=args.body_length,
        throat_diameter_m=args.throat_diameter,
    )
    print(json.dumps({"calibration": asdict(calibration), "breakdown": asdict(breakdown)}, indent=2))
    return 0


def _level0_bounds(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    report = evaluate_level0_feasibility(case)
    output = asdict(report)
    print(json.dumps(output, indent=2))
    if not report.all_pass:
        print(f"\nLevel 0 FAILING checks: {', '.join(report.failing_checks)}")
    return 0 if report.all_pass or not args.strict else 1


def _fuel_trade(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    fuels_path = (
        Path(args.fuels)
        if args.fuels is not None
        else Path(args.config).with_name("fuels.yaml")
    )
    points = fuel_performance_trade(
        case,
        load_fuels(fuels_path),
        propulsion_derate_fraction=args.propulsion_derate,
    )
    if args.csv:
        _write_samples(Path(args.csv), points)
    print(json.dumps([asdict(point) for point in points], indent=2))
    return 0


def _altitude_trade(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    points = peak_mach_altitude_trade_sweep(
        case,
        args.minimum_altitude,
        args.maximum_altitude,
        args.altitude_step,
        propulsion_derate_fraction=args.propulsion_derate,
    )
    if args.csv:
        _write_samples(Path(args.csv), points)
    print(json.dumps([asdict(point) for point in points], indent=2))
    return 0


def _robustness_trade(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    result = run_robustness_trade(case, args.robustness)
    print(json.dumps(asdict(result), indent=2))
    return 0


def _key_variables(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    pulsejet_warmup_s = (
        case.simulation.pulsejet_steady_warmup_s
        if args.pulsejet_warmup is None
        else args.pulsejet_warmup
    )
    pulsejet_measurement_s = (
        case.simulation.pulsejet_steady_measurement_s
        if args.pulsejet_measurement is None
        else args.pulsejet_measurement
    )
    pulsejet_time_step_s = (
        case.simulation.time_step_s
        if args.pulsejet_dt is None
        else args.pulsejet_dt
    )
    points = []
    if args.engine in {"pulsejet", "both"}:
        points.extend(
            pulsejet_local_sensitivities(
                case,
                perturbation_fraction=args.perturbation,
                warmup_s=pulsejet_warmup_s,
                measurement_s=pulsejet_measurement_s,
                time_step_s=pulsejet_time_step_s,
            )
        )
    if args.engine in {"ramjet", "both"}:
        points.extend(
            ramjet_local_sensitivities(
                case,
                perturbation_fraction=args.perturbation,
            )
        )
    ranked = rank_by_net_thrust_sensitivity(points)
    if args.csv:
        _write_samples(Path(args.csv), ranked)
    print(json.dumps([asdict(point) for point in ranked], indent=2))
    return 0


def _mission_trajectory(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    scenario = _MISSION_SCENARIOS[args.scenario]
    result = simulate_mission(case, scenario, time_step_s=args.dt, max_time_s=args.max_time)
    if args.csv:
        _write_samples(Path(args.csv), list(result.points))
    output = asdict(result)
    if not args.include_points:
        output.pop("points", None)
    print(json.dumps(output, indent=2))
    return 0


def _design_optimize(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_path = output_dir / "checkpoint.json"
    csv_path = output_dir / "generation_log.csv"

    def on_generation(record) -> None:
        print(
            f"generation {record.generation:3d}  best_score={record.best_score:12.1f}  "
            f"feasible={record.feasible_count}/{record.population_size}  "
            f"nominal_mach={record.best_evaluation.nominal_peak_mach}  "
            f"adverse_mach={record.best_evaluation.adverse_peak_mach}"
        )

    records = run_differential_evolution(
        case,
        mass_budget_path=args.mass_budget,
        population_size=args.population,
        generations=args.generations,
        mutation_factor=args.mutation_factor,
        crossover_probability=args.crossover_probability,
        seed=args.seed,
        checkpoint_path=checkpoint_path,
        progress_callback=on_generation,
    )
    write_generation_log_csv(csv_path, records)
    best = records[-1].best_evaluation
    print(json.dumps(asdict(best), indent=2))
    print(f"\nCheckpoint: {checkpoint_path}\nGeneration log: {csv_path}")
    return 0


def _jsbsim_build(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    scenario = _MISSION_SCENARIOS[args.scenario]
    root_dir = Path(args.root)
    output_path, summary = write_jsbsim_aircraft(case, root_dir / "aircraft", scenario)
    print(json.dumps(asdict(summary), indent=2))
    if args.check:
        check = validate_with_jsbsim(
            root_dir,
            output_path.parent.name,
            altitude_m=args.check_altitude,
            mach=args.check_mach,
            run_seconds=args.check_seconds,
        )
        print(json.dumps(asdict(check), indent=2))
    return 0


def _openvsp_build(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    summary = build_openvsp_geometry(case, args.output)
    print(json.dumps(asdict(summary), indent=2))
    return 0


def _vspaero_sweep(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    summary = run_vspaero_sweep(case, args.model)
    if args.csv:
        _write_samples(Path(args.csv), list(summary.points))
    print(json.dumps(asdict(summary), indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="douglas-dart")
    subparsers = parser.add_subparsers(required=True)

    pulsejet = subparsers.add_parser("pulsejet", help="run the unsteady reference chamber")
    pulsejet.add_argument("--config", default="configs/reference_case.yaml")
    pulsejet.add_argument("--fuels", default=None)
    pulsejet.add_argument("--altitude", type=float, default=None)
    pulsejet.add_argument("--mach", type=float, default=None)
    pulsejet.add_argument("--duration", type=float, default=None)
    pulsejet.add_argument("--dt", type=float, default=None)
    pulsejet.add_argument(
        "--summary-start",
        type=float,
        default=0.0,
        help="exclude earlier startup samples from the reported statistics",
    )
    pulsejet.add_argument("--csv", default=None)
    pulsejet.set_defaults(func=_pulsejet)

    ramjet = subparsers.add_parser("ramjet", help="evaluate one steady ramjet point")
    ramjet.add_argument("--config", default="configs/reference_case.yaml")
    ramjet.add_argument("--fuels", default=None)
    ramjet.add_argument("--altitude", type=float, default=None)
    ramjet.add_argument("--mach", type=float, required=True)
    ramjet.set_defaults(func=_ramjet)

    ramjet_sweep = subparsers.add_parser(
        "ramjet-sweep", help="sweep handoff Mach and report throat flow-match geometry"
    )
    ramjet_sweep.add_argument("--config", default="configs/reference_case.yaml")
    ramjet_sweep.add_argument("--fuels", default=None)
    ramjet_sweep.add_argument("--altitude", type=float, default=None)
    ramjet_sweep.add_argument("--minimum-mach", type=float, default=0.80)
    ramjet_sweep.add_argument("--maximum-mach", type=float, default=1.10)
    ramjet_sweep.add_argument("--mach-step", type=float, default=0.05)
    ramjet_sweep.add_argument("--csv", default=None)
    ramjet_sweep.set_defaults(func=_ramjet_sweep)

    diameter_trade = subparsers.add_parser(
        "diameter-trade",
        help=(
            "trade outer-body throat packaging and scaled drag target at peak Mach; "
            "speed-run duration is derived from the allocated ramjet fuel"
        ),
    )
    diameter_trade.add_argument("--config", default="configs/reference_case.yaml")
    diameter_trade.add_argument("--fuels", default=None)
    diameter_trade.add_argument("--minimum-body-diameter", type=float, default=None)
    diameter_trade.add_argument(
        "--maximum-body-diameter",
        type=float,
        required=True,
        help="analysis sweep bound in meters, not a vehicle diameter constraint",
    )
    diameter_trade.add_argument("--body-diameter-step", type=float, default=0.005)
    diameter_trade.add_argument("--csv", default=None)
    diameter_trade.set_defaults(func=_diameter_trade)

    shared_nozzle = subparsers.add_parser(
        "shared-nozzle-trade",
        help=(
            "evaluate one fixed C-D nozzle in pulsejet and ramjet modes, including "
            "intentional spillage, packaging allowances, drag budget, and fuel limit"
        ),
    )
    shared_nozzle.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    shared_nozzle.add_argument("--fuels", default=None)
    shared_nozzle.add_argument(
        "--body-diameters",
        type=_float_list,
        default=(0.205,),
        help="comma-separated outer-body diameters in meters",
    )
    shared_nozzle.add_argument(
        "--throat-diameters",
        type=_float_list,
        default=(0.110, 0.120, 0.130, 0.140),
        help="comma-separated fixed throat diameters in meters",
    )
    shared_nozzle.add_argument(
        "--area-ratios",
        type=_float_list,
        default=(1.05, 1.10, 1.20),
        help="comma-separated exit-to-throat area ratios",
    )
    shared_nozzle.add_argument("--propulsion-derate", type=float, default=0.15)
    shared_nozzle.add_argument("--pulsejet-warmup", type=float, default=None)
    shared_nozzle.add_argument("--pulsejet-measurement", type=float, default=None)
    shared_nozzle.add_argument("--pulsejet-dt", type=float, default=None)
    shared_nozzle.add_argument("--csv", default=None)
    shared_nozzle.set_defaults(func=_shared_nozzle_trade)

    design_convergence = subparsers.add_parser(
        "design-convergence",
        help=(
            "report local body and throat feasibility bounds around the configured "
            "shared-nozzle architecture"
        ),
    )
    design_convergence.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    design_convergence.add_argument("--fuels", default=None)
    design_convergence.add_argument("--propulsion-derate", type=float, default=0.15)
    design_convergence.set_defaults(func=_design_convergence)

    propulsion_map = subparsers.add_parser(
        "propulsion-map",
        help=(
            "Gate 2 authoritative propulsion map (docs/design_workflow.md, "
            "propulsion_map.py): thrust, fuel flow, TSFC, spillage, recovery, and "
            "operability vs. Mach and altitude for pulsejet and/or ramjet"
        ),
    )
    propulsion_map.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    propulsion_map.add_argument("--fuels", default=None)
    propulsion_map.add_argument(
        "--mode",
        choices=("pulsejet", "ramjet", "both"),
        default="both",
    )
    propulsion_map.add_argument(
        "--mach-values",
        type=float,
        nargs="+",
        default=[0.2, 0.4, 0.6, 0.8, 1.0, 1.1],
    )
    propulsion_map.add_argument(
        "--altitude-values",
        type=float,
        nargs="+",
        default=[0.0, 4500.0],
    )
    propulsion_map.add_argument("--csv", default=None)
    propulsion_map.set_defaults(func=_propulsion_map)

    mass_breakdown = subparsers.add_parser(
        "mass-breakdown",
        help=(
            "docs/design_workflow.md Level 3 parametric mass model "
            "(mass_model.py): geometry-linked empty-mass breakdown, calibrated "
            "against a named mass-budget YAML"
        ),
    )
    mass_breakdown.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    mass_breakdown.add_argument("--fuels", default=None)
    mass_breakdown.add_argument(
        "--mass-budget",
        default="configs/robustness_candidate_b.yaml",
    )
    mass_breakdown.add_argument(
        "--body-diameter",
        type=float,
        default=None,
        help="override body diameter (m); defaults to the configured value",
    )
    mass_breakdown.add_argument(
        "--body-length",
        type=float,
        default=None,
        help="override body length (m); defaults to the configured value",
    )
    mass_breakdown.add_argument(
        "--throat-diameter",
        type=float,
        default=None,
        help="override nozzle throat diameter (m); defaults to the configured value",
    )
    mass_breakdown.set_defaults(func=_mass_breakdown)

    level0_bounds = subparsers.add_parser(
        "level0-bounds",
        help=(
            "Gate 1 hand-calculation feasibility bounds (docs/design_workflow.md): "
            "stall speed, lift area, thrust-to-weight, climb rate, dive energy, "
            "fuel/endurance, packaging -- rejects impossible concepts, does not "
            "select a design"
        ),
    )
    level0_bounds.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    level0_bounds.add_argument("--fuels", default=None)
    level0_bounds.add_argument(
        "--strict",
        action="store_true",
        help="exit with a nonzero status if any Level 0 check fails",
    )
    level0_bounds.set_defaults(func=_level0_bounds)

    fuel_trade = subparsers.add_parser(
        "fuel-trade",
        help=(
            "compare provisional fuel performance and tank volume without an "
            "unsupported safety or hardware score"
        ),
    )
    fuel_trade.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    fuel_trade.add_argument("--fuels", default=None)
    fuel_trade.add_argument("--propulsion-derate", type=float, default=0.15)
    fuel_trade.add_argument("--csv", default=None)
    fuel_trade.set_defaults(func=_fuel_trade)

    altitude_trade = subparsers.add_parser(
        "altitude-trade",
        help="sweep static Mach-peak thrust, drag budget, and fuel hold with altitude",
    )
    altitude_trade.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    altitude_trade.add_argument("--fuels", default=None)
    altitude_trade.add_argument("--minimum-altitude", type=float, default=3000.0)
    altitude_trade.add_argument("--maximum-altitude", type=float, default=6500.0)
    altitude_trade.add_argument("--altitude-step", type=float, default=500.0)
    altitude_trade.add_argument("--propulsion-derate", type=float, default=0.15)
    altitude_trade.add_argument("--csv", default=None)
    altitude_trade.set_defaults(func=_altitude_trade)

    robustness_trade = subparsers.add_parser(
        "robustness-trade",
        help=(
            "evaluate component mass and named nominal, conservative, and adverse "
            "peak-Mach scenarios with visible objective weights"
        ),
    )
    robustness_trade.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    robustness_trade.add_argument("--fuels", default=None)
    robustness_trade.add_argument(
        "--robustness",
        default="configs/robustness_candidate_b.yaml",
    )
    robustness_trade.set_defaults(func=_robustness_trade)

    key_variables = subparsers.add_parser(
        "key-variables",
        help="rank local pulsejet and ramjet sensitivities around a configured point",
    )
    key_variables.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    key_variables.add_argument("--fuels", default=None)
    key_variables.add_argument(
        "--engine",
        choices=("pulsejet", "ramjet", "both"),
        default="both",
    )
    key_variables.add_argument("--perturbation", type=float, default=0.10)
    key_variables.add_argument("--pulsejet-warmup", type=float, default=None)
    key_variables.add_argument("--pulsejet-measurement", type=float, default=None)
    key_variables.add_argument("--pulsejet-dt", type=float, default=None)
    key_variables.add_argument("--csv", default=None)
    key_variables.set_defaults(func=_key_variables)

    mission_trajectory = subparsers.add_parser(
        "mission-trajectory",
        help=(
            "integrate the phase-based sled-release-to-landing mission at a named "
            "nominal/conservative/adverse robustness scenario"
        ),
    )
    mission_trajectory.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    mission_trajectory.add_argument("--fuels", default=None)
    mission_trajectory.add_argument(
        "--scenario",
        choices=tuple(_MISSION_SCENARIOS),
        default="nominal",
    )
    mission_trajectory.add_argument("--dt", type=float, default=0.05)
    mission_trajectory.add_argument("--max-time", type=float, default=900.0)
    mission_trajectory.add_argument("--include-points", action="store_true")
    mission_trajectory.add_argument("--csv", default=None)
    mission_trajectory.set_defaults(func=_mission_trajectory)

    design_optimize = subparsers.add_parser(
        "design-optimize",
        help=(
            "run a local, unattended differential-evolution search over body "
            "diameter/length, throat, area ratio, fuel split, sled release speed, "
            "climb/dive geometry, wing area, and ramjet lightoff Mach against the "
            "nominal and adverse trajectory scenarios, with mass_model.py's "
            "geometry-linked mass; no AI model calls"
        ),
    )
    design_optimize.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    design_optimize.add_argument("--fuels", default=None)
    design_optimize.add_argument(
        "--mass-budget",
        default="configs/robustness_candidate_b.yaml",
        help="mass budget YAML used to calibrate mass_model.py's geometry-linked scaling",
    )
    design_optimize.add_argument("--population", type=int, default=20)
    design_optimize.add_argument("--generations", type=int, default=30)
    design_optimize.add_argument("--mutation-factor", type=float, default=0.6)
    design_optimize.add_argument("--crossover-probability", type=float, default=0.7)
    design_optimize.add_argument("--seed", type=int, default=0)
    design_optimize.add_argument(
        "--output-dir",
        default="results/generated/design_optimize",
    )
    design_optimize.set_defaults(func=_design_optimize)

    jsbsim_build = subparsers.add_parser(
        "jsbsim-build",
        help=(
            "generate a JSBSim aircraft model from the configured propulsion, drag, "
            "mass, and geometry data, with mutually exclusive pulsejet/ramjet "
            "external-reactions thrust tables"
        ),
    )
    jsbsim_build.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    jsbsim_build.add_argument("--fuels", default=None)
    jsbsim_build.add_argument(
        "--scenario",
        choices=tuple(_MISSION_SCENARIOS),
        default="nominal",
    )
    jsbsim_build.add_argument("--root", default="jsbsim/generated")
    jsbsim_build.add_argument(
        "--check",
        action="store_true",
        help="load the model in real JSBSim and run a short sanity simulation",
    )
    jsbsim_build.add_argument("--check-altitude", type=float, default=4500.0)
    jsbsim_build.add_argument("--check-mach", type=float, default=0.20)
    jsbsim_build.add_argument("--check-seconds", type=float, default=5.0)
    jsbsim_build.set_defaults(func=_jsbsim_build)

    openvsp_build = subparsers.add_parser(
        "openvsp-build",
        help="generate the configured flow-through body and radial surfaces",
    )
    openvsp_build.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    openvsp_build.add_argument("--fuels", default=None)
    openvsp_build.add_argument(
        "--output",
        default="openvsp/generated/shared_nozzle_candidate_b.vsp3",
    )
    openvsp_build.set_defaults(func=_openvsp_build)

    vspaero_sweep = subparsers.add_parser(
        "vspaero-sweep",
        help="run the explicitly configured OpenVSP Mach/alpha/beta points",
    )
    vspaero_sweep.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    vspaero_sweep.add_argument("--fuels", default=None)
    vspaero_sweep.add_argument(
        "--model",
        default="openvsp/generated/shared_nozzle_candidate_b.vsp3",
    )
    vspaero_sweep.add_argument("--csv", default=None)
    vspaero_sweep.set_defaults(func=_vspaero_sweep)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
