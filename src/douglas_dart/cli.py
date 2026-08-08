"""Command-line entry points for transparent reference calculations."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, replace
from pathlib import Path

from .aero import BudgetAeroModel, SupplementedVSPAeroModel
from .aero_tables import (
    VSPAeroCoefficientTable,
    load_vspaero_summary_json,
    write_vspaero_summary_json,
)
from .config import load_fuels, load_reference_case
from .fuel_trade import fuel_performance_trade
from .mission import MissionPolicy, MissionSimulator
from .mission_trade import mission_trade_sweep
from .openvsp_geometry import build_openvsp_geometry
from .performance_maps import build_pulsejet_performance_map
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


def _mission(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    if args.ramjet_fuel_allocation is not None:
        case = replace(
            case,
            mission=replace(
                case.mission,
                ramjet_speed_run_fuel_budget_kg=args.ramjet_fuel_allocation,
            ),
        )
    map_build = build_pulsejet_performance_map(
        case,
        time_step_s=args.pulsejet_map_dt,
    )
    policy = MissionPolicy.from_case(
        case,
        top_of_climb_altitude_m=args.top_of_climb_altitude,
        climb_flight_path_angle_deg=args.climb_angle,
        dive_flight_path_angle_deg=args.dive_angle,
        ramjet_spillage_drag_momentum_fraction=(
            args.spillage_momentum_fraction
        ),
        ramjet_handoff_mach=args.handoff_mach,
        allow_forced_ramjet_below_self_sustaining=args.allow_forced_ramjet,
    )
    aero_model = None
    if args.vspaero_json is not None:
        table = VSPAeroCoefficientTable.from_summary(
            load_vspaero_summary_json(args.vspaero_json),
            beta_deg=0.0,
        )
        budget_model = BudgetAeroModel(case)
        aero_model = SupplementedVSPAeroModel(
            case,
            table,
            budget_model.parasitic_drag_area_m2,
            supplementary_drag_status=(
                "configured_parasitic_budget_added_to_vspaero_inviscid_drag"
            ),
        )
    result = MissionSimulator(
        case,
        map_build.engine_map,
        policy=policy,
        aero_model=aero_model,
    ).run(
        release_speed_m_per_s=args.release_speed,
        integration_time_step_s=args.mission_dt,
    )
    if args.csv:
        _write_samples(Path(args.csv), list(result.samples))
    if args.map_csv:
        _write_samples(Path(args.map_csv), list(map_build.points))
    output = {
        "launch_screen": asdict(result.launch_screen),
        "policy": asdict(result.policy),
        "summary": asdict(result.summary),
        "events": [asdict(event) for event in result.events],
        "pulsejet_map": {
            "altitudes_m": map_build.engine_map.altitudes_m,
            "mach_values": map_build.engine_map.mach_values,
            "point_count": len(map_build.points),
            "numerical_reference_only": True,
        },
        "aerodynamics": {
            "source": (
                "vspaero_inviscid_plus_configured_parasitic_budget"
                if aero_model is not None
                else "provisional_budget_model"
            ),
            "vspaero_json": args.vspaero_json,
        },
    }
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


def _mission_trade(args: argparse.Namespace) -> int:
    case = load_reference_case(args.config, args.fuels)
    map_build = build_pulsejet_performance_map(
        case,
        time_step_s=args.pulsejet_map_dt,
    )
    points = mission_trade_sweep(
        case,
        map_build.engine_map,
        release_speeds_m_per_s=args.release_speeds,
        top_of_climb_altitudes_m=args.top_of_climb_altitudes,
        climb_flight_path_angles_deg=args.climb_angles,
        dive_flight_path_angles_deg=args.dive_angles,
        ramjet_fuel_allocations_kg=args.ramjet_fuel_allocations,
        loaded_fuel_masses_kg=args.loaded_fuel_masses,
        ramjet_handoff_mach=args.handoff_mach,
        allow_forced_ramjet_below_self_sustaining=(
            args.allow_forced_ramjet
        ),
        ramjet_spillage_drag_momentum_fractions=(
            args.spillage_momentum_fractions
        ),
        integration_time_step_s=args.mission_dt,
    )
    if args.csv:
        _write_samples(Path(args.csv), points)
    if args.map_csv:
        _write_samples(Path(args.map_csv), list(map_build.points))
    closing_points = [point for point in points if point.full_mission_numerically_closes]
    output = {
        "selection": None,
        "selection_note": (
            "points are intentionally unranked because forced operability and "
            "spillage-drag evidence are unresolved gates"
        ),
        "point_count": len(points),
        "numerical_closure_count": len(closing_points),
        "points": [asdict(point) for point in points],
    }
    print(json.dumps(output, indent=2))
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
    if args.json:
        write_vspaero_summary_json(summary, args.json)
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
            "diameter, throat, area ratio, fuel split, and climb/dive geometry "
            "against the nominal and adverse trajectory scenarios; no AI model calls"
        ),
    )
    design_optimize.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_b.yaml",
    )
    design_optimize.add_argument("--fuels", default=None)
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

    mission = subparsers.add_parser(
        "mission",
        help="run the fuel-limited phase-based longitudinal mission model",
    )
    mission.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_a.yaml",
    )
    mission.add_argument("--fuels", default=None)
    mission.add_argument("--release-speed", type=float, default=None)
    mission.add_argument("--top-of-climb-altitude", type=float, default=None)
    mission.add_argument("--climb-angle", type=float, default=None)
    mission.add_argument("--dive-angle", type=float, default=None)
    mission.add_argument("--ramjet-fuel-allocation", type=float, default=None)
    mission.add_argument("--handoff-mach", type=float, default=None)
    mission.add_argument(
        "--allow-forced-ramjet",
        action="store_true",
        default=None,
        help="override the config to permit forced operation below self-sustaining Mach",
    )
    mission.add_argument("--spillage-momentum-fraction", type=float, default=None)
    mission.add_argument("--mission-dt", type=float, default=None)
    mission.add_argument("--pulsejet-map-dt", type=float, default=None)
    mission.add_argument("--csv", default=None)
    mission.add_argument("--map-csv", default=None)
    mission.add_argument(
        "--vspaero-json",
        default=None,
        help=(
            "load a validated beta=0 VSPAERO summary and add the configured "
            "parasitic drag-area budget to its inviscid drag"
        ),
    )
    mission.set_defaults(func=_mission)

    mission_trade = subparsers.add_parser(
        "mission-trade",
        help=(
            "sweep release, climb, dive, fuel split, handoff, and spillage inputs "
            "without an unsupported aggregate score"
        ),
    )
    mission_trade.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_a.yaml",
    )
    mission_trade.add_argument("--fuels", default=None)
    mission_trade.add_argument(
        "--release-speeds",
        type=_float_list,
        default=(90.0, 105.0, 120.0),
    )
    mission_trade.add_argument(
        "--top-of-climb-altitudes",
        type=_float_list,
        default=(6000.0, 6250.0, 6500.0),
    )
    mission_trade.add_argument(
        "--climb-angles",
        type=_float_list,
        default=(45.0, 55.0, 65.0),
    )
    mission_trade.add_argument(
        "--dive-angles",
        type=_float_list,
        default=(-20.0, -30.0, -40.0),
    )
    mission_trade.add_argument(
        "--ramjet-fuel-allocations",
        type=_float_list,
        default=(0.55, 0.65, 0.75),
    )
    mission_trade.add_argument(
        "--loaded-fuel-masses",
        type=_float_list,
        default=None,
        help=(
            "comma-separated loaded fuel masses in kilograms; inferred dry mass "
            "is held fixed and takeoff mass changes with fuel"
        ),
    )
    mission_trade.add_argument("--handoff-mach", type=float, default=1.10)
    mission_trade.add_argument("--allow-forced-ramjet", action="store_true")
    mission_trade.add_argument(
        "--spillage-momentum-fractions",
        type=_float_list,
        default=(0.0,),
    )
    mission_trade.add_argument("--mission-dt", type=float, default=None)
    mission_trade.add_argument("--pulsejet-map-dt", type=float, default=None)
    mission_trade.add_argument("--csv", default=None)
    mission_trade.add_argument("--map-csv", default=None)
    mission_trade.set_defaults(func=_mission_trade)

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
    vspaero_sweep.add_argument(
        "--json",
        default=None,
        help="write the complete solver summary with reference metadata",
    )
    vspaero_sweep.set_defaults(func=_vspaero_sweep)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
