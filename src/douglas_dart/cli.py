"""Command-line entry points for transparent reference calculations."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

from .config import load_fuels, load_reference_case
from .fuel_trade import fuel_performance_trade
from .openvsp_geometry import build_openvsp_geometry
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet
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
from .vspaero import run_vspaero_sweep


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
        default="configs/shared_nozzle_candidate_a.yaml",
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
        default="configs/shared_nozzle_candidate_a.yaml",
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
        default="configs/shared_nozzle_candidate_a.yaml",
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
        default="configs/shared_nozzle_candidate_a.yaml",
    )
    altitude_trade.add_argument("--fuels", default=None)
    altitude_trade.add_argument("--minimum-altitude", type=float, default=3000.0)
    altitude_trade.add_argument("--maximum-altitude", type=float, default=6500.0)
    altitude_trade.add_argument("--altitude-step", type=float, default=500.0)
    altitude_trade.add_argument("--propulsion-derate", type=float, default=0.15)
    altitude_trade.add_argument("--csv", default=None)
    altitude_trade.set_defaults(func=_altitude_trade)

    key_variables = subparsers.add_parser(
        "key-variables",
        help="rank local pulsejet and ramjet sensitivities around a configured point",
    )
    key_variables.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_a.yaml",
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

    openvsp_build = subparsers.add_parser(
        "openvsp-build",
        help="generate the configured flow-through body and radial surfaces",
    )
    openvsp_build.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_a.yaml",
    )
    openvsp_build.add_argument("--fuels", default=None)
    openvsp_build.add_argument(
        "--output",
        default="openvsp/generated/shared_nozzle_candidate_a.vsp3",
    )
    openvsp_build.set_defaults(func=_openvsp_build)

    vspaero_sweep = subparsers.add_parser(
        "vspaero-sweep",
        help="run the explicitly configured OpenVSP Mach/alpha/beta points",
    )
    vspaero_sweep.add_argument(
        "--config",
        default="configs/shared_nozzle_candidate_a.yaml",
    )
    vspaero_sweep.add_argument("--fuels", default=None)
    vspaero_sweep.add_argument(
        "--model",
        default="openvsp/generated/shared_nozzle_candidate_a.vsp3",
    )
    vspaero_sweep.add_argument("--csv", default=None)
    vspaero_sweep.set_defaults(func=_vspaero_sweep)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
