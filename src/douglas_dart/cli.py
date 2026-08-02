"""Command-line entry points for transparent reference calculations."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

from .config import load_reference_case
from .pulsejet import PulsejetSimulator, summarize_pulsejet
from .ramjet import evaluate_ramjet


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
    print(json.dumps(asdict(summarize_pulsejet(samples)), indent=2))
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
    pulsejet.add_argument("--csv", default=None)
    pulsejet.set_defaults(func=_pulsejet)

    ramjet = subparsers.add_parser("ramjet", help="evaluate one steady ramjet point")
    ramjet.add_argument("--config", default="configs/reference_case.yaml")
    ramjet.add_argument("--fuels", default=None)
    ramjet.add_argument("--altitude", type=float, default=None)
    ramjet.add_argument("--mach", type=float, required=True)
    ramjet.set_defaults(func=_ramjet)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
