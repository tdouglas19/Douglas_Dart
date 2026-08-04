#!/usr/bin/env python3
"""Run the complete Douglas Dart analysis and geometry-generation pipeline."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parent
SRC_DIRECTORY = REPOSITORY_ROOT / "src"
if str(SRC_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SRC_DIRECTORY))

from douglas_dart.pipeline import run_all_analyses  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run all implemented Douglas Dart propulsion and sizing analyses, write "
            "CSV/JSON results, generate plots, build the OpenVSP model, and run the "
            "configured VSPAERO sweep."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs" / "shared_nozzle_candidate_a.yaml",
    )
    parser.add_argument("--fuels", type=Path, default=None)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPOSITORY_ROOT
        / "results"
        / "generated"
        / "shared_nozzle_candidate_a",
    )
    parser.add_argument(
        "--openvsp-model",
        type=Path,
        default=REPOSITORY_ROOT
        / "openvsp"
        / "generated"
        / "shared_nozzle_candidate_a.vsp3",
    )
    parser.add_argument(
        "--body-diameter-max",
        type=float,
        default=0.300,
        help=(
            "upper numerical bound for the diameter trade in meters; this is an "
            "analysis sweep bound, not a vehicle requirement"
        ),
    )
    parser.add_argument("--body-diameter-step", type=float, default=0.005)
    parser.add_argument("--altitude-min", type=float, default=3000.0)
    parser.add_argument("--altitude-max", type=float, default=6500.0)
    parser.add_argument("--altitude-step", type=float, default=500.0)
    parser.add_argument("--propulsion-derate", type=float, default=0.15)
    parser.add_argument(
        "--skip-openvsp",
        action="store_true",
        help="skip .vsp3 generation; useful for Python-only development environments",
    )
    parser.add_argument(
        "--skip-vspaero",
        action="store_true",
        help="build the .vsp3 file but skip the live VSPAERO point sweep",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    summary = run_all_analyses(
        args.config,
        fuels_path=args.fuels,
        output_directory=args.output_dir,
        openvsp_model_path=args.openvsp_model,
        body_diameter_max_m=args.body_diameter_max,
        body_diameter_step_m=args.body_diameter_step,
        altitude_min_m=args.altitude_min,
        altitude_max_m=args.altitude_max,
        altitude_step_m=args.altitude_step,
        propulsion_derate_fraction=args.propulsion_derate,
        run_openvsp=not args.skip_openvsp,
        run_vspaero=not args.skip_vspaero,
    )
    print(json.dumps(asdict(summary), indent=2))
    return 0 if summary.successful else 1


if __name__ == "__main__":
    raise SystemExit(main())
