"""Run find_converged_time_step across a representative test set and report.

See dt_convergence_solver_spec.md (2026-08-08). Representative set spans:
  - a low-Mach point (M<0.2)
  - a point near the pulsejet duty-cycle regime transition (~M=0.2)
  - a mid-Mach strong-thrust point (M=0.5)
  - a Mach 0.9-1.0 tail point, where fast-vs-full dt diverged most visibly
  - the same mid-Mach point on two other geometry candidates (shared_nozzle_
    candidate_a.yaml, shared_nozzle_candidate_b.yaml), not just
    reference_case.yaml, since dt sensitivity could plausibly vary with
    chamber volume or intake diameter.

Re-runnable, not a one-off: run this again if Gate 3's search later converges
toward a design region very different from today's candidates, to re-validate
the fixed dt constant rather than assuming today's answer holds forever.
"""

from __future__ import annotations

import time

from douglas_dart.config import load_reference_case
from douglas_dart.pulsejet import (
    DtConvergenceHalvingRecord,
    DtConvergenceTestPoint,
    find_converged_time_step,
)

REFERENCE = load_reference_case("configs/reference_case.yaml")
CANDIDATE_A = load_reference_case("configs/shared_nozzle_candidate_a.yaml")
CANDIDATE_B = load_reference_case("configs/shared_nozzle_candidate_b.yaml")


def _point(label: str, case, mach: float) -> DtConvergenceTestPoint:
    return DtConvergenceTestPoint(
        label=label,
        config=case.pulsejet,
        selector=case.selector,
        nozzle=case.nozzle,
        fuel=case.fuel,
        altitude_m=case.mission.field_elevation_msl_m,
        mach=mach,
    )


TEST_POINTS = [
    _point("reference_case low-Mach M=0.10", REFERENCE, 0.10),
    _point("reference_case regime-transition M=0.20", REFERENCE, 0.20),
    _point("reference_case mid-strong M=0.50", REFERENCE, 0.50),
    _point("reference_case tail M=0.95", REFERENCE, 0.95),
    _point("shared_nozzle_candidate_a mid-strong M=0.50", CANDIDATE_A, 0.50),
    _point("shared_nozzle_candidate_b mid-strong M=0.50", CANDIDATE_B, 0.50),
]


def _print_progress(label: str, record: DtConvergenceHalvingRecord) -> None:
    flag = "" if record.inner_converged else "  <-- DID NOT CONVERGE"
    elapsed = time.strftime("%H:%M:%S")
    print(
        f"[{elapsed}] {label}: dt={record.time_step_s:.3e}s "
        f"net_thrust={record.mean_net_thrust_n:.3f}N "
        f"cycles={record.inner_cycles}{flag}",
        flush=True,
    )


def main() -> None:
    report = find_converged_time_step(TEST_POINTS, on_halving=_print_progress)

    print(f"\nWall-clock cost: {report.wall_clock_cost_s:.1f}s\n")
    for point_result in report.point_results:
        print(f"=== {point_result.label} (Mach {point_result.mach}) ===")
        for halving in point_result.halvings:
            flag = "" if halving.inner_converged else "  <-- INNER LOOP DID NOT CONVERGE"
            print(
                f"  dt={halving.time_step_s:.3e}s  "
                f"net_thrust={halving.mean_net_thrust_n:.3f}N  "
                f"cycles={halving.inner_cycles}{flag}"
            )
        if point_result.converged_time_step_s is not None:
            print(f"  -> converged dt: {point_result.converged_time_step_s:.3e}s")
        else:
            print("  -> DID NOT CONVERGE within the halving cap")
        if point_result.inner_loop_needed_extended_cycles:
            print("  -> NOTE: needed more cycles than the production cap at some dt")
        print()

    print("=" * 60)
    if report.recommended_time_step_s is not None:
        print(f"Recommended fixed dt (finest required across all points): "
              f"{report.recommended_time_step_s:.3e}s")
    else:
        print("NO POINT CONVERGED -- no dt can be recommended from this set.")
    if report.unconverged_point_labels:
        print(f"Unconverged points: {', '.join(report.unconverged_point_labels)}")


if __name__ == "__main__":
    main()
