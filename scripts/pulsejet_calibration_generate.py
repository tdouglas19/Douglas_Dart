"""Generate a pulsejet calibration dataset (Mach-only, fixed candidate geometry).

Phased calibration: this pass holds reference_case.yaml's geometry
(chamber_volume_m3, throat_diameter_m, exit_to_throat_area_ratio) fixed and
sweeps only Mach -- the immediate need (a calibrated closed-form Mach curve
for one candidate). The full multi-dimensional DOE (varying the geometry
Gate 3 actually searches) is the next phase, not done here.

Was originally written against a raw `pulsejet_warmup_s`/`pulsejet_measurement_s`
API that this session's cycle-boundary convergent-averaging work
(docs/design_convergence.md, "Cycle-based convergent averaging") retired in
favor of two named fidelity tiers -- that raw-float API no longer exists on
`evaluate_propulsion_map_point`, so this script would not run as originally
written. Updated to request `PULSEJET_FIDELITY_FULL`, which chases genuine
per-cycle convergence (running-mean stability AND raw per-cycle swing, up to
100 cycles) rather than trusting any fixed wall-clock window -- a strictly
better calibration target than the wide-but-still-fixed 0.5s/1.0s window this
script used before, not just an API-compatibility patch. This also means the
resulting dataset (and everything fit from it) reflects this session's
nozzle-onset-smoothing, side-inlet-ram-recovery, and low-Mach duty-cycle
corrections, none of which were in effect when the previous calibration
(configs/pulsejet_closed_form_calibration.yaml, train RMSE 24.6N) was fit --
that file is stale and must be regenerated from this script's new output.
"""

from __future__ import annotations

import csv
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import (
    NOMINAL,
    PULSEJET_FIDELITY_FULL,
    PULSEJET_MODE,
    evaluate_propulsion_map_point,
)

OUT_PATH = Path("results/generated/pulsejet_calibration/mach_sweep_reference_case.csv")
OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

CASE = load_reference_case("configs/reference_case.yaml")
ALTITUDE_M = 0.0
MACH_VALUES = [round(i * 0.05, 2) for i in range(0, 21)]  # 0.00 .. 1.00

rows = []
for mach in MACH_VALUES:
    point = evaluate_propulsion_map_point(
        CASE, mach, ALTITUDE_M, PULSEJET_MODE,
        scenario=NOMINAL, pulsejet_fidelity=PULSEJET_FIDELITY_FULL,
    )
    rows.append(
        {
            "mach": mach,
            "altitude_m": ALTITUDE_M,
            "net_thrust_n": point.net_thrust_n,
            "fuel_mass_flow_kg_per_s": point.fuel_mass_flow_kg_per_s,
        }
    )
    print(f"mach={mach:.2f}  thrust={point.net_thrust_n:8.2f}N  fuel_flow={point.fuel_mass_flow_kg_per_s:.5f}kg/s", flush=True)

with OUT_PATH.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["mach", "altitude_m", "net_thrust_n", "fuel_mass_flow_kg_per_s"])
    writer.writeheader()
    writer.writerows(rows)

print(f"\nWrote {OUT_PATH.resolve()}")
