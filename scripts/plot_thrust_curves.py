"""Plot pulsejet/ramjet net thrust vs Mach, and the fast-vs-full pulsejet
integration-step-size (dt) discrepancy, for visual (non-physical-behavior)
inspection.

Reads the same evaluate_propulsion_map_point()/evaluate_total_drag() the model
itself uses -- these are not independent calculations, just the model's own
numbers laid out on an axis. Writes PNGs to results/generated/thrust_plots/.

Previously compared fast/full *warmup_s+measurement_s windows* as well as dt.
That window no longer exists (cycle_based_averaging_fix.md, 2026-08-08): the
pulsejet path now runs to a converged real-cycle-boundary average regardless
of caller. dt itself is also no longer a raw caller-chosen float
(dt_convergence_solver_spec.md, 2026-08-08) -- PULSEJET_FIDELITY_FAST/FULL
select between the two fixed, named tiers instead. Dense Mach sampling
(0.01 step, 100 points) and the convergence-driven averaging (up to 100
cycles per point, checking both running-mean stability AND raw per-cycle
swing, not a handful of cycles at a mean-only check) are both intentional
here -- this script exists specifically to check whether the sawtooth
jaggedness the fixed window (and later the mean-only convergence check) used
to produce is actually gone. Expect this run to take considerably longer than
earlier sessions -- that is the direct, known cost of both fixes, not a
regression.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import matplotlib.pyplot as plt

from douglas_dart.config import load_reference_case
from douglas_dart.drag import evaluate_total_drag
from douglas_dart.mass_model import calibrate_mass_model
from douglas_dart.optimizer import DesignVariables, apply_design_variables
from douglas_dart.propulsion_map import (
    NOMINAL,
    PULSEJET_FIDELITY_FAST,
    PULSEJET_FIDELITY_FULL,
    PULSEJET_MODE,
    RAMJET_MODE,
    evaluate_propulsion_map_point,
)

OUT_DIR = Path("results/generated/thrust_plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)

CASE = load_reference_case("configs/shared_nozzle_candidate_b.yaml", None)
CAL = calibrate_mass_model(CASE, "configs/robustness_candidate_b.yaml")

checkpoint_path = Path("results/generated/design_optimize_v8/checkpoint.json")
with checkpoint_path.open() as f:
    checkpoint = json.load(f)
variables = DesignVariables(**checkpoint[-1]["best_variables"])
candidate = apply_design_variables(CASE, variables, CAL)

mach_values = [i * 0.01 for i in range(1, 101)]  # 0.01 .. 1.00


def pulsejet_curve(pulsejet_fidelity: str) -> list[float]:
    thrusts = []
    for i, mach in enumerate(mach_values):
        t0 = time.time()
        point = evaluate_propulsion_map_point(
            candidate, mach, 0.0, PULSEJET_MODE,
            scenario=NOMINAL,
            pulsejet_fidelity=pulsejet_fidelity,
        )
        thrusts.append(point.net_thrust_n)
        print(
            f"  [{pulsejet_fidelity}] [{i + 1}/{len(mach_values)}] mach={mach:.2f} "
            f"thrust={thrusts[-1]:.2f}N  ({time.time() - t0:.1f}s)",
            flush=True,
        )
    return thrusts


def ramjet_curve() -> list[float]:
    thrusts = []
    for mach in mach_values:
        point = evaluate_propulsion_map_point(
            candidate, mach, 0.0, RAMJET_MODE, scenario=NOMINAL,
        )
        thrusts.append(point.net_thrust_n)
    return thrusts


def drag_curve() -> list[float]:
    drags = []
    weight_n = candidate.flight.initial_mass_kg * 9.80665
    for mach in mach_values:
        # Rough level-flight CL for an illustrative drag line (not a trimmed solve).
        speed_guess = max(mach * 340.0, 1.0)
        q_guess = 0.5 * 1.225 * speed_guess**2
        cl = min(weight_n / (q_guess * candidate.flight.reference_area_m2), candidate.flight.maximum_lift_coefficient)
        breakdown = evaluate_total_drag(
            candidate, candidate.flight, 0.0, mach, cl,
        )
        drags.append(breakdown.total_drag_n)
    return drags


fast_pulsejet = pulsejet_curve(PULSEJET_FIDELITY_FAST)
full_pulsejet = pulsejet_curve(PULSEJET_FIDELITY_FULL)
ramjet_thrust = ramjet_curve()
drag = drag_curve()

# --- Plot 1: pulsejet fast vs full dt (no window left to compare) ---
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(mach_values, fast_pulsejet, label="fast: dt=1e-4s (search's setting)", color="tab:red")
ax.plot(mach_values, full_pulsejet, label="full: dt=6.25e-6s (verification setting)", color="tab:blue")
ax.axhline(0.0, color="black", linewidth=0.8)
ax.set_xlabel("Mach")
ax.set_ylabel("Net thrust (N)")
ax.set_title(
    "Pulsejet net thrust vs Mach: fast vs full dt, converged cycle averaging\n"
    "(v8 current-best candidate, sea level, nominal scenario)"
)
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT_DIR / "pulsejet_fast_vs_full_fidelity.png", dpi=150)
plt.close(fig)

# --- Plot 2: pulsejet (full) + ramjet + drag, the "is this physical" chart ---
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(mach_values, full_pulsejet, label="pulsejet net thrust (full fidelity)", color="tab:blue")
ax.plot(mach_values, ramjet_thrust, label="ramjet net thrust", color="tab:green")
ax.plot(mach_values, drag, label="total drag (rough level-flight CL)", color="tab:orange", linestyle="--")
ax.axvline(candidate.ramjet.minimum_lightoff_test_mach, color="gray", linestyle=":", label="minimum_lightoff_test_mach")
ax.axhline(0.0, color="black", linewidth=0.8)
ax.set_xlabel("Mach")
ax.set_ylabel("Force (N)")
ax.set_title("Thrust vs drag, both engine modes (v8 current-best candidate, sea level)")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT_DIR / "thrust_vs_drag_both_engines.png", dpi=150)
plt.close(fig)

print(f"Wrote plots to {OUT_DIR.resolve()}")
for p in sorted(OUT_DIR.glob("*.png")):
    print(" -", p)
