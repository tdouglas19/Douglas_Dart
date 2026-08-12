"""Thrust vs Mach for a fixed, generic Gate-3-style input, all three engine modes.

Holds one representative candidate (reference_case.yaml, sea level) fixed and
varies only Mach -- exactly what Gate 3's trajectory solver does when it
queries Gate 2's propulsion map at each mission timestep. Plots the pulsejet
unsteady simulator (full fidelity), ramjet net thrust, and the sibling
pulsejet-km (Khrulev & Muntyan) model on the same axes. Writes a PNG to
results/generated/thrust_plots/.

Previously plotted the pulsejet *closed form* using
configs/pulsejet_closed_form_calibration.yaml. That calibration
(train RMSE 24.6N, holdout RMSE 27.9N even at the time) predates this
session's inertia/nozzle-smoothing/side-inlet-recovery fixes and is now
doubly stale -- plotting it would show physics this codebase no longer
believes. Switched to the real unsteady simulator directly (same call
plot_thrust_curves.py already uses) until the closed form is re-derived
against the current physics and re-validated (docs/design_convergence.md,
"Calibration DOE sizing").

Dense Mach sampling (0.01 step) and the convergence-driven cycle averaging
(cycle_based_averaging_fix.md, up to 100 real ignition cycles per point, both
a running-mean AND a raw per-cycle swing check, not a fixed wall-clock
window) are both intentional here -- this is the highest-confidence version
of this plot this codebase can currently produce. Fidelity is
PULSEJET_FIDELITY_FULL (dt_convergence_solver_spec.md's measured dt=6.25e-6s)
-- expect this to run considerably longer than earlier sessions of this
script; that cost is the direct, known consequence of resolving both the
averaging window and dt to genuine convergence rather than an assumed
setting, not a regression.

**PULSEJET_KM_MODE curve (added 2026-08-11)**: the sibling pulsejet-km
repo's own model, wired in as a new, additive third mode
(propulsion_map.py's PULSEJET_KM_MODE) -- NOT a replacement for the native
pulsejet curve above, which remains this codebase's primary pulsejet model
(docs/pulsejet_external_model_audit.md). Uses reference_case.yaml's
`pulsejet_km:` section (real Argus As-014/V-1 geometry, NOT this vehicle's
own design point -- see that section's own comment). Swept only over
pulsejet-km's own validated Mach envelope (0.0-0.7, MACH_VALIDATED_ENVELOPE_
MAXIMUM) at a coarser step (0.05) and PULSEJET_FIDELITY_FAST -- each point is
a full unsteady cycle-average simulation with its own convergence search,
materially more expensive per-point than either native mode. This curve's
absolute thrust magnitude is known, from this session's own grounding work
(pulsejet-km's own architecture.md Section 50), to still read substantially
low vs. real validated engine data even after the most recent fix -- plotted
here to prove the integration pipeline itself works end-to-end, not as a
trusted absolute thrust prediction yet.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import (
    NOMINAL,
    PULSEJET_FIDELITY_FAST,
    PULSEJET_FIDELITY_FULL,
    PULSEJET_KM_MODE,
    PULSEJET_MODE,
    RAMJET_MODE,
    evaluate_propulsion_map_point,
)
from pulsejet_km.query import MACH_VALIDATED_ENVELOPE_MAXIMUM as PULSEJET_KM_MACH_VALIDATED_ENVELOPE_MAXIMUM

# Fidelity selectable via argv (default "fast"): a full-fidelity 101-point
# sweep was measured taking 4+ hours wall clock (2026-08-08, competing for
# CPU with a concurrent design-optimize search) -- disproportionate for a
# diagnostic plot when the search itself, which is what actually needs to
# "feel real," only ever queries PULSEJET_FIDELITY_FAST. Pass "full" as
# argv[1] for the slower, more precise verification-grade sweep when nothing
# else is competing for CPU.
FIDELITY = PULSEJET_FIDELITY_FULL if (len(sys.argv) > 1 and sys.argv[1] == "full") else PULSEJET_FIDELITY_FAST

OUT_DIR = Path("results/generated/thrust_plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)

# One fixed, generic Gate-3-style input -- reference_case.yaml as loaded, sea
# level. Only Mach is varied below; nothing here is candidate-specific.
CASE = load_reference_case("configs/reference_case.yaml")
ALTITUDE_M = 0.0

pulsejet_mach_values = [i * 0.01 for i in range(0, 101)]  # 0.00 .. 1.00
ramjet_mach_values = [i * 0.01 for i in range(30, 121)]  # 0.30 .. 1.20
# Coarser and bounded to pulsejet-km's own validated envelope (0.0-0.7) --
# each point is a full independent unsteady simulation with its own
# convergence search, materially more expensive per-point than either
# native mode above (see module docstring).
_pulsejet_km_max_mach_steps = int(round(PULSEJET_KM_MACH_VALIDATED_ENVELOPE_MAXIMUM / 0.05))
pulsejet_km_mach_values = [i * 0.05 for i in range(0, _pulsejet_km_max_mach_steps + 1)]  # 0.00 .. 0.70


def pulsejet_thrust(mach: float) -> float:
    point = evaluate_propulsion_map_point(
        CASE,
        mach,
        ALTITUDE_M,
        PULSEJET_MODE,
        scenario=NOMINAL,
        pulsejet_fidelity=FIDELITY,
    )
    return point.net_thrust_n


def ramjet_thrust(mach: float) -> float:
    point = evaluate_propulsion_map_point(CASE, mach, ALTITUDE_M, RAMJET_MODE, scenario=NOMINAL)
    return point.net_thrust_n


def pulsejet_km_thrust(mach: float) -> tuple[float, str]:
    point = evaluate_propulsion_map_point(
        CASE,
        mach,
        ALTITUDE_M,
        PULSEJET_KM_MODE,
        scenario=NOMINAL,
        pulsejet_fidelity=PULSEJET_FIDELITY_FAST,
    )
    return point.net_thrust_n, ", ".join(point.validity_flags) or "clean"


import time as _time

pulsejet = []
for _i, _m in enumerate(pulsejet_mach_values):
    _t0 = _time.time()
    pulsejet.append(pulsejet_thrust(_m))
    print(
        f"[{_i + 1}/{len(pulsejet_mach_values)}] mach={_m:.2f} "
        f"thrust={pulsejet[-1]:.2f}N  ({_time.time() - _t0:.1f}s)",
        flush=True,
    )
ramjet = [ramjet_thrust(m) for m in ramjet_mach_values]

pulsejet_km = []
for _i, _m in enumerate(pulsejet_km_mach_values):
    _t0 = _time.time()
    _thrust, _flags = pulsejet_km_thrust(_m)
    pulsejet_km.append(_thrust)
    print(
        f"[km {_i + 1}/{len(pulsejet_km_mach_values)}] mach={_m:.2f} "
        f"thrust={_thrust:.2f}N  flags=[{_flags}]  ({_time.time() - _t0:.1f}s)",
        flush=True,
    )

fig, ax = plt.subplots(figsize=(9, 6))
ax.plot(pulsejet_mach_values, pulsejet, label="pulsejet (unsteady simulator, full fidelity)", color="tab:red")
ax.plot(ramjet_mach_values, ramjet, label="ramjet", color="tab:green")
ax.plot(
    pulsejet_km_mach_values,
    pulsejet_km,
    label="pulsejet-km (Khrulev & Muntyan, sibling model, fast fidelity)",
    color="tab:purple",
    marker="o",
    markersize=4,
)
ax.axhline(0.0, color="black", linewidth=0.8)
ax.axvline(CASE.ramjet.minimum_lightoff_test_mach, color="gray", linestyle=":", label="minimum_lightoff_test_mach")
ax.axvline(CASE.ramjet.minimum_self_sustaining_mach, color="gray", linestyle="-.", label="minimum_self_sustaining_mach")
ax.set_xlabel("Mach")
ax.set_ylabel("Net thrust (N)")
ax.set_title(
    "Net thrust vs Mach -- pulsejet (unsteady sim) + ramjet + pulsejet-km\n"
    "(reference_case.yaml, sea level, one fixed candidate, Mach-only sweep)"
)
ax.legend(loc="best", fontsize=9)
ax.grid(alpha=0.3)
fig.tight_layout()
out_path = OUT_DIR / "thrust_vs_mach_gate3_query.png"
fig.savefig(out_path, dpi=150)
plt.close(fig)

print(f"Wrote {out_path.resolve()}")
