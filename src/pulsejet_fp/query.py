"""Public query function: pulsejet_thrust(...) -> ThrustResult
(derivation.md #13), plus the FP-1 reference design and cycle analysis.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .engine import (IntakeDesign, Numerics, PulsejetEngine, StartCondition,
                     TurbulenceParams)
from .gas import GasModel, propane_air
from .geometry import EngineGeometry
from .valve import PetalValveDesign


# ---------------------------------------------------------------------------
# FP-1 reference design: a "small valved" class engine (hobby scale), sized
# from physics in derivation.md (quarter-wave/Helmholtz resonance ~150 Hz,
# intake open-area ~35% of chamber cross-section, petal natural frequency
# ~2.5-3x cycle frequency). All numbers chosen, none copied.
# ---------------------------------------------------------------------------

def reference_geometry() -> EngineGeometry:
    return EngineGeometry(
        chamber_diameter=0.078,
        chamber_length=0.150,
        cone_length=0.130,
        tailpipe_diameter=0.042,
        tailpipe_length=0.620,
    )


def reference_valve() -> PetalValveDesign:
    return PetalValveDesign(
        n_petals=9,
        petal_length=0.020,
        petal_width=0.014,
        petal_thickness=0.20e-3,
        port_area=0.012 * 0.0155,   # 12 x 15.5 mm port under each petal
        youngs_modulus=200e9,
        density=7850.0,
        damping_ratio=0.03,
        restitution=0.3,
        max_lift=3.5e-3,
    )


def reference_gas(phi: float = 1.0) -> GasModel:
    return propane_air(phi=phi)


# ---------------------------------------------------------------------------

@dataclass
class ThrustResult:
    thrust_n: float
    status: str                    # converged | unconverged | quenched | diverged
    frequency_hz: float = float("nan")
    thrust_surface_n: float = float("nan")   # eq. 25 cross-check
    mdot_air_kg_s: float = float("nan")
    mdot_fuel_kg_s: float = float("nan")
    tsfc_kg_per_n_hr: float = float("nan")
    p_min_ratio: float = float("nan")
    p_max_ratio: float = float("nan")
    rayleigh_index: float = float("nan")     # J (proxy: head p' x global q')
    n_cycles: int = 0
    mach: float = 0.0
    traces: dict | None = None


def _upcrossings(t, y, level):
    s = y - level
    idx = np.where((s[:-1] <= 0.0) & (s[1:] > 0.0))[0]
    if len(idx) == 0:
        return np.array([])
    frac = -s[idx] / (s[idx + 1] - s[idx])
    return t[idx] + frac * (t[idx + 1] - t[idx])


def _seg_stats(t, y, t0, t1):
    m = (t >= t0) & (t <= t1)
    if m.sum() < 4:
        return None
    return np.trapezoid(y[m], t[m]) / (t1 - t0)


def analyze_cycles(hist: dict, p_a: float, gas: GasModel,
                   n_avg: int = 6, settle_frac: float = 0.35):
    """Cycle detection + limit-cycle averaging (derivation.md #13.4-5)."""
    t = hist["t"]
    p1 = hist["p_head"]
    out = {"n_cycles": 0, "converged": False}
    if len(t) < 100:
        return out
    t_settle = t[0] + settle_frac * (t[-1] - t[0])
    m = t >= t_settle
    if m.sum() < 100:
        return out
    level = float(np.mean(p1[m]))
    tc = _upcrossings(t[m], p1[m], level)
    if len(tc) < 3:
        return out
    periods = np.diff(tc)
    ok = periods > 1e-4  # ignore sub-0.1ms jitter crossings
    tc = np.concatenate([[tc[0]], tc[1:][ok]])
    periods = np.diff(tc)
    n_cyc = len(periods)
    out["n_cycles"] = n_cyc
    if n_cyc < 3:
        return out

    k = min(n_avg, n_cyc)
    t0, t1 = tc[-k - 1], tc[-1]
    win = (t >= t0) & (t <= t1)

    F_cyc, per = [], periods[-k:]
    q = hist["q_tot"]
    for a, b in zip(tc[-k - 1:-1], tc[-k:]):
        F_cyc.append(_seg_stats(t, hist["F_mom"], a, b))
    F_cyc = np.array([f for f in F_cyc if f is not None])

    thrust = float(np.trapezoid(hist["F_mom"][win], t[win]) / (t1 - t0))
    thrust_surf = float(np.trapezoid(hist["F_surf"][win], t[win]) / (t1 - t0))
    mdot_air_mix = float(np.trapezoid(np.maximum(hist["mdot_v"][win], 0.0),
                                      t[win]) / (t1 - t0))
    mdot_fuel = mdot_air_mix * gas.f / (1.0 + gas.f)
    per_rel = float(np.std(per) / np.mean(per))
    if len(F_cyc) >= 3:
        # absolute floor ~0.5% of p_a * A_e (exit-plane force scale, ~0.7 N)
        f_tol = max(0.05 * abs(np.mean(F_cyc)), 0.75)
        conv = per_rel < 0.02 and float(np.std(F_cyc)) < f_tol
    else:
        conv = False

    # Rayleigh proxy (derivation.md #10): head-pressure vs global heat release
    pw, qw, tw = p1[win], q[win], t[win]
    pbar = np.trapezoid(pw, tw) / (t1 - t0)
    qbar = np.trapezoid(qw, tw) / (t1 - t0)
    rayleigh = float(np.trapezoid((pw - pbar) * (qw - qbar), tw) / k)

    out.update(
        converged=bool(conv),
        thrust=thrust, thrust_surf=thrust_surf,
        frequency=float(1.0 / np.mean(per)),
        mdot_air_mix=mdot_air_mix, mdot_fuel=mdot_fuel,
        p_min_ratio=float(np.min(p1[win]) / p_a),
        p_max_ratio=float(np.max(p1[win]) / p_a),
        rayleigh=rayleigh, period_rel_spread=per_rel,
    )
    return out


def pulsejet_thrust(mach: float = 0.0, altitude_m: float = 0.0, *,
                    gas: GasModel | None = None,
                    geom: EngineGeometry | None = None,
                    valve: PetalValveDesign | None = None,
                    numerics: Numerics | None = None,
                    turb: TurbulenceParams | None = None,
                    start: StartCondition | None = None,
                    intake: IntakeDesign | None = None,
                    t_end: float = 0.30,
                    keep_traces: bool = False) -> ThrustResult:
    """Primary query (derivation.md #13): cycle-averaged thrust at a flight
    condition, from the transient first-principles simulation."""
    gas = gas or reference_gas()
    geom = geom or reference_geometry()
    valve = valve or reference_valve()

    eng = PulsejetEngine(gas, geom, valve, mach=mach, altitude_m=altitude_m,
                         numerics=numerics, turb=turb, start=start,
                         intake=intake)
    hist = eng.run(t_end)

    if eng.status == "diverged":
        return ThrustResult(thrust_n=float("nan"), status="diverged",
                            mach=mach, traces=hist if keep_traces else None)

    an = analyze_cycles(hist, eng.p_a, gas)

    # quench check: no sustained hot zone in the last quarter of the run
    tail = hist["t"] > hist["t"][-1] - 0.25 * (hist["t"][-1] - hist["t"][0])
    quenched = bool(np.max(hist["T_max"][tail]) < 900.0)

    if "thrust" not in an:
        status = "quenched" if quenched else "unconverged"
        return ThrustResult(thrust_n=float("nan"), status=status, mach=mach,
                            n_cycles=an.get("n_cycles", 0),
                            traces=hist if keep_traces else None)

    status = "quenched" if quenched else (
        "converged" if an["converged"] else "unconverged")
    mdot_fuel = an["mdot_fuel"]
    thrust = an["thrust"]
    tsfc = mdot_fuel * 3600.0 / thrust if thrust > 1e-6 else float("nan")
    return ThrustResult(
        thrust_n=thrust, status=status,
        frequency_hz=an["frequency"], thrust_surface_n=an["thrust_surf"],
        mdot_air_kg_s=an["mdot_air_mix"] - mdot_fuel,
        mdot_fuel_kg_s=mdot_fuel, tsfc_kg_per_n_hr=tsfc,
        p_min_ratio=an["p_min_ratio"], p_max_ratio=an["p_max_ratio"],
        rayleigh_index=an["rayleigh"], n_cycles=an["n_cycles"], mach=mach,
        traces=hist if keep_traces else None,
    )
