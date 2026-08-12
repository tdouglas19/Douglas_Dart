"""Engine smoke + physics-behavior tests (short, coarse runs)."""
import numpy as np

from pulsejet_fp import (Numerics, PulsejetEngine, reference_gas,
                         reference_geometry, reference_valve)


def short_engine(mach=0.0, n_cells=120):
    return PulsejetEngine(reference_gas(), reference_geometry(),
                          reference_valve(), mach=mach,
                          numerics=Numerics(n_cells=n_cells))


def test_smoke_run_finite():
    eng = short_engine()
    hist = eng.run(0.02)
    assert eng.status in ("completed", "running")
    assert np.all(np.isfinite(hist["p_head"]))
    assert np.all(hist["mass_total"] > 0)


def test_blowdown_then_suction_opens_valve():
    """The hot start must blow down, drop below ambient, and pull the
    valve open -- the Kadenacy effect emerging from the equations."""
    eng = short_engine()
    hist = eng.run(0.03)
    assert np.min(hist["p_head"]) < 0.97 * eng.p_a   # suction happened
    assert np.max(hist["lift"]) > 1e-4               # valve opened
    assert np.max(hist["mdot_v"]) > 0.0              # charge admitted


def test_fresh_charge_enters_and_burns():
    eng = short_engine()
    hist = eng.run(0.04)
    assert np.max(hist["reactant_mass"]) > 1e-6      # reactant present
    assert np.max(hist["q_tot"]) > 1e3               # combustion occurred


def test_mass_bookkeeping_closes():
    """d(mass)/dt must equal net boundary flux, integrated: mass change
    = swallowed - exhausted (within integration tolerance)."""
    eng = short_engine()
    hist = eng.run(0.02)
    t = hist["t"]
    dm = hist["mass_total"][-1] - hist["mass_total"][0]
    inflow = np.trapezoid(hist["mdot_v"], t)
    outflow = np.trapezoid(hist["mdot_exit"], t)
    residual = dm - (inflow - outflow)
    scale = max(abs(inflow), abs(outflow), 1e-9)
    assert abs(residual) / scale < 0.05
