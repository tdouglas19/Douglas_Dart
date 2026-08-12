"""Fidelity contract for the numba kernels: the JIT paths must reproduce
the numpy reference paths to floating-point roundoff on a stressing state
(shocks, contacts, area variation, active reaction)."""
import numpy as np
import pytest

from pulsejet_fp import solver, solver_jit
from pulsejet_fp.gas import propane_air

pytestmark = pytest.mark.skipif(not solver_jit.HAS_NUMBA,
                                reason="numba not installed")


def stressing_state(gas, n=180, seed=7):
    rng = np.random.default_rng(seed)
    x = np.linspace(0.0, 1.0, n)
    A = 4e-3 - 3e-3 * 0.5 * (1 - np.cos(np.pi * np.clip((x - 0.2) / 0.3,
                                                        0, 1)))
    A_f = np.concatenate([[A[0]], 0.5 * (A[:-1] + A[1:]), [A[-1]]])
    rho = 0.3 + 1.2 * rng.random(n)
    u = 250.0 * (rng.random(n) - 0.5)
    p = 5e4 + 1.5e5 * rng.random(n)
    Y = np.clip(rng.random(n) * 1.4 - 0.2, 0.0, 1.0)
    # sharpen: add a shock-like jump and a contact
    p[n // 3:] *= 2.0
    Y[: n // 2] = 1.0
    T = p / (rho * gas.R_mix(Y))
    # make a hot pocket so the reaction fires
    T[2 * n // 3:] = 1500.0
    rho2 = p / (gas.R_mix(Y) * T)
    U = solver.conserved(rho2, u, p, Y, gas, A)
    nu = 0.05 + 0.3 * rng.random(n)
    return U, A, A_f, nu


def test_primitives_match():
    gas = propane_air()
    U, A, A_f, nu = stressing_state(gas)
    ref = solver.primitives(U, A, gas)
    jit = solver_jit.primitives_jit(U, A, gas.cv_R, gas.cv_P,
                                    gas.R_R, gas.R_P)
    for r, j in zip(ref, jit):
        assert np.allclose(r, j, rtol=1e-12, atol=1e-12)


def test_rhs_match():
    gas = propane_air()
    U, A, A_f, nu = stressing_state(gas)
    dx = 1.0 / U.shape[1]
    prims = solver.primitives(U, A, gas)
    F_head = np.array([0.02, 150.0 * A_f[0], 6000.0, 0.02])
    F_exit = np.array([0.5, 9e4 * A_f[-1], 2e5, 0.1])
    ref, _ = solver.hyperbolic_rhs(U, A, A_f, dx, gas, F_head, F_exit, nu,
                                   prims=prims)
    jit = solver_jit.rhs_jit(U, A, A_f, dx, gas.cv_R, gas.cv_P, gas.R_R,
                             gas.R_P, 0.0, F_head, F_exit, nu, *prims)
    scale = np.max(np.abs(ref), axis=1, keepdims=True) + 1e-30
    assert np.max(np.abs(ref - jit) / scale) < 1e-10


def test_reaction_match():
    gas = propane_air()
    U, A, A_f, nu = stressing_state(gas)
    dt = 1.5e-6
    quench = np.clip(np.linspace(0.0, 1.2, U.shape[1]), 0.0, 1.0)

    U_ref = U.copy()
    prims_ref = solver.primitives(U_ref, A, gas)
    h_ref, q_ref = solver.reaction_substep(U_ref, A, gas, dt, quench,
                                           prims=prims_ref)

    U_jit = U.copy()
    p = solver.primitives(U_jit, A, gas)
    h_jit, q_jit = solver_jit.reaction_jit(
        U_jit, A, gas.cv_R, gas.cv_P, gas.R_R, gas.R_P,
        gas.A_r, gas.T_a, gas.q_R, dt, quench, p[0], p[4], p[3])

    assert np.allclose(U_ref, U_jit, rtol=1e-11)
    assert np.allclose(q_ref, q_jit, rtol=1e-9, atol=1e-12)
    assert abs(h_ref - h_jit) <= 1e-9 * max(abs(h_ref), 1.0)
