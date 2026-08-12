"""Solver validation: Sod shock tube vs the exact Riemann solution (computed
here from the Rankine-Hugoniot and isentropic relations, first principles),
closed-box conservation, and acoustic standing-wave frequency.
"""
import math

import numpy as np

from pulsejet_fp import solver
from pulsejet_fp.gas import propane_air


def inert_gas():
    """Single-species-like gas: run everything at Y=1 (gamma=1.4), A_r=0."""
    return propane_air(phi=1.0, A_r=0.0)


# ---------------------------------------------------------------------------
# exact Riemann solver (two-wave, ideal gas) -- Newton on p*
# ---------------------------------------------------------------------------

def _f_wave(p, pK, rhoK, g):
    aK = math.sqrt(g * pK / rhoK)
    if p > pK:  # shock: Rankine-Hugoniot
        A = 2.0 / ((g + 1.0) * rhoK)
        B = (g - 1.0) / (g + 1.0) * pK
        return (p - pK) * math.sqrt(A / (p + B))
    # rarefaction: isentropic + Riemann invariant
    return 2.0 * aK / (g - 1.0) * ((p / pK) ** ((g - 1.0) / (2 * g)) - 1.0)


def exact_riemann(rhoL, uL, pL, rhoR, uR, pR, g):
    p = 0.5 * (pL + pR)
    for _ in range(60):
        f = _f_wave(p, pL, rhoL, g) + _f_wave(p, pR, rhoR, g) + (uR - uL)
        dp = max(1e-8 * p, 1e-9)
        fp = (_f_wave(p + dp, pL, rhoL, g) + _f_wave(p + dp, pR, rhoR, g)
              + (uR - uL) - f) / dp
        p_new = max(p - f / fp, 1e-8)
        if abs(p_new - p) < 1e-12 * p:
            p = p_new
            break
        p = p_new
    u = 0.5 * (uL + uR) + 0.5 * (_f_wave(p, pR, rhoR, g)
                                 - _f_wave(p, pL, rhoL, g))
    return p, u


def test_sod_shock_tube():
    gas = inert_gas()
    g = gas.gamma_R
    N = 400
    L = 1.0
    dx = L / N
    x = (np.arange(N) + 0.5) * dx
    A = np.ones(N)
    A_f = np.ones(N + 1)

    # dimensional Sod-like states (scaled to engine-like magnitudes)
    pL, pR = 1e5, 1e4
    TL = pL / (gas.R_R * 1.0)   # rhoL = 1.0
    rhoL, rhoR = 1.0, 0.125
    rho = np.where(x < 0.5, rhoL, rhoR)
    p = np.where(x < 0.5, pL, pR)
    u = np.zeros(N)
    Y = np.ones(N)
    U = solver.conserved(rho, u, p, Y, gas, A)
    nu = np.zeros(N)

    t_end = 5.0e-4
    t = 0.0
    while t < t_end:
        r_, u_, p_, Y_, T_, a_ = solver.primitives(U, A, gas)
        dt = 0.4 * dx / float(np.max(np.abs(u_) + a_))
        dt = min(dt, t_end - t)

        def F_edge(i):
            rr, uu, pp, YY = r_[i], u_[i], p_[i], Y_[i]
            EE = pp / ((g - 1.0) * rr) + 0.5 * uu * uu
            return np.array([rr * uu, rr * uu * uu + pp,
                             uu * (rr * EE + pp), rr * uu * YY])

        rhs1, _ = solver.hyperbolic_rhs(U, A, A_f, dx, gas,
                                        F_edge(0), F_edge(-1), nu)
        U1 = U + dt * rhs1
        r_, u_, p_, Y_, T_, a_ = solver.primitives(U1, A, gas)
        rhs2, _ = solver.hyperbolic_rhs(U1, A, A_f, dx, gas,
                                        F_edge(0), F_edge(-1), nu)
        U = 0.5 * U + 0.5 * (U1 + dt * rhs2)
        t += dt

    r_, u_, p_, Y_, T_, a_ = solver.primitives(U, A, gas)

    p_star, u_star = exact_riemann(rhoL, 0.0, pL, rhoR, 0.0, pR, g)
    # star-region plateau values must appear in the numerical solution
    mid = (x > 0.55) & (x < 0.60)   # contact/star region at this t
    assert abs(np.median(p_[mid]) - p_star) / p_star < 0.03
    assert abs(np.median(u_[mid]) - u_star) / u_star < 0.03

    # shock position: speed from RH mass/momentum jump across right shock
    rho_star_R = rhoR * ((p_star / pR + (g - 1) / (g + 1))
                         / ((g - 1) / (g + 1) * p_star / pR + 1.0))
    # mass conservation across the right shock (uR = 0):
    s_shock = (rho_star_R * u_star) / (rho_star_R - rhoR)
    x_shock = 0.5 + s_shock * t_end
    i_num = np.argmax(np.abs(np.diff(p_)))  # steepest jump
    assert abs(x[i_num] - x_shock) < 0.02


def _wall_flux(U, A, gas, i):
    r_, u_, p_, Y_, T_, a_ = solver.primitives(U, A, gas)
    side = "left" if i == 0 else "right"
    pw = solver.wall_pressure(float(r_[i]), float(u_[i]), float(p_[i]),
                              float(Y_[i]), gas, side=side)
    return np.array([0.0, pw * A[i], 0.0, 0.0])


def _march_closed_box(U, A, A_f, dx, gas, nu, t_end, react=False):
    t = 0.0
    heat = 0.0
    while t < t_end:
        r_, u_, p_, Y_, T_, a_ = solver.primitives(U, A, gas)
        dt = 0.35 * dx / float(np.max(np.abs(u_) + a_))
        rhs1, _ = solver.hyperbolic_rhs(U, A, A_f, dx, gas,
                                        _wall_flux(U, A, gas, 0),
                                        _wall_flux(U, A, gas, -1), nu)
        U1 = U + dt * rhs1
        rhs2, _ = solver.hyperbolic_rhs(U1, A, A_f, dx, gas,
                                        _wall_flux(U1, A, gas, 0),
                                        _wall_flux(U1, A, gas, -1), nu)
        U = 0.5 * U + 0.5 * (U1 + dt * rhs2)
        if react:
            h, _ = solver.reaction_substep(U, A, gas, dt)
            heat += h
        t += dt
    return U, heat


def test_closed_box_mass_energy_conserved():
    gas = inert_gas()
    N = 100
    dx = 0.01
    A = np.ones(N)
    A_f = np.ones(N + 1)
    x = (np.arange(N) + 0.5) * dx
    p = 1e5 * (1.0 + 0.3 * np.exp(-((x - 0.5) / 0.1) ** 2))
    T = np.full(N, 600.0)
    Y = np.ones(N)
    rho = p / (gas.R_mix(Y) * T)
    U = solver.conserved(rho, np.zeros(N), p, Y, gas, A)

    m0 = np.sum(U[0]) * dx
    E0 = np.sum(U[2]) * dx
    U, _ = _march_closed_box(U, A, A_f, dx, gas, np.zeros(N), 3e-3)
    m1 = np.sum(U[0]) * dx
    E1 = np.sum(U[2]) * dx
    assert abs(m1 - m0) / m0 < 1e-12
    assert abs(E1 - E0) / E0 < 1e-12


def test_closed_box_reaction_energy_bookkeeping():
    """Energy gain must equal q_R x reactant mass burned, exactly."""
    gas = propane_air(phi=1.0)  # reaction ON
    N = 60
    dx = 0.01
    A = np.ones(N)
    A_f = np.ones(N + 1)
    T = np.full(N, 1400.0)  # hot enough to burn
    Y = np.full(N, 0.5)
    p = np.full(N, 2e5)
    rho = p / (gas.R_mix(Y) * T)
    U = solver.conserved(rho, np.zeros(N), p, Y, gas, A)

    m0 = np.sum(U[0]) * dx
    E0 = np.sum(U[2]) * dx
    Yr0 = np.sum(U[3]) * dx
    U, _ = _march_closed_box(U, A, A_f, dx, gas, np.zeros(N), 2e-3,
                             react=True)
    m1 = np.sum(U[0]) * dx
    E1 = np.sum(U[2]) * dx
    Yr1 = np.sum(U[3]) * dx
    burned = Yr0 - Yr1
    assert burned > 0.2 * Yr0            # substantial combustion happened
    assert abs(m1 - m0) / m0 < 1e-12     # mass unaffected by reaction
    dE = E1 - E0
    assert abs(dE - gas.q_R * burned) / (gas.q_R * burned) < 1e-9


def test_acoustic_standing_wave_frequency():
    """Closed-closed tube fundamental: f = a / (2 L)."""
    gas = inert_gas()
    N = 200
    L = 1.0
    dx = L / N
    A = np.ones(N)
    A_f = np.ones(N + 1)
    x = (np.arange(N) + 0.5) * dx
    T0 = 300.0
    p0 = 1e5
    Y = np.ones(N)
    p = p0 * (1.0 + 1e-3 * np.cos(np.pi * x / L))
    rho = p / (gas.R_mix(Y) * T0)  # isothermal tiny perturbation
    U = solver.conserved(rho, np.zeros(N), p, Y, gas, A)

    a_th = math.sqrt(gas.gamma_R * gas.R_R * T0)
    f_th = a_th / (2.0 * L)
    t_end = 6.0 / f_th

    t = 0.0
    ts, ps = [], []
    nu = np.zeros(N)
    while t < t_end:
        r_, u_, p_, Y_, T_, a_ = solver.primitives(U, A, gas)
        dt = 0.35 * dx / float(np.max(np.abs(u_) + a_))
        rhs1, _ = solver.hyperbolic_rhs(U, A, A_f, dx, gas,
                                        _wall_flux(U, A, gas, 0),
                                        _wall_flux(U, A, gas, -1), nu)
        U1 = U + dt * rhs1
        rhs2, _ = solver.hyperbolic_rhs(U1, A, A_f, dx, gas,
                                        _wall_flux(U1, A, gas, 0),
                                        _wall_flux(U1, A, gas, -1), nu)
        U = 0.5 * U + 0.5 * (U1 + dt * rhs2)
        t += dt
        ts.append(t)
        ps.append(float(solver.primitives(U, A, gas)[2][0]))

    ps = np.array(ps) - np.mean(ps)
    ts = np.array(ts)
    # count upcrossings -> frequency
    s = np.where((ps[:-1] <= 0) & (ps[1:] > 0))[0]
    f_meas = (len(s) - 1) / (ts[s[-1]] - ts[s[0]])
    assert abs(f_meas - f_th) / f_th < 0.03


def test_interdiffusion_preserves_uniform_temperature():
    """Species diffusion at uniform T must not manufacture temperature:
    the interdiffusion enthalpy flux (audit finding) keeps a uniform-T,
    uniform-p state uniform to ~1 K despite a sharp Y step and cv_R != cv_P.
    Without the (cp_R - cp_P) T dY/dx term this drifts by ~30 K."""
    gas = inert_gas()
    N = 40
    dx = 0.01
    A = np.ones(N)
    A_f = np.ones(N + 1)
    T0 = 1000.0
    p0 = 1e5
    Y = np.where(np.arange(N) < N // 2, 1.0, 0.0)
    rho = p0 / (gas.R_mix(Y) * T0)
    U = solver.conserved(rho, np.zeros(N), p0 * np.ones(N), Y, gas, A)
    nu = 0.05 * np.ones(N)

    t = 0.0
    while t < 1.3e-3:
        r_, u_, p_, Y_, T_, a_ = solver.primitives(U, A, gas)
        dt = min(0.3 * dx / float(np.max(np.abs(u_) + a_)),
                 0.25 * dx * dx / 0.05)
        rhs1, _ = solver.hyperbolic_rhs(U, A, A_f, dx, gas,
                                        _wall_flux(U, A, gas, 0),
                                        _wall_flux(U, A, gas, -1), nu)
        U1 = U + dt * rhs1
        rhs2, _ = solver.hyperbolic_rhs(U1, A, A_f, dx, gas,
                                        _wall_flux(U1, A, gas, 0),
                                        _wall_flux(U1, A, gas, -1), nu)
        U = 0.5 * U + 0.5 * (U1 + dt * rhs2)
        t += dt

    T_fin = solver.primitives(U, A, gas)[4]
    assert float(np.max(np.abs(T_fin - T0))) < 3.0
