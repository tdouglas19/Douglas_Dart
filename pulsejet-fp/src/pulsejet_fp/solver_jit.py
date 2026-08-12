"""Numba-compiled kernels implementing exactly the same discrete operators
as solver.py's numpy paths (derivation.md #12), fused into explicit loops.

Fidelity contract: identical formulas, identical floors, identical limiter.
tests/test_jit_equivalence.py pins numpy-vs-JIT agreement to ~1e-12.
Compiled code is disk-cached (cache=True): the first process pays a one-time
compile, later processes load in ~a second.
"""
from __future__ import annotations

import numpy as np

try:
    from numba import njit
    HAS_NUMBA = True
except Exception:  # pragma: no cover
    HAS_NUMBA = False

    def njit(*a, **k):
        def wrap(f):
            return f
        return wrap(a[0]) if a and callable(a[0]) else wrap

RHO_FLOOR = 1e-3
P_FLOOR = 200.0


@njit(cache=True, fastmath=False)
def primitives_jit(U, A_eff, cv_R, cv_P, R_R, R_P):
    n = U.shape[1]
    rho = np.empty(n); u = np.empty(n); p = np.empty(n)
    Y = np.empty(n); T = np.empty(n); a = np.empty(n)
    for i in range(n):
        r = U[0, i] / A_eff[i]
        if r < RHO_FLOOR:
            r = RHO_FLOOR
        inv_m = 1.0 / (r * A_eff[i])
        ui = U[1, i] * inv_m
        Yi = U[3, i] * inv_m
        if Yi < 0.0:
            Yi = 0.0
        elif Yi > 1.0:
            Yi = 1.0
        e = U[2, i] * inv_m - 0.5 * ui * ui
        cv = Yi * cv_R + (1.0 - Yi) * cv_P
        R = Yi * R_R + (1.0 - Yi) * R_P
        Ti = e / cv
        if Ti < 150.0:
            Ti = 150.0
        pi = r * R * Ti
        if pi < P_FLOOR:
            pi = P_FLOOR
        rho[i] = r; u[i] = ui; p[i] = pi; Y[i] = Yi; T[i] = Ti
        a[i] = np.sqrt((1.0 + R / cv) * pi / r)
    return rho, u, p, Y, T, a


@njit(cache=True, fastmath=False)
def _minmod(a, b):
    if a > 0.0 and b > 0.0:
        return a if a < b else b
    if a < 0.0 and b < 0.0:
        return a if a > b else b
    return 0.0


@njit(cache=True, fastmath=False)
def rhs_jit(U, A_eff, A_f, dx, cv_R, cv_P, R_R, R_P, q_gap_unused,
            F_head, F_exit, nu_t,
            rho, u, p, Y, T, a):
    """Fused MUSCL + HLLC + area source + diffusion. Primitives are passed
    in (already computed). Returns rhs (4, n)."""
    n = U.shape[1]
    rhs = np.zeros((4, n))
    inv_dx = 1.0 / dx
    cp_R = cv_R + R_R
    cp_P = cv_P + R_P

    # slopes (minmod), per primitive
    srho = np.zeros(n); su = np.zeros(n); sp = np.zeros(n); sY = np.zeros(n)
    for i in range(1, n - 1):
        srho[i] = _minmod(rho[i] - rho[i - 1], rho[i + 1] - rho[i])
        su[i] = _minmod(u[i] - u[i - 1], u[i + 1] - u[i])
        sp[i] = _minmod(p[i] - p[i - 1], p[i + 1] - p[i])
        sY[i] = _minmod(Y[i] - Y[i - 1], Y[i + 1] - Y[i])

    # face fluxes: j = 0 .. n  (0 = head, n = exit given; interior computed)
    F0 = np.empty(n + 1); F1 = np.empty(n + 1)
    F2 = np.empty(n + 1); F3 = np.empty(n + 1)
    F0[0] = F_head[0]; F1[0] = F_head[1]; F2[0] = F_head[2]; F3[0] = F_head[3]
    F0[n] = F_exit[0]; F1[n] = F_exit[1]; F2[n] = F_exit[2]; F3[n] = F_exit[3]

    for j in range(1, n):
        L = j - 1
        Rc = j
        rL = rho[L] + 0.5 * srho[L]
        uL = u[L] + 0.5 * su[L]
        pL = p[L] + 0.5 * sp[L]
        YL = Y[L] + 0.5 * sY[L]
        rR = rho[Rc] - 0.5 * srho[Rc]
        uR = u[Rc] - 0.5 * su[Rc]
        pR = p[Rc] - 0.5 * sp[Rc]
        YR = Y[Rc] - 0.5 * sY[Rc]
        if rL < RHO_FLOOR: rL = RHO_FLOOR
        if rR < RHO_FLOOR: rR = RHO_FLOOR
        if pL < P_FLOOR: pL = P_FLOOR
        if pR < P_FLOOR: pR = P_FLOOR
        if YL < 0.0: YL = 0.0
        elif YL > 1.0: YL = 1.0
        if YR < 0.0: YR = 0.0
        elif YR > 1.0: YR = 1.0

        gL = 1.0 + (YL * R_R + (1.0 - YL) * R_P) / (YL * cv_R + (1.0 - YL) * cv_P)
        gR = 1.0 + (YR * R_R + (1.0 - YR) * R_P) / (YR * cv_R + (1.0 - YR) * cv_P)
        aL = np.sqrt(gL * pL / rL)
        aR = np.sqrt(gR * pR / rR)
        EL = pL / ((gL - 1.0) * rL) + 0.5 * uL * uL
        ER = pR / ((gR - 1.0) * rR) + 0.5 * uR * uR

        SL = uL - aL
        t2 = uR - aR
        if t2 < SL: SL = t2
        SR = uL + aL
        t2 = uR + aR
        if t2 > SR: SR = t2
        dL = rL * (SL - uL)
        dR = rR * (SR - uR)
        Sstar = (pR - pL + uL * dL - uR * dR) / (dL - dR)

        if Sstar >= 0.0:
            rK = rL; uK = uL; pK = pL; EK = EL; YK = YL; SK = SL
            need = SL < 0.0
        else:
            rK = rR; uK = uR; pK = pR; EK = ER; YK = YR; SK = SR
            need = SR > 0.0

        f0 = rK * uK
        f1 = f0 * uK + pK
        f2 = uK * (rK * EK + pK)
        f3 = f0 * YK
        if need:
            dK = rK * (SK - uK)
            coef = dK / (SK - Sstar)
            Estar = EK + (Sstar - uK) * (Sstar + pK / dK)
            f0 += SK * (coef - rK)
            f1 += SK * (coef * Sstar - rK * uK)
            f2 += SK * (coef * Estar - rK * EK)
            f3 += SK * (coef - rK) * YK
        Af = A_f[j]
        F0[j] = f0 * Af; F1[j] = f1 * Af; F2[j] = f2 * Af; F3[j] = f3 * Af

    for i in range(n):
        rhs[0, i] = (F0[i] - F0[i + 1]) * inv_dx
        rhs[1, i] = (F1[i] - F1[i + 1]) * inv_dx \
            + p[i] * (A_f[i + 1] - A_f[i]) * inv_dx
        rhs[2, i] = (F2[i] - F2[i + 1]) * inv_dx
        rhs[3, i] = (F3[i] - F3[i + 1]) * inv_dx

    # diffusion (interior faces), identical to solver.hyperbolic_rhs
    for i in range(n - 1):
        rhoA_face = 0.5 * (rho[i] * A_eff[i] + rho[i + 1] * A_eff[i + 1])
        nu_face = 0.5 * (nu_t[i] + nu_t[i + 1])
        k_face = rhoA_face * nu_face * inv_dx
        cp_i = Y[i] * cp_R + (1.0 - Y[i]) * cp_P
        cp_ip = Y[i + 1] * cp_R + (1.0 - Y[i + 1]) * cp_P
        cp_face = 0.5 * (cp_i + cp_ip)
        u_face = 0.5 * (u[i] + u[i + 1])
        T_face = 0.5 * (T[i] + T[i + 1])
        du = u[i + 1] - u[i]
        dT = T[i + 1] - T[i]
        dY = Y[i + 1] - Y[i]
        flux_mom = k_face * du
        flux_Y = k_face * dY
        flux_E = k_face * (cp_face * dT) + flux_mom * u_face \
            + (cp_R - cp_P) * T_face * flux_Y
        rhs[1, i] += flux_mom * inv_dx; rhs[1, i + 1] -= flux_mom * inv_dx
        rhs[2, i] += flux_E * inv_dx;   rhs[2, i + 1] -= flux_E * inv_dx
        rhs[3, i] += flux_Y * inv_dx;   rhs[3, i + 1] -= flux_Y * inv_dx

    return rhs


@njit(cache=True, fastmath=False)
def reaction_jit(U, A_eff, cv_R, cv_P, R_R, R_P, A_r, T_a, q_R, dt, quench,
                 rho, T, Y):
    """Operator-split reaction, identical to solver.reaction_substep.
    quench: array (may be all ones). Returns (heat_sum, q_rate array)."""
    n = U.shape[1]
    q_rate = np.zeros(n)
    kmax = 0.0
    any_burn = False
    for i in range(n):
        if Y[i] > 1e-8 and T[i] > 500.0:
            k = A_r * np.exp(-T_a / T[i]) * quench[i]
            if k * dt > kmax:
                kmax = k * dt
            any_burn = True
    if not any_burn:
        return 0.0, q_rate
    nsub = int(np.ceil(kmax / 0.2))
    if nsub < 1:
        nsub = 1
    elif nsub > 64:
        nsub = 64
    dtn = dt / nsub

    heat_sum = 0.0
    for i in range(n):
        Yi = Y[i]
        Ti = T[i]
        e = (Yi * cv_R + (1.0 - Yi) * cv_P) * Ti
        burned = 0.0
        for _ in range(nsub):
            Tc = Ti if Ti > 150.0 else 150.0
            k = A_r * np.exp(-T_a / Tc) * quench[i]
            dYb = Yi * (1.0 - np.exp(-k * dtn))
            Yi -= dYb
            e += q_R * dYb
            Ti = e / (Yi * cv_R + (1.0 - Yi) * cv_P)
            burned += dYb
        dE = rho[i] * A_eff[i] * q_R * burned
        U[2, i] += dE
        U[3, i] = rho[i] * A_eff[i] * Yi
        q_rate[i] = dE / dt
        heat_sum += dE
    return heat_sum, q_rate
