"""Finite-volume core: MUSCL-minmod reconstruction, HLLC flux, turbulent
diffusion, area source, reaction sub-stepping (derivation.md #12).

State per cell (conserved, per unit length): U = [rho*A, rho*u*A, rho*E*A,
rho*Y*A]. The first cell's effective area may differ from geometric (valve
swept volume, eq. 29); callers pass the A_eff array used for primitive
recovery.

Implementation note: hot paths are written allocation-lean (single output
buffers, no np.stack, mask algebra instead of nested np.where) -- the
formulas are identical to derivation.md eqs. 31-32 and are pinned by the
exact-Riemann/conservation/acoustic tests.
"""
from __future__ import annotations

import math

import numpy as np

RHO_FLOOR = 1e-3
P_FLOOR = 200.0


def conserved(rho, u, p, Y, gas, A):
    e = p / ((gas.gamma_mix(Y) - 1.0) * rho)
    E = e + 0.5 * u * u
    return np.stack([rho * A, rho * u * A, rho * E * A, rho * Y * A])


def primitives(U, A_eff, gas):
    """Return rho, u, p, Y, T, a (all ndarray) with positivity floors."""
    rho = np.maximum(U[0] / A_eff, RHO_FLOOR)
    inv_m = 1.0 / (rho * A_eff)
    u = U[1] * inv_m
    Y = np.clip(U[3] * inv_m, 0.0, 1.0)
    e = U[2] * inv_m - 0.5 * u * u
    cv = Y * gas.cv_R + (1.0 - Y) * gas.cv_P
    R = Y * gas.R_R + (1.0 - Y) * gas.R_P
    T = np.maximum(e / cv, 150.0)
    p = np.maximum(rho * R * T, P_FLOOR)
    a = np.sqrt((1.0 + R / cv) * p / rho)
    return rho, u, p, Y, T, a


def _minmod(a, b):
    s = np.sign(a)
    return s * np.maximum(0.0, np.minimum(np.abs(a), s * b))


def muscl_faces_block(W):
    """Piecewise-linear minmod reconstruction of a (K, N) primitive block.
    Returns (WL, WR) at the N-1 interior interfaces, each (K, N-1)."""
    dW = np.diff(W, axis=1)
    slope = np.zeros_like(W)
    slope[:, 1:-1] = _minmod(dW[:, :-1], dW[:, 1:])
    WL = W[:, :-1] + 0.5 * slope[:, :-1]
    WR = W[:, 1:] - 0.5 * slope[:, 1:]
    return WL, WR


def muscl_faces(w):
    """1D convenience wrapper (kept for tests/back-compat)."""
    WL, WR = muscl_faces_block(w[None, :])
    return WL[0], WR[0]


def hllc(rhoL, uL, pL, YL, rhoR, uR, pR, YR, gas):
    """Vectorized HLLC flux per unit area (eqs. 31-32). Returns (4, M)."""
    gL = 1.0 + (YL * gas.R_R + (1.0 - YL) * gas.R_P) \
        / (YL * gas.cv_R + (1.0 - YL) * gas.cv_P)
    gR = 1.0 + (YR * gas.R_R + (1.0 - YR) * gas.R_P) \
        / (YR * gas.cv_R + (1.0 - YR) * gas.cv_P)
    aL = np.sqrt(gL * pL / rhoL)
    aR = np.sqrt(gR * pR / rhoR)
    EL = pL / ((gL - 1.0) * rhoL) + 0.5 * uL * uL
    ER = pR / ((gR - 1.0) * rhoR) + 0.5 * uR * uR

    SL = np.minimum(uL - aL, uR - aR)
    SR = np.maximum(uL + aL, uR + aR)
    dL = rhoL * (SL - uL)
    dR = rhoR * (SR - uR)
    Sstar = (pR - pL + uL * dL - uR * dR) / (dL - dR)

    left = Sstar >= 0.0
    rhoK = np.where(left, rhoL, rhoR)
    uK = np.where(left, uL, uR)
    pK = np.where(left, pL, pR)
    EK = np.where(left, EL, ER)
    YK = np.where(left, YL, YR)
    SK = np.where(left, SL, SR)

    FK0 = rhoK * uK
    FK1 = FK0 * uK + pK
    FK2 = uK * (rhoK * EK + pK)
    FK3 = FK0 * YK

    # star-region correction fires only when the K-side wave straddles x/t=0
    need = np.where(left, SL < 0.0, SR > 0.0)
    dK = rhoK * (SK - uK)
    coef = dK / (SK - Sstar)
    Estar = EK + (Sstar - uK) * (Sstar + pK / dK)
    fac = need * SK

    F = np.empty((4, np.shape(rhoL)[0] if np.ndim(rhoL) else 1))
    F[0] = FK0 + fac * (coef - rhoK)
    F[1] = FK1 + fac * (coef * Sstar - rhoK * uK)
    F[2] = FK2 + fac * (coef * Estar - rhoK * EK)
    F[3] = FK3 + fac * (coef - rhoK) * YK
    return F


def hllc_scalar(rhoL, uL, pL, YL, rhoR, uR, pR, YR, gas):
    """Pure-scalar HLLC (boundary faces) -- same algebra as hllc()."""
    gL = 1.0 + (YL * gas.R_R + (1.0 - YL) * gas.R_P) \
        / (YL * gas.cv_R + (1.0 - YL) * gas.cv_P)
    gR = 1.0 + (YR * gas.R_R + (1.0 - YR) * gas.R_P) \
        / (YR * gas.cv_R + (1.0 - YR) * gas.cv_P)
    aL = math.sqrt(gL * pL / rhoL)
    aR = math.sqrt(gR * pR / rhoR)
    EL = pL / ((gL - 1.0) * rhoL) + 0.5 * uL * uL
    ER = pR / ((gR - 1.0) * rhoR) + 0.5 * uR * uR
    SL = min(uL - aL, uR - aR)
    SR = max(uL + aL, uR + aR)
    dL = rhoL * (SL - uL)
    dR = rhoR * (SR - uR)
    Sstar = (pR - pL + uL * dL - uR * dR) / (dL - dR)
    if Sstar >= 0.0:
        rhoK, uK, pK, EK, YK, SK, need = rhoL, uL, pL, EL, YL, SL, SL < 0.0
    else:
        rhoK, uK, pK, EK, YK, SK, need = rhoR, uR, pR, ER, YR, SR, SR > 0.0
    F0 = rhoK * uK
    F1 = F0 * uK + pK
    F2 = uK * (rhoK * EK + pK)
    F3 = F0 * YK
    if need:
        dK = rhoK * (SK - uK)
        coef = dK / (SK - Sstar)
        Estar = EK + (Sstar - uK) * (Sstar + pK / dK)
        F0 += SK * (coef - rhoK)
        F1 += SK * (coef * Sstar - rhoK * uK)
        F2 += SK * (coef * Estar - rhoK * EK)
        F3 += SK * (coef - rhoK) * YK
    return np.array([F0, F1, F2, F3])


def wall_pressure(rho1, u1, p1, Y1, gas, side: str = "left"):
    """Exact HLLC star pressure against a rigid wall (mirror-state Riemann
    problem; S* = 0 by symmetry). `side` is which side of the cell the wall
    is on: gas moving toward the wall must read as compression."""
    if side == "right":
        u1 = -u1
    g = float(gas.gamma_mix(Y1))
    a1 = math.sqrt(g * p1 / rho1)
    # left-wall convention: ghost is the mirror (-u1); SL = min(-u1, u1) - a1
    SL = min(-u1, u1) - a1
    uLm = -u1
    p_star = p1 + rho1 * (SL - uLm) * (0.0 - uLm)
    return max(p_star, P_FLOOR)


def hyperbolic_rhs(U, A_eff, A_f, dx, gas, F_head, F_exit, nu_t, prims=None):
    """dU/dt from fluxes + area source + turbulent diffusion.
    F_head, F_exit: total boundary fluxes (already include face areas).
    `prims`: optionally pass primitives(U, A_eff, gas) to avoid recompute.
    Returns (rhs, p)."""
    if prims is None:
        prims = primitives(U, A_eff, gas)
    rho, u, p, Y, T, a = prims

    # -- interior interfaces: MUSCL + HLLC --
    W = np.empty((4, rho.shape[0]))
    W[0] = rho; W[1] = u; W[2] = p; W[3] = Y
    WL, WR = muscl_faces_block(W)
    rL = np.maximum(WL[0], RHO_FLOOR); rR = np.maximum(WR[0], RHO_FLOOR)
    pL = np.maximum(WL[2], P_FLOOR); pR = np.maximum(WR[2], P_FLOOR)
    YL = np.clip(WL[3], 0.0, 1.0); YR = np.clip(WR[3], 0.0, 1.0)
    F_int = hllc(rL, WL[1], pL, YL, rR, WR[1], pR, YR, gas) * A_f[1:-1]

    F = np.empty((4, U.shape[1] + 1))
    F[:, 1:-1] = F_int
    F[:, 0] = F_head
    F[:, -1] = F_exit

    rhs = (F[:, :-1] - F[:, 1:]) * (1.0 / dx)

    # -- area-pressure source (momentum), exactly balancing rest states --
    rhs[1] += p * (A_f[1:] - A_f[:-1]) * (1.0 / dx)

    # -- turbulent diffusion of momentum, heat, species (A7) --
    cp = Y * (gas.cv_R + gas.R_R) + (1.0 - Y) * (gas.cv_P + gas.R_P)
    rhoA = rho * A_eff
    rhoA_face = 0.5 * (rhoA[:-1] + rhoA[1:])
    nu_face = 0.5 * (nu_t[:-1] + nu_t[1:])
    k_face = rhoA_face * nu_face * (1.0 / dx)
    du = np.diff(u)
    dT = np.diff(T)
    dY = np.diff(Y)
    cp_face = 0.5 * (cp[:-1] + cp[1:])
    u_face = 0.5 * (u[:-1] + u[1:])

    flux_mom = k_face * du
    flux_E = k_face * (cp_face * dT) + flux_mom * u_face
    flux_Y = k_face * dY

    inv_dx = 1.0 / dx
    rhs[1, :-1] += flux_mom * inv_dx; rhs[1, 1:] -= flux_mom * inv_dx
    rhs[2, :-1] += flux_E * inv_dx;   rhs[2, 1:] -= flux_E * inv_dx
    rhs[3, :-1] += flux_Y * inv_dx;   rhs[3, 1:] -= flux_Y * inv_dx

    return rhs, p


def reaction_substep(U, A_eff, gas, dt, quench=None, prims=None):
    """Operator-split reaction update: exact exponential decay of Y at
    (piecewise) frozen T, sub-cycled for stiffness (derivation.md #12.3).
    `quench` (optional array in [0,1]) multiplies the kinetic rate -- the
    strain-extinction factor of derivation.md #5b (A23).
    Returns total heat released per unit length summed [J/m] and the
    volumetric heat release rate array [W/m]."""
    if prims is None:
        prims = primitives(U, A_eff, gas)
    rho, u, p, Y, T, a = prims
    mask = (Y > 1e-8) & (T > 500.0)
    if not mask.any():
        return 0.0, np.zeros_like(Y)
    if quench is None:
        quench = 1.0

    k = gas.A_r * np.exp(-gas.T_a / T) * quench
    kmax = float(np.max(k[mask] * dt))
    nsub = int(np.clip(np.ceil(kmax / 0.2), 1, 64))
    dtn = dt / nsub

    e_sens = (Y * gas.cv_R + (1.0 - Y) * gas.cv_P) * T  # per unit mass
    Yn = Y.copy()
    Tn = T.copy()
    burned_total = np.zeros_like(Y)
    for _ in range(nsub):
        k = gas.A_r * np.exp(-gas.T_a / np.maximum(Tn, 150.0)) * quench
        dYb = Yn * (1.0 - np.exp(-k * dtn))
        Yn = Yn - dYb
        e_sens = e_sens + gas.q_R * dYb
        Tn = e_sens / (Yn * gas.cv_R + (1.0 - Yn) * gas.cv_P)
        burned_total += dYb

    # write back: total energy per length gains rho*A*qR*dYb; species updates
    dE = rho * A_eff * gas.q_R * burned_total
    U[2] += dE
    U[3] = rho * A_eff * Yn
    q_rate = dE / dt  # W per metre of duct
    heat_j = float(np.sum(dE))
    return heat_j, q_rate
