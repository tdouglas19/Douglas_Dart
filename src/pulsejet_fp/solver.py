"""Finite-volume core: MUSCL-minmod reconstruction, HLLC flux, turbulent
diffusion, area source, reaction sub-stepping (derivation.md #12).

State per cell (conserved, per unit length): U = [rho*A, rho*u*A, rho*E*A,
rho*Y*A]. The first cell's effective area may differ from geometric (valve
swept volume, eq. 29); callers pass the A_eff array used for primitive
recovery.
"""
from __future__ import annotations

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
    u = U[1] / (rho * A_eff)
    Y = np.clip(U[2 + 1] / (rho * A_eff), 0.0, 1.0)
    e = U[2] / (rho * A_eff) - 0.5 * u * u
    cv = gas.cv_mix(Y)
    T = np.maximum(e / cv, 150.0)
    p = np.maximum(rho * gas.R_mix(Y) * T, P_FLOOR)
    a = np.sqrt(gas.gamma_mix(Y) * p / rho)
    return rho, u, p, Y, T, a


def _minmod(a, b):
    s = np.sign(a)
    return s * np.maximum(0.0, np.minimum(np.abs(a), s * b))


def muscl_faces(w):
    """Piecewise-linear minmod reconstruction of one primitive array w (N,).
    Returns (wL, wR) at the N-1 interior interfaces."""
    dw = np.diff(w)
    slope = np.zeros_like(w)
    slope[1:-1] = _minmod(dw[:-1], dw[1:])
    wL = w[:-1] + 0.5 * slope[:-1]
    wR = w[1:] - 0.5 * slope[1:]
    return wL, wR


def hllc(rhoL, uL, pL, YL, rhoR, uR, pR, YR, gas):
    """Vectorized HLLC flux per unit area (eqs. 31-32). Returns (4, M)."""
    gL = gas.gamma_mix(YL)
    gR = gas.gamma_mix(YR)
    aL = np.sqrt(gL * pL / rhoL)
    aR = np.sqrt(gR * pR / rhoR)
    EL = pL / ((gL - 1.0) * rhoL) + 0.5 * uL * uL
    ER = pR / ((gR - 1.0) * rhoR) + 0.5 * uR * uR

    SL = np.minimum(uL - aL, uR - aR)
    SR = np.maximum(uL + aL, uR + aR)
    dL = rhoL * (SL - uL)
    dR = rhoR * (SR - uR)
    Sstar = (pR - pL + uL * dL - uR * dR) / (dL - dR)

    def flux(rho, u, p, E, Y):
        return np.stack([rho * u,
                         rho * u * u + p,
                         u * (rho * E + p),
                         rho * u * Y])

    FL = flux(rhoL, uL, pL, EL, YL)
    FR = flux(rhoR, uR, pR, ER, YR)

    def star_U(rho, u, p, E, Y, S, Ss):
        coef = rho * (S - u) / (S - Ss)
        Estar = E + (Ss - u) * (Ss + p / (rho * (S - u)))
        return np.stack([coef, coef * Ss, coef * Estar, coef * Y])

    UL = np.stack([rhoL, rhoL * uL, rhoL * EL, rhoL * YL])
    UR = np.stack([rhoR, rhoR * uR, rhoR * ER, rhoR * YR])
    UsL = star_U(rhoL, uL, pL, EL, YL, SL, Sstar)
    UsR = star_U(rhoR, uR, pR, ER, YR, SR, Sstar)

    F = np.where(SL >= 0.0, FL,
        np.where(Sstar >= 0.0, FL + SL * (UsL - UL),
        np.where(SR >= 0.0, FR + SR * (UsR - UR), FR)))
    return F


def hllc_scalar(rhoL, uL, pL, YL, rhoR, uR, pR, YR, gas):
    """Scalar-state convenience wrapper (boundary faces)."""
    arr = lambda v: np.array([v], dtype=float)
    F = hllc(arr(rhoL), arr(uL), arr(pL), arr(YL),
             arr(rhoR), arr(uR), arr(pR), arr(YR), gas)
    return F[:, 0]


def wall_pressure(rho1, u1, p1, Y1, gas, side: str = "left"):
    """Exact HLLC star pressure against a rigid wall (mirror-state Riemann
    problem; S* = 0 by symmetry). `side` is which side of the cell the wall
    is on: gas moving toward the wall must read as compression."""
    if side == "right":
        u1 = -u1
    g = float(gas.gamma_mix(Y1))
    a1 = np.sqrt(g * p1 / rho1)
    # left-wall convention: ghost is the mirror (-u1); SL = min(-u1, u1) - a1
    SL = min(-u1, u1) - a1
    uLm = -u1
    p_star = p1 + rho1 * (SL - uLm) * (0.0 - uLm)
    return max(p_star, P_FLOOR)


def hyperbolic_rhs(U, A_eff, A_f, dx, gas, F_head, F_exit, nu_t):
    """dU/dt from fluxes + area source + turbulent diffusion.
    F_head, F_exit: total boundary fluxes (already include face areas).
    Returns (rhs, p) -- p returned for reuse."""
    rho, u, p, Y, T, a = primitives(U, A_eff, gas)

    # -- interior interfaces: MUSCL + HLLC --
    rL, rR = muscl_faces(rho)
    uL, uR = muscl_faces(u)
    pL, pR = muscl_faces(p)
    YL, YR = muscl_faces(Y)
    rL = np.maximum(rL, RHO_FLOOR); rR = np.maximum(rR, RHO_FLOOR)
    pL = np.maximum(pL, P_FLOOR); pR = np.maximum(pR, P_FLOOR)
    YL = np.clip(YL, 0.0, 1.0); YR = np.clip(YR, 0.0, 1.0)
    F_int = hllc(rL, uL, pL, YL, rR, uR, pR, YR, gas) * A_f[1:-1]

    F = np.empty((4, U.shape[1] + 1))
    F[:, 1:-1] = F_int
    F[:, 0] = F_head
    F[:, -1] = F_exit

    rhs = -(F[:, 1:] - F[:, :-1]) / dx

    # -- area-pressure source (momentum), exactly balancing rest states --
    rhs[1] += p * (A_f[1:] - A_f[:-1]) / dx

    # -- turbulent diffusion of momentum, heat, species (A7) --
    cp = gas.cp_mix(Y)
    rhoA_face = 0.5 * (rho[:-1] * A_eff[:-1] + rho[1:] * A_eff[1:])
    nu_face = 0.5 * (nu_t[:-1] + nu_t[1:])
    du = np.diff(u) / dx
    dT = np.diff(T) / dx
    dY = np.diff(Y) / dx
    cp_face = 0.5 * (cp[:-1] + cp[1:])
    u_face = 0.5 * (u[:-1] + u[1:])

    flux_mom = rhoA_face * nu_face * du                      # shear
    flux_E = rhoA_face * nu_face * (cp_face * dT) + flux_mom * u_face
    flux_Y = rhoA_face * nu_face * dY

    rhs[1, :-1] += flux_mom / dx; rhs[1, 1:] -= flux_mom / dx
    rhs[2, :-1] += flux_E / dx;   rhs[2, 1:] -= flux_E / dx
    rhs[3, :-1] += flux_Y / dx;   rhs[3, 1:] -= flux_Y / dx

    return rhs, p


def reaction_substep(U, A_eff, gas, dt, quench=None):
    """Operator-split reaction update: exact exponential decay of Y at
    (piecewise) frozen T, sub-cycled for stiffness (derivation.md #12.3).
    `quench` (optional array in [0,1]) multiplies the kinetic rate -- the
    strain-extinction factor of derivation.md #5b (A23).
    Returns total heat released [J] this step (for the Rayleigh record)
    and the volumetric heat release rate array [W/m] (per unit length)."""
    rho, u, p, Y, T, a = primitives(U, A_eff, gas)
    mask = (Y > 1e-8) & (T > 500.0)
    if not mask.any():
        return 0.0, np.zeros_like(Y)
    if quench is None:
        quench = 1.0

    k = gas.A_r * np.exp(-gas.T_a / T) * quench
    kmax = float(np.max(k[mask] * dt))
    nsub = int(np.clip(np.ceil(kmax / 0.2), 1, 64))
    dtn = dt / nsub

    e_sens = gas.cv_mix(Y) * T  # per unit mass
    Yn = Y.copy()
    Tn = T.copy()
    burned_total = np.zeros_like(Y)
    for _ in range(nsub):
        k = gas.A_r * np.exp(-gas.T_a / np.maximum(Tn, 150.0)) * quench
        dYb = Yn * (1.0 - np.exp(-k * dtn))
        Yn = Yn - dYb
        e_sens = e_sens + gas.q_R * dYb
        Tn = e_sens / gas.cv_mix(Yn)
        burned_total += dYb

    # write back: total energy per length gains rho*A*qR*dYb; species updates
    dE = rho * A_eff * gas.q_R * burned_total
    U[2] += dE
    U[3] = rho * A_eff * Yn
    q_rate = dE / dt  # W per metre of duct
    heat_j = float(np.sum(dE))  # per unit length * dx applied by caller
    return heat_j, q_rate
