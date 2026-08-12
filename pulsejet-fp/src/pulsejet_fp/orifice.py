"""Compressible orifice flow derived from energy conservation + isentropy,
with choking derived from dG/dr = 0 (derivation.md #7, eqs. 21-23).
No empirical discharge coefficient: the throat area is purely geometric (A17).
"""
from __future__ import annotations

import math


def critical_ratio(gamma: float) -> float:
    """r* = (2/(gamma+1))^(gamma/(gamma-1)) (eq. 23)."""
    return (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))


def mass_flux(p0: float, T0: float, gamma: float, R: float,
              p_down: float) -> tuple[float, float, float]:
    """Mass flux G [kg/(m^2 s)], throat velocity u_t, throat density rho_t
    for flow from stagnation (p0, T0) to downstream static p_down.

    Chokes automatically at r* (eq. 22-23). Returns (0,0,rho0) if p_down >= p0.
    """
    if p0 <= 0.0 or T0 <= 0.0 or p_down >= p0:
        rho0 = p0 / (R * max(T0, 1.0)) if p0 > 0 else 0.0
        return 0.0, 0.0, rho0
    r = max(p_down / p0, critical_ratio(gamma))
    ex = (gamma - 1.0) / gamma
    T_t = T0 * r ** ex
    cp = gamma * R / (gamma - 1.0)
    u_t = math.sqrt(max(2.0 * cp * (T0 - T_t), 0.0))
    p_t = p0 * r
    rho_t = p_t / (R * T_t)
    return rho_t * u_t, u_t, rho_t
