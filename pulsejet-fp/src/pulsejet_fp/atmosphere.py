"""Ambient state from hydrostatic equilibrium (derivation.md #2)."""
from __future__ import annotations

import math

P_SL = 101_325.0  # Pa
T_SL = 288.15     # K
LAPSE = 6.5e-3    # K/m (observed tropospheric mean, input)
G0 = 9.80665      # m/s^2
R_AIR = 8.314463 / 28.965e-3  # ~287.05 J/(kg K)


def ambient(altitude_m: float = 0.0) -> tuple[float, float]:
    """(p_a, T_a) from dp/dh = -rho g with linear lapse, eq. 6."""
    if altitude_m < 0:
        altitude_m = 0.0
    T = T_SL - LAPSE * altitude_m
    p = P_SL * (T / T_SL) ** (G0 / (R_AIR * LAPSE))
    return p, T


def ram_state(p_a: float, T_a: float, mach: float,
              gamma: float = 1.4) -> tuple[float, float, float]:
    """Vehicle-frame stagnation state of the free stream (derivation.md #8).

    Returns (p0, T0, u_inf). Energy conservation gives T0 exactly;
    isentropic external compression (A20) gives p0.
    """
    m2 = 0.5 * (gamma - 1.0) * mach * mach
    T0 = T_a * (1.0 + m2)
    p0 = p_a * (1.0 + m2) ** (gamma / (gamma - 1.0))
    a_a = math.sqrt(gamma * R_AIR * T_a)
    return p0, T0, mach * a_a
