"""Two-pseudo-species gas model derived from kinetic theory (derivation.md #1).

Species R = premixed fuel-air reactant at equivalence ratio phi.
Species P = combustion products.
Mixture state is carried by the reactant mass fraction Y.

Everything here follows from the ideal-gas law, the equipartition theorem,
and reaction stoichiometry; the only material-property inputs are atomic
weights, the fuel's lower heating value, and the Arrhenius parameters.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

R_UNIVERSAL = 8.314463  # J / (mol K)

# Atomic weights, kg/mol (periodic-table constants, not correlations)
W_C = 12.011e-3
W_H = 1.008e-3
W_O = 15.999e-3
W_N = 14.007e-3

W_O2 = 2 * W_O
W_N2 = 2 * W_N
W_CO2 = W_C + 2 * W_O
W_H2O = 2 * W_H + W_O

# Standard dry air: 20.95% O2, 78.09% N2, ~0.96% Ar-and-trace by mole.
# Model air as O2 + "effective N2" carrying the inert remainder so the molar
# mass matches the true 28.965 g/mol (composition fact, not a correlation).
X_O2_AIR = 0.2095
W_AIR = 28.965e-3
Y_O2_AIR = X_O2_AIR * W_O2 / W_AIR  # 0.2314 oxygen mass fraction


@dataclass(frozen=True)
class GasModel:
    """Frozen thermochemical description for one fuel/phi choice."""

    # Reactant pseudo-species (premixed fuel-air)
    W_R: float
    cv_R: float  # J/(kg K)
    # Product pseudo-species
    W_P: float
    cv_P: float
    # Heat released per kg of reactant mixture converted (eq. 4)
    q_R: float
    # Arrhenius kinetics (eq. 11)
    T_a: float  # activation temperature, K
    A_r: float  # pre-exponential, 1/s
    # Bookkeeping
    phi: float = 1.0
    f: float = 0.0  # fuel/air mass ratio actually mixed

    @property
    def R_R(self) -> float:
        return R_UNIVERSAL / self.W_R

    @property
    def R_P(self) -> float:
        return R_UNIVERSAL / self.W_P

    @property
    def cp_R(self) -> float:
        return self.cv_R + self.R_R

    @property
    def cp_P(self) -> float:
        return self.cv_P + self.R_P

    @property
    def gamma_R(self) -> float:
        return self.cp_R / self.cv_R

    @property
    def gamma_P(self) -> float:
        return self.cp_P / self.cv_P

    # ---- mixture rules (mass-weighted, eq. 2); Y may be ndarray ----
    def R_mix(self, Y):
        return Y * self.R_R + (1.0 - Y) * self.R_P

    def cv_mix(self, Y):
        return Y * self.cv_R + (1.0 - Y) * self.cv_P

    def cp_mix(self, Y):
        return self.cv_mix(Y) + self.R_mix(Y)

    def gamma_mix(self, Y):
        cv = self.cv_mix(Y)
        return (cv + self.R_mix(Y)) / cv

    def T_from_e(self, e_sens, Y):
        return e_sens / self.cv_mix(Y)

    def e_from_T(self, T, Y):
        return self.cv_mix(Y) * T

    def p_from_rho_T(self, rho, T, Y):
        return rho * self.R_mix(Y) * T

    def sound_speed(self, p, rho, Y):
        return np.sqrt(self.gamma_mix(Y) * np.maximum(p, 1e-8) / np.maximum(rho, 1e-12))

    def reaction_rate(self, T, Y):
        """omega_dot, kg reactant per kg mixture per second (eq. 11)."""
        return Y * self.A_r * np.exp(-self.T_a / np.maximum(T, 150.0))


def propane_air(phi: float = 1.0,
                A_r: float = 2.0e7,
                E_a: float = 125.5e3,
                lhv: float = 46.35e6) -> GasModel:
    """Build the GasModel for premixed propane-air from stoichiometry alone.

    C3H8 + 5 O2 -> 3 CO2 + 4 H2O  (derivation.md eq. 3)
    """
    W_fuel = 3 * W_C + 8 * W_H  # 44.097e-3

    f_st = (W_fuel / (5 * W_O2)) * Y_O2_AIR
    f = phi * f_st  # kg fuel per kg air actually mixed

    # Reactant molar mass: 1 kg air + f kg fuel
    n_R = 1.0 / W_AIR + f / W_fuel
    W_R = (1.0 + f) / n_R

    # Products: lean/stoich burns all fuel with excess O2 remaining; rich
    # burns only the O2-limited fuel fraction, leaving unburned fuel vapor
    # in the product mixture. Moles per kg air basis:
    burnable = min(1.0, 1.0 / phi) if phi > 0 else 0.0
    n_fuel = f / W_fuel
    n_fuel_burned = n_fuel * burnable
    n_O2_avail = Y_O2_AIR / W_O2
    n_O2_used = 5.0 * n_fuel_burned
    # inert moles: air minus O2, treated as effective-N2 carrying Ar/trace
    n_inert = (1.0 / W_AIR) - n_O2_avail
    n_CO2 = 3.0 * n_fuel_burned
    n_H2O = 4.0 * n_fuel_burned
    n_O2_left = max(n_O2_avail - n_O2_used, 0.0)
    n_fuel_left = n_fuel - n_fuel_burned
    n_P = n_inert + n_CO2 + n_H2O + n_O2_left + n_fuel_left
    W_P = (1.0 + f) / n_P

    R_R = R_UNIVERSAL / W_R
    R_P = R_UNIVERSAL / W_P

    # Equipartition (derivation.md #1): reactant ~ diatomic, cv = 5/2 R.
    # Products: effective 7 quadratic modes (triatomics + hot vibration),
    # cv = 7/2 R  ->  gamma_P = 9/7.
    cv_R = 2.5 * R_R
    cv_P = 3.5 * R_P

    # Heat release per kg reactant mixture (eq. 4); phi > 1 burns only the
    # oxygen-limited fraction. The sensible-energy convention e = cv(Y) T is
    # referenced to 0 K, so the constant must be the 0 K heat of reaction:
    # q_0 = De(298) + (cv_P - cv_R) * 298.15  (the cv-swap correction; the
    # Dh-vs-De p-v term is ~0.1% and neglected). With this, the effective
    # heat released when converting at temperature T is q_0 - (cv_P-cv_R) T,
    # exactly as energy conservation with per-species cv requires.
    q_298 = (f * burnable) / (1.0 + f) * lhv
    q_R = q_298 + (cv_P - cv_R) * 298.15

    return GasModel(
        W_R=W_R, cv_R=cv_R, W_P=W_P, cv_P=cv_P, q_R=q_R,
        T_a=E_a / R_UNIVERSAL, A_r=A_r, phi=phi, f=f,
    )
