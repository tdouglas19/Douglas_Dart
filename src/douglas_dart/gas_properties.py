"""Temperature-dependent air/combustion-gas properties (real-gas correction).

docs/ramjet_enginesim_comparison.md "Difference 2": NASA Glenn Research
Center's EngineSim (`Turbo.java`'s `getGama`/`getCp`, the ``gamopt == 1``
branch -- the *default*, not an optional mode) evaluates gamma and cp as
cubic polynomials of local station temperature rather than holding them
constant across the whole engine. This module ports those same fitted
polynomials (confirmed against the downloaded EngineSimU v1.5 source,
https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/Enginesim/index.htm)
so ``ramjet.py`` can evaluate combustor-inlet cp and combustor-exit gamma at
their own local temperatures instead of one constant calorically-perfect
value for the whole cycle.

The polynomial coefficients were fit in imperial units (temperature in
degrees Rankine; cp in BTU/(lbm*R)) -- callers here work in Kelvin/J-per-kg-K,
so every evaluation converts in and back out.
"""

from __future__ import annotations

_RANKINE_PER_KELVIN = 1.8
# 1 BTU/(lbm*R) = 1 BTU/(lbm*F) = 4186.8 J/(kg*K) -- the classic BTU/calorie
# identity. NOT the same conversion factor as a plain energy/mass quantity
# (e.g. a fuel heating value in BTU/lbm, which only needs the 2326 J/kg
# per BTU/lbm factor): cp is energy per mass *per degree*, and a Rankine
# degree-of-temperature-change is 5/9 the size of a Kelvin one, so the
# per-degree rate picks up an extra 9/5 = 1.8x on top of the 2326 energy/mass
# conversion (2326 * 1.8 = 4186.8).
_JOULES_PER_KG_K_PER_BTU_PER_LBM_R = 4186.8

# Turbo.java getGama, gamopt==1 branch (opt==0 is a constant-1.4 fallback we
# don't need here). Verified this session: evaluates to ~1.400 at 288 K
# (cold air) and ~1.30 at ramjet combustor-exit temperatures (~1900-2200 K),
# both physically sane for air vs. hydrocarbon combustion products.
_GAMMA_A = -7.6942651e-13
_GAMMA_B = 1.3764661e-08
_GAMMA_C = -7.8185709e-05
_GAMMA_D = 1.436914

# Turbo.java getCp, gamopt==1 branch. Evaluates to ~1000 J/(kg*K) at 288 K,
# matching standard air cp.
_CP_A = -4.4702130e-13
_CP_B = -5.1286514e-10
_CP_C = 2.8323331e-05
_CP_D = 0.2245283


def real_gas_gamma(temperature_k: float) -> float:
    """Return the temperature-dependent ratio of specific heats at ``temperature_k``."""

    if temperature_k <= 0.0:
        raise ValueError("temperature must be positive")
    t_rankine = temperature_k * _RANKINE_PER_KELVIN
    return (
        _GAMMA_A * t_rankine**3
        + _GAMMA_B * t_rankine**2
        + _GAMMA_C * t_rankine
        + _GAMMA_D
    )


def real_gas_specific_heat_j_per_kg_k(temperature_k: float) -> float:
    """Return the temperature-dependent specific heat at constant pressure, in J/(kg*K)."""

    if temperature_k <= 0.0:
        raise ValueError("temperature must be positive")
    t_rankine = temperature_k * _RANKINE_PER_KELVIN
    cp_btu_per_lbm_r = (
        _CP_A * t_rankine**3
        + _CP_B * t_rankine**2
        + _CP_C * t_rankine
        + _CP_D
    )
    return cp_btu_per_lbm_r * _JOULES_PER_KG_K_PER_BTU_PER_LBM_R
