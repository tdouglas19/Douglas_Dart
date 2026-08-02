"""Small SI standard-atmosphere model used by all disciplines."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp, sqrt

G0_M_PER_S2 = 9.80665
R_AIR_J_PER_KG_K = 287.05287
GAMMA_AIR = 1.4
SEA_LEVEL_TEMPERATURE_K = 288.15
SEA_LEVEL_PRESSURE_PA = 101_325.0
TROPOPAUSE_M = 11_000.0
TROPOSPHERE_LAPSE_K_PER_M = -0.0065


@dataclass(frozen=True)
class Atmosphere:
    altitude_m: float
    temperature_k: float
    pressure_pa: float
    density_kg_per_m3: float
    speed_of_sound_m_per_s: float


def standard_atmosphere(altitude_m: float) -> Atmosphere:
    """Return a dry-air atmosphere from -1 to 20 km geometric altitude.

    The two-layer model is sufficient for early trajectory trades. Geopotential
    conversion, humidity, winds, and day-of-flight temperature offsets are deferred.
    """

    if not -1_000.0 <= altitude_m <= 20_000.0:
        raise ValueError("standard_atmosphere supports altitudes from -1,000 to 20,000 m")

    if altitude_m <= TROPOPAUSE_M:
        temperature_k = SEA_LEVEL_TEMPERATURE_K + TROPOSPHERE_LAPSE_K_PER_M * altitude_m
        exponent = -G0_M_PER_S2 / (TROPOSPHERE_LAPSE_K_PER_M * R_AIR_J_PER_KG_K)
        pressure_pa = SEA_LEVEL_PRESSURE_PA * (
            temperature_k / SEA_LEVEL_TEMPERATURE_K
        ) ** exponent
    else:
        tropopause_temperature_k = (
            SEA_LEVEL_TEMPERATURE_K + TROPOSPHERE_LAPSE_K_PER_M * TROPOPAUSE_M
        )
        exponent = -G0_M_PER_S2 / (TROPOSPHERE_LAPSE_K_PER_M * R_AIR_J_PER_KG_K)
        tropopause_pressure_pa = SEA_LEVEL_PRESSURE_PA * (
            tropopause_temperature_k / SEA_LEVEL_TEMPERATURE_K
        ) ** exponent
        temperature_k = tropopause_temperature_k
        pressure_pa = tropopause_pressure_pa * exp(
            -G0_M_PER_S2
            * (altitude_m - TROPOPAUSE_M)
            / (R_AIR_J_PER_KG_K * temperature_k)
        )

    density_kg_per_m3 = pressure_pa / (R_AIR_J_PER_KG_K * temperature_k)
    speed_of_sound_m_per_s = sqrt(GAMMA_AIR * R_AIR_J_PER_KG_K * temperature_k)
    return Atmosphere(
        altitude_m=altitude_m,
        temperature_k=temperature_k,
        pressure_pa=pressure_pa,
        density_kg_per_m3=density_kg_per_m3,
        speed_of_sound_m_per_s=speed_of_sound_m_per_s,
    )
