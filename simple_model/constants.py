"""Shared constants: fuels, calorically-perfect gas properties, unit factors.

Every number here is a deliberately simple, easy-to-override placeholder --
this module exists so the pulsejet/ramjet/drag modules pull from one place
instead of repeating magic numbers.
"""

from __future__ import annotations

from dataclasses import dataclass

G0_M_PER_S2 = 9.80665
KG_PER_LB = 0.45359237
M_PER_IN = 0.0254


@dataclass(frozen=True)
class Fuel:
    key: str
    display_name: str
    lower_heating_value_j_per_kg: float
    stoichiometric_air_fuel_ratio: float
    density_kg_per_m3: float


# Same representative values as configs/fuels.yaml, copied rather than
# YAML-loaded so this package has zero file-path dependencies on the main
# douglas_dart config tree.
FUELS: dict[str, Fuel] = {
    "jet_a": Fuel("jet_a", "Jet-A representative", 43_000_000.0, 14.7, 800.0),
    "gasoline": Fuel("gasoline", "Gasoline representative", 44_000_000.0, 14.7, 740.0),
    "propane": Fuel("propane", "Propane representative", 46_400_000.0, 15.7, 493.0),
    "ethanol": Fuel("ethanol", "Ethanol representative", 26_800_000.0, 9.0, 789.0),
}

# --- Calorically-perfect gas properties -----------------------------------
# Pre-combustion (intake air) and post-combustion (products) are each held at
# one constant (gamma, R, cp) rather than the temperature-dependent
# polynomials in douglas_dart/gas_properties.py -- that real-gas correction
# is exactly the kind of complexity this standalone model is meant to skip.
GAMMA_AIR = 1.40
R_AIR_J_PER_KG_K = 287.05
CP_AIR_J_PER_KG_K = GAMMA_AIR * R_AIR_J_PER_KG_K / (GAMMA_AIR - 1.0)

# Representative hydrocarbon-air combustion products. gamma=1.30 is a common
# round-number choice for hot combustion gas (vs. 1.40 for cold air);
# R is left equal to air's since dissociation/species-shift effects on the
# gas constant are second-order next to the gamma shift, for this level of
# fidelity.
GAMMA_COMB = 1.30
R_COMB_J_PER_KG_K = 287.05
CP_COMB_J_PER_KG_K = GAMMA_COMB * R_COMB_J_PER_KG_K / (GAMMA_COMB - 1.0)
CV_COMB_J_PER_KG_K = CP_COMB_J_PER_KG_K / GAMMA_COMB

# Combustion efficiency applied to both engines' energy release (fraction of
# LHV actually realized as a temperature rise). A single constant, not swept.
COMBUSTION_EFFICIENCY = 0.95

# Ceiling on combustor/chamber stagnation temperature -- guards against
# unphysical results (dissociation caps real hydrocarbon-air flames well
# below their calorically-perfect adiabatic value). Matches the existing
# douglas_dart pulsejet config's own maximum_gas_temperature_k.
MAX_CHAMBER_TEMPERATURE_K = 2600.0

# Real valved pulsejets peak around ~2.25 atm absolute (~1.25 atm / ~18 psi
# gauge) chamber pressure -- user-supplied hardware reference, 2026-08-11.
# That is well below what an idealized constant-volume adiabatic combustion
# energy balance predicts (~9 atm for stoichiometric propane at this scale):
# real engines never actually hold the charge at constant volume for the
# full heat-release duration. The valve/tailpipe system lets substantial
# blowdown happen *while* combustion is still occurring, so a large share of
# the heat release shows up as exit gas velocity rather than static pressure
# measured at the peak. Modeling that blowdown-during-combustion coupling
# properly needs the ODE dynamics this model deliberately avoids, so peak
# pressure is anchored directly to this hardware figure instead of derived
# from the (still-used-for-temperature) constant-volume energy balance.
PULSEJET_PEAK_PRESSURE_RATIO = 2.25

# Oswald span efficiency factor for the induced-drag formula
# D_i = L^2 / (q * pi * b^2 * e). Held constant rather than swept.
OSWALD_EFFICIENCY = 0.80

# Zero-lift (parasitic) drag coefficient referenced to vehicle frontal area.
# A flat constant, not Mach-dependent -- see run_demo.py's notes on what
# this leaves out (transonic drag rise).
CD0_FRONTAL = 0.30

# Landing/stall-speed parameters (see drag.py's stall_speed_m_per_s). The
# induced-drag formula only ever needs wingspan, not a reference area, so
# there is no wing area anywhere else in this model -- WING_ASPECT_RATIO
# backs one out from wingspan (S = b^2/AR) purely to define a stall speed,
# rather than adding a 7th sweep variable. Both are simple, tunable
# placeholders, not sourced values.
WING_ASPECT_RATIO = 3.0
CL_MAX = 1.0

# Zero-lift wing drag coefficient, referenced to the same backed-out wing
# reference area (S = b^2/AR) used by stall_speed_m_per_s. Representative of
# a thin, unoptimized flat-plate-ish lifting surface at this Reynolds number
# -- a simple, tunable placeholder like CD0_FRONTAL, not a sourced value.
CD0_WING = 0.02
