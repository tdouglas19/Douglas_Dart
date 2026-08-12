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


# --- Calibration against the first-principles pulsejet model (2026-08-12) --
# The constants below were fitted/derived from pulsejet-fp (this repo,
# `pulsejet-fp/` -- quasi-1D transient wave dynamics + reed-valve ODE +
# Arrhenius kinetics), using three validated operating points spanning 8x in
# thrust and 2.8x in linear scale (FP-1 18.6 N, 25 L vehicle-scale ~146 N,
# T/W-66mm 52.2 N). Provenance: pulsejet-fp/architecture.md sections 9-11.
# Caveat inherited from the truth model: adiabatic walls make it optimistic,
# so these calibrated numbers are best-estimates, not conservative bounds.

# Fitted jointly with the duty-cycle factor (log-least-squares over the three
# anchors, residual +/-9%): the old hardware-anchored 2.25 was nearly right.
# (This REPLACES the 2.25 definition above at import time.)
PULSEJET_PEAK_PRESSURE_RATIO = 2.16

# Side-inlet pulsejet thrust DOES fall with Mach (boundary-layer momentum
# drag + viscous recovery heating of the ingested charge): linear fit to the
# pulsejet-fp FP-1S operating branch, F(M)/F(0) ~= 1 - 0.43*M over M 0-0.9.
# Supersedes this model's original "no Mach dependence" assumption.
PULSEJET_MACH_THRUST_SLOPE = 0.43

# Altitude: the near-threshold oscillator amplifies density loss -- measured
# -18% thrust for -5.7% charge density at 2000 ft (single-point calibration,
# uncertainty ~+/-1 on the exponent). Total scaling ~ (rho/rho_SL)^3;
# the charge mass already contributes rho^1, the rest is applied explicitly.
PULSEJET_ALTITUDE_DENSITY_EXPONENT = 3.0

# Operability gates (closed-form inequalities, zero runtime cost):
# (1) Throat/chamber AREA ratio: the resonator cannot sustain combustion if
#     the throat vents too freely. Bracketed empirically with pulsejet-fp:
#     0.29 sustains strongly, 0.43 is stone dead (the un-constrained
#     optimizer's own "T/W-optimized" 81/123 mm pick landed exactly there).
PULSEJET_MAX_THROAT_AREA_FRACTION = 0.30
# (2) Cycle frequency ceiling: charge transport/mixing/ignition needs ~1 ms
#     absolute, so above roughly 220-250 Hz the heat release physically
#     cannot phase-lock with the pressure wave (pulsejet-fp demonstrated a
#     ~350 Hz compact design as unconditionally dead). Real valved engines
#     cluster below ~250 Hz at ANY scale.
PULSEJET_MAX_FREQUENCY_HZ = 220.0

# Ramjet minimum viable speed: below this the flameholder cannot stabilize /
# net thrust is not positive. Constant borrowed from douglas_dart's own
# RamjetConfig lightoff machinery (vehicle configs use ~0.49); a proper
# first-principles number is the planned ramjet-fp effort's headline output.
RAMJET_MIN_LIGHTOFF_MACH = 0.45
RAMJET_LIGHTOFF_RAMP_MACH = 0.10  # linear ramp width above the minimum

# Transonic drag rise on the body CD0 (flat CD0 badly underestimates drag
# right where a supersonic dart works hardest). Simple continuous blend:
# flat to M=0.8, quadratic rise to a peak multiplier at M=1.1, then decaying
# supersonic wave-drag tail ~ (1.1/M)^2. Representative slender-body values,
# not fitted to this vehicle.
TRANSONIC_ONSET_MACH = 0.8
TRANSONIC_PEAK_MACH = 1.1
TRANSONIC_PEAK_CD0_MULTIPLIER = 2.2

# Helmholtz frequency correction: the idealized no-end-correction, hot-gas
# formula overpredicts the real cycle frequency by a consistent 1.65-1.8x
# across all three pulsejet-fp anchors (model 277/98/147 Hz vs true
# 150.5/59.2/88.7 Hz -> ratios 0.56/0.60/0.60). Physically: neck end
# corrections plus the cold fresh-charge fraction of the oscillating gas
# column. One multiplicative constant, fitted 2026-08-12.
HELMHOLTZ_FREQUENCY_CALIBRATION = 0.59

# Chamber fill fraction per cycle: the "one full ambient chamber volume per
# cycle" assumption overfeeds the engine ~6-7x. pulsejet-fp charge-per-cycle
# vs rho*V_chamber: 0.16 (FP-1), 0.13 (25L), 0.18 (TW66) -> 0.15 fitted.
# Affects air/fuel flow and Isp only -- peak thrust is choked-flow-based and
# unaffected, so the thrust calibration above stands independently.
PULSEJET_CHAMBER_FILL_FRACTION = 0.15

# Sensitivity hook: the flat CD0_FRONTAL=0.30 placeholder is now the single
# constant deciding overall feasibility (calibrated pulsejet thrust tops out
# at M~0.40 level-flight against it -- below ramjet lightoff -- while a
# realistic clean slender body at 0.10-0.20 closes the transition gap).
# Overridable via environment for sensitivity campaigns, without touching
# call sites; import-time only, so the hot loop cost is zero.
import os as _os

CD0_FRONTAL = float(_os.environ.get("SIMPLE_MODEL_CD0_FRONTAL", CD0_FRONTAL))

# Minimum powered-flight thrust margin (2026-08-12, user requirement):
# feasibility requires thrust >= (1 + margin) * (drag + weight-along-path)
# at EVERY powered timestep, not just net-positive acceleration. The
# calibrated optima otherwise ride thrust ~= drag exactly at the
# pulsejet->ramjet transition -- a vehicle underperforming by a few percent
# would stall below ramjet lightoff and never complete the mission. 0.15
# covers the calibration's own ~10% cross-scale residuals plus closure
# uncertainty with headroom; the multiplicative form maps directly onto
# "engines deliver X% less thrust than modeled."
MIN_POWERED_THRUST_MARGIN_FRACTION = 0.15

# --- Wing-concept model (2026-08-12, wing optimizer) ----------------------
# Closed-form wing description for simple_model/wing_optimize.py. The
# DEFAULT concept reproduces the original fixed-wing constants exactly
# (AR=3, rectangular, unswept, CL_MAX=1.0, CD0_WING=0.02, e=0.80), so the
# vehicle optimizer's behavior is unchanged unless a concept is supplied.
from dataclasses import dataclass as _dataclass


@_dataclass(frozen=True)
class Airfoil:
    key: str
    display_name: str
    cl_max: float          # low-speed, unswept maximum lift coefficient
    cd0_wing: float        # profile drag coefficient (wing reference area)
    thickness_ratio: float # t/c, drives supersonic wave drag


# Representative closed-form values for four buildable concepts -- simple
# placeholders in the same spirit as FUELS, not section data.
AIRFOILS: dict[str, Airfoil] = {
    "flat_plate": Airfoil("flat_plate", "Flat plate (sharp)", 0.80, 0.015, 0.03),
    "thin_cambered": Airfoil("thin_cambered", "Thin cambered plate", 1.20, 0.020, 0.04),
    "naca_symmetric": Airfoil("naca_symmetric", "Symmetric NACA-ish", 1.00, 0.020, 0.09),
    "supersonic_wedge": Airfoil("supersonic_wedge", "Double wedge (supersonic)", 0.70, 0.012, 0.04),
}

# Oswald efficiency vs taper ratio, lifting-line flavor: induced-drag
# factor delta(lambda) is minimal near lambda ~ 0.35 (closest to elliptic
# loading) and worst for rectangular (lambda = 1). Quadratic fit anchored
# so a rectangular AR=3 wing returns exactly the legacy e = 0.80.
OSWALD_TAPER_MIN_DELTA_AT = 0.35
OSWALD_BASE_E = 0.85          # near-optimal taper, with fuselage interference
OSWALD_TAPER_PENALTY = 0.118  # (lambda - 0.35)^2 coefficient -> e(1.0)=0.80

# Supersonic wing wave drag: cd_wave ~ K_WAVE * (t/c)^2 / sqrt(M_n^2 - 1)
# on the wing area, onset when the component of Mach normal to the leading
# edge (M * cos(sweep)) exceeds the critical value. Standard thin-wing
# closed form, smoothly blended over WING_WAVE_ONSET_WIDTH in normal Mach.
WING_WAVE_K = 4.0
WING_CRITICAL_NORMAL_MACH = 0.85
WING_WAVE_ONSET_WIDTH = 0.15

# --- Parametric mass model (2026-08-12, user requirement) ------------------
# Closed-form structural/auxiliary mass so oversized geometry busts the
# 50 lb wet-mass budget instead of being free. See simple_model/mass_model.py.
# Engine duct (chamber + resonance tube): STEEL -- it runs hot (pulsejet
# cycle peaks ~2400 K wall-adjacent); composite is not credible there.
STEEL_DENSITY_KG_M3 = 7850.0
STEEL_ALLOWABLE_STRESS_PA = 125e6   # ~250 MPa yield / SF 2, hot-degraded
STEEL_MIN_GAUGE_M = 1.0e-3          # manufacturable rolled-sheet floor
# Outer airframe skin + nose/tail: carbon fiber laminate.
CFRP_DENSITY_KG_M3 = 1600.0
CFRP_MIN_GAUGE_M = 1.5e-3
# Pressure-vessel sizing: hoop stress t = p_gauge * R / sigma_allow, with
# the design gauge pressure taken from the pulsejet peak chamber pressure
# (PULSEJET_PEAK_PRESSURE_RATIO - 1) at sea level -- the worst case the
# duct sees. At ~1.2 atm gauge the min-gauge floor dominates for any sane
# diameter, which is itself the realistic outcome at this scale.
STRUCTURAL_OVERHEAD_FRACTION = 0.35  # frames, longerons, fasteners, fins
NOSE_TAIL_LENGTH_DIAMETERS = 3.0     # nose cone + boattail length, in D
WING_AREAL_MASS_KG_M2 = 6.0          # solid-ish small supersonic wing panel
AVIONICS_FIXED_MASS_KG = 2.0         # autopilot, batteries, servos, RF
TANK_HARDWARE_FIXED_KG = 0.5         # valves, plumbing, regulator
TANK_HARDWARE_FUEL_FRACTION = 0.15   # tank shell scales with fuel carried
LANDING_HARDWARE_KG = 0.5            # skids/attach points

# --- Composite design objective (2026-08-12, user requirement) -------------
# "Some relationship between optimizing for both T/W, overall diameter,
# overall length, and wingspan": a weighted sum of normalized terms,
# score = W_TW*(peakTW/10) + W_D*(D/0.30) + W_L*(L_body/2.5) + W_B*(b/1.5)
# (denominators = rough upper-bound scales so each term is O(1)). Lower is
# better; feasibility gates are unchanged and absolute. THE tunable knob
# for design taste -- re-weighting only needs the saved Pareto set
# (out_simple_model/*pareto*.json), not a re-run.
OBJECTIVE_WEIGHT_TW = 1.0
OBJECTIVE_WEIGHT_DIAMETER = 0.5
OBJECTIVE_WEIGHT_LENGTH = 0.3
OBJECTIVE_WEIGHT_SPAN = 0.2
OBJECTIVE_TW_SCALE = 10.0
OBJECTIVE_DIAMETER_SCALE_M = 0.30
OBJECTIVE_LENGTH_SCALE_M = 2.5
OBJECTIVE_SPAN_SCALE_M = 1.5
