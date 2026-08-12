# simple_model overview — closed-form pulsejet→ramjet dart sizing

Purpose: size and optimize a 50 lb wet-mass pulsejet→ramjet supersonic dart
(diameter, throat, chamber/tube lengths, wingspan, fuel, climb angle) by
running a full launch-to-landing flight simulation for thousands of
candidate geometries and picking the one that minimizes peak
thrust-to-weight ratio, subject to feasibility (reaches cutoff Mach, lands
safely at a capped stall speed, fuel fits in the vehicle).

The core design constraint is **speed**: every physics relationship is a
single closed-form algebraic expression — no ODE integration inside a
physics call, no iterative solvers, nothing that can fail to converge. The
only marching is the outer point-mass trajectory (explicit Euler, fixed
small dt). Seven search variables (diameter, throat diameter, chamber
length, throat/tube length, wingspan, climb angle, fuel type); everything
else is a fixed constant. This is the deliberately-simple sibling of the
higher-fidelity `douglas_dart/` package and the first-principles
`pulsejet-fp/` model.

## Physics, module by module

**Pulsejet** (`pulsejet_simple.py`) — Helmholtz resonance frequency from
chamber+neck geometry; a fresh ambient-static charge each cycle (side
inlets, no ram credit); constant-volume energy balance for peak
temperature; hardware-anchored peak pressure; choked-throat mass flow and
isentropic exit velocity for peak thrust; a duty-cycle factor for average
thrust.

**Ramjet** (`ramjet_simple.py`) — captured flow `rho*V*A`; ideal isentropic
ram compression; stoichiometric constant-cp energy balance; compressible-
orifice throat flow (chokes automatically); perfectly-expanded nozzle;
momentum-flux net thrust.

**Drag** (`drag.py`) — body parasitic (`CD0*q*A_frontal`, with a transonic
rise multiplier), wing parasitic (area backed out from span via aspect
ratio), induced (`L^2/(q*pi*b^2*e)`, span-only form).

**Trajectory** (`flight_sim.py`) — powered climb at an optimized fixed
angle with BOTH engines summed through the transition; motor cutoff at
target Mach; unpowered level **deceleration segment** (drag sheds the
supersonic speed while altitude holds), then a fixed-angle glide, then a
kinematic flare to stall speed = landing.

## Calibration against the first-principles model (2026-08-12)

Five constants were calibrated against `pulsejet-fp/` (quasi-1D transient
wave dynamics + reed-valve ODE + Arrhenius kinetics), using three validated
operating points spanning 8x thrust and 2.8x scale. Each showed a
*consistent* cross-scale correction — the signature of a systematic model
bias rather than noise:

| constant | old | calibrated | meaning |
|---|---|---|---|
| duty-cycle factor | 1/3 (guess) | **0.08** | old value overpredicted thrust 4.0–4.7x at every scale |
| peak pressure ratio | 2.25 (hardware anchor) | **2.16** | the anchor was nearly right |
| Helmholtz frequency factor | 1.0 | **0.59** | end corrections + cold-charge column (model read 1.7x high) |
| chamber fill fraction | 1.0 | **0.15** | real engines swallow ~15% of a chamber volume per cycle |
| Mach thrust slope | 0 (no dependence) | **0.43** | side-inlet thrust falls ~43%/Mach (BL drag + recovery heating) |

Post-calibration residuals vs the truth model: thrust, frequency, air flow,
and fuel flow all within ~10% at all three anchor scales.

**Operability gates** (closed-form inequalities, calibrated boundaries):
throat/chamber area fraction ≤ 0.30 (0.43 is a dead resonator — the
previous unconstrained optimizer picked exactly such an engine) and cycle
frequency ≤ 220 Hz (absolute ~1 ms mixing/ignition lag cannot phase-lock
faster). The ramjet has a minimum-lightoff Mach gate (0.45, placeholder
until ramjet-fp derives it).

**Feasibility geography under calibrated physics** (sea level, 50 lb):
peak thrust scales with throat area and drag with frontal area, so the
pulsejet's level-flight Mach ceiling depends almost only on CD0 and the
area fractions — at the old CD0=0.30 placeholder the ceiling (~M 0.40)
sits *below* ramjet lightoff for every geometry (mission infeasible); at
CD0≈0.10–0.15 the handoff closes. The feasible corner is narrow: large
diameter (~0.28–0.30 m), max operable throat fraction, small-but-landable
wingspan (~0.7–0.8 m), near-level climb (~1°). Hence the CD0-sensitivity
campaign in `scripts/simple_model_overnight.py`.

## Known simplifications (unchanged)

Calorically-perfect gas; no dissociation chemistry (flat LHV x efficiency,
2600 K cap); no valve dynamics; no spillage drag; no structural/dry-mass
model; no CG/stability check; fixed climb/glide angles; kinematic flare;
no wind; quasi-steady thrust; explicit-Euler integration.
