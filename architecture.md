# pulsejet-fp — technical log

Running record of every decision and result, newest section last. (Convention
borrowed from the sibling pulsejet-km repo, which proved its worth there.)

## 1. Charter and clean-room boundary (2026-08-11)

Request: derive a complete first-principles computational query function for
a valved pulsejet cycle from scratch — explicitly NOT copying/adapting
external source code, textbook pulsejet equations, or empirical charts; no
static discharge coefficients, no pre-calculated valve delay times; valve
area must come from physical deflection dynamics. Output: a queryable
function (inputs → thrust), proven by a thrust-vs-Mach plot for a chosen
standard design. Kept deliberately separate from Douglas_Dart AND from
pulsejet-km (whose Khrulev & Muntyan paper-transcription equations are
exactly what must not be copied here).

Context that shaped the architecture (from pulsejet-km's hard-won lessons,
used as *requirements*, not as equations): lumped 0D models cannot produce
Rayleigh-criterion phasing naturally (heat release ends up out of phase and
damps the oscillation — that project's central unresolved struggle);
quasi-steady valve laws are known to give large mass-flow/phase errors
(NASA finding cited there: ~75%); the sibling Douglas_Dart audit flagged the
same two gaps (no heat-release phasing physics, no tailpipe inertia).
Consequences here: (1) the whole engine is ONE quasi-1D unsteady
compressible domain (wave dynamics resolved, so phasing and the "liquid
piston" emerge), (2) the valve is a genuine inertial ODE from beam theory,
(3) combustion is Arrhenius kinetics with real activation energy so
ignition timing responds to wave-driven compression.

## 2. Model + code complete; validation battery green (2026-08-11)

`docs/derivation.md` written first (the contract), code implements it:
gas (equipartition thermochemistry from stoichiometry), atmosphere
(hydrostatic), geometry (smooth quasi-1D area), valve (beam ODE + contact +
geometric curtain area), orifice (choked compressible flow, choking derived),
solver (HLLC with derived star states, MUSCL-minmod, SSP-RK2, turbulent
diffusion, exact-exponential reaction substep), engine (couples everything;
variable-volume head cell carries the petal swept-volume work term), query
(cycle detection, limit-cycle test, Rayleigh index, thrust + surface-integral
cross-check), diagnostics (standard two-panel p/p0-va/a0-ve/a0 + T/T0 plot,
per the standing convention).

31/31 tests pass: Sod shock tube vs an exact Riemann solution computed
in-test from Rankine-Hugoniot (star pressure/velocity within 3%, shock
position within 2 cells); closed-box mass/energy conservation to 1e-12;
reaction energy bookkeeping exact (dE = q_R * burned mass to 1e-9);
closed-closed acoustic fundamental within 3% of a/2L; valve free-vibration
frequency within 2% of sqrt(k/m)/2pi; contact restitution; orifice choked
flux vs independent closed form to 1e-10; engine smoke tests.

**Finding 2a (real bug found by the acoustic test): `wall_pressure` was
left-wall-only.** Used on a right wall the sign convention inverts —
compression read as suction, negative damping, the standing wave got pumped
until nonlinear collapse (pressure at the wall fell to ~400 Pa while mass
and energy stayed conserved to 1e-12 — conservation tests alone would never
have caught it). The engine only uses the left-wall case (head plate), so
engine results were never affected. Fixed with an explicit `side` argument.

**Finding 2b (the first physics lesson, and the reason the strain-quench
closure exists): a bare 1D cell-average model predicts a valve-anchored
continuous flame.** First engine runs admitted fresh charge but flash-burned
it in the entry cell (reactant inventory peaked at 3.6e-8 kg, four orders
low, no pulsation possible — an eerily exact echo of pulsejet-km's
DECAY_TO_FIXED_POINT). Diagnosis: the entering charge dilutes into a
~1700 K residual-gas cell whose Arrhenius time (~10 us) is far shorter than
the cell fill time (~0.2 ms). The missing physics is jet-scale: the intake
jet's shear strain (u_j/xi ~ 1e5 1/s) exceeds propane's extinction strain
(~1e3-1e4 1/s) by orders of magnitude — a real flame CANNOT anchor in the
valve jet; it blows off, which is exactly why real valved engines
accumulate charge during intake. Added derivation.md #5b: Zeldovich-number
strain-extinction factor F_q = 1/(1+(s Ze^2 tau_chem)^2) with the jet
strain field s = (u_j/xi) exp(-x/L_jet), L_jet = 15 xi from self-similar
jet decay (A23). The Ze^2 reaction-zone scaling also exposed that the
original A_r = 8e8 gave flame speeds ~80x too fast; recalibrated once to
A_r = 2e7 (still the single kinetic calibration constant, A9). With this,
charge genuinely accumulates during intake and burns after it — pulsation
physics restored.

Next: full-resolution reference run at M=0 — does FP-1 sustain a limit
cycle, and are thrust/frequency in the physical class (small-valved-engine
sanity anchors: ~20-30 N, ~130-150 Hz)?
