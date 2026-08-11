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

## 3. First sustained limit cycle -- and the weak-attractor finding (2026-08-11)

FP-1 at M=0 (N=200 probe): **genuine converged limit cycle** -- 183.7 Hz
(right in the small-valved class), **positive Rayleigh index** (the emergent
thermoacoustic driving pulsejet-km could never get), thrust +3.57 N, clean
periodic valve action with realistic back-spit at closing, exhaust pulses
with suction backflow. Two verification wins: the eq.24 vs eq.25 thrust
cross-check discrepancy (3.57 vs 5.63 N) turned out to be a missing term in
the *recording code* (the doc's own intake-jet reaction -mdot*u_j, ~2.1 N --
almost exactly the gap); fixed to match the doc.

But the attractor is WEAK: p/p0 swings only 0.90-1.16 (real class:
~0.6-2.2), per-cycle swallow 0.075 g vs the ~1-3 g/cycle real engines
ingest, and the overview shows heat release never switching off (10-25 kW
floor between 60 kW pulses) -- a smeared burn whose tail overlaps the next
intake. Notably the startup transient (t=35-55 ms) was STRONGER (2.5 mm
lift, 27 N thrust peaks) and decayed INTO the weak attractor.

## 4. Soft valves kill the oscillation; turbulence budget completed (2026-08-11)

**Design probe (negative result, kept):** softer/taller petals (h=0.15 mm,
k=295-337 N/m, xi_max=4.5 mm; two variants incl. wider ports) collapsed the
cycle almost completely (air flow 13.7 -> 0.4 g/s, amplitude to +/-1%,
frequency drifting up to the passive acoustic mode). Physics: a soft valve
leaks at small dP, acting as an acoustic absorber AT the head pressure
antinode -- it destroys the resonator Q. The stiff spring is the nonlinear
gate (rectifier) the oscillation depends on: shut for small fluctuations,
open only under real suction. Flow capacity must come from PORT AREA at
constant spring dynamics, not from softening the spring. (Echoes
pulsejet-km's Section 33 finding that smaller valve = more amplitude, now
with a mechanism.)

**Model completeness fix:** the k_c budget had only jet production, so
chamber turbulence died ~0.5 ms after valve closing and the flame crawled
through the slug (~4 m/s effective) -- the source of the smeared burn tail.
Added the standard k-equation mean-shear production term
nu_t*(du/dx)^2 (first-principles Reynolds-stress work against mean shear,
weighted to the chamber zone) and set C_eps 1.0 -> 0.5 (decaying-turbulence
integral-scale value; still the O(1) dimensional-analysis constant A11
declares, revised once during initial calibration, never per-case).

**Performance:** step cost 7.7 -> 2.79 ms (2.8x) via allocation-lean HLLC,
threaded primitives (10 -> 4 computations/step), scalar boundary HLLC, and
fused mixture-property arithmetic. Full suite still 31/31 (and 2x faster).
One real bug caught during the rewrite before it ran: gamma_mix interpolated
as sum-of-ratios instead of ratio-of-sums in the fast path -- would have
introduced a small thermodynamic inconsistency between reconstruction and
recovery.

## 5. Intake-column inertia: the biggest single physics gain (2026-08-11)

Run C (turbulence completeness, baseline valve): 5.24 N converged, and the
eq.24/eq.25 thrust cross-check now agrees to 2.3% (5.24 vs 5.36) -- the
recording fix verified. Run D (12 petals): more air (19.8 g/s) but similar
thrust, unconverged wobble -- port area alone is not the binding constraint.

**Run E: added the missing intake subsystem** (derivation.md #6b, eq. 23b,
A24): the valve now draws from a duct-column + plenum with real momentum
(inertance ODE pair, symplectic update, Borda-Carnot dump loss damping the
44 kHz plenum mode) instead of an infinite reservoir. Result: **7.85 N**
(+50% over C; progression 3.57 -> 5.24 -> 7.85 across the three physics
completions), p/p0 0.843-1.273, 168.3 Hz, Rayleigh +1.55e6, and the two
independent thrust formulations agree to **0.4%** (7.85 vs 7.82 N) -- the
strongest internal-consistency verification the model has produced. The
cycle zoom shows the signature the reservoir model could never make: a
TWO-LOBED intake pulse (suction pull, then the column's ram-through second
push), plus the asymmetric relaxation-oscillator pressure shape emerging.

Interpretation of the remaining gap to the 20-30 N class: specific thrust
per unit air is HIGH (~420 m/s effective) -- the model engine is
charge-starved, not inefficient. Real engines of this class likely swallow
far more air (and run rich, wasting fuel -- their TSFC ~3+ vs our 0.55).
Next: combined hardware-plausible config (12 petals + 100 mm intake runner
+ phi=1.05) as run F.

## 6. Corrected 0K heat referencing; the high-Mach quench mechanism (2026-08-11)

**Thermo audit fix (solo audit, subagent quota exhausted for the session):**
the sensible-energy convention e = cv(Y)*T is 0K-referenced, so the release
constant must be the 0K heat of reaction q_0 = De(298) + (cv_P - cv_R)*298
= 2.875 MJ/kg (was using the 298K value 2.780 -- 3.4% low). Verified the
scheme's structure was already correct (the cv-swap makes effective release
q_0 - (cv_P-cv_R)T automatically); only the constant moved. Eq. 4 updated.
M=0 production (N=300, 32 cycles): **10.04 N, 169.4 Hz, p 0.832/1.300,
cross-check 9.70 N (3.4%), TSFC 0.484.**

**First full Mach sweep (19 pts, N=300) exposed a real failure mode:**
thrust declines from M=0.05 (10.0 -> 5.2 N by M=0.15), goes chaotic around
M=0.2-0.3, and above M~=0.35 the engine QUENCHES into a steady blow-through
state (passive ~700 Hz acoustic ring, p_head pinned near the ram stagnation
value, "thrust" = pure ram drag of a dead engine, -3 to -36 N). Mechanism
(same physics as the Section 4 soft-valve death): ram pressure biases the
plenum above the chamber's cycle-mean pressure, holding the zero-preload
petals cracked open continuously -- the valve stops rectifying, leaks at
the head antinode, kills the resonator Q, combustion dies. The low-M thrust
decline is the onset of the same valve-dwell leak.

**Fix (real hardware physics, not a tuning knob): seat preload** -- real reed
petals are manufactured with residual curvature pressed flat on the seat,
storing deflection xi_0 in the spring; the valve cracks open only above
dP_crack = k*xi_0/((3/8)A_petal). Added to valve.py (spring term k(xi+xi_0),
A25, eq. 19 updated); this is precisely what lets real valved engines hold
seal under forward-flight ram bias. Probing xi_0=0.5mm (dP_crack=3.3kPa)
at M=0/0.15/0.4.
