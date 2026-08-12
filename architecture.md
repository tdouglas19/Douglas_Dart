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

## 7. Seat preload frozen at 0.5 mm; production campaign launched (2026-08-11)

Preload probes (N=150, t=0.15): xi_0=0.5mm (dP_crack=3.3 kPa) at M=0 gives
**18.08 N @ 150.2 Hz, p 0.792/1.491** -- nearly double the zero-preload
10.04 N, and the frequency lands exactly in the real small-valved class
(130-150 Hz). M=0.15: 14.46 N (the valve-dwell decline mostly cured at low
M). M=0.4 cold-start still quenches (ram bias 11.8 kPa > crack 3.3), as
expected. xi_0=1.0mm (6.7 kPa) FAILS TO SELF-START from the standard
starting condition (air flow 0.00 g/s) -- the start window is sharp, which
mirrors how finicky real pulsejet starting is. **Design freeze: FP-1 uses
xi_0 = 0.5 mm** (now the reference_valve default; 31/31 tests pass with it).

The high-M cold-start quench is reframed as REAL hysteresis physics: cold
starts at speed die by ram blow-through, but a real vehicle accelerates
with the engine running -- the chamber's elevated cycle-mean defends the
valve seal. Production campaign = TWO branches: (a) operating branch by
Mach CONTINUATION (each point seeded from the previous limit cycle via new
engine.snapshot/restore; scripts/mach_continuation.py, N=200 sequential);
(b) cold-start branch (scripts/thrust_vs_mach.py, N=300 parallel, every
point from the static start condition). Their divergence IS the
start/quench hysteresis diagram.

## 8. Adversarial audit round; side-inlet requirement (2026-08-11)

**14-agent audit (6 lenses + verify) ran once quota returned; the verify
phase hit the quota again, so each finding was verified solo before
acting.** Triage of ~25 unique findings:

*Real code defects, fixed (each verified by re-derivation or the finder's
own quantitative repro):* (1) **interdiffusion enthalpy flux missing** --
Y-diffusion at uniform T manufactured ~±33 K at fresh/burned contacts
(cv_R != cv_P), biasing Arrhenius ignition ~35%; added the
(cp_R - cp_P) T dY/dx term to the energy diffusion + a uniform-T regression
test (32nd test). (2) **back-spit Bernoulli relief on the wrong face** --
relief now reduces the driving |dP| in whichever direction the jet flows
(copysign form). (3) **turbulence budget advection** -- intensive form
requires dilution by valve INFLOW (outflow cancels); was decaying by
outflow. (4) **swept volume lumped into one grid cell** -- grid-dependent,
unbounded under refinement; now distributed over the physical petal zone.
(5) **diffusive dt bound missing the gamma factor** (effective diffusivity
is gamma*nu); coefficient 0.4 -> 0.28. (6) **rich-mixture product molar
mass** ignored unburned fuel vapor (phi>1 only).

*Doc corrected to match the (correct) code:* eq. 25 rewritten as the exact
discrete momentum-closure identity (the printed cone-integral sign was
wrong AND the old check was partly coincidental -- now it is exact by
construction, using the wall-star pressure and +|mdot|u_j, with the ram
term made explicit); eq. 9 gains the shear-work and interdiffusion terms;
eq. 23b gains the Borda dump loss + clamp note; valve integrator stated as
symplectic Euler (not RK2); Rayleigh index documented as the unweighted
head-p'/global-q' proxy it is; convergence criterion text aligned;
head-plane flux documented as using the mirror-Riemann star pressure.

*Honest-limitation findings, documented not "fixed":* A6 adiabatic walls is
NOT a few-percent effect at this engine size (real losses ~10-30% of heat
release) -- thrust/TSFC are optimistic by a corresponding margin; the
near-threshold oscillator amplifies parameter changes (a +3.4% q_0 shift
moved thrust +34%), so headline thrust carries the closure uncertainty
(factor ~2 on burn-rate constants) while curve SHAPES and mechanisms are
the robust outputs; my earlier architecture.md "real-class" anchors
(1-3 g/cycle vs TSFC ~3) were mutually inconsistent -- retracted; class
data itself spans TSFC ~0.5 (Dynajet-type claims) to ~3.4 (pulsejet-sim),
so class-anchoring is loose at best.

**New requirement from the user (mid-session): side-mounted valve inlets,
perpendicular to vehicle velocity, ingesting boundary-layer air at ~static
pressure** (the Douglas Dart configuration). Implemented as derivation.md
#8c + A26/A27: p_plen feed = p_a (wall-normal momentum equation), intake
T = recovery temperature (Crocco-Busemann, r=0.9), momentum drag charged at
k_bl*u_inf (BL momentum-integral bracket 0.4-0.9, default 0.6).
IntakeDesign.orientation="side"; all scripts take --side-inlet.
**Smoke-verified: at M=0.3 the side-inlet engine COLD-STARTS and runs at
~13 N where the forward inlet was dying** -- the ram-bias valve-dwell
mechanism is absent exactly as the physics says. Final campaign running:
frozen-config grid study, M=0 production plots, side-inlet operating +
cold-start branches, forward-inlet operating branch for comparison.

## 9. Final campaign complete: the deliverable (2026-08-11)

All post-audit-fix production data, frozen FP-1 design (0.5 mm preload):

**M=0 (N=300, 31 cycles)**: 18.2 N @ 162.3 Hz (N=300 grid-study row);
production plot run 18.6-19.1 N depending on resolution/branch -- p/p0
0.77-1.65, eq.24/eq.25 agreement <1%, Rayleigh strongly positive. Grid
study (N=150/200/300/450): 20.2/19.1/18.2/17.5 N, 157.6/160.4/162.3/164.2
Hz -- ~4%/level thrust drift (the interdiffusion fix made contact physics
sharper and more resolution-sensitive than the pre-fix study's 1.2%);
quoted uncertainty: discretization ~+/-5%.

**Side-inlet (user's vehicle config): operates M 0 -> 0.9, all 19 points
converged on BOTH branches** -- operating: 19.13 -> 11.62 N; cold-start:
18.24 -> 11.39 N; the branches nearly coincide (no start hysteresis; no
ram bias on the petals). Gentle monotone decline from BL momentum drag
(k_bl=0.6) + recovery heating (r=0.9); frequency 160 -> 174 Hz.

**Forward inlet: ram supercharging then valve defeat** -- operating branch
rises to 24.9 N at M=0.15 (+30%), collapses through M=0.25-0.35
(period-irregularity at 0.35), quenched from M=0.40 on both branches.

Final deliverables: out/thrust_vs_mach_FINAL.png (side vs forward, both
branches), thrust_vs_mach_fp1s_side_{operating,coldstart}.csv,
thrust_vs_mach_fp1_fwd_operating.csv, cycle_fp1s_m0*.png (standard
two-panel diagnostics), overview_fp1s_m0.png, grid study log. 32/32 tests.

Open follow-ups, priority order: (1) numba/JIT port of the solver hot loop
(~5-20x) before plugging pulsejet_thrust into any optimizer loop; (2) wall
heat loss (A6) is the biggest known bias at this scale -- needs either an
empirical Nusselt closure (clean-room exception to discuss) or a conjugate
wall model; (3) the audit's verify-phase agents never ran (quota) -- the 6
finder outputs were self-verified, but an independent verification pass
would strengthen the audit record; (4) side-inlet external aerodynamics
(suction/entrainment by the crossflow past the inlet orifice) is neglected
beyond the static-pressure assumption.

## 10. Query-speed package: JIT + early-stop + warm-start (2026-08-12)

User asked for faster single-point queries WITHOUT touching fidelity.
Three levers, all preserving the physics, grid, and convergence criterion:

1. **Numba JIT kernels** (`solver_jit.py`): primitives, fused
   MUSCL+HLLC+sources+diffusion RHS, and the reaction substep as explicit
   loops -- identical formulas/floors/limiter to the numpy reference paths,
   which remain as fallback. Fidelity pinned by
   `tests/test_jit_equivalence.py` (agreement ~1e-10..1e-12 on a stressing
   state with shocks/contacts/area variation/active reaction). Disk-cached
   compilation (cache=True) so only the first process ever compiles.
2. **Online convergence stop** in `pulsejet_thrust`
   (stop_when_converged=True, t_end becomes a cap): the run halts as soon
   as the SAME limit-cycle criterion (6 complete cycles, periods <2%,
   stable per-cycle thrust) is met. Averaging remains cycle-synchronous --
   integer complete cycles between interpolated pressure-crossing
   boundaries -- so early stopping can never truncate mid-pulse (the user
   specifically checked this; it was the design from #13.4-5 day one).
3. **Warm-start** (`seed_state=` / `return_state=True` -> res.end_state):
   restore the full dynamic state from a nearby converged point and
   re-converge in ~5-10 cycles. Branch-safe for the side inlet (both
   branches measured coincident across M 0-0.9); for hysteretic configs
   the seed deliberately selects running-vs-cold-start.

Benchmark, M=0.21 @ 2000 ft side-inlet (N=200, same fidelity):
**317 s (pre-JIT baseline) -> 76 s cold (4.2x) -> 42.8 s warm-started
(7.4x)**; answers 14.02 -> 14.05 N (0.2%, roundoff-path difference),
166.3 Hz both. Suite: 35/35 (3 new equivalence tests; engine tests now
exercise the JIT paths end-to-end). Physics note logged with the M=0.21
point: the altitude thrust penalty (-18% for -5.7% charge density) is ~3x
the linear density-scaling estimate -- the near-threshold amplitude
feedback amplifies ambient changes, so flight-condition queries need the
transient sim, not scaling corrections.
