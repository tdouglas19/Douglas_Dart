# First-Principles Derivation of a Valved Pulsejet Transient Cycle Model

This document derives, from fundamental physical laws only, the complete
governing-equation set, discretization, and query-loop algorithm implemented in
`src/pulsejet_fp/`. No external pulsejet source code, textbook pulsejet
correlations, empirical discharge coefficients, or pre-calculated valve delay
times are used anywhere. Every closure that cannot be obtained analytically is
stated explicitly as a numbered assumption (A1, A2, ...) with the fundamental
law that justifies its form, collected in §14.

Notation: $\rho$ density, $u$ axial velocity, $p$ pressure, $T$ temperature,
$E = e + u^2/2$ specific total energy, $e$ specific internal energy, $Y$ mass
fraction of unburned reactant mixture, $A(x)$ duct cross-sectional area,
$a$ speed of sound, $\gamma$ ratio of specific heats, $R$ specific gas
constant. Subscript $a$ = ambient, subscript 0 = stagnation. SI units
throughout.

---

## 1. Gas model from kinetic theory

**Equation of state.** For a dilute gas of non-interacting molecules, kinetic
theory (momentum flux of molecules striking a wall, Maxwell–Boltzmann velocity
distribution) gives

$$ p = \rho R T, \qquad R = R_u / W \tag{1} $$

with $R_u = 8.314463\ \mathrm{J\,mol^{-1}K^{-1}}$ and $W$ the molar mass.

**Species.** The gas is a binary mixture of two pseudo-species (A1):

- **Reactant** "R": premixed fuel–air at equivalence ratio $\phi$.
- **Product** "P": equilibrium combustion products.

$Y$ is the reactant mass fraction. Mixture properties are mass-weighted:

$$ R_{mix} = Y R_R + (1{-}Y) R_P, \quad c_{v,mix} = Y c_{v,R} + (1{-}Y) c_{v,P} \tag{2} $$

**Heat capacities from equipartition.** Each fully-excited quadratic degree of
freedom carries $\tfrac12 R T$ per unit mass (equipartition theorem). The
reactant is ~94% diatomic air by mass: 3 translational + 2 rotational modes,
$c_{v,R} = \tfrac52 R_R$, $c_{p,R} = \tfrac72 R_R$, $\gamma_R = 7/5$. Products
contain triatomic CO₂/H₂O whose bending/vibrational modes are partially
excited at flame temperatures; we take an effective 7 quadratic modes,
$c_{v,P} = \tfrac72 R_P$, $c_{p,P} = \tfrac92 R_P$, $\gamma_P = 9/7 \approx 1.286$
(A2: constant effective $c_v$ per species — calorically perfect pseudo-species;
justified by equipartition with a fixed vibrational-activation fraction over
the ~800–2600 K product temperature range).

**Molar masses** follow from stoichiometry, no data tables needed beyond
atomic weights. For propane, $\mathrm{C_3H_8} + 5\,\mathrm{O_2} \to
3\,\mathrm{CO_2} + 4\,\mathrm{H_2O}$. Air is 23.14% O₂ by mass, so the
stoichiometric fuel–air mass ratio is

$$ f_{st} = \frac{W_{C_3H_8}}{5\,W_{O_2}} \times 0.2314 = \frac{44.097}{159.99}\times 0.2314 = 0.0638 \tag{3} $$

At mixture ratio $f = \phi f_{st}$ (per kg air), the reactant molar mass is
$W_R = (1+f) / (1/W_{air} + f/W_{fuel})$; the product molar mass follows from
mole balance of the reaction (6 → 7 moles per mole propane). For $\phi = 1$:
$W_R = 29.58$, $W_P = 28.43\ \mathrm{g\,mol^{-1}}$.

**Internal energy and heat release.** We use *sensible* internal energy
$e = c_{v,mix}(Y)\,T$ referenced to 0 K, and account for chemical energy
explicitly: converting reactant to product at fixed $T$ releases heat $q_R$
per kg of reactant converted, where

$$ q_R = \frac{f}{1+f}\,\Delta h_c + (c_{v,P} - c_{v,R})\,T_{ref} \tag{4} $$

with $\Delta h_c$ the fuel's lower heating value (a thermophysical material
property, input; propane $46.35\ \mathrm{MJ\,kg^{-1}}$, quoted at
$T_{ref} = 298.15$ K). The second term is the 0 K-referencing correction:
since the sensible energy convention is $e = c_{v,mix}(Y)T$ from 0 K, the
release constant must be the 0 K heat of reaction, obtained from the 298 K
value by accounting for the $c_v$ swap between species; the effective heat
released when converting at temperature $T$ is then
$q_R - (c_{v,P}-c_{v,R})T$, exactly as energy conservation with per-species
$c_v$ requires (the $\Delta(pv)$ term between $\Delta h$ and $\Delta e$ is
~0.1% and neglected). This sensible-energy
+ explicit-source formalism is algebraically identical to the
formation-enthalpy formalism (shift of energy reference per species) (A3:
dissociation of products neglected — overpredicts flame temperature by
~5–8%; justification: dissociation equilibria are strongly endothermic only
above ~2400 K, which the cycle mean stays below).

**Speed of sound.** Perturbing (1) isentropically: $ds = 0$ with
$T\,ds = de + p\,d(1/\rho)$ gives $dp/d\rho|_s = \gamma R T$, so

$$ a = \sqrt{\gamma_{mix} R_{mix} T}, \qquad \gamma_{mix} = \frac{c_{v,mix}+R_{mix}}{c_{v,mix}} \tag{5} $$

---

## 2. Ambient atmosphere from hydrostatics

For altitude $h$, hydrostatic equilibrium $dp/dh = -\rho g$ combined with (1)
and a linear troposphere lapse $T = T_{sl} - L h$ (observed mean lapse
$L = 6.5\ \mathrm{K\,km^{-1}}$, an input) integrates to

$$ p(h) = p_{sl}\left(1 - \frac{L h}{T_{sl}}\right)^{g/(R_{air} L)} \tag{6} $$

with $p_{sl} = 101325$ Pa, $T_{sl} = 288.15$ K. The demo runs at sea level;
(6) exists so altitude is a valid query input.

---

## 3. Unsteady quasi-1D conservation laws (chamber + tailpipe as one domain)

The engine interior — valve head plane ($x=0$), combustion chamber, conical
transition, tailpipe, exit plane ($x=L$) — is a single rigid duct of smooth
area $A(x)$ (A4: flow quantities uniform over each cross-section — quasi-1D;
justified because duct slenderness $D/L \ll 1$ and acoustic wavelengths
$\gg D$ make transverse equilibration fast compared to axial dynamics).

Apply the Reynolds transport theorem to the control volume between $x$ and
$x+dx$ bounded by the duct wall.

**Mass.** No mass crosses the wall:

$$ \frac{\partial (\rho A)}{\partial t} + \frac{\partial (\rho u A)}{\partial x} = 0 \tag{7} $$

**Axial momentum** (Newton's second law). Surface forces: pressure $pA$ on the
two faces, plus the axial projection of wall pressure. The wall element has
axial-projected area $dA = A(x{+}dx) - A(x)$, and pushes on the gas with axial
force $+p\,dA$ (outward wall normal has axial component $-dA/ds$). Adding the
turbulent momentum diffusion closure (§6):

$$ \frac{\partial (\rho u A)}{\partial t} + \frac{\partial \left[(\rho u^2 + p) A\right]}{\partial x} = p \frac{dA}{dx} + \frac{\partial}{\partial x}\!\left(\rho \nu_t A \frac{\partial u}{\partial x}\right) \tag{8} $$

(A5: wall shear/skin friction neglected relative to pressure forces —
inviscid walls; justified by scale analysis: for this geometry the
friction impulse per cycle $\sim \tfrac{f_D}{2}\rho u^2 \pi D L\, \tau$ is
below ~3% of the pressure impulse; consequence noted: slight overprediction
of exhaust velocity.)

**Energy** (first law for the CV). Work is done by pressure at the moving
fluid faces ($puA$ flux) and heat is released by reaction and transported by
turbulent diffusion:

$$ \frac{\partial (\rho E A)}{\partial t} + \frac{\partial \left[(\rho E + p) u A\right]}{\partial x} = \rho A\, q_R\, \dot\omega + \frac{\partial}{\partial x}\!\left[\rho \nu_t A\left( c_{p,mix} \frac{\partial T}{\partial x} + (c_{p,R} - c_{p,P})\, T \frac{\partial Y}{\partial x} + u \frac{\partial u}{\partial x}\right)\right] \tag{9} $$

The three diffusive contributions are: heat conduction; the **interdiffusion
enthalpy flux** — species carry their partial sensible enthalpy
$h_k = c_{p,k}T$, and since $c_{v,R} \ne c_{v,P}$ omitting this term would
let $Y$-diffusion at uniform temperature manufacture spurious ~30 K extrema
at every fresh/burned contact (pinned by a regression test); and the work of
the turbulent shear stress on the mean flow (the energy conjugate of the
momentum diffusion term in (8)).

(A6: adiabatic walls — wall heat loss neglected. Justification: energy flux
balance — convective wall loss per cycle is a few % of heat release for
engine-scale surface/volume ratios; consequence: slightly hot cycle.)

**Reactant species.** The reactant is carried by the flow, destroyed by
reaction at rate $\dot\omega$ (kg reactant per kg mixture per second), and
mixed by turbulent diffusion:

$$ \frac{\partial (\rho Y A)}{\partial t} + \frac{\partial (\rho u Y A)}{\partial x} = -\rho A \dot\omega + \frac{\partial}{\partial x}\!\left(\rho \nu_t A \frac{\partial Y}{\partial x}\right) \tag{10} $$

(A7: turbulent Prandtl and Schmidt numbers of unity — the same eddies carry
momentum, heat, and species; this is the Reynolds analogy, justified by the
common turbulent-transport mechanism.)

Equations (7)–(10) are the complete field equations. They support shocks,
rarefactions, and contact discontinuities (the hyperbolic part is exactly the
1D Euler system extended by area variation and species), which is how the
tailpipe's "liquid piston" gas column, Kadenacy suction phase, and returning
compression waves emerge without any separate ad-hoc submodel.

---

## 4. Combustion closure: single-step Arrhenius kinetics

The law of mass action for a global one-step reaction, first-order in
reactant (A8: single-step global kinetics, first order in the premixed
reactant mass fraction — justified because the premixture has fixed
stoichiometry so a single progress variable suffices), with the Arrhenius
temperature dependence that follows from the Maxwell–Boltzmann fraction of
collisions exceeding the activation energy:

$$ \dot\omega = Y\, A_r\, e^{-T_a / T}, \qquad T_a = E_a / R_u \tag{11} $$

Parameters: $E_a = 125.5\ \mathrm{kJ\,mol^{-1}}$ ($T_a = 15\,100$ K), inside
the well-established 100–170 kJ/mol range for global hydrocarbon oxidation —
a *gas property* input, giving real temperature sensitivity (ignition delay
falls steeply as residual-gas mixing heats the fresh charge). The
pre-exponential $A_r$ is the single kinetic calibration constant (A9): a
one-step surrogate cannot match a full mechanism at all temperatures, so
$A_r$ is fixed once (default $2\times10^{7}\ \mathrm{s^{-1}}$) such that the
ZFK front speed (12) with the Zeldovich correction of §5b lands in the
10–40 m/s turbulent-deflagration range while the reaction time
$1/(A_r e^{-T_a/T})$ is ~0.6 ms at 1600 K (hot-interface ignition), ~5 ms at
1300 K, and effectively infinite below 1000 K; it is *never* re-tuned per
flight condition or per cycle.

With (11), ignition phasing is **emergent**: fresh charge (300–400 K) is
chemically frozen; where turbulent diffusion mixes it with hot residual gas
the rate rises exponentially; combustion accelerates as pressure waves
compress and heat the mixture. This is precisely the coupling the Rayleigh
criterion (§10) measures.

---

## 5. Chamber turbulence: energy-budget closure for deflagration rate

A planar laminar flame in 1D is far too slow to burn the charge in the
few-ms available; the real mechanism is intense turbulence generated by the
valve jets, which wrinkles the flame and multiplies the burning rate. The
1D model carries this with a turbulent eddy viscosity $\nu_t(x,t)$ entering
(8)–(10). The Zeldovich–Frank-Kamenetskii balance of diffusion and reaction
then propagates the reaction front at

$$ S_T \sim \sqrt{\nu_t / \tau_{chem}}, \qquad \tau_{chem} = 1/(A_r e^{-T_a/T_f}) \tag{12} $$

which is grid-independent provided the front thickness
$\delta \sim \sqrt{\nu_t \tau_{chem}}$ exceeds the cell size (satisfied by
the reference design at production resolution — $\delta \sim 5{-}10$ mm vs
$\Delta x = 3$ mm — and corroborated by the grid-convergence study).

**Turbulent kinetic energy budget (chamber zone).** Let $k_c$ be the
turbulent kinetic energy per unit mass of gas in the chamber zone (0D — one
scalar; A10: chamber turbulence treated as spatially uniform within the
chamber/cone zone, because the driving jets fill the chamber in a transit
time short compared to the cycle). Its budget follows from the kinetic
energy theorem:

- **Jet production**: the valve jets enter at speed $u_j$ into near-stagnant
  gas; a momentum balance on the sudden expansion (Borda–Carnot, derived in
  §7) shows the jet's excess kinetic energy relative to the mixed-out state
  is lost from the mean flow — it becomes turbulence:
  $\dot P_K = \tfrac12 \dot m_v (u_j - u_c)^2$ with $u_c$ the local mean.
- **Shear production**: the standard k-equation production term — the work
  of the Reynolds stress against the resolved mean shear,
  $\rho\,\nu_t (\partial u/\partial x)^2$ per unit volume, integrated over
  the chamber zone. This is what sustains mixing through the blowdown after
  the valve closes.
- **Dissipation**: the Richardson–Kolmogorov cascade argument (dimensional
  analysis: eddies of size $\ell_m$ and velocity $\sqrt{k_c}$ turn over and
  hand energy down in one eddy time) gives
  $\varepsilon = C_\varepsilon k_c^{3/2} / \ell_m$ per unit mass.

$$ \frac{d(m_{cz} k_c)}{dt} = \tfrac12 \dot m_v (u_j - u_c)^2 + \int_{cz} \rho\, \nu_t \left(\frac{\partial u}{\partial x}\right)^{\!2} A\, dx - m_{cz}\, C_\varepsilon \frac{k_c^{3/2}}{\ell_m} - k_c\,\dot m_{out} \tag{13} $$

($m_{cz}$ = gas mass in the chamber zone; last term = turbulence advected out
with outflow. The implementation advances the *intensive* form obtained by
expanding the left side with the zone mass balance: the outflow terms cancel
— leaving mass carries $k$ at concentration $k_c$ — and what survives is
dilution by the incoming jet mass, $\dot k_c = P/m_{cz} - \varepsilon -
k_c \dot m_v / m_{cz}$.) The eddy viscosity, from the mixing-length argument
($\nu_t \sim u' \ell$, Prandtl):

$$ \nu_{t,cz} = C_\nu \sqrt{k_c}\, \ell_m, \qquad u' = \sqrt{2k_c/3} \tag{14} $$

with $\ell_m = 0.35 D_c$ (the chamber's dominant toroidal recirculation eddy
scales with chamber diameter), $C_\nu = 0.5$, $C_\varepsilon = 0.5$ (A11:
O(1) closure constants — dimensional analysis fixes the *form*; the constants
are stated inputs, fixed once, not per-case tuning knobs). In the tailpipe
zone the same mixing-length argument with wall-generated turbulence gives
$\nu_{t,pipe} = C_t |u| D(x)$, $C_t = 0.02$ (A11). A small molecular floor
$\nu_{min}=2\times10^{-5}\ \mathrm{m^2 s^{-1}}$ is added. $\nu_t(x)$ blends
smoothly between zones over the cone.

Note the energy bookkeeping is consistent and not double-counted: in the FV
momentum equation the jet's excess mean KE is *already* converted to internal
energy at the mixing cell (numerical Borda–Carnot, §7); $k_c$ is a parallel
scalar used **only** to set $\nu_t$, never added to (9). (A12: the transient
storage of turbulence energy — ~1% of thermal energy — is neglected in the
energy equation.)

### 5b. Strain extinction at the intake jet (flame blow-off)

A cell-averaged 1D model artificially lets a flame anchor *inside* the
intake jet: the cell mean velocity is small even though the physical jet
crosses the valve gap at 100–300 m s⁻¹. Real premixed flames extinguish
under strain: activation-energy asymptotics (Zeldovich–Frank-Kamenetskii)
shows the reaction zone occupies only $1/Ze$ of the flame thickness, where

$$ Ze = \frac{T_a (T_{ad} - T_u)}{T_{ad}^2} \tag{12b} $$

is the Zeldovich number, so the flame's effective response time is
$Ze^2\,\tau_{chem}$, and the flame is extinguished where the imposed strain
rate $s$ satisfies $s\,Ze^2 \tau_{chem} \gtrsim 1$ — the classic
extinction-strain criterion, here *derived* from the reaction-zone scaling
rather than taken from burner data. (For propane-air this predicts
extinction strains of a few $10^3\ \mathrm{s^{-1}}$, consistent with
reality; the intake jet's shear-layer strain $u_j/\xi \sim 10^5\ \mathrm{s^{-1}}$
is far beyond it, which is *why* real valved engines do not hold a
continuous flame at the valve grid.)

The model therefore multiplies the kinetic rate by the smooth Damköhler
switch

$$ F_q(x) = \frac{1}{1 + \left(s(x)\, Ze^2\, \tau_{chem}(T)\right)^2}, \qquad s(x) = \frac{u_j}{\xi}\, e^{-x/L_{jet}} \tag{12c} $$

active only while the valve admits flow. $L_{jet} = 15\,\xi$ is the jet
penetration/decay length (self-similar jet momentum-integral scaling: a slot
jet's excess velocity decays over 10–20 slot heights) (A23: the strain field
of the 3D valve jets is represented by its gap-shear magnitude decaying over
the self-similar penetration length; justification: momentum-integral
entrainment analysis of free jets; consequence: the ignition-onset timing
inherits ~±30% uncertainty in $L_{jet}$, but the *mechanism* — charge
accumulates unburned while intake is fast, burns when intake slows — is
strain physics, not a fitted delay).

**Damköhler mixing limit (eq. 12d).** At high Damköhler number the burn
rate saturates at the turbulent mixing rate — reactants cannot be consumed
faster than eddies deliver them to reaction surfaces (Damköhler's classic
limit; the eddy-turnover argument). The chamber-zone kinetic rate is
therefore capped:

$$ k_{eff} = \min\!\left(A_r e^{-T_a/T},\; C_{EBU}\frac{\sqrt{k_c}}{\ell_m}\right) \tag{12d} $$

(A28: $C_{EBU} = 4$, an O(1) eddy-breakup constant fixed once, same status
as A11.) This is what makes burn *duration* scale with engine size — the
mixing rate goes as eddy turnover $\sqrt{k_c}/\ell_m \propto 1/s$ while the
Arrhenius rate is scale-absolute. Without it, a geometrically-scaled-up
engine burns its (proportionally larger) charge in the same absolute time:
an impulsive, near-constant-volume bang that drives every acoustic mode
broadband instead of feeding the fundamental — observed directly as the
failure mode of a 2.84×-scale engine before this term was added. At the
FP-1 reference scale the cap only grazes the hottest instants
(kinetics-limited regime), which is why the small-engine validation was
insensitive to its absence.

---

## 6. Valve structural dynamics from beam theory

Each of the $N_p$ valve petals is a thin rectangular spring-steel cantilever:
length $L_v$, width $w_v$, thickness $h_v$, Young's modulus $E_s$, density
$\rho_s$, clamped at the root, covering a port of area $A_{port}$.

**Stiffness (Euler–Bernoulli statics).** For a tip load $P$, bending moment
$M(x) = P(L_v - x)$; integrating $E_s I\, y'' = M$ twice with clamped-root
conditions ($y(0)=y'(0)=0$, $I = w_v h_v^3/12$) gives the classic tip
deflection $y(L_v) = P L_v^3 / 3 E_s I$, i.e. modal (tip) stiffness

$$ k = \frac{3 E_s I}{L_v^3} = \frac{E_s w_v h_v^3}{4 L_v^3} \tag{15} $$

and static shape $\psi(\xi_*) = \tfrac12(3\xi_*^2 - \xi_*^3)$, $\xi_* = x/L_v$,
normalized to $\psi(1)=1$.

**Effective mass (Rayleigh energy method).** Kinetic energy of the beam moving
in shape $\psi$ with tip velocity $\dot\xi$:
$T = \tfrac12 \rho_s w_v h_v L_v \dot\xi^2 \int_0^1 \psi^2 d\xi_* $, and
$\int_0^1 \psi^2 d\xi_* = 33/140$, so

$$ m_{eff} = \frac{33}{140}\, \rho_s w_v h_v L_v \tag{16} $$

**Generalized pressure force.** A uniform pressure differential $\Delta P$
over the petal face does virtual work
$\delta W = \Delta P\, w_v L_v \int_0^1 \psi\, d\xi_* \,\delta\xi
= \tfrac38 \Delta P A_{petal}\, \delta\xi$ (since $\int_0^1 \psi\,d\xi_* = 3/8$),
giving generalized force $Q_p = \tfrac38 \Delta P A_{petal}$. Consistency
check: static tip deflection $Q_p/k = \Delta P\, w_v L_v^4/8E_sI$, exactly the
uniform-load cantilever result — the shape assumption is self-consistent.

**Face pressures.** The upstream face sees the intake plenum; the downstream
face sees the chamber-side gas at the head, pressure $p_1$. When the petal is
open, flow accelerates over the face toward the gap; along a streamline the
mechanical-energy (Bernoulli) integral of the Euler equation reduces the face
pressure by the local dynamic pressure near the free edges. The face-averaged
differential is modeled (A13) as

$$ \Delta P = \left(p_{0,up} - \beta_f\, \tfrac12 \rho_j u_j^2\right) - p_1, \qquad \beta_f = A_v / (N_p A_{port}) \in [0,1] \tag{17} $$

— i.e. the Bernoulli reduction acts over the open-area fraction of the face.
(A13 justification: the 3D face-pressure distribution cannot be resolved in
1D; the two limits are exact — fully closed $\beta_f = 0$ recovers stagnation
loading, wide open the face rides at gap static pressure — and Bernoulli
fixes the magnitude of the reduction.)

**Aerodynamic drag on the moving petal.** A flat plate moving at velocity
$\dot\xi$ normal to itself stagnates the flow it runs into; the momentum
theorem bounds the resulting face-pressure excess by the full momentum flux
$\rho |\dot\xi| \dot\xi$. Taking the bound as the model (A14 — coefficient
exactly 1 by the momentum-flux argument):

$$ Q_d = -\tfrac38 A_{petal}\; \rho_h\, |\dot\xi|\, \dot\xi \tag{18} $$

with $\rho_h$ the gas density at the head. Structural damping (internal
friction in the steel and at the clamp) cannot be derived from continuum
elasticity; it is a measured material behavior, modeled as linear viscous
with damping ratio $\zeta$ (A15, default 0.03):
$c = 2\zeta\sqrt{k\, m_{eff}}$.

**Equation of motion** (Newton's second law in the modal coordinate $\xi$ =
tip lift):

$$ m_{eff} \ddot\xi = \tfrac38 A_{petal}\, \Delta P - k \xi - c \dot\xi + Q_d \tag{19} $$

**Seat preload.** Petals are manufactured with residual curvature and sit
pressed flat against the seat, storing deflection $\xi_0$ in the spring
(standard reed-valve practice). The spring term in (19) is therefore
$k(\xi + \xi_0)$, and the valve cracks open only when the pressure force
exceeds $k\xi_0$ — a cracking pressure
$\Delta P_{crack} = k\xi_0/(\tfrac38 A_{petal})$ that follows from the same
beam mechanics as $k$ itself (A25: the preload deflection is a design
input; its restoring force is exactly the derived cantilever stiffness).
This is what lets a reed valve stay sealed under a steady ram-pressure bias
in forward flight while still opening on the suction stroke.

**Contact constraints.** The petal is confined to $0 \le \xi \le \xi_{max}$
(seat and mechanical stop). Impacts conserve momentum and dissipate energy in
plastic deformation of the contact region; the fraction retained is the
restitution coefficient $e_r$ (A16, default 0.3 — thin steel on steel with a
compliant seat). On penetration the state is projected back:
$\xi \to$ boundary, $\dot\xi \to -e_r \dot\xi$ if moving into the boundary.

**Valve open area from the deflected shape** — geometric, no discharge
coefficient. Flow escapes through the curtain formed along the petal's free
edges: the tip edge (width $w_v$) opens a gap $\xi$; each side edge (length
$L_v$) opens the local deflection $\psi(x)\xi$. The curtain area is the
integral of gap along the free perimeter:

$$ A_{curt}(\xi) = \xi \left(w_v + 2 L_v \int_0^1 \psi\, d\xi_*\right) = \xi\left(w_v + \tfrac34 L_v\right) \tag{20} $$

The flow's true throat is the smaller of curtain and port:
$A_v(\xi) = N_p \min\left(A_{curt}(\xi), A_{port}\right)$. (A17: the vena
contracta — further contraction of the free jet past the geometric throat —
is neglected; the throat area is purely geometric, per the model boundary
conditions. Consequence: valve mass flow is biased high by roughly 10–30%;
this is a stated fidelity limit, not a tunable.)

---

## 7. Valve gap flow: compressible orifice relations from energy conservation

The gap is centimeters long; transit time ($\sim$ mm / 300 m s⁻¹ $\sim$ 3 µs)
is tiny compared to the cycle (~7 ms), so the gap flow is quasi-steady (A18:
quasi-steady valve-passage flow; justification: timescale separation ≫ 100×).
Steady adiabatic flow with no work between the upstream reservoir
(stagnation state $p_0, T_0$) and the throat conserves total enthalpy:

$$ c_p T_0 = c_p T_t + \tfrac12 u_t^2 \;\Rightarrow\; u_t = \sqrt{2 c_p (T_0 - T_t)} \tag{21} $$

The acceleration up to the throat is smooth and nearly loss-free, hence
isentropic (Gibbs relation with $ds=0$):
$T_t/T_0 = (p_t/p_0)^{(\gamma-1)/\gamma}$. Substituting into $G = \rho_t u_t$
with the ideal-gas law gives the mass flux through the throat as a function
of pressure ratio $r = p_t/p_0$:

$$ G(r) = \frac{p_0}{\sqrt{R T_0}} \sqrt{\frac{2\gamma}{\gamma - 1}\left(r^{2/\gamma} - r^{(\gamma+1)/\gamma}\right)} \tag{22} $$

**Choking is derived, not imposed**: $dG/dr = 0$ yields the critical ratio

$$ r^* = \left(\frac{2}{\gamma+1}\right)^{\gamma/(\gamma-1)} \tag{23} $$

If the downstream pressure $p_d < r^* p_0$ the throat cannot respond
(downstream states communicate at most at sonic speed) and $G = G(r^*)$;
otherwise $G = G(\max(p_d/p_0, r^*))$. Mass flow: $\dot m_v = A_v G$, jet
velocity $u_j = G / \rho_t$.

**Jet dump and Borda–Carnot.** The jet (area $A_v$, velocity $u_j$) enters
the chamber (area $A_c \gg A_v$). Writing mass and momentum conservation on
the sudden expansion shows the mixed-out state loses stagnation pressure and
converts the jet's excess KE to heat — the classic Borda–Carnot loss. In this
model that balance is performed *automatically* by the finite-volume update:
the boundary flux injects the jet's mass, momentum ($\dot m_v u_j$), and
total enthalpy into the first cell, and the cell's momentum/energy balance
mixes it out exactly as the control-volume analysis dictates. No separate
loss coefficient exists or is needed.

**Direction handling.** If $p_1 > p_{0,up}$ while the valve is ajar, the same
relations apply with the chamber-side stagnation state upstream — the model
permits physical back-spitting through a closing valve.

---

## 8. Boundary conditions

**Head plane ($x=0$).** The FV boundary flux on the first face, per unit
face area $A(0)$ (total-flux form $[\dot m, \dot m u_j + p_1 A(0), \dot m h_0, \dot m Y]$):

- Valve closed ($A_v = 0$): wall — $[0,\; p_w A(0),\; 0,\; 0]$ with $p_w$
  the exact mirror-state HLLC star pressure (§12), so reflection includes
  the acoustic $\pm\rho a u$ correction a rigid plate produces.
- Valve open, inflow: $[\dot m_v,\; \dot m_v u_j + p_w A(0),\; \dot m_v h_{0,up},\; \dot m_v \cdot 1]$
  — the incoming charge is reactant ($Y=1$) premixed at $\phi$ (A19: perfect
  carburetion of fuel into intake air at the valve; justified: real valved
  engines aspirate atomized fuel at the valve head; mixture preparation time
  ≪ residence time).
- Valve open, outflow: same structure with chamber-side stagnation state and
  the local $Y_1$.

**Ram intake state.** In the vehicle frame the free stream arrives at speed
$u_\infty = M a_a$; adiabatic deceleration to the valve plenum conserves
total enthalpy: $T_{0,up} = T_a (1 + \frac{\gamma-1}{2} M^2)$, and with
isentropic external diffusion (A20: ideal ram recovery — loss-free external
compression; the *internal* dump loss is captured by §7's Borda–Carnot
mixing; consequence: mild optimism in $p_{0,up}$ at high subsonic M):
$p_{0,up} = p_a (1 + \frac{\gamma-1}{2} M^2)^{\gamma/(\gamma-1)}$.

**Intake column inertia (§6b).** The valve does not draw from an infinite
reservoir: it sits behind an intake duct (length $L_i$, area $A_i$) ending
in a small plenum (volume $V_{pl}$) at the valve face. Newton's second law
applied to the duct's air column (the classic inertance element — unsteady
Bernoulli integrated along the duct) and mass conservation + the isentropic
EOS applied to the plenum give two ODEs:

$$ \frac{d\dot m_i}{dt} = \frac{A_i}{L_i}\left(p_{0,up} - p_{pl} - \tfrac12 \rho_0 u_i |u_i| \right), \qquad \frac{dp_{pl}}{dt} = \frac{\gamma_R R_R T_{0,up}}{V_{pl}}\left(\dot m_i - \dot m_v\right) \tag{23b} $$

The $\tfrac12\rho_0 u_i|u_i|$ term is the Borda–Carnot loss where the duct
dumps into the larger plenum (derived from the momentum theorem on the
sudden expansion, like §7's); it is also what physically damps the
duct–plenum Helmholtz mode (~44 kHz). Numerically the pair is advanced with
the symplectic ordering (column first, then plenum) so the mode is stable
at the fluid time step, and $p_{pl}$ carries a wide numerical safety clamp
$[0.2, 5]\,p_a$ that no reported run has touched.

The valve's upstream state is then $(p_{pl}, T_{0,up})$ rather than the ram
stagnation state. This carries the two real intake effects a reservoir
cannot: **ram-through** (the accelerated column keeps feeding after chamber
pressure recovers, extending the effective intake) and the **closing-hammer
spike** ($p_{pl}$ overshoots $p_{0,up}$ when the valve shuts against a
moving column, supercharging the next opening). (A24: the column is treated
as a frictionless incompressible slug with a loss-free bellmouth entry, the
plenum as adiabatic at $T_{0,up}$, and back-spit contamination of the plenum
is neglected; justification: the duct is short — its own acoustic timescale
$L_i/a \sim 0.2$ ms is marginal but below the cycle scale — and bellmouth
entries are near-loss-free by design.)

**Side-mounted (boundary-layer) intake (§8c).** When the valve inlets face
*perpendicular* to the flight velocity and swallow boundary-layer air off
the vehicle skin, the forward-flight feed changes in three derivable ways:

1. **Pressure**: the inlet sees the wall static pressure of the external
   flow, $p_{0,up} = p_a$ — no ram recovery (the wall-normal momentum
   equation across a thin boundary layer gives $\partial p/\partial y
   \approx 0$, so the wall rides at the edge static pressure).
2. **Temperature**: boundary-layer air is viscously heated. The
   Crocco–Busemann energy integral (an exact consequence of the boundary
   layer equations at $Pr = 1$) gives full recovery to $T_0$ at the wall;
   real air ($Pr = 0.71$, turbulent) recovers a fraction $r$:
   $T_{0,up} = T_a\!\left(1 + r\,\tfrac{\gamma-1}{2}M^2\right)$ (A26:
   $r = 0.9$, bounded above by the derived $Pr = 1$ limit $r = 1$). The
   charge density therefore *falls* with $M^2$ even though pressure holds.
3. **Momentum drag**: the swallowed stream arrives already
   momentum-depleted — the vehicle paid for that momentum loss through
   skin friction upstream, so charging the engine the full
   $\dot m_v u_\infty$ would double-count it. For a 1/7-power turbulent
   profile the mass-flow-weighted momentum of the *whole* layer is
   $\tfrac{8}{9}u_\infty$, and less for partial capture; the drag term
   becomes $\dot m_v\, k_{bl} u_\infty$ (A27: $k_{bl}$ is a design input,
   default 0.6, derived bracket 0.4–0.9 by capture-height-to-thickness
   ratio).

A major systems consequence follows directly: with $p_{0,up} = p_a$ the
plenum never develops the steady ram bias that holds petals off their seats
at speed — the §8b forward-inlet quench mechanism is absent, at the price
of forgoing ram supercharging and accepting recovery-heated (less dense)
charge.

**Exit plane ($x=L$).** Characteristics of the 1D Euler system (eigenvalues
$u-a$, $u$, $u+a$; Riemann invariants $J_\pm = u \pm \frac{2a}{\gamma-1}$
along $dx/dt = u \pm a$, entropy along $dx/dt = u$ — derived by diagonalizing
the primitive-variable system):

- **Subsonic outflow** (one incoming characteristic): impose the base
  pressure $p_{ghost} = p_a$ (A21: exit discharges into the base region at
  ambient static pressure), carry $\rho, u, Y$ out along the outgoing
  characteristics (extrapolated).
- **Supersonic outflow**: no incoming characteristics — full extrapolation.
- **Backflow** (suction phase, two incoming characteristics): ambient air
  re-enters from the base region, quiescent in the vehicle frame (A22: the
  near-wake/base air travels with the vehicle; its stagnation state is
  ambient static $p_a, T_a$). The ghost state accelerates isentropically
  from that reservoir to the interior pressure; entering gas is burned-out
  ambient air, $Y = 0$.

---

## 9. Thrust from the momentum theorem

Control volume: the engine's wetted exterior + intake capture streamtube +
exit plane, in the vehicle frame. A uniform ambient pressure field exerts
zero net force on any closed surface, so only *gauge* pressures and momentum
fluxes contribute. The instantaneous axial force on the engine is

$$ F(t) = \underbrace{\dot m_e u_e + (p_e - p_a) A_e}_{\text{exit plane, interior state}} \;-\; \underbrace{\dot m_v\, u_\infty}_{\text{ram drag of captured stream}} \tag{24} $$

where $\dot m_e = (\rho u A)_e$ (sign carries the suction-phase backflow
automatically). External aerodynamic drag of the cowl is deliberately
excluded — this is engine thrust, not vehicle net force.

**Momentum-closure verification identity.** Integrating the discrete
momentum equation over the duct and one cycle (storage
$\frac{d}{dt}\int \rho u A\,dx$ averages to zero on a periodic cycle) and
substituting the head boundary flux $|\dot m_v| u_j + p_w A(0)$ (§8, with
$p_w$ the mirror-Riemann wall-star pressure the scheme itself uses) gives
the exact identity the implementation checks:

$$ \overline{F}_{(24)} = \overline{(p_w - p_a) A(0)} + \overline{\int_0^L (p - p_a) \frac{dA}{dx} dx} + \overline{|\dot m_v|\, u_j} - \overline{\dot m_v u_\infty} \tag{25} $$

The area integral is *positive-signed*; for this geometry $dA/dx \le 0$ in
the cone with $p > p_a$, so the term is negative — the rearward pull of the
contracting cone enters through the sign of $dA/dx$. The right side is
recorded as $F_{surf}(t)$; its cycle-averaged agreement with (24) verifies
the flux/source/storage bookkeeping of the whole discretization (boundary
fluxes, the $p\,dA/dx$ source, and the cycle closure), and its
instantaneous difference is the momentum storage. (During back-spit windows
the recorded (24)/(25) both clip the ram term to inflow only,
$\max(\dot m_v, 0)\,u_\infty$ — spit mass re-swallowed by the intake is not
charged ram drag twice.)

**Cycle-averaged thrust** — the primary query output:

$$ \bar F = \frac{1}{n T_c} \int_{t_s}^{t_s + n T_c} F(t)\, dt \tag{26} $$

over the last $n$ complete cycles of the converged limit cycle (§12).

---

## 10. Rayleigh criterion, derived

Linearize (7)–(9) about the cycle-mean state ($\bar p(x)$, mean flow
neglected for the acoustic balance): $p = \bar p + p'$, etc. Standard
manipulation — multiply the linearized momentum equation by $u'$, the
linearized pressure equation
$\partial_t p' + \gamma \bar p \nabla\!\cdot u' = (\gamma - 1) \dot q'$ by
$p'/\gamma \bar p$, and add — yields the acoustic energy balance

$$ \frac{\partial}{\partial t}\underbrace{\left[\frac{p'^2}{2\gamma \bar p} + \frac{\bar\rho u'^2}{2}\right]}_{E_{ac}} + \nabla\!\cdot(p' u') = \frac{\gamma - 1}{\gamma \bar p}\, p'\, \dot q' \tag{27} $$

with $\dot q' = (\rho q_R \dot\omega)'$ the heat-release-rate fluctuation.
Integrating over the duct volume and one period: boundary flux terms are
losses (radiation from the exit), and the oscillation grows if and only if

$$ \mathcal{R} = \oint\!\!\int_0^L \frac{\gamma-1}{\gamma \bar p}\, p'\, \dot q'\; A\, dx\, dt \;>\; \text{(cycle losses)} \tag{28} $$

— heat release in phase with pressure feeds the wave (Rayleigh's criterion,
here *derived* as the source term of the acoustic energy equation). The
implementation reports an unweighted *proxy* of (28): the per-cycle
covariance of head pressure with globally-integrated heat release (the
chamber is the fundamental mode's pressure antinode, so head $p'$ stands in
for the modal amplitude; the $(\gamma{-}1)/\gamma\bar p$ weight and spatial
integral are not assembled). Its sign — drive vs damp — is the diagnostic. In this
model the phasing is emergent: the returning tailpipe compression wave
raises $T$, the Arrhenius rate (11) spikes, and heat release lands on the
pressure crest — or fails to, in which case the design decays, which is
itself a physical prediction.

---

## 11. Variable-volume chamber cell

The requirement's "variable-volume chamber" is honored exactly: petal
deflection sweeps volume $\Delta V(\xi) = \tfrac38 N_p A_{petal}\, \xi$ (the
mode-shape integral again) out of the first cell. For a control volume with
one moving wall, the Reynolds transport theorem adds a moving-boundary work
term to the energy balance and a volume rate to the stored quantities:

$$ V_1(t) = V_1^{geo} - \tfrac38 N_p A_{petal}\, \xi(t), \qquad \frac{dE_1^{tot}}{dt}\Big|_{wall} = -\,p_1 \frac{dV_1}{dt} \tag{29} $$

Mass and species totals are unaffected by wall motion; density and pressure
respond through the reduced volume. The term is $O(10^{-2})$ of $V_1$ but is
carried exactly (cell 1 stores conserved *totals* with its own $V_1(t)$).

---

## 12. Discretization and stability constraints

**Finite-volume form.** Integrate (7)–(10) over cell $i$ of width
$\Delta x$: state $\mathbf U_i = (\rho A, \rho u A, \rho E A, \rho Y A)_i$,

$$ \frac{d\mathbf U_i}{dt} = -\frac{\mathbf F_{i+1/2} - \mathbf F_{i-1/2}}{\Delta x} + \mathbf S_i \tag{30} $$

$\mathbf S_i$ = area-pressure source $p\,dA/dx$ (momentum), reaction terms,
and diffusion (central-differenced).

**Interface flux: HLLC approximate Riemann solver, derived.** Integral
conservation over the Riemann fan with a three-wave model (left/right
acoustic waves at speeds $S_L, S_R$, contact at $S_*$) gives the
Rankine–Hugoniot conditions across each wave; solving them for the two star
states yields

$$ S_* = \frac{p_R - p_L + \rho_L u_L (S_L - u_L) - \rho_R u_R (S_R - u_R)}{\rho_L (S_L - u_L) - \rho_R (S_R - u_R)} \tag{31} $$

$$ \mathbf U_{*K} = \rho_K \frac{S_K - u_K}{S_K - S_*} \left[1,\; S_*,\; E_K + (S_* - u_K)\left(S_* + \frac{p_K}{\rho_K (S_K - u_K)}\right),\; Y_K \right]^T \tag{32} $$

with wave-speed bounds $S_L = \min(u_L - a_L,\, u_R - a_R)$,
$S_R = \max(u_L + a_L,\, u_R + a_R)$ (Davis bounds — provably outer bounds
for the exact fan, so the scheme is positivity-friendly). The flux is
$\mathbf F = \mathbf F_K + S_K (\mathbf U_{*K} - \mathbf U_K)$ for the region
containing the interface. HLLC preserves the contact wave exactly — essential
here, because the fresh-charge/residual-gas interface *is* a contact, and its
sharpness controls ignition phasing.

**Second order: MUSCL reconstruction.** Piecewise-linear primitive states
$(\rho, u, p, Y)$ with minmod-limited slopes
$\sigma_i = \mathrm{minmod}(w_i - w_{i-1}, w_{i+1} - w_i)/\Delta x$;
interface values $w_{i+1/2}^- = w_i + \tfrac12 \sigma_i \Delta x$ etc. Minmod
makes the reconstruction total-variation-diminishing, preventing spurious
oscillations at the shock fronts the engine generates every cycle.

**Time integration: SSP-RK2 (Heun).**
$\mathbf U^{(1)} = \mathbf U^n + \Delta t\, \mathcal L(\mathbf U^n)$;
$\mathbf U^{n+1} = \tfrac12 \mathbf U^n + \tfrac12\left[\mathbf U^{(1)} + \Delta t \mathcal L(\mathbf U^{(1)})\right]$.
As a convex combination of forward-Euler steps it inherits the TVD property
under the Euler CFL limit.

**Time-step constraints** (all enforced every step; $\Delta t$ = the minimum):

1. *Convective (CFL)*: information travels one cell per step at most —
   $\Delta t \le C_{CFL}\, \min_i \Delta x / (|u_i| + a_i)$, $C_{CFL} = 0.4$.
2. *Diffusive*: explicit central diffusion is stable for
   $\Delta t \le \tfrac12 \min_i \Delta x^2 / \nu_{t,i}$ (von Neumann
   analysis of the heat equation).
3. *Chemistry*: the reaction sub-step uses the exact exponential solution of
   $dY/dt = -kY$ at frozen $T$, with internal sub-cycling whenever
   $k \Delta t > 0.2$ so the released heat and the rate stay consistent
   (temperature re-evaluated each sub-step). This removes stiffness without
   an implicit solver.
4. *Valve*: resolve the petal's natural period —
   $\Delta t \le 2\pi/(20\,\omega_n)$, $\omega_n = \sqrt{k/m_{eff}}$
   (in practice constraint 1 is ~20× stricter).

**Operator ordering per step**: transport (hyperbolic + diffusion, RK2) →
reaction sub-step → valve ODE update (semi-implicit *symplectic Euler* on
(19) at the fluid $\Delta t$ — velocity from acceleration, then position
from the new velocity — which is ~20× finer than the petal period, and
symplectic so the contact-free oscillation neither gains nor loses energy) →
contact projection → chamber-volume work (29, distributed over the petal
zone so the area correction is grid-independent) → turbulence budget (13).

---

## 13. The query loop

`pulsejet_thrust(design, flight, params) → ThrustResult`

Inputs: geometry (chamber/cone/tailpipe dimensions), valve pack ($N_p$,
petal dimensions, $E_s$, $\rho_s$, $\zeta$, $e_r$, $\xi_{max}$, port size),
fuel/thermochemistry ($\Delta h_c$, $f_{st}$, $\phi$, $E_a$, $A_r$, species
$W$, $c_v$), flight state ($M$, altitude), numerics ($N$, $C_{CFL}$,
durations). Output: cycle-averaged thrust (primary), plus frequency, air and
fuel mass flows, TSFC, $p_{min}/p_{max}$ at the head, valve-lift statistics,
Rayleigh index, both thrust estimates (24)/(25), limit-cycle convergence
flag, and (optionally) full time series.

Algorithm:

1. **Ambient**: (6) → $p_a, T_a$; ram state → $p_{0,up}, T_{0,up}$ (§8).
2. **Initialization**: whole duct at ambient; chamber zone filled with hot
   combustion products ($Y=0$, $T = 1900$ K, $p = 1.4\,p_a$) — the state an
   instant after the starting fire; valve seated ($\xi = 0$, $\dot\xi = 0$).
   The first blowdown, suction, refill, and ignition then follow from the
   equations alone.
3. **March** (30) with the step ordering of §12, recording head pressure,
   valve state, exit flux, thrust integrands, and the Rayleigh integrand.
4. **Cycle detection**: a cycle boundary is a positive-going crossing of the
   head pressure through its trailing mean. Per cycle, compute impulse,
   frequency, swallowed mass, burned fuel, $\mathcal R$.
5. **Limit-cycle test**: converged when the last $n=6$ cycle periods vary
   < 2% (relative RMS) and cycle-mean thrusts vary by less than
   $\max(5\%\,|\bar F|,\ 0.75\ \mathrm{N})$; diverged/quenched flags otherwise
   (quench = no reignition: $\max_x T$ falls below 900 K and stays there;
   the model reports failure to self-sustain as a *result*, not an error).
6. **Outputs**: (26) over the converged window, plus diagnostics; optional
   full traces for plotting.

A thrust-vs-Mach sweep is a loop over `flight.M` reusing the same converged
numerical settings, run in parallel worker processes.

---

## 14. Assumptions registry

| # | Assumption | Fundamental-law justification | Known consequence |
|---|---|---|---|
| A1 | Two pseudo-species (reactant/product) | Fixed premixed stoichiometry ⇒ one progress variable spans composition space | No local φ variation |
| A2 | Calorically perfect species, effective vibrational excitation | Equipartition theorem; fixed activation fraction over 800–2600 K | ±3–5% in $T$ |
| A3 | No product dissociation | Endothermic equilibria negligible < 2400 K cycle mean | Flame T high ~5–8% |
| A4 | Quasi-1D | $D/L \ll 1$, transverse acoustic equilibration ≫ faster | No radial modes |
| A5 | Inviscid walls | Friction impulse < ~3% of pressure impulse (scale analysis) | $u_e$ slightly high |
| A6 | Adiabatic walls | Wall heat loss omitted (a defensible convective estimate needs an empirical Nusselt correlation, excluded by the clean-room rules) | **Significant** at this size class: real losses are ~10–30% of heat release, so the model runs hot and its thrust/TSFC are optimistic by a corresponding margin |
| A7 | $Pr_t = Sc_t = 1$ | Reynolds analogy — common eddy transport mechanism | — |
| A8 | Single-step, 1st-order kinetics | Law of mass action on a single progress variable | No multi-stage ignition chemistry |
| A9 | Fixed $A_r$ (one calibration) | One-step surrogate can't match all regimes; $E_a$ held at physical value | Absolute delay times approximate |
| A10 | 0D chamber turbulence scalar | Jet transit ≪ cycle time ⇒ chamber-filling | No spatial $k$ gradient in zone |
| A11 | O(1) turbulence closure constants | Dimensional analysis (Prandtl mixing length, Kolmogorov cascade) fixes forms | Quantitative burn rate ±factor ~2 |
| A12 | Turbulence energy not in (9) | $k_c \sim 1\%$ of thermal energy | — |
| A13 | Face-pressure interpolation (17) | Exact in both limits; Bernoulli fixes magnitude | Valve force model ±20% mid-lift |
| A14 | Plate drag = momentum-flux bound | Momentum theorem upper bound taken as model | Slight over-damping of petal |
| A15 | Linear structural damping $\zeta$ | Internal friction not derivable from continuum elasticity; measured behavior | $\zeta$ is an input |
| A16 | Restitution $e_r$ | Momentum conservation + inelastic contact energy loss | $e_r$ is an input |
| A17 | No vena contracta | Geometric throat only (model boundary condition — no empirical $C_d$) | Valve flow high 10–30% |
| A18 | Quasi-steady gap flow | Transit time ≪ cycle by >100× | — |
| A19 | Perfect carburetion | Atomization/mixing ≪ residence time | No droplet lag |
| A20 | Ideal ram recovery | Loss-free external compression; internal dump loss captured by Borda–Carnot | Optimistic at high M |
| A21 | Base pressure = $p_a$ | Exit discharges to near-wake at ambient static | ±few % on pressure thrust |
| A22 | Backflow from quiescent base region | Near-wake air convects with vehicle | Backflow enthalpy approximate |
| A23 | Jet strain field $s = (u_j/\xi)e^{-x/L_{jet}}$, $L_{jet}=15\xi$ | Self-similar free-jet momentum-integral decay; extinction criterion from ZFK asymptotics (eq. 12b-c) | Ignition-onset timing ±30% |
| A24 | Intake column: frictionless slug + adiabatic plenum (eq. 23b) | Newton's law on the duct column (inertance); isentropic plenum compliance; bellmouth ~loss-free | Duct acoustics unresolved (~0.2 ms) |
| A25 | Seat preload $\xi_0$ (spring term $k(\xi+\xi_0)$) | Residual-curvature preload restored by the derived cantilever stiffness; standard reed-valve cracking mechanism | $\xi_0$ is a design input |
| A26 | Side inlet: recovery factor $r = 0.9$ | Crocco–Busemann energy integral gives $r = 1$ at $Pr = 1$; real turbulent air recovers slightly less | Intake density vs M approximate |
| A28 | Damköhler mixing cap $C_{EBU}=4$ (eq. 12d) | Eddy-turnover delivery limit on consumption rate (Damköhler/eddy-breakup argument) | Burn duration at large scale ± the O(1) constant |
| A27 | Side inlet: ingested-momentum fraction $k_{bl} = 0.6$ | 1/7-power-law boundary-layer momentum integral brackets 0.4–0.9 by capture height | Ram-drag charge ±30% at high M |

Every other relation in this document — the field equations, valve beam
mechanics, orifice/choking relations, HLLC construction, Rayleigh balance,
thrust theorem, atmosphere — is derived directly from conservation of mass,
momentum, and energy, the ideal-gas kinetic theory, the equipartition
theorem, the law of mass action, Euler–Bernoulli beam theory, and the
momentum theorem, with no empirical charts, discharge coefficients, or
pre-set delay times anywhere in the model.
