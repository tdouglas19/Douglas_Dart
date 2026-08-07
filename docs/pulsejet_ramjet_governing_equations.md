# Governing Equations & Switchable Combined Design — Pulsejet/Ramjet Sizing Reference

Covers (1) everything found on the switchable pulsejet↔ramjet design, and (2) the standard governing equations for sizing each subsystem (intake/inlet, combustion chamber, nozzle, resonance tube) and computing overall thrust, specific impulse, and cycle performance for both engine types.

## TL;DR
- The **switchable family** (Collins 1954, Winter/McDonnell 1956, Ghougasian 1969–71) is a single duct that operates as a valved/valveless pulsejet at low speed and mechanically reconfigures into a ramjet at high speed — via a retractable/rotating check valve (Ghougasian) or a drum-type inlet valve (Winter). No patent gives a closed-form transition law; the switch is geometric/mechanical, not aerodynamically automatic.
- **Pulsejet cycle physics** is governed by the **Humphrey cycle** (constant-volume heat addition) at flight speed, degrading toward the **Lenoir cycle** (heat addition at ~constant volume with no pre-compression) at zero/low flight speed or with side-inlet geometries — this is an experimentally-confirmed distinction, not just an assumption, and it matters for which cycle equations are valid in the low-speed "pulsejet mode" of a switchable engine.
- **Ramjet cycle physics** is the standard **Brayton cycle** with ram (dynamic) compression instead of a mechanical compressor; performance is fully determined by station-to-station stagnation pressure/temperature ratios, and is conventionally solved with the **stream-thrust (impulse) function** rather than a naive momentum-only thrust equation, because ramjet nozzles are very often not fully expanded.
- Below are the actual governing equations for both cycles, organized by subsystem (inlet/diffuser → combustor → nozzle → thrust/Isp), plus the resonance/frequency equations specific to pulsejet sizing that ramjet-only references omit.

## Part 1 — Switchable Combined Design: What We Know

### 1.1 Design lineage and mechanisms
| Patent | Inventor / Assignee | Year | Mechanism | Key detail |
|---|---|---|---|---|
| US2677232A | Whitney Collins | 1954 | Single duct, pulsejet→ramjet transition near sonic speed | Established the crossover concept: pulsejet superior below ~sonic, ramjet superior above it; no detailed valve geometry given |
| US2745248A | Winter, Kallal, Mazzoni / McDonnell Aircraft | 1956 | Drum-type/bulkhead inlet valve, pilot- or automatic-selectable | First to formalize active mode selection rather than passive transition |
| US3533239A | John N. Ghougasian | 1969/1970 | Retractable check valve | Power-operated positioning member moves the check valve into a housing-wall cavity for ramjet mode; a valve-orienting mechanism angularly aligns the valve to enter the cavity. Known drawback: external fairings needed to house the retracted valve increase supersonic drag. |
| US3604211A | John N. Ghougasian (CIP of above) | 1969/1971 | Axially shiftable center body + tubular check valve | Explicitly built to reach cruise in pulsejet mode then convert to ramjet operation by shifting an axial center body and tubular check-valve assembly — redesigned specifically to eliminate the external protuberances of the 3,533,239 mechanism |
| US4962641A | John N. Ghougasian | 1990 | Standalone valveless pulsejet (resonator-based) | Vaneless pulsejet using a resonator chamber to fill and discharge exhaust gas by oscillation, producing pressure pulses for closely-spaced combustion events; requires compressed air for starting like conventional pulsejets — usable as the low-speed-mode building block if a switchable design needs a valveless (not valved) pulsejet core |

### 1.2 Adjacent variable-geometry approach (relevant to ramjet-mode operation across a wide Mach range)
Two related patents address how a ramjet's own geometry should morph across a wide Mach range, directly relevant once the switchable engine is in ramjet mode and needs to cover more than a narrow Mach band:
- One variable-geometry ramjet patent morphs the transition area between combustion chamber and nozzle from convergent-then-divergent at low Mach to nearly constant-area-then-divergent at high Mach, using hinged plate walls in the throat region.
- A companion patent extends this to an extremely wide Mach 1–2 to Mach 15–20 range, with the nozzle throat effectively disappearing (constant cross-section combustor followed by divergent nozzle) above roughly Mach 8.

### 1.3 Net engineering assessment
The switchable approach trades **mechanical complexity and reliability risk in one moving valve/centerbody** (thermal cycling near the combustion zone, sealing, fatigue) against **fewer total flow-path components** than the nested/composite pulsejet-in-ramjet-nacelle family. No patent in this family provides quantitative valve actuation timing, pressure-loss coefficients, or a transition Mach-number control law — these would need to be engineered from first principles or from general valve/check-valve loss correlations.

---

## Part 2 — Governing Equations

### 2.1 Station numbering (standard ramjet/airbreathing convention)
- **Station 0**: freestream
- **Station 1**: inlet/diffuser entry (cowl lip)
- **Station 2**: diffuser exit / combustor entry
- **Station 3**: combustor exit (pre-nozzle) — sometimes labeled station 4 depending on reference
- **Station 4/5**: nozzle throat
- **Station 5/9/e**: nozzle exit

In a ramjet, the exhaust geometry and gas velocity depend on the interaction and balance between the pressure developed in the inlet and the pressure developed in the combustor. This inlet/combustor/nozzle pressure balance — not just a single momentum equation — is what actually determines mass flow and thrust; see §2.5.

### 2.2 Inlet / Diffuser Sizing (both engine types)

**Stagnation conditions and area-Mach relation** (compressible, isentropic, ideal gas):
```
T_t/T = 1 + [(γ-1)/2] M²
p_t/p = [1 + ((γ-1)/2) M²]^(γ/(γ-1))
A/A* = (1/M) · { [2/(γ+1)] · [1 + ((γ-1)/2)M²] }^[(γ+1)/(2(γ-1))]
```
These size the inlet capture area (A1), throat, and diffuser exit (A2) for a target Mach number and mass flow. This applies identically to ramjet and (during ramjet-mode) switchable-engine inlets.

**Diffuser total pressure recovery** — real diffusers lose stagnation pressure via shocks/friction:
```
π_d = p_t2 / p_t0     (total pressure recovery factor, π_d ≤ 1)
```
For supersonic inlets this is typically modeled via MIL-E-5008B-style empirical correlations or an oblique-shock-train calculation rather than assumed as a constant — a common oversimplification in early-stage sizing codes is to use a single flat π_d across the whole Mach range, when in reality π_d degrades sharply above ~Mach 3–5 depending on the number/angle of shocks in the inlet.

**Pulsejet intake nuance**: for the low-speed/pulsejet mode, at zero flight speed, valved pulsejets exhibit the Lenoir cycle (no pre-compression before heat release), while with increasing flight speed and a straight inlet, the cycle shifts toward the Humphrey cycle with a genuine polytropic pre-compression stage — meaning the "inlet" of a pulsejet is not a passive duct but an active determinant of which thermodynamic cycle applies, and this is speed-dependent. A side-mounted inlet, by contrast, shows little pre-compression regardless of flight speed and the indicator diagram changes only slightly with increasing speed — so inlet orientation (axial vs. side) is a first-order modeling choice, not a detail.

### 2.3 Ramjet Combustor Sizing & Cycle (Brayton Cycle)

The basic thermodynamic cycle of the ramjet is the Brayton cycle, and the pressure and temperature ratios have the decisive effect on ramjet performance.

**Combustor heat addition** (constant-pressure, i.e., Brayton):
```
f = (cp·(T_t3 - T_t2)) / (η_b · h_PR - cp·T_t3)     [fuel-air ratio, energy balance]
```
where η_b = combustion efficiency, h_PR = fuel heating value.

**Combustor total pressure loss** — real combustors lose stagnation pressure to friction and to the Rayleigh-flow effect of heat addition at finite Mach number:
```
π_b = p_t3 / p_t2     (typically 0.90–0.97 for well-designed ramjet combustors; must be modeled, not assumed = 1)
```

**Combustor sizing** (residence time / Damköhler-based sizing, first-order):
```
V_cc = ṁ_air · (1+f) / ρ_3 · t_res
```
where t_res is the required chemical residence time for the fuel/oxidizer pair at the local pressure and temperature — this is where a sizing code needs actual chemical kinetics data or empirical loading-parameter correlations (e.g., combustor loading parameter Θ), not just a guessed constant.

### 2.4 Pulsejet Combustor Cycle (Humphrey/Lenoir Cycle) and Resonance Sizing

The Humphrey cycle is the generic cycle for pulsejet combustion — a modified Brayton cycle in which the constant-pressure heat-addition (isobaric) phase is replaced by a constant-volume (isochoric) heat-addition phase.

**Humphrey cycle relations** (idealized, per unit mass):
```
Process 1→2: isentropic compression      p2/p1 = (V1/V2)^γ
Process 2→3: constant-volume heat addition   p3/p2 = T3/T2   (V constant)
Process 3→4: isentropic expansion        p4/p3 = (V3/V4)^γ
Thermal efficiency: η_Humphrey = 1 - γ·(τ^(1/γ) - 1)/(τ - 1),   τ = p3/p2 (pressure ratio at constant volume)
```
This gives the Humphrey cycle a theoretically higher thermal efficiency than an equal-pressure-ratio Brayton cycle — the entire motivating claim behind pulse-detonation/pulsejet-hybrid performance advantages found in the earlier research survey traces back to this equation.

**Which cycle actually applies is speed- and geometry-dependent, not fixed**: the real operating cycle of a valved pulsejet during static (zero-speed) conditions is the Lenoir cycle, not Humphrey; the Humphrey cycle only becomes the correct model as flight speed increases with a straight air intake. A sizing code that hard-codes "pulsejet = Humphrey cycle" across the entire low-speed operating envelope is applying the wrong cycle at static/near-static conditions — the correct condition-dependent model is Lenoir → Humphrey transition, not a fixed cycle choice.

**Resonance tube sizing** — pulsejets are fundamentally acoustic-thermodynamic coupled devices, and their operating frequency (and thus thrust pulsing/mean thrust) is set by tube geometry, not just combustor thermodynamics:
```
f_resonance ≈ a / (4·L_eff)     [quarter-wave organ-pipe approximation, one open/one closed end]
```
where `a` = local speed of sound in the hot gas, and L_eff = effective acoustic length of the resonance tube (combustion-chamber-center to tailpipe exit, with end corrections). The length from the combustion-chamber center to the edge of the resonance tube (L) is chosen as the characteristic sizing dimension, since it determines overall engine length and is important for operational stability, startup behavior, and thrust force. Note the literature explicitly flags unresolved debate over whether valved-pulsejet combustion behaves like a quarter-wave tube or a Helmholtz resonator — meaning the frequency equation above is one competing model, not a settled closed-form truth, and a sizing code should treat it as such (i.e., validate against test data rather than trust blindly).

**Mean thrust and operating frequency from cycle analysis** (semi-empirical, valveless case):
A physical/mathematical model for valveless pulsejets derives the relationship between mean discharge velocity and thermodynamic combustor parameters, then yields equations for operating frequency and mean thrust; results show non-dimensional mean discharge velocity increases (with diminishing returns) as compression ratio increases, with reported errors under 10% for frequency and ±17% for mean thrust versus experimental data — and the same thrust equation is stated to be applicable to valved pulsejets as well. This ±17% thrust uncertainty band is a useful reality check for what "good" pulsejet sizing-code accuracy currently looks like in the literature — a sizing tool claiming much tighter agreement without equivalent experimental validation should be treated with suspicion.

### 2.5 Nozzle Sizing & Overall Thrust Calculation

**Stream-thrust (impulse) function** — the standard and more robust way to compute ramjet internal thrust, superior to naive `ṁV` momentum bookkeeping when the nozzle is not perfectly expanded:
```
𝓕(x) = ṁ(x)·V(x) + p(x)·A(x)
```
The stream thrust at a given cross-section is a particularly useful quantity because the difference in stream thrust between two stations equals the axial thrust exerted on the duct walls between those stations; stream thrust can also be expressed as a function of mass flow, stagnation temperature, and Mach number. Mach number reaches unity at the combustor/nozzle transition station and at the nozzle throat.

**Net installed thrust** (overall engine thrust equation, general airbreathing form):
```
F_net = ṁ_9·V_9 - ṁ_0·V_0 + A_9·(p_9 - p_0)
```
where subscript 9 = nozzle exit, 0 = freestream. The `A_9·(p_9 - p_0)` pressure term is **not negligible for ramjets**, which frequently run under- or over-expanded across their operating Mach range — a sizing code that drops this term and uses only the momentum difference will systematically mis-predict thrust away from the design point.

**Specific thrust and specific impulse** (standard performance metrics):
```
F_s = F_net / ṁ_0                    [specific thrust, N·s/kg]
I_sp = F_net / (ṁ_fuel · g0)          [specific impulse, s]
```
Specific thrust is the ratio of internal thrust to the air mass flow rate at the inlet entrance, and specific impulse is the ratio of internal thrust to the fuel weight flow rate.

**Nozzle mass-flow balancing (iterative)**: for a properly balanced ramjet cycle calculation, both the mass flow and total pressure approaching the nozzle must be known at the combustor exit, calculated from inlet ingestion, fuel injection control, and all component efficiencies/loss factors; if the nozzle's mass-flow capacity at that total pressure is less than the engine's ingested mass flow, inlet spillage must increase; if nozzle capacity exceeds engine mass flow, the inlet must go supercritical, reducing delivered stagnation pressure — both cases require an iterative balance, not a single-pass calculation. This is a critical modeling-fidelity point: **naive sizing codes that compute inlet, combustor, and nozzle sequentially in one pass without an inner mass-flow/pressure iteration loop are not doing real ramjet cycle balancing** — they're doing an open-loop estimate that can diverge from a physically consistent solution, especially off-design.

This is also why typical simplified textbook cycle-analysis models, which work acceptably well for turbine engines, are often inappropriate for ramjets without this iterative balancing — a direct, citable warning against porting turbojet-style sizing-code architecture directly into ramjet or hybrid sizing without modification.

### 2.6 Combined-Cycle / PDE-Ramjet Performance Comparison (context for hybrid claims)
For reference, the parametric-cycle basis of the modern "PDE beats ramjet" claims found in the earlier research survey: a parametric cycle analysis of an ideal pulse-detonation engine for the supersonic-heat-addition branch, built with parameters analogous to standard ideal-ramjet parametric cycle analysis, found that both specific thrust and thrust-flux of the ideal PDE exceed those of the ideal ramjet specifically when the diffuser-exit Mach number M2 is between 1.1 and 2 — i.e., the advantage is regime-specific, not universal across all Mach numbers. If the sizing code will ever model a detonation-augmented mode, this M2-bounded advantage window should be encoded explicitly rather than assumed to hold everywhere.

---

## Recommendations for the Sizing Code
1. Encode the **Lenoir→Humphrey cycle transition** as a function of flight speed and inlet type (straight vs. side), not a fixed cycle assumption, for the pulsejet-mode calculations.
2. Replace any pure-momentum thrust equation with the full **stream-thrust / pressure-term-inclusive** net thrust equation (§2.5), since ramjet nozzles are routinely under/over-expanded.
3. Add an **iterative inlet–combustor–nozzle mass-flow/pressure balance loop** rather than a sequential one-pass calculation, per the explicit warning in §2.5.
4. Model diffuser/combustor **total pressure recovery (π_d, π_b) as Mach- and geometry-dependent**, not flat constants, especially if the code will sweep across the pulsejet→ramjet transition Mach range.
5. Treat the **resonance-tube frequency equation** as one candidate model (quarter-wave) among a genuinely unresolved debate (quarter-wave vs. Helmholtz) — validate against test data rather than hard-coding.
6. If/when a detonation-augmented mode is added, bound any performance-advantage claims to the **M2 regime** in which they're actually derived (§2.6), not applied blanket-wide.

## Caveats
- Equations above are the standard, textbook/peer-reviewed forms for ideal or first-order real cycles; several (especially pulsejet resonance and mean-thrust equations) carry explicitly stated experimental uncertainty (±10–17%) and unresolved theoretical debate (quarter-wave vs. Helmholtz) — they are the best available closed-form models, not settled ground truth.
- This document does not replace a full derivation reference (Mattingly's *Elements of Gas Turbine Propulsion*, Heiser & Pratt's *Hypersonic Airbreathing Propulsion*) — where the sizing code needs a term not covered here, those remain the references to consult.
- No switchable-engine patent gives quantitative valve-loss coefficients or transition timing; anything the sizing code assumes for the transition itself will be an engineering estimate, not a sourced value, until validated.

## Sources
- US2677232A, US2745248A, US3533239A, US3604211A, US4962641A — Google Patents
- US5722234, US5894722 — variable-geometry ramjet patents (Justia/USPTO)
- US5372005, US6250069 — stream-thrust/ramjet cycle patents (USPTO)
- "Thermodynamic performance analysis of ramjet engine at wide working conditions" — ScienceDirect
- "Study of pulse jet engine thermodynamic cycle using workflow mathematical modeling" — Combustion Engines journal
- "Frequency-Response Study for Acoustic Characterization of Valved-Pulsejet Combustors" — ResearchGate
- "Mathematical model and computer program development for online modeling of pulse jet engine working cycle" — Canadian Science Publishing
- "Simulation of a working process in the pulsejet engine with an aerodynamic valve" — ResearchGate
- "An Ex Rocket Man's Take On It: Ramjet Cycle Analyses" — exrocketman.blogspot.com
- "Parametric cycle analysis of an ideal pulse detonation engine – Supersonic branch" — ScienceDirect
