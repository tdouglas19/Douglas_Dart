# Governing-equations integration: changelog and open questions

Traceability from code to the physics in `pulsejet_ramjet_governing_equations.md`
(supplied 2026-08-06), covering the pulsejet/ramjet switchable-engine patent
lineage (sec. 1) and the governing inlet/combustor/nozzle/resonance equations
(sec. 2). Organized by subsystem, in the order integrated.

## 1. Inlet / diffuser (sec. 2.2)

| File | Change | Source |
|---|---|---|
| `compressible.py` `area_ratio_from_mach`, `subsonic_mach_from_area_ratio`, `supersonic_mach_from_area_ratio`, `stagnation_temperature`, `stagnation_pressure` | **Already correct, not changed.** These already implement the isentropic area-Mach relation and stagnation temperature/pressure ratios exactly as given. Not hardcoded constants -- genuine Mach-dependent functions used throughout the nozzle solver. | sec. 2.2 |
| `ramjet.py` `ideal_inlet_shock_recovery` | **Already correct, not changed** (comment added citing the section). pi_d is already Mach-dependent: 1.0 (lossless) at/below Mach 1, the real normal-shock total-pressure ratio above it -- not a flat constant. Noted as a single-normal-shock model, adequate at this vehicle's ~Mach 1.0-1.2 design range but not a multi-oblique-shock MIL-E-5008B correlation; would need replacing before extending to higher supersonic Mach. | sec. 2.2 |
| `config.py` `SelectorConfig.inlet_type` (new field, default `"straight"`) | **Added.** Selects which pulsejet-mode cycle regime applies. | sec. 2.2 |
| `pulsejet.py` `pulsejet_cycle_mode()` (new) | **Added.** Diagnostic function reporting Lenoir / transitional / Humphrey based on Mach and inlet type. | sec. 2.2, 2.4 |
| `pulsejet.py` `PulsejetSimulator.__init__` inlet pressure calc | **Changed.** For `inlet_type="side"`, only `SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION` (0.15, **unsourced placeholder**) of the Mach-dependent ram-pressure rise is credited, since a side inlet shows little pre-compression at any speed. `"straight"` (the default) is numerically unchanged from before this pass. | sec. 2.2 |

**Note on the existing straight-inlet behavior:** `PulsejetSimulator` was already using `stagnation_pressure(ambient, mach)` for the inlet total pressure -- at Mach 0 this equals static pressure (no ram rise, Lenoir-like), rising with Mach (Humphrey-like), which is the *correct qualitative trend* even before this pass. What was missing was making that behavior inlet-type-dependent instead of applying it uniformly regardless of geometry, and giving it a name/citation.

## 2. Combustor (sec. 2.3, 2.4)

| File | Change | Source |
|---|---|---|
| `ramjet.py` `evaluate_ramjet` fuel-air ratio | **Already correct, not changed** (comment added). `f = cp*(Tt3-Tt2) / (eta_b*h_PR - cp*Tt3)` matches the reference exactly. | sec. 2.3 |
| `ramjet.py` `combustor_exit_pressure_pa` | **Already correct, not changed** (comment added). `pi_b = combustor_total_pressure_loss_fraction` complement is a nonzero configured loss, not assumed = 1. | sec. 2.3 |
| `pulsejet.py` `humphrey_cycle_thermal_efficiency()` (new) | **Added.** Idealized closed-form Humphrey efficiency, `eta = 1 - gamma*(tau^(1/gamma)-1)/(tau-1)`. Reference/diagnostic only -- `PulsejetSimulator` integrates real unsteady mass/energy flow, it does not evaluate this formula directly. | sec. 2.4 |
| `pulsejet.py` `_ignite_if_ready` | **Comment added, no logic change.** Mapped the existing mass-flow-in / constant-volume-heat-release / nozzle-expansion structure onto the Humphrey cycle's process 1-2 / 2-3 / 3-4 stages. | sec. 2.4 |

**Combustor/inlet consistency check (explicitly requested):** both subsystems now agree that cycle regime is a function of `(mach, inlet_type)`, sourced from the same `SelectorConfig.inlet_type` field -- there is one source of truth, not two subsystems each guessing independently.

## 3. Resonance tube (sec. 2.4)

| File | Change | Source |
|---|---|---|
| `pulsejet.py` `quarter_wave_resonance_frequency_hz()` (new) | **Added.** `f = a / (4*L_eff)`. `RESONANCE_FREQUENCY_MODEL = "quarter_wave"` / `RESONANCE_FREQUENCY_MODEL_ALTERNATIVES = ("helmholtz",)` flag the unresolved quarter-wave-vs-Helmholtz debate; Helmholtz is **not implemented**. | sec. 2.4 |
| `pulsejet.py` `RESONANCE_FREQUENCY_RELATIVE_UNCERTAINTY` (0.10), `RESONANCE_MEAN_THRUST_RELATIVE_UNCERTAINTY` (0.17), `resonance_mean_thrust_uncertainty_band_n()` (new) | **Added.** Encodes the reference's stated experimental error bands as metadata/helper rather than a silent point estimate. | sec. 2.4 |

**Not implemented, and explicitly flagged as such:** the reference describes the existence and reported accuracy of a semi-empirical valveless-pulsejet mean-thrust/frequency correlation, but does not give its closed-form equation -- only the quarter-wave frequency formula and the error bounds are reproducible from this document. See "Still open" below.

## 4. Nozzle & overall thrust (sec. 2.5) -- highest priority

| File | Change | Source |
|---|---|---|
| `compressible.py` `fixed_cd_nozzle` supersonic-exit branch | **Already correct, not changed** (comment added). The `A9*(p9-p0)` pressure term was already present in `raw_thrust_n`. Explicitly verified this is *not* a naive momentum-only thrust equation. | sec. 2.5 |
| `ramjet.py` `net_thrust_n` | **Already correct, not changed** (comment added). `gross_thrust_n` (pressure-inclusive) minus `air_mass_flow*V0` already matches `F_net = m9V9 - m0V0 + A9(p9-p0)`. | sec. 2.5 |
| `ramjet.py` `specific_thrust_n_s_per_kg_air` | **Already correct, not changed.** `F_net/m0_air` matches `F_s` exactly. | sec. 2.5 |
| `ramjet.py` `RamjetResult.specific_impulse_s` (new field) | **Added.** Was absent from the library entirely (only ever computed ad hoc in a one-off script). `I_sp = F_net/(mdot_fuel*g0)`. | sec. 2.5 |
| `pulsejet.py` `PulsejetSummary.specific_impulse_s` (new field) | **Added,** same definition, for the pulsejet side. | sec. 2.5 |
| `config.py` `RamjetConfig.supercritical_recovery_penalty_coefficient` (new field, default 0.10, **unsourced placeholder**) | **Added.** | sec. 2.5 |
| `ramjet.py` `evaluate_ramjet` -- **structural fix** | **Changed.** This was the real fidelity gap the reference calls out: the nozzle capacity vs. demanded-mass-flow mismatch was resolved with a single post-hoc gross-thrust scaling, not an iterative pressure/mass-flow balance. Now: the **subcritical** case (nozzle throat-limited, capacity < demand) is unchanged -- the existing inlet-spillage treatment already satisfies the reference's requirement here, since spillage happens upstream and doesn't itself change the captured stream's stagnation pressure. The **supercritical** case (capacity >= demand, nozzle under-filled) now iterates: an additional total-pressure-recovery penalty, driven by how far capacity exceeds demand (`oversupply_ratio`), is applied and the nozzle re-solved, repeating until `oversupply_ratio` converges (`\|delta\| < 1e-6`, capped at 30 iterations). Verified to measurably change results in a supercritical test case (~9.5% thrust reduction at the tested oversized-throat point) and to reduce to the prior one-pass behavior when the new coefficient is set to 0. | sec. 2.5 |

**Convergence criterion:** absolute change in `oversupply_ratio` (nozzle capacity / demanded flow - 1) below `1e-6` between iterations, or 30 iterations, whichever comes first. In practice this low-order model converges in a handful of iterations.

## 5. Combined / switchable mode (sec. 1, 2.6)

| File | Change | Source |
|---|---|---|
| `config.py` `RamjetConfig` docstring | **Added.** No logic change -- `minimum_lightoff_test_mach` / `minimum_self_sustaining_mach` were already ordinary tunable config fields (not hardcoded), and `docs/design_convergence.md` already treats them as open trade variables. The new docstring makes explicit, at the source, that no patent in the switchable lineage (Collins/Winter-McDonnell/Ghougasian) gives a quantitative transition law -- these two thresholds are an engineering assumption, not a sourced value. | sec. 1.1, 1.3 |
| (no detonation-augmented / PDE mode exists in this codebase) | **Not applicable.** Sec. 2.6's M2-bounded (Mach 1.1-2.0) PDE-vs-ramjet advantage window has nothing to bound yet. If a detonation-augmented mode is ever added, that M2 window must be encoded explicitly at that time, not assumed to hold everywhere. | sec. 2.6 |

## Still open (reference doesn't have enough information to resolve)

- **Valveless-pulsejet mean-thrust/frequency closed-form correlation** (sec. 2.4): the reference describes its existence, its non-dimensional discharge-velocity-vs-compression-ratio trend, and its reported error (<10% frequency, +-17% thrust), but not the equation itself. Need the underlying paper ("Study of pulse jet engine thermodynamic cycle using workflow mathematical modeling," or the specific valveless-pulsejet mean-thrust-and-frequency paper referenced) to implement it directly rather than only its error bounds.
- **Quarter-wave vs. Helmholtz resonance model resolution**: both remain unimplemented-as-settled by design; resolving which (or under what conditions each) applies needs test data, not more literature review.
- **Resonance tube effective length (`L_eff`)**: no geometry field for this exists in `PulsejetConfig` yet (chamber volume is tracked, tube length is not). `quarter_wave_resonance_frequency_hz()` takes it as a parameter rather than pulling from config, deliberately, to avoid inventing a vehicle-specific tube length.
- **Supercritical recovery penalty magnitude** (sec. 2.5): the reference confirms the physical coupling (supercritical operation reduces delivered stagnation pressure) but gives no correlation for its size. `supercritical_recovery_penalty_coefficient` (default 0.10) is a tunable placeholder pending real data.
- **Side-inlet ram-pressure credit fraction** (sec. 2.2): same situation -- qualitative direction is sourced ("little pre-compression"), magnitude (`SIDE_INLET_RAM_PRESSURE_CREDIT_FRACTION = 0.15`) is not.
- **Combustor residence-time / Damköhler sizing** (sec. 2.3): `V_cc` sizing needs real chemical kinetics data or an empirical loading-parameter correlation for the specific fuel/oxidizer pair; out of scope for this pass and not attempted.
- **MIL-E-5008B-style multi-oblique-shock inlet correlation** (sec. 2.2): not needed at this vehicle's current ~Mach 1.0-1.2 design range (single normal shock is adequate there) but would be needed before extending to higher supersonic Mach.
- **Switchable-mode valve-loss coefficients and transition timing** (sec. 1.3): explicitly stated in the reference as absent from every patent in the lineage; nothing to integrate until first-principles or general check-valve loss correlations are sourced separately.
