# Ramjet model comparison vs. NASA EngineSim

Comparison only, per the request that produced it -- no code changed. Cross-checks
`ramjet.py` (`evaluate_ramjet`) plus its `compressible.py`/`atmosphere.py`/`config.py`
dependencies against NASA Glenn Research Center's **EngineSim** (`EngineSimU`, v1.5,
the "engineering undergraduate" build), downloaded from the official public-domain
distribution at <https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/Enginesim/index.htm>
(`EngineSimU.zip`, 511,034 bytes). The zip ships full Java source (`Turbo.java`,
7,864 lines), not just compiled bytecode, so this is a genuine line-by-line
comparison against NASA's own equations, not a black-box behavioral diff. All six
findings below were independently re-derived from the source by separate verification
passes (not just asserted) and every one came back **CONFIRMED** against the actual
code on both sides.

EngineSim is a single monolithic `Turbo` class handling turbojet/turbofan/ramjet/
afterburner cycles together (AWT GUI code interleaved with the physics), gated by an
`entype` field (`entype == 3` selects ramjet). The physics core lives in the inner
`Solver` class: `getFreeStream()` (atmosphere + stagnation conditions), `getThermo()`
(station-by-station cycle), `getPerform()` (airflow/thrust/SFC), and `getGeo()`
(area scheduling) -- `Turbo.java:245-1356`. Everything below cites that range unless
noted. EngineSim works in imperial units (°R, psi, ft/sec, lbm/sec, BTU/lbm-R);
`ramjet.py` works in SI -- units differ throughout, physics is what's being compared.

## Match 1 -- Fuel-air ratio: algebraically identical Brayton energy balance

**Confirmed by direct algebra, not just inspection.**

EngineSim's fuel-air-ratio formula (`Turbo.java:1111-1112`):

```java
fa = (trat[4]-1.0)/(eta[4]*fhv/(cp3*tt[3])-trat[4]) +
     (trat[7]-1.0)/(fhv/(cpe*tt[15])-trat[7]) ;
```

For a base ramjet (`loadRamj()` sets `abflag = 0`, `Turbo.java:626`), `trat[7]` stays
at its unconditional initial value of `1.0` (`Turbo.java:735`) since the afterburner
block that would change it never runs (`Turbo.java:1032-1035` is gated on
`abflag > 0`). That zeroes the second term entirely, leaving:

```
fa = (trat[4]-1.0) / (eta[4]*fhv/(cp3*tt[3]) - trat[4])
```

Substituting `trat[4] = tt[4]/tt[3]` (confirmed at `Turbo.java:841` and `:1000`) and
multiplying numerator and denominator through by `tt[3]` then `cp3`:

```
fa = cp3*(tt[4]-tt[3]) / (eta[4]*fhv - cp3*tt[4])
```

This is a term-by-term match to `ramjet.py:111-121`:

```python
numerator = cp_j_per_kg_k * max(target_temperature_k - inlet_total_temperature_k, 0.0)
denominator = (
    config.combustor_efficiency * fuel.lower_heating_value_j_per_kg
    - cp_j_per_kg_k * target_temperature_k
)
fuel_air_ratio = numerator / denominator if denominator > 0.0 else 0.0
```

`cp3 <-> cp_j_per_kg_k`, `tt[4] <-> target_temperature_k`, `tt[3] <-> inlet_total_temperature_k`,
`eta[4] <-> combustor_efficiency`, `fhv <-> lower_heating_value_j_per_kg`. `ramjet.py`'s own
comment (lines 111-113) already claims this matches "the reference" exactly -- confirmed
true against NASA's own source, not just an internal governing-equations doc. The only
real difference is **where cp is evaluated** -- see Difference 2 below.

## Match 2 -- Nozzle gross thrust: both use the full stream-thrust equation

**Confirmed: neither model is momentum-only.**

EngineSim's ramjet thrust (`Turbo.java:1079-1085`) adds an explicit pressure-imbalance
term on top of the base momentum term used for turbine engines:

```java
fgros = (uexit + (pexit - ps0)*arexitd*arthd*a2/eair/144.) / g0 ;
```

`ramjet.py` hands gross thrust off to `fixed_cd_nozzle` (`ramjet.py:127-136`), whose
supersonic-exit branch (`compressible.py:415-432`) computes:

```python
raw_thrust_n = (
    mass_flow_kg_per_s * exit_velocity_m_per_s
    + (exit_pressure_pa - ambient_pressure_pa) * exit_area_m2 * discharge_coefficient
)
```

Both are `m_dot * V + A * (p_exit - p_ambient)` -- the compressible stream-thrust form
that matters for a ramjet nozzle routinely running over- or under-expanded away from
its design point. No correction needed here.

## Difference 1 -- Inlet recovery: two different physical models, diverging fast with Mach

**Implemented (2026-08-10).** `ramjet.py`'s `ideal_inlet_shock_recovery` now uses the
MIL-E-5008B correlation directly for M>1 instead of the normal-shock relation; see
that function's current docstring. What follows is the finding as it stood before
the change, kept for the record.

EngineSim's default inlet recovery (`Turbo.java:716-722`, the "Mil Spec Recovery"
option, `pt2flag == 0`) is the **MIL-E-5008B empirical correlation**:

```java
if (fsmach > 1.0) { prat[2] = 1.0 - .075*Math.pow(fsmach - 1.0, 1.35); }
else               { prat[2] = 1.0; }
```

`ramjet.py`'s `ideal_inlet_shock_recovery` (`ramjet.py:17-48`) instead returns the
**stationary normal-shock total-pressure ratio** above M=1 (`compressible.py:178-191`),
which the function's own docstring already flags as "a single-normal-shock model, not
a multi-oblique-shock MIL-E-5008B-style correlation... it would need replacing... before
extending to higher supersonic Mach numbers" (`ramjet.py:24-28`). Direct computation
(gamma=1.4) confirms exactly the gap that docstring anticipated, and shows it isn't
small:

| Mach | MIL-E-5008B (EngineSim) | Normal shock (`ramjet.py`) | Gap |
|------|--------------------------|------------------------------|-----|
| 1.5  | 0.9706 | 0.9298 | 4.1 points |
| 2.0  | 0.9250 | 0.7209 | 20.4 points |

The gap roughly quintuples between M=1.5 and M=2.0. At this vehicle's transonic design
point (M~1.0-1.2) the two models nearly agree (both are close to 1.0 just above M=1),
so this isn't an active bug today -- but it is the single most concrete, literature-backed
upgrade available if the design envelope ever pushes past M~1.3-1.5. Since NASA's own
formula is now sitting in this repo's scratchpad, adopting `1.0 - 0.075*(M-1)^1.35` in
`ideal_inlet_shock_recovery` for M>1 (as an alternative to the normal-shock branch) is a
drop-in change, not a research problem.

## Difference 2 -- Real-gas properties: EngineSim varies gamma/cp with temperature by default; `ramjet.py` does not

**Implemented (2026-08-10).** New `gas_properties.py` ports EngineSim's `getGama`/
`getCp` cubic fits (unit-converted to SI). `evaluate_ramjet` now evaluates cp at the
combustor-inlet temperature for the fuel-air-ratio balance, and a separate exhaust
gamma at the combustor-exit temperature for the nozzle expansion -- matching
EngineSim's `cp3`/`game` station-specific evaluations. What follows is the finding as
it stood before the change, kept for the record.

EngineSim's `getGama`/`getCp` (`Turbo.java:1253-1284`) are cubic polynomials in local
static/total temperature:

```java
number = a*temp*temp*temp + b*temp*temp + c*temp + d ;   // per-station gamma or cp
```

`gamopt` (which selects this branch over a constant fallback) defaults to `1` in both
`setDefaults()` (`Turbo.java:271`) and the ramjet-specific `loadRamj()`
(`Turbo.java:640`) -- i.e. **variable properties are the default**, not an optional
mode. They are genuinely invoked per-station throughout the ramjet cycle (at least 12
call sites across `getThermo`/`getPerform`, e.g. `Turbo.java:794-795, 890-891, 997-998,
1054-1059`), each passing that station's own `tt[n]`.

`ramjet.py` derives one `cp_j_per_kg_k` from the single scalar `RamjetConfig.gamma`
once (`ramjet.py:114`), and reuses that same gamma/cp unchanged from the freestream
through the inlet, combustor, and nozzle (`ramjet.py:103,134`, and every iteration of
the supercritical-recovery loop at `ramjet.py:184`) -- a calorically-perfect-gas
assumption end to end. `compressible.py` has no temperature-dependent property
function anywhere in the file.

This is a real fidelity gap (combustion products at ~1900-2200 K genuinely have lower
gamma / higher cp than cold inlet air), but it's a second-order effect next to
Difference 1 above -- worth a note, not the first thing to fix.

## Difference 3 -- Airflow determination: EngineSim always assumes a choked throat; `ramjet.py` models inlet/nozzle mismatch explicitly

**The largest architectural difference between the two models -- neither is "wrong," they solve different problems.**

EngineSim's ramjet airflow (`Turbo.java:1067-1069`) is one unconditional, non-iterative
line, always assuming the nozzle throat is exactly choked at M=1:

```java
if (entype == 3) {// for ramjets - airflow determined at nozzle throat
     eair = getAir(1.0,game)*144.0*a2*arthd * epr*prat[2]*pt[0]/14.7 /
             Math.sqrt(etr*tt[0]/518.) ;
}
```

Nothing downstream corrects `eair` against a separately computed freestream-capture
value -- there is no spillage fraction, no oversupply iteration, anywhere in the
ramjet path (confirmed by exhaustive grep across the file).

`ramjet.py` instead computes a freestream-capture-limited "potential" airflow first
(`ramjet.py:95-100`), separately evaluates the fixed nozzle's actual mass-flow capacity
(`ramjet.py:127-136`), and explicitly reconciles the two: spillage if the nozzle is
undersized, or an iterative supercritical recovery-penalty loop (up to 30 iterations,
`ramjet.py:161-194`) if the nozzle could pass more than the engine is delivering. This
is materially more general than EngineSim's ramjet mode -- it's the reason `ramjet.py`
reports an `inlet_spillage_fraction` and `nozzle_mass_flow_residual_fraction` at all,
concepts EngineSim's ramjet path doesn't represent. Nothing to port here; if anything
this repo's model already goes further than EngineSim's on this specific point.

## Difference 4 -- Atmosphere: `ramjet.py` omits EngineSim's third layer, but never reaches it

**Confirmed inconsequential for this vehicle's stated envelope.**

EngineSim's atmosphere (`Turbo.java:674-706`) has three layers: troposphere (<36,152
ft), isothermal stratosphere (36,152-82,345 ft), and a third layer above 82,345 ft
where temperature rises again. `atmosphere.py` implements only the first two (tropopause
at 11,000 m, isothermal above), with its documented valid range capped at -1,000 to
20,000 m.

Converting: 20,000 m = 65,617 ft, EngineSim's third layer starts at 82,345 ft =
25,099 m. `atmosphere.py`'s entire supported range sits ~16,700 ft (~5,100 m) below
where the omitted layer would even start, so this omission never bites this vehicle's
actual operating envelope. (The tropopause breakpoints also cross-check: EngineSim's
36,152 ft = 11,019 m vs. `atmosphere.py`'s 11,000 m, a 0.17% difference -- both are the
same standard-atmosphere tropopause, just rounded differently between imperial and SI
conventions.) No action needed.

## Bottom line

Two genuine matches (fuel-air ratio, nozzle stream-thrust form) confirmed the core
combustor and nozzle equations were already sound against NASA's own reference.
Differences 1 (inlet recovery) and 2 (real-gas properties) have since been
implemented -- see `ramjet.py`'s `ideal_inlet_shock_recovery` and the new
`gas_properties.py`. Difference 3 (airflow/choking structure) was not a gap to
close and nothing changed there -- `ramjet.py` already models a broader range of
physical regimes than EngineSim's ramjet mode does. Difference 4 (atmosphere
layers) needed no action.
