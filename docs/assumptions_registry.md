# Assumptions and evidence registry

`docs/assumptions.md` already documents *what* every current value is and
*why*. This document adds the missing dimension the design-review feedback
asked for: an explicit **evidence-class label** per item, using one fixed
taxonomy, so nothing calculated ever gets silently read as validated.

## Taxonomy

- **Requirement** -- fixed by the competition rules or explicit user
  direction. Not a design variable.
- **Provisional assumption** -- an engineering placeholder standing in for
  data that doesn't exist yet (a literature-typical constant, a round-number
  starting geometry). Expected to change.
- **Calculated output** -- produced by this repository's own models from
  other inputs. Only as trustworthy as the model and the inputs it consumed.
- **Externally sourced** -- taken from a cited outside document (a patent, a
  published paper, a competition rules page) without independent
  verification here.
- **Validated** -- checked against real hardware, a real solver run (not a
  recording fake), or physical test data. As of 2026-08 this repo has real
  OpenVSP/VSPAero and real JSBSim runs (Level 4/5 tooling **executes**
  correctly), but no candidate has passed the Level 4/5 **gates** in
  `docs/design_workflow.md` -- so "the solver ran on real geometry" is not
  the same claim as "validated."

## Classification by section

| Section | Evidence class | Detail |
|---|---|---|
| `RequirementsConfig` (mass limit, supersonic duration, transonic no-altitude-loss rule, allowed propulsion, control/recovery/verification/eligibility rules) | **Requirement** | Sourced from boomsupersonic.com/prize, fetched 2026-08-05 -- see `docs/assumptions.md`'s "Requirements and owner direction" table for the citation per row. |
| Intake mode exclusivity, peak Mach target, speed-run termination rule, body-diameter freedom | **Requirement** (user direction, not competition rule) | Same table; labeled "User requirement" / "User direction" there already. |
| Sled release speed (`MissionConfig.sled_release_speed_min/max_m_per_s`) | **Reclassified this session: from Requirement to a rail-length-bounded Level 2 search variable** | Previously filed as a fixed requirement; it isn't one -- a higher release speed reduces the required lift area, which `feasibility.py`'s Gate 1 check found insufficient at the current configured speed. Now a search variable in `optimizer.py` (`DesignVariableBounds.sled_release_speed_m_per_s`, 35-90 m/s). The upper bound is **not** derived from real sled-rail physics (`v_max = sqrt(2 a_sled rail_length_m)`) -- no launch-acceleration limit has been set for this sled -- so it's a provisional wide search bound, same status as other unsourced placeholders below. `MissionConfig.sled_rail_length_m` (75 m, midpoint of a stated 50-100 m range) is recorded for when a real acceleration limit exists. |
| Pulsejet/ramjet switchable-mode transition Mach thresholds (`minimum_lightoff_test_mach`, `minimum_self_sustaining_mach`) | **Provisional assumption**, explicitly flagged as unsourced | `config.py RamjetConfig` docstring (added this session): no patent in the switchable-engine lineage gives a quantitative transition law. |
| Governing equations for area-Mach relation, stagnation ratios, Brayton/Humphrey cycle relations, stream-thrust net-thrust equation | **Externally sourced**, cited per equation | `docs/pulsejet_ramjet_governing_equations.md`; traceability table in `docs/governing_equations_integration.md`. |
| Side-inlet ram-pressure credit fraction (0.15), supercritical recovery penalty coefficient (0.10) | **Provisional assumption**, explicitly flagged as unsourced | Both added this session with an inline comment stating the reference confirms the physical direction but not the magnitude. |
| Pulsejet/ramjet installed-efficiency recovery factors (0.99 / 0.92), selector discharge coefficient (0.78), combustion efficiency (0.58), target equivalence ratio, burn duration/period, combustor pressure loss, target combustor exit temperature | **Provisional assumption** | `docs/assumptions.md` "Propulsion-model assumptions" table -- every row already carries a "closure path," which is exactly what a provisional-assumption entry needs. |
| Pulsejet cycle-timing values adopted this session (0.014 s period, 0.002 s burn, 1.01 ignition ratio, &Phi;=1.00) | **Calculated output** of a 3,000-point grid search, not yet **validated** | `docs/pulsejet_sweep_results.csv`, `docs/design_convergence.md` "Pulsejet cycle-timing sweep" -- the ±17% thrust / ±10% frequency uncertainty bands from the governing-equations reference apply to real pulsejet correlations in the literature, not yet demonstrated for this specific simulator against test data. |
| Candidate B geometry (body diameter/length, throat, intake, fin/surface planform) | **Calculated output** of static trade screens (`sizing.py`), reported with explicit rejection rationale for Candidate A | `docs/design_convergence.md` -- correctly not labeled validated; still "the current integration point," per the working rules in `docs/design_workflow.md`. |
| Mass budget (21.0 kg current / 23.9 kg high-side) | **Calculated output**, reconciled arithmetic only | `docs/assumptions.md`: "Component allocations reconcile; weighing and hardware definition required" -- i.e. explicitly not validated, and not yet a parametric model tied to geometry (see `docs/design_workflow.md` Level 3 gap). |
| Robustness scenario multipliers (conservative/adverse thrust, drag, mass, recovery factors) | **Provisional assumption**, explicitly non-probabilistic | `docs/assumptions.md`: "engineering screens, not probability statements." |
| OpenVSP-generated geometry, VSPAero coefficient sweeps | **Calculated output** of a real (non-fake) solver run | This session ran both against the current ram-inlet cowl-ratio geometry; genuine solver output, but not yet fed back into the trajectory drag-area budget (Gate 5 not met -- see `docs/design_workflow.md`) and not validated against test data. |
| JSBSim model load/run checks | **Calculated output** of a real (non-fake) solver run, explicitly not a stability gate | `docs/design_workflow.md` Level 5: "provisional stability derivatives and no true control surfaces... treat as an implementation/force-balance cross-check." |
| Fuel properties (LHV, stoichiometric AFR, density) for all four fuels in `fuels.yaml` | **Provisional assumption**, `source_status` field per fuel | `Fuel.source_status` in `config.py` is machine-readable already (`"provisional"` / `"provisional_liquid_density"`); `fuel_trade.py` propagates a `fuel_properties_require_source_validation` status flag when not `"validated"`. |

## What this registry does not yet do

This is a classification pass over existing documented values, not a new
per-field database. It does not (yet):

- Cover every single YAML key individually -- `docs/assumptions.md` already
  does that at the section level; duplicating it field-by-field here would
  create a second copy to keep in sync for no added information.
- Replace the config-file separation `docs/design_workflow.md` proposes
  (`requirements.yaml`, `propulsion_assumptions.yaml`, etc.). That's a
  structural change to `config.py`'s loader, tracked as a later step in the
  design-workflow priority list, not done in this pass.
- Add machine-readable evidence-class metadata to the YAML/dataclasses
  themselves (only `Fuel.source_status` has that today). A natural next step
  once the config-file separation above happens, since each split file could
  carry one evidence class by construction instead of needing a per-field tag.
