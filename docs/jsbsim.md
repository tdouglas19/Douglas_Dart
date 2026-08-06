# JSBSim integration

`src/douglas_dart/jsbsim_model.py` generates a JSBSim aircraft model directly from
`configs/*.yaml` — the same authoritative configuration used by every other
discipline in this repository. It is the bridge from the repository's own low-order
propulsion, drag, and geometry models into a real 6-DOF flight-dynamics engine.

## Why this exists

`trajectory.py` is a fast, phase-based *energy-state* mission model: flight-path
angle is prescribed per phase, not solved from a lift/trim balance, and it has no
stability or control representation at all. JSBSim gives an independent way to check
whether the same propulsion and drag numbers hold up under real 6-DOF dynamics, and
is the eventual path to trim analysis, short-period/phugoid inspection, and — once
control surfaces are defined — closed-loop handling-qualities work.

## What is generated

For a given `ReferenceCase` and named scenario (`nominal`, `conservative`, or
`adverse` — the same `MissionScenario` objects used by `trajectory.py`, so a JSBSim
run and a `mission-trajectory` run at the same scenario are directly comparable):

- **Metrics/mass_balance**: reference area, span, and MAC from the same
  `vspaero_reference_quantities`/MAC helpers OpenVSP uses; CG from
  `openvsp.analysis.reference_cg_x_m`; `ixx`/`iyy`/`izz` from a **uniform-density
  solid-cylinder approximation** of the outer mold line — not a real distributed mass
  model. Empty weight is loaded mass minus the configured fuel mass; the fuel itself
  is a `<tank>` for bookkeeping only (nothing drains it in real time — mission fuel
  burn stays `trajectory.py`'s job).
- **Propulsion**: JSBSim has no native pulsejet or ramjet `FGEngine` type. Both are
  represented as `<external_reactions>` body-axis forces driven by 2-D
  Mach/altitude thrust tables built from this repository's own `pulsejet.py` and
  `ramjet.py`, converted from Newtons to the pound-force JSBSim uses internally. A
  `<switch>` gated on `velocities/mach` against
  `ramjet.minimum_lightoff_test_mach` keeps the two propulsion modes mutually
  exclusive, mirroring the physical selector — never both engines contributing
  thrust at once.
- **Drag**: `aero/coefficient/CD0` is a Mach table built directly from
  `drag.mach_indexed_zero_lift_drag_area_m2`, so the JSBSim model and the flight
  kernel/sizing screens read the same transonic drag-rise shape. Induced drag uses
  the configured `induced_drag_factor`.
- **Static/damping derivatives** (`Cmalpha`, `Cmq`, `Cnbeta`, `Cnr`, `Clp`,
  `CYbeta`): textbook tail-volume-coefficient and strip-theory formulas computed from
  this vehicle's *actual* configured fin count, clocking angle, area, and arm to the
  reference CG (`estimate_stability_derivatives`) — not borrowed or invented
  constants. They are explicitly marked provisional; `docs/roadmap.md`'s Phase 3
  aerodynamic closure (VSPAERO-derived derivatives) is the intended replacement.
- **No control surfaces.** The configuration does not yet define any aileron,
  elevator, or rudder geometry, so none is fabricated here. The fins and lifting
  surfaces are fixed stabilizers; only static and damping derivatives are present.
  This supports open-loop trim/dynamic-mode inspection, not closed-loop
  handling-qualities analysis.
- **Ground reactions**: a single placeholder "STRUCTURE" contact at the belly.
  Recovery method is an explicit open decision in `docs/roadmap.md`
  ("Decisions still owned by the user"); this contact must be replaced once that is
  settled.

## Running it

```bash
python -m pip install -e .   # jsbsim is now a core dependency

douglas-dart jsbsim-build \
  --config configs/shared_nozzle_candidate_b.yaml \
  --scenario adverse \
  --root jsbsim/generated \
  --check --check-mach 1.10 --check-altitude 4500 --check-seconds 5
```

`--check` loads the generated model in the **real, installed** JSBSim engine
(`jsbsim.FGFDMExec`) and runs it for the requested number of seconds, reporting final
Mach/altitude/alpha and which propulsion mode was active — genuine verification
against the actual solver, not a recording fake. `run_all.py` runs this build+check
for the nominal and adverse scenarios automatically unless `--skip-jsbsim` or
`--skip-jsbsim-check` is passed.

Generated models live under `jsbsim/generated/aircraft/<case>_<scenario>/` and are
git-ignored, same as `openvsp/generated/`.

## Cross-check so far

Running the generated Candidate B model at Mach 1.10 / 4,500 m for a few seconds of
simulated time reproduces the same qualitative result as the independent static
screen in `robustness_candidate_b.yaml` and the `trajectory.py` mission integrator:
the **nominal** scenario accelerates past Mach 1.10, while the **adverse** scenario
(0.82 recovery, 0.75 thrust multiplier, 1.20 drag multiplier, +2.9 kg) decelerates
below it. Three independently implemented models now agree on this result, which is
stronger evidence than any one of them alone — but all three still share the same
underlying propulsion/drag assumptions, so this is corroboration of internal
consistency, not external validation.

## Opening in JSBSim's own tools

Point `jsbsim --root=jsbsim/generated <model_name>` (or FlightGear via the JSBSim
FDM) at `jsbsim/generated` to fly the model interactively, or load
`jsbsim/generated/aircraft/<model_name>/<model_name>.xml` directly in any JSBSim-aware
GUI (e.g. FlightGear's aircraft selector once copied into its own `aircraft/`
directory).

## What this does not do yet

- No fuel-limited mission simulation (see `trajectory.py` for that).
- No trim solver call — JSBSim's Python bindings expose the state and controls, but
  an automated trim routine (finding alpha/elevator for level flight at a point) is
  not wired up yet.
- No control-surface or actuator model.
- No landing-gear/recovery-hardware model — the single placeholder ground contact is
  not a skid, parachute, or gear design.
- No STL/visual-model export path from OpenVSP into JSBSim yet (see
  `openvsp/README.md` for the OpenVSP side and the STL/JSBSim discussion in this
  session's transcript for the human-in-the-loop rationale).
