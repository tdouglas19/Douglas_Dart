"""First-principles propulsion marched ALONG the trajectory (#25).

No precomputed tables and no interpolation anywhere: the engines are
transient simulations that are carried forward in time with the vehicle,
seeded from their own previous converged state, and re-converged at the
REAL flight condition whenever that condition has drifted enough to
matter.

Why not a table. A grid table in (Mach, altitude) has to interpolate, and
the flame-stability boundary is a FOLD, not a gradient -- blending a lit
corner with a blown-off one produces an engine that is neither lit nor
dead, burning half the fuel. Marching sidesteps that entirely: every
value is a real answer at a state actually flown.

Why not one FP run per timestep. A warm-started convergence costs ~40 s
wall; the powered phase is ~600 timesteps, so that is ~14 h per mission.
It would also be asking for resolution the engine does not have: in the
time the condition drifts by one continuation step the pulsejet completes
~50 cycles and the ramjet's chugging ~640, so its cycle-mean simply
cannot resolve a dM of 0.003.

So the trajectory integrates at its own dt, and the ENGINE is
re-converged on drift triggers (dM 0.05, d_alt 250 ft -- the validated
continuation limits), holding its last cycle-mean in between while
keeping the snapshot live to seed the next re-convergence.

Handoff and flameout are events, not table lookups:
  * Below the lightoff Mach the ramjet is simply not asked.
  * At and above it, the ramjet is asked "can you COLD LIGHT here?" --
    the first-principles answer, not a config threshold.
  * If it will not light, the pulsejet keeps running and the question is
    re-asked at every subsequent trigger, until it lights or the flight
    ends. (Re-asking more often than that is meaningless: the model
    cannot distinguish M 0.450 from M 0.451.)
  * Once lit it is marched warm, and if it blows out it goes back to
    needing a cold light.

Both engines run ADDITIVELY through the transition, matching the
ancestor: the pulsejet keeps pulsing while the ramjet duct lights.
"""
from __future__ import annotations

from dataclasses import dataclass, field

METRES_PER_FOOT = 0.3048

# Validated continuation limits (ramjet-fp campaign): dM 0.05 is safe;
# dM 0.1 produced artifact blow-offs where the flame died from the step
# transient rather than from physics. The freestream ramp removes that
# transient, but the step is kept small deliberately -- the big step is
# validated at one condition pair only, and the flame is most fragile
# exactly where it has not been tested.
DEFAULT_MACH_STEP = 0.05
DEFAULT_ALTITUDE_STEP_M = 250.0 * METRES_PER_FOOT   # ~76 m

# The powered phase of this mission spans only ~250 ft of altitude, so the
# altitude trigger fires once or twice all flight; Mach is the real driver.


@dataclass
class EngineTrace:
    """What the engine did at each re-convergence -- the record the
    fuel-consumption and propulsion plots are built from."""

    time_s: list[float] = field(default_factory=list)
    mach: list[float] = field(default_factory=list)
    altitude_m: list[float] = field(default_factory=list)
    pulsejet_thrust_n: list[float] = field(default_factory=list)
    ramjet_thrust_n: list[float] = field(default_factory=list)
    pulsejet_fuel_kg_s: list[float] = field(default_factory=list)
    ramjet_fuel_kg_s: list[float] = field(default_factory=list)
    ramjet_phi: list[float | None] = field(default_factory=list)
    ramjet_lit: list[bool] = field(default_factory=list)
    events: list[tuple[float, str]] = field(default_factory=list)

    def note(self, t: float, what: str) -> None:
        self.events.append((t, what))


@dataclass
class _Held:
    """The last converged cycle-mean, held between re-convergences."""
    thrust_n: float = 0.0
    fuel_kg_s: float = 0.0
    valid: bool = False


class FpPropulsion:
    """Marched first-principles propulsion for one flight.

    Call :meth:`thrust_and_fuel` every timestep; it decides internally
    whether the condition has drifted enough to warrant a new transient.
    """

    def __init__(self, spec, *, fuel: str = "propane",
                 lightoff_mach: float = 0.45,
                 mach_step: float = DEFAULT_MACH_STEP,
                 altitude_step_m: float = DEFAULT_ALTITUDE_STEP_M,
                 n_cells: int = 162,
                 snapshot_store=None):
        self.spec = spec
        self.fuel = fuel
        self.lightoff_mach = lightoff_mach
        self.mach_step = mach_step
        self.altitude_step_m = altitude_step_m
        self.n_cells = n_cells
        self.store = snapshot_store

        self.trace = EngineTrace()
        self._pj = _Held()
        self._rj = _Held()
        self._pj_state = None
        self._rj_state = None
        self._rj_phi = None
        self._rj_lit = False
        self._last_pj_condition = None      # (mach, alt) of last converge
        self._last_rj_condition = None
        self._n_transients = 0

    # -- drift test ----------------------------------------------------
    def _drifted(self, last, mach: float, altitude_m: float) -> bool:
        if last is None:
            return True
        dm = abs(mach - last[0])
        dh = abs(altitude_m - last[1])
        return dm >= self.mach_step or dh >= self.altitude_step_m

    # -- the per-timestep entry point ----------------------------------
    def thrust_and_fuel(self, t_s: float, mach: float, altitude_m: float
                        ) -> tuple[float, float]:
        """(total thrust N, total fuel kg/s) for both engines combined."""
        if self._drifted(self._last_pj_condition, mach, altitude_m):
            self._converge_pulsejet(t_s, mach, altitude_m)
        if mach >= self.lightoff_mach and self._drifted(
                self._last_rj_condition, mach, altitude_m):
            self._converge_ramjet(t_s, mach, altitude_m)
        thrust = (self._pj.thrust_n if self._pj.valid else 0.0) \
            + (self._rj.thrust_n if self._rj.valid else 0.0)
        fuel = (self._pj.fuel_kg_s if self._pj.valid else 0.0) \
            + (self._rj.fuel_kg_s if self._rj.valid else 0.0)
        return thrust, fuel

    # -- pulsejet ------------------------------------------------------
    def _converge_pulsejet(self, t_s, mach, altitude_m):
        from pulsejet_fp import (Numerics, pulsejet_thrust, reference_gas,
                                 reference_valve)

        prev = self._last_pj_condition
        try:
            res = pulsejet_thrust(
                mach=mach, altitude_m=altitude_m,
                gas=reference_gas(phi=1.0),
                geom=self.spec.pulsejet_geometry(),
                valve=self.spec.pulsejet_valve(reference_valve()),
                intake=self.spec.pulsejet_intake(),
                numerics=Numerics(n_cells=self.n_cells),
                t_end=0.9 if prev is None else 0.5,
                stop_when_converged=True,
                seed_state=self._pj_state,
                return_state=True,
            )
        except Exception as exc:                     # never kill a flight
            self.trace.note(t_s, f"pulsejet_fp_error:{type(exc).__name__}")
            self._pj.valid = False
            self._last_pj_condition = (mach, altitude_m)
            return

        self._n_transients += 1
        ok = res.status == "converged" and res.thrust_n > 0.0
        if ok:
            self._pj = _Held(res.thrust_n, res.mdot_fuel_kg_s, True)
            self._pj_state = res.end_state
        else:
            # a quenched pulsejet is a real outcome, not an error
            if self._pj.valid:
                self.trace.note(t_s, f"pulsejet_quenched_M{mach:.3f}")
            self._pj = _Held(0.0, 0.0, False)
            self._pj_state = None
        self._last_pj_condition = (mach, altitude_m)
        self._record(t_s, mach, altitude_m)

    # -- ramjet --------------------------------------------------------
    def _converge_ramjet(self, t_s, mach, altitude_m):
        from ramjet_fp import Numerics, ramjet_operating_point

        prev = self._last_rj_condition
        cold_light = not self._rj_lit
        try:
            op = ramjet_operating_point(
                mach, altitude_m,
                geom=self.spec.ramjet_geometry(),
                fh=self.spec.ramjet_flameholder(),
                numerics=Numerics(n_cells=self.n_cells),
                fuel=self.fuel,
                phi_seed=None if cold_light else self._rj_phi,
                t_end=0.35,
                seed_state=None if cold_light else self._rj_state,
                ramp_from=None if (cold_light or prev is None) else prev,
                return_state=True,
            )
        except Exception as exc:
            self.trace.note(t_s, f"ramjet_fp_error:{type(exc).__name__}")
            self._last_rj_condition = (mach, altitude_m)
            return

        self._n_transients += op.n_transients or 1
        if op.viable:
            if not self._rj_lit:
                self.trace.note(
                    t_s, f"ramjet_lit_M{mach:.3f}_alt{altitude_m:.0f}"
                         f"_phi{op.required_phi:.3f}")
            self._rj_lit = True
            self._rj_phi = op.required_phi
            self._rj = _Held(op.net_thrust_n, op.mdot_fuel_kg_s, True)
            self._rj_state = (op.result.end_state
                              if op.result is not None else None)
        else:
            if self._rj_lit:
                self.trace.note(t_s, f"ramjet_flameout_M{mach:.3f}")
            elif prev is None:
                self.trace.note(t_s, f"ramjet_light_refused_M{mach:.3f}")
            self._rj_lit = False
            self._rj_state = None
            self._rj_phi = None
            # engine-out is still a FORCE: cold-throughflow drag, fuel cut
            self._rj = _Held(min(op.net_thrust_n, 0.0), 0.0, True)
        self._last_rj_condition = (mach, altitude_m)
        self._record(t_s, mach, altitude_m)

    # -- bookkeeping ---------------------------------------------------
    def _record(self, t_s, mach, altitude_m):
        tr = self.trace
        tr.time_s.append(t_s)
        tr.mach.append(mach)
        tr.altitude_m.append(altitude_m)
        tr.pulsejet_thrust_n.append(self._pj.thrust_n if self._pj.valid else 0.0)
        tr.ramjet_thrust_n.append(self._rj.thrust_n if self._rj.valid else 0.0)
        tr.pulsejet_fuel_kg_s.append(self._pj.fuel_kg_s if self._pj.valid else 0.0)
        tr.ramjet_fuel_kg_s.append(self._rj.fuel_kg_s if self._rj.valid else 0.0)
        tr.ramjet_phi.append(self._rj_phi)
        tr.ramjet_lit.append(self._rj_lit)

    @property
    def n_transients(self) -> int:
        """FP runs spent on this flight -- the cost meter."""
        return self._n_transients
