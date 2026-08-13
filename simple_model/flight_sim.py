"""Point-mass flight simulator: combined-cycle (pulsejet -> ramjet) powered
climb from a standing start, motor cutoff at a target Mach, an unpowered
glide back down, then a flare to bleed the last bit of speed off at roughly
level flight until landing at stall speed.

Explicit-Euler time marching -- each step is just an algebraic evaluation of
the pulsejet/ramjet/drag closed forms above, so a small fixed dt is cheap and
sufficient; there is no internal ODE inside any single physics call, only the
outer trajectory integration itself.

Mass bookkeeping matches the user's own approach: the vehicle always starts
at its fixed max wet mass (50 lb); there is no separately-specified dry mass.
The simulation just tracks cumulative fuel burned, so "fuel required" is read
off as (initial wet mass - mass remaining) at motor cutoff.

The sim starts from a modest non-zero release speed rather than a dead
stop. This isn't just numerical convenience: the induced-drag formula
L^2/(q*pi*b^2*e) divides by dynamic pressure, which is genuinely singular
as V -> 0 while the vehicle still needs to support its full weight
aerodynamically during the climb -- no finite thrust can push through an
infinite drag term. Real vehicles of this kind avoid the problem by never
actually flying at near-zero airspeed: this program's own reference case
(configs/reference_case.yaml) assumes a sled launch releasing at
39-42 m/s, i.e. aerodynamic flight only begins once there is real dynamic
pressure to fly on. DEFAULT_RELEASE_VELOCITY_M_PER_S below is set well
inside that band for the same reason, not picked to paper over the
singularity. The same singularity is why MINIMUM_FLIGHT_SPEED_MARGIN
below is defined relative to stall speed rather than the old flat 10 m/s:
below some fraction of stall speed the quasi-steady-lift assumption behind
every drag call in this module has broken down regardless of phase, and
grinding through steps of runaway induced drag there would just waste time
computing garbage.

Glide phase: at motor cutoff, thrust and fuel flow both drop straight to
zero -- no idle thrust. The flight path angle is a *fixed* constant
(GLIDE_ANGLE_DEG), not re-solved for zero acceleration every step -- that
was tried first (a per-step equilibrium-seeking bisection) and doesn't
actually decelerate the vehicle: solving for the angle that zeroes out net
acceleration at *whatever speed the vehicle currently has* accepts that
speed as a valid trim point for any speed at all, so a supersonic-ish
cutoff speed just gets held, not shed, and a bigger wingspan (which only
lowers the stall-speed *target*) can't help if the vehicle never
decelerates toward it. A fixed angle instead has a genuine, physically
grounded equilibrium speed of its own (drag grows with v^2 at fixed gamma,
so there's a specific v where it balances the fixed gravity-along-path
term) that the vehicle relaxes toward -- entering faster than that speed
means net deceleration, which is exactly the "shed speed during the glide"
behavior a real unpowered descent needs before a landing flare can ever be
reached.

Flare phase: once speed decays to FLARE_SPEED_MARGIN * stall_speed, the sim
switches to level, unpowered flight and lets speed keep bleeding off (now
mostly via induced drag, which is large this close to stall) until it
reaches stall speed -- that's the landing. This is a kinematic
simplification, not a solved flare trajectory or a real
lift-vs-angle-of-attack/round-out maneuver: altitude is treated as
~constant during the flare (a genuine landing flare loses the last little
bit of height while arresting sink rate; this model doesn't have the
low-altitude ground effect or a true round-out geometry to capture that),
so "landed" here means "reached stall speed while flying level," not
literal ground contact at h=0. If the glide instead reaches h=0 before ever
slowing enough to flare, that's reported separately (safe_landing=False)
as flying into the ground still too fast, not a proper touchdown.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, radians, sin
from typing import NamedTuple

from douglas_dart.atmosphere import standard_atmosphere

from .constants import G0_M_PER_S2, Fuel
from .drag import (WingConcept, stall_speed_concept_m_per_s,
                   stall_speed_m_per_s, total_drag_n, wing_concept_drag_n,
                   parasitic_drag_n, cd0_transonic_multiplier)
from .constants import CD0_FRONTAL
from .pulsejet_simple import pulsejet_thrust
from .ramjet_simple import ramjet_thrust

# Hard sanity floor on remaining mass -- not a real dry-mass estimate, just a
# guard against the integrator driving mass to zero/negative if a geometry
# can never reach motor cutoff. Hitting it means "this configuration cannot
# make cutoff on any plausible fuel load," which run_demo.py reports
# explicitly rather than silently returning a bogus trajectory.
MASS_FLOOR_KG = 1.0

# See module docstring: a sled/rail release speed, not a dead stop, avoids
# the induced-drag singularity at V -> 0 and matches how this vehicle class
# actually launches (this program's own reference case releases at
# 39-42 m/s).
DEFAULT_RELEASE_VELOCITY_M_PER_S = 40.0

# A severely underpowered geometry can climb at the fixed 15-degree angle
# almost indefinitely without ever reaching motor-cutoff Mach (confirmed by
# direct observation during optimizer tuning, not hypothetical) -- well
# past where douglas_dart.atmosphere.standard_atmosphere's valid range
# (-1,000 to 20,000 m) ends, which would otherwise crash the run instead of
# just reporting "infeasible." This is a numerical/domain guard, not a
# mission requirement -- treat exceeding it as equivalent to stalling.
ALTITUDE_CEILING_M = 15_000.0

# See module docstring: below this fraction of the *current* stall speed,
# the quasi-steady-lift assumption behind every drag call has broken down
# (induced drag runs away as q -> 0) regardless of phase -- treat it as a
# hard stop and report it, rather than grinding through a garbage tail end
# of the trajectory.
MINIMUM_FLIGHT_SPEED_MARGIN = 0.5

# Grace period before the check above starts applying. Needed for a real
# reason, not just to silence noise: DEFAULT_RELEASE_VELOCITY_M_PER_S is one
# fixed number, but stall speed varies a lot across the optimizer's search
# space (a small-wingspan/large-diameter candidate can have a stall speed
# well above 80 m/s), so release can legitimately start below half of some
# candidate's stall speed while still accelerating hard (confirmed by direct
# observation: T/W > 10 at release for one such candidate, immediately
# flagged "stalled" at t=0.02s with no grace period even though it was
# nowhere near the actual runaway-induced-drag failure mode this check
# exists to catch). The real failure mode this guards against takes many
# seconds to develop (a geometry decelerating back into the singularity
# over the course of a climb), so a short grace period costs nothing there.
MINIMUM_FLIGHT_SPEED_GRACE_PERIOD_S = 2.0

# Landing parameters -- deliberately just a few knobs, all easy to retune:
# the fixed unpowered-glide angle, how far above stall speed *and* how
# close to the ground the flare begins, and (via stall_speed_m_per_s's
# WING_ASPECT_RATIO/CL_MAX in constants.py) what stall speed even means for
# a given wingspan/mass/altitude. GLIDE_ANGLE_DEG is steep enough that most
# geometries actually decelerate substantially before running out of
# altitude (see module docstring) without being so steep it dives into the
# ground -- a tuned engineering choice, not a sourced value.
GLIDE_ANGLE_DEG = -5.0
FLARE_SPEED_MARGIN = 1.2
# The flare holds altitude ~constant (see module docstring), so triggering
# it purely on speed is not enough: a shallow enough glide angle can decay
# below flare-trigger speed while still hundreds of meters up, which would
# then get reported as a "safe landing" despite the vehicle being nowhere
# near the ground -- confirmed by direct observation while tuning
# GLIDE_ANGLE_DEG, not a hypothetical edge case. The flare now also
# requires altitude below this threshold; below flare-trigger speed but
# still higher than this, the sim just keeps gliding at GLIDE_ANGLE_DEG
# (still losing altitude, even if not decelerating much further) until it's
# actually close enough to the ground for "hold level and settle onto the
# runway" to be a sensible description of what's happening.
FLARE_ALTITUDE_M = 50.0

# Speed multiple of stall below which the unpowered vehicle stops holding
# altitude and starts the descending glide (see the "decel" mode branch).
# Held just above FLARE_SPEED_MARGIN so the glide phase only needs to bleed
# the last few m/s (plus altitude) before the flare window opens.
DECEL_SPEED_FACTOR = 1.25

# --- Return-to-launch profile (run_flight(return_to_launch=True)) -----------
# Instead of bleeding all the cutoff energy flying straight downrange, the
# vehicle comes home and the trajectory closes into a loop. Sequencing is
# dictated by the energy budget: at M 1.1 cutoff the KINETIC energy
# (V^2/2g ~ 7 km of equivalent height) dwarfs the actual altitude (~130 m
# on the current winner), so the reversal must happen FIRST -- a "bleed
# then turn" order spends everything going outbound and leaves nothing to
# fly home with.
#
# The reversal is a PITCH-UP HALF-LOOP (Immelmann, the roll is free in a
# point-mass model), not a level banked turn: a flat 5g turn was tried
# first and measured to waste ~80% of the cutoff energy grinding through
# transonic drag while level (landed 5.7 km short of home), whereas the
# half-loop converts speed to altitude DURING the reversal -- and altitude
# is the range currency on the way home (ground range =
# altitude/tan(RETURN_GLIDE_ANGLE)), while a level bleed converts speed to
# range at roughly 1:1 for this high-drag vehicle. Loop dynamics: constant
# load factor n, pitch rate gamma_dot = g*(n - cos(gamma))/V, exit when
# gamma reaches 180 deg (level, heading home); drag is charged the full
# n*W lift via the n*m mass argument. The quasi-steady stall-speed guard
# is suspended during the loop (over the top the vehicle is deliberately
# ballistic, like any real loop).
#
# Once back over the launch point the legacy decel/glide/flare sequence
# runs with downrange distance frozen -- a kinematic stand-in for circling
# overhead ("spiral") -- so the landing itself is the already-validated
# one. If energy runs out short of home, the normal flare lands the
# vehicle wherever it is (reported honestly by the trajectory).
RETURN_LOOP_LOAD_FACTOR = 6.0
# Matched to the airframe's actual best glide slope (~L/D 5.5-6.5 for the
# 615 mm wing at ~70 m/s): shallower angles have an equilibrium speed
# BELOW stall -- the vehicle bleeds speed all the way down and stalls
# mid-return (observed directly at -3 and -6 deg) -- while steeper wastes
# altitude that is exactly the range budget home.
RETURN_GLIDE_ANGLE_DEG = -9.0


@dataclass(frozen=True)
class VehicleGeometry:
    diameter_m: float
    throat_diameter_m: float
    chamber_length_m: float
    throat_length_m: float
    wingspan_m: float
    fuel: Fuel


class FlightState(NamedTuple):
    # Constructed once per timestep (thousands of times per flight, tens of
    # thousands of times per optimizer candidate) -- a NamedTuple builds
    # noticeably faster than a frozen dataclass (which routes every field
    # set through object.__setattr__ to stay immutable) while keeping the
    # exact same dot-attribute access every caller already uses.
    time_s: float
    altitude_m: float
    distance_m: float
    velocity_m_per_s: float
    mach: float
    mass_kg: float
    fuel_burned_kg: float
    mode: str
    thrust_n: float
    drag_n: float
    acceleration_m_per_s2: float
    thrust_to_weight: float
    specific_impulse_s: float
    stall_speed_m_per_s: float


@dataclass(frozen=True)
class FlightResult:
    states: list[FlightState]
    motor_cutoff_reached: bool
    landed: bool
    safe_landing: bool
    hit_mass_floor: bool
    stalled: bool
    crossover_mach: float | None
    min_powered_thrust_margin: float = float("inf")
    """min over powered steps of thrust/(drag + weight-along-path) -- the
    worst velocity-regime thrust margin. See the tracking comment in
    run_flight and MIN_POWERED_THRUST_MARGIN_FRACTION in constants.py."""
    min_margin_mach: float = 0.0
    """Mach at which that worst margin occurred (the mission pinch point)."""
    min_powered_accel_g: float = float("inf")
    """min over powered steps of along-path acceleration in g's -- the
    additive counterpart to the multiplicative thrust margin. Gated by
    MIN_POWERED_ACCELERATION_G in constants.py."""
    min_accel_mach: float = 0.0
    """Mach at which that worst powered acceleration occurred."""


def _concept_drag(diameter_m, wing_concept, v, rho, m, gamma_rad, mach):
    from math import cos as _c
    q = 0.5 * rho * v * v
    lift = m * G0_M_PER_S2 * _c(gamma_rad)
    body = parasitic_drag_n(diameter_m, q, CD0_FRONTAL * cd0_transonic_multiplier(mach))
    wing_par, induced = wing_concept_drag_n(wing_concept, q, lift, mach=mach)
    from .drag import DragResult
    return DragResult(body, wing_par, induced, body + wing_par + induced, lift)


def run_flight(
    geometry: VehicleGeometry,
    initial_mass_kg: float,
    climb_angle_deg: float = 15.0,
    motor_cutoff_mach: float = 1.1,
    flare_speed_margin: float = FLARE_SPEED_MARGIN,
    flare_altitude_m: float = FLARE_ALTITUDE_M,
    dt_s: float = 0.02,
    max_time_s: float = 240.0,
    initial_velocity_m_per_s: float = DEFAULT_RELEASE_VELOCITY_M_PER_S,
    wing_concept: WingConcept | None = None,
    max_fuel_burn_kg: float | None = None,
    return_to_launch: bool = False,
) -> FlightResult:
    # max_fuel_burn_kg: physical usable-fuel limit (tank capacity minus
    # reserve). Exceeding it is a FLAMEOUT: engine off wherever the flight
    # is, cutoff NOT credited. Without this cap, marginal designs could
    # hover at thrust ~= demand for hundreds of seconds and 'succeed' by
    # burning 20+ kg of phantom fuel down to the mass floor.
    # wing_concept=None keeps the legacy fixed wing (AR=3 rectangular,
    # geometry.wingspan_m) -- bit-identical to the pre-wing-optimizer sim.
    # A WingConcept replaces stall speed and the wing drag terms with the
    # concept's own closed forms (drag.py's wing-concept block); its span
    # OVERRIDES geometry.wingspan_m so the wing optimizer owns all wing
    # variables in one place.
    if return_to_launch:
        # the overhead spiral descends at the airframe's equilibrium glide
        # slope (RETURN_GLIDE_ANGLE_DEG), whose steady speed sits at
        # ~1.25x stall for this vehicle class -- the flare window must
        # open there or the spiral arrives at flare altitude a couple m/s
        # too fast and flies into the ground instead of flaring
        flare_speed_margin = max(flare_speed_margin, DECEL_SPEED_FACTOR)
    climb_angle_rad = radians(climb_angle_deg)
    sin_climb, cos_climb = sin(climb_angle_rad), cos(climb_angle_rad)
    glide_angle_rad = radians(GLIDE_ANGLE_DEG)
    sin_glide, cos_glide = sin(glide_angle_rad), cos(glide_angle_rad)
    return_glide_rad = radians(RETURN_GLIDE_ANGLE_DEG)
    sin_rglide, cos_rglide = sin(return_glide_rad), cos(return_glide_rad)

    # Local aliases for the hot loop below: geometry is fixed for the whole
    # flight, but Python attribute lookups (geometry.diameter_m) are slower
    # than local-variable reads, and these fields get read on every single
    # timestep.
    diameter_m = geometry.diameter_m
    throat_diameter_m = geometry.throat_diameter_m
    chamber_length_m = geometry.chamber_length_m
    throat_length_m = geometry.throat_length_m
    wingspan_m = geometry.wingspan_m
    fuel = geometry.fuel

    t, h, x, v, m = 0.0, 0.0, 0.0, initial_velocity_m_per_s, initial_mass_kg
    fuel_burned_kg = 0.0
    on_ramjet = False
    engine_off = False
    crossover_mach: float | None = None
    states: list[FlightState] = []
    motor_cutoff_reached = False
    landed = False
    safe_landing = False
    hit_mass_floor = False
    stalled = False
    # Worst powered-flight thrust margin: min over powered steps of
    # thrust / (drag + weight-along-path). 1.0 = exactly hanging on; below
    # 1.0 the vehicle decelerates in that regime. Feasibility gating on
    # this (optimize.py, MIN_POWERED_THRUST_MARGIN_FRACTION) is what keeps
    # designs safely clear of the pulsejet->ramjet transition pinch instead
    # of riding thrust ~= drag exactly where underperformance strands them.
    min_powered_thrust_margin = float("inf")
    min_margin_mach = 0.0
    min_powered_accel_g = float("inf")
    min_accel_mach = 0.0
    # return-to-launch state machine (see the constant block above)
    loop_gamma = 0.0        # flight-path angle swept so far in the half-loop
    heading_reversed = False
    overhead = False        # back over the launch point -> spiral down

    while True:
        atmosphere = standard_atmosphere(h)
        mach = v / atmosphere.speed_of_sound_m_per_s
        if wing_concept is None:
            stall_speed = stall_speed_m_per_s(wingspan_m, m, atmosphere.density_kg_per_m3, G0_M_PER_S2)
        else:
            stall_speed = stall_speed_concept_m_per_s(wing_concept, m, atmosphere.density_kg_per_m3, G0_M_PER_S2)

        if not engine_off and mach >= motor_cutoff_mach:
            engine_off = True
            motor_cutoff_reached = True
        if (not engine_off and max_fuel_burn_kg is not None
                and fuel_burned_kg >= max_fuel_burn_kg):
            engine_off = True  # flameout: tank dry before cutoff

        if engine_off:
            thrust_n = 0.0
            fuel_mdot_kg_per_s = 0.0
            specific_impulse_s = 0.0
            drag_mass_kg = m  # lift = 1g except during the banked turn
            if return_to_launch and not heading_reversed:
                # pitch-up half-loop: gamma sweeps 0 -> 180 deg, converting
                # speed to altitude while reversing heading (see constant
                # block); x follows cos(gamma) so the trajectory shows the
                # loop-over naturally
                mode = "loop"
                gamma_rad = loop_gamma
                sin_gamma, cos_gamma = sin(loop_gamma), cos(loop_gamma)
                x_rate = cos_gamma
                drag_mass_kg = m * RETURN_LOOP_LOAD_FACTOR
            elif return_to_launch and not overhead and not (
                    v <= flare_speed_margin * stall_speed and h <= flare_altitude_m):
                # glide home; no level-bleed on the way (range is the goal,
                # the overhead spiral does the decelerating)
                if x <= 0.0:
                    overhead = True
                mode = "return"
                gamma_rad, sin_gamma, cos_gamma = return_glide_rad, sin_rglide, cos_rglide
                x_rate = -cos_rglide
            else:
                if return_to_launch and x <= 0.0:
                    overhead = True
                direction = 0.0 if overhead else (-1.0 if return_to_launch else 1.0)
                if v <= flare_speed_margin * stall_speed and h <= flare_altitude_m:
                    mode = "flare"
                    gamma_rad, sin_gamma, cos_gamma = 0.0, 0.0, 1.0
                    x_rate = direction
                elif v > DECEL_SPEED_FACTOR * stall_speed and h > flare_altitude_m:
                    # Level deceleration segment (2026-08-12): a fixed -5 deg
                    # glide from supersonic cutoff reaches the ground at ~200 m/s
                    # -- gravity feeds back most of what drag removes (the
                    # already-documented "glide angle doesn't decelerate"
                    # limitation). Holding altitude until speed decays below
                    # DECEL_SPEED_FACTOR x stall lets drag do the work first
                    # (kinematically standing in for S-turns / speed brakes),
                    # then the normal descending glide + flare take over. Same
                    # closed-form force evaluations, one extra branch. In the
                    # return profile this same branch runs with x frozen --
                    # circling overhead the landing site ("spiral").
                    mode = "spiral" if overhead else "decel"
                    gamma_rad, sin_gamma, cos_gamma = 0.0, 0.0, 1.0
                    x_rate = direction
                elif overhead:
                    # descending spiral at the equilibrium glide slope --
                    # -5 deg bleeds through stall over a long descent
                    # (observed), the steeper return slope holds speed
                    mode = "spiral"
                    gamma_rad, sin_gamma, cos_gamma = return_glide_rad, sin_rglide, cos_rglide
                    x_rate = 0.0
                else:
                    mode = "glide"
                    gamma_rad, sin_gamma, cos_gamma = glide_angle_rad, sin_glide, cos_glide
                    x_rate = direction * cos_glide
            if wing_concept is None:
                drag_result = total_drag_n(
                    diameter_m, wingspan_m, v, atmosphere.density_kg_per_m3,
                    drag_mass_kg, gamma_rad, G0_M_PER_S2, mach=mach,
                )
            else:
                drag_result = _concept_drag(
                    diameter_m, wing_concept, v, atmosphere.density_kg_per_m3,
                    drag_mass_kg, gamma_rad, mach,
                )
        else:
            # BOTH engines run through the transition (2026-08-12): the
            # pulsejet keeps pulsing while the ramjet duct lights -- the
            # first-principles model shows the side-inlet pulsejet operating
            # (declining) all the way to M 0.9, and the ramjet lightoff gate
            # (ramjet_simple.RAMJET_MIN_LIGHTOFF_MACH) already zeroes the
            # ramjet below its viable speed. The old either/or switch made
            # the transition gap artificially lethal: total thrust dipped to
            # a single engine exactly where drag peaks. Total = simple sum;
            # crossover_mach still reports where the ramjet first dominates.
            ramjet_result = ramjet_thrust(
                diameter_m, throat_diameter_m, mach, h, fuel, atmosphere=atmosphere
            )
            pulsejet_result = pulsejet_thrust(
                diameter_m, chamber_length_m, throat_diameter_m, throat_length_m, mach, h, fuel,
                atmosphere=atmosphere,
            )
            if not on_ramjet and ramjet_result.net_thrust_n > pulsejet_result.average_thrust_n:
                on_ramjet = True
                crossover_mach = mach

            thrust_n = pulsejet_result.average_thrust_n + ramjet_result.net_thrust_n
            fuel_mdot_kg_per_s = (
                pulsejet_result.fuel_mass_flow_kg_per_s
                + ramjet_result.fuel_mass_flow_kg_per_s
            )
            weight_flow = fuel_mdot_kg_per_s * G0_M_PER_S2
            specific_impulse_s = thrust_n / weight_flow if weight_flow > 0.0 else 0.0
            mode = "ramjet" if on_ramjet else "pulsejet"
            sin_gamma, cos_gamma = sin_climb, cos_climb
            x_rate = cos_climb
            if wing_concept is None:
                drag_result = total_drag_n(
                    diameter_m, wingspan_m, v, atmosphere.density_kg_per_m3, m,
                    climb_angle_rad, G0_M_PER_S2, mach=mach,
                )
            else:
                drag_result = _concept_drag(
                    diameter_m, wing_concept, v, atmosphere.density_kg_per_m3,
                    m, climb_angle_rad, mach,
                )

        weight_n = m * G0_M_PER_S2
        thrust_to_weight = thrust_n / weight_n
        weight_along_path_n = weight_n * sin_gamma
        acceleration_m_per_s2 = (thrust_n - drag_result.total_n - weight_along_path_n) / m
        if not engine_off:
            demand_n = drag_result.total_n + weight_along_path_n
            if demand_n > 1e-9:
                margin = thrust_n / demand_n
                if margin < min_powered_thrust_margin:
                    min_powered_thrust_margin = margin
                    min_margin_mach = mach
            accel_g = acceleration_m_per_s2 / G0_M_PER_S2
            if accel_g < min_powered_accel_g:
                min_powered_accel_g = accel_g
                min_accel_mach = mach

        states.append(
            FlightState(
                t, h, x, v, mach, m, fuel_burned_kg, mode, thrust_n, drag_result.total_n,
                acceleration_m_per_s2, thrust_to_weight, specific_impulse_s, stall_speed,
            )
        )

        if mode == "flare" and v <= stall_speed:
            landed = True
            safe_landing = True
            break
        if engine_off and h <= 0.0 and t > 0.0:
            landed = True
            safe_landing = False
            break
        if (v < MINIMUM_FLIGHT_SPEED_MARGIN * stall_speed
                and t > MINIMUM_FLIGHT_SPEED_GRACE_PERIOD_S
                and mode != "loop"):
            # the loop is deliberately ballistic over the top -- the
            # quasi-steady stall guard doesn't apply mid-maneuver
            stalled = True
            break
        if h >= ALTITUDE_CEILING_M:
            stalled = True
            break
        if m <= MASS_FLOOR_KG:
            hit_mass_floor = True
            break
        if t >= max_time_s:
            break

        v = max(v + acceleration_m_per_s2 * dt_s, 0.0)
        h = max(h + v * sin_gamma * dt_s, 0.0)
        x = x + v * x_rate * dt_s
        if mode == "loop":
            loop_gamma += (G0_M_PER_S2
                           * (RETURN_LOOP_LOAD_FACTOR - cos(loop_gamma))
                           / max(v, 1.0)) * dt_s
            if loop_gamma >= pi:
                heading_reversed = True
        fuel_step_kg = fuel_mdot_kg_per_s * dt_s
        fuel_burned_kg += fuel_step_kg
        m = max(m - fuel_step_kg, MASS_FLOOR_KG)
        t += dt_s

    return FlightResult(
        states, motor_cutoff_reached, landed, safe_landing, hit_mass_floor,
        stalled, crossover_mach, min_powered_thrust_margin, min_margin_mach,
        min_powered_accel_g, min_accel_mach,
    )
