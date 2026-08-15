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
from math import asin, cos, pi, radians, sin, sqrt, tan
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

# Overhead spiral only: the level bleed must hand off to the speed-holding
# equilibrium descent already INSIDE the flare window, not at its edge. The
# equilibrium spiral holds whatever speed it inherits, so if that speed sits
# above FLARE_SPEED_MARGIN the flare never triggers and the vehicle rides a
# perfectly good glide into the ground (observed: arrived at h=0 holding
# 55.9 m/s against a 55.6 m/s trigger -- 0.3 m/s short). Held below
# FLARE_SPEED_MARGIN so the handoff always lands inside the window.
SPIRAL_BLEED_SPEED_FACTOR = 1.15

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
# --- V3 climb-dive profile (run_flight(climb_dive=ClimbDiveProfile(...))) ---
# The pulsejet->ramjet thrust deficit is a narrow NOTCH at lightoff, not a
# broad valley: level acceleration measured on the V2 winner runs +0.32 g at
# M 0.40, dips to +0.21 g at M 0.44, and is back to +0.96 g by M 0.50. The
# V3 idea (user, 2026-08-12) is to bank the pulsejet's surplus low-Mach
# thrust as ALTITUDE, then spend it as gravity through that notch:
# a_gravity/g = sin(dive), so a 10 deg dive adds +0.17 g exactly where the
# engine is weakest. Measured trade crossing M 0.35 -> 0.60:
#     dive   min accel   altitude spent
#      0 deg   +0.13 g        0 ft
#      5 deg   +0.23 g      650 ft
#     10 deg   +0.33 g     1050 ft
#     15 deg   +0.42 g     1340 ft
# The prize is not the acceleration itself but a SMALLER ENGINE: the
# min-acceleration gate is what forced V2's peak T/W from 6.89 to 10.08, and
# a 10/15 deg dive lets the same gate pass with 11%/24% less thrust.
#
# Two measured facts shape the profile:
#   * Being LOW is good for the ramjet -- at M 1.05 the vehicle makes
#     +3.21 g at 500 ft vs +2.65 g at 5000 ft (ramjet thrust scales with
#     density and beats the drag rise), so the post-dive "drag strip" run
#     wants to be near the floor, not up high.
#   * Climbing costs THRUST, not just fuel -- the same engine makes +0.38 g
#     at 300 m and +0.32 g at 600 m (M 0.40). Roughly a third of the dive's
#     benefit is paid back as a climb tax, so a lower top with a steeper
#     dive beats a gentle dive from high up.
#
# Competition rule (user, 2026-08-12): flight path angle must be >= 0 from
# M 0.80 all the way through cutoff. It does not bind -- the vehicle is at
# +2.3 g by M 0.60 and pulls out long before M 0.80 -- but violations are
# tracked and reported rather than assumed away.
V3_FLOOR_ALTITUDE_M = 400.0 * 0.3048      # 121.9 m -- user-specified hard floor
V3_DIVE_START_MACH = 0.35
V3_DIVE_END_MACH = 0.60
V3_RULE_MACH_LO = 0.80
V3_RULE_MACH_HI = 1.10
# Cap on the DERIVED top-of-climb. A shallow dive behind a weak engine
# traverses the Mach band so slowly that drop = distance * sin(dive) runs
# away (it fed standard_atmosphere an out-of-range altitude and crashed the
# optimizer). Capping is the honest response: such a candidate simply never
# reaches its top, runs out of altitude ceiling or time, and is reported
# infeasible -- rather than the sizing pass exploding.
V3_MAX_TOP_ALTITUDE_M = 4000.0

# --- V4 pitch arcs and body loading (2026-08-13, user requirement) ---------
# V3 switched flight path angle from climb to dive, and from dive to the drag
# strip, in a SINGLE TIMESTEP -- no arc, no radius, no load factor anywhere in
# the phase machine (docs/v3_learnings_for_v4.md section 3.6 flags this as an
# unmodelled gap; load factor never exceeded 1 in any V3 flight). V4 flies
# both transitions as real constant-load-factor circular arcs:
#
#   gamma_dot = g0 * (n - cos gamma) / V        turn rate
#   R         = V / |gamma_dot| = V^2 / (g0 |n - cos gamma|)   arc radius
#
# and the altitude the pull-out arc costs is charged against the hard floor:
# the dive now has to END high enough that the arc BOTTOMS OUT at the floor,
# rather than the old behaviour of diving to the floor and teleporting level.
# For a pull-up from -theta to level, dh/dgamma = R sin(gamma), so the drop is
# R*(1 - cos theta) -- ~20 m at 3 g for V3's 9.89 deg dive at M 0.48.
#
# PUSHOVER at 0 g (user choice, 2026-08-13): the fastest altitude-neutral
# nose-over that never unloads the airframe in reverse. Propane feed at 0 g is
# a flagged hardware caveat, not a modelled violation.
# PULL-OUT at 3 g (user choice): 667 N (150 lbf) through the wing joint on a
# 22.68 kg airframe, far under the ~15 g aerodynamic CL_max ceiling. Nothing
# in this repo models structure, so this is a DESIGN LIMIT, not a capability.
V4_PUSHOVER_LOAD_FACTOR = 0.0
V4_PULLOUT_LOAD_FACTOR = 3.0
# The floor is a HARD constraint and the load factor is what gives.
#
# A fixed-g arc triggered off a predicted drop does not work here, and the
# failure is not subtle: the ramjet lights mid-dive, so the vehicle is
# accelerating hard THROUGH the pull-out, the radius runs larger than
# whatever was predicted at entry, and the arc bottoms out below the floor.
# Measured directly -- 632 of 1350 campaign flights busted the floor with a
# 1.15 safety factor on the predicted drop.
#
# So the pull-out is flown the way it would actually be flown: hold the
# nominal load factor, and if the arc is no longer going to make the floor,
# pull harder -- up to a hard limit. The dive therefore runs until the
# NOMINAL-g arc can just barely still make the floor (which is the longest
# legal dive, i.e. the most Mach available for lightoff), and any shortfall
# after that shows up as load factor, which is a reported output. If even
# the limit cannot hold the floor, floor_violated says so.
V4_PULLOUT_MAX_LOAD_FACTOR = 6.0
# Explicit-Euler discretization allowance on the floor check. A dt=0.02 s step
# at ~190 m/s covers ~3.8 m of flight path, so an arc commanded to bottom out
# exactly ON the floor lands a few millimetres either side of it -- measured
# 121.9078 m against a 121.92 m floor, a 12 mm shortfall that an exact-1e-6
# tolerance reported as a floor bust on 561 of 1350 flights. The flown minimum
# is reported as min_powered_altitude_m regardless, so this hides nothing.
# SPIRAL CLIMB (user, 2026-08-13). V4 wants a high top of climb -- that is
# the one lever that actually moves the dive-exit Mach -- but a straight
# climb to 1000 m at 8 deg spends 7.1 km of GROUND track getting there, and
# the return-to-launch glide then lands 4-6 km short of home (measured: every
# configuration in the campaign box failed its landing for this reason and
# this reason only; the powered mission itself closed).
#
# A helical climb over the launch point fixes it exactly: identical air path,
# speed, flight path angle, drag and fuel -- only the ground track changes,
# from a straight line to a circle. The point-mass model represents that by
# zeroing the downrange rate, which is the same device the unpowered "spiral"
# mode already uses.
#
# What a spiral is NOT free of, and is charged here: the bank needed to turn.
# A climbing turn at bank phi carries load factor n = cos(gamma)/cos(phi),
# which feeds the maneuvering-lift path and therefore the induced drag, and
# shows up in the body-load trace. 30 deg is a standard climbing-turn bank
# (n = 1.15). What is NOT modelled: the roll-in/roll-out, and the heading
# alignment at the top before the pushover.
V4_SPIRAL_BANK_DEG = 30.0

V4_FLOOR_TOLERANCE_M = 1.0
# 1 - cos(1.5 deg): below this the pull-out arc is finished for all practical
# purposes (the altitude still to be lost getting from 1.5 deg to level is
# ~0.3 m at 190 m/s and 3 g), so the floor-holding load demand is switched off
# to avoid the 0/0 at the bottom of the arc. See required_pullout_load_factor.
_PULLOUT_ARC_DONE_SHAPE = 3.4e-4
# Guard on the 1/cos(gamma) in the maneuvering-lift conversion below.
_MIN_COS_GAMMA = 0.1

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


@dataclass(frozen=True)
class ClimbDiveProfile:
    """V3 powered trajectory: climb steeply on surplus low-Mach thrust to
    top_altitude_m, push over and dive through the lightoff notch, pull out
    at floor_altitude_m, then run the level-ish "drag strip" to cutoff.

    top_altitude_m is normally DERIVED (derive_top_altitude) from how much
    altitude the dive actually spends crossing the notch -- the user's "mgh
    sets how high we need to climb" framing -- rather than searched
    independently, which keeps the three phases self-consistent."""

    initial_climb_angle_deg: float
    dive_angle_deg: float                 # positive; flown as -angle
    floor_altitude_m: float = V3_FLOOR_ALTITUDE_M
    dive_start_mach: float = V3_DIVE_START_MACH
    dive_end_mach: float = V3_DIVE_END_MACH
    top_altitude_m: float | None = None
    # V4 pitch arcs. pullout_load_factor=None keeps the V3 behaviour exactly
    # (one-timestep gamma switch, no arc, no radius) so docs/v3_frozen re-flies
    # bit-identical; set it to a load factor to fly both transitions as arcs.
    pullout_load_factor: float | None = None
    pushover_load_factor: float = V4_PUSHOVER_LOAD_FACTOR
    pullout_max_load_factor: float = V4_PULLOUT_MAX_LOAD_FACTOR
    # Spiral (helical) climb: same air path, same speed, same flight path
    # angle, same fuel -- but the GROUND track is a circle, so the climb
    # spends no net downrange. See the V4 constant block.
    spiral_climb: bool = False
    spiral_bank_deg: float = V4_SPIRAL_BANK_DEG

    @property
    def uses_arcs(self) -> bool:
        return self.pullout_load_factor is not None


@dataclass(frozen=True)
class RamjetStart:
    """When to light the ramjet -- a POLICY, not a scalar constant.

    docs/v3_learnings_for_v4.md section 3.3: a Mach gate's consequence is
    positional, because it decides *where in the trajectory* the ramjet
    lights, and the same gate is worth 0.052 g or 0.331 g depending only on
    whether it fires mid-climb or just past the top. A bare
    RAMJET_MIN_LIGHTOFF_MACH cannot express "light it in the dive."

    gate_mach          -- overrides RAMJET_MIN_LIGHTOFF_MACH for this flight.
    require_descending -- veto the light while the flight path angle is still
                          positive, i.e. the ramjet can only come alive once
                          the vehicle has pushed over (user requirement,
                          2026-08-13: V4 lights the ramjet in the dive).
    Once lit, it stays lit -- a real flameholder does not blow out because
    the vehicle pulled level again.

    medium_model note: with ``propulsion=None`` (the closed-form engines)
    this policy DECIDES the light, exactly as in simple_model. With an
    ``FpPropulsion``, lightoff is a first-principles event, so the policy
    decides only when the ramjet is ASKED to cold-light -- the FP model
    keeps the right to refuse, and the report says whether it did (user
    decision, 2026-08-13)."""

    gate_mach: float
    require_descending: bool = True
    light_at_pullout: bool = False
    """Last-chance override (user, 2026-08-13): if the ramjet is still unlit
    when the PULL-OUT begins, light it there regardless of Mach.

    Why it exists. The gate is a *goal*, not a physical threshold -- the dive
    is supposed to deliver `gate_mach` and hand a lit engine to the drag
    strip. If the dive under-delivers, the pure-gate policy does the worst
    possible thing: it keeps waiting for a Mach the vehicle is never going to
    see unpowered, and the vehicle coasts to a stop with an unlit ramjet
    (medium_model rung C, 2026-08-13: peak Mach 0.488 against a 0.50 gate,
    ramjet never even asked, whole tank burnt at M ~0.44). The pull-out is
    the last moment the decision still matters, so it is where the gate is
    waived.

    LATCHED once the pull-out starts, so the waiver survives into the drag
    strip -- otherwise the Mach gate would re-arm and put the engine out
    again the instant it was lit.

    What it does NOT do: it does not make the ramjet light. Against the
    closed-form engine it does (that engine has no opinion beyond the gate).
    Against `FpPropulsion` it only means the ramjet is ASKED at the pull-out
    instead of never -- the first-principles model keeps the right to refuse,
    and `FlightResult.ramjet_light_refused` records it if it does.

    Defaults to False, so every V2/V3/V4 re-fly is unchanged."""


def pullout_arc_drop_m(
    velocity_m_per_s: float,
    dive_angle_deg: float,
    load_factor: float,
) -> float:
    """Altitude a constant-load-factor pull-up from -dive_angle to level
    costs: R*(1 - cos theta), R = V^2/(g0*(n - cos gamma)).

    cos(gamma) is taken as 1 (its value at the top of the arc, where the
    turn is loosest) rather than cos(dive_angle) -- that is the conservative
    end of the arc, so the drop is never under-estimated. Single closed-form
    expression, no integration."""
    if dive_angle_deg <= 0.0:
        return 0.0
    n = max(load_factor, 1.05)   # n <= 1 cannot pull out at all
    radius_m = velocity_m_per_s * velocity_m_per_s / (G0_M_PER_S2 * (n - 1.0))
    return radius_m * (1.0 - cos(radians(dive_angle_deg)))


def required_pullout_load_factor(
    velocity_m_per_s: float,
    flight_path_angle_rad: float,
    altitude_m: float,
    floor_altitude_m: float,
) -> float:
    """The load factor whose arc bottoms out EXACTLY at the floor from the
    state given -- the inverse of pullout_arc_drop_m.

    Setting R*(1 - cos gamma) equal to the altitude still in hand and solving
    n = 1 + V^2/(g0*R). cos(gamma) is taken as 1 at the top of the arc, the
    same conservative end used in pullout_arc_drop_m, so the two agree.
    Returns inf once the floor is already gone (nothing can hold it)."""
    drop_shape = 1.0 - cos(flight_path_angle_rad)
    if drop_shape <= _PULLOUT_ARC_DONE_SHAPE:
        # Effectively level -- there is no arc left to fly, so there is
        # nothing left to demand. Guarding on this matters: at the bottom of
        # the arc the angle and the remaining altitude go to zero TOGETHER,
        # and the 0/0 spiked the commanded load factor to the 6 g limiter on
        # the final step of every flight while the load actually carried
        # through the arc was ~3.4 g. That artifact is what the peak-load
        # gate was reading.
        return 1.0
    remaining_m = altitude_m - floor_altitude_m
    if remaining_m <= 0.0:
        return float("inf")
    radius_m = remaining_m / drop_shape
    return 1.0 + velocity_m_per_s * velocity_m_per_s / (G0_M_PER_S2 * radius_m)


def derive_top_altitude(
    geometry: VehicleGeometry,
    wing_concept: WingConcept | None,
    mass_kg: float,
    profile: ClimbDiveProfile,
    n_steps: int = 48,
) -> float:
    """Closed-form sizing: march the dive Mach band at the dive angle and
    integrate the altitude it spends, so top = floor + that. Two fixed
    passes (not a convergence loop) -- the first estimates the drop at the
    floor's density, the second re-runs it at the resulting mid-dive
    altitude, which is where the air actually is.

    Returns the floor unchanged for a zero/negative dive angle, and caps the
    result if the dive cannot be sustained (the flight itself still enforces
    the floor, so an under-estimate is safe, not silently wrong)."""
    sin_dive = sin(radians(profile.dive_angle_deg))
    if sin_dive <= 0.0 or profile.dive_end_mach <= profile.dive_start_mach:
        return profile.floor_altitude_m

    # V4: the dive no longer ends AT the floor -- it ends where the pull-out
    # arc can still bottom out at the floor, so the arc's drop is part of the
    # altitude the climb has to buy. Sized at the dive-exit speed evaluated
    # at the floor (the fastest, hence deepest-arcing, case).
    base_altitude_m = profile.floor_altitude_m
    if profile.uses_arcs:
        floor_atmosphere = standard_atmosphere(profile.floor_altitude_m)
        base_altitude_m += pullout_arc_drop_m(
            profile.dive_end_mach * floor_atmosphere.speed_of_sound_m_per_s,
            profile.dive_angle_deg, profile.pullout_load_factor,
        )

    max_drop = max(V3_MAX_TOP_ALTITUDE_M - base_altitude_m, 0.0)
    step = (profile.dive_end_mach - profile.dive_start_mach) / n_steps
    drop = 0.0
    for _pass in range(2):
        reference_alt = min(base_altitude_m + 0.5 * drop,
                            V3_MAX_TOP_ALTITUDE_M)
        drop = 0.0
        for i in range(n_steps):
            mach = profile.dive_start_mach + step * i
            atmosphere = standard_atmosphere(reference_alt)
            a_sound = atmosphere.speed_of_sound_m_per_s
            v = mach * a_sound
            thrust_n = (
                pulsejet_thrust(geometry.diameter_m, geometry.chamber_length_m,
                                geometry.throat_diameter_m, geometry.throat_length_m,
                                mach, reference_alt, geometry.fuel,
                                atmosphere=atmosphere).average_thrust_n
                + ramjet_thrust(geometry.diameter_m, geometry.throat_diameter_m,
                                mach, reference_alt, geometry.fuel,
                                atmosphere=atmosphere).net_thrust_n
            )
            if wing_concept is None:
                drag_n = total_drag_n(
                    geometry.diameter_m, geometry.wingspan_m, v,
                    atmosphere.density_kg_per_m3, mass_kg, -radians(profile.dive_angle_deg),
                    G0_M_PER_S2, mach=mach).total_n
            else:
                drag_n = _concept_drag(
                    geometry.diameter_m, wing_concept, v,
                    atmosphere.density_kg_per_m3, mass_kg,
                    -radians(profile.dive_angle_deg), mach).total_n
            accel = (thrust_n - drag_n) / mass_kg + G0_M_PER_S2 * sin_dive
            if accel <= 0.0:
                # cannot sustain the dive band; fall back to whatever the
                # partial integration bought (the floor still binds in flight)
                return base_altitude_m + min(drop, max_drop)
            dv = step * a_sound
            dt = dv / accel
            drop += (v + 0.5 * dv) * dt * sin_dive
            if drop >= max_drop:
                return V3_MAX_TOP_ALTITUDE_M
    return profile.floor_altitude_m + min(drop, max_drop)


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
    # --- V4 additions (defaulted, so nothing that reads the first 14 fields
    # positionally changes) -------------------------------------------------
    flight_path_angle_rad: float = 0.0
    load_n_roll: float = 0.0
    """Body ROLL-axis (axial) specific force in g: (T - D)/(m*g0). This is
    what an accelerometer on the longitudinal axis reads -- gravity is NOT
    included, which is why it differs from acceleration_m_per_s2 (that one
    does include the weight-along-path term). 2-D trajectory, so roll and yaw
    are the only two axes carrying load."""
    load_n_yaw: float = 0.0
    """Body YAW-axis (normal, in the trajectory plane) load factor:
    cos(gamma) + V*gamma_dot/g0. Equals the commanded load factor during a
    pitch arc and cos(gamma) on any straight leg."""
    load_n_total: float = 0.0
    """sqrt(roll^2 + yaw^2) -- total specific force the airframe carries."""
    turn_radius_m: float = 0.0
    """Instantaneous pitch-arc radius V/|gamma_dot|; 0.0 on a straight leg."""


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
    min_traverse_accel_g: float = float("inf")
    """min acceleration over powered steps EXCLUDING the commanded V3 climb
    -- i.e. over the regimes the vehicle must actually get *through* (the
    lightoff notch and the drag strip). This is what the acceleration gate
    tests; see the reasoning in run_flight where it is tracked. Identical to
    min_powered_accel_g for any non-V3 flight."""
    min_traverse_accel_mach: float = 0.0
    climb_dive_top_altitude_m: float | None = None
    """V3 only: the derived top-of-climb altitude actually flown."""
    rule_violated: bool = False
    """V3 rule: True if the flight path angle went NEGATIVE anywhere in
    V3_RULE_MACH_LO..HI. Gated by the optimizer; see the V3 constant block."""
    rule_violation_mach: float | None = None

    # --- V4: pitch arcs, body loading, ramjet start -------------------------
    peak_load_n_total: float = 0.0
    peak_load_n_yaw: float = 0.0
    peak_load_n_roll: float = 0.0
    peak_load_mode: str = ""
    """Phase in which the worst total body load occurred."""
    pushover_radius_m: float = 0.0
    pullout_radius_m: float = 0.0
    spiral_radius_m: float = 0.0
    """Ground-track turn radius of the spiral climb, 0.0 if not spiralling.
    The two pitch radii are the representative (entry) values, 0.0 when arcs
    are disabled."""
    pushover_duration_s: float = 0.0
    pullout_duration_s: float = 0.0
    pullout_entry_altitude_m: float = 0.0
    pullout_entry_mach: float = 0.0
    min_powered_altitude_m: float = float("inf")
    """Lowest altitude reached under power -- the MEASURED bottom of the
    pull-out arc, which is what the 400 ft floor is actually checked against
    (the trigger only predicts it)."""
    floor_violated: bool = False
    dive_exit_mach: float = 0.0
    """Mach at the moment the dive hands off to the pull-out. With a Mach
    gate for the ramjet, the headroom between the gate and THIS is the whole
    margin -- see docs/v3_learnings_for_v4.md section 4.1 on why 'can it
    reach X' must never be trusted from a closed-form screen alone."""
    ramjet_lightoff_mach: float | None = None
    ramjet_lightoff_altitude_m: float | None = None
    ramjet_lightoff_time_s: float | None = None
    ramjet_lightoff_mode: str = ""
    ramjet_lit_in_dive: bool = False
    """True only if the ramjet came alive during the pushover or the dive --
    the V4 requirement (user, 2026-08-13). A gate that fires in the climb or
    after the pull-out satisfies the Mach test and fails the mission."""
    ramjet_light_refused: bool = False
    """FP path only: the ramjet was ASKED to light inside the policy window
    and the first-principles model declined. Never set on the closed-form
    path, where the policy decides the light outright."""
    min_pushover_accel_g: float = float("inf")
    """Reported separately rather than folded into the traverse figure, so
    excluding the commanded pushover from the gate cannot hide a problem."""


def _concept_drag(diameter_m, wing_concept, v, rho, m, gamma_rad, mach):
    from math import cos as _c
    q = 0.5 * rho * v * v
    lift = m * G0_M_PER_S2 * _c(gamma_rad)
    body = parasitic_drag_n(diameter_m, q, CD0_FRONTAL * cd0_transonic_multiplier(mach))
    wing_par, induced = wing_concept_drag_n(wing_concept, q, lift, mach=mach)
    from .drag import DragResult
    return DragResult(body, wing_par, induced, body + wing_par + induced, lift)


_COLD_DUCT_FLOW_COEFFICIENT = 0.90
"""An unlit duct is a straight-through pipe: it passes very nearly what
the lip offers (minus internal losses), so it barely spills. Spillage is
what a HOT, restrictive engine does."""


def _duct_swallowed_kg_per_s(ramjet_result, rho, v, lip_area_m2):
    """Mass flow actually entering the duct, for the spillage term.

    NOTE (real inconsistency between the two models, found 2026-08-13):
    ``ramjet_simple`` defines its capture area as the vehicle's FULL
    FRONTAL area (pi D_body^2 / 4), i.e. it assumes the whole nose is
    inlet. The physical duct is the throat diameter -- 0.0179 m^2 vs
    0.0616 m^2, a 3.4x disagreement -- so its reported
    ``captured_air_mass_flow_kg_per_s`` is fictitious and its implied
    ~90% spillage is largely an artifact. Engine THRUST is unaffected
    (it is throat-limited either way, below both capture figures), but
    every spillage-derived force must use the real lip area, so this
    helper deliberately ignores the engine's capture figure.
    """
    lit = ramjet_result.net_thrust_n > 0.0
    if lit:
        return ramjet_result.air_mass_flow_kg_per_s
    return _COLD_DUCT_FLOW_COEFFICIENT * rho * v * lip_area_m2


def _buildup_drag(geom_cache, wing_concept, v, rho, temperature_k, m,
                  gamma_rad, mach, engine_on, captured_mdot_kg_per_s):
    """medium_model step-1 drag: component build-up instead of the flat
    CD0 placeholder (see drag_buildup.py). Returns the same DragResult
    shape so the integrator is untouched, with the build-up's body terms
    (friction + form + base + wave + spillage) collapsed into the
    'parasitic' slot and the full breakdown carried alongside."""
    from math import cos as _c

    from .drag import DragResult
    from .drag_buildup import total_drag_buildup

    lift = m * G0_M_PER_S2 * _c(gamma_rad)
    b = total_drag_buildup(
        diameter_m=geom_cache["diameter_m"],
        body_length_m=geom_cache["body_length_m"],
        duct_exit_diameter_m=geom_cache["duct_exit_diameter_m"],
        tail_length_m=geom_cache["tail_length_m"],
        wing_reference_area_m2=geom_cache["wing_area_m2"],
        wing_thickness_ratio=geom_cache["wing_thickness_ratio"],
        wing_sweep_deg=geom_cache["wing_sweep_deg"],
        velocity_m_per_s=v, density_kg_per_m3=rho,
        temperature_k=temperature_k, mach=mach, required_lift_n=lift,
        oswald_efficiency=geom_cache["oswald_e"],
        wing_aspect_ratio=geom_cache["aspect_ratio"],
        engine_on=engine_on,
        captured_mass_flow_kg_per_s=captured_mdot_kg_per_s,
        lip_area_m2=geom_cache["lip_area_m2"],
        cowl_suction_recovery=geom_cache["cowl_suction_recovery"],
        fin_area_m2=geom_cache["fin_area_m2"],
        fin_aspect_ratio=geom_cache["fin_aspect_ratio"],
        fin_thickness_ratio=geom_cache["fin_thickness_ratio"],
        fin_sweep_deg=geom_cache["fin_sweep_deg"],
    )
    body_terms = b.friction_n + b.form_n + b.base_n + b.wave_n + b.spillage_n
    # The fin term rides with the wing's profile drag: same physical kind of
    # term, and the integrator only consumes the total anyway.
    return DragResult(body_terms, b.wing_profile_n + b.fin_profile_n,
                      b.induced_n, b.total_n, lift), b


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
    climb_dive: ClimbDiveProfile | None = None,
    ramjet_start: RamjetStart | None = None,
    drag_model: str = "legacy",
    cowl_suction_recovery: float | None = None,
    propulsion: object | None = None,
    fin: dict | None = None,
) -> FlightResult:
    # fin (2026-08-14): tail/fin surfaces, which this repo otherwise does not
    # have at all. None = no fins, i.e. every flight flown before this existed
    # -- including the V4 freeze -- is bit-identical. Pass
    # {"area_m2": .., "aspect_ratio": .., "thickness_ratio": .., "sweep_deg":..}
    # to charge their profile drag. This exists because VSPAERO found the
    # airframe statically unstable in pitch AND yaw, and the fix is fin area
    # the trajectory model cannot currently see the cost of. Only drag is
    # modelled here: fin MASS does not affect the trajectory at all, because
    # the vehicle always launches at the fixed 50 lb wet mass -- fin mass comes
    # straight out of payload margin instead. Requires drag_model="buildup".
    # ramjet_start (V4): policy object deciding WHEN the ramjet lights, not
    # just at what Mach -- see RamjetStart. None keeps the pre-V4 behaviour
    # (bare RAMJET_MIN_LIGHTOFF_MACH, lights wherever it happens to fire),
    # which is what docs/v2_frozen and docs/v3_frozen re-fly under.
    # propulsion=None uses the ancestor's closed-form engines (parity).
    # Pass a medium_model.fp_propulsion.FpPropulsion to march the
    # first-principles engines ALONG this trajectory instead -- thrust and
    # fuel then come from transient simulations at the real flight
    # condition, and the ramjet's lightoff becomes a computed event rather
    # than the configured RAMJET_MIN_LIGHTOFF_MACH gate.
    # cowl_suction_recovery=None uses the geometry/Mach-based value derived
    # in drag_buildup.cowl_suction_recovery_fn; pass a float to override
    # (the campaign brackets it, since it is the single largest remaining
    # uncertainty in the drag model).
    # drag_model: "legacy" reproduces the ancestor EXACTLY (flat
    # CD0_FRONTAL x transonic multiplier) and is what the V2 parity test
    # asserts; "buildup" switches to medium_model's component build-up
    # (drag_buildup.py) -- step 1 of the fidelity ladder. The switch exists
    # so the parity baseline stays re-runnable forever, and so the drag
    # delta is MEASURED by flying one design both ways rather than argued.
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
    climb_angle_rad = radians(climb_angle_deg)
    sin_climb, cos_climb = sin(climb_angle_rad), cos(climb_angle_rad)
    glide_angle_rad = radians(GLIDE_ANGLE_DEG)
    sin_glide, cos_glide = sin(glide_angle_rad), cos(glide_angle_rad)
    return_glide_rad = radians(RETURN_GLIDE_ANGLE_DEG)
    sin_rglide, cos_rglide = sin(return_glide_rad), cos(return_glide_rad)

    # V3 climb-dive setup (no-op when climb_dive is None, which keeps every
    # pre-V3 flight bit-identical -- see tests/test_v2_frozen.py)
    v3_top_altitude_m: float | None = None
    if climb_dive is not None:
        v3_climb_rad = radians(climb_dive.initial_climb_angle_deg)
        sin_v3climb, cos_v3climb = sin(v3_climb_rad), cos(v3_climb_rad)
        v3_dive_rad = -radians(climb_dive.dive_angle_deg)
        sin_v3dive, cos_v3dive = sin(v3_dive_rad), cos(v3_dive_rad)
        v3_top_altitude_m = (
            climb_dive.top_altitude_m
            if climb_dive.top_altitude_m is not None
            else derive_top_altitude(geometry, wing_concept, initial_mass_kg,
                                     climb_dive)
        )
    v3_climb_done = False
    v3_dive_done = False
    rule_violated = False
    rule_violation_mach: float | None = None

    # V4 arc state. v4_arcs is False for every V2/V3 flight (pullout_load_factor
    # defaults to None), in which case the phase machine below reduces exactly
    # to V3's two-boolean latch.
    v4_arcs = climb_dive is not None and climb_dive.uses_arcs
    v4_phase = "climb"
    v4_gamma_rad = radians(climb_dive.initial_climb_angle_deg) if v4_arcs else 0.0
    strip_gamma_rad = climb_angle_rad
    peak_load_n_total = peak_load_n_yaw = peak_load_n_roll = 0.0
    peak_load_mode = ""
    pushover_radius_m = pullout_radius_m = spiral_radius_m = 0.0
    pushover_duration_s = pullout_duration_s = 0.0
    pullout_entry_altitude_m = pullout_entry_mach = 0.0
    min_powered_altitude_m = float("inf")
    dive_exit_mach = 0.0
    min_pushover_accel_g = float("inf")
    ramjet_lit = False
    ramjet_asked = False
    ramjet_light_refused = False
    pullout_light_override = False
    ramjet_lightoff_mach: float | None = None
    ramjet_lightoff_altitude_m: float | None = None
    ramjet_lightoff_time_s: float | None = None
    ramjet_lightoff_mode = ""
    ramjet_lit_in_dive = False
    if ramjet_start is not None and propulsion is not None:
        # ONE source of truth for the gate Mach. FpPropulsion carries its own
        # lightoff_mach (it decides when to spend an FP solve asking the
        # ramjet to cold light); if a trajectory policy is supplied, the
        # policy owns that number, so the two can never silently disagree.
        propulsion.lightoff_mach = ramjet_start.gate_mach

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

    # Geometry the build-up needs that the ancestor never had to compute
    # (it drew everything from frontal area alone). Assembled once.
    use_buildup = drag_model == "buildup"
    geom_cache = None
    if use_buildup:
        from math import pi as _pi

        from .constants import (NOSE_TAIL_LENGTH_DIAMETERS,
                                TAIL_LENGTH_DIAMETERS)
        _wc = wing_concept
        geom_cache = {
            "diameter_m": diameter_m,
            "duct_exit_diameter_m": throat_diameter_m,
            "tail_length_m": TAIL_LENGTH_DIAMETERS * diameter_m,
            "body_length_m": (chamber_length_m + throat_length_m
                              + NOSE_TAIL_LENGTH_DIAMETERS * diameter_m),
            "wing_area_m2": (_wc.span_m ** 2 / _wc.aspect_ratio if _wc
                             else wingspan_m ** 2 / 3.0),
            "aspect_ratio": _wc.aspect_ratio if _wc else 3.0,
            "wing_sweep_deg": _wc.sweep_deg if _wc else 0.0,
            "wing_thickness_ratio": getattr(
                getattr(_wc, "airfoil", None), "thickness_ratio", 0.03),
            "oswald_e": _wc.oswald_e if _wc else 0.80,
            # capture area for the spillage term: the intake is the duct
            # ahead of the chamber, i.e. the same throat-sized flowpath
            "lip_area_m2": _pi * throat_diameter_m ** 2 / 4.0,
            "cowl_suction_recovery": cowl_suction_recovery,
            "fin_area_m2": (fin or {}).get("area_m2", 0.0),
            "fin_aspect_ratio": (fin or {}).get("aspect_ratio", 1.2),
            "fin_thickness_ratio": (fin or {}).get("thickness_ratio", 0.04),
            "fin_sweep_deg": (fin or {}).get("sweep_deg", 35.0),
        }

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
    min_traverse_accel_g = float("inf")
    min_traverse_accel_mach = 0.0
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
                elif v > (SPIRAL_BLEED_SPEED_FACTOR if overhead
                          else DECEL_SPEED_FACTOR) * stall_speed and h > flare_altitude_m:
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
                    # Descending spiral at the TRUE equilibrium glide slope,
                    # solved closed-form each step rather than flown at a
                    # fixed angle: steady unpowered flight means
                    # 0 = -D - W sin(gamma), so sin(gamma) = -D/W holds speed
                    # constant all the way down. A fixed angle cannot -- at
                    # the old -9 deg the vehicle bled monotonically and
                    # arrived at the flare window below stall (V2 landed with
                    # 3 m/s to spare, V3's higher arrival altitude stalled it
                    # outright). Drag is evaluated level here to pick the
                    # angle; cos(gamma) ~ 1 at these slopes, so the one-pass
                    # estimate is exact to well under a percent.
                    mode = "spiral"
                    if wing_concept is None:
                        drag_level_n = total_drag_n(
                            diameter_m, wingspan_m, v, atmosphere.density_kg_per_m3,
                            drag_mass_kg, 0.0, G0_M_PER_S2, mach=mach).total_n
                    else:
                        drag_level_n = _concept_drag(
                            diameter_m, wing_concept, v, atmosphere.density_kg_per_m3,
                            drag_mass_kg, 0.0, mach).total_n
                    sin_gamma = -min(drag_level_n / (m * G0_M_PER_S2), 0.5)
                    gamma_rad = asin(sin_gamma)
                    cos_gamma = cos(gamma_rad)
                    x_rate = 0.0
                else:
                    mode = "glide"
                    gamma_rad, sin_gamma, cos_gamma = glide_angle_rad, sin_glide, cos_glide
                    x_rate = direction * cos_glide
            if use_buildup:
                # engine OFF: the dead duct exit is base too, and there is
                # no captured stream, so no spillage term
                drag_result, _bd = _buildup_drag(
                    geom_cache, wing_concept, v,
                    atmosphere.density_kg_per_m3,
                    atmosphere.temperature_k, drag_mass_kg, gamma_rad, mach,
                    engine_on=False, captured_mdot_kg_per_s=0.0,
                )
            elif wing_concept is None:
                drag_result = total_drag_n(
                    diameter_m, wingspan_m, v, atmosphere.density_kg_per_m3,
                    drag_mass_kg, gamma_rad, G0_M_PER_S2, mach=mach,
                )
            else:
                drag_result = _concept_drag(
                    diameter_m, wing_concept, v, atmosphere.density_kg_per_m3,
                    drag_mass_kg, gamma_rad, mach,
                )
            # Unpowered body loading. The return half-loop is the one
            # unpowered maneuver that pulls g (a fixed RETURN_LOOP_LOAD_FACTOR
            # by construction, not a searched quantity); every other unpowered
            # leg is straight, so n_yaw = cos(gamma).
            v4_gamma_dot = 0.0
            turn_radius_m = 0.0
            if mode == "loop":
                n_yaw = RETURN_LOOP_LOAD_FACTOR
                loop_gamma_dot = G0_M_PER_S2 * (n_yaw - cos_gamma) / max(v, 1.0)
                if abs(loop_gamma_dot) > 1e-9:
                    turn_radius_m = abs(v / loop_gamma_dot)
            else:
                n_yaw = cos_gamma
        else:
            # ORDER NOTE (V4): the trajectory phase is resolved BEFORE the
            # engines are called, because the ramjet-start policy needs to
            # know whether the vehicle is descending yet ("light it in the
            # dive"). Nothing in the phase machine reads an engine result and
            # nothing in the engine calls read gamma, so this reordering is
            # bit-identical for every pre-V4 flight (locked by
            # tests/test_medium_model_v2_parity.py).
            phase_mode: str | None = None
            gamma_rad = climb_angle_rad
            sin_gamma, cos_gamma = sin_climb, cos_climb
            x_rate = cos_climb
            n_yaw_cmd: float | None = None   # None -> straight leg, n = cos(gamma)
            turn_radius_m = 0.0
            if climb_dive is not None:
                # Phase machine. Latched (not re-tested) so a phase never
                # re-opens: climb to the derived top, push over, dive through
                # the lightoff notch, pull out onto the drag strip. With
                # pullout_load_factor=None (V2/V3) the two arc phases are
                # skipped entirely and this reduces to V3's two-boolean latch.
                if not v3_climb_done and h >= v3_top_altitude_m:
                    v3_climb_done = True
                    v4_phase = "pushover" if v4_arcs else "dive"
                if v4_arcs and v4_phase == "pushover" and v4_gamma_rad <= v3_dive_rad:
                    v4_gamma_rad = v3_dive_rad
                    v4_phase = "dive"
                # The dive ends when the NOMINAL-g arc can only just still
                # make the floor -- the latest possible pull-out, hence the
                # longest dive and the most Mach available for lightoff. The
                # arc's drop is real altitude the dive does not get to use.
                if v4_arcs:
                    pullout_due = required_pullout_load_factor(
                        v, v3_dive_rad, h, climb_dive.floor_altitude_m
                    ) >= climb_dive.pullout_load_factor
                else:
                    pullout_due = h <= climb_dive.floor_altitude_m
                if not v3_dive_done and v3_climb_done and (
                        v4_phase in ("dive", "pushover")) and (
                        mach >= climb_dive.dive_end_mach or pullout_due):
                    v3_dive_done = True
                    dive_exit_mach = mach
                    if v4_arcs:
                        v4_phase = "pullout"
                        pullout_entry_altitude_m = h
                        pullout_entry_mach = mach
                    else:
                        v4_phase = "strip"
                if v4_arcs and v4_phase == "pullout" and v4_gamma_rad >= strip_gamma_rad:
                    v4_gamma_rad = strip_gamma_rad
                    v4_phase = "strip"

                if not v3_climb_done:
                    phase_mode = "v3_climb"
                    gamma_rad = v3_climb_rad
                    sin_gamma, cos_gamma = sin_v3climb, cos_v3climb
                    if climb_dive.spiral_climb:
                        # Helical: the air path is unchanged, the ground
                        # track closes into a circle. Bank is charged as
                        # load factor (hence induced drag), not assumed free.
                        n_yaw_cmd = cos_gamma / cos(radians(climb_dive.spiral_bank_deg))
                elif v4_arcs and v4_phase == "pushover":
                    phase_mode = "v4_pushover"
                    gamma_rad = v4_gamma_rad
                    sin_gamma, cos_gamma = sin(gamma_rad), cos(gamma_rad)
                    n_yaw_cmd = climb_dive.pushover_load_factor
                elif not v3_dive_done:
                    phase_mode = "v3_dive"
                    gamma_rad = v3_dive_rad
                    sin_gamma, cos_gamma = sin_v3dive, cos_v3dive
                elif v4_arcs and v4_phase == "pullout":
                    phase_mode = "v4_pullout"
                    gamma_rad = v4_gamma_rad
                    sin_gamma, cos_gamma = sin(gamma_rad), cos(gamma_rad)
                    # Hold the nominal g; pull harder only if the arc is no
                    # longer going to make the floor, and never past the
                    # limit. What actually gets flown is reported as
                    # peak_load_n_yaw -- it is an output, not an assumption.
                    n_yaw_cmd = min(
                        max(climb_dive.pullout_load_factor,
                            required_pullout_load_factor(
                                v, gamma_rad, h, climb_dive.floor_altitude_m)),
                        climb_dive.pullout_max_load_factor,
                    )
                else:
                    phase_mode = "drag_strip"
                x_rate = 0.0 if phase_mode == "v3_climb" and climb_dive.spiral_climb \
                    else cos_gamma
                # the rule (gamma >= 0 from M 0.80 through cutoff) is checked
                # on what is actually flown, not assumed from the schedule
                if (V3_RULE_MACH_LO <= mach <= V3_RULE_MACH_HI
                        and sin_gamma < -1e-9 and not rule_violated):
                    rule_violated = True
                    rule_violation_mach = mach

            # --- Ramjet start policy (V4) -------------------------------
            # Latched: once the flameholder is alive it stays alive even
            # after the vehicle pulls level again.
            if ramjet_start is None:
                allow_light = True
                gate_mach = None
            else:
                # Last-chance override: the pull-out is where the Mach gate
                # stops being a goal worth waiting for. Latched, so it does
                # not lapse when the vehicle levels out (see RamjetStart).
                if (ramjet_start.light_at_pullout and not ramjet_lit
                        and phase_mode == "v4_pullout"):
                    pullout_light_override = True
                gate_mach = ramjet_start.gate_mach
                if ramjet_lit:
                    allow_light = True
                elif pullout_light_override:
                    # Gate waived entirely -- "no matter what speed you are
                    # at". Deliberately NOT applied on the merely-already-lit
                    # path above, where the closed form's lightoff ramp is
                    # still doing real work.
                    allow_light = True
                    gate_mach = 0.0
                elif ramjet_start.require_descending:
                    allow_light = sin_gamma < 0.0
                else:
                    allow_light = True
                if allow_light and (mach >= gate_mach
                                    or pullout_light_override):
                    ramjet_asked = True

            # BOTH engines run through the transition (2026-08-12): the
            # pulsejet keeps pulsing while the ramjet duct lights -- the
            # first-principles model shows the side-inlet pulsejet operating
            # (declining) all the way to M 0.9, and the ramjet lightoff gate
            # (ramjet_simple.RAMJET_MIN_LIGHTOFF_MACH) already zeroes the
            # ramjet below its viable speed. The old either/or switch made
            # the transition gap artificially lethal: total thrust dipped to
            # a single engine exactly where drag peaks. Total = simple sum;
            # crossover_mach still reports where the ramjet first dominates.
            if propulsion is not None:
                # First-principles engines marched along the trajectory.
                # The policy only decides when the ramjet is ASKED to cold
                # light; whether it CAN is the FP model's answer (user
                # decision, 2026-08-13). The gate Mach itself was pushed onto
                # the propulsion object at setup, so there is one source of
                # truth for it.
                thrust_n, fuel_mdot_kg_per_s = propulsion.thrust_and_fuel(
                    t, mach, h, allow_ramjet_light=allow_light,
                    ignore_gate_mach=pullout_light_override)
                if not on_ramjet and propulsion.ramjet_lit and (
                        propulsion.ramjet_thrust_n
                        > propulsion.pulsejet_thrust_n):
                    on_ramjet = True
                    crossover_mach = mach
                ramjet_result = pulsejet_result = None
                ramjet_is_lit = bool(propulsion.ramjet_lit)
            else:
                ramjet_result = ramjet_thrust(
                    diameter_m, throat_diameter_m, mach, h, fuel, atmosphere=atmosphere,
                    lightoff_mach=gate_mach, allow_light=allow_light,
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
                ramjet_is_lit = (ramjet_result.lit
                                 and ramjet_result.net_thrust_n > 0.0)
            weight_flow = fuel_mdot_kg_per_s * G0_M_PER_S2
            specific_impulse_s = thrust_n / weight_flow if weight_flow > 0.0 else 0.0
            mode = phase_mode if phase_mode is not None else (
                "ramjet" if on_ramjet else "pulsejet")

            if not ramjet_lit and ramjet_is_lit:
                ramjet_lit = True
                ramjet_lightoff_mach = mach
                ramjet_lightoff_altitude_m = h
                ramjet_lightoff_time_s = t
                ramjet_lightoff_mode = mode
                ramjet_lit_in_dive = mode in ("v3_dive", "v4_pushover")

            # Maneuvering lift: during a pitch arc the wing carries n*W, not
            # W*cos(gamma), and induced drag goes as lift^2 -- charging only
            # the straight-flight lift would make a hard pull-out free. The
            # drag calls take a MASS, so the load factor is converted back
            # through cos(gamma); on any straight leg n_yaw == cos(gamma) and
            # this collapses to exactly `m`, which is what keeps V2/V3
            # bit-identical.
            n_yaw = cos_gamma if n_yaw_cmd is None else n_yaw_cmd
            v4_gamma_dot = 0.0
            if n_yaw_cmd is None:
                drag_mass_kg = m
            else:
                drag_mass_kg = m * n_yaw / max(cos_gamma, _MIN_COS_GAMMA)
            if phase_mode in ("v4_pushover", "v4_pullout"):
                # PITCH arc: the load factor curves the flight path in the
                # vertical plane, so it integrates gamma.
                v4_gamma_dot = G0_M_PER_S2 * (n_yaw - cos_gamma) / max(v, 1.0)
                if abs(v4_gamma_dot) > 1e-9:
                    turn_radius_m = abs(v / v4_gamma_dot)
                if phase_mode == "v4_pushover":
                    if pushover_radius_m == 0.0:
                        pushover_radius_m = turn_radius_m
                    pushover_duration_s += dt_s
                else:
                    if pullout_radius_m == 0.0:
                        pullout_radius_m = turn_radius_m
                    pullout_duration_s += dt_s
            elif phase_mode == "v3_climb" and n_yaw_cmd is not None:
                # SPIRAL climb: the load factor curves the ground track in
                # the HORIZONTAL plane, so gamma is untouched and the radius
                # reported is the turn circle, R = V^2/(g0 tan(bank)).
                tan_bank = tan(radians(climb_dive.spiral_bank_deg))
                if tan_bank > 1e-9:
                    turn_radius_m = v * v / (G0_M_PER_S2 * tan_bank)
                    spiral_radius_m = turn_radius_m
            if use_buildup:
                # engine ON: exhaust fills the duct exit (smaller base),
                # and the inlet spills whatever it does not swallow
                drag_result, _bd = _buildup_drag(
                    geom_cache, wing_concept, v,
                    atmosphere.density_kg_per_m3,
                    atmosphere.temperature_k, drag_mass_kg, gamma_rad, mach,
                    engine_on=True,
                    # Spillage is an INLET phenomenon and the ramjet duct is
                    # the one with a capture streamtube: ramjet_simple
                    # already reports both what the streamtube offers
                    # (captured_*) and what the engine actually swallows.
                    # When the ramjet is not producing, the duct is cold and
                    # wide open, so it passes what it is given and there is
                    # no spillage -- spillage arises when the HOT engine
                    # restricts the flow it will accept.
                    captured_mdot_kg_per_s=(
                        propulsion.captured_mdot_kg_per_s(
                            atmosphere.density_kg_per_m3, v,
                            geom_cache["lip_area_m2"])
                        if propulsion is not None
                        else _duct_swallowed_kg_per_s(
                            ramjet_result, atmosphere.density_kg_per_m3, v,
                            geom_cache["lip_area_m2"])),
                )
            elif wing_concept is None:
                drag_result = total_drag_n(
                    diameter_m, wingspan_m, v, atmosphere.density_kg_per_m3,
                    drag_mass_kg, gamma_rad, G0_M_PER_S2, mach=mach,
                )
            else:
                drag_result = _concept_drag(
                    diameter_m, wing_concept, v, atmosphere.density_kg_per_m3,
                    drag_mass_kg, gamma_rad, mach,
                )

        weight_n = m * G0_M_PER_S2
        thrust_to_weight = thrust_n / weight_n
        weight_along_path_n = weight_n * sin_gamma
        acceleration_m_per_s2 = (thrust_n - drag_result.total_n - weight_along_path_n) / m
        # --- Body loading (V4) ------------------------------------------
        # Accelerometer convention: specific force, gravity EXCLUDED. That is
        # why load_n_roll is not acceleration_m_per_s2/g0 -- the latter
        # includes the weight-along-path term, which an accelerometer riding
        # the vehicle cannot feel. 2-D trajectory, so roll (axial) and yaw
        # (normal, in the trajectory plane) carry all of it.
        load_n_roll = (thrust_n - drag_result.total_n) / weight_n
        load_n_yaw = n_yaw
        load_n_total = sqrt(load_n_roll * load_n_roll + load_n_yaw * load_n_yaw)
        if not engine_off:
            # Peaks tracked over POWERED steps only: the return half-loop's
            # 6 g is a fixed constant of the landing profile, not something
            # the V4 trajectory search controls, and letting it dominate
            # would mask the pull-out it is meant to measure.
            if load_n_total > peak_load_n_total:
                peak_load_n_total = load_n_total
                peak_load_mode = mode
            peak_load_n_yaw = max(peak_load_n_yaw, abs(load_n_yaw))
            peak_load_n_roll = max(peak_load_n_roll, abs(load_n_roll))
            # Only meaningful once the vehicle is over the top and coming
            # DOWN. The flight starts at h = 0, which is below the 400 ft
            # floor by definition, so tracking from launch flags every single
            # flight as a floor bust (it did -- 646 of 1350).
            if v3_climb_done:
                min_powered_altitude_m = min(min_powered_altitude_m, h)
        if not engine_off:
            # ENGINE-ONLY margin (2026-08-12, V3): the gravity term is
            # clamped at zero so a dive cannot inflate the margin. In a dive
            # weight_along_path_n is negative, which would shrink "demand"
            # and make an underpowered engine look healthy purely because it
            # is pointed downhill. A climb's gravity term is real work the
            # engine must do, so it still counts. Identical to the old form
            # for every non-diving flight (V2 and earlier climb at +1 deg),
            # which tests/test_v2_frozen.py verifies.
            demand_n = drag_result.total_n + max(weight_along_path_n, 0.0)
            if demand_n > 1e-9:
                margin = thrust_n / demand_n
                if margin < min_powered_thrust_margin:
                    min_powered_thrust_margin = margin
                    min_margin_mach = mach
            accel_g = acceleration_m_per_s2 / G0_M_PER_S2
            if accel_g < min_powered_accel_g:
                min_powered_accel_g = accel_g
                min_accel_mach = mach
            # TRAVERSE acceleration excludes the commanded V3 climb. The
            # gate exists so the vehicle is never stranded in a regime it
            # cannot accelerate through; a deliberate climb is not such a
            # regime -- level-equivalent acceleration there is 0.6-0.8 g and
            # the vehicle can shallow out at any instant to get it back. The
            # lightoff notch and the transonic drag strip ARE such regimes,
            # so those are what get gated. (The engine-only thrust margin
            # above still applies to every powered step, climb included, so
            # an underpowered engine cannot hide inside the climb.)
            # The V4 pushover is excluded on the same reasoning as the climb:
            # it is a commanded maneuver the vehicle can abandon at any
            # instant, not a regime it can be stranded in. Excluding it
            # cannot hide anything, because its own worst acceleration is
            # reported separately as min_pushover_accel_g.
            if mode == "v4_pushover":
                min_pushover_accel_g = min(min_pushover_accel_g, accel_g)
            elif mode != "v3_climb" and accel_g < min_traverse_accel_g:
                min_traverse_accel_g = accel_g
                min_traverse_accel_mach = mach

        states.append(
            FlightState(
                t, h, x, v, mach, m, fuel_burned_kg, mode, thrust_n, drag_result.total_n,
                acceleration_m_per_s2, thrust_to_weight, specific_impulse_s, stall_speed,
                gamma_rad, load_n_roll, load_n_yaw, load_n_total, turn_radius_m,
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
        if v4_gamma_dot != 0.0:
            v4_gamma_rad += v4_gamma_dot * dt_s
        fuel_step_kg = fuel_mdot_kg_per_s * dt_s
        fuel_burned_kg += fuel_step_kg
        m = max(m - fuel_step_kg, MASS_FLOOR_KG)
        t += dt_s

    floor_violated = (
        climb_dive is not None
        and min_powered_altitude_m
        < climb_dive.floor_altitude_m - V4_FLOOR_TOLERANCE_M
    )
    # FP path only: the policy opened the window and the first-principles
    # model still would not light. On the closed-form path the policy IS the
    # light, so a refusal is not a thing that can happen.
    ramjet_light_refused = (propulsion is not None and ramjet_asked
                            and not ramjet_lit)
    return FlightResult(
        states, motor_cutoff_reached, landed, safe_landing, hit_mass_floor,
        stalled, crossover_mach, min_powered_thrust_margin, min_margin_mach,
        min_powered_accel_g, min_accel_mach,
        min_traverse_accel_g, min_traverse_accel_mach, v3_top_altitude_m,
        rule_violated, rule_violation_mach,
        peak_load_n_total, peak_load_n_yaw, peak_load_n_roll, peak_load_mode,
        pushover_radius_m, pullout_radius_m, spiral_radius_m,
        pushover_duration_s, pullout_duration_s,
        pullout_entry_altitude_m, pullout_entry_mach,
        min_powered_altitude_m, floor_violated, dive_exit_mach,
        ramjet_lightoff_mach, ramjet_lightoff_altitude_m,
        ramjet_lightoff_time_s, ramjet_lightoff_mode, ramjet_lit_in_dive,
        ramjet_light_refused, min_pushover_accel_g,
    )
