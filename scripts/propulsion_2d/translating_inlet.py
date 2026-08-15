"""EXPERIMENT: a translating centrebody that can shut the ramjet inlet.

Status: **feasibility study**, not part of the frozen 2D section.  It imports
the frozen geometry kernel but builds its own contours, so nothing here can
change what ``python -m scripts.propulsion_2d`` draws.

The idea (user, 2026-08-14): shut the ramjet intake while the pulsejet runs,
by moving the inlet annulus out near the nose's external profile and letting
the centrebody translate forward to plug it.

Why moving it outboard is the whole trick
-----------------------------------------
For a fixed capture area the annulus gets THINNER as it moves out::

    A = pi (r_lip^2 - r_fore^2) = pi (r_lip + r_fore) h     ->   h ~ A / (2 pi R)

and the stroke needed to close a gap of radial height ``h`` with a seal cone
of half-angle ``theta`` is just::

    stroke = h / tan(theta)

So both levers multiply.  Going from the current r = 66 mm annulus to r = 92 mm
takes h from 10.28 to 7.08 mm, and a 45 deg seat instead of the 20 deg spike
angle turns a 28 mm stroke into ~7 mm.

Architecture (fixed OML -- user directive)
------------------------------------------
The outer mould line never moves.  Three pieces::

    fixed nose fairing            translating sleeve
    (OML, on a spar,       SLOT   (the valve)
     carries avionics)      ||
    ------------------------||=========================
    0                   x_slot                      chamber head

  * The forebody cone is FIXED and part of the OML.  It carries the avionics,
    so no harness crosses a moving joint.
  * An annular slot at its shoulder is the capture plane.
  * The centrebody sleeve behind the slot telescopes FORWARD along the spar.
    Its conical nose seats against the cowl lip and blocks the annulus.
  * Nothing outside the cowl line moves, so the OML is unchanged in both
    states.

The second reason to want this
------------------------------
The pulsejet shares this duct and its reed valves sit at the chamber head.
With the nose inlet open, pulsejet blowdown can vent FORWARD out of the nose.
Closing it gives the pulsejet a closed head end, which may matter more than
the spillage drag it also saves.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .geometry import ASSUMED, DERIVED, MM, Chain, Segment, Vehicle

# What a poppet/plug valve normally uses: self-centring, tolerant of a little
# misalignment, and short-stroke.  Steeper seals faster but jams harder.
DEFAULT_SEAL_HALF_ANGLE_DEG = 45.0


@dataclass(frozen=True)
class TranslatingInlet:
    """A solved translating-plug inlet.  All lengths in metres."""

    capture_area_m2: float
    x_slot_m: float               # slot / capture plane station
    r_lip_m: float                # cowl lip inner radius (annulus outer edge)
    r_fore_m: float               # fairing aft rim  (annulus inner edge)
    seal_half_angle_deg: float
    cowl_lip_thickness_m: float
    x_chamber_head_m: float
    # Axial length the seal cone continues PAST the crest, so that at full
    # stroke a band of the sleeve lies flat on a matching band of the seat.
    # Zero gives line contact on a single circle -- a knife edge, not a seal.
    faying_land_m: float = 0.0

    # ---- the numbers the study exists to produce ------------------------
    @property
    def gap_m(self) -> float:
        """Radial height of the annulus when fully open."""
        return self.r_lip_m - self.r_fore_m

    @property
    def stroke_m(self) -> float:
        """Axial travel from fully open to sealed."""
        return self.gap_m / math.tan(math.radians(self.seal_half_angle_deg))

    @property
    def diffuser_length_m(self) -> float:
        return self.x_chamber_head_m - self.x_slot_m

    @property
    def forebody_half_angle_deg(self) -> float:
        return math.degrees(math.atan2(self.r_fore_m, self.x_slot_m))

    @property
    def forebody_volume_m3(self) -> float:
        """Fixed nose volume available for avionics (a cone)."""
        return math.pi / 3.0 * self.r_fore_m ** 2 * self.x_slot_m

    @property
    def faying_slant_m(self) -> float:
        """Slant length of the contact band, along the cone surface."""
        return self.faying_land_m / math.cos(
            math.radians(self.seal_half_angle_deg))

    @property
    def faying_seat_outer_r_m(self) -> float:
        """Radius the seat reaches at the aft end of the faying land."""
        return self.r_lip_m + self.faying_land_m * math.tan(
            math.radians(self.seal_half_angle_deg))

    @property
    def faying_area_m2(self) -> float:
        """Annular contact area: mean circumference x slant length."""
        r_mean = 0.5 * (self.r_lip_m + self.faying_seat_outer_r_m)
        return 2.0 * math.pi * r_mean * self.faying_slant_m

    @property
    def cowl_lip_frontal_area_m2(self) -> float:
        """Frontal area of the cowl lip itself -- the annulus between the lip
        inner and outer radius.

        This is the honest drag item.  The radial offset between the fairing
        rim and the cowl outer is NOT a bluff step: it is the slot, and the
        capture streamtube flows through it.  Only the lip's own thickness
        presents frontal area.
        """
        r_o = self.r_lip_m + self.cowl_lip_thickness_m
        return math.pi * (r_o ** 2 - self.r_lip_m ** 2)

    @property
    def cowl_lip_frontal_fraction(self) -> float:
        """Lip frontal area as a fraction of the slot's own capture area."""
        return self.cowl_lip_frontal_area_m2 / self.capture_area_m2

    # ---- kinematics -----------------------------------------------------
    def radius_at_slot(self, stroke_m: float) -> float:
        """Sleeve radius at the slot plane after translating forward."""
        s = min(max(stroke_m, 0.0), self.stroke_m)
        return self.r_fore_m + s * math.tan(
            math.radians(self.seal_half_angle_deg))

    def open_area_m2(self, stroke_m: float) -> float:
        """Flow area through the slot at a given stroke.  Zero when seated."""
        r = self.radius_at_slot(stroke_m)
        return max(math.pi * (self.r_lip_m ** 2 - r ** 2), 0.0)

    def area_fraction(self, stroke_m: float) -> float:
        return self.open_area_m2(stroke_m) / self.capture_area_m2

    def stroke_for_area_fraction(self, frac: float) -> float:
        """Inverse: stroke that leaves ``frac`` of the capture area open.

        Closed-form -- the area is quadratic in the sleeve radius, so::

            r = sqrt(r_lip^2 - frac * A / pi)
        """
        r = math.sqrt(max(self.r_lip_m ** 2
                          - frac * self.capture_area_m2 / math.pi, 0.0))
        return ((r - self.r_fore_m)
                / math.tan(math.radians(self.seal_half_angle_deg)))


def solve(capture_area_m2: float, x_slot_m: float, r_lip_m: float,
          x_chamber_head_m: float,
          seal_half_angle_deg: float = DEFAULT_SEAL_HALF_ANGLE_DEG,
          cowl_lip_thickness_m: float = 0.003,
          faying_land_m: float = 0.0) -> TranslatingInlet:
    """Solve the fairing rim radius that gives the required capture area."""
    inner_sq = r_lip_m ** 2 - capture_area_m2 / math.pi
    if inner_sq <= 0.0:
        raise ValueError(
            f"a {r_lip_m * MM:.1f} mm lip is too small to pass "
            f"{capture_area_m2 * 1e4:.2f} cm2 even with no centrebody")
    return TranslatingInlet(
        capture_area_m2=capture_area_m2,
        x_slot_m=x_slot_m,
        r_lip_m=r_lip_m,
        r_fore_m=math.sqrt(inner_sq),
        seal_half_angle_deg=seal_half_angle_deg,
        cowl_lip_thickness_m=cowl_lip_thickness_m,
        x_chamber_head_m=x_chamber_head_m,
        faying_land_m=faying_land_m,
    )


def from_vehicle(v: Vehicle, x_slot_m: float, r_lip_m: float,
                 seal_half_angle_deg: float = DEFAULT_SEAL_HALF_ANGLE_DEG,
                 faying_land_m: float = 0.0) -> TranslatingInlet:
    """Solve against the frozen vehicle's own capture requirement."""
    return solve(
        capture_area_m2=v.dims["inlet"]["implied_capture_area_m2"],
        x_slot_m=x_slot_m,
        r_lip_m=r_lip_m,
        x_chamber_head_m=v.station("chamber_start"),
        seal_half_angle_deg=seal_half_angle_deg,
        cowl_lip_thickness_m=v.assumptions["inlet"]["cowl_lip_thickness_m"],
        faying_land_m=faying_land_m,
    )


# ==========================================================================
# contours, for drawing.  Two states: OPEN (stroke 0) and SEALED.
# ==========================================================================

SPAR_RADIUS_M = 0.020      # the spar the sleeve rides on
DIFFUSER_ANGLE_DEG = 7.0   # same rule the frozen build uses


def build_chains(t: TranslatingInlet, v: Vehicle, stroke_m: float = 0.0
                 ) -> dict[str, Chain]:
    """Forward-end contours at a given stroke.

    The sleeve is an ANNULAR member riding on a spar, not a solid plug: the
    fixed fairing is a hollow shell and the sleeve slides inside its aft end,
    so only the conical seal face ever crosses the slot.

    Kinematics, stated exactly because it is easy to draw backwards.  The
    sleeve is a rigid body TRANSLATED by ``-stroke_m``; its radius at the
    fixed slot plane is therefore ``r_sleeve(x_slot + stroke)``.  For moving
    forward to CLOSE, the profile must RISE going aft at the slot -- so the
    seal cone runs from ``r_fore`` up to ``r_lip``, and its axial length IS
    the stroke.  Retracted, the slot plane sits at the foot of that cone
    (fully open); extended, at its crest (sealed).
    """
    r_body = 0.5 * v.d_body
    x_nose_end = v.station("nose_fairing_end")
    x_slot = t.x_slot_m
    tan_seal = math.tan(math.radians(t.seal_half_angle_deg))

    fairing = Chain("fairing", "fixed nose fairing (OML, ASSUMED)", [
        Segment(name="forebody cone", chain="fairing", curve="cone",
                x0=0.0, x1=x_slot, r0=0.0, r1=t.r_fore_m, source=ASSUMED,
                detail=f"{t.forebody_half_angle_deg:.2f} deg half-angle, "
                       f"FIXED shell on a spar -- carries the avionics"),
    ])

    cowl = Chain("cowl", "cowl (OML aft of the slot, ASSUMED)", [
        Segment(name="cowl outer", chain="cowl", curve="parabolic_tangent",
                x0=x_slot, x1=x_nose_end,
                r0=t.r_lip_m + t.cowl_lip_thickness_m, r1=r_body,
                source=ASSUMED, detail="fairs to the barrel"),
    ])

    # Diffuser outer wall: hold a 7 deg equivalent-cone angle on the annulus.
    # With the sleeve closing out to the spar by the chamber head, the outer
    # wall actually turns INWARD -- normal for an annular-to-round transition.
    r_eq0 = math.sqrt(t.capture_area_m2 / math.pi)
    r_eq1 = r_eq0 + t.diffuser_length_m * math.tan(
        math.radians(DIFFUSER_ANGLE_DEG))
    # the seat mirrors the seal cone, so the two are parallel in contact
    land = t.stroke_m
    cowl_inner = Chain("cowl_inner", "cowl inner / seat + diffuser (ASSUMED)", [
        Segment(name=f"{t.seal_half_angle_deg:.0f} deg seat", chain="cowl_inner",
                curve="cone", x0=x_slot, x1=x_slot + land,
                r0=t.r_lip_m, r1=t.r_lip_m + land * tan_seal,
                source=ASSUMED, detail="matching conical seat for the sleeve"),
        Segment(name="diffuser", chain="cowl_inner", curve="cone",
                x0=x_slot + land, x1=x_nose_end,
                r0=t.r_lip_m + land * tan_seal, r1=r_eq1, source=ASSUMED,
                detail=f"{DIFFUSER_ANGLE_DEG:.0f} deg equivalent-cone angle"),
    ])

    # The sleeve as a rigid body, translated by -stroke.  Seal cone rises
    # from r_fore to r_lip over exactly one stroke length.
    dx = -stroke_m
    sleeve = Chain("sleeve", "translating sleeve (the valve, ASSUMED)", [
        Segment(name="seal cone", chain="sleeve", curve="cone",
                x0=x_slot + dx, x1=x_slot + land + dx,
                r0=t.r_fore_m, r1=t.r_lip_m, source=ASSUMED,
                detail=f"{t.seal_half_angle_deg:.0f} deg seal face; "
                       f"translated {stroke_m * MM:.2f} mm forward of "
                       f"retracted"),
        Segment(name="closure", chain="sleeve", curve="smoothstep",
                x0=x_slot + land + dx, x1=x_nose_end, r0=t.r_lip_m,
                r1=SPAR_RADIUS_M, source=ASSUMED,
                detail="closes onto the spar ahead of the chamber head"),
    ])

    spar = Chain("spar", "spar (fixed, carries the fairing)", [
        Segment(name="spar", chain="spar", curve="line",
                x0=0.0, x1=x_nose_end, r0=SPAR_RADIUS_M, r1=SPAR_RADIUS_M,
                source=ASSUMED, detail="fairing support + actuator reaction"),
    ])

    return {"fairing": fairing, "cowl": cowl, "cowl_inner": cowl_inner,
            "sleeve": sleeve, "spar": spar}


# ==========================================================================
# the study
# ==========================================================================

def sweep(v: Vehicle,
          x_slots_m: Sequence[float] = (0.20, 0.25, 0.30, 0.35, 0.40),
          seal_angles_deg: Sequence[float] = (30.0, 45.0, 60.0),
          lip_fraction_of_body: float = 0.86) -> list[dict]:
    """Stroke / diffuser / volume trade across slot station and seal angle.

    ``lip_fraction_of_body`` places the lip at a constant fraction of the body
    radius, i.e. "as far outboard as the nose profile plausibly allows".
    """
    r_body = 0.5 * v.d_body
    rows = []
    for x in x_slots_m:
        for a in seal_angles_deg:
            t = from_vehicle(v, x, lip_fraction_of_body * r_body, a)
            rows.append({
                "x_slot_m": x, "seal_deg": a,
                "r_lip_m": t.r_lip_m, "r_fore_m": t.r_fore_m,
                "gap_m": t.gap_m, "stroke_m": t.stroke_m,
                "diffuser_len_m": t.diffuser_length_m,
                "forebody_deg": t.forebody_half_angle_deg,
                "forebody_vol_m3": t.forebody_volume_m3,
                "lip_frontal_m2": t.cowl_lip_frontal_area_m2,
            })
    return rows


def area_curve(t: TranslatingInlet, n: int = 21
               ) -> list[tuple[float, float, float]]:
    """(stroke m, area m2, fraction of capture) from open to sealed."""
    return [(s, t.open_area_m2(s), t.area_fraction(s))
            for s in (t.stroke_m * i / n for i in range(n + 1))]
