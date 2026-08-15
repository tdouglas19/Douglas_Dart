"""Map medium_model's vehicle geometry onto the two FP engine models.

The ancestor describes a vehicle with five numbers (body diameter, throat
diameter, chamber length, throat length, wingspan). The FP models want
real flowpaths: a pulsejet with a valve pack and an intake runner, a
ramjet with a diffuser, a flameholder and a nozzle. This module is where
that gap is bridged, and every bridging assumption is stated here rather
than buried at a call site.

Deliberately NOT reusing ``src/douglas_dart/*_fp_bridge.py``: those speak
``ReferenceCase``, a different parameterization with its own config
plumbing. medium_model refines the frozen V2 design, which is described
in the ancestor's terms.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# --- RJ-1 flameholder proportions (validated reference design) ---------
# ramjet-fp's reference engine sits in a 0.190 m combustor with a 0.025 m
# gutter at 0.060 m mid-radius. V2 has no flameholder concept at all, so
# these proportions are scaled to its combustor. Flagged, not hidden.
_RJ1_COMBUSTOR_M = 0.190
_RJ1_GUTTER_WIDTH_FRACTION = 0.025 / _RJ1_COMBUSTOR_M     # 0.1316
_RJ1_GUTTER_RADIUS_FRACTION = 0.060 / _RJ1_COMBUSTOR_M    # 0.3158

# The gutter must never choke ahead of the nozzle throat: past it, the
# flow area must keep this margin over the throat. (A Douglas Dart
# candidate with a 0.170 m shared throat hit this cap and its flame could
# not hold at any condition -- the throat sizes the flameholder.)
_GUTTER_MIN_AREA_MARGIN_OVER_THROAT = 1.15


@dataclass(frozen=True)
class MediumModelFpSpec:
    """Frozen, hashable engine description derived from V2's geometry."""

    diameter_m: float
    throat_diameter_m: float
    chamber_length_m: float
    throat_length_m: float
    gutter_capped: bool = False
    chamber_diameter_fraction: float = 0.95
    """Chamber (= combustor) diameter as a fraction of the BODY diameter.

    0.95 is the historical value and the default, so every V3b/V3c/V3d FP
    result stays reproducible: it assumed a thin annular gap between the
    duct and the skin. V4's premise is that the chamber IS the body -- the
    gap was deleted on 2026-08-13 because it never fit the side valve
    runners anyway (docs/v4_frozen/design.json) -- so a V4 run passes 1.0.

    It scales more than the chamber bore. The valve pack and the intake are
    sized isometrically OFF the chamber diameter, and the ramjet burns in
    the same duct, so all four move together. That is the point: "the
    chamber is the body" is a statement about the flowpath, not about one
    number (user decision, 2026-08-13)."""

    @property
    def chamber_diameter_m(self) -> float:
        return self.chamber_diameter_fraction * self.diameter_m

    # -- ramjet ---------------------------------------------------------
    @property
    def combustor_diameter_m(self) -> float:
        """The duct the ramjet burns in IS the pulsejet duct on this
        shared-flowpath vehicle: chamber-sized, floored above the throat
        so the nozzle stays convergent."""
        return max(self.chamber_diameter_m, 1.10 * self.throat_diameter_m)

    def ramjet_geometry(self):
        from ramjet_fp import RamjetGeometry

        # Axial stations follow the ancestor's own layout: the duct is
        # chamber + tube, with nose/tail allowances fore and aft.
        duct = self.chamber_length_m + self.throat_length_m
        return RamjetGeometry(
            lip_diameter=self.throat_diameter_m,
            combustor_diameter=self.combustor_diameter_m,
            throat_diameter=self.throat_diameter_m,
            exit_area_ratio=1.05,
            x_combustor_start=0.18 * duct,
            x_combustor_end=0.80 * duct,
            x_throat=0.95 * duct,
            x_exit=duct,
        )

    def ramjet_flameholder(self):
        from ramjet_fp import FlameholderDesign

        d_c = self.combustor_diameter_m
        r_g = _RJ1_GUTTER_RADIUS_FRACTION * d_c
        w_g = _RJ1_GUTTER_WIDTH_FRACTION * d_c
        a_c = 0.25 * math.pi * d_c ** 2
        a_th = 0.25 * math.pi * self.throat_diameter_m ** 2
        a_g_max = a_c - _GUTTER_MIN_AREA_MARGIN_OVER_THROAT * a_th
        if 2.0 * math.pi * r_g * w_g > a_g_max:
            w_g = max(a_g_max / (2.0 * math.pi * r_g), 1e-3)
        duct = self.chamber_length_m + self.throat_length_m
        return FlameholderDesign(
            x_fh=0.28 * duct,
            gutter_width=w_g,
            frontal_area=2.0 * math.pi * r_g * w_g,
            shear_perimeter=4.0 * math.pi * r_g,
        )

    # -- pulsejet -------------------------------------------------------
    # FP-1 splits its tail section (everything aft of the chamber) as
    # cone 0.130 : tailpipe 0.620, i.e. the cone is 17.33% of it.
    _CONE_FRACTION_OF_TAIL = 0.130 / (0.130 + 0.620)

    def pulsejet_geometry(self):
        from pulsejet_fp.geometry import EngineGeometry

        # The DUCT LENGTH BUDGET IS chamber_length + throat_length, and the
        # cone comes OUT of it, not on top of it. (Adding the cone as extra
        # length built an engine 30% longer than the vehicle's duct --
        # found 2026-08-13, and it flattered the acoustics, since a longer
        # gas column is more favourable.)
        tail_total = self.throat_length_m
        cone = self._CONE_FRACTION_OF_TAIL * tail_total
        return EngineGeometry(
            chamber_diameter=self.chamber_diameter_m,
            chamber_length=self.chamber_length_m,
            cone_length=cone,
            tailpipe_diameter=self.throat_diameter_m,
            tailpipe_length=tail_total - cone,
        )

    def pulsejet_valve(self, reference_valve):
        """Petals scaled ISOMETRICALLY from the validated FP-1 pack.

        Euler-Bernoulli similarity is why: f_n ~ h/L^2 ~ 1/s matches the
        engine's own cycle-frequency scaling, cracking pressure is
        scale-invariant and lift is proportional, so the validated valve
        dynamics carry over. (Scaling petal COUNT at fixed petal size
        instead breaks that similarity and kills the engine -- a finding
        from the Douglas Dart pulsejet-fp integration.)
        """
        from dataclasses import replace

        s = self.chamber_diameter_m / 0.078      # vs FP-1 chamber dia
        return replace(
            reference_valve,
            petal_length=reference_valve.petal_length * s,
            petal_width=reference_valve.petal_width * s,
            petal_thickness=reference_valve.petal_thickness * s,
            port_area=reference_valve.port_area * s * s,
            max_lift=reference_valve.max_lift * s,
            seat_preload=reference_valve.seat_preload * s,
        )

    def pulsejet_intake(self):
        from pulsejet_fp import IntakeDesign

        s = self.chamber_diameter_m / 0.078
        # Side-mounted inlets ingesting boundary-layer air at static
        # pressure -- the Douglas Dart configuration (pulsejet-fp #8c).
        return IntakeDesign(
            duct_length=0.060 * s,
            duct_diameter=0.050 * s,
            plenum_volume=5e-5 * s ** 3,
            orientation="side",
            bl_momentum_fraction=0.6,
        )


def spec_from_geometry(geometry, chamber_diameter_fraction: float = 0.95
                       ) -> MediumModelFpSpec:
    """Build the FP spec from a medium_model ``VehicleGeometry``.

    chamber_diameter_fraction defaults to the historical 0.95 so existing
    callers are unchanged; pass 1.0 for a V4 airframe, where the chamber is
    the body."""
    return MediumModelFpSpec(
        diameter_m=geometry.diameter_m,
        throat_diameter_m=geometry.throat_diameter_m,
        chamber_length_m=geometry.chamber_length_m,
        throat_length_m=geometry.throat_length_m,
        chamber_diameter_fraction=chamber_diameter_fraction,
    )
