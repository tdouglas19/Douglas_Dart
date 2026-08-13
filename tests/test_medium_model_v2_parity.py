"""medium_model must re-fly the frozen V2 design to its ancestor's numbers.

This is step 0 of the fidelity ladder and the whole basis of the method:
medium_model starts as a proven-identical copy of simple_model, so that
every LATER difference in a V2 re-fly is attributable to the one piece of
fidelity that was added, not to a transcription error in the copy.

Two things are asserted:
  1. medium_model reproduces docs/v2_frozen/design.json's headline numbers
     (the same contract tests/test_v2_frozen.py holds simple_model to).
  2. medium_model and simple_model agree STATE FOR STATE on that flight --
     the strong form, which catches drift the headline numbers would miss.

Once step 1 (the drag build-up) lands, assertion 2 is expected to fail by
design; at that point it moves behind the model's "legacy drag" switch and
the delta it measures becomes the first row of the attribution table.

CD0 is baked in at IMPORT time in both packages, so the override is set
before importing either.
"""
from __future__ import annotations

import json
import os
import unittest
from pathlib import Path

FROZEN = Path(__file__).resolve().parent.parent / "docs" / "v2_frozen" / "design.json"
DESIGN = json.loads(FROZEN.read_text())
_CD0 = str(DESIGN["constants_at_freeze"]["CD0_FRONTAL"])
os.environ["SIMPLE_MODEL_CD0_FRONTAL"] = _CD0
os.environ["MEDIUM_MODEL_CD0_FRONTAL"] = _CD0

from medium_model.constants import AIRFOILS, FUELS  # noqa: E402
from medium_model.drag import WingConcept  # noqa: E402
from medium_model.flight_sim import VehicleGeometry, run_flight  # noqa: E402
from medium_model.mission import (  # noqa: E402
    MAX_WET_MASS_KG, MOTOR_CUTOFF_MACH, usable_fuel_kg)


def _inputs(pkg_airfoils, pkg_fuels, pkg_geometry_cls, pkg_concept_cls):
    c = DESIGN["vehicle_candidate"]
    w = DESIGN["wing_concept"]
    geometry = pkg_geometry_cls(
        c["diameter_m"], c["throat_diameter_m"], c["chamber_length_m"],
        c["throat_length_m"], c["wingspan_m"], pkg_fuels[c["fuel_key"]],
    )
    concept = pkg_concept_cls(w["span_m"], w["aspect_ratio"], w["taper_ratio"],
                              w["sweep_deg"], pkg_airfoils[w["airfoil_key"]])
    burn = usable_fuel_kg(c["diameter_m"], c["throat_diameter_m"],
                          c["throat_length_m"],
                          pkg_fuels[c["fuel_key"]].density_kg_per_m3)
    return geometry, concept, burn, c["climb_angle_deg"]


def _fly_medium():
    geometry, concept, burn, climb = _inputs(
        AIRFOILS, FUELS, VehicleGeometry, WingConcept)
    return run_flight(
        geometry, MAX_WET_MASS_KG, climb_angle_deg=climb,
        motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02, max_time_s=900.0,
        wing_concept=concept, max_fuel_burn_kg=burn, return_to_launch=True,
    )


def _fly_simple():
    from simple_model.constants import AIRFOILS as A, FUELS as F
    from simple_model.drag import WingConcept as WC
    from simple_model.flight_sim import VehicleGeometry as VG, run_flight as rf

    geometry, concept, burn, climb = _inputs(A, F, VG, WC)
    return rf(geometry, MAX_WET_MASS_KG, climb_angle_deg=climb,
              motor_cutoff_mach=MOTOR_CUTOFF_MACH, dt_s=0.02,
              max_time_s=900.0, wing_concept=concept,
              max_fuel_burn_kg=burn, return_to_launch=True)


class MediumModelV2ParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = _fly_medium()

    def test_mission_constants_match_the_freeze(self):
        from medium_model import constants as C
        frozen = DESIGN["constants_at_freeze"]
        for name in ("MIN_POWERED_ACCELERATION_G",
                     "MIN_POWERED_THRUST_MARGIN_FRACTION",
                     "PULSEJET_MAX_THROAT_AREA_FRACTION",
                     "RAMJET_MIN_LIGHTOFF_MACH"):
            self.assertAlmostEqual(getattr(C, name), frozen[name], places=9,
                                   msg=f"{name} drifted in the copy")
        self.assertAlmostEqual(MOTOR_CUTOFF_MACH, 1.1, places=9)

    def test_mission_still_closes(self):
        r = self.result
        self.assertTrue(r.motor_cutoff_reached)
        self.assertTrue(r.safe_landing)
        self.assertFalse(r.stalled)
        self.assertFalse(r.hit_mass_floor)

    def test_headline_numbers_reproduce(self):
        r = self.result
        v = DESIGN["verified_mission"]
        peak_tw = max(s.thrust_to_weight for s in r.states)
        self.assertAlmostEqual(r.min_powered_accel_g, v["min_powered_accel_g"],
                               places=2)
        self.assertAlmostEqual(r.min_accel_mach, v["min_accel_mach"], places=2)
        self.assertAlmostEqual(r.min_powered_thrust_margin,
                               v["min_powered_thrust_margin"], places=2)
        self.assertAlmostEqual(peak_tw, v["peak_thrust_to_weight"], places=2)

    def test_still_returns_to_launch(self):
        r = self.result
        self.assertLess(abs(r.states[-1].distance_m), 50.0)
        self.assertGreater(max(s.distance_m for s in r.states), 5_000.0)

    def test_state_for_state_identical_to_simple_model(self):
        """The strong form: the copy is not merely close, it is the same
        model. Expected to be retired when step 1 (drag build-up) lands."""
        med = self.result
        sim = _fly_simple()
        self.assertEqual(len(med.states), len(sim.states))
        for i in range(0, len(med.states), 37):    # sample, not all 8500
            a, b = med.states[i], sim.states[i]
            self.assertAlmostEqual(a.mach, b.mach, places=12, msg=f"i={i}")
            self.assertAlmostEqual(a.altitude_m, b.altitude_m, places=9,
                                   msg=f"i={i}")
            self.assertAlmostEqual(a.thrust_n, b.thrust_n, places=9,
                                   msg=f"i={i}")
            self.assertAlmostEqual(a.drag_n, b.drag_n, places=9, msg=f"i={i}")
            self.assertAlmostEqual(a.mass_kg, b.mass_kg, places=12,
                                   msg=f"i={i}")
            self.assertEqual(a.mode, b.mode, msg=f"i={i}")
        self.assertAlmostEqual(med.states[-1].distance_m,
                               sim.states[-1].distance_m, places=6)


if __name__ == "__main__":
    unittest.main()
