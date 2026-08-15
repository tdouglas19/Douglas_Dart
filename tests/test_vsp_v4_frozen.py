"""Guard the VSP V4 aero/stability freeze.

Rebuilds the frozen configuration from `docs/vsp_v4_frozen/design.json` and
checks the geometry, mass, stability, flutter and control results still come out
where they were frozen.

Cheap by construction: everything here is analytic (Barrowman, the flutter
criterion, strip-theory roll). NO VSPAERO -- that solver does not converge on
this configuration, which is exactly why the freeze uses analytic methods as its
primary source. See `scripts/vsp_model/README.md`.
"""

from __future__ import annotations

import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "vsp_model"))

FREEZE = REPO_ROOT / "docs" / "vsp_v4_frozen" / "design.json"


def _rebuild():
    import mass_cg
    from geometry_inputs import GeometryInputs
    from sizing_input import SizingInput
    from vehicle_geometry import build_vehicle_geometry

    frozen = json.loads(FREEZE.read_text(encoding="utf-8"))
    sizing = SizingInput.from_json(REPO_ROOT / frozen["sizing_source"].replace("\\", "/"))
    inputs = replace(
        GeometryInputs(),
        fin_area_ratio=0.80,
        wing_root_le_x_m=1.60,
        fin_aspect_ratio=2.1,
        fin_thickness_to_chord=0.08,
        fin_clocking_deg_explicit=(45.0, 135.0),
    )
    mass = mass_cg.vehicle_mass_properties(sizing, inputs)
    geometry = build_vehicle_geometry(sizing, inputs, cg_x_m=mass.cg_x_m)
    return frozen, sizing, inputs, mass, geometry


class VspV4FreezeTest(unittest.TestCase):
    def setUp(self):
        if not FREEZE.is_file():
            self.skipTest(f"freeze not present: {FREEZE}")
        self.frozen, self.sizing, self.inputs, self.mass, self.geometry = _rebuild()

    def test_tail_geometry(self):
        tail = self.frozen["tail"]
        panels = self.geometry.fin_panels
        self.assertEqual(len(panels), 2, "the freeze is a two-panel V-tail")
        self.assertEqual(
            sorted(p.clocking_deg for p in panels), [45.0, 135.0],
            "both panels must sit in the upper half -- ground clearance is why "
            "this is a V-tail and not the cruciform",
        )
        total = sum(p.exposed_area_m2 for p in panels)
        self.assertAlmostEqual(total, tail["total_area_m2"], places=6)
        self.assertAlmostEqual(panels[0].root_chord_m, tail["root_chord_m"], places=6)
        self.assertAlmostEqual(panels[0].semispan_m, tail["semispan_m"], places=6)

    def test_no_surface_projects_below_the_body(self):
        """A belly landing must not land on a fin."""
        from math import radians, sin

        lowest = min(
            (p.mount_radius_m + p.semispan_m) * sin(radians(p.clocking_deg))
            for p in self.geometry.fin_panels
        )
        self.assertGreater(
            lowest, -0.5 * self.sizing.body_diameter_m,
            "a fin hangs below the body underside; the cruciform did this and it "
            "is the defect the V-tail exists to fix",
        )

    def test_mass_and_cg(self):
        frozen = self.frozen["mass"]
        self.assertAlmostEqual(self.mass.mass_kg, frozen["release_kg"], places=5)
        self.assertAlmostEqual(self.mass.cg_x_m, frozen["release_cg_x_m"], places=6)

    def test_static_stability(self):
        import barrowman

        results = barrowman.analyse(self.sizing, self.inputs, self.mass.cg_x_m, mach=0.5)
        frozen = self.frozen["stability_barrowman"]["release"]
        self.assertAlmostEqual(
            results["pitch"].static_margin_calibres, frozen["pitch_cal"], places=4
        )
        self.assertAlmostEqual(
            results["yaw"].static_margin_calibres, frozen["yaw_cal"], places=4
        )
        self.assertTrue(results["pitch"].stable and results["yaw"].stable)

    def test_flutter_margin_holds(self):
        """Flutter is a GATE. t/c 0.08 and AR 2.1 exist to satisfy it."""
        import flutter

        for result in flutter.check_vehicle(self.geometry):
            self.assertGreaterEqual(
                result.margin, flutter.REQUIRED_FLUTTER_MARGIN,
                f"{result.surface} fails the flutter margin at "
                f"M {result.flutter_mach:.2f}",
            )

    def test_control_authority_covers_the_envelope(self):
        """A 40% chord flap must still reach the 6 g structural cap."""
        import control

        cases = control.required_deflections(self.sizing, self.inputs, self.mass)
        verdict = control.evaluate(cases, 0.40)
        self.assertTrue(
            verdict.all_cases_fit,
            "40% chord ruddervator no longer covers every commanded manoeuvre",
        )

    def test_roll_is_damped(self):
        import barrowman

        self.assertLess(
            barrowman.roll_damping(self.geometry, 0.5), 0.0,
            "roll damping must be negative",
        )


if __name__ == "__main__":
    unittest.main()
