from __future__ import annotations

from dataclasses import replace
from math import radians
import tempfile
import unittest
from pathlib import Path

from douglas_dart.aero import SupplementedVSPAeroModel
from douglas_dart.aero_tables import (
    VSPAeroCoefficientTable,
    load_vspaero_summary_json,
    write_vspaero_summary_json,
)
from douglas_dart.atmosphere import standard_atmosphere
from douglas_dart.config import load_reference_case
from douglas_dart.flight import PointMassState
from douglas_dart.vspaero import VSPAeroPoint, VSPAeroSweepSummary


ROOT = Path(__file__).resolve().parents[1]


def synthetic_point(
    mach: float,
    alpha_deg: float,
    beta_deg: float = 0.0,
    *,
    zero_drag: bool = False,
) -> VSPAeroPoint:
    return VSPAeroPoint(
        mach=mach,
        alpha_deg=alpha_deg,
        beta_deg=beta_deg,
        lift_coefficient=0.10 + 0.20 * mach + 0.03 * alpha_deg,
        drag_coefficient_inviscid=(
            0.0 if zero_drag else 0.010 + 0.005 * mach + 0.0005 * alpha_deg
        ),
        side_force_coefficient=0.01 * beta_deg,
        rolling_moment_coefficient=-0.001 * beta_deg,
        pitching_moment_coefficient=-0.02 - 0.01 * alpha_deg,
        yawing_moment_coefficient=0.002 * beta_deg,
        sweep_results_id=f"sweep_{mach}_{alpha_deg}_{beta_deg}",
        history_results_id=f"history_{mach}_{alpha_deg}_{beta_deg}",
        status=("synthetic_test_point",),
    )


def synthetic_summary(case, *, zero_drag: bool = False) -> VSPAeroSweepSummary:
    return VSPAeroSweepSummary(
        case_name=case.name,
        openvsp_version="OpenVSP 3.51.2 synthetic",
        analysis_method="panel",
        model_path="synthetic.vsp3",
        compute_geometry_results_id="synthetic_geometry",
        reference_area_m2=case.flight.reference_area_m2,
        reference_span_m=0.525,
        reference_chord_m=0.3033,
        reference_cg_x_m=case.geometry.reference_cg_x_m,
        points=tuple(
            synthetic_point(mach, alpha, zero_drag=zero_drag)
            for mach in (0.20, 1.10)
            for alpha in (-4.0, 4.0, 12.0)
        ),
        live_solver_run=False,
    )


class VSPAeroTableTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
        )

    def test_bilinear_interpolation_recovers_linear_coefficients(self):
        table = VSPAeroCoefficientTable.from_summary(synthetic_summary(self.case))
        value = table.evaluate(0.65, 2.0)
        self.assertAlmostEqual(value.lift_coefficient, 0.10 + 0.20 * 0.65 + 0.03 * 2.0)
        self.assertAlmostEqual(
            value.drag_coefficient_inviscid,
            0.010 + 0.005 * 0.65 + 0.0005 * 2.0,
        )
        self.assertFalse(value.clamped_to_table_boundary)

    def test_out_of_domain_query_clamps_visibly(self):
        table = VSPAeroCoefficientTable.from_summary(synthetic_summary(self.case))
        value = table.evaluate(1.30, 20.0)
        endpoint = table.evaluate(1.10, 12.0)
        self.assertTrue(value.clamped_to_table_boundary)
        self.assertEqual(value.lift_coefficient, endpoint.lift_coefficient)

    def test_incomplete_longitudinal_grid_is_rejected(self):
        summary = synthetic_summary(self.case)
        incomplete = replace(summary, points=summary.points[:-1])
        with self.assertRaisesRegex(ValueError, "not rectangular"):
            VSPAeroCoefficientTable.from_summary(incomplete)

    def test_summary_json_round_trip_preserves_reference_metadata(self):
        summary = synthetic_summary(self.case)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vspaero_summary.json"
            write_vspaero_summary_json(summary, path)
            restored = load_vspaero_summary_json(path)
        self.assertEqual(restored, summary)

    def test_model_adds_solver_and_supplementary_drag_areas(self):
        table = VSPAeroCoefficientTable.from_summary(synthetic_summary(self.case))
        supplementary_drag_area_m2 = 0.006
        model = SupplementedVSPAeroModel(
            self.case,
            table,
            lambda mach: supplementary_drag_area_m2,
            supplementary_drag_status="synthetic_supplementary_drag",
            allow_nonlive_table=True,
        )
        atmosphere = standard_atmosphere(4500.0)
        state = PointMassState(
            downrange_m=0.0,
            altitude_m=4500.0,
            speed_m_per_s=0.65 * atmosphere.speed_of_sound_m_per_s,
            flight_path_angle_rad=0.0,
            mass_kg=self.case.flight.initial_mass_kg,
        )
        forces = model.forces(state, radians(2.0))
        expected_inviscid_area_m2 = (
            (0.010 + 0.005 * 0.65 + 0.0005 * 2.0)
            * self.case.flight.reference_area_m2
        )
        self.assertAlmostEqual(
            forces.inviscid_external_drag_area_m2,
            expected_inviscid_area_m2,
        )
        self.assertAlmostEqual(
            forces.total_drag_area_m2,
            expected_inviscid_area_m2 + supplementary_drag_area_m2,
        )
        self.assertFalse(forces.provisional_budget_model)
        self.assertAlmostEqual(
            model.angle_of_attack_for_lift_coefficient(
                0.65,
                0.10 + 0.20 * 0.65 + 0.03 * 2.0,
            ),
            radians(2.0),
        )

    def test_zero_solver_and_supplementary_drag_produce_zero_drag(self):
        table = VSPAeroCoefficientTable.from_summary(
            synthetic_summary(self.case, zero_drag=True)
        )
        model = SupplementedVSPAeroModel(
            self.case,
            table,
            lambda mach: 0.0,
            supplementary_drag_status="zero_drag_limiting_case",
            allow_nonlive_table=True,
        )
        atmosphere = standard_atmosphere(4500.0)
        state = PointMassState(
            0.0,
            4500.0,
            0.65 * atmosphere.speed_of_sound_m_per_s,
            0.0,
            self.case.flight.initial_mass_kg,
        )
        forces = model.forces(state, 0.0)
        self.assertEqual(forces.total_drag_area_m2, 0.0)
        self.assertEqual(forces.drag_n, 0.0)

    def test_reference_area_mismatch_is_rejected(self):
        summary = replace(
            synthetic_summary(self.case),
            reference_area_m2=self.case.flight.reference_area_m2 * 1.01,
        )
        table = VSPAeroCoefficientTable.from_summary(summary)
        with self.assertRaisesRegex(ValueError, "reference area"):
            SupplementedVSPAeroModel(
                self.case,
                table,
                lambda mach: 0.0,
                supplementary_drag_status="test",
                allow_nonlive_table=True,
            )

    def test_nonlive_table_requires_explicit_test_override(self):
        table = VSPAeroCoefficientTable.from_summary(synthetic_summary(self.case))
        with self.assertRaisesRegex(ValueError, "non-live VSPAERO"):
            SupplementedVSPAeroModel(
                self.case,
                table,
                lambda mach: 0.0,
                supplementary_drag_status="test",
            )


if __name__ == "__main__":
    unittest.main()
