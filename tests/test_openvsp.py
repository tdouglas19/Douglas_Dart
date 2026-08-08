from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from dataclasses import replace

from douglas_dart.config import load_reference_case
from douglas_dart.openvsp_geometry import (
    _radial_mount_overlap_m,
    body_diameter_at_x_m,
    body_stations,
    build_openvsp_geometry,
    clocking_angles_deg,
    shell_outer_diameter_m,
    shell_stations,
    validate_openvsp_api,
    vspaero_reference_quantities,
)
from douglas_dart.vspaero import run_vspaero_sweep


ROOT = Path(__file__).resolve().parents[1]


class FakeOpenVSP:
    ENGINE_GEOM_FLOWTHROUGH = 1
    ENGINE_GEOM_INLET_OUTLET = 2
    ENGINE_LOC_INDEX = 0
    ENGINE_MODE_FLOWTHROUGH = 3
    ROOTC_WSECT_DRIVER = 4
    SET_ALL = 0
    SET_NONE = -1
    SPAN_WSECT_DRIVER = 2
    SYM_NONE = 0
    TIPC_WSECT_DRIVER = 5
    XS_CIRCLE = 7
    MANUAL_REF = 0
    PANEL = 1
    VORTEX_LATTICE = 0

    def __init__(self) -> None:
        self.added_geometries: list[tuple[str, str, str]] = []
        self.named_geometries: dict[str, str] = {}
        self.parameter_calls: list[tuple] = []
        self.driver_calls: list[tuple] = []
        self.changed_xsecs: list[tuple] = []
        self.written_files: list[tuple[str, int]] = []
        self.analysis_int_inputs: list[tuple] = []
        self.analysis_double_inputs: list[tuple] = []
        self.analysis_defaults: list[str] = []
        self.read_files: list[str] = []
        self.sweep_count = 0

    def GetVSPVersion(self):
        return "OpenVSP 3.51.2"

    def VSPRenew(self):
        return None

    def ClearVSPModel(self):
        return None

    def AddGeom(self, geom_type, parent_id=""):
        geom_id = f"geom_{len(self.added_geometries) + 1}"
        self.added_geometries.append((geom_id, geom_type, parent_id))
        return geom_id

    def SetGeomName(self, geom_id, name):
        self.named_geometries[geom_id] = name

    def SetParmVal(self, *args):
        self.parameter_calls.append(tuple(args))
        return float(args[-1])

    def GetXSecSurf(self, geom_id, index):
        return f"{geom_id}_xsec_surf_{index}"

    def GetNumXSec(self, xsec_surf_id):
        return 5

    def ChangeXSecShape(self, xsec_surf_id, index, shape):
        self.changed_xsecs.append((xsec_surf_id, index, shape))

    def GetXSec(self, xsec_surf_id, index):
        return f"{xsec_surf_id}_xsec_{index}"

    def GetXSecParm(self, xsec_id, name):
        return f"{xsec_id}_{name}"

    def SetDriverGroup(self, *args):
        self.driver_calls.append(tuple(args))

    def Update(self):
        return None

    def WriteVSPFile(self, path, geom_set):
        self.written_files.append((path, geom_set))
        Path(path).touch()

    def GetNumTotalErrors(self):
        return 0

    def PopLastError(self):
        raise AssertionError("no errors should be popped")

    def ReadVSPFile(self, path):
        self.read_files.append(path)

    def SetAnalysisInputDefaults(self, analysis):
        self.analysis_defaults.append(analysis)

    def SetIntAnalysisInput(self, *args):
        valid_names = {
            "VSPAEROComputeGeometry": {"GeomSet", "ThinGeomSet"},
            "VSPAEROSweep": {
                "AlphaNpts",
                "AnalysisMethod",
                "BetaNpts",
                "GeomSet",
                "MachNpts",
                "RefFlag",
                "Symmetry",
                "ThinGeomSet",
                "WakeNumIter",
            },
        }
        if args[1] not in valid_names[args[0]]:
            raise AssertionError(f"invalid integer analysis input: {args[:2]}")
        self.analysis_int_inputs.append(tuple(args))

    def SetDoubleAnalysisInput(self, *args):
        self.analysis_double_inputs.append(tuple(args))

    def ExecAnalysis(self, analysis):
        if analysis == "VSPAEROComputeGeometry":
            return "compute_geometry_results"
        self.sweep_count += 1
        return f"sweep_{self.sweep_count}"

    def GetStringResults(self, result_id, name, index=0):
        if name != "ResultsVec":
            return []
        return [f"history_{result_id}"]

    def GetAllDataNames(self, result_id):
        return ["CLtot", "CDtot", "CStot", "CMxtot", "CMytot", "CMztot"]

    def GetDoubleResults(self, result_id, name, index=0):
        values = {
            "CLtot": 0.20,
            "CDtot": 0.02,
            "CStot": 0.01,
            "CMxtot": 0.001,
            "CMytot": -0.03,
            "CMztot": 0.002,
        }
        return [values[name] * 0.9, values[name]]


class OpenVSPGeometryTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )

    def test_body_stations_preserve_intake_body_and_shared_exit(self):
        stations = body_stations(self.case)
        self.assertEqual(len(stations), 5)
        self.assertAlmostEqual(stations[0].diameter_m, 0.195)
        self.assertAlmostEqual(stations[1].diameter_m, 0.205)
        self.assertAlmostEqual(stations[-1].diameter_m, 0.130 * 1.05**0.5)
        self.assertEqual(stations[-1].x_location_m, 2.30)

    def test_candidate_b_geometry_uses_fixed_intake_and_larger_body_and_exit(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_b.yaml"
        )
        stations = body_stations(case)
        self.assertAlmostEqual(stations[0].diameter_m, 0.195)
        self.assertAlmostEqual(stations[1].diameter_m, 0.210)
        self.assertAlmostEqual(stations[-1].diameter_m, 0.170 * 1.05**0.5)
        self.assertEqual(len(case.geometry.mach_values), 6)
        self.assertIn(12.0, case.geometry.alpha_deg_values)

    def test_clocking_is_explicit_and_evenly_spaced(self):
        self.assertEqual(clocking_angles_deg(2, 0.0), (0.0, 180.0))
        self.assertEqual(clocking_angles_deg(4, 45.0), (45.0, 135.0, 225.0, 315.0))

    def test_build_creates_flowthrough_body_two_surfaces_and_four_fins(self):
        fake = FakeOpenVSP()
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "test_candidate.vsp3"
            summary = build_openvsp_geometry(self.case, model_path, vsp=fake)
            self.assertTrue(model_path.is_file())
        self.assertEqual(len(fake.added_geometries), 9)
        self.assertEqual(
            [item[1] for item in fake.added_geometries],
            ["FUSELAGE", "FUSELAGE", "FUSELAGE"] + ["WING"] * 6,
        )
        self.assertEqual(len(fake.changed_xsecs), 15)
        self.assertEqual(len(summary.lifting_surface_ids), 2)
        self.assertEqual(len(summary.fin_ids), 4)
        self.assertTrue(summary.flowthrough_open_end_configuration_requested)
        self.assertTrue(summary.shell_id)
        self.assertTrue(summary.ram_inlet_id)
        self.assertEqual(len(summary.shell_stations), 5)
        self.assertGreater(summary.shell_outer_diameter_m, self.case.vehicle.body_diameter_m)
        self.assertEqual(fake.written_files[-1][1], fake.SET_ALL)
        self.assertIn(
            (
                summary.body_id,
                "GeomIOType",
                "EngineModel",
                float(fake.ENGINE_GEOM_INLET_OUTLET),
            ),
            fake.parameter_calls,
        )

    def test_fin_rotation_is_normalized_into_vsp_xrot_range(self):
        fake = FakeOpenVSP()
        with tempfile.TemporaryDirectory() as directory:
            build_openvsp_geometry(self.case, Path(directory) / "test_candidate.vsp3", vsp=fake)
        rotation_calls = [
            call[-1]
            for call in fake.parameter_calls
            if len(call) == 4 and call[1] == "X_Rel_Rotation" and call[2] == "XForm"
        ]
        self.assertTrue(rotation_calls)
        self.assertTrue(all(-180.0 <= value <= 180.0 for value in rotation_calls))
        self.assertIn(-135.0, rotation_calls)
        self.assertIn(-45.0, rotation_calls)

    def test_fins_and_lifting_surfaces_mount_to_shell_outer_diameter(self):
        fake = FakeOpenVSP()
        with tempfile.TemporaryDirectory() as directory:
            build_openvsp_geometry(self.case, Path(directory) / "test_candidate.vsp3", vsp=fake)
        # A small commanded radial overlap, sized from each surface's own root
        # airfoil thickness, avoids an exact-tangency mesh defect confirmed against
        # the real installed OpenVSP API (see openvsp_geometry.py). The lifting
        # surface sits at 0/180 degree clocking, so its Y offset equals its mount
        # radius directly (no cosine projection like the 45-degree-clocked fins).
        expected_radius_m = 0.5 * shell_outer_diameter_m(self.case) - _radial_mount_overlap_m(
            self.case.geometry.lifting_surface
        )
        y_calls = [
            call[-1]
            for call in fake.parameter_calls
            if len(call) == 4 and call[1] == "Y_Rel_Location" and call[2] == "XForm"
        ]
        self.assertTrue(
            any(abs(abs(value) - expected_radius_m) < 1e-9 for value in y_calls)
        )

    def test_reference_area_is_sum_of_exposed_lifting_surfaces(self):
        references = vspaero_reference_quantities(self.case)
        expected_area_m2 = 2.0 * 0.5 * (0.42 + 0.14) * 0.16
        self.assertAlmostEqual(references.area_m2, expected_area_m2)
        self.assertAlmostEqual(references.span_m, 0.205 + 2.0 * 0.16)
        self.assertGreater(
            references.mean_aerodynamic_chord_m,
            self.case.geometry.lifting_surface.tip_chord_m,
        )
        self.assertLess(
            references.mean_aerodynamic_chord_m,
            self.case.geometry.lifting_surface.root_chord_m,
        )

    def test_namespace_only_openvsp_module_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "not a functional OpenVSP API"):
            validate_openvsp_api(object(), "3.51.2")

    def test_body_diameter_at_x_interpolates_through_aft_taper(self):
        stations = body_stations(self.case)
        aft_taper_start = stations[3]
        exit_station = stations[-1]
        self.assertAlmostEqual(
            body_diameter_at_x_m(self.case, aft_taper_start.x_location_m),
            aft_taper_start.diameter_m,
        )
        self.assertAlmostEqual(
            body_diameter_at_x_m(self.case, exit_station.x_location_m),
            exit_station.diameter_m,
        )
        midpoint_x = 0.5 * (aft_taper_start.x_location_m + exit_station.x_location_m)
        midpoint_diameter = body_diameter_at_x_m(self.case, midpoint_x)
        self.assertLess(midpoint_diameter, aft_taper_start.diameter_m)
        self.assertGreater(midpoint_diameter, exit_station.diameter_m)

    def test_shell_extending_into_aft_taper_fairs_to_local_body_diameter(self):
        # Candidate A's shell now extends past the body's own aft_taper_start_m to
        # keep the fin root on its constant-diameter span (see the YAML comment).
        self.assertGreater(
            self.case.geometry.shell.end_x_m, self.case.geometry.aft_taper_start_m
        )
        stations = shell_stations(self.case)
        expected_end_diameter = body_diameter_at_x_m(
            self.case, self.case.geometry.shell.end_x_m
        )
        self.assertAlmostEqual(stations[-1].diameter_m, expected_end_diameter)
        self.assertLess(stations[-1].diameter_m, shell_outer_diameter_m(self.case))

    def test_fin_root_chord_fits_entirely_within_shell_constant_span(self):
        shell = self.case.geometry.shell
        fin = self.case.geometry.fin
        constant_start = shell.start_x_m + shell.forward_taper_length_m
        constant_end = shell.end_x_m - shell.aft_taper_length_m
        self.assertGreaterEqual(fin.x_location_m, constant_start)
        self.assertLessEqual(fin.x_location_m + fin.root_chord_m, constant_end)

    def test_reference_case_rejects_fin_root_chord_crossing_shell_taper(self):
        bad_shell = replace(self.case.geometry.shell, end_x_m=self.case.geometry.fin.x_location_m)
        bad_geometry = replace(self.case.geometry, shell=bad_shell)
        with self.assertRaisesRegex(ValueError, "entire root chord"):
            replace(self.case, geometry=bad_geometry)

    def test_ram_inlet_is_centered_on_axis_and_flow_through(self):
        fake = FakeOpenVSP()
        with tempfile.TemporaryDirectory() as directory:
            build_openvsp_geometry(self.case, Path(directory) / "test_candidate.vsp3", vsp=fake)
        duct_calls = [call for call in fake.parameter_calls if len(call) == 4]
        y_offsets = {
            call[-1]
            for call in duct_calls
            if call[1] == "Y_Rel_Location" and call[2] == "XForm" and call[-1] == 0.0
        }
        z_offsets = {
            call[-1]
            for call in duct_calls
            if call[1] == "Z_Rel_Location" and call[2] == "XForm" and call[-1] == 0.0
        }
        self.assertIn(0.0, y_offsets)
        self.assertIn(0.0, z_offsets)
        self.assertIn(
            (
                fake.added_geometries[2][0],  # third AddGeom call is the ram inlet duct
                "GeomIOType",
                "EngineModel",
                float(fake.ENGINE_GEOM_INLET_OUTLET),
            ),
            fake.parameter_calls,
        )


class VSPAeroContractTests(unittest.TestCase):
    def test_nonuniform_grid_runs_as_explicit_single_points(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        fake = FakeOpenVSP()
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "candidate.vsp3"
            model_path.touch()
            summary = run_vspaero_sweep(case, model_path, vsp=fake)

        expected_count = (
            len(case.geometry.mach_values)
            * len(case.geometry.alpha_deg_values)
            * len(case.geometry.beta_deg_values)
        )
        self.assertEqual(len(summary.points), expected_count)
        self.assertEqual(fake.sweep_count, expected_count)
        self.assertTrue(all(point.drag_coefficient_inviscid == 0.02 for point in summary.points))

        mach_start_values = [
            call[2][0]
            for call in fake.analysis_double_inputs
            if call[0] == "VSPAEROSweep" and call[1] == "MachStart"
        ]
        self.assertEqual(set(mach_start_values), set(case.geometry.mach_values))
        self.assertEqual(len(mach_start_values), expected_count)
        point_count_inputs = [
            call
            for call in fake.analysis_int_inputs
            if call[0] == "VSPAEROSweep" and call[1].endswith("Npts")
        ]
        self.assertTrue(point_count_inputs)
        self.assertTrue(all(call[2] == [1] for call in point_count_inputs))
        self.assertFalse(
            any(
                call[0] == "VSPAEROComputeGeometry"
                and call[1] == "AnalysisMethod"
                for call in fake.analysis_int_inputs
            )
        )


if __name__ == "__main__":
    unittest.main()
