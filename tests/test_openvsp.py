from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.openvsp_geometry import (
    body_stations,
    build_openvsp_geometry,
    clocking_angles_deg,
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

    def test_clocking_is_explicit_and_evenly_spaced(self):
        self.assertEqual(clocking_angles_deg(2, 0.0), (0.0, 180.0))
        self.assertEqual(clocking_angles_deg(4, 45.0), (45.0, 135.0, 225.0, 315.0))

    def test_build_creates_flowthrough_body_two_surfaces_and_four_fins(self):
        fake = FakeOpenVSP()
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "test_candidate.vsp3"
            summary = build_openvsp_geometry(self.case, model_path, vsp=fake)
            self.assertTrue(model_path.is_file())
        self.assertEqual(len(fake.added_geometries), 7)
        self.assertEqual([item[1] for item in fake.added_geometries], ["FUSELAGE"] + ["WING"] * 6)
        self.assertEqual(len(fake.changed_xsecs), 5)
        self.assertEqual(len(summary.lifting_surface_ids), 2)
        self.assertEqual(len(summary.fin_ids), 4)
        self.assertTrue(summary.flowthrough_open_end_configuration_requested)
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
