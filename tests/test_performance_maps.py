import unittest

from douglas_dart.performance_maps import RectilinearEngineMap


class PerformanceMapTests(unittest.TestCase):
    def setUp(self):
        self.engine_map = RectilinearEngineMap(
            altitudes_m=(0.0, 1000.0),
            mach_values=(0.0, 1.0),
            net_thrust_rows_n=((0.0, 10.0), (10.0, 20.0)),
            fuel_flow_rows_kg_per_s=((0.0, 0.1), (0.1, 0.2)),
        )

    def test_bilinear_interpolation_recovers_linear_field(self):
        value = self.engine_map.evaluate(500.0, 0.5)
        self.assertAlmostEqual(value.net_thrust_n, 10.0)
        self.assertAlmostEqual(value.fuel_mass_flow_kg_per_s, 0.1)
        self.assertFalse(value.clamped_to_map_boundary)

    def test_out_of_domain_query_clamps_visibly(self):
        value = self.engine_map.evaluate(-100.0, 0.5)
        self.assertAlmostEqual(value.net_thrust_n, 5.0)
        self.assertTrue(value.clamped_to_map_boundary)

    def test_grid_shape_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            RectilinearEngineMap(
                altitudes_m=(0.0, 1000.0),
                mach_values=(0.0, 1.0),
                net_thrust_rows_n=((1.0,), (2.0,)),
                fuel_flow_rows_kg_per_s=((0.1, 0.1), (0.1, 0.1)),
            )


if __name__ == "__main__":
    unittest.main()
