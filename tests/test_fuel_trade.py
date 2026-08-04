import unittest
from pathlib import Path

from douglas_dart.config import load_fuels, load_reference_case
from douglas_dart.fuel_trade import fuel_performance_trade


ROOT = Path(__file__).resolve().parents[1]


class FuelTradeTests(unittest.TestCase):
    def test_trade_reports_all_fuels_without_automatic_ranking(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        fuels = load_fuels(ROOT / "configs" / "fuels.yaml")
        points = fuel_performance_trade(case, fuels)
        self.assertEqual({point.fuel_key for point in points}, set(fuels))
        self.assertTrue(
            all(
                "performance_only_no_automatic_fuel_ranking" in point.status
                for point in points
            )
        )
        self.assertTrue(
            all(not point.storage_and_feed_system_mass_included for point in points)
        )

    def test_volumetric_and_gravimetric_results_follow_configured_properties(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        points = {
            point.fuel_key: point
            for point in fuel_performance_trade(
                case,
                load_fuels(ROOT / "configs" / "fuels.yaml"),
            )
        }
        self.assertGreater(
            points["propane_reference"].loaded_fuel_volume_l,
            points["jet_a_reference"].loaded_fuel_volume_l,
        )
        self.assertGreater(
            points["jet_a_reference"].loaded_chemical_energy_mj,
            points["ethanol_reference"].loaded_chemical_energy_mj,
        )
        self.assertIn(
            "pressurized_liquid_storage_hardware_not_modeled",
            points["propane_reference"].status,
        )


if __name__ == "__main__":
    unittest.main()
