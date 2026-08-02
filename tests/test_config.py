import unittest
from pathlib import Path

from douglas_dart.config import SelectorConfig, load_reference_case


ROOT = Path(__file__).resolve().parents[1]


class ConfigTests(unittest.TestCase):
    def test_user_half_area_requirement(self):
        selector = SelectorConfig(0.3, 0.5, 0.8, 0.9)
        self.assertAlmostEqual(selector.available_area_m2, 0.5 * selector.circular_area_m2)

    def test_more_than_half_open_is_rejected(self):
        with self.assertRaises(ValueError):
            SelectorConfig(0.3, 0.6, 0.8, 0.9)

    def test_reference_case_loads(self):
        case = load_reference_case(ROOT / "configs" / "reference_case.yaml")
        self.assertEqual(case.fuel.key, "jet_a_reference")
        self.assertGreater(case.nozzle.exit_area_m2, case.nozzle.throat_area_m2)


if __name__ == "__main__":
    unittest.main()
