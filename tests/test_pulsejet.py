import unittest
from pathlib import Path

from douglas_dart.config import load_reference_case
from douglas_dart.pulsejet import PulsejetSimulator, summarize_pulsejet


ROOT = Path(__file__).resolve().parents[1]


class PulsejetTests(unittest.TestCase):
    def setUp(self):
        self.case = load_reference_case(ROOT / "configs" / "reference_case.yaml")

    def test_short_run_remains_finite_and_positive(self):
        simulator = PulsejetSimulator(
            self.case.pulsejet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.altitude_m,
            self.case.mach,
        )
        samples = simulator.run(0.03, 0.00002)
        self.assertGreater(len(samples), 100)
        self.assertTrue(all(sample.chamber_pressure_pa > 0.0 for sample in samples))
        self.assertTrue(all(sample.chamber_temperature_k > 0.0 for sample in samples))
        self.assertGreaterEqual(samples[-1].cycle_count, 1)
        phases = {sample.phase for sample in samples}
        self.assertIn("combustion", phases)
        self.assertIn("blowdown", phases)
        self.assertIn("refill", phases)
        self.assertTrue(any(sample.event == "ignition" for sample in samples))
        pressures = [sample.chamber_pressure_pa for sample in samples]
        self.assertGreater(max(pressures), 2.0 * min(pressures))

    def test_summary_marks_reference_only(self):
        simulator = PulsejetSimulator(
            self.case.pulsejet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.altitude_m,
            self.case.mach,
        )
        summary = summarize_pulsejet(simulator.run(0.01, 0.00002))
        self.assertTrue(summary.numerical_reference_only)
        self.assertGreater(summary.peak_chamber_pressure_pa, 101_325.0)

    def test_control_volume_mass_and_energy_ledgers_close(self):
        simulator = PulsejetSimulator(
            self.case.pulsejet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.altitude_m,
            self.case.mach,
        )
        simulator.run(0.05, self.case.simulation.time_step_s)
        audit = simulator.conservation_audit()
        self.assertLess(abs(audit.relative_mass_balance_residual), 1e-12)
        self.assertLess(abs(audit.relative_energy_balance_residual), 1e-12)
        self.assertGreater(audit.cumulative_air_ingested_kg, 0.0)
        self.assertGreater(audit.cumulative_exhaust_discharged_kg, 0.0)
        self.assertGreater(audit.cumulative_combustion_heat_added_j, 0.0)


if __name__ == "__main__":
    unittest.main()
