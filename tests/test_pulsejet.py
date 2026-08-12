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
        self.assertTrue(
            all(
                abs(
                    sample.net_thrust_n
                    - (sample.gross_thrust_n - sample.inlet_momentum_drag_n)
                )
                < 1e-12
                for sample in samples
            )
        )

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
        self.assertLessEqual(summary.mean_net_thrust_n, summary.mean_gross_thrust_n)

    def test_pulsejet_recovery_is_total_pressure_ratio(self):
        simulator = PulsejetSimulator(
            self.case.pulsejet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.altitude_m,
            self.case.mach,
        )
        from douglas_dart.compressible import stagnation_pressure
        from douglas_dart.pulsejet import side_inlet_ram_recovery_ratio

        full_ram_total_pressure_pa = stagnation_pressure(
            simulator.atmosphere.pressure_pa,
            self.case.mach,
        )
        # reference_case.yaml configures inlet_type: side. PulsejetSimulator
        # now recomputes the side-inlet ram-recovery ratio every step() from
        # the inertance model's own captured mass flow (Hall & Frank, NACA RM
        # A8I29 -- see side_inlet_ram_recovery_ratio), but this simulator is
        # fresh (no step() called yet), so __init__ has only initialized it
        # at the zero-flow anchor -- replicate that here rather than
        # comparing against the full-ram (straight-inlet) stagnation pressure.
        ideal_total_pressure_pa = full_ram_total_pressure_pa
        if self.case.selector.inlet_type == "side":
            ram_pressure_rise_pa = full_ram_total_pressure_pa - simulator.atmosphere.pressure_pa
            ideal_total_pressure_pa = (
                simulator.atmosphere.pressure_pa
                + side_inlet_ram_recovery_ratio(0.0) * ram_pressure_rise_pa
            )
        self.assertAlmostEqual(
            simulator.inlet_total_pressure_pa / ideal_total_pressure_pa,
            self.case.selector.pulsejet_total_pressure_recovery,
            places=12,
        )

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
        self.assertAlmostEqual(
            audit.cumulative_heat_rejected_j,
            audit.cumulative_wall_heat_rejected_j
            + audit.cumulative_temperature_limit_heat_rejected_j,
        )

    def test_steady_window_excludes_initial_charged_chamber_bias(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        simulator = PulsejetSimulator(
            case.pulsejet,
            case.selector,
            case.nozzle,
            case.fuel,
            case.altitude_m,
            case.mach,
        )
        samples = simulator.run(0.50, 0.00004)
        full = summarize_pulsejet(samples)
        steady = summarize_pulsejet(samples, minimum_time_s=0.25)
        self.assertGreater(full.mean_net_thrust_n, steady.mean_net_thrust_n)
        self.assertAlmostEqual(steady.window_start_s, 0.25, delta=0.00004)
        self.assertLess(steady.completed_cycles, full.completed_cycles)

    def test_steady_mean_net_thrust_converges_with_time_step(self):
        case = load_reference_case(
            ROOT / "configs" / "shared_nozzle_candidate_a.yaml"
        )
        means = []
        for time_step_s in (0.00004, 0.00002):
            simulator = PulsejetSimulator(
                case.pulsejet,
                case.selector,
                case.nozzle,
                case.fuel,
                case.altitude_m,
                case.mach,
            )
            samples = simulator.run(0.50, time_step_s)
            means.append(
                summarize_pulsejet(
                    samples,
                    minimum_time_s=0.25,
                ).mean_net_thrust_n
            )
        relative_change = abs(means[1] - means[0]) / abs(means[1])
        self.assertLess(relative_change, 0.02)


if __name__ == "__main__":
    unittest.main()
