"""One REAL end-to-end pulsejet-fp query through PULSEJET_MODE's dispatch
at the vehicle candidate's own (scaled) geometry. Slow (~1-3 min including
the one-time numba compile) -- kept in its own file so it is easy to
deselect while iterating: pytest -q --ignore=tests/test_pulsejet_fp_bridge_live.py
"""
from __future__ import annotations

import math

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import NOMINAL, _pulsejet_mode_point


def test_vehicle_candidate_fp_point_end_to_end(monkeypatch):
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_PULSEJET_FP", "0")
    case = load_reference_case("configs/shared_nozzle_candidate_a.yaml")
    pt = _pulsejet_mode_point(case, 0.2, 500.0, NOMINAL, pulsejet_fidelity="fast")

    # the first-principles primary answered (no fallback flag), and the
    # answer is physically coherent for a ~25 L chamber-class pulsejet
    assert "pulsejet_fp_primary_rejected_fell_back" not in pt.validity_flags
    assert "pulsejet_fp_status_converged" in pt.validity_flags
    assert math.isfinite(pt.net_thrust_n)
    assert 20.0 < pt.net_thrust_n < 2000.0
    assert pt.gross_thrust_n > pt.net_thrust_n  # momentum drag charged
    assert pt.fuel_mass_flow_kg_per_s > 0.0
    assert pt.tsfc_per_hour is not None and pt.tsfc_per_hour > 0.0
    assert pt.peak_chamber_pressure_pa is not None
    assert pt.peak_chamber_pressure_pa > 1.1e5  # above ambient: real cycling
