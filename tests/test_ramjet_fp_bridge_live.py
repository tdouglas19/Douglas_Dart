"""Real end-to-end ramjet-fp operating-point queries through the Gate 2
dispatch and the Gate 3 lazy table (fast fidelity; the phi search runs
4-9 transients per point, ~1-3 min each test with a warm numba cache;
kept separate so iteration can deselect them)."""
from __future__ import annotations

import math

try:
    import pytest
except ImportError:
    import unittest

    raise unittest.SkipTest("bridge tests require pytest (run locally via pytest)")

from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import NOMINAL, _ramjet_point
from douglas_dart.ramjet_fp_bridge import (
    derive_ramjet_fp_spec,
    get_ramjet_fp_mission_table,
)


@pytest.fixture(scope="module")
def case():
    return load_reference_case("configs/shared_nozzle_candidate_a.yaml")


def test_live_gate2_point_self_selects_mixture(case, monkeypatch):
    """M=1.1 sea level, candidate A: the engine must self-select a
    near-stoich mixture (campaign phi_min ~ 0.98 there), light, and
    report positive thrust WITH the required phi as an output -- the
    config's target_equivalence_ratio (0.60, unviable premixed) must
    play no role in the FP path."""
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    pt = _ramjet_point(case, 1.1, 0.0, NOMINAL)
    assert pt.self_sustaining_status is True
    assert pt.ramjet_required_equivalence_ratio is not None
    assert 0.90 <= pt.ramjet_required_equivalence_ratio <= 1.00
    assert pt.net_thrust_n > 200.0
    assert pt.fuel_mass_flow_kg_per_s > 0.01
    assert pt.lightoff_status.startswith("ramjet_fp_flame_stable_phi_")


def test_live_gate3_table_interpolates(case, monkeypatch):
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    table = get_ramjet_fp_mission_table(derive_ramjet_fp_spec(case), "fast")
    thrust, fuel, flame = table.query(1.12, 100.0)
    assert math.isfinite(thrust) and math.isfinite(fuel)
    assert flame                     # self-selected mixture holds here
    assert thrust > 0.0 and fuel > 0.0
    # memoized corners: the repeat query reuses them
    thrust2, fuel2, flame2 = table.query(1.13, 200.0)
    assert math.isfinite(thrust2)
