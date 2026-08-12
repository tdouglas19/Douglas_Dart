"""One real end-to-end ramjet-fp query through the Gate 2 dispatch and the
Gate 3 lazy table (fast fidelity, ~15-60 s each with a warm numba cache;
kept separate from test_ramjet_fp_bridge.py so iteration can deselect it)."""
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
    RAMJET_FP_BLOWN_OFF_FLAG,
    derive_ramjet_fp_spec,
    get_ramjet_fp_mission_table,
)


@pytest.fixture(scope="module")
def case():
    return load_reference_case("configs/shared_nozzle_candidate_a.yaml")


def test_live_gate2_point_reports_lean_blowoff(case, monkeypatch):
    """The vehicle config's phi=0.60 premixed flame does not hold at
    M=1.1 (ramjet-fp architecture.md #3-#5): the map must carry that
    verdict with the cold-throughflow drag, not a lit thrust number."""
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    pt = _ramjet_point(case, 1.1, 0.0, NOMINAL)
    assert "ramjet_fp_status_blown_off" in pt.validity_flags
    assert RAMJET_FP_BLOWN_OFF_FLAG in pt.validity_flags
    assert pt.self_sustaining_status is False
    assert pt.net_thrust_n < 0.0        # cold-throughflow drag
    assert -200.0 < pt.net_thrust_n     # and only drag, not garbage


def test_live_gate3_table_interpolates(case, monkeypatch):
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    table = get_ramjet_fp_mission_table(derive_ramjet_fp_spec(case), "fast")
    thrust, fuel, flame = table.query(1.12, 100.0)
    assert math.isfinite(thrust) and math.isfinite(fuel)
    assert flame is False               # phi=0.60: blown off here too
    # memoized corners: the repeat query is effectively free
    thrust2, fuel2, flame2 = table.query(1.13, 200.0)
    assert math.isfinite(thrust2)
