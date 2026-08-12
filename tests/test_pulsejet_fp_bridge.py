"""pulsejet-fp bridge: spec derivation, drift guards against the sibling
repo's reference design, schema mapping, and the PULSEJET_MODE dispatch
preferring a trustworthy pulsejet-fp answer (fake result -- no transient
sim runs in this file; the one real end-to-end query lives in
test_pulsejet_fp_bridge_live.py so it can be deselected when iterating)."""
from __future__ import annotations

import math
from dataclasses import replace

try:
    import pytest
except ImportError:  # CI runs unittest discover without pytest installed
    import unittest

    raise unittest.SkipTest("bridge tests require pytest (run locally via pytest)")

from douglas_dart import propulsion_map
from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import (
    NOMINAL,
    PULSEJET_MODE,
    _fp_result_to_propulsion_map_point,
    _pulsejet_mode_point,
)
from douglas_dart.pulsejet_fp_bridge import (
    PULSEJET_FP_SCALED_GEOMETRY_FLAG,
    _fp1_chamber_zone_volume_m3,
    derive_pulsejet_fp_spec,
    pulsejet_fp_result_is_trustworthy,
)


@pytest.fixture(scope="module")
def case():
    return load_reference_case("configs/shared_nozzle_candidate_a.yaml")


def _fake_fp_result(**overrides):
    from pulsejet_fp import ThrustResult

    base = dict(
        thrust_n=150.0, status="converged", frequency_hz=57.0,
        thrust_surface_n=151.0, mdot_air_kg_s=0.150, mdot_fuel_kg_s=0.0096,
        tsfc_kg_per_n_hr=0.23, p_min_ratio=0.78, p_max_ratio=1.62,
        rayleigh_index=1e6, n_cycles=14, mach=0.2,
    )
    base.update(overrides)
    return ThrustResult(**base)


def test_fp1_reference_constants_match_sibling_repo():
    """Drift guard: the bridge's hard-coded FP-1 constants must equal the
    live pulsejet_fp reference design."""
    from pulsejet_fp import reference_geometry, reference_valve

    g = reference_geometry()
    assert g.chamber_diameter == pytest.approx(0.078)
    assert g.chamber_length == pytest.approx(0.150)
    assert g.cone_length == pytest.approx(0.130)
    assert g.tailpipe_diameter == pytest.approx(0.042)
    assert g.tailpipe_length == pytest.approx(0.620)
    assert reference_valve().n_petals == 9
    # bridge's analytic chamber-zone volume vs numerical integration of the
    # real A(x): the sibling's cone is cosine-blended, holding ~0.5% more
    # volume than the straight frustum the bridge's formula assumes -- a
    # 1% guard catches real drift while tolerating that known blend offset
    # (which shifts the derived scale factor by only ~0.2%).
    import numpy as np
    x = np.linspace(0.0, g.chamber_zone_length, 20001)
    v_num = float(np.trapezoid(g.area_at(x), x))
    assert _fp1_chamber_zone_volume_m3() == pytest.approx(v_num, rel=1e-2)


def test_spec_scales_isometrically(case):
    spec = derive_pulsejet_fp_spec(case)
    s = (case.pulsejet.chamber_volume_m3 / _fp1_chamber_zone_volume_m3()) ** (1 / 3)
    assert spec.chamber_diameter_m == pytest.approx(0.078 * s)
    assert spec.tailpipe_length_m == pytest.approx(0.620 * s)
    # dynamic similarity: petal GEOMETRY scales isometrically, count fixed
    assert spec.petal_scale == pytest.approx(s)
    assert spec.n_petals == 9
    assert spec.geometry_derived_from_chamber_volume
    # 0.025 m^3 candidate: ~2.84x FP-1 scale
    assert 2.5 < s < 3.2


def test_trustworthiness_guard():
    assert pulsejet_fp_result_is_trustworthy(_fake_fp_result())
    assert not pulsejet_fp_result_is_trustworthy(_fake_fp_result(status="unconverged"))
    assert not pulsejet_fp_result_is_trustworthy(_fake_fp_result(status="quenched"))
    assert not pulsejet_fp_result_is_trustworthy(_fake_fp_result(thrust_n=-2.0))
    assert not pulsejet_fp_result_is_trustworthy(_fake_fp_result(rayleigh_index=-1e5))
    # two thrust formulations wildly apart -> broken run, reject
    assert not pulsejet_fp_result_is_trustworthy(_fake_fp_result(thrust_surface_n=90.0))


def test_schema_mapping_bookkeeping(case):
    spec = derive_pulsejet_fp_spec(case)
    res = _fake_fp_result()
    pt = _fp_result_to_propulsion_map_point(
        res, spec, PULSEJET_MODE, mach=0.2, altitude_m=500.0, scenario=NOMINAL
    )
    assert pt.net_thrust_n == pytest.approx(150.0)
    # gross - drag == net exactly, same bookkeeping both sides
    assert pt.gross_thrust_n - pt.inlet_momentum_drag_n == pytest.approx(pt.net_thrust_n)
    assert pt.inlet_momentum_drag_n > 0.0
    # Isp/TSFC weight-flow convention
    assert pt.specific_impulse_s == pytest.approx(150.0 / (0.0096 * 9.80665))
    assert pt.tsfc_per_hour == pytest.approx(3600.0 / pt.specific_impulse_s)
    # side inlet: installed recovery is the static/stagnation ratio < 1
    assert 0.9 < pt.installed_total_pressure_recovery < 1.0
    assert PULSEJET_FP_SCALED_GEOMETRY_FLAG in pt.validity_flags
    assert pt.mode == PULSEJET_MODE


def test_pulsejet_mode_prefers_trustworthy_fp(case, monkeypatch):
    calls = {}

    def fake_query(spec, mach, altitude_m, fidelity):
        calls["spec"] = spec
        return _fake_fp_result()

    monkeypatch.setenv("DOUGLAS_DART_DISABLE_PULSEJET_FP", "0")
    monkeypatch.setattr(propulsion_map, "run_pulsejet_fp_query", fake_query)
    pt = _pulsejet_mode_point(case, 0.2, 500.0, NOMINAL, pulsejet_fidelity="fast")
    assert pt.net_thrust_n == pytest.approx(150.0)
    assert "pulsejet_fp_status_converged" in pt.validity_flags
    assert "pulsejet_fp_primary_rejected_fell_back" not in pt.validity_flags
    assert calls["spec"].geometry_derived_from_chamber_volume


def test_pulsejet_mode_falls_back_when_fp_untrustworthy(case, monkeypatch):
    def fake_query(spec, mach, altitude_m, fidelity):
        return _fake_fp_result(status="quenched", thrust_n=float("nan"))

    monkeypatch.setenv("DOUGLAS_DART_DISABLE_PULSEJET_FP", "0")
    monkeypatch.setattr(propulsion_map, "run_pulsejet_fp_query", fake_query)
    pt = _pulsejet_mode_point(case, 0.2, 500.0, NOMINAL, pulsejet_fidelity="fast")
    # fell back to the native simulator, visibly
    assert "pulsejet_fp_primary_rejected_fell_back" in pt.validity_flags


def test_pulsejet_mode_survives_fp_crash(case, monkeypatch):
    def exploding_query(spec, mach, altitude_m, fidelity):
        raise RuntimeError("sibling repo broke")

    monkeypatch.setenv("DOUGLAS_DART_DISABLE_PULSEJET_FP", "0")
    monkeypatch.setattr(propulsion_map, "run_pulsejet_fp_query", exploding_query)
    pt = _pulsejet_mode_point(case, 0.2, 500.0, NOMINAL, pulsejet_fidelity="fast")
    assert "pulsejet_fp_primary_rejected_fell_back" in pt.validity_flags
    assert math.isfinite(pt.net_thrust_n)
