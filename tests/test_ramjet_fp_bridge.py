"""ramjet-fp bridge: spec derivation with drift guards against the sibling
repo's RJ-1 reference design, the gutter/throat cap, the usability guard,
schema mapping, and RAMJET_MODE's guarded-primary dispatch (fake results --
no transient sim runs here; the one real end-to-end query lives in
test_ramjet_fp_bridge_live.py so it can be deselected when iterating)."""
from __future__ import annotations

import math

try:
    import pytest
except ImportError:  # CI runs unittest discover without pytest installed
    import unittest

    raise unittest.SkipTest("bridge tests require pytest (run locally via pytest)")

from douglas_dart import propulsion_map
from douglas_dart.config import load_reference_case
from douglas_dart.propulsion_map import (
    NOMINAL,
    RAMJET_MODE,
    _ramjet_fp_result_to_point,
    _ramjet_point,
)
from douglas_dart.ramjet_fp_bridge import (
    RAMJET_FP_BLOWN_OFF_FLAG,
    RAMJET_FP_FALLBACK_FLAG,
    RAMJET_FP_GUTTER_CAPPED_FLAG,
    RAMJET_FP_RECOVERY_OUTPUT_FLAG,
    derive_ramjet_fp_spec,
    ramjet_fp_result_is_usable,
)


@pytest.fixture(scope="module")
def case():
    return load_reference_case("configs/shared_nozzle_candidate_a.yaml")


@pytest.fixture(scope="module")
def case_b():
    return load_reference_case("configs/shared_nozzle_candidate_b.yaml")


def _fake_fp_result(**overrides):
    from ramjet_fp import RamjetResult

    base = dict(
        net_thrust_n=1150.0, status="oscillatory", mach=1.1, altitude_m=0.0,
        gross_thrust_n=2400.0, thrust_surface_n=1160.0,
        mdot_air_kg_s=2.20, mdot_fuel_kg_s=0.150,
        tsfc_kg_per_n_hr=0.47, isp_s=780.0,
        spillage_fraction=0.83, recovery_p0=0.998, shock_recovery=0.999,
        combustion_efficiency=1.0, peak_combustor_mach=0.47,
        t_rz_k=2340.0, flame_stable=True,
        oscillation_amplitude_n=1250.0, oscillation_freq_hz=750.0,
    )
    base.update(overrides)
    return RamjetResult(**base)


def test_rj1_reference_constants_match_sibling_repo():
    """Drift guard: the bridge's RJ-1 proportions must equal the live
    ramjet_fp reference design."""
    from ramjet_fp import reference_flameholder, reference_geometry

    g = reference_geometry()
    assert g.lip_diameter == pytest.approx(0.195)
    assert g.combustor_diameter == pytest.approx(0.190)
    assert g.throat_diameter == pytest.approx(0.130)
    assert g.exit_area_ratio == pytest.approx(1.05)
    assert g.x_exit == pytest.approx(1.62)
    fh = reference_flameholder()
    assert fh.gutter_width == pytest.approx(0.025)
    assert fh.frontal_area == pytest.approx(2.0 * math.pi * 0.060 * 0.025)
    assert fh.shear_perimeter == pytest.approx(4.0 * math.pi * 0.060)


def test_spec_reproduces_rj1_for_candidate_a(case):
    """Candidate A IS the RJ-1 scale: the derived spec must round-trip."""
    spec = derive_ramjet_fp_spec(case)
    assert spec.lip_diameter_m == pytest.approx(0.195)
    assert spec.combustor_diameter_m == pytest.approx(0.190)
    assert spec.throat_diameter_m == pytest.approx(0.130)
    assert spec.x_exit_m == pytest.approx(1.62)
    assert spec.gutter_width_m == pytest.approx(0.025, rel=1e-6)
    assert not spec.gutter_capped_by_throat
    assert spec.equivalence_ratio == pytest.approx(0.60)


def test_gutter_capped_by_big_throat(case_b):
    """Candidate B's 0.170 m shared throat forces a slim gutter: the
    minimum flow area past the gutter must keep >=15% margin over the
    throat (a gutter that chokes ahead of the nozzle is a different
    engine, not a bigger flameholder)."""
    spec = derive_ramjet_fp_spec(case_b)
    assert spec.gutter_capped_by_throat
    a_c = 0.25 * math.pi * spec.combustor_diameter_m ** 2
    a_th = 0.25 * math.pi * spec.throat_diameter_m ** 2
    assert a_c - spec.gutter_frontal_area_m2 >= 1.15 * a_th - 1e-9


def test_usability_guard():
    assert ramjet_fp_result_is_usable(_fake_fp_result())
    # blown_off IS usable physics (flame won't hold; cold drag reported)
    assert ramjet_fp_result_is_usable(_fake_fp_result(
        status="blown_off", flame_stable=False, net_thrust_n=-21.0,
        thrust_surface_n=-21.0))
    assert not ramjet_fp_result_is_usable(_fake_fp_result(status="unconverged"))
    assert not ramjet_fp_result_is_usable(_fake_fp_result(net_thrust_n=float("nan")))
    # the two independent thrust formulations wildly apart -> broken run
    assert not ramjet_fp_result_is_usable(_fake_fp_result(thrust_surface_n=500.0))


def test_schema_mapping_bookkeeping(case):
    spec = derive_ramjet_fp_spec(case)
    pt = _ramjet_fp_result_to_point(
        _fake_fp_result(), spec, mach=1.1, altitude_m=0.0, scenario=NOMINAL
    )
    assert pt.net_thrust_n == pytest.approx(1150.0)
    assert pt.gross_thrust_n - pt.inlet_momentum_drag_n == pytest.approx(pt.net_thrust_n)
    assert pt.specific_impulse_s == pytest.approx(1150.0 / (0.150 * 9.80665))
    assert pt.tsfc_per_hour == pytest.approx(3600.0 / pt.specific_impulse_s)
    # spillage/potential reconstruction
    assert pt.spilled_mass_flow_fraction == pytest.approx(0.83)
    assert pt.potential_air_mass_flow_kg_per_s == pytest.approx(2.20 / 0.17)
    # recovery is the model's OUTPUT, flagged as such
    assert pt.installed_total_pressure_recovery == pytest.approx(0.998)
    assert RAMJET_FP_RECOVERY_OUTPUT_FLAG in pt.validity_flags
    assert pt.self_sustaining_status is True
    assert pt.mode == RAMJET_MODE


def test_blown_off_maps_honestly(case):
    spec = derive_ramjet_fp_spec(case)
    pt = _ramjet_fp_result_to_point(
        _fake_fp_result(status="blown_off", flame_stable=False,
                        net_thrust_n=-21.4, thrust_surface_n=-21.4,
                        gross_thrust_n=2150.0, combustion_efficiency=0.0),
        spec, mach=1.1, altitude_m=0.0, scenario=NOMINAL,
    )
    assert pt.net_thrust_n == pytest.approx(-21.4)
    assert pt.self_sustaining_status is False
    assert RAMJET_FP_BLOWN_OFF_FLAG in pt.validity_flags
    assert pt.lightoff_status == "ramjet_fp_flame_out_blown_off"
    assert pt.tsfc_per_hour is None      # no TSFC for negative thrust


def test_ramjet_mode_prefers_usable_fp(case, monkeypatch):
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    monkeypatch.setattr(propulsion_map, "run_ramjet_fp_query",
                        lambda spec, mach, alt, fid: _fake_fp_result())
    pt = _ramjet_point(case, 1.1, 0.0, NOMINAL)
    assert pt.net_thrust_n == pytest.approx(1150.0)
    assert "ramjet_fp_status_oscillatory" in pt.validity_flags
    assert RAMJET_FP_FALLBACK_FLAG not in pt.validity_flags


def test_ramjet_mode_falls_back_when_fp_unusable(case, monkeypatch):
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    monkeypatch.setattr(propulsion_map, "run_ramjet_fp_query",
                        lambda spec, mach, alt, fid: _fake_fp_result(
                            status="unconverged"))
    pt = _ramjet_point(case, 1.1, 0.0, NOMINAL)
    assert RAMJET_FP_FALLBACK_FLAG in pt.validity_flags
    assert math.isfinite(pt.net_thrust_n)


def test_ramjet_mode_survives_fp_crash(case, monkeypatch):
    def exploding_query(spec, mach, alt, fid):
        raise RuntimeError("sibling repo broke")

    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "0")
    monkeypatch.setattr(propulsion_map, "run_ramjet_fp_query", exploding_query)
    pt = _ramjet_point(case, 1.1, 0.0, NOMINAL)
    assert RAMJET_FP_FALLBACK_FLAG in pt.validity_flags
    assert math.isfinite(pt.net_thrust_n)


def test_kill_switch_restores_native_without_flags(case, monkeypatch):
    monkeypatch.setenv("DOUGLAS_DART_DISABLE_RAMJET_FP", "1")
    pt = _ramjet_point(case, 1.1, 0.0, NOMINAL)
    assert RAMJET_FP_FALLBACK_FLAG not in pt.validity_flags
    assert not any(f.startswith("ramjet_fp_status") for f in pt.validity_flags)
