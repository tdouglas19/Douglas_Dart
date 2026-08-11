import numpy as np

from pulsejet_fp.gas import propane_air


def test_stoichiometric_ratio_derived():
    g = propane_air(phi=1.0)
    # f_st = W_C3H8 / (5 W_O2) * Y_O2  ~= 0.0638 (derivation eq. 3)
    assert abs(g.f - 0.0638) < 0.0015


def test_molar_masses():
    g = propane_air(phi=1.0)
    assert 0.0290 < g.W_R < 0.0301   # heavier than air (propane W=44)
    assert 0.0280 < g.W_P < 0.0290   # mole count rises on reaction
    assert g.W_P < g.W_R


def test_gammas():
    g = propane_air()
    assert abs(g.gamma_R - 1.4) < 1e-9      # diatomic equipartition
    assert abs(g.gamma_P - 9.0 / 7.0) < 1e-9


def test_adiabatic_flame_temperature_plausible():
    g = propane_air(phi=1.0)
    # constant-pressure adiabatic flame from 300 K:
    # h balance: cp_P * T_ad = cp_R * T0 + q_R
    T_ad = (g.cp_R * 300.0 + g.q_R) / g.cp_P
    # true propane-air ~2267 K; dissociation neglected (A3) biases high
    assert 2150.0 < T_ad < 2650.0


def test_rich_mixture_burns_oxygen_limited():
    lean = propane_air(phi=0.8)
    stoi = propane_air(phi=1.0)
    rich = propane_air(phi=1.3)
    assert lean.q_R < stoi.q_R
    # rich: more fuel mass in mixture but only 1/phi of it burns
    assert rich.q_R < stoi.q_R


def test_mixture_rules_vectorized():
    g = propane_air()
    Y = np.array([0.0, 0.5, 1.0])
    cv = g.cv_mix(Y)
    assert cv[0] == g.cv_P and cv[2] == g.cv_R
    T = g.T_from_e(g.e_from_T(np.array([300.0, 1000.0, 2000.0]), Y), Y)
    assert np.allclose(T, [300.0, 1000.0, 2000.0])


def test_arrhenius_cold_frozen_hot_fast():
    g = propane_air()
    assert g.reaction_rate(300.0, 1.0) < 1e-9      # fresh charge inert
    assert g.reaction_rate(2000.0, 1.0) > 1e3       # flame-zone fast
    r1200 = g.reaction_rate(1200.0, 1.0)
    assert 1e1 < r1200 < 1e5                        # ignition range ~ms
