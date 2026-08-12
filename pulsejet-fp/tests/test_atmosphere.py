from pulsejet_fp.atmosphere import ambient, ram_state


def test_sea_level():
    p, T = ambient(0.0)
    assert abs(p - 101325.0) < 1.0
    assert abs(T - 288.15) < 1e-6


def test_5km():
    p, T = ambient(5000.0)
    assert abs(T - (288.15 - 32.5)) < 1e-6
    assert abs(p - 54000.0) < 800.0  # hydrostatic integral, ~54 kPa


def test_ram_state_static():
    p0, T0, u = ram_state(101325.0, 288.15, 0.0)
    assert p0 == 101325.0 and T0 == 288.15 and u == 0.0


def test_ram_state_m07():
    p0, T0, u = ram_state(101325.0, 288.15, 0.7)
    assert abs(T0 / 288.15 - 1.098) < 1e-3       # 1 + 0.2*0.49
    assert abs(p0 / 101325.0 - 1.387) < 5e-3     # isentropic
    assert abs(u - 0.7 * 340.3) < 1.0
