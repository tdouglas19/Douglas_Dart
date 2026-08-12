import math

from pulsejet_fp.orifice import critical_ratio, mass_flux


def test_critical_ratio_gamma14():
    assert abs(critical_ratio(1.4) - 0.5283) < 1e-3


def test_no_flow_without_pressure_drop():
    G, u, _ = mass_flux(1e5, 300.0, 1.4, 287.0, 1e5)
    assert G == 0.0 and u == 0.0


def test_choked_flux_matches_closed_form():
    # G* = p0 sqrt(gamma/RT0) (2/(g+1))^((g+1)/(2(g-1))) -- derived by
    # substituting r* into eq. 22; independent algebraic path
    p0, T0, g, R = 3e5, 400.0, 1.4, 287.0
    G, u, rho_t = mass_flux(p0, T0, g, R, 0.3 * p0)  # deeply choked
    G_star = p0 * math.sqrt(g / (R * T0)) * (2.0 / (g + 1.0)) ** (
        (g + 1.0) / (2.0 * (g - 1.0)))
    assert abs(G - G_star) / G_star < 1e-10
    # throat is exactly sonic when choked
    T_t = T0 * (2.0 / (g + 1.0))
    assert abs(u - math.sqrt(g * R * T_t)) / u < 1e-10


def test_flux_continuous_at_choking():
    p0, T0, g, R = 2e5, 350.0, 1.4, 287.0
    rc = critical_ratio(g)
    G1, _, _ = mass_flux(p0, T0, g, R, (rc + 1e-6) * p0)
    G2, _, _ = mass_flux(p0, T0, g, R, (rc - 1e-6) * p0)
    assert abs(G1 - G2) / G2 < 1e-4


def test_subcritical_bernoulli_limit():
    # tiny pressure drop: G -> rho0 * sqrt(2 dp / rho0) (incompressible)
    p0, T0, g, R = 1e5, 300.0, 1.4, 287.0
    dp = 200.0
    G, u, _ = mass_flux(p0, T0, g, R, p0 - dp)
    rho0 = p0 / (R * T0)
    u_bern = math.sqrt(2.0 * dp / rho0)
    assert abs(u - u_bern) / u_bern < 0.01
