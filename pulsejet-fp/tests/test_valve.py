import math

from pulsejet_fp.valve import PetalValveDesign, PetalValveState


def design(**kw):
    base = dict(n_petals=9, petal_length=0.020, petal_width=0.014,
                petal_thickness=0.20e-3, port_area=0.012 * 0.0155)
    base.update(kw)
    return PetalValveDesign(**base)


def test_stiffness_matches_hand_calculation():
    d = design()
    I = 0.014 * (0.20e-3) ** 3 / 12.0
    k = 3.0 * 200e9 * I / 0.020 ** 3
    assert abs(d.stiffness - k) / k < 1e-12


def test_effective_mass_fraction():
    d = design()
    assert abs(d.effective_mass / d.petal_mass - 33.0 / 140.0) < 1e-12


def test_natural_frequency_plausible():
    # spring-steel reed of these dimensions: hundreds of Hz
    f = design().natural_frequency_hz
    assert 150.0 < f < 1500.0


def test_curtain_area_monotonic_and_capped():
    d = design()
    a1 = d.curtain_area(0.5e-3)
    a2 = d.curtain_area(1.5e-3)
    assert 0 < a1 < a2 <= d.n_petals * d.port_area
    big = d.curtain_area(1.0)  # absurd lift: capped at total port area
    assert abs(big - d.n_petals * d.port_area) < 1e-12


def test_free_vibration_frequency():
    """Small oscillation about a lifted equilibrium (away from the seat and
    stop, so contact never fires): frequency must match sqrt(k/m)/2pi."""
    d = design(damping_ratio=0.0)
    v = PetalValveState(d)
    # constant pressure differential -> equilibrium lift Q/k, mid-range
    from pulsejet_fp.valve import PSI_INT
    xi_eq = 1.5e-3
    dp = d.stiffness * xi_eq / (PSI_INT * d.petal_face_area)
    v.lift = xi_eq + 0.5e-3  # perturb; stays within (0, max_lift)
    dt = 1e-7
    n = int(0.02 / dt)
    crossings = []
    prev = v.lift - xi_eq
    for i in range(n):
        v.step(dt, dp, 0.0, 0.0, 0.0, 0.0)  # p0_up-p_head=dp, no drag
        cur = v.lift - xi_eq
        if (prev <= 0 < cur) or (prev > 0 >= cur):
            crossings.append(i * dt)
        prev = cur
    import numpy as np
    gaps = np.diff(crossings)
    f_meas = 1.0 / (2.0 * float(np.mean(gaps)))
    f_th = d.natural_frequency_hz
    assert abs(f_meas - f_th) / f_th < 0.02


def test_contact_restitution():
    d = design(restitution=0.5)
    v = PetalValveState(d)
    v.lift = -1e-4  # penetrated seat
    v.lift_rate = -2.0
    v.step(1e-9, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert v.lift >= 0.0
    assert v.lift_rate > 0.0  # bounced with reduced speed
    assert v.lift_rate <= 1.0 + 1e-6


def test_swept_volume_positive():
    d = design()
    assert d.swept_volume(2e-3) > 0
    assert d.swept_volume(0.0) == 0.0
