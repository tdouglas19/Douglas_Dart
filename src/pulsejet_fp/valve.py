"""Petal-valve structural dynamics from Euler-Bernoulli beam theory
(derivation.md #6). No empirical discharge coefficients or delay times:
stiffness, effective mass, forcing, open area, and swept volume all follow
from the beam's derived mode shape psi(s) = (3 s^2 - s^3)/2.

Mode-shape integrals (exact):
  int psi     ds = 3/8
  int psi^2   ds = 33/140
"""
from __future__ import annotations

from dataclasses import dataclass
import math

PSI_INT = 3.0 / 8.0
PSI2_INT = 33.0 / 140.0


@dataclass(frozen=True)
class PetalValveDesign:
    n_petals: int
    petal_length: float      # m, cantilever free length L_v
    petal_width: float       # m, w_v
    petal_thickness: float   # m, h_v
    port_area: float         # m^2 per petal (the hole the petal covers)
    youngs_modulus: float = 200e9   # spring steel
    density: float = 7850.0         # kg/m^3
    damping_ratio: float = 0.03     # A15
    restitution: float = 0.3        # A16
    max_lift: float = 3.5e-3        # m, mechanical stop
    # Seat preload: petals are manufactured with residual curvature and sit
    # pressed against the seat with deflection xi_0 already stored in the
    # spring -- the standard reed-valve cracking-pressure mechanism (A25).
    # The valve opens only when the pressure force exceeds k*xi_0.
    seat_preload: float = 0.0       # m, equivalent preload deflection xi_0

    @property
    def cracking_pressure(self) -> float:
        """dP needed to lift off the seat: k xi_0 / (3/8 A_petal)."""
        return self.stiffness * self.seat_preload / (PSI_INT * self.petal_face_area)

    @property
    def second_moment(self) -> float:
        return self.petal_width * self.petal_thickness ** 3 / 12.0

    @property
    def stiffness(self) -> float:
        """k = 3 E I / L^3 (eq. 15)."""
        return 3.0 * self.youngs_modulus * self.second_moment / self.petal_length ** 3

    @property
    def petal_mass(self) -> float:
        return self.density * self.petal_width * self.petal_thickness * self.petal_length

    @property
    def effective_mass(self) -> float:
        """m_eff = (33/140) m_petal (eq. 16)."""
        return PSI2_INT * self.petal_mass

    @property
    def natural_frequency_hz(self) -> float:
        return math.sqrt(self.stiffness / self.effective_mass) / (2.0 * math.pi)

    @property
    def damping_coefficient(self) -> float:
        return 2.0 * self.damping_ratio * math.sqrt(self.stiffness * self.effective_mass)

    @property
    def petal_face_area(self) -> float:
        return self.petal_width * self.petal_length

    def curtain_area(self, lift: float) -> float:
        """Total geometric throat area at tip lift xi (eq. 20), all petals."""
        if lift <= 0.0:
            return 0.0
        per_petal = lift * (self.petal_width + 0.75 * self.petal_length)
        return self.n_petals * min(per_petal, self.port_area)

    def swept_volume(self, lift: float) -> float:
        """Volume displaced into the chamber by the deflected petals (eq. 29)."""
        return PSI_INT * self.n_petals * self.petal_face_area * max(lift, 0.0)


class PetalValveState:
    """Integrates eq. 19 with contact projection. One modal DOF (tip lift)."""

    def __init__(self, design: PetalValveDesign):
        self.d = design
        self.lift = 0.0
        self.lift_rate = 0.0

    def generalized_force(self, p0_up: float, p_head: float,
                          rho_jet: float, u_jet: float, rho_head: float) -> float:
        """RHS forcing of eq. 19 (per petal): pressure differential with the
        Bernoulli face-relief closure (eq. 17, A13) + plate drag (eq. 18, A14)."""
        d = self.d
        open_frac = 0.0
        if d.port_area > 0.0:
            open_frac = min(self.curtain_per_petal() / d.port_area, 1.0)
        # Bernoulli relief acts on the face the jet accelerates OVER, i.e.
        # the upstream face of whichever flow direction is active -- so it
        # always reduces the magnitude of the driving differential.
        dp_raw = p0_up - p_head
        relief = open_frac * 0.5 * rho_jet * u_jet * u_jet
        dp_eff = dp_raw - math.copysign(relief, dp_raw) if dp_raw != 0.0 else 0.0
        q_pressure = PSI_INT * d.petal_face_area * dp_eff
        q_drag = -PSI_INT * d.petal_face_area * rho_head * abs(self.lift_rate) * self.lift_rate
        return q_pressure + q_drag

    def curtain_per_petal(self) -> float:
        return self.lift * (self.d.petal_width + 0.75 * self.d.petal_length)

    def open_area(self) -> float:
        return self.d.curtain_area(self.lift)

    def step(self, dt: float, p0_up: float, p_head: float,
             rho_jet: float, u_jet: float, rho_head: float) -> None:
        """Semi-implicit (symplectic) Euler + contact projection.
        dt is the fluid step, ~20x finer than needed for the petal ODE."""
        d = self.d
        q = self.generalized_force(p0_up, p_head, rho_jet, u_jet, rho_head)
        spring = d.stiffness * (self.lift + d.seat_preload)
        acc = (q - spring - d.damping_coefficient * self.lift_rate) / d.effective_mass
        self.lift_rate += acc * dt
        self.lift += self.lift_rate * dt
        # Contact constraints (A16): momentum-conserving inelastic projection
        if self.lift < 0.0:
            self.lift = 0.0
            if self.lift_rate < 0.0:
                self.lift_rate = -d.restitution * self.lift_rate
        elif self.lift > d.max_lift:
            self.lift = d.max_lift
            if self.lift_rate > 0.0:
                self.lift_rate = -d.restitution * self.lift_rate
