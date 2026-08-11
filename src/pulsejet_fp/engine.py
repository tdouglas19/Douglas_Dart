"""Transient valved-pulsejet engine: couples the quasi-1D FV gas dynamics,
petal-valve ODE, orifice boundary flow, Arrhenius reaction, and the chamber
turbulence budget (derivation.md #12-13 step ordering).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import atmosphere, orifice, solver
from .gas import GasModel
from .geometry import EngineGeometry, Grid, build_grid
from .valve import PetalValveDesign, PetalValveState


@dataclass
class Numerics:
    n_cells: int = 300
    cfl: float = 0.4
    dt_max: float = 5e-6
    record_stride: int = 1


@dataclass
class TurbulenceParams:
    """A10/A11 closure constants -- fixed once, never tuned per case."""
    c_nu: float = 0.5
    c_eps: float = 0.5
    c_pipe: float = 0.02
    mixing_length_frac: float = 0.35  # l_m = frac * D_chamber
    nu_min: float = 2e-5
    k_init: float = 10.0


@dataclass
class StartCondition:
    """State an instant after the starting fire (derivation.md #13.2)."""
    T_hot: float = 1900.0
    p_ratio: float = 1.4


@dataclass(frozen=True)
class IntakeDesign:
    """Intake duct + valve-face plenum (derivation.md #6b, eq. 23b, A24)."""
    duct_length: float = 0.060    # m
    duct_diameter: float = 0.050  # m
    plenum_volume: float = 5e-5   # m^3 (valve-cap volume)

    @property
    def duct_area(self) -> float:
        return 0.25 * math.pi * self.duct_diameter ** 2


class History:
    FIELDS = ("t", "p_head", "T_head", "u_head", "Y_head", "u_jet_signed",
              "mdot_v", "lift", "lift_rate", "p_exit", "u_exit", "mdot_exit",
              "F_mom", "F_surf", "q_tot", "k_c", "T_max", "mass_total",
              "energy_total", "reactant_mass", "p_plen", "mdot_i")

    def __init__(self):
        for f in self.FIELDS:
            setattr(self, f, [])

    def append(self, **kw):
        for f in self.FIELDS:
            getattr(self, f).append(kw[f])

    def as_arrays(self):
        return {f: np.asarray(getattr(self, f)) for f in self.FIELDS}


class PulsejetEngine:
    def __init__(self, gas: GasModel, geom: EngineGeometry,
                 valve: PetalValveDesign, mach: float = 0.0,
                 altitude_m: float = 0.0,
                 numerics: Numerics | None = None,
                 turb: TurbulenceParams | None = None,
                 start: StartCondition | None = None,
                 intake: IntakeDesign | None = None):
        self.gas = gas
        self.geom = geom
        self.valve_design = valve
        self.numerics = numerics or Numerics()
        self.turb = turb or TurbulenceParams()
        self.start = start or StartCondition()
        self.intake = intake or IntakeDesign()

        self.p_a, self.T_a = atmosphere.ambient(altitude_m)
        self.p0_up, self.T0_up, self.u_inf = atmosphere.ram_state(
            self.p_a, self.T_a, mach)
        self.mach = mach
        # intake column + plenum state (eq. 23b)
        self.mdot_i = 0.0
        self.p_plen = self.p0_up

        self.grid: Grid = build_grid(geom, self.numerics.n_cells)
        self.valve = PetalValveState(valve)
        self.l_m = self.turb.mixing_length_frac * geom.chamber_diameter
        self.k_c = self.turb.k_init

        g = self.grid
        # zone-exit face index for the turbulence advection-loss term
        self._iz = int(np.searchsorted(g.x, geom.chamber_zone_length))

        # Zeldovich number for the strain-extinction closure (A23):
        # Ze = T_a (T_ad - T_u) / T_ad^2, activation-energy asymptotics --
        # the reaction zone is ~1/Ze of the flame, so the flame's effective
        # response time is Ze^2 * tau_chem.
        T_ad = (gas.cp_R * 300.0 + gas.q_R) / gas.cp_P
        self.Ze2 = (gas.T_a * (T_ad - 300.0) / T_ad ** 2) ** 2

        # ---- initial condition ----
        gas_ = self.gas
        hot = g.x < geom.chamber_zone_length
        rho = np.empty_like(g.x)
        T = np.where(hot, self.start.T_hot, self.T_a)
        p = np.where(hot, self.start.p_ratio * self.p_a, self.p_a)
        Y = np.zeros_like(g.x)  # products / ambient air (A1 mapping)
        rho = p / (gas_.R_mix(Y) * T)
        u = np.zeros_like(g.x)
        self.A_eff = g.A_c.copy()
        self.U = solver.conserved(rho, u, p, Y, gas_, self.A_eff)

        self.t = 0.0
        self.step_count = 0
        self.history = History()
        self.status = "running"
        self._record_now()

    # ------------------------------------------------------------------
    def _head_flux(self, rho1, u1, p1, Y1, T1):
        """Boundary flux at x=0 (derivation.md #8) + valve flow numbers.
        Returns (F_head(4,), mdot_v, u_jet_signed, rho_throat)."""
        gas = self.gas
        A0 = self.grid.A_f[0]
        p_w = solver.wall_pressure(rho1, u1, p1, Y1, gas)
        A_v = self.valve.open_area()
        if A_v <= 0.0:
            return np.array([0.0, p_w * A0, 0.0, 0.0]), 0.0, 0.0, rho1

        if self.p_plen > p1:
            G, u_j, rho_t = orifice.mass_flux(
                self.p_plen, self.T0_up, gas.gamma_R, gas.R_R, p1)
            mdot = G * A_v
            h0 = gas.cp_R * self.T0_up
            F = np.array([mdot, mdot * u_j + p_w * A0, mdot * h0, mdot * 1.0])
            return F, mdot, u_j, rho_t
        # back-spit: chamber-side stagnation state drives outflow
        g1 = float(gas.gamma_mix(Y1))
        a1 = math.sqrt(g1 * p1 / rho1)
        m1 = min(abs(u1) / a1, 1.0)
        fac = 1.0 + 0.5 * (g1 - 1.0) * m1 * m1
        p01 = p1 * fac ** (g1 / (g1 - 1.0))
        T01 = T1 * fac
        G, u_j, rho_t = orifice.mass_flux(
            p01, T01, g1, float(gas.R_mix(Y1)), self.p_plen)
        mdot = -G * A_v
        h01 = float(gas.cp_mix(Y1)) * T01
        F = np.array([mdot, abs(mdot) * u_j + p_w * A0, mdot * h01, mdot * Y1])
        return F, mdot, -u_j, rho_t

    def _exit_flux(self, rhoN, uN, pN, YN):
        """Boundary flux at x=L via ghost state + HLLC (derivation.md #8)."""
        gas = self.gas
        Ae = self.grid.A_f[-1]
        gN = float(gas.gamma_mix(YN))
        aN = math.sqrt(gN * pN / rhoN)
        if uN >= aN:  # supersonic outflow: full extrapolation
            EN = pN / ((gN - 1.0) * rhoN) + 0.5 * uN * uN
            return np.array([rhoN * uN, rhoN * uN * uN + pN,
                             uN * (rhoN * EN + pN), rhoN * uN * YN]) * Ae
        if uN >= 0.0:  # subsonic outflow: ambient back-pressure (A21)
            rho_g = rhoN * (self.p_a / pN) ** (1.0 / gN)
            F = solver.hllc_scalar(rhoN, uN, pN, YN,
                                   max(rho_g, solver.RHO_FLOOR), uN,
                                   self.p_a, YN, gas)
            return F * Ae
        # backflow: quiescent base-region reservoir at (p_a, T_a), Y=0 (A22)
        gP = gas.gamma_P
        if pN < self.p_a:
            T_g = self.T_a * (pN / self.p_a) ** ((gP - 1.0) / gP)
            cpP = gas.cp_P
            u_g = -math.sqrt(max(2.0 * cpP * (self.T_a - T_g), 0.0))
            rho_g = pN / (gas.R_P * T_g)
            F = solver.hllc_scalar(rhoN, uN, pN, YN, rho_g, u_g, pN, 0.0, gas)
        else:
            rho_g = self.p_a / (gas.R_P * self.T_a)
            F = solver.hllc_scalar(rhoN, uN, pN, YN, rho_g, uN, self.p_a, 0.0, gas)
        return F * Ae

    # ------------------------------------------------------------------
    def _nu_t_field(self, u):
        t = self.turb
        nu_cz = t.c_nu * math.sqrt(max(self.k_c, 0.0)) * self.l_m
        nu_pipe = t.c_pipe * np.abs(u) * self.grid.D_c
        w = self.grid.chamber_weight
        return w * nu_cz + (1.0 - w) * nu_pipe + t.nu_min

    def _rhs(self, U, nu_t, prims=None):
        if prims is None:
            prims = solver.primitives(U, self.A_eff, self.gas)
        rho, u, p, Y, T, a = prims
        F_head, mdot_v, uj, rho_t = self._head_flux(
            float(rho[0]), float(u[0]), float(p[0]), float(Y[0]), float(T[0]))
        F_exit = self._exit_flux(
            float(rho[-1]), float(u[-1]), float(p[-1]), float(Y[-1]))
        rhs, _ = solver.hyperbolic_rhs(U, self.A_eff, self.grid.A_f,
                                       self.grid.dx, self.gas,
                                       F_head, F_exit, nu_t, prims=prims)
        return rhs, (mdot_v, uj, rho_t)

    def _dt_from(self, prims, nu):
        rho, u, p, Y, T, a = prims
        dx = self.grid.dx
        dt_conv = self.numerics.cfl * dx / float(np.max(np.abs(u) + a))
        dt_diff = 0.4 * dx * dx / float(np.max(nu))
        dt_valve = 2.0 * math.pi / (20.0 * math.sqrt(
            self.valve_design.stiffness / self.valve_design.effective_mass))
        return min(dt_conv, dt_diff, dt_valve, self.numerics.dt_max)

    def compute_dt(self):
        prims = solver.primitives(self.U, self.A_eff, self.gas)
        return self._dt_from(prims, self._nu_t_field(prims[1]))

    def step(self):
        gas = self.gas
        dx = self.grid.dx

        prims0 = solver.primitives(self.U, self.A_eff, gas)
        nu_t = self._nu_t_field(prims0[1])
        dt = self._dt_from(prims0, nu_t)

        # ---- SSP-RK2 transport ----
        rhs1, bc1 = self._rhs(self.U, nu_t, prims0)
        U1 = self.U + dt * rhs1
        rhs2, _ = self._rhs(U1, nu_t)
        self.U = 0.5 * self.U + 0.5 * (U1 + dt * rhs2)

        # ---- jet-strain extinction field (derivation.md #5b, A23) ----
        mdot_v, uj, rho_t = bc1
        prims2 = solver.primitives(self.U, self.A_eff, gas)
        quench = None
        if mdot_v > 0.0 and abs(uj) > 1.0 and self.valve.lift > 1e-5:
            T2 = prims2[4]
            s_jet = abs(uj) / max(self.valve.lift, 3e-4)
            L_jet = float(np.clip(15.0 * self.valve.lift, 5e-3,
                                  0.6 * self.geom.chamber_zone_length))
            s = s_jet * np.exp(-self.grid.x / L_jet)
            k_arr = gas.A_r * np.exp(-gas.T_a / T2)
            quench = 1.0 / (1.0 + (s * self.Ze2 / np.maximum(k_arr, 1e-30)) ** 2)

        # ---- reaction sub-step (operator split) ----
        heat_per_len, q_rate = solver.reaction_substep(
            self.U, self.A_eff, gas, dt, quench, prims=prims2)
        q_tot = float(np.sum(q_rate)) * dx  # W

        # ---- intake column + plenum ODEs (eq. 23b, symplectic order) ----
        ik = self.intake
        rho0_i = self.p0_up / (gas.R_R * self.T0_up)
        u_i = self.mdot_i / (rho0_i * ik.duct_area)
        # Borda-Carnot dump loss where the duct expands into the plenum
        dp_loss = 0.5 * rho0_i * u_i * abs(u_i)
        self.mdot_i += dt * ik.duct_area / ik.duct_length \
            * (self.p0_up - self.p_plen - dp_loss)
        self.p_plen += dt * (gas.gamma_R * gas.R_R * self.T0_up
                             / ik.plenum_volume) * (self.mdot_i - mdot_v)
        self.p_plen = min(max(self.p_plen, 0.2 * self.p_a), 5.0 * self.p_a)

        # ---- valve ODE + contact (eq. 19) ----
        rho, u, p, Y, T, a = solver.primitives(self.U, self.A_eff, gas)
        lift_old = self.valve.lift
        self.valve.step(dt, self.p_plen, float(p[0]), rho_t, abs(uj), float(rho[0]))

        # ---- variable-volume chamber-cell work (eq. 29) ----
        S_old = self.valve_design.swept_volume(lift_old)
        S_new = self.valve_design.swept_volume(self.valve.lift)
        if S_new != S_old:
            self.U[2, 0] += float(p[0]) * (S_new - S_old) / dx
            self.A_eff[0] = self.grid.A_c[0] - S_new / dx

        # ---- turbulence energy budget (eq. 13) ----
        w = self.grid.chamber_weight
        m_cz = float(np.sum(rho * self.A_eff * w)) * dx
        prod = 0.5 * max(mdot_v, 0.0) * (abs(uj) - float(u[0])) ** 2
        # mean-shear production (standard k-equation term): nu_t (du/dx)^2
        dudx = np.diff(u) / dx
        nu_face = 0.5 * (nu_t[:-1] + nu_t[1:])
        rhoA_face = 0.5 * (rho[:-1] * self.A_eff[:-1] + rho[1:] * self.A_eff[1:])
        w_face = 0.5 * (w[:-1] + w[1:])
        prod += float(np.sum(w_face * rhoA_face * nu_face * dudx * dudx)) * dx
        iz = min(self._iz, len(u) - 1)
        mdot_out = float(rho[iz] * u[iz] * self.grid.A_c[iz])
        tp = self.turb
        dk = (prod / max(m_cz, 1e-9)
              - tp.c_eps * self.k_c ** 1.5 / self.l_m
              - self.k_c * max(mdot_out, 0.0) / max(m_cz, 1e-9))
        self.k_c = max(self.k_c + dt * dk, 1e-6)

        self.t += dt
        self.step_count += 1

        if not np.all(np.isfinite(self.U)):
            self.status = "diverged"
        if self.step_count % self.numerics.record_stride == 0:
            self._record_now(mdot_v=mdot_v, uj_signed=uj, q_tot=q_tot)
        return dt

    # ------------------------------------------------------------------
    def _record_now(self, mdot_v=0.0, uj_signed=0.0, q_tot=0.0):
        gas = self.gas
        g = self.grid
        rho, u, p, Y, T, a = solver.primitives(self.U, self.A_eff, gas)
        dx = g.dx
        F_exit = self._exit_flux(float(rho[-1]), float(u[-1]),
                                 float(p[-1]), float(Y[-1]))
        mdot_e = float(F_exit[0])
        F_mom = float(F_exit[1]) - self.p_a * g.A_f[-1] \
            - max(mdot_v, 0.0) * self.u_inf
        # eq. 25: head push + cone pull + intake-jet reaction on the head
        F_surf = (float(p[0]) - self.p_a) * g.A_f[0] \
            + float(np.sum((p - self.p_a) * (g.A_f[1:] - g.A_f[:-1]))) \
            - max(mdot_v, 0.0) * abs(uj_signed)
        self.history.append(
            t=self.t, p_head=float(p[0]), T_head=float(T[0]),
            u_head=float(u[0]), Y_head=float(Y[0]),
            u_jet_signed=uj_signed, mdot_v=mdot_v,
            lift=self.valve.lift, lift_rate=self.valve.lift_rate,
            p_exit=float(p[-1]), u_exit=float(u[-1]), mdot_exit=mdot_e,
            F_mom=F_mom, F_surf=F_surf, q_tot=q_tot, k_c=self.k_c,
            T_max=float(np.max(T)),
            mass_total=float(np.sum(self.U[0])) * dx,
            energy_total=float(np.sum(self.U[2])) * dx,
            reactant_mass=float(np.sum(self.U[3])) * dx,
            p_plen=self.p_plen, mdot_i=self.mdot_i,
        )

    def run(self, t_end: float, progress_every: float = 0.0):
        next_report = progress_every
        while self.t < t_end and self.status == "running":
            self.step()
            if progress_every and self.t >= next_report:
                next_report += progress_every
        if self.status == "running":
            self.status = "completed"
        return self.history.as_arrays()
