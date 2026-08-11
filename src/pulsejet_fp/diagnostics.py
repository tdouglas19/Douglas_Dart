"""Standard diagnostic plots. The two-panel dimensionless trace (p/p0,
va/a0, ve/a0 + T/T0) follows the established troubleshooting convention:
produce it for every diagnostic attempt.
"""
from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COL_P = "#2a78d6"
COL_VA = "#008300"
COL_VE = "#eda100"
COL_T = "#e34948"


def standard_cycle_plot(hist: dict, p_a: float, T_a: float, path: str,
                        t_window_ms: tuple[float, float] | None = None,
                        title: str = ""):
    """Panel 1: p/p0, va/a0, ve/a0 vs t [ms]; panel 2: T/T0."""
    a0 = math.sqrt(1.4 * 287.05 * T_a)
    t_ms = hist["t"] * 1e3
    m = np.ones_like(t_ms, dtype=bool)
    if t_window_ms is not None:
        m = (t_ms >= t_window_ms[0]) & (t_ms <= t_window_ms[1])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                   height_ratios=[2, 1])
    ax1.plot(t_ms[m], hist["p_head"][m] / p_a, color=COL_P, lw=1.0,
             label="p/p0 (head)")
    ax1.plot(t_ms[m], hist["u_jet_signed"][m] / a0, color=COL_VA, lw=0.9,
             label="va/a0 (valve jet)")
    ax1.plot(t_ms[m], hist["u_exit"][m] / a0, color=COL_VE, lw=0.9,
             label="ve/a0 (exit)")
    ax1.axhline(1.0, color="gray", lw=0.5, ls="--")
    ax1.axhline(0.0, color="gray", lw=0.5)
    ax1.legend(loc="upper right", fontsize=9)
    ax1.set_ylabel("dimensionless")
    if title:
        ax1.set_title(title)

    ax2.plot(t_ms[m], hist["T_head"][m] / T_a, color=COL_T, lw=1.0,
             label="T/T0 (head)")
    ax2.legend(loc="upper right", fontsize=9)
    ax2.set_ylabel("T/T0")
    ax2.set_xlabel("t [ms]")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def overview_plot(hist: dict, p_a: float, path: str, title: str = ""):
    t_ms = hist["t"] * 1e3
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(t_ms, hist["F_mom"], lw=0.8, label="F momentum (eq.24)")
    axes[0].plot(t_ms, hist["F_surf"], lw=0.8, alpha=0.7,
                 label="F surface (eq.25)")
    axes[0].axhline(0, color="gray", lw=0.5)
    axes[0].set_ylabel("thrust [N]")
    axes[1].plot(t_ms, hist["lift"] * 1e3, lw=0.8, color="#7a3fbf",
                 label="valve lift [mm]")
    axes[1].set_ylabel("lift [mm]")
    axes[2].plot(t_ms, hist["q_tot"] / 1e3, lw=0.8, color=COL_T,
                 label="heat release [kW]")
    axes[2].set_ylabel("q [kW]")
    axes[3].plot(t_ms, hist["mdot_v"], lw=0.8, color=COL_VA,
                 label="valve mdot [kg/s]")
    axes[3].plot(t_ms, hist["mdot_exit"], lw=0.8, color=COL_VE, alpha=0.7,
                 label="exit mdot [kg/s]")
    axes[3].axhline(0, color="gray", lw=0.5)
    axes[3].set_ylabel("mdot [kg/s]")
    axes[3].set_xlabel("t [ms]")
    for ax in axes:
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.25)
    if title:
        axes[0].set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def thrust_vs_mach_plot(machs, results, path, title="Thrust vs Mach"):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True,
                                   height_ratios=[2, 1])
    F = [r.thrust_n for r in results]
    ok = [r.status == "converged" for r in results]
    ax1.plot(machs, F, "-o", color=COL_P, ms=4)
    for m_, f_, o_ in zip(machs, F, ok):
        if not o_ and np.isfinite(f_):
            ax1.plot([m_], [f_], "o", color="#aaaaaa", ms=7, mfc="none")
    ax1.axhline(0, color="gray", lw=0.6)
    ax1.set_ylabel("cycle-averaged thrust [N]")
    ax1.set_title(title)
    fr = [r.frequency_hz for r in results]
    ax2.plot(machs, fr, "-s", color=COL_T, ms=4)
    ax2.set_ylabel("frequency [Hz]")
    ax2.set_xlabel("flight Mach number")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
