"""Engine internal geometry -> quasi-1D area profile (derivation.md #3)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EngineGeometry:
    """Rigid duct: chamber (cylinder) -> cone -> tailpipe, head at x=0."""

    chamber_diameter: float   # m
    chamber_length: float     # m (cylindrical section)
    cone_length: float        # m
    tailpipe_diameter: float  # m
    tailpipe_length: float    # m

    @property
    def total_length(self) -> float:
        return self.chamber_length + self.cone_length + self.tailpipe_length

    @property
    def chamber_zone_length(self) -> float:
        """Chamber + cone: the combustion / turbulence zone."""
        return self.chamber_length + self.cone_length

    def diameter_at(self, x: np.ndarray) -> np.ndarray:
        """Duct diameter, with a smooth (cosine) cone blend to avoid slope
        discontinuities that would radiate spurious waves at section joints."""
        x = np.asarray(x, dtype=float)
        d = np.empty_like(x)
        x1 = self.chamber_length
        x2 = x1 + self.cone_length
        dc, dt = self.chamber_diameter, self.tailpipe_diameter
        d[:] = dc
        in_cone = (x >= x1) & (x <= x2)
        s = (x[in_cone] - x1) / max(self.cone_length, 1e-12)
        # cosine ramp: C1-continuous at both ends
        d[in_cone] = dc + (dt - dc) * 0.5 * (1.0 - np.cos(np.pi * s))
        d[x > x2] = dt
        return d

    def area_at(self, x: np.ndarray) -> np.ndarray:
        d = self.diameter_at(x)
        return 0.25 * np.pi * d * d


@dataclass(frozen=True)
class Grid:
    x: np.ndarray        # cell centers, (N,)
    x_f: np.ndarray      # faces, (N+1,)
    dx: float
    A_c: np.ndarray      # area at centers
    A_f: np.ndarray      # area at faces
    D_c: np.ndarray      # diameter at centers
    chamber_weight: np.ndarray  # 1 in chamber, ->0 in pipe (smooth over cone)


def build_grid(geom: EngineGeometry, n_cells: int) -> Grid:
    L = geom.total_length
    x_f = np.linspace(0.0, L, n_cells + 1)
    dx = x_f[1] - x_f[0]
    x = 0.5 * (x_f[:-1] + x_f[1:])
    A_c = geom.area_at(x)
    A_f = geom.area_at(x_f)
    D_c = geom.diameter_at(x)

    x1 = geom.chamber_length
    x2 = x1 + geom.cone_length
    w = np.clip((x2 - x) / max(x2 - x1, 1e-12), 0.0, 1.0)
    # smooth the ramp
    w = 0.5 * (1.0 - np.cos(np.pi * w))
    return Grid(x=x, x_f=x_f, dx=dx, A_c=A_c, A_f=A_f, D_c=D_c,
                chamber_weight=w)
