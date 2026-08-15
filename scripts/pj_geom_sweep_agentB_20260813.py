"""Pulsejet-fp geometry sweep: minimum duct length that makes >= 165 N @ M0.15.

UNIQUE FILE (agent B, 2026-08-13) -- do not merge with other sweep scripts.

Sweeps chamber diameter (193/214/235 mm, the +-10% band around V3a's 214)
against tail/chamber-diameter ratio t/D. Everything else follows the V3a
frozen design and medium_model/fp_spec.py exactly:

  chamber_diameter = 0.95 * body_diameter   (so body_dia = D / 0.95)
  chamber_length   = V3a's 0.38930734 m (held fixed)
  tail_total       = throat_length_m ; cone = 0.17333 * tail_total
  tailpipe_length  = tail_length = 0.82667 * throat_length  == (t/D) * D
  tailpipe_diameter: throat/body diameter ratio held at V3a's 0.54000
                     (throat AREA fraction 0.2916, under the 0.30 cap)
  valve + intake    scaled isometrically by s = D / 0.078

Usage:
  python scripts/pj_geom_sweep_agentB_20260813.py <out.jsonl> D:ratio [D:ratio ...]
where D is chamber diameter in mm and ratio is t/D.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import replace

from pulsejet_fp import (IntakeDesign, Numerics, pulsejet_thrust,
                         reference_gas, reference_valve)
from pulsejet_fp.geometry import EngineGeometry

CHAMBER_LENGTH_M = 0.38930734448088206      # V3a, held fixed
CONE_FRACTION_OF_TAIL = 0.130 / (0.130 + 0.620)   # 0.173333 (fp_spec FP-1)
TAILPIPE_FRACTION = 1.0 - CONE_FRACTION_OF_TAIL   # 0.826667
THROAT_OVER_BODY_DIA = 0.12161734042955713 / 0.22521729709177246  # 0.54000
FP1_CHAMBER_DIA = 0.078


def build(chamber_dia_m: float, t_over_d: float) -> dict:
    body_dia = chamber_dia_m / 0.95
    tail_length = t_over_d * chamber_dia_m           # tailpipe (post-cone)
    throat_length = tail_length / TAILPIPE_FRACTION  # medium_model duct budget
    cone = CONE_FRACTION_OF_TAIL * throat_length
    duct = CHAMBER_LENGTH_M + throat_length
    body_length = duct + 3.0 * body_dia
    throat_dia = THROAT_OVER_BODY_DIA * body_dia

    geom = EngineGeometry(
        chamber_diameter=chamber_dia_m,
        chamber_length=CHAMBER_LENGTH_M,
        cone_length=cone,
        tailpipe_diameter=throat_dia,
        tailpipe_length=tail_length,
    )
    s = chamber_dia_m / FP1_CHAMBER_DIA
    rv = reference_valve()
    valve = replace(
        rv,
        petal_length=rv.petal_length * s,
        petal_width=rv.petal_width * s,
        petal_thickness=rv.petal_thickness * s,
        port_area=rv.port_area * s * s,
        max_lift=rv.max_lift * s,
        seat_preload=rv.seat_preload * s,
    )
    intake = IntakeDesign(
        duct_length=0.060 * s,
        duct_diameter=0.050 * s,
        plenum_volume=5e-5 * s ** 3,
        orientation="side",
        bl_momentum_fraction=0.6,
    )
    return dict(geom=geom, valve=valve, intake=intake,
                body_dia_m=body_dia, tail_length_m=tail_length,
                throat_length_m=throat_length, duct_m=duct,
                body_length_m=body_length, throat_dia_m=throat_dia,
                fineness=body_length / body_dia,
                throat_area_frac=(throat_dia / body_dia) ** 2)


def run_point(chamber_dia_mm: float, t_over_d: float,
              n_cells: int = 200) -> dict:
    d = chamber_dia_mm / 1000.0
    b = build(d, t_over_d)
    t0 = time.time()
    r = pulsejet_thrust(
        mach=0.15, altitude_m=60.0, gas=reference_gas(phi=1.0),
        geom=b["geom"], valve=b["valve"], intake=b["intake"],
        numerics=Numerics(n_cells=n_cells), t_end=1.5,
        stop_when_converged=True,
    )
    span = r.p_max_ratio - r.p_min_ratio
    sustains = bool(span > 0.25 and r.thrust_n > 50.0)
    return {
        "chamber_dia_mm": chamber_dia_mm, "t_over_d": t_over_d,
        "n_cells": n_cells,
        "body_dia_mm": 1000 * b["body_dia_m"],
        "throat_dia_mm": 1000 * b["throat_dia_m"],
        "throat_area_frac": b["throat_area_frac"],
        "tail_length_mm": 1000 * b["tail_length_m"],
        "throat_length_mm": 1000 * b["throat_length_m"],
        "duct_mm": 1000 * b["duct_m"],
        "body_length_mm": 1000 * b["body_length_m"],
        "fineness": b["fineness"],
        "thrust_n": r.thrust_n, "status": r.status,
        "frequency_hz": r.frequency_hz,
        "p_min_ratio": r.p_min_ratio, "p_max_ratio": r.p_max_ratio,
        "p_span": span, "n_cycles": r.n_cycles,
        "mdot_fuel_kg_s": r.mdot_fuel_kg_s,
        "sustains": sustains, "wall_s": time.time() - t0,
    }


def main() -> None:
    out_path = sys.argv[1]
    pts = []
    for tok in sys.argv[2:]:
        parts = tok.split(":")
        pts.append((float(parts[0]), float(parts[1]),
                    int(parts[2]) if len(parts) > 2 else 200))
    with open(out_path, "a", buffering=1) as fh:
        for dia, ratio, nc in pts:
            rec = run_point(dia, ratio, nc)
            fh.write(json.dumps(rec) + "\n")
            print(f"D={dia:.0f} t/D={ratio:.2f} nc={nc} "
                  f"L_body={rec['body_length_mm']:7.1f} "
                  f"F={rec['thrust_n']:7.1f} N f={rec['frequency_hz']:6.1f} Hz "
                  f"p/p0 {rec['p_min_ratio']:.3f}-{rec['p_max_ratio']:.3f} "
                  f"{rec['status']} sustain={rec['sustains']} "
                  f"({rec['wall_s']:.0f}s)", flush=True)


if __name__ == "__main__":
    main()
