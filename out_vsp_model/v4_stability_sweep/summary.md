# Douglas Dart V4 — wing station / fin area stability sweep

Source: `out_medium_model\v4_export\structural_dimensions.json` (git `d7b0670`). Vortex-lattice, Mach 0.50, 6 solves per configuration.

15/15 configurations solved; **9** are pitch-stable at BOTH release and burnout.

Static margin is quoted in reference chords (cref = 296.3 mm), positive
stable. Both fuel states are shown because the tank is an annulus around
the tailpipe, well aft of the dry CG, so the CG moves ~81 mm forward over
the burn and the margin moves with it.

| wing LE (m) | fin ratio | fin span (mm) | x_np (mm) | SM release | SM burnout | Cn_beta | ballast for +0.10 SM |
|---|---|---|---|---|---|---|---|
| 1.00 | 0.35 | 455 | 341.5 | -1.959 | -1.686 | -2.3187 | -2741 mm (OUTSIDE BODY) |
| 1.00 | 0.80 | 579 | 689.8 | -0.784 | -0.511 | -1.2124 | -1055 mm (OUTSIDE BODY) |
| 1.00 | 1.30 | 690 | 923.2 | +0.004 | +0.277 | -0.1561 | 76 mm |
| 1.00 | 1.80 | 775 | 1117.3 | +0.659 | +0.932 | +1.3532 | 1016 mm |
| 1.00 | 2.30 | 848 | 1216.8 | +0.995 | +1.268 | +2.4886 | 1498 mm |
| 1.30 | 0.35 | 455 | -56.4 | -3.343 | -3.076 | -2.3877 | -4727 mm (OUTSIDE BODY) |
| 1.30 | 0.80 | 579 | 679.5 | -0.860 | -0.593 | -1.3582 | -1163 mm (OUTSIDE BODY) |
| 1.30 | 1.30 | 690 | 929.8 | -0.015 | +0.252 | -0.3067 | 49 mm |
| 1.30 | 1.80 | 775 | 1140.8 | +0.697 | +0.964 | +1.1739 | 1071 mm |
| 1.30 | 2.30 | 848 | 1239.2 | +1.029 | +1.296 | +2.2763 | 1548 mm |
| 1.60 | 0.35 | 455 | 699.2 | -0.834 | -0.574 | -2.3358 | -1126 mm (OUTSIDE BODY) |
| 1.60 | 0.80 | 579 | 963.2 | +0.057 | +0.317 | -1.2887 | 152 mm |
| 1.60 | 1.30 | 690 | 1117.2 | +0.577 | +0.837 | -0.2403 | 898 mm |
| 1.60 | 1.80 | 775 | 1250.2 | +1.026 | +1.286 | +1.2261 | 1542 mm |
| 1.60 | 2.30 | 848 | 1331.4 | +1.300 | +1.560 | +2.2846 | 1936 mm |

> ## DO NOT SIZE ANYTHING FROM THIS TABLE YET
>
> These derivatives are **mesh-dependent to the point of changing the answer**. Holding the aerodynamics completely fixed (wing LE 1.00 m, fin ratio 1.40, M 0.50) and varying only the surface-root mount depth -- a meshing artifact with no physical meaning -- gives:
>
> | mount depth | static margin | Cn_beta |
> |---|---|---|
> | 6.0 mm | +0.391 | +0.656 |
> | 10.5 mm | -1.885 | -6.066 |
> | 13.5 mm | +0.072 | -0.080 |
>
> `Cm_alpha` changes sign; static margin spans 2.3 chords. Four of seven depths did not converge at all. So a single solve does not determine stability for a given geometry, and this sweep was partly measuring fin area and partly measuring which mesh each configuration landed on -- configurations that needed a retry got a different mesh from their neighbours, which is the likely source of the fin 1.50 outlier.
>
> **Prerequisite:** a mesh-convergence study at the fin/body junction (raise `surface_tessellation` and `fuselage_tessellation` until the derivatives stop responding to mount depth). See `README.md`.


**Smallest configuration stable in BOTH axes:** wing LE 1.00 m, fin ratio 1.80 (0.274 m^2 of fin, span 775 mm) — static margin +0.659 release / +0.932 burnout, Cn_beta +1.353.

That configuration is over-stable in pitch; moving the 4.68 kg ballast to 1016 mm trims it to the +0.10 cref target without touching the aerodynamics.

3 further configurations are pitch-stable but still directionally UNSTABLE. Cn_beta is set almost entirely by fin area and is nearly independent of wing station, so yaw -- not pitch -- is what sizes the tail on this vehicle.
