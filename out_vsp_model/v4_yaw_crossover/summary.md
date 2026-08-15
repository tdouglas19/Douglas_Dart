# Douglas Dart V4 — wing station / fin area stability sweep

Source: `out_medium_model\v4_export\structural_dimensions.json` (git `d7b0670`). Vortex-lattice, Mach 0.50, 6 solves per configuration.

5/5 configurations solved; **5** are pitch-stable at BOTH release and burnout.

Static margin is quoted in reference chords (cref = 296.3 mm), positive
stable. Both fuel states are shown because the tank is an annulus around
the tailpipe, well aft of the dry CG, so the CG moves ~81 mm forward over
the burn and the margin moves with it.

| wing LE (m) | fin ratio | fin span (mm) | x_np (mm) | SM release | SM burnout | Cn_beta | ballast for +0.10 SM |
|---|---|---|---|---|---|---|---|
| 1.00 | 1.30 | 690 | 923.2 | +0.004 | +0.277 | -0.1561 | 76 mm |
| 1.00 | 1.35 | 699 | 1017.8 | +0.323 | +0.596 | +0.5027 | 534 mm |
| 1.00 | 1.40 | 708 | 1037.9 | +0.391 | +0.664 | +0.6561 | 631 mm |
| 1.00 | 1.50 | 723 | 976.6 | +0.184 | +0.457 | +0.2131 | 335 mm |
| 1.00 | 1.60 | 740 | 1068.1 | +0.493 | +0.766 | +0.8144 | 778 mm |

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


**Smallest configuration stable in BOTH axes:** wing LE 1.00 m, fin ratio 1.35 (0.206 m^2 of fin, span 699 mm) — static margin +0.323 release / +0.596 burnout, Cn_beta +0.503.

That configuration is over-stable in pitch; moving the 4.68 kg ballast to 534 mm trims it to the +0.10 cref target without touching the aerodynamics.

1 further configurations are pitch-stable but still directionally UNSTABLE. Cn_beta is set almost entirely by fin area and is nearly independent of wing station, so yaw -- not pitch -- is what sizes the tail on this vehicle.
