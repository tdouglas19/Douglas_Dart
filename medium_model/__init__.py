"""Medium-fidelity vehicle model -- Gate 3's mission solver.

Lineage: a faithful copy of the frozen ``simple_model`` (the fast screening
ancestor, which must NOT change), grown deliberately in the places where
uncertainty actually lives. ``simple_model`` answers "which designs are
worth looking at" in 0.7 s; ``medium_model`` answers "does THIS design
really close" at the expense of reasonable compute time, and is where
Gate 3's confidence comes from.

What medium_model adds over its ancestor, in the order it was added (each
change re-flies the frozen V2 design so its effect is ATTRIBUTABLE -- see
``docs/gate3_v2_refinement_plan.md`` and the attribution table):

0. Nothing -- a proven-identical copy (``tests/test_medium_model_v2_parity``).
1. A real drag build-up replacing the flat ``CD0_FRONTAL`` placeholder:
   skin friction at the true Reynolds number, form drag from fineness,
   base drag split engine-on/engine-off, area-rule wave drag.
2. Inlet spillage / additive drag -- the installed-drag term that today is
   charged to nobody (the FP engine models exclude it by assumption, the
   vehicle model does not know it exists).
3. First-principles propulsion: pulsejet-fp and ramjet-fp marched ALONG
   the trajectory (no precomputed tables), with the fuel taken from the
   input and the ramjet's mixture ratio an OUTPUT.
4. Finite-wing lift with compressibility and CLmax vs Mach, and a
   cutoff loop limited by available CL rather than a fixed load factor.

Empiricism policy (deliberate, and different from the engine repos): the
FP engine internals stay strictly clean-room -- no correlations, no
charts. The VEHICLE aerodynamics here uses standard engineering build-up
with conventional correlations, each cited where it enters. Vehicle drag
cannot be derived end-to-end without CFD, and pretending otherwise would
be less honest than citing the correlation.
"""
