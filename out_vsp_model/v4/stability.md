# Douglas Dart V4 -- stability from VSPAERO

```
[panel] Static stability at Mach 0.30

  CL_alpha         +1.4255 /rad
  Cm_alpha         +4.9173 /rad   UNSTABLE in pitch
  neutral point     -100.1 mm from nose
  CG                 922.1 mm from nose
  static margin     -3.450 cref (-1022.2 mm)

Dynamic modes at Mach 0.30, 164 m (V=101.9 m/s, q=6261 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Cm_q unavailable: short-period damping excludes pitch rate.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).

[panel] Static stability at Mach 0.60

  CL_alpha         +1.8885 /rad
  Cm_alpha         +5.2124 /rad   UNSTABLE in pitch
  neutral point      104.3 mm from nose
  CG                 922.1 mm from nose
  static margin     -2.760 cref (-817.8 mm)

Dynamic modes at Mach 0.60, 164 m (V=203.8 m/s, q=25043 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Cm_q unavailable: short-period damping excludes pitch rate.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).

[vortex_lattice] Static stability at Mach 0.20

  CL_alpha         +1.6037 /rad
  Cm_alpha         +3.1963 /rad   UNSTABLE in pitch
  neutral point      331.5 mm from nose
  CG                 922.1 mm from nose
  static margin     -1.993 cref (-590.6 mm)

Dynamic modes at Mach 0.20, 164 m (V=67.9 m/s, q=2783 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Cm_q unavailable: short-period damping excludes pitch rate.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).

[vortex_lattice] Static stability at Mach 0.30

  CL_alpha         +1.6255 /rad
  Cm_alpha         +3.2508 /rad   UNSTABLE in pitch
  neutral point      329.5 mm from nose
  CG                 922.1 mm from nose
  static margin     -2.000 cref (-592.6 mm)
  Cn_beta          -2.1762 /rad   UNSTABLE directionally
  Cl_beta          -0.0010 /rad   STABLE laterally

  Two independent estimates of the same derivatives:
  CL_alpha   finite-difference  +1.6255   stability-mode  +1.6994   ratio  0.96x
  Cm_alpha   finite-difference  +3.2508   stability-mode  +3.6476   ratio  0.89x
  CY_beta    finite-difference  -0.6077   stability-mode  -0.5562   ratio  1.09x
  Cn_beta    finite-difference  -2.1762   stability-mode  -2.3363   ratio  0.93x
  Cl_beta    finite-difference  -0.0010   stability-mode  -0.0166   ratio  0.06x  <-- DISAGREE

Dynamic modes at Mach 0.30, 164 m (V=101.9 m/s, q=6261 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  tau=0.344 s

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).
  ! Cn_beta <= 0: the vehicle is directionally UNSTABLE, so there is no dutch-roll oscillation to report.

[vortex_lattice] Static stability at Mach 0.40

  CL_alpha         +1.6594 /rad
  Cm_alpha         +3.3005 /rad   UNSTABLE in pitch
  neutral point      332.7 mm from nose
  CG                 922.1 mm from nose
  static margin     -1.989 cref (-589.3 mm)

Dynamic modes at Mach 0.40, 164 m (V=135.9 m/s, q=11130 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Cm_q unavailable: short-period damping excludes pitch rate.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).

[vortex_lattice] Static stability at Mach 0.50

  CL_alpha         +1.7219 /rad
  Cm_alpha         +3.4450 /rad   UNSTABLE in pitch
  neutral point      329.3 mm from nose
  CG                 922.1 mm from nose
  static margin     -2.001 cref (-592.8 mm)
  Cn_beta          -2.0955 /rad   UNSTABLE directionally
  Cl_beta          +0.3310 /rad   UNSTABLE laterally

  Two independent estimates of the same derivatives:
  CL_alpha   finite-difference  +1.7219   stability-mode  +2.0128   ratio  0.86x
  Cm_alpha   finite-difference  +3.4450   stability-mode  +0.3958   ratio  8.70x  <-- DISAGREE

Dynamic modes at Mach 0.50, 164 m (V=169.8 m/s, q=17391 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).
  ! Cn_beta <= 0: the vehicle is directionally UNSTABLE, so there is no dutch-roll oscillation to report.

[vortex_lattice] Static stability at Mach 0.60

  CL_alpha         +1.7888 /rad
  Cm_alpha         +3.6183 /rad   UNSTABLE in pitch
  neutral point      322.7 mm from nose
  CG                 922.1 mm from nose
  static margin     -2.023 cref (-599.4 mm)
  Cn_beta          -2.4724 /rad   UNSTABLE directionally
  Cl_beta          -0.0084 /rad   STABLE laterally

Dynamic modes at Mach 0.60, 164 m (V=203.8 m/s, q=25043 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Cm_q unavailable: short-period damping excludes pitch rate.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).
  ! Cn_beta <= 0: the vehicle is directionally UNSTABLE, so there is no dutch-roll oscillation to report.

[vortex_lattice] Static stability at Mach 0.70

  CL_alpha         +1.9280 /rad
  Cm_alpha         +3.9892 /rad   UNSTABLE in pitch
  neutral point      309.0 mm from nose
  CG                 922.1 mm from nose
  static margin     -2.069 cref (-613.1 mm)

Dynamic modes at Mach 0.70, 164 m (V=237.8 m/s, q=34086 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  not computable

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Cm_q unavailable: short-period damping excludes pitch rate.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).

[vortex_lattice] Static stability at Mach 0.80

  CL_alpha         +2.2633 /rad
  Cm_alpha         +5.5144 /rad   UNSTABLE in pitch
  neutral point      200.1 mm from nose
  CG                 922.1 mm from nose
  static margin     -2.436 cref (-721.9 mm)
  Cn_beta          -0.4763 /rad   UNSTABLE directionally
  Cl_beta          -0.0181 /rad   STABLE laterally

  Two independent estimates of the same derivatives:
  CL_alpha   finite-difference  +2.2633   stability-mode  +1.7567   ratio  1.29x  <-- DISAGREE
  Cm_alpha   finite-difference  +5.5144   stability-mode  -4.7060   ratio -1.17x

Dynamic modes at Mach 0.80, 164 m (V=271.7 m/s, q=44521 Pa)

  short period     not computable
  dutch roll       not computable
  roll subsidence  tau=0.128 s

  ! Inertias come from assigned mass stations; the project has no CG model.
  ! Decoupled modal approximations, not a 6-DOF eigenvalue solve.
  ! Short-period frequency is imaginary: the pitch mode is divergent, not oscillatory (statically unstable at this condition).
  ! Cn_beta <= 0: the vehicle is directionally UNSTABLE, so there is no dutch-roll oscillation to report.

```
